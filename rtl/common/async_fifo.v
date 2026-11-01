`timescale 1ns/1ps
`default_nettype none

// 精确 DEPTH 个字的双时钟 FIFO；Gray 指针两级同步，BRAM 同步读取。
// 唯一共同 arst 同时覆盖两端；运行期清空由写域 flush_req/ready 握手发起。
// 参数前提：DATA_WIDTH>=1、ADDR_WIDTH>=1，DEPTH=2**ADDR_WIDTH。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module async_fifo #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 11,
    parameter USE_EG_BRAM = 1,
    parameter RAM_IMPLEMENT = "32K",
    // 0 直接计算写端已缓存数据项数；1 先寄存 Gray 解码的读指针，拆开解码和减法路径。
    // 仅使写端已缓存数据项数晚一拍反映读出；计数与 full_reg 使用同一读指针样本。
    // 满判据、DEPTH 容量、读数据、读端计数及 flush 握手不变；不得据此提前读出。
    parameter PIPE_WR_LEVEL = 0,
    // 1只提前缓存Gray(wbin+1)，把满判据前的指针加一移到独立寄存器。
    // 每次真实写入才推进该镜像；容量、full/ready时刻、同步器和已缓存数据项数均不改变。
    // 默认关闭，整机仅为两个 PCM FIFO 启用；同步级及 CDC 约束范围固定。
    parameter FULL_LOOKAHEAD = 0,
    // 只提前做读空比较，消费沿和输出有效沿不变；两个PCM实例启用。
    // keep阻止综合把比较器重新合并到晚到的read_fire之后，默认 0 直接计算读空条件。
    parameter READ_PRECOMPARE = 0,
    // 已缓存数据项数由读写指针之差计算，按四位组并行相减，各组借位直接比较低位前缀。
    // 不增加流水拍，只对PCM实例启用，避免计算已缓存数据项数的长借位链串入 DMA 请求条件比较。
    parameter LEVEL_LOOKAHEAD = 0
) (
    input  wire arst,
    input  wire wr_clk,
    input  wire wr_valid,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire wr_ready,
    output wire wr_active,
    output wire [ADDR_WIDTH:0] wr_level,
    input  wire flush_req,
    output wire flush_ready,
    output wire flush_busy,
    output reg  flush_done,
    input  wire rd_clk,
    input  wire rd_ready,
    output wire rd_valid,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire rd_active,
    output wire [ADDR_WIDTH:0] rd_level
);
    wire wr_rst_logic;
    wire rd_rst_logic;
    localparam [ADDR_WIDTH:0] FULL_MASK = {2'b11, {(ADDR_WIDTH-1){1'b0}}};
    localparam [1:0] STARTUP = 2'd0, RUN = 2'd1, HOLD = 2'd2, WAIT_LINK = 2'd3;
    reg [1:0] state;
    reg flush_hold;
    // flush会同时清两边的指针；先让双方停稳，再走这个复位握手。
    wire fifo_arst = arst | flush_hold;
    wire wr_rst, rd_rst, control_rst;
    // 协调器由外部共同复位同步释放，不能接含自身flush_hold的fifo_arst。
    reset_sync u_control_reset(.clk(wr_clk), .arst(arst), .rst(control_rst));
    reset_sync u_wr_reset(.clk(wr_clk), .arst(fifo_arst), .rst(wr_rst),.logic_rst(wr_rst_logic));
    reset_sync u_rd_reset(.clk(rd_clk), .arst(fifo_arst), .rst(rd_rst),.logic_rst(rd_rst_logic));

    reg [ADDR_WIDTH:0] wbin, rbin;
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg [ADDR_WIDTH:0] wgray, rgray;
    // 相邻级之间保持完整的目标域时序；跨域例外仅用于第一级入口。
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg [ADDR_WIDTH:0] rgray_w1, rgray_w2;
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg [ADDR_WIDTH:0] wgray_r1, wgray_r2;
    reg wr_up, rd_up;
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg wr_up_r1, wr_up_r2, rd_up_w1, rd_up_w2;
    reg full_reg, output_valid;


    function [ADDR_WIDTH:0] gray_to_bin;
        input [ADDR_WIDTH:0] gray;
        integer bit_index;
        begin
            gray_to_bin[ADDR_WIDTH] = gray[ADDR_WIDTH];
            for (bit_index = ADDR_WIDTH-1; bit_index >= 0; bit_index = bit_index-1)
                gray_to_bin[bit_index] = gray_to_bin[bit_index+1] ^ gray[bit_index];
        end
    endfunction

    // fifo_arst含写域flush_hold，只允许进入两端reset_sync的异步置位口。
    // 各端rst异步置位并同步释放，已覆盖原始复位门控；读端业务不直接组合依赖写域flush。
    // 清空后仍等待两端 up 信号的往返确认，不能提前给wr_ready或泄露旧output_valid。
    assign wr_active = (state == RUN) && wr_up && rd_up_w2 && !wr_rst_logic ;
    assign rd_active = rd_up && !rd_rst_logic ;
    assign flush_ready = wr_active;
    assign flush_busy = !wr_active;
    // flush 优先于同沿写入；生产者必须以 wr_valid && wr_ready 计算接受数量。
    assign wr_ready = wr_active && !full_reg && !flush_req;
    assign rd_valid = rd_active && output_valid;
    wire write_fire = wr_valid && wr_ready;
    wire read_fire = rd_valid && rd_ready;
    // 先算无条件+1，握手只在末级选择，避免byte_ready进入进位链再穿过Gray/空判据。
    wire [ADDR_WIDTH:0] wbin_inc = wbin + {{ADDR_WIDTH{1'b0}}, 1'b1};
    wire [ADDR_WIDTH:0] rbin_inc = rbin + {{ADDR_WIDTH{1'b0}}, 1'b1};
    wire [ADDR_WIDTH:0] wbin_next = write_fire ? wbin_inc : wbin;
    wire [ADDR_WIDTH:0] rbin_next = read_fire ? rbin_inc : rbin;
    wire [ADDR_WIDTH:0] wgray_next = (wbin_next >> 1) ^ wbin_next;
    wire full_next;
    generate if(FULL_LOOKAHEAD) begin:g_full_ahead
        // 复位wbin=0时，下一指针为1；不把这个本域镜像送往读域。
        // write_fire沿旧wbin将变成wbin+1，所以镜像同沿装Gray(wbin+2)。
        reg [ADDR_WIDTH:0] gray_ahead;
        wire [ADDR_WIDTH:0] bin_two_ahead=wbin+2;
        always @(posedge wr_clk or posedge wr_rst) begin
            if(wr_rst) gray_ahead<=1;
            else if(write_fire) gray_ahead<=(bin_two_ahead>>1)^bin_two_ahead;
        end
        assign full_next=write_fire ? (gray_ahead==(rgray_w2^FULL_MASK)) : (wgray==(rgray_w2^FULL_MASK));
    end else begin:g_full_original
        assign full_next=(wgray_next==(rgray_w2^FULL_MASK));
    end endgenerate
    wire [ADDR_WIDTH:0] rgray_next = (rbin_next >> 1) ^ rbin_next;
    wire next_empty;
    wire [ADDR_WIDTH:0] rgray_inc = (rbin_inc >> 1) ^ rbin_inc;
    wire empty_now,empty_after_pop;
    generate if(READ_PRECOMPARE) begin:g_read_precompare
        (* keep="true" *) wire current_empty = (((rbin >> 1) ^ rbin) == wgray_r2);
        (* keep="true" *) wire popped_empty = (rgray_inc == wgray_r2);
        assign empty_now=current_empty;
        assign empty_after_pop=popped_empty;
        assign next_empty=read_fire ? popped_empty : current_empty;
    end else begin:g_read_original
        assign empty_now=(((rbin >> 1) ^ rbin) == wgray_r2);
        assign empty_after_pop=(rgray_inc == wgray_r2);
        assign next_empty=(rgray_next == wgray_r2);
    end endgenerate
    // output_valid=0时没有消费，只预取当前字；有效时仅在实际消费后取下一字。
    // RAM CE与output_valid更新共用预比较结果；晚到ready只经过末级选择。
    wire fetch = rd_active && (output_valid ? (rd_ready && !empty_after_pop) : !empty_now);

    // 输出寄存器只是 RAM 当前字的副本；rbin 只在消费握手时前进。
    // 因此提前读入 output 不释放 RAM 槽，整体容量仍为 DEPTH，不会变成 DEPTH+1。
    // 可选状态流水：同步Gray先解码写入寄存器，再相减计算已缓存数据项数；不进入满判据或数据握手。
    // 采样点与full_reg使用的rgray_w2同沿，报告更保守一拍，准确容量及flush接口不变。
    wire [ADDR_WIDTH:0] rbin_for_level;
    generate if(PIPE_WR_LEVEL) begin:g_level_pipe
        (* preserve="true",allow_forward_retiming="false",allow_backward_retiming="false" *) reg [ADDR_WIDTH:0] rbin_q;
        always @(posedge wr_clk or posedge wr_rst) begin
            if(wr_rst) rbin_q<=0;
            else rbin_q<=gray_to_bin(rgray_w2);
        end
        assign rbin_for_level=rbin_q;
    end else begin:g_level_comb
        assign rbin_for_level=gray_to_bin(rgray_w2);
    end endgenerate
    wire [ADDR_WIDTH:0] write_level_difference;
    generate if(LEVEL_LOOKAHEAD) begin:g_level_lookahead
        localparam GROUPS=(ADDR_WIDTH+4)/4;
        localparam PAD=GROUPS*4-ADDR_WIDTH-1;
        wire [GROUPS*4-1:0] a={{PAD{1'b0}},wbin};
        wire [GROUPS*4-1:0] b={{PAD{1'b0}},rbin_for_level};
        wire [GROUPS*4-1:0] difference;
        genvar group_index;
        for(group_index=0;group_index<GROUPS;group_index=group_index+1) begin:g_subtract
            if(group_index==0) begin:g_first
                assign difference[group_index*4+:4]=a[group_index*4+:4]-b[group_index*4+:4];
            end else begin:g_follow
                // 每组借位独立从原低位前缀得到，不等上一组的算术输出。
                wire borrow=(a[group_index*4-1:0]<b[group_index*4-1:0]);
                assign difference[group_index*4+:4]=a[group_index*4+:4]-b[group_index*4+:4]-borrow;
            end
        end
        assign write_level_difference=difference[ADDR_WIDTH:0];
    end else begin:g_level_subtract_original
        assign write_level_difference=wbin-rbin_for_level;
    end endgenerate
    assign wr_level = wr_active ? write_level_difference : {ADDR_WIDTH+1{1'b0}};
    assign rd_level = rd_active ? gray_to_bin(wgray_r2) - rbin : {ADDR_WIDTH+1{1'b0}};

    // 不复位 RAM 或输出数据；有效位和指针隔离旧内容，平台层负责物理块型选择。
    fifo_dual_clock_ram #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .USE_EG_BRAM(USE_EG_BRAM), .RAM_IMPLEMENT(RAM_IMPLEMENT)) u_memory (
        .wr_clk(wr_clk), .wr_enable(write_fire), .wr_addr(wbin[ADDR_WIDTH-1:0]), .wr_data(wr_data),
        .rd_clk(rd_clk), .rd_enable(fetch), .rd_addr(rbin_next[ADDR_WIDTH-1:0]), .rd_data(rd_data));

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wbin <= 0; wgray <= 0; full_reg <= 0;
            rgray_w1 <= 0; rgray_w2 <= 0;
            wr_up <= 0; rd_up_w1 <= 0; rd_up_w2 <= 0;
        end else begin
            wr_up <= 1;
            rd_up_w1 <= rd_up; rd_up_w2 <= rd_up_w1;
            rgray_w1 <= rgray; rgray_w2 <= rgray_w1;
            wbin <= wbin_next; wgray <= wgray_next;
            full_reg <= full_next;
        end
    end

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rbin <= 0; rgray <= 0; output_valid <= 0;
            wgray_r1 <= 0; wgray_r2 <= 0;
            wr_up_r1 <= 0; wr_up_r2 <= 0; rd_up <= 0;
        end else begin
            wr_up_r1 <= wr_up; wr_up_r2 <= wr_up_r1;
            // 读端确认必须看到本次写端已释放，不能用清空前残留的 up 代替确认。
            rd_up <= wr_up_r2;
            wgray_r1 <= wgray; wgray_r2 <= wgray_r1;
            rbin <= rbin_next; rgray <= rgray_next;
            if (!rd_active)
                output_valid <= 0;
            else if (!output_valid || rd_ready)
                output_valid <= !next_empty;
        end
    end

    // 协调器只由外部 arst 复位，不能让自己产生的 flush_hold 清掉本次请求。
    // HOLD 至少一个写域周期；之后等待两端同步释放和新一轮 up 往返确认。
    always @(posedge wr_clk or posedge control_rst) begin
        if (control_rst) begin
            state <= STARTUP; flush_hold <= 0; flush_done <= 0;
        end else begin
            flush_done <= 0;
            case (state)
                STARTUP: if (wr_up && rd_up_w2 && !wr_rst_logic) state <= RUN;
                RUN: if (flush_req) begin
                    flush_hold <= 1; state <= HOLD;
                end
                HOLD: begin flush_hold <= 0; state <= WAIT_LINK; end
                WAIT_LINK: if (wr_up && rd_up_w2 && !wr_rst_logic) begin
                    state <= RUN; flush_done <= 1;
                end
                default: begin state <= STARTUP; flush_hold <= 1; end
            endcase
        end
    end
endmodule

`default_nettype wire
