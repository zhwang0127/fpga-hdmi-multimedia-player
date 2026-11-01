`timescale 1ns/1ps
`default_nettype none

// CMD17只读客户端：独占spi_byte_master业务口，clk域；不伪造卡初始化。
// 整扇区先进入512B RAM，通过CRC16并完成CS高尾时钟后才允许下游读取。
// 单在途、带完整session/seq；取消与成功同沿时取消优先。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_spi_sector_reader #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32,
    parameter USE_EG_BRAM=1,SHARE_CONTEXT=0,
    parameter WAIT_CYCLES=5000000, // 50MHz下100ms，覆盖协议允许的等待时间。
    parameter R1_BYTES=16         // 等待 R1 响应时最多读取的字节数。
) (
    input wire clk, arst,
    input wire card_qualified, card_requalified,
    input wire [31:0] card_last_lba,
    output reg media_fault,
    input wire req_valid, output wire req_ready,
    input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_lba,
    input wire req_block_addressed,
    input wire cancel_valid, output wire cancel_ready,
    input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire data_valid, input wire data_ready,
    output wire [7:0] data,
    output wire [8:0] data_index,
    output wire data_last,
    output wire [KEY_W-1:0] data_session,data_seq,
    output wire result_valid, input wire result_ready,
    output wire [KEY_W-1:0] result_session,result_seq,
    output wire [3:0] result_code,
    output wire [7:0] result_detail,
    output wire [9:0] result_delivered,
    output wire result_card_fault,
    output wire busy,
    output reg spi_cs_n,
    output wire spi_tx_valid, input wire spi_tx_ready,
    output reg [7:0] spi_tx_data,
    output wire [15:0] spi_tx_half_period,
    input wire spi_rx_valid, output wire spi_rx_ready,
    input wire [7:0] spi_rx_data,
    output wire spi_stop_req, input wire spi_stop_ack,
    input wire context_claim,output wire context_available,
    input wire context_wr_enable,context_rd_enable,
    input wire [7:0] context_wr_addr,context_rd_addr,context_wr_data,
    output wire [7:0] context_rd_data
);
    wire rst_logic;
    localparam IDLE=0, READY=1, COMMAND=2, R1=3, TOKEN=4, BLOCK=5,
               QUIESCE=6, TAIL=7, FINAL_STOP=8, PREFETCH=9, DELIVER=10, RESULT=11;
    localparam OK=0, ADDRESS=1, READY_TIMEOUT=2, R1_TIMEOUT=3, R1_ERROR=4,
               TOKEN_TIMEOUT=5, TOKEN_ERROR=6, CRC_ERROR=7, CANCELED=8;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    reg [3:0] state, status;
    reg [KEY_W-1:0] session_key,seq_key; reg [31:0] argument;
    // 计数位宽 = 恰好容纳“阈值本身”的位数（counter_width(P+1)）。
    // 计数器在达到阈值前递增，到达后离开计数状态或置位完成标志
    // 位宽需覆盖阈值本身，避免计数到达上限前发生回绕。
    function integer counter_width;
        input integer value;
        integer v;
        begin v=value-1;counter_width=1;while(v>1) begin v=v>>1;counter_width=counter_width+1;end end
    endfunction
    localparam integer WAIT_W=counter_width(WAIT_CYCLES+1); // 产品5,000,000 -> 23位
    reg [WAIT_W-1:0] wait_count;
    reg [2:0] command_index;
    reg [6:0] command_crc;
    reg [7:0] response_count, detail, crc_high;
    reg [9:0] sent_count, received_count, delivered_count;
    reg [8:0] read_index;
    reg issued, canceled, card_fault;
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    assign cancel_ready=enabled && state!=IDLE && !canceled &&
                        cancel_session==session_key && cancel_seq==seq_key;
    wire cancel_now=cancel_valid && cancel_ready;
    // READY 尚未发送 CMD17，可立即取消；COMMAND/R1/TOKEN 收到取消时只锁存取消标志。
    // 命令发送后仍按协议等待令牌，并排空数据块及 CRC，最后拉高 CS 发送尾时钟。
    // 等待受 WAIT_CYCLES 截止约束；数据块、CRC 及 CS 高电平尾时钟全部结束后才释放 SPI 总线。
    wire abort_early=(state==READY);
    wire serial_state=(state==READY || state==COMMAND || state==R1 ||
                       state==TOKEN || state==TAIL);
    assign req_ready=enabled && state==IDLE && card_qualified && !media_fault && !cancel_valid && !(SHARE_CONTEXT && context_claim);
    assign busy=state!=IDLE;
    assign spi_tx_valid=enabled && ((serial_state && !issued && !(cancel_now && abort_early)) ||
                                   (state==BLOCK && sent_count<10'd514));
    assign spi_tx_half_period=16'd1; // 初始化成功后的数据读取时钟固定为 25MHz；初始化要另用H=63。
    assign spi_rx_ready=enabled && (serial_state || state==BLOCK || state==QUIESCE || state==FINAL_STOP);
    assign spi_stop_req=enabled && (state==QUIESCE || state==FINAL_STOP);
    wire tx_fire=spi_tx_valid && spi_tx_ready;
    wire rx_fire=spi_rx_valid && spi_rx_ready;
    assign data_valid=enabled && state==DELIVER && !canceled && !cancel_now;
    wire data_fire=data_valid && data_ready;
    assign data_index=read_index;
    assign data_last=read_index==9'd511;
    assign data_session=session_key;
    assign data_seq=seq_key;
    assign result_valid=enabled && state==RESULT && !cancel_now;
    assign result_session=session_key;
    assign result_seq=seq_key;
    assign result_code=status;
    assign result_detail=detail;
    assign result_delivered=delivered_count;
    assign result_card_fault=card_fault;

    // CRC7逐字节计算五个命令正文；尾字节为CRC7左移一位加结束位1。
    function [6:0] crc7_byte;
        input [6:0] previous;
        input [7:0] value;
        reg [6:0] remainder;
        integer k;
        begin
            remainder=previous;
            for(k=7;k>=0;k=k-1)
                if(remainder[6]^value[k]) remainder={remainder[5:0],1'b0} ^ 7'h09;
                else remainder={remainder[5:0],1'b0};
            crc7_byte=remainder;
        end
    endfunction
    always @* begin
        spi_tx_data=8'hff;
        if(state==COMMAND) case(command_index)
            0:spi_tx_data=8'h51;
            1:spi_tx_data=argument[31:24];
            2:spi_tx_data=argument[23:16];
            3:spi_tx_data=argument[15:8];
            4:spi_tx_data=argument[7:0];
            default:spi_tx_data={command_crc,1'b1};
        endcase
    end

    wire [15:0] block_crc;
    wire block_write=rx_fire && state==BLOCK && received_count<10'd512;
    crc16_sd u_crc(.clk(clk),.rst(rst_logic),.clear(state!=BLOCK),
                   .data_valid(block_write),.data(spi_rx_data),.crc(block_crc));
    // 默认使用 512B 缓冲；共享模式使用同一物理 1024×8 位块：
    // 0..511为扇区，512..767为32x64上下文，768..1023尚未分配。
    // 可用信号不依赖claim，父级只在整个 extent 传输或初始化事务结束后才申请。
    // 卡事务在途时拒绝上下文访问；非法claim也不能夺走在途SD写入。
    assign context_available=SHARE_CONTEXT && enabled && state==IDLE && card_qualified &&
                             !media_fault && !req_valid && !cancel_valid && !card_requalified;
    wire context_grant=context_claim && context_available;
    wire [9:0] sector_wr_addr={context_grant,context_grant ? {1'b0,context_wr_addr} : received_count[8:0]};
    wire [9:0] sector_rd_addr={context_grant,context_grant ? {1'b0,context_rd_addr} : read_index};
    assign context_rd_data=SHARE_CONTEXT ? data : 8'd0;
    localparam RAM_ADDR_WIDTH=SHARE_CONTEXT ? 10 : 9;
    fifo_dual_clock_ram #(.DATA_WIDTH(8),.ADDR_WIDTH(RAM_ADDR_WIDTH),.USE_EG_BRAM(USE_EG_BRAM),
                          .RAM_IMPLEMENT("9K"),.PHYSICAL_BYTE(SHARE_CONTEXT)) u_sector(
        .wr_clk(clk),.wr_enable(context_grant ? context_wr_enable : block_write && !canceled && !cancel_now),
        .wr_addr(sector_wr_addr[RAM_ADDR_WIDTH-1:0]),.wr_data(context_grant ? context_wr_data : spi_rx_data),
        .rd_clk(clk),.rd_enable(context_grant ? context_rd_enable : enabled && state==PREFETCH),
        .rd_addr(sector_rd_addr[RAM_ADDR_WIDTH-1:0]),.rd_data(data));

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;status<=OK;session_key<=0;seq_key<=0;argument<=0;wait_count<=0;
            command_index<=0;command_crc<=0;response_count<=0;detail<=0;crc_high<=0;
            sent_count<=0;received_count<=0;delivered_count<=0;read_index<=0;
            issued<=0;canceled<=0;card_fault<=0;spi_cs_n<=1;media_fault<=1;
        end else begin
            // 仅在空闲时接受初始化完成信号，以清除 media_fault 并允许继续读卡。
            if(state==IDLE && card_qualified && card_requalified) media_fault<=0;
            if(cancel_now) canceled<=1;
            if(tx_fire && serial_state) issued<=1;
            if(rx_fire && serial_state) issued<=0;
            if(state==READY || state==TOKEN) wait_count<=wait_count+1'b1;
            case(state)
                IDLE: if(req_valid && req_ready) begin
                    session_key<=req_session;seq_key<=req_seq;status<=OK;detail<=0;
                    delivered_count<=0;read_index<=0;canceled<=0;card_fault<=0;issued<=0;
                    command_index<=0;command_crc<=0;response_count<=0;wait_count<=0;
                    argument<=req_block_addressed ? req_lba : {req_lba[22:0],9'd0};
                    if(req_lba>card_last_lba || (!req_block_addressed && req_lba[31:23]!=0)) begin
                        status<=ADDRESS;state<=RESULT; // 移位前检查，不发截断后的卡地址。
                    end else begin spi_cs_n<=0;state<=READY;end
                end
                READY: begin
                    if(rx_fire && spi_rx_data==8'hff) state<=COMMAND;
                    else if(wait_count==WAIT_CYCLES-1) begin
                        status<=READY_TIMEOUT;card_fault<=1;media_fault<=1;state<=QUIESCE;
                    end
                end
                COMMAND: begin
                    if(tx_fire && command_index<5) command_crc<=crc7_byte(command_crc,spi_tx_data);
                    if(rx_fire) begin
                        if(command_index==5) begin response_count<=0;state<=R1;end
                        else command_index<=command_index+3'd1;
                    end
                end
                R1: if(rx_fire) begin
                    if(!spi_rx_data[7]) begin
                        detail<=spi_rx_data;
                        if(spi_rx_data==8'h00) begin state<=TOKEN;wait_count<=0;end
                        else begin status<=R1_ERROR;card_fault<=1;media_fault<=1;state<=QUIESCE;end
                    end else if(response_count==R1_BYTES-1) begin
                        status<=R1_TIMEOUT;card_fault<=1;media_fault<=1;state<=QUIESCE;
                    end else response_count<=response_count+8'd1;
                end
                TOKEN: begin
                    if(rx_fire && spi_rx_data==8'hfe) begin
                        sent_count<=0;received_count<=0;state<=BLOCK;
                    end else if(rx_fire && spi_rx_data!=8'hff) begin
                        detail<=spi_rx_data;status<=TOKEN_ERROR;card_fault<=1;media_fault<=1;state<=QUIESCE;
                    end else if(wait_count==WAIT_CYCLES-1) begin
                        status<=TOKEN_TIMEOUT;card_fault<=1;media_fault<=1;state<=QUIESCE;
                    end
                end
                BLOCK: begin
                    // 全扇区预留空间；连续tx，不把下游data_ready接到SPI字节反压。
                    if(tx_fire) sent_count<=sent_count+10'd1;
                    if(rx_fire) begin
                        received_count<=received_count+10'd1;
                        if(received_count==10'd512) crc_high<=spi_rx_data;
                        if(received_count==10'd513) begin
                            if({crc_high,spi_rx_data}!=block_crc) status<=CRC_ERROR;
                            state<=QUIESCE;
                        end
                    end
                end
                QUIESCE: if(spi_stop_ack) begin
                    // 必须先收回SPI在途字节与所有结果，才改变CS。
                    spi_cs_n<=1;issued<=0;state<=TAIL;
                end
                TAIL: if(rx_fire) state<=FINAL_STOP; // CS高实际再发8个时钟，结果消费后再停。
                FINAL_STOP: if(spi_stop_ack) begin
                    if(canceled || cancel_now) begin status<=CANCELED;state<=RESULT;end
                    else if(status!=OK) state<=RESULT;
                    else state<=PREFETCH;
                end
                PREFETCH: state<=DELIVER; // RAM固有同步读一拍，进入DELIVER后才有效。
                DELIVER: if(data_fire) begin
                    delivered_count<=delivered_count+10'd1;
                    if(read_index==9'd511) state<=RESULT;
                    else begin read_index<=read_index+9'd1;state<=PREFETCH;end
                end
                RESULT: if(result_valid && result_ready) state<=IDLE;
                default:begin state<=IDLE;spi_cs_n<=1;media_fault<=1;end
            endcase
            // READY 取消：卡还没收到命令，只结束当前 SPI 字节，不因取消而设置 media_fault。
            // COMMAND/R1/TOKEN/BLOCK 取消：仅由上面的 canceled 记账，事务走完后 FINAL_STOP 报 CANCELED。
            if(cancel_now) begin
                if(abort_early) begin
                    state<=QUIESCE;status<=CANCELED;
                end else if(state==PREFETCH || state==DELIVER || state==RESULT) begin
                    state<=RESULT;status<=CANCELED;
                end
            end
        end
    end
endmodule

`default_nettype wire
