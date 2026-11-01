`timescale 1ns/1ps
`default_nettype none

// 一次连续extent的真实SD -> 双扇区缓冲 -> 异步字节通路。
// 当前仅支持1—8个完整扇区；卡容量在请求沿检查完整extent；有效文件长度及裁剪范围由 FAT 控制模块检查。
// dst_stop_ack必须由实际消费客户端在未决写入排空后返回，不能由本层伪造。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_extent_stream #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    // TIME_W 为内部预算、已用和剩余周期计数的位宽，默认 32 位；
    // 产品由 tf_bmp_boot 传入 counter_width(SYS_BUDGET+1)=26，该宽度按构造恰好容纳预算值本身。
    // 所有比较/减法都在 TIME_W 位内完成；截止判定仍是逐周期精确的，预算值不变。
    parameter integer KEY_W=32,
    parameter integer TIME_W=32,
    // CSD容量上限位数，透传到 sd_csd_reader（默认 33 位）。
    parameter integer CARD_SECTOR_W=33,
    parameter USE_EG_BRAM=1,SHARE_CONTEXT=0,POWER_CYCLES=50000,POLL_CYCLES=50000,
    parameter INIT_TIMEOUT_CYCLES=50000000,READ_WAIT_CYCLES=5000000,
    parameter EXACT_LENGTH=0
)(
    input wire clk,dst_clk,arst,
    input wire init_valid,output wire init_ready,input wire [KEY_W-1:0] init_epoch,
    input wire init_abort_valid,output wire init_abort_ready,input wire [KEY_W-1:0] init_abort_epoch,
    output wire init_done_valid,input wire init_done_ready,
    output wire [KEY_W-1:0] init_done_epoch, output wire [31:0] init_done_register,
    output wire [3:0] init_done_code,output wire [5:0] init_done_command,output wire [7:0] init_done_r1,
    output wire card_qualified,card_block_addressed,media_fault,
    output wire card_initialized,output wire [31:0] card_last_lba,output wire [32:0] card_sector_count,
    input wire req_valid,output wire req_ready,
    input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_lba,req_budget_cycles,input wire [3:0] req_sectors,
    input wire [12:0] req_byte_count,input wire [1:0] req_user,
    input wire cancel_valid,input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire busy,result_valid,input wire result_ready,
    output wire [KEY_W-1:0] result_session,result_seq, output wire [31:0] result_count,result_elapsed,
    output wire [3:0] result_code,output wire [7:0] result_detail,
    output wire [2:0] result_bridge_reason,
    output wire [5:0] result_attempts,output wire [4:0] result_retries,
    output wire result_card_fault,
    output wire out_valid,input wire out_ready,output wire [7:0] out_data,output wire out_last,
    output wire [KEY_W-1:0] out_session,out_seq,output wire [1:0] out_user,
    output wire dst_stop_req,input wire dst_stop_ack,input wire [KEY_W-1:0] dst_ack_session,dst_ack_seq,
    output wire [10:0] src_level,dst_level,
    // 系统时钟域观察口只报告已经被桥接入口接受的字节，不增加反压来源。
    // 挂载器可在不复制扇区RAM的前提下解析CRC通过的完整扇区。
    output wire tap_valid,output wire [7:0] tap_data,output wire [8:0] tap_index,
    output wire tap_sector_last,output wire tap_last,
    output wire [KEY_W-1:0] tap_session,tap_seq,
    output wire spi_cs_n,spi_sclk,spi_mosi,input wire spi_miso,
    input wire context_claim,output wire context_available,
    input wire context_wr_enable,context_rd_enable,
    input wire [7:0] context_wr_addr,context_rd_addr,context_wr_data,
    output wire [7:0] context_rd_data
);
    wire rst_logic;
    localparam IDLE=0,ACTIVE=1,COMPLETE=2,CHECK_RANGE=3;
    localparam OK=0,CANCELED=8,DESCRIPTOR=9,DEADLINE=10,PROTOCOL=11;
    reg [1:0] state;
    // prefetch_done 仅表示读卡端已结束；还要等字节桥结果，才能发布整段完成。
    reg aborting,prefetch_done;
    reg [12:0] forwarded,active_length;
    reg [3:0] status;
    reg [TIME_W-1:0] budget,remaining_budget;
    wire [TIME_W-1:0] elapsed=budget-remaining_budget;
    // 子请求在接受沿锁存，下一拍检查完整 33 位范围，检查期间不转发数据。
    // CHECK_RANGE计入原deadline，不延长卡/桥预算；失败仍由真实两端取消/排空闭合。
    reg [32:0] req_end_cached;
    reg [31:0] card_last_cached;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    wire media_init_ready,prefetch_ready,bridge_ready;
    wire [KEY_W-1:0] active_session,active_seq;
    wire cancel_match=enabled && state!=IDLE && cancel_valid &&
                      cancel_session==active_session && cancel_seq==active_seq;
    wire cancel_now=cancel_match && status!=CANCELED;
    // 完整32位模剩余预算：remaining_budget恒等于budget-elapsed，取消后同沿停止。
    // deadline_eq_q 在剩余预算为 2 时提前建立截止条件。
    // result_elapsed 根据已用周期还原；未连接统计输出时，相关输出算术可由综合工具裁去。
    reg deadline_eq_q;
    wire deadline_now=enabled && (state==ACTIVE || state==CHECK_RANGE) && !aborting && deadline_eq_q;
    wire p_valid,p_ready,p_last,p_sector_last,p_result_valid,p_result_ready,p_stop_ack,p_busy;
    wire [7:0] p_data;
    wire [8:0] p_index;
    wire [KEY_W-1:0] p_session,p_seq,p_result_session,p_result_seq,p_stop_session,p_stop_seq;
    wire [3:0] p_code;
    wire [12:0] p_delivered;
    wire bridge_result_valid,bridge_canceled,bridge_result_ready,source_stop,bridge_busy;
    wire bridge_input_ready;
    wire p_result_match=p_result_session==active_session && p_result_seq==active_seq;
    wire p_failure=p_result_valid && (!p_result_match || p_code!=OK);
    wire [12:0] full_length={req_sectors,9'd0};
    wire [12:0] request_length=EXACT_LENGTH ? req_byte_count : full_length;
    wire [1:0] request_user=EXACT_LENGTH ? req_user : 2'b00;
    wire bridge_failure=bridge_result_valid &&
                        (bridge_canceled || result_count!={19'd0,active_length});
    wire abort_now=cancel_now || deadline_now ||
                   (!aborting && ((!prefetch_done && p_failure) || bridge_failure));
    // 与预取器stop同规：失败触发的abort_now不能组合进入bridge取消，
    // 否则会当拍遮掉报告该失败的bridge结果，形成result_valid组合环；
    // 失败沿先置aborting寄存器，下一拍再由它发起真实取消。
    wire stop_immediate=aborting || cancel_now || deadline_now;
    wire bridge_cancel=state!=IDLE && stop_immediate;
    // fatal_result不能组合反馈到预取器stop，否则会遮掉触发错误的结果形成环；
    // 与bridge取消共用同一停止锥，仅源端额外叠加source_stop。
    wire prefetch_stop=source_stop || bridge_cancel;
    assign busy=state!=IDLE;
    assign init_ready=enabled && state==IDLE && media_init_ready;
    assign req_ready=enabled && state==IDLE && !init_valid && card_qualified &&
                     prefetch_ready && bridge_ready;
    wire req_fire=req_valid && req_ready;
    // 两个子描述符在同沿提交。原子启动避免一端接收而另一端漏接。
    wire [31:0] extent_length={19'd0,request_length};
    wire [32:0] req_end={1'b0,req_lba}+{29'd0,req_sectors}-33'd1;
    wire forward_byte=forwarded<active_length;
    // 有效长度交给跨域桥；同一末扇区的尾部字节仍在源端真实排空，不能提前伪造源停止。
    assign p_ready=state==ACTIVE && (forward_byte ? bridge_input_ready : 1'b1) && !aborting && !abort_now;
    assign tap_valid=p_valid && p_ready;
    assign tap_data=p_data;assign tap_index=p_index;
    assign tap_sector_last=p_sector_last;assign tap_last=p_last;
    assign tap_session=p_session;assign tap_seq=p_seq;
    assign p_result_ready=enabled && state==ACTIVE && !prefetch_done;
    assign result_valid=enabled && state==COMPLETE && bridge_result_valid && !cancel_now;
    // 正常结果留在bridge内直到顶层结果握手；反压期间取消仍能走真实双端停止。
    assign bridge_result_ready=result_valid && result_ready;
    assign result_session=active_session;assign result_seq=active_seq;
    assign result_code=status;assign result_elapsed=elapsed;

    wire b_req_valid,b_req_ready,b_cancel_valid,b_cancel_ready,b_data_valid,b_data_ready,b_data_last;
    wire b_result_valid,b_result_ready,b_result_fault;
    wire [KEY_W-1:0] b_req_session,b_req_seq,b_cancel_session,b_cancel_seq; wire [31:0] b_req_lba;
    wire [KEY_W-1:0] b_data_session,b_data_seq,b_result_session,b_result_seq;
    wire [7:0] b_data,b_detail;
    wire [8:0] b_index;
    wire [3:0] b_code;
    wire [9:0] b_delivered;
    sd_spi_media_port #(.KEY_W(KEY_W),.CARD_SECTOR_W(CARD_SECTOR_W),.USE_EG_BRAM(USE_EG_BRAM),.SHARE_CONTEXT(SHARE_CONTEXT),.POWER_CYCLES(POWER_CYCLES),
        .POLL_CYCLES(POLL_CYCLES),.INIT_TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES),.READ_WAIT_CYCLES(READ_WAIT_CYCLES)) u_media(
        .clk(clk),.arst(arst),.init_valid(init_valid && state==IDLE && enabled),.init_ready(media_init_ready),.init_epoch(init_epoch),
        .init_abort_valid(init_abort_valid),.init_abort_ready(init_abort_ready),.init_abort_epoch(init_abort_epoch),
        .init_done_valid(init_done_valid),.init_done_ready(init_done_ready),.init_done_epoch(init_done_epoch),
        .init_done_register(init_done_register),.init_done_code(init_done_code),.init_done_command(init_done_command),.init_done_r1(init_done_r1),
        .card_qualified(card_qualified),.card_block_addressed(card_block_addressed),.media_fault(media_fault),
        .card_initialized(card_initialized),.card_last_lba(card_last_lba),.card_sector_count(card_sector_count),
        .req_valid(b_req_valid),.req_ready(b_req_ready),.req_session(b_req_session),.req_seq(b_req_seq),.req_lba(b_req_lba),
        .cancel_valid(b_cancel_valid),.cancel_ready(b_cancel_ready),.cancel_session(b_cancel_session),.cancel_seq(b_cancel_seq),
        .data_valid(b_data_valid),.data_ready(b_data_ready),.data(b_data),.data_index(b_index),.data_last(b_data_last),
        .data_session(b_data_session),.data_seq(b_data_seq),.result_valid(b_result_valid),.result_ready(b_result_ready),
        .result_session(b_result_session),.result_seq(b_result_seq),.result_code(b_code),.result_detail(b_detail),
        .result_delivered(b_delivered),.result_card_fault(b_result_fault),
        .context_claim(context_claim),.context_available(context_available),
        .context_wr_enable(context_wr_enable),.context_rd_enable(context_rd_enable),
        .context_wr_addr(context_wr_addr),.context_rd_addr(context_rd_addr),
        .context_wr_data(context_wr_data),.context_rd_data(context_rd_data),
        .spi_cs_n(spi_cs_n),.spi_sclk(spi_sclk),.spi_mosi(spi_mosi),.spi_miso(spi_miso));
    sd_sector_prefetch #(.KEY_W(KEY_W),.TIME_W(TIME_W),.USE_EG_BRAM(USE_EG_BRAM)) u_prefetch(
        .clk(clk),.arst(arst),.req_valid(req_fire),.req_ready(prefetch_ready),
        .req_session(req_session),.req_seq(req_seq),.req_lba(req_lba),.req_budget_cycles(req_budget_cycles),.req_sectors(req_sectors),
        .stop_req(prefetch_stop),.stop_session(active_session),.stop_seq(active_seq),
        .stop_ack(p_stop_ack),.stop_ack_session(p_stop_session),.stop_ack_seq(p_stop_seq),.busy(p_busy),
        .data_valid(p_valid),.data_ready(p_ready),.data(p_data),.data_index(p_index),.data_sector(),.data_sector_last(p_sector_last),.data_last(p_last),
        .data_session(p_session),.data_seq(p_seq),.result_valid(p_result_valid),.result_ready(p_result_ready),
        .result_session(p_result_session),.result_seq(p_result_seq),.result_elapsed(),.result_code(p_code),
        .result_detail(result_detail),.result_delivered(p_delivered),.result_attempts(result_attempts),
        .result_retries(result_retries),.result_card_fault(result_card_fault),
        .b_req_valid(b_req_valid),.b_req_ready(b_req_ready),.b_req_session(b_req_session),.b_req_seq(b_req_seq),.b_req_lba(b_req_lba),
        .b_cancel_valid(b_cancel_valid),.b_cancel_ready(b_cancel_ready),.b_cancel_session(b_cancel_session),.b_cancel_seq(b_cancel_seq),
        .b_data_valid(b_data_valid),.b_data_ready(b_data_ready),.b_data(b_data),.b_data_index(b_index),.b_data_last(b_data_last),
        .b_data_session(b_data_session),.b_data_seq(b_data_seq),.b_result_valid(b_result_valid),.b_result_ready(b_result_ready),
        .b_result_session(b_result_session),.b_result_seq(b_result_seq),.b_result_code(b_code),.b_result_detail(b_detail),
        .b_result_delivered(b_delivered),.b_result_card_fault(b_result_fault));
    // 单次扩展最多15扇区(4位req_sectors)=7680B，13位长度足够；桥内对超长请求显式拒绝。
    file_byte_cdc #(.KEY_W(KEY_W),.LEN_W(13),.ADDR_WIDTH(10),.USE_EG_BRAM(USE_EG_BRAM),.ENABLE_USER(1)) u_bridge(
        .arst(arst),.src_clk(clk),.dst_clk(dst_clk),.req_valid(req_fire),.req_ready(bridge_ready),
        .req_session(req_session),.req_seq(req_seq),.req_length(extent_length),.req_user(request_user),
        .in_valid(state==ACTIVE && p_valid && forward_byte && !aborting && !abort_now),.in_ready(bridge_input_ready),
        .in_data(p_data),.in_last(forwarded==active_length-13'd1),
        .in_session(p_session),.in_seq(p_seq),.cancel_valid(bridge_cancel),.cancel_session(active_session),.cancel_seq(active_seq),
        .src_stop_req(source_stop),.src_session(active_session),.src_seq(active_seq),
        .src_stop_ack(p_stop_ack),.src_ack_session(p_stop_session),.src_ack_seq(p_stop_seq),
        .result_valid(bridge_result_valid),.result_ready(bridge_result_ready),.result_canceled(bridge_canceled),
        .result_reason(result_bridge_reason),.result_count(result_count),.busy(bridge_busy),
        .out_valid(out_valid),.out_ready(out_ready),.out_data(out_data),.out_last(out_last),
        .out_session(out_session),.out_seq(out_seq),.out_user(out_user),
        .dst_stop_req(dst_stop_req),.dst_stop_ack(dst_stop_ack),.dst_ack_session(dst_ack_session),.dst_ack_seq(dst_ack_seq),
        .src_level(src_level),.dst_level(dst_level));
    always @(posedge clk or posedge rst) begin
        if(rst) begin state<=IDLE;aborting<=0;prefetch_done<=0;forwarded<=0;active_length<=0;status<=OK;budget<=0;remaining_budget<=0;req_end_cached<=0;card_last_cached<=0;
            deadline_eq_q<=0;end
        else begin
            if(req_fire) begin
                state<=CHECK_RANGE;prefetch_done<=0;forwarded<=0;active_length<=request_length;
                req_end_cached<=req_end;card_last_cached<=card_last_lba;
                budget<=req_budget_cycles[TIME_W-1:0];remaining_budget<=req_budget_cycles[TIME_W-1:0];aborting<=0;status<=OK;
                // 与请求真实接受同沿装载剩余预算，剩余1拍时保留原截止提示。
                deadline_eq_q<=req_budget_cycles[TIME_W-1:0]==1;
                // 无效描述符也通过真实桥取消闭合，保证键和源/目标描述符一致。
                if(req_sectors==0 || req_sectors>8 || request_length==0 ||
                   request_length>full_length) begin status<=DESCRIPTOR;aborting<=1;end
                else if(req_budget_cycles[TIME_W-1:0]==0) begin status<=DEADLINE;aborting<=1;end
            end
            if(state==CHECK_RANGE) begin
                state<=ACTIVE;
                // 剩余预算递减时同步更新下一拍是否到期；已取消时保持剩余预算及到期条件。
                if(!aborting) begin remaining_budget<=remaining_budget-1'b1;deadline_eq_q<=remaining_budget==2;end
                if(req_end_cached[32] || req_end_cached>{1'b0,card_last_cached}) begin
                    status<=DESCRIPTOR;aborting<=1;
                end else if(deadline_now) begin status<=DEADLINE;aborting<=1;end
            end
            if(state==ACTIVE) begin
                if(p_valid && p_ready && forward_byte) forwarded<=forwarded+13'd1;
                if(!aborting) begin remaining_budget<=remaining_budget-1'b1;deadline_eq_q<=remaining_budget==2;end
                if(p_result_valid && p_result_ready) begin
                    prefetch_done<=1;
                    if(!aborting && p_failure) begin
                        aborting<=1;status<=!p_result_match || p_code==CANCELED ? PROTOCOL : p_code;
                    end
                end
                if(!aborting && bridge_failure) begin aborting<=1;status<=PROTOCOL;end
                if(prefetch_done && bridge_result_valid && (!aborting || bridge_canceled) && !abort_now) state<=COMPLETE;
                if(deadline_now) begin aborting<=1;status<=DEADLINE;end
            end
            if(state==COMPLETE && result_valid && result_ready) state<=IDLE;
            // 匹配取消优先于同沿正常结果。回到ACTIVE等待桥的真实取消结果。
            if(cancel_now) begin state<=ACTIVE;aborting<=1;status<=CANCELED;end
        end
    end
`ifdef FORMAL
    // 剩余预算为 1 时，寄存的截止条件有效。
    always @* if(!rst) assert(deadline_eq_q==(remaining_budget==1));
`endif
endmodule
`default_nettype wire
