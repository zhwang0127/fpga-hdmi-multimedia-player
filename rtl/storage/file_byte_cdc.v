`timescale 1ns/1ps
`default_nettype none

// 单在途文件字节通路。键/目录长度走mailbox，纯8bit数据走FIFO。
// 完成表示字节已被下游接受；格式检查、CRC 和 SDRAM 写入由后级确认。
// 取消必须等两端真实停流/排空，再flush；外部确认缺失时保持占用。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module file_byte_cdc #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32,
    // LEN_W = 本桥长度/计数(length/sent/received/delivered/last/penultimate)位宽。
    // 默认 32 位；sd_extent_stream 传13（单次扩展最多15扇区=7680B<8192）。
    // req_length 高位非0不会被静默截断：IDLE 里与“长度为0”一样走 reason=1 的拒绝路径（ABORT_SEND）。
    parameter integer LEN_W=32,
    parameter ADDR_WIDTH = 10,
    parameter USE_EG_BRAM = 1,
    parameter ENABLE_USER = 0
) (
    input wire arst, src_clk, dst_clk,
    input wire req_valid, output wire req_ready,
    input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_length,
    input wire [1:0] req_user,
    input wire in_valid, output wire in_ready,
    input wire [7:0] in_data, input wire in_last,
    input wire [KEY_W-1:0] in_session,in_seq,
    input wire cancel_valid, input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire src_stop_req,
    output wire [KEY_W-1:0] src_session,src_seq,
    input wire src_stop_ack, input wire [KEY_W-1:0] src_ack_session,src_ack_seq,
    output wire result_valid, input wire result_ready,
    output wire result_canceled, output wire [2:0] result_reason,
    output wire [31:0] result_count, output wire busy,
    output wire out_valid, input wire out_ready,
    output wire [7:0] out_data, output wire out_last,
    output wire [KEY_W-1:0] out_session,out_seq,
    output wire [1:0] out_user,
    output wire dst_stop_req,
    input wire dst_stop_ack, input wire [KEY_W-1:0] dst_ack_session,dst_ack_seq,
    output wire [ADDR_WIDTH:0] src_level, dst_level
);
    wire src_rst_logic;
    wire dst_rst_logic;
    localparam [3:0] IDLE=0, SEND=1, WAIT_END=2, COMPLETE=3,
                     ABORT_SEND=4, ABORT_WAIT=5, FLUSH_SEND=6,
                     FLUSH_WAIT=7, ABORT_COMPLETE=8;
    // 原因：0正常、1零长度、2字节键错、3last位置错、4主动取消、5返回计数错。
    reg [3:0] state;
    reg [KEY_W-1:0] active_session,active_seq; reg [LEN_W-1:0] length,sent,received;
    reg [2:0] reason;
    reg descriptor_latched, producer_stopped, consumer_stopped;
    // 业务门控只使用本端reset_sync输出；该rst异步置位、三级同步释放，
    // 已覆盖arst有效区间。原始arst仅进入复位链，避免sys复位直接穿过mem数据/握手锥。
    wire src_rst, dst_rst;
    reset_sync u_src_reset(.clk(src_clk), .arst(arst), .rst(src_rst),.logic_rst(src_rst_logic));
    reset_sync u_dst_reset(.clk(dst_clk), .arst(arst), .rst(dst_rst),.logic_rst(dst_rst_logic));
    wire cancel_match = cancel_valid && cancel_session==active_session && cancel_seq==active_seq;
    wire aborting = state>=ABORT_SEND;
    assign src_session=active_session; assign src_seq=active_seq;
    assign busy=(state!=IDLE);
    assign src_stop_req=aborting && !src_rst_logic;
    assign result_valid=(state==COMPLETE || state==ABORT_COMPLETE) && !src_rst_logic &&
                        !(state==COMPLETE && cancel_match);
    assign result_canceled=(state==ABORT_COMPLETE);
    assign result_reason=reason; assign result_count=received;

    // 描述符、停止命令、响应各走自己的mailbox，避免停止命令被数据反压堵住。
    wire descriptor_ready, descriptor_done, descriptor_active, descriptor_valid, descriptor_take;
    // 三个mailbox的数据宽度随KEY_W变化（CDC结构不变：仍是整包源端保持+翻转位双级同步）。
    // 描述符={session,seq,user[1:0],length}，取消={session,seq}，结果={session,seq,count,canceled}。
    // KEY_W 决定命令、取消和结果 mailbox 中的事务键宽度，所有端口须同宽。
    wire req_too_long=(LEN_W<32) && ((req_length>>LEN_W)!=0);
    wire [LEN_W-1:0] req_length_n=req_too_long ? {LEN_W{1'b0}} : req_length[LEN_W-1:0];
    localparam integer DESC_W=2*KEY_W+2+LEN_W;
    localparam integer STOP_W=2*KEY_W;
    localparam integer REPLY_W=2*KEY_W+LEN_W+1;
    wire [DESC_W-1:0] descriptor;
    wire [1:0] request_user=ENABLE_USER ? req_user : 2'b00;
    wire fifo_wr_ready, fifo_wr_active, fifo_rd_valid;
    wire flush_ready, flush_done;
    wire response_active, cancel_active;
    assign req_ready=state==IDLE && !src_rst_logic && fifo_wr_active &&
                     descriptor_ready && descriptor_active && response_active && cancel_active;
    cdc_mailbox #(.DATA_WIDTH(DESC_W)) u_descriptor(
        .arst(arst), .src_clk(src_clk), .src_valid(req_valid && req_ready), .src_ready(descriptor_ready),
        .src_data({req_session,req_seq,request_user,req_length_n}), .src_active(descriptor_active),
        .src_busy(), .src_done(descriptor_done), .dst_clk(dst_clk), .dst_valid(descriptor_valid),
        .dst_ready(descriptor_take), .dst_data(descriptor), .dst_active());

    // cancel走独立控制通道，不排在被数据背压阻塞的字节FIFO之后。
    // 等描述符真实目标锁存，保证停止命令不越过该事务的描述符。
    wire cancel_ready, stop_valid, stop_ready;
    wire [STOP_W-1:0] stop_key;
    cdc_mailbox #(.DATA_WIDTH(STOP_W)) u_cancel(
        .arst(arst), .src_clk(src_clk), .src_valid(state==ABORT_SEND && descriptor_latched),
        .src_ready(cancel_ready), .src_data({active_session,active_seq}), .src_active(cancel_active),
        .src_busy(), .src_done(), .dst_clk(dst_clk), .dst_valid(stop_valid),
        .dst_ready(stop_ready), .dst_data(stop_key), .dst_active());

    // 数据也要带当前事务键；FIFO里只放字节，身份先在入口核对。
    wire input_key_good=(in_session==active_session && in_seq==active_seq);
    wire input_last_good=(in_last == (sent==length-1'b1));
    assign in_ready=state==SEND && !src_rst_logic && !cancel_match && fifo_wr_ready;
    wire input_fire=in_valid && in_ready;
    // 接受非法字段触发取消，但非法字节不进入FIFO；sent只统计实际入队的合法字节。
    wire fifo_write = input_fire && input_key_good && input_last_good;
    wire fifo_read;
    async_fifo #(.DATA_WIDTH(8), .ADDR_WIDTH(ADDR_WIDTH), .USE_EG_BRAM(USE_EG_BRAM),
                 .RAM_IMPLEMENT("9K")) u_bytes(
        .arst(arst), .wr_clk(src_clk), .wr_valid(fifo_write), .wr_data(in_data),
        .wr_ready(fifo_wr_ready), .wr_active(fifo_wr_active), .wr_level(src_level),
        .flush_req(state==FLUSH_SEND), .flush_ready(flush_ready), .flush_busy(), .flush_done(flush_done),
        .rd_clk(dst_clk), .rd_ready(fifo_read), .rd_valid(fifo_rd_valid), .rd_data(out_data),
        .rd_active(), .rd_level(dst_level));

    reg has_descriptor, closed, stopping, normal_pending, abort_pending;
    reg [KEY_W-1:0] dst_session,dst_seq; reg [LEN_W-1:0] dst_length,delivered;
    // 目录长度锁存时预计算末字下标；后续末字判断直接比较已锁存的下标。
    reg [LEN_W-1:0] dst_last_index;
    // 把末字比较寄存在消费计数同拍，避免delivered一路穿过媒体/SDRAM许可。
    // next delivered==last 等价于 current delivered==last-1（32位模运算）。
    reg [LEN_W-1:0] dst_penultimate_index;
    reg dst_last_q;
    reg [1:0] dst_user;
    assign out_session=dst_session; assign out_seq=dst_seq;
    assign out_user=dst_user;
    wire stop_match=stop_valid && has_descriptor && stop_key=={dst_session,dst_seq};
    assign descriptor_take=(!has_descriptor || closed) && !stopping && !normal_pending && !abort_pending;
    assign stop_ready=has_descriptor;
    assign out_valid=has_descriptor && !closed && !stopping && !stop_match &&
                     !dst_rst_logic && fifo_rd_valid;
    // last由已锁存目录长度及消费计数重建，不能受下一事务输入字段影响。
    assign out_last=dst_last_q;
    assign fifo_read=out_valid && out_ready;
    assign dst_stop_req=stopping && !dst_rst_logic;

    wire reply_src_ready, reply_valid;
    wire [REPLY_W-1:0] reply;
    // 正常结果若已在反压中不能被取消结果覆盖；先发送正常，再发送取消结果。
    cdc_mailbox #(.DATA_WIDTH(REPLY_W)) u_result(
        .arst(arst), .src_clk(dst_clk), .src_valid(normal_pending || abort_pending),
        .src_ready(reply_src_ready), .src_data({dst_session,dst_seq,delivered,!normal_pending}),
        .src_active(), .src_busy(), .src_done(), .dst_clk(src_clk), .dst_valid(reply_valid),
        .dst_ready(1'b1), .dst_data(reply), .dst_active(response_active));
    wire reply_match=reply_valid && reply[REPLY_W-1:LEN_W+1]=={active_session,active_seq};
    wire source_ack_match=src_stop_ack && src_ack_session==active_session && src_ack_seq==active_seq;

    always @(posedge src_clk or posedge src_rst) begin
        if (src_rst) begin
            state<=IDLE; active_session<=0; active_seq<=0; length<=0; sent<=0; received<=0;
            reason<=0; descriptor_latched<=0; producer_stopped<=0; consumer_stopped<=0;
        end else begin
            if (descriptor_done) descriptor_latched<=1;
            if (aborting && source_ack_match) producer_stopped<=1;
            case (state)
                IDLE: if (req_valid && req_ready) begin
                    active_session<=req_session; active_seq<=req_seq; length<=req_length_n;
                    sent<=0; received<=0; reason<=req_length_n==0 ? 3'd1 : 3'd0;
                    descriptor_latched<=0; producer_stopped<=0; consumer_stopped<=0;
                    state<=req_length_n==0 ? ABORT_SEND : SEND;
                end
                SEND: if (input_fire) begin
                    if (!input_key_good) begin reason<=2; state<=ABORT_SEND; end
                    else if (!input_last_good) begin reason<=3; state<=ABORT_SEND; end
                    else begin sent<=sent+1'b1; if (sent==length-1'b1) state<=WAIT_END; end
                end
                WAIT_END: if (reply_match && !reply[0]) begin
                    received<=reply[LEN_W:1];
                    if (reply[LEN_W:1]==length && sent==length) state<=COMPLETE;
                    else begin reason<=5; state<=ABORT_SEND; end
                end
                COMPLETE: if (result_valid && result_ready) state<=IDLE;
                ABORT_SEND: if (descriptor_latched && cancel_ready) state<=ABORT_WAIT;
                ABORT_WAIT: begin
                    // 正常尾包在取消期间只丢弃；必须等待明确的取消排空返回。
                    if (reply_match && reply[0]) begin consumer_stopped<=1; received<=reply[LEN_W:1]; end
                    if ((producer_stopped || source_ack_match) &&
                        (consumer_stopped || (reply_match && reply[0]))) state<=FLUSH_SEND;
                end
                FLUSH_SEND: if (flush_ready) state<=FLUSH_WAIT;
                FLUSH_WAIT: if (flush_done) state<=ABORT_COMPLETE;
                ABORT_COMPLETE: if (result_valid && result_ready) state<=IDLE;
                default: begin reason<=5; state<=ABORT_SEND; end
            endcase
            // 匹配取消优先于同沿输入及正常结果输出；旧session/seq取消无效。
            if (cancel_match && (state==SEND || state==WAIT_END || state==COMPLETE)) begin
                reason<=4; state<=ABORT_SEND;
            end
        end
    end

    always @(posedge dst_clk or posedge dst_rst) begin
        if (dst_rst) begin
            has_descriptor<=0; closed<=0; stopping<=0; normal_pending<=0; abort_pending<=0;
            dst_session<=0; dst_seq<=0; dst_user<=0; dst_length<=0; dst_last_index<={LEN_W{1'b1}}; dst_penultimate_index<={{(LEN_W-1){1'b1}},1'b0}; dst_last_q<=0; delivered<=0;
        end else begin
            if (reply_src_ready) begin
                if (normal_pending) normal_pending<=0;
                else if (abort_pending) abort_pending<=0;
            end
            if (descriptor_valid && descriptor_take) begin
                has_descriptor<=1; closed<=descriptor[LEN_W-1:0]==0;
                dst_session<=descriptor[DESC_W-1:KEY_W+LEN_W+2]; dst_seq<=descriptor[KEY_W+LEN_W+1:LEN_W+2];
                dst_user<=descriptor[LEN_W+1:LEN_W];
                dst_length<=descriptor[LEN_W-1:0]; dst_last_index<=descriptor[LEN_W-1:0]-1'b1;
                dst_penultimate_index<=descriptor[LEN_W-1:0]-2'd2;dst_last_q<=descriptor[LEN_W-1:0]==1;delivered<=0;
            end
            if (fifo_read) begin
                delivered<=delivered+1'b1;dst_last_q<=delivered==dst_penultimate_index;
                if (out_last) begin closed<=1; normal_pending<=1; end
            end
            if (stop_valid && stop_ready && stop_match) begin
                stopping<=1; closed<=1;
            end
            // stop_ack代表消费客户端无未决写入；不能用out_ready或FIFO空代替。
            if (stopping && dst_stop_ack && dst_ack_session==dst_session && dst_ack_seq==dst_seq) begin
                stopping<=0; abort_pending<=1;
            end
        end
    end
endmodule

`default_nettype wire
