`timescale 1ns/1ps
// 管理单个媒体装载事务，源端结束且写入端完成后进入 READY，允许上层提交媒体。
// session 和 seq 共同标识本次任务；取消后须等在途数据和写请求全部结束，才能复用标识和缓冲区。
// READY 状态继续保留缓冲区，直到上层接受；取消或失败时进入 ABORT，等待匹配的停止确认。
// rst 为同步高有效复位，须与数据通路共同复位，或在数据传输结束后再复位本模块。
// 单独复位会清除本地占用记录，但不会停止 SDRAM 写入，不能据此允许覆盖原缓冲区。
module load_txn #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32
)(
    input wire clk,input wire rst,
    // 请求在 req_valid && req_ready 的上升沿被接受，描述符随后保持不变。
    input wire req_valid,output wire req_ready,
    input wire [KEY_W-1:0] req_session,input wire [KEY_W-1:0] req_seq,
    input wire [1:0] req_slot,
    // 两端可以采用不同计数单位，但各自的 expected/count 必须使用同一单位。
    // 本模块比较最终计数，实际已接受数据的计数器由对应数据通路实现。
    input wire [31:0] expected_src_count,input wire [31:0] expected_sink_count,
    // 源端和写入端分别携带标识，可独立、任意顺序或同拍报告完成/错误。
    input wire src_done,input wire src_error,input wire [KEY_W-1:0] src_session,
    input wire [KEY_W-1:0] src_seq,input wire [31:0] src_count,
    input wire sink_done,input wire sink_error,input wire [KEY_W-1:0] sink_session,
    input wire [KEY_W-1:0] sink_seq,input wire [31:0] sink_count,
    // cancel 必须指向当前事务；下游确认突发写、命令与 CDC 全部排空后，
    // 才能返回同标识的 abort_ack。没有 ack 就持续保留占用，不自动超时释放。
    input wire cancel,input wire [KEY_W-1:0] cancel_session,input wire [KEY_W-1:0] cancel_seq,
    input wire abort_ack,input wire [KEY_W-1:0] ack_session,input wire [KEY_W-1:0] ack_seq,
    // ready_valid 与 ready_accept 同时有效时，向上层提交装载结果，后续 vblank 帧交换由播放控制器负责。
    output wire ready_valid,input wire ready_accept,
    output wire busy,output wire abort_req,
    output reg [KEY_W-1:0] active_session,output reg [KEY_W-1:0] active_seq,
    output reg [1:0] active_slot,output reg [2:0] error_reason // 0无错误/1源端/2写入端/3取消。
);
    // IDLE 接受请求；ACTIVE 等双完成；READY 等待上层接受装载结果；ABORT 等排空 ack。
    localparam IDLE=2'd0,ACTIVE=2'd1,READY=2'd2,ABORT=2'd3;
    reg [1:0] state;
    reg src_seen,sink_seen; // 锁存键匹配且成功的完成事件，完成脉冲不需要同时到达。
    reg [31:0] src_expected,sink_expected;
    // 比较两级标识，旧装载或旧介质会话的迟到回包无法完成当前事务。
    wire src_match=(src_session==active_session)&&(src_seq==active_seq);
    wire sink_match=(sink_session==active_session)&&(sink_seq==active_seq);
    wire cancel_match=cancel&&(cancel_session==active_session)&&(cancel_seq==active_seq);
    // 完成时计数必须精确相等；不足和超量都失败，显式 error 也立即失败。
    wire src_bad=src_match&&(src_error||(src_done&&(src_count!=src_expected)));
    wire sink_bad=sink_match&&(sink_error||(sink_done&&(sink_count!=sink_expected)));
    wire src_good=src_match&&src_done&&!src_error&&(src_count==src_expected);
    wire sink_good=sink_match&&sink_done&&!sink_error&&(sink_count==sink_expected);
    assign req_ready=(state==IDLE)&&!rst;
    // A same-cycle cancellation must also suppress the downstream handshake.
    // 不只在状态机里优先处理取消，还同时拉低 ready_valid，避免下游在
    // 同一个上升沿按 ready_valid && ready_accept 接收已经被取消的媒体。
    assign ready_valid=(state==READY)&&!rst&&!cancel_match;
    assign abort_req=(state==ABORT)&&!rst;
    assign busy=(state!=IDLE); // READY 与 ABORT 都仍占用，不能据此提前复用帧槽。
    always @(posedge clk) begin
        if(rst) begin
            state<=IDLE;src_seen<=0;sink_seen<=0;
            active_session<=0;active_seq<=0;active_slot<=0;
            src_expected<=0;sink_expected<=0;error_reason<=0;
        end else begin
            case(state)
                IDLE: if(req_valid) begin
                    // 在接受边沿拍下描述符和期望计数，清除上一次事务的完成记录。
                    // IDLE 分支不处理完成事件，防止新请求同拍旧回包被误接收。
                    active_session<=req_session;active_seq<=req_seq;active_slot<=req_slot;
                    src_expected<=expected_src_count;sink_expected<=expected_sink_count;
                    src_seen<=0;sink_seen<=0;error_reason<=0;state<=ACTIVE;
                end
                ACTIVE: begin
                    // 同拍事件优先级：取消 > 源端失败 > 写入端失败 > 正常完成。
                    if(cancel_match) begin state<=ABORT;error_reason<=3;end
                    else if(src_bad) begin state<=ABORT;error_reason<=1;end
                    else if(sink_bad) begin state<=ABORT;error_reason<=2;end
                    else begin
                        if(src_good) src_seen<=1;
                        if(sink_good) sink_seen<=1;
                        // 非阻塞赋值本拍尚未更新 seen，故同时合并当前 good 事件，
                        // 保证双端同拍完成也能立即进入 READY，不再多等一个事件。
                        if((src_seen||src_good)&&(sink_seen||sink_good)) state<=READY;
                    end
                end
                READY: begin
                    // 等待消费期间保持 active_*；取消优先于消费，并进入排空流程。
                    if(cancel_match) begin state<=ABORT;error_reason<=3;end
                    else if(ready_accept) state<=IDLE;
                end
                ABORT: if(abort_ack&&(ack_session==active_session)&&(ack_seq==active_seq)) state<=IDLE;
                default: state<=ABORT; // 非法状态优先保留占用，避免未知事务继续写入已复用槽。
            endcase
        end
    end
endmodule
