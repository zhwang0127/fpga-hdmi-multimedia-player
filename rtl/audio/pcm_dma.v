`timescale 1ns/1ps
`default_nettype none
// mem 域 PCM 预取 DMA，作为 sdram_shared_rw 的读客户端向 mem→pix FIFO 填充样本。
// SDRAM 每字为一个双声道样本 {R16,L16}，按 BL8 读取，同一时刻最多一个在途段。
// 发请求前预留 8 个样本的 FIFO 空间；已缓存样本数低于 LOW 时给出 urgent，优先补充音频。
// nwords 不含音频缓冲区尾部的 BL8 补零，读到超出有效长度的数据时丢弃，不写入 FIFO。
// 取消时保持已挂出的请求直到接受，等待在途段完成并丢弃其数据，再执行 FIFO flush。
// 只有 flush_done 后才发布 cancel_done；即使已经 fetch_done，取消仍须清除未播放的 FIFO 数据。
// 运行时若 FIFO 拒绝返回数据，锁存 overflow，由上层按音频故障处理。
module pcm_dma #(
    parameter FIFO_AW=11,                 // FIFO 深 2**FIFO_AW
    parameter [FIFO_AW:0] LOW=512, HIGH=1536
)(
    input  wire        clk, rst,          // mem 域；rst 为本域业务同步复位
    input  wire        start,             // 开始一段配音（IDLE 时有效）
    input  wire [20:0] base_addr,         // bank 首字地址（BL8 对齐）
    input  wire [17:0] nwords,            // 样本数 48,000—240,000
    input  wire        cancel,            // 取消当前段（任意状态，IDLE 忽略）
    // SDRAM 读段客户端（每条 1 个 BL8）
    output reg         req_valid,
    input  wire        req_ready,
    output reg  [20:0] req_addr,
    output wire [3:0]  req_count,
    input  wire        rd_valid,          // 读数据拍
    input  wire [31:0] rd_data,
    input  wire        req_done,          // 本段结束（数据拍全部给出后）
    // mem→pix 异步 FIFO 写端
    output reg         fifo_wr_valid,
    output reg  [31:0] fifo_wr_data,
    input  wire        fifo_wr_ready,
    input  wire [FIFO_AW:0] fifo_wr_level, // 写端看到的 FIFO 已缓存样本数，每项含左右声道。
    output reg         fifo_flush_req,
    input  wire        fifo_flush_done,
    // 状态
    output wire        urgent,
    output reg         fetch_done,        // 全部样本已写入 FIFO（粘滞到下一次 start/cancel）
    output reg         cancel_done,       // 一拍脉冲：取消完成（FIFO 已清空）
    output reg         overflow,          // 粘滞错误
    output wire        busy
);
    localparam [2:0] S_IDLE=3'd0, S_RUN=3'd1, S_WAIT=3'd2, S_FLUSH=3'd3, S_FLW=3'd4;
    reg [2:0]  st;
    reg [17:0] req_words;     // 已请求的字数（按 8 递增）
    reg [17:0] got_words;     // 已收到的数据拍数（含尾部补零拍）
    reg [17:0] total;
    reg [20:0] next_addr;
    reg        cancel_pend;   // 已收到取消；待已发请求均已接受且返回数据全部处理完后，再清 FIFO。
    assign req_count = 4'd1;
    assign busy = (st != S_IDLE);
    // 提前一拍判断 FIFO 已缓存样本数是否低于 LOW（默认 512 个）。
    // 样本不足且 DMA 正在运行、未取消时，urgent 提醒共享控制器优先补充音频。
    // 先寄存数量比较结果，缩短 FIFO 计数到仲裁选择之间的组合路径。
    reg level_low_q;
    always @(posedge clk) level_low_q <= (fifo_wr_level < LOW);
    assign urgent = (st == S_RUN || st == S_WAIT) && !cancel_pend && level_low_q;
    // 提前一拍判断 FIFO 是否还有空间接收下一段的 8 个样本。
    // 使用 HIGH-9（HIGH 默认 1536）：预留 8 个返回样本，再为判断延迟多留 1 个样本空间。
    // 这样实际发请求时仍满足“已缓存样本数+8<=HIGH”；播放端取走样本只会增加空闲空间。
    reg level_room_q;
    always @(posedge clk) level_room_q <= fifo_wr_level <= HIGH-12'd9;
    wire want = (req_words < total) && level_room_q;
    // 地址、计数、长度及数据在独立时序块更新，避免被工具并入控制状态的 one-hot 编码。
    // 数据更新条件互斥，状态机只负责请求、等待和取消排空的控制。
    always @(posedge clk) begin
        if (rst) begin req_words<=18'd0;got_words<=18'd0;total<=18'd0;next_addr<=21'd0;req_addr<=21'd0;fifo_wr_data<=32'd0;end
        else begin
            if (rd_valid && st == S_WAIT) begin
                got_words <= got_words + 18'd1;
                if (got_words < total && !cancel_pend && !cancel) fifo_wr_data <= rd_data;
            end
            if (st == S_IDLE && !(cancel && fetch_done) && start) begin
                total <= nwords; next_addr <= base_addr; req_words <= 18'd0; got_words <= 18'd0;
            end
            if (st == S_RUN && req_valid && req_ready) begin
                next_addr <= next_addr + 21'd8; req_words <= req_words + 18'd8;
            end
            if (st == S_RUN && !req_valid && !(cancel_pend || cancel) && !(req_words >= total) && want)
                req_addr <= next_addr;
        end
    end
    // flush_req/fetch_done 等标志在独立时序块中按事件置位和清零，避免并入状态编码。
    // 同拍存在多个事件时，flush 完成和 start 的清除操作优先于外部 cancel。
    wire idle_cancel=st==S_IDLE && cancel && fetch_done;
    wire start_fire=st==S_IDLE && !idle_cancel && start;
    wire flush_finish=st==S_FLW && fifo_flush_done;
    always @(posedge clk) begin
        if(rst) begin req_valid<=0;fifo_flush_req<=0;fetch_done<=0;cancel_pend<=0;end
        else begin
            if(cancel && st!=S_IDLE) cancel_pend<=1;
            if(idle_cancel) begin cancel_pend<=1;fetch_done<=0;end
            if(start_fire) begin fetch_done<=0;cancel_pend<=0;end
            if(st==S_RUN) begin
                if(req_valid) begin if(req_ready) req_valid<=0;end
                else if(!(cancel_pend || cancel)) begin
                    if(req_words>=total) fetch_done<=1;
                    else if(want) req_valid<=1;
                end
            end
            if(st==S_FLUSH) fifo_flush_req<=1;
            if(flush_finish) begin fifo_flush_req<=0;cancel_pend<=0;fetch_done<=0;end
        end
    end
    always @(posedge clk) begin
        cancel_done <= 1'b0; fifo_wr_valid <= 1'b0;
        if (rst) begin
            st <= S_IDLE; overflow <= 1'b0;
        end else begin
            // 写 FIFO 是寄存输出：写拍 FIFO 不收即记 overflow（上述空间预留规则用于避免此情况）
            if (fifo_wr_valid && !fifo_wr_ready) overflow <= 1'b1;
            // 数据拍：未取消且在播放长度内才写 FIFO（超出长度的是 bank 尾部 BL8 补零）
            if (rd_valid && st == S_WAIT) begin
                if (got_words < total && !cancel_pend && !cancel) begin fifo_wr_valid <= 1'b1; end
            end
            case (st)
                S_IDLE: if (cancel && fetch_done) begin
                    // 所有SDRAM请求完成不等于FIFO空：尾段可能仍有最多HIGH个样本。
                    // 切图或取消这时也要有真实flush_done，不能直接复用音频缓冲区/FIFO。
                    st <= S_FLUSH;
                end else if (start) begin
                    overflow <= 1'b0; st <= S_RUN;
                end
                S_RUN: if (req_valid) begin
                    // 请求已挂出：按握手规则保持 valid 直到 ready（取消也不撤回）
                    if (req_ready) begin
                        st <= S_WAIT;
                    end
                end else if (cancel_pend || cancel) begin
                    st <= S_FLUSH;                                  // 无在途段：直接清 FIFO
                end else if (req_words >= total) begin
                    st <= S_IDLE;
                end
                S_WAIT: if (req_done) st <= (cancel_pend || cancel) ? S_FLUSH : S_RUN;   // 取消时在途数据已在上面丢弃
                S_FLUSH: begin st <= S_FLW; end
                S_FLW: if (fifo_flush_done) begin
                    cancel_done <= 1'b1; st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
