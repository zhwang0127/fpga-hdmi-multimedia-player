`timescale 1ns/1ps
`default_nettype none
// pix 域 PCM 样本输出，每次 sample_ce 产生一个采样时刻，输出晚一拍。
// play 有效且未播完时从 FIFO 取一个双声道字：L=低 16 位，R=高 16 位。
// 采样时 FIFO 无数据则输出零并发出 underrun；正常 EOF 后持续输出零，不重复播放。
// start 锁存新配音长度并清除播放计数；停止播放时不消费 FIFO。
module pcm_sample_out (
    input  wire        clk, rst,          // pix 域；rst 为本域业务同步复位
    input  wire        start,
    input  wire [17:0] n_play,
    input  wire        play,
    input  wire        sample_ce,
    input  wire        fifo_rd_valid,
    input  wire [31:0] fifo_rd_data,
    output wire        fifo_rd_ready,
    output reg         out_ce,            // 与 sample_ce 晚 1 拍的样本时刻
    output reg  [15:0] out_l, out_r,
    output reg         out_valid,         // 1=来自配音的样本；0=零样本
    output reg         underrun,          // 一拍脉冲
    output reg         eof,               // 已播完（粘滞到下一次 start）
    output reg  [17:0] played
);
    reg [17:0] n;
    wire active = play && !eof && (played < n);
    // 只在样本时刻弹出；start 同拍不弹出（start 分支不消费数据，否则会静默丢掉一个字）
    assign fifo_rd_ready = sample_ce && active && !start;
    always @(posedge clk) begin
        out_ce <= 1'b0; underrun <= 1'b0;
        if (rst) begin
            out_l <= 16'd0; out_r <= 16'd0; out_valid <= 1'b0; eof <= 1'b0; played <= 18'd0; n <= 18'd0;
        end else if (start) begin
            n <= n_play; played <= 18'd0; eof <= 1'b0; out_valid <= 1'b0; out_l <= 16'd0; out_r <= 16'd0;
        end else if (sample_ce) begin
            out_ce <= 1'b1;
            if (active && fifo_rd_valid) begin
                out_l <= fifo_rd_data[15:0]; out_r <= fifo_rd_data[31:16]; out_valid <= 1'b1;
                played <= played + 18'd1;
                if (played + 18'd1 == n) eof <= 1'b1;
            end else begin
                out_l <= 16'd0; out_r <= 16'd0; out_valid <= 1'b0;
                if (active) underrun <= 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
