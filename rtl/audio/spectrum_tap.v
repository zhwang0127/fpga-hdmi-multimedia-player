`timescale 1ns/1ps
`default_nettype none
// pix 域 PCM 旁路采样，供频谱环形 RAM 使用，不对音频播放施加反压。
// 左右声道以 17 位相加，再取高 16 位得到 (L+R)>>>1，避免满幅加法溢出。
// 数据和 valid 经过两拍流水；这一延迟与 spectrum_top 的启动快照时序配套。
// wr_en 表示实际 RAM 写沿的条件，wptr 是该沿之前的下一个空位。
// 同拍写入与启动快照由上层协调，使 FFT 窗口包含刚提交的样本。
// 连续 valid 每拍均可接受；整机输入通常为 48kHz 采样使能。
module spectrum_tap(
    input wire clk,rst,in_valid,
    input wire signed [15:0] in_l,in_r,
    output wire wr_en,
    output wire [8:0] wr_addr,
    output wire [15:0] wr_data,
    output reg [8:0] wptr,
    output reg [9:0] sample_count
);
    reg valid_q,mono_valid;
    // half_q 保存左右声道的平均值，mono_q 再延迟一拍供采样环写入。
    reg signed [15:0] half_q,mono_q;
    wire signed [16:0] stereo_sum;
    assign stereo_sum={in_l[15],in_l}+{in_r[15],in_r};
    assign wr_en=mono_valid && !rst;
    assign wr_addr=wptr;
    assign wr_data=mono_q;
    always @(posedge clk) begin
        half_q<=stereo_sum[16:1]; mono_q<=half_q;
        if (rst) begin valid_q<=0; mono_valid<=0; wptr<=0; sample_count<=0; end
        else begin
            valid_q<=in_valid; mono_valid<=valid_q;
            if (wr_en) begin
                wptr<=wptr+1'b1;
                // 满512的最高位单独粘住；下9位只做短计数，最高位不串进位后的选择。
                // 511->512这拍下9位回零、最高位置1，之后整组保持，达到 512 后计数保持饱和。
                if (!sample_count[9]) begin
                    sample_count[8:0]<=sample_count[8:0]+1'b1;
                    if(&sample_count[8:0]) sample_count[9]<=1'b1;
                end
            end
        end
    end
endmodule
`default_nettype wire
