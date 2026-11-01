`timescale 1ns/1ps
// 功能：在 74.25 MHz 像素时钟域产生平均 48 kHz 的单周期采样使能。
// 接口：rst 为同步高有效复位；消费方在 sample_ce 为高的上升沿接收样本。
// 使用约束：sample_ce 是使能，不是时钟；暂停播放时仍保持本模块运行。
// 74.25 MHz / 48 kHz = 12375 / 8. No derived fabric clock.
module audio_ce_48k(input wire clk, input wire rst, output reg sample_ce);
    // phase 保持在 0..12374；加法扩展一位，明确中间结果的比较位宽。
    reg [13:0] phase;
    wire [14:0] advanced = {1'b0, phase} + 15'd8;
    always @(posedge clk) begin
        if (rst) begin
            // 重建采样相位；消费链路应采用同域复位，避免恢复时样本错位。
            phase <= 14'd0;
            sample_ce <= 1'b0;
        end else if (advanced >= 15'd12375) begin
            // 模累加器越界时输出一次使能；保留余数，避免整数分频的累计频差。
            // 完整 12375 周期产生 8 次使能，间隔为 1546 或 1547 周期。
            phase <= advanced - 15'd12375;
            sample_ce <= 1'b1;
        end else begin
            // 没有越界时仅推进相位，输出使能拉低，脉冲不连续保持。
            phase <= advanced[13:0];
            sample_ce <= 1'b0;
        end
    end
endmodule
