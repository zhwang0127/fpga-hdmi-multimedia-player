`timescale 1ns/1ps
`default_nettype none
// TERC4 4bit→10bit 编码（HDMI 数据岛期与数据岛保护带使用，规格 §10.3）。
// 数据岛里每个 lane 每个像素周期只传 4bit，用这张固定 16 项表映射成 10bit 符号；
// 它和视频期的 TMDS 8b/10b 编码是两回事，不能把音频包字节送进视频编码器（规格 §10.3 原文要求）。
// 编码常量参考 hdl-util/hdmi 的 tmds_channel.sv。
// 纯组合，无时钟和复位；q[0] 最先发送，与 TMDS 视频符号使用相同的 LSB-first 顺序。
module hdmi_terc4(
    input  wire [3:0] d,
    output reg  [9:0] q
);
    always @* begin
        case(d)
            4'b0000: q=10'b1010011100;
            4'b0001: q=10'b1001100011;
            4'b0010: q=10'b1011100100;
            4'b0011: q=10'b1011100010;
            4'b0100: q=10'b0101110001;
            4'b0101: q=10'b0100011110;
            4'b0110: q=10'b0110001110;
            4'b0111: q=10'b0100111100;
            4'b1000: q=10'b1011001100;
            4'b1001: q=10'b0100111001;
            4'b1010: q=10'b0110011100;
            4'b1011: q=10'b1011000110;
            4'b1100: q=10'b1010001110;
            4'b1101: q=10'b1001110001;
            4'b1110: q=10'b0101100011;
            default: q=10'b1011000011;   // 4'b1111
        endcase
    end
endmodule
`default_nettype wire
