`timescale 1ns/1ps
`default_nettype none
// 720p 彩条图样，由顶层 VIDEO_TEST_PATTERN 选择启用。
// 8 条竖彩条各宽 160 像素，依次为白、黄、青、绿、品红、红、蓝、黑。
// RGB 与 video_timing_720p 的 x/de 同拍，消隐期输出零。
module colorbar_720p(input wire [10:0] x,input wire de,output wire [7:0] r,g,b);
    wire [10:0] q160=x/11'd160;
    wire [2:0] bar=(x>=11'd1280) ? 3'd7 : q160[2:0];
    // 位 2/1/0 = R/G/B 是否点亮，顺序为标准 SMPTE 风格 8 条
    reg [2:0] rgb;
    always @* case(bar)
        3'd0: rgb=3'b111; 3'd1: rgb=3'b110; 3'd2: rgb=3'b011; 3'd3: rgb=3'b010;
        3'd4: rgb=3'b101; 3'd5: rgb=3'b100; 3'd6: rgb=3'b001; default: rgb=3'b000;
    endcase
    assign r={8{de&rgb[2]}}; assign g={8{de&rgb[1]}}; assign b={8{de&rgb[0]}};
endmodule
`default_nettype wire
