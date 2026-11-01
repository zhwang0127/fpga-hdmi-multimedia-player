`timescale 1ns/1ps
`default_nettype none
// pix 域固定三角波音源，供无有效 WAV 时的 HDMI 音频回退使用。
// 左右声道频率分别为 1kHz 和 660Hz，幅度约为满幅的八分之一。
// ce 与 accept 同时有效时输出 valid 并推进相位；其余拍相位保持。
// mute 输出静音；采样时钟使能由 audio_ce_48k 产生。
module audio_test_tone #(
    parameter [31:0] STEP_L=32'd89478485,
    parameter [31:0] STEP_R=32'd59055800
)(
    input  wire clk,rst,
    input  wire ce,accept,mute,
    output wire valid,
    output wire [15:0] l,r
);
    reg [31:0] ph_l,ph_r;
    // 折叠：上升半周期取 [30:18]，下降半周期取反；结果 0..8191，减 4096 居中。
    function [15:0] tri16(input [13:0] p);   // 只用相位高 14 位
        reg [12:0] t;
        begin t=p[13] ? ~p[12:0] : p[12:0]; tri16={3'b000,t}-16'd4096; end
    endfunction
    assign valid=ce && accept;
    assign l=mute ? 16'd0 : tri16(ph_l[31:18]);
    assign r=mute ? 16'd0 : tri16(ph_r[31:18]);
    always @(posedge clk) begin
        if(rst) begin ph_l<=32'd0; ph_r<=32'd0; end
        else if(ce && accept) begin ph_l<=ph_l+STEP_L; ph_r<=ph_r+STEP_R; end
    end
endmodule
`default_nettype wire
