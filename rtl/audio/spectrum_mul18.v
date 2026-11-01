`timescale 1ns/1ps
`default_nettype none
// 频谱唯一的乘法器，全部使用 pix 上升沿。输入和输出各一拍，无数据复位。
// 调度器在送入后第二个沿取得 p；不能把它当组合乘法或无延迟 IP 使用。
module spectrum_mul18 #(parameter USE_EG_DSP=1)(
    input wire clk,
    input wire signed [17:0] a,b,
    output wire signed [35:0] p
);
    generate if (USE_EG_DSP) begin : anlogic
        EG_LOGIC_MULT #(.INPUT_WIDTH_A(18),.INPUT_WIDTH_B(18),.OUTPUT_WIDTH(36),
            .INPUTFORMAT("SIGNED"),.INPUTREGA("ENABLE"),.INPUTREGB("ENABLE"),
            .OUTPUTREG("ENABLE"),.IMPLEMENT("AUTO")) u_mult(
            .clk(clk),.a(a),.b(b),.p(p),.cea(1'b1),.ceb(1'b1),.cepd(1'b1),
            .rstan(1'b1),.rstbn(1'b1),.rstpdn(1'b1));
    end else begin : portable
        reg signed [17:0] a_q,b_q;
        reg signed [35:0] p_q;
        always @(posedge clk) begin a_q<=a; b_q<=b; p_q<=a_q*b_q; end
        assign p=p_q;
    end endgenerate
endmodule
`default_nettype wire
