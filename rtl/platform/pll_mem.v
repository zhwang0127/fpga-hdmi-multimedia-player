`timescale 1ns/1ps
`default_nettype none
// 存储时钟：50MHz 参考输入，生成控制器和合封 SDRAM 所需的两路 100MHz 时钟。
// EG_PHY_PLL 使用 C0 经 BUFG 外部反馈：反馈 100MHz、VCO 1000MHz，C1/C2 各十分频。
// 模块默认 device 与 mem 同相；整机传入 DEVICE_CPHASE=6、DEVICE_FPHASE=3，
// 使 device 相对 mem 延后 7.375ns（粗相位 7ns，细相位 3×0.125ns）。
// 该相位与寄存 PHY、SDC 的输入输出窗口配套，改变频率时须一起重新计算。
// locked 直接取自 PLL 原语；顶层 pll_lock_guard 在连续锁定合格后释放业务复位。
module pll_mem #(parameter DEVICE_CPHASE=9,parameter DEVICE_FPHASE=0)(
    input wire clk_ref,reset,
    output wire clk_mem,clk_device,locked
);
    wire feedback_raw,feedback;
    wire [1:0] unused_clocks;
    EG_LOGIC_BUFG u_feedback(.i(feedback_raw),.o(feedback));
    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),.DYNCFG("DISABLE"),.FIN("50.000000"),
        .FEEDBK_MODE("NORMAL"),.FEEDBK_PATH("CLKC0_EXT"),
        .STDBY_ENABLE("DISABLE"),.PLLRST_ENA("ENABLE"),.SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(4),.ICP_CURRENT(29),.KVCO(4),.LPF_CAPACITOR(1),.LPF_RESISTOR(2),
        .REFCLK_DIV(1),.FBCLK_DIV(2),
        .CLKC0_ENABLE("ENABLE"),.CLKC0_DIV(10),.CLKC0_CPHASE(9),.CLKC0_FPHASE(0),
        .CLKC1_ENABLE("ENABLE"),.CLKC1_DIV(10),.CLKC1_CPHASE(9),.CLKC1_FPHASE(0),
        .CLKC2_ENABLE("ENABLE"),.CLKC2_DIV(10),.CLKC2_CPHASE(DEVICE_CPHASE),.CLKC2_FPHASE(DEVICE_FPHASE)
    ) u_pll(
        .refclk(clk_ref),.reset(reset),.stdby(1'b0),.extlock(locked),
        .load_reg(1'b0),.psclk(1'b0),.psdown(1'b0),.psstep(1'b0),.psclksel(3'b000),
        .psdone(),.dclk(1'b0),.dcs(1'b0),.dwe(1'b0),.di(8'b0),.daddr(6'b0),.do(),
        .fbclk(feedback),.clkc({unused_clocks,clk_device,clk_mem,feedback_raw})
    );
endmodule
`default_nettype wire
