`timescale 1ns/1ps
`default_nettype none
// 视频两级 PLL：50MHz 经第一级生成 27MHz，再由第二级生成 HDMI 时钟。
// PLL1 的 VCO 为 1080MHz，C0 四十分频得到 27MHz。
// PLL2 的 VCO 为 742.5MHz，输出 pix=74.25MHz、serial=371.25MHz、gear=185.625MHz、word=37.125MHz。
// serial 相位为 135°，其余输出同相；相位码与 tmds_gearbox20_phy 的采样边沿配套。
// 两级均使用 NOCOMP/VCO_PHASE_0 内部反馈，fbclk 接零；环路参数采用 TD 器件向导值。
// 第二级在第一级未锁定时保持复位，顶层再统一检查全部时钟的锁定确认。
module pll_video(
    input wire clk_ref,reset,
    output wire clk_video_ref,clk_pix,clk_serial,clk_gear,clk_word,locked_ref,locked_pix
);
    wire [3:0] unused_a; wire unused_b;
    // 像素时钟经全局 BUFG 分发，减少高扇出局部时钟布线的偏斜。
    wire clk_pix_raw;
    EG_LOGIC_BUFG u_pix_bufg(.i(clk_pix_raw),.o(clk_pix));
    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),.DYNCFG("DISABLE"),.FIN("50.000000"),
        .FEEDBK_MODE("NOCOMP"),.FEEDBK_PATH("VCO_PHASE_0"),
        .STDBY_ENABLE("DISABLE"),.PLLRST_ENA("ENABLE"),.SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(0),.ICP_CURRENT(9),.KVCO(2),.LPF_CAPACITOR(2),.LPF_RESISTOR(8),
        // 环路参数采用 TD 的 EG4S20BG256、Medium 带宽配置。
        .REFCLK_DIV(5),.FBCLK_DIV(108),
        .CLKC0_ENABLE("ENABLE"),.CLKC0_DIV(40),.CLKC0_CPHASE(39),.CLKC0_FPHASE(0)
    ) u_pll_ref(
        .refclk(clk_ref),.reset(reset),.stdby(1'b0),.extlock(locked_ref),
        .load_reg(1'b0),.psclk(1'b0),.psdown(1'b0),.psstep(1'b0),.psclksel(3'b000),
        .psdone(),.dclk(1'b0),.dcs(1'b0),.dwe(1'b0),.di(8'b0),.daddr(6'b0),.do(),
        .fbclk(1'b0),.clkc({unused_a,clk_video_ref})
    );
    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),.DYNCFG("DISABLE"),.FIN("27.000000"),
        .FEEDBK_MODE("NOCOMP"),.FEEDBK_PATH("VCO_PHASE_0"),
        .STDBY_ENABLE("DISABLE"),.PLLRST_ENA("ENABLE"),.SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(0),.ICP_CURRENT(9),.KVCO(2),.LPF_CAPACITOR(2),.LPF_RESISTOR(8),
        // 环路参数采用 TD 的 EG4S20BG256、Medium 带宽配置。
        .REFCLK_DIV(2),.FBCLK_DIV(55),
        .CLKC0_ENABLE("ENABLE"),.CLKC0_DIV(10),.CLKC0_CPHASE(9),.CLKC0_FPHASE(0),
        .CLKC1_ENABLE("ENABLE"),.CLKC1_DIV(2),.CLKC1_CPHASE(1),.CLKC1_FPHASE(6),
        .CLKC2_ENABLE("ENABLE"),.CLKC2_DIV(4),.CLKC2_CPHASE(3),.CLKC2_FPHASE(0),
        .CLKC3_ENABLE("ENABLE"),.CLKC3_DIV(20),.CLKC3_CPHASE(19),.CLKC3_FPHASE(0)
    ) u_pll_pix(
        .refclk(clk_video_ref),.reset(reset | !locked_ref),.stdby(1'b0),.extlock(locked_pix),
        .load_reg(1'b0),.psclk(1'b0),.psdown(1'b0),.psstep(1'b0),.psclksel(3'b000),
        .psdone(),.dclk(1'b0),.dcs(1'b0),.dwe(1'b0),.di(8'b0),.daddr(6'b0),.do(),
        .fbclk(1'b0),.clkc({unused_b,clk_word,clk_gear,clk_serial,clk_pix_raw})
    );
endmodule
`default_nettype wire
