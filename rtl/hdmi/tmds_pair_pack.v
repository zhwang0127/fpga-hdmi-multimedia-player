`timescale 1ns/1ps
`default_nettype none
// pix(74.25MHz) → word(37.125MHz) 符号配对（规格 §10.3「20bit 配对」）。
// 两时钟来自同一 PLL2（word=pix/2，同相 0°），是相关生成时钟，按规格 §7 由 STA 分析 setup/hold，
// 不是异步 CDC，不加同步器/false_path。pix 域每 2 拍把 {后符号,先符号} 组成 20bit/lane；
// word 域每拍采样一次已稳定 2 个 pix 周期的配对寄存器：无论 word 沿对齐 pix 的哪一相，
// 每个配对恰被采一次（速率严格 1:1），只影响固定延迟，不丢不重。
// lane 顺序：[0]=D0(B) [1]=D1(G) [2]=D2(R) [3]=时钟 lane（固定 10'b0000011111，LSB 先发 → 5 个 1 后 5 个 0）。
module tmds_pair_pack(
    input  wire clk_pix,rst_pix,
    input  wire [9:0] s0,s1,s2,
    input  wire clk_word,rst_word,
    output reg  [79:0] word,          // {lane3,lane2,lane1,lane0}，每 lane 20bit，[9:0] 为先发符号
    output reg  word_tog              // 每个新 word 翻转一次，供 gear 域对齐装载
);
    localparam [9:0] CLK_SYM=10'b0000011111;
    reg ph; reg [29:0] first; reg [79:0] pair;
    always @(posedge clk_pix) begin
        if(rst_pix) begin ph<=1'b0; first<=30'd0; pair<=80'd0; end
        else begin
            ph<=~ph;
            if(!ph) first<={s2,s1,s0};
            else pair<={CLK_SYM,CLK_SYM, s2,first[29:20], s1,first[19:10], s0,first[9:0]};
        end
    end
    // 在已有pix下降沿保持配对数据，word上升沿从半周期前稳定的寄存器采样。
    // pix/word 的两种对齐相位均读取该沿之前稳定的 pair，保持符号字序。
    // 数据级不复位：pair在pix复位期间为0，word本域复位保持足够长，释放前本级已取到0。
    // 这样不把下降沿释放的rst_pix又接到同沿数据级的同步复位入口，避免新增同沿短路径。
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg [79:0] pair_guard;
    always @(negedge clk_pix) pair_guard<=pair;
    always @(posedge clk_word) begin
        if(rst_word) begin word<=80'd0; word_tog<=1'b0; end
        else begin word<=pair_guard; word_tog<=~word_tog; end
    end
endmodule
`default_nettype wire
