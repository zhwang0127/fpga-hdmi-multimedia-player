`timescale 1ns/1ps
`default_nettype none
// word(37.125) → gear(185.625) 20:4 齿轮箱 + 4 路 EG_LOGIC_ODDRx2（规格 §10.3）。
// gear=5×word、serial=2×gear，三者同源 PLL2（相关时钟，STA 分析）。gear 域检测 word_tog 变化
// 装载 20bit，随后 5 拍各输出 4bit；每个 word 恰好 5 拍，故装载节奏确定、无丢重。
// ODDRx2：pclk=gear，sclk=serial；按 DS300 3.9.4 图 2-56，单个 PCLK 周期内串行顺序为 D0,D1,D2,D3，
// 因此 d0 接本拍最先要发的 bit（TMDS LSB first）。四位输入按此顺序映射，保持 TMDS 符号低位先发。
// ODDRx2 rst 接 0，消除 gear→serial recovery。复位期间 nib 被 rst_gear 同步清 0，
// ODDR 每 gear 拍输出确定的 0；释放后由上游 word（复位期为控制/定义值）装载。上电前 nib 由配置初值定义。
module tmds_gearbox20_phy(
    input  wire clk_gear,rst_gear,clk_serial,
    input  wire [79:0] word,input wire word_tog,
    output wire [3:0] pad_p                   // [0]=D0 [1]=D1 [2]=D2 [3]=CLK，接差分 P 端
);
    reg t1,t2; reg [79:0] sh; reg [15:0] nib;   // nib: 每 lane 4bit
    integer i;
    always @(posedge clk_gear) begin
        if(rst_gear) begin t1<=1'b0; t2<=1'b0; sh<=80'd0; nib<=16'd0; end
        else begin
            t1<=word_tog; t2<=t1;
            for(i=0;i<4;i=i+1) begin
                if(t1!=t2) begin
                    nib[i*4+:4]<=word[i*20+:4];
                    sh[i*20+:20]<={4'd0,word[i*20+4+:16]};
                end else begin
                    nib[i*4+:4]<=sh[i*20+:4];
                    sh[i*20+:20]<={4'd0,sh[i*20+4+:16]};
                end
            end
        end
    end
    genvar l;
    generate for(l=0;l<4;l=l+1) begin: lane
        EG_LOGIC_ODDRx2 u_oddr(.q(pad_p[l]),.pclk(clk_gear),.sclk(clk_serial),
            .d0(nib[l*4+0]),.d1(nib[l*4+1]),.d2(nib[l*4+2]),.d3(nib[l*4+3]),.rst(1'b0));
    end endgenerate
endmodule
`default_nettype wire
