`timescale 1ns/1ps
`default_nettype none
// HDMI 最终 PCM 音量，所有端口在 pix 域；频谱仍从本级之前取样。
// level=0..16，超范围按16；目标增益=level*16（Q8，256为原样）。每个 in_valid
// 让 gain_cur 向目标移动1，满量程256样本约5.33ms；复位即256，不加上电淡入。
// 一只18x18 DSP分时处理L/R：t0接valid并更新增益，t1锁L，t2锁R/产L，
// t3舍入L/产R，t4舍入R并发out_valid。延迟固定4拍，valid只持续1拍。
// 输入契约：两次in_valid至少间隔5拍，in_l/r从t0保持至t2采样沿之后。
// 真实源间隔至少1546拍且寄存样本保持到下一个CE，满足乘法流水的输入保持要求。
// 不另存32位输入副本；不反压HDMI，不在默认档增加零延迟旁路。
// round_sym：正数(p+128)>>>8，负数(p+127)>>>8，半值远离0。
// |gain|<=256，所以结果不超过signed16，不需要饱和器。
// 数据乘法流水不复位，由state/out_valid隔离；所有控制和可见输出同步复位。
module audio_volume #(parameter USE_EG_DSP=1)(
    input wire clk,rst,
    input wire [4:0] level,
    input wire in_valid,
    input wire [15:0] in_l,in_r,
    output reg out_valid,
    output reg [15:0] out_l,out_r
);
    wire [4:0] safe_level=level>5'd16 ? 5'd16 : level;
    wire [8:0] target={safe_level,4'd0};
    reg [8:0] gain_cur;
    reg [2:0] state;
    wire [15:0] channel=state==3'd2 ? in_r : in_l;
    wire signed [35:0] product_full;
    spectrum_mul18 #(.USE_EG_DSP(USE_EG_DSP)) u_mul(.clk(clk),
        .a({{2{channel[15]}},channel}),.b({9'd0,gain_cur}),.p(product_full));
    // signed16*0..256 恰在signed25范围，先截到有效位宽再做一次公共舍入加法。
    wire signed [24:0] product=product_full[24:0];
    // 只递增保留的16位；低8位显式求舍入进位，避免25位偏置加法。
    // 负数恰好位于半值时不向上进位，实现半值远离零的舍入规则。
    wire round_up=product[7] && ((|product[6:0]) || !product[24]);
    wire [15:0] scaled=product[23:8]+{15'd0,round_up};
    // 增益上下步进共用一个9位加法器；9'h1ff即模512的-1，用于减一档。
    wire [8:0] gain_step=gain_cur<target ? 9'd1 : 9'h1ff;
    wire [8:0] gain_next=gain_cur+gain_step;
    always @(posedge clk) begin
        if(rst) begin gain_cur<=9'd256;state<=3'd0;out_valid<=1'b0;out_l<=16'd0;out_r<=16'd0;end
        else begin
            out_valid<=1'b0;
            case(state)
                3'd0: if(in_valid) begin
                    if(gain_cur!=target) gain_cur<=gain_next;
                    state<=3'd1;
                end
                3'd1: state<=3'd2;
                3'd2: state<=3'd3;
                3'd3: begin out_l<=scaled;state<=3'd4;end
                3'd4: begin out_r<=scaled;out_valid<=1'b1;state<=3'd0;end
                default: state<=3'd0;
            endcase
        end
    end
endmodule
`default_nettype wire
