`timescale 1ns/1ps
`default_nettype none
// HDMI 数据岛包串行器（BCH ECC + 32 周期交织，规格 §10.5）。

// - 一个包 = 24bit 包头 + 4 个 56bit 子包。包头每拍出 1 位（共 24 拍），再出 8 位 ECC（8 拍）；
//   每个子包每拍出 2 位（共 28 拍），再出 8 位 ECC（4 拍）。合计正好 32 个像素周期。
// - ECC 按规格给的递推：ecc_next=(ecc>>1) XOR ((ecc[0] XOR bit) ? 8'h83 : 0)，逐 LSB 输入、初值 0。
//   不预先算整包 ECC（那需要很宽的组合 XOR 树），而是边发边算：数据位从移位寄存器低位移出，
//   最后一个数据位发出的那一拍把算好的 ECC 填进移位寄存器低 8 位，接着从低位依次发出。
// - 输出为本拍组合值（直接来自寄存器），由上层在同一拍寄存进 TERC4 前一级。
//   load 在“第 0 位要出现的前一拍”拉高：load 那个沿装入数据，下一拍起 k=0..31 依次输出。
//   连续两个包时，前一包 k=31 那拍可以同时 load 下一包（load 优先），中间没有空拍。
// - 位到 lane 的映射（由上层使用）：包头位→lane0 D[2]；子包 j 的偶数位（2k）→lane1 D[j]，
//   奇数位（2k+1）→lane2 D[j]。每个像素周期将对应的包位送入两条数据 lane。
// 复位：rst 同步高有效，只清 active/计数；数据寄存器不需要复位（active=0 时上层不使用输出）。
module hdmi_packet_serializer(
    input  wire clk,rst,
    input  wire load,                 // 本拍装入新包，下一拍输出第 0 位
    input  wire [23:0] header,        // {HB2,HB1,HB0}，HB0 在 [7:0]，最先发 HB0 的 bit0
    input  wire [223:0] sub,          // {sub3,sub2,sub1,sub0}，每个 56bit，SB0 在各自 [7:0]
    output wire active,               // 1=本拍输出有效（k=0..31 之内）
    output wire hbit,                 // 本拍包头/包头 ECC 位
    output wire [3:0] even_bits,      // {sub3,sub2,sub1,sub0} 的第 2k 位（或 ECC 位）
    output wire [3:0] odd_bits        // {sub3,sub2,sub1,sub0} 的第 2k+1 位（或 ECC 位）
);
    // 规格 §10.5 的 ECC 递推，作为组合函数；两个函数调用串起来就是每拍 2 位的子包 ECC。
    function [7:0] ecc_step(input [7:0] e, input bit_in);
        begin ecc_step=(e>>1)^(((e[0]^bit_in)==1'b1) ? 8'h83 : 8'h00); end
    endfunction

    reg        act;
    reg [4:0]  k;                     // 当前输出的是第 k 个周期（0..31）
    reg [31:0] hsr;                   // 包头移位寄存器：低 24 位为数据，后 8 位在第 23 拍填入 ECC
    reg [7:0]  hecc;                  // 包头 ECC 累加
    reg [63:0] ssr [0:3];             // 子包移位寄存器：低 56 位为数据，后 8 位在第 27 拍填入 ECC
    reg [7:0]  secc[0:3];             // 子包 ECC 累加
    assign active=act;
    assign hbit=hsr[0];
    assign even_bits={ssr[3][0],ssr[2][0],ssr[1][0],ssr[0][0]};
    assign odd_bits ={ssr[3][1],ssr[2][1],ssr[1][1],ssr[0][1]};

    wire [7:0] hecc_n=ecc_step(hecc,hsr[0]);
    // 每个子包本拍 2 位之后的 ECC（组合，先第 2k 位再第 2k+1 位）。
    wire [7:0] secc_n[0:3];
    genvar gj;
    generate for(gj=0;gj<4;gj=gj+1) begin : g_ecc
        assign secc_n[gj]=ecc_step(ecc_step(secc[gj],ssr[gj][0]),ssr[gj][1]);
    end endgenerate
    integer j;
    always @(posedge clk) begin
        if(rst) begin
            act<=1'b0; k<=5'd0;
        end else if(load) begin
            // 装入新包：数据放低位，高 8 位先填 0（之后由 ECC 覆盖）；ECC 累加器清零。
            act<=1'b1; k<=5'd0;
            hsr<={8'd0,header}; hecc<=8'd0;
            for(j=0;j<4;j=j+1) begin ssr[j]<={8'd0,sub[56*j +: 56]}; secc[j]<=8'd0; end
        end else if(act) begin
            k<=k+5'd1;
            if(k==5'd31) act<=1'b0;       // 32 拍发完；若同拍有 load，上一分支优先
            // 包头：前 24 拍每拍移出 1 位并累加 ECC；第 23 拍把最终 ECC 放进低 8 位，第 24..31 拍发出。
            if(k<5'd24) begin
                hecc<=hecc_n;
                if(k==5'd23) hsr<={1'b0,hsr[31:1]} & 32'hFFFFFF00 | {24'd0,hecc_n};
                else         hsr<={1'b0,hsr[31:1]};
            end else hsr<={1'b0,hsr[31:1]};
            // 子包：前 28 拍每拍移出 2 位并累加 ECC；第 27 拍填入 ECC，第 28..31 拍每拍发出 2 位。
            for(j=0;j<4;j=j+1) begin
                if(k<5'd28) begin
                    secc[j]<=secc_n[j];
                    if(k==5'd27) ssr[j]<={2'b00,ssr[j][63:2]} & 64'hFFFFFFFFFFFFFF00 | {56'd0,secc_n[j]};
                    else         ssr[j]<={2'b00,ssr[j][63:2]};
                end else ssr[j]<={2'b00,ssr[j][63:2]};
            end
        end
    end
endmodule
`default_nettype wire
