`timescale 1ns/1ps
`default_nettype none
// TMDS 8b/10b 视频/控制编码（DVI 1.0 §3.2.3 算法，规格 §10.3）。自主编写。
// 两步：按输入 1 的个数选 XOR/XNOR 得 q_m[8:0]；再按运行差异 cnt 决定是否反相并输出 10bit。
// 控制期（de=0）输出四个标准控制符号之一并把 cnt 清 0（DVI 1.0 规定）。
// 时延：1 拍（输入组合 → 输出寄存）。rst 同步高有效：输出控制符号 c=00、cnt=0。
// TERC4数据岛与保护带由上级HDMI包层负责；这里保持一拍的视频/控制输出。
module tmds_encoder(
    input  wire clk,rst,
    input  wire de,
    input  wire [1:0] c,          // {c1,c0}；lane0 为 {VS,HS}
    input  wire [7:0] d,
    output reg  [9:0] q            // q[0] 先发（LSB first）
);
    // 平衡计数树：四组两位计数、两组四位计数，最后合并；不串八次加法。
    function [3:0] ones8(input [7:0] v);
        reg [1:0] p0,p1,p2,p3;reg [2:0] h0,h1;
        begin
            p0={1'b0,v[0]}+{1'b0,v[1]};p1={1'b0,v[2]}+{1'b0,v[3]};
            p2={1'b0,v[4]}+{1'b0,v[5]};p3={1'b0,v[6]}+{1'b0,v[7]};
            h0={1'b0,p0}+{1'b0,p1};h1={1'b0,p2}+{1'b0,p3};
            ones8={1'b0,h0}+{1'b0,h1};
        end
    endfunction
    wire [3:0] n1d=ones8(d);
    wire use_xnor=(n1d>4'd4)||(n1d==4'd4 && d[0]==1'b0);
    // XNOR递推每遇奇数下标翻转一次；直接用前缀奇偶校验，保持同一输入拍。
    wire [8:0] qm;
    genvar k;
    generate for(k=0;k<8;k=k+1) begin:g_prefix
        assign qm[k]=(^d[k:0]) ^ ((k%2)!=0 && use_xnor);
    end endgenerate
    assign qm[8]=~use_xnor;
    wire [3:0] n1q=ones8(qm[7:0]);
    // 差异以2为步长变化；只更新高5位，低位原样保持，按 6 位模运算更新偏差。
    reg signed [5:0] cnt;
    wire normal_case=(cnt==0)||(n1q==4);
    wire same_direction=(cnt[5]==(n1q<4));
    wire invert_data=normal_case ? !qm[8] : same_direction;
    wire signed [4:0] diff_half=$signed({1'b0,n1q})-5'sd4;
    wire signed [4:0] signed_half=invert_data ? -diff_half : diff_half;
    wire signed [4:0] correction=normal_case ? 5'sd0 : invert_data ?
        (qm[8] ? 5'sd1 : 5'sd0) : (qm[8] ? 5'sd0 : -5'sd1);
    wire signed [4:0] next_high=$signed(cnt[5:1])+signed_half+correction;
    always @(posedge clk) begin
        if(rst) begin q<=10'b1101010100; cnt<=6'sd0; end
        else if(!de) begin
            cnt<=6'sd0;
            case(c)
                2'b00: q<=10'b1101010100;
                2'b01: q<=10'b0010101011;
                2'b10: q<=10'b0101010100;
                default: q<=10'b1010101011;
            endcase
        end else if(normal_case) begin
            q<={~qm[8],qm[8],(qm[8]?qm[7:0]:~qm[7:0])};
            cnt<={next_high,cnt[0]};
        end else if(same_direction) begin
            q<={1'b1,qm[8],~qm[7:0]};
            cnt<={next_high,cnt[0]};
        end else begin
            q<={1'b0,qm[8],qm[7:0]};
            cnt<={next_high,cnt[0]};
        end
    end
endmodule
`default_nettype wire
