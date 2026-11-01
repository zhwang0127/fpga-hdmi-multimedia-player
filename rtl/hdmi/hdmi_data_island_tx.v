`timescale 1ns/1ps
`default_nettype none
// HDMI 720p60 视频和音频数据岛发送，运行在像素时钟域。
// 根据输入 HS/VS 上升沿重建行列位置；两者均已检测到后，locked 置 1。
// enable 和 locked 同时为 1 时接收音频样本，并在视频消隐期间发送前导、保护带和数据岛。
// 每行 x=1284..1423 为数据岛区域，其中 x=1294..1421 包含四个 32 拍包。
// 下一行有有效视频时，行末 x=1640..1649 发送视频前导和保护带。
// 包调度顺序：音频时钟再生包 ACR、音频样本包 ASP、AVI 信息帧、音频信息帧、空包。
// ACR 每 40 行请求一次；AVI 和音频信息帧每帧进入垂直消隐时请求一次。
// 最多缓存四组左右声道样本；发送样本包时取走已缓存样本，缓存满且本拍未发送样本包时，新样本被丢弃并输出 drop。
// IEC 60958 帧计数仅随实际发送的样本数递增，按 192 回绕。
// 输入、周期类型和符号编码依次经过三级流水；三条 lane 保持相同延迟。
// enable=0 时仅输出 TMDS 视频和控制符号；rst 为同步高有效复位。
module hdmi_data_island_tx(
    input  wire clk,rst,
    input  wire enable,
    input  wire de,hs,vs,
    input  wire [7:0] r,g,b,
    input  wire s_valid,              // 新音频样本有效，需在 s_accept=1 时提供。
    input  wire [15:0] s_l,s_r,       // signed16 左/右
    output wire s_accept,             // enable=1 且 locked=1 时为 1，允许接收音频样本。
    output wire [9:0] sym0,sym1,sym2, // lane0(B)/lane1(G)/lane2(R)，q[0] 先发
    output reg  drop,                 // 缓存已满且未取走样本时收到新样本，输出一拍。
    output wire locked
);
    localparam [2:0] SEL_NULL=3'd0,SEL_ACR=3'd1,SEL_ASP=3'd2,SEL_AVI=3'd3,SEL_AIF=3'd4;
    localparam [10:0] X_DI_PRE=11'd1284, X_DI_GB0=11'd1292, X_DI_PKT=11'd1294, X_DI_GB1=11'd1422,
                      X_DI_END=11'd1424, X_V_PRE=11'd1640, X_V_GB=11'd1648, X_HS=11'd1390;
    localparam [5:0]  ACR_LINES=6'd40;
    localparam [9:0] VGB0=10'b1011001100, VGB1=10'b0100110011, VGB2=10'b1011001100, DGB12=10'b0100110011;

    // ---------------- A 级：寄存输入，重建坐标 ----------------
    reg a_de,a_hs,a_vs; reg [7:0] a_r,a_g,a_b;
    reg [10:0] a_x; reg [9:0] a_y; reg have_h,have_v;
    wire hs_rise=hs && !a_hs;          // 与 a_hs（上一拍输入）比较
    wire vs_rise=vs && !a_vs;
    wire [10:0] x_inc=(a_x==11'd1649) ? 11'd0 : a_x+11'd1;
    wire [10:0] x_new=hs_rise ? X_HS : x_inc;
    wire [9:0]  y_inc=(a_y==10'd749) ? 10'd0 : a_y+10'd1;
    always @(posedge clk) begin
        if(rst) begin
            a_de<=1'b0;a_hs<=1'b0;a_vs<=1'b0;a_r<=8'd0;a_g<=8'd0;a_b<=8'd0;
            a_x<=11'd0;a_y<=10'd0;have_h<=1'b0;have_v<=1'b0;
        end else begin
            a_de<=de;a_hs<=hs;a_vs<=vs;a_r<=r;a_g<=g;a_b<=b;
            a_x<=x_new;
            // vs 上升沿就是第 725 行的 x0；否则在行末回绕时 y+1。
            if(vs_rise) a_y<=10'd725;
            else if(x_new==11'd0) a_y<=y_inc;
            if(hs_rise) have_h<=1'b1;
            if(vs_rise) have_v<=1'b1;
        end
    end
    assign locked=have_h && have_v;
    wire run=locked && enable;
    assign s_accept=run;

    // ---------------- 调度状态（与 A 级同拍更新） ----------------
    // 包槽起点 1294/1326/1358/1390；在“起点前一拍”（A 级坐标=起点-1）装入串行器。
    wire load_slot=run && (a_x==X_DI_PKT-11'd1 || a_x==X_DI_PKT+11'd31 || a_x==X_DI_PKT+11'd63 || a_x==X_DI_PKT+11'd95);
    reg [5:0] acr_line; reg acr_due,avi_pend,aif_pend;
    reg [15:0] buf_l[0:3]; reg [15:0] buf_r[0:3]; reg [2:0] buf_n;
    reg [7:0] iec_frame;
    wire [2:0] sel=acr_due ? SEL_ACR : (buf_n!=3'd0) ? SEL_ASP : avi_pend ? SEL_AVI : aif_pend ? SEL_AIF : SEL_NULL;
    wire line_start=(a_x==11'd1649);   // 下一拍 a_x=0，即新行开始
    wire take_asp=load_slot && sel==SEL_ASP;
    // IEC 帧号推进：本包样本数加到帧号上并模 192。
    wire [8:0] iec_sum={1'b0,iec_frame}+{6'd0,buf_n};
    wire [8:0] iec_wrap=(iec_sum>=9'd192) ? iec_sum-9'd192 : iec_sum;
    wire [7:0] iec_next=iec_wrap[7:0];
    always @(posedge clk) begin
        if(rst) begin
            acr_line<=6'd0; acr_due<=1'b0; avi_pend<=1'b0; aif_pend<=1'b0;
            buf_n<=3'd0; iec_frame<=8'd0; drop<=1'b0;
        end else begin
            drop<=1'b0;
            if(!run) begin
                // 未锁定或未使能：不调度，暂存清空，IEC 计数保持（只在链路复位时回到块边界）。
                acr_due<=1'b0; avi_pend<=1'b0; aif_pend<=1'b0; buf_n<=3'd0;
            end else begin
                // 行计数：每行开始加 1，满 40 行置 ACR 到期（在该行第 0 槽发出）。
                if(line_start) begin
                    if(acr_line==ACR_LINES-6'd1) begin acr_line<=6'd0; acr_due<=1'b1; end
                    else acr_line<=acr_line+6'd1;
                    // 进入第 720 行（垂直消隐首行）时置本帧 InfoFrame 待发。
                    if(a_y==10'd719) begin avi_pend<=1'b1; aif_pend<=1'b1; end
                end
                if(load_slot) begin
                    case(sel)
                        SEL_ACR: acr_due<=1'b0;
                        SEL_AVI: avi_pend<=1'b0;
                        SEL_AIF: aif_pend<=1'b0;
                        default: ;
                    endcase
                end
                if(take_asp) iec_frame<=iec_next;
                // 样本暂存：同拍既取走又来新样本时，新样本放到清空后的第 0 格。
                if(take_asp) begin
                    if(s_valid) begin buf_l[0]<=s_l; buf_r[0]<=s_r; buf_n<=3'd1; end
                    else buf_n<=3'd0;
                end else if(s_valid) begin
                    if(buf_n<3'd4) begin buf_l[buf_n[1:0]]<=s_l; buf_r[buf_n[1:0]]<=s_r; buf_n<=buf_n+3'd1; end
                    else drop<=1'b1;
                end
            end
        end
    end
    // 包内容（组合）→ 串行器
    wire [23:0] pk_header; wire [223:0] pk_sub;
    hdmi_packet_builder u_build(.sel(sel),
        .samp_l({buf_l[3],buf_l[2],buf_l[1],buf_l[0]}),.samp_r({buf_r[3],buf_r[2],buf_r[1],buf_r[0]}),
        .nsamp(buf_n),.iec_frame(iec_frame),.header(pk_header),.sub(pk_sub));
    wire ser_act,ser_h; wire [3:0] ser_even,ser_odd;
    hdmi_packet_serializer u_ser(.clk(clk),.rst(rst),.load(load_slot),.header(pk_header),.sub(pk_sub),
        .active(ser_act),.hbit(ser_h),.even_bits(ser_even),.odd_bits(ser_odd));

    // ---------------- B 级：周期类型与 TERC4 输入 ----------------
    // 下一行是否有视频：当前行 y<719（下一行 ≤719）或 y=749（下一行是新帧第 0 行）。
    wire next_has_video=(a_y<10'd719) || (a_y==10'd749);
    reg b_de,b_hs,b_vs; reg [7:0] b_r,b_g,b_b;
    reg b_vpre,b_vgb,b_dipre,b_digb,b_di;
    reg [3:0] b_n0,b_n1,b_n2;
    always @(posedge clk) begin
        if(rst) begin
            b_de<=1'b0;b_hs<=1'b0;b_vs<=1'b0;b_r<=8'd0;b_g<=8'd0;b_b<=8'd0;
            b_vpre<=1'b0;b_vgb<=1'b0;b_dipre<=1'b0;b_digb<=1'b0;b_di<=1'b0;
            b_n0<=4'd0;b_n1<=4'd0;b_n2<=4'd0;
        end else begin
            b_de<=a_de;b_hs<=a_hs;b_vs<=a_vs;b_r<=a_r;b_g<=a_g;b_b<=a_b;
            b_vpre <=run && next_has_video && a_x>=X_V_PRE && a_x<X_V_GB;
            b_vgb  <=run && next_has_video && a_x>=X_V_GB;              // 1648、1649
            b_dipre<=run && a_x>=X_DI_PRE && a_x<X_DI_GB0;
            b_digb <=run && ((a_x>=X_DI_GB0 && a_x<X_DI_PKT) || (a_x>=X_DI_GB1 && a_x<X_DI_END));
            b_di   <=run && a_x>=X_DI_PKT && a_x<X_DI_GB1 && ser_act;
            // lane0：{D3=非数据岛首拍, D2=包头位, D1=VS, D0=HS}；保护带 lane0 = {1,1,VS,HS}。
            b_n0<=(a_x>=X_DI_PKT && a_x<X_DI_GB1) ? {a_x!=X_DI_PKT,ser_h,a_vs,a_hs} : {2'b11,a_vs,a_hs};
            b_n1<=ser_even;   // lane1：各子包偶数位
            b_n2<=ser_odd;    // lane2：各子包奇数位
        end
    end

    // ---------------- C 级：TMDS 视频/控制编码与 TERC4/保护带 ----------------
    // 前导用编码器自身的控制符号：数据岛前导 CTL0..3=1,0,1,0；视频前导 CTL0..3=1,0,0,0。
    // lane1 的 c={CTL1,CTL0}，lane2 的 c={CTL3,CTL2}（数据岛与视频前导的控制位排列）。
    wire [1:0] c1=(b_vpre||b_dipre) ? 2'b01 : 2'b00;
    wire [1:0] c2=b_dipre ? 2'b01 : 2'b00;
    wire [9:0] e0,e1,e2;
    tmds_encoder u_e0(.clk(clk),.rst(rst),.de(b_de),.c({b_vs,b_hs}),.d(b_b),.q(e0));
    tmds_encoder u_e1(.clk(clk),.rst(rst),.de(b_de),.c(c1),.d(b_g),.q(e1));
    tmds_encoder u_e2(.clk(clk),.rst(rst),.de(b_de),.c(c2),.d(b_r),.q(e2));
    wire [9:0] t0,t1,t2;
    hdmi_terc4 u_t0(.d(b_n0),.q(t0));
    hdmi_terc4 u_t1(.d(b_n1),.q(t1));
    hdmi_terc4 u_t2(.d(b_n2),.q(t2));
    reg c_alt; reg [9:0] c_s0,c_s1,c_s2;
    always @(posedge clk) begin
        if(rst) begin c_alt<=1'b0; c_s0<=10'd0; c_s1<=10'd0; c_s2<=10'd0; end
        else begin
            c_alt<=b_vgb || b_digb || b_di;
            if(b_vgb)       begin c_s0<=VGB0; c_s1<=VGB1;  c_s2<=VGB2;  end
            else if(b_digb) begin c_s0<=t0;   c_s1<=DGB12; c_s2<=DGB12; end
            else            begin c_s0<=t0;   c_s1<=t1;    c_s2<=t2;    end
        end
    end
    assign sym0=c_alt ? c_s0 : e0;
    assign sym1=c_alt ? c_s1 : e1;
    assign sym2=c_alt ? c_s2 : e2;
endmodule
`default_nettype wire
