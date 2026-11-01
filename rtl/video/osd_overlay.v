`timescale 1ns/1ps
`default_nettype none
// OSD 与亮度叠加，运行在 pix 域。
// sys 状态经顶层 cdc_mailbox 送入；仅在 vblank_start 将整组状态快照到 disp，帧内保持。
// 图片编号和 WAV/TONE 来源取自本域已提交的显示引用，避免菜单状态与实际播放不同步。
// 字符格为 16×32 像素，网格起点 (192,32)，共 44 列，含状态行及 MENU_ROWS 个菜单行。
// 白色正文、黄色选中项；文字底框将原像素降到四分之一亮度。
// 流水共 3 拍：版面 ROM 同步读、字库 ROM 同步读、取字形位并混色寄存输出。
// DE/HS/VS/RGB 与坐标同步推进；字库和版面各使用一块 BRAM9K。
// 控制和有效位同步复位，纯数据流水不复位，由有效位保证先写后用。
// spec_hide 和 vol_level 来自同一帧快照，分别控制频谱可见性及最终 PCM 音量。
module osd_overlay #(
    parameter integer STATUS_W=48,
    parameter integer MENU_ROWS=5,
    // 1 使用器件 BRAM 字库和版面，0 使用相同内容的同步行为 ROM。
    parameter USE_EG_BRAM=1,
    // 1 使用器件乘法宏，0 使用行为乘法；输入和输出各寄存一拍。
    parameter USE_EG_DSP=USE_EG_BRAM
)(
    input  wire clk,rst,
    // mailbox 目标口：valid 一拍有效（顶层 dst_ready 恒 1）；st_data 在两次 valid 之间保持（= mailbox received_bus）。
    input  wire st_valid,
    input  wire [STATUS_W-1:0] st_data,
    input  wire vblank_start,
    input  wire ref_valid,
    input  wire [1:0] ref_slot,
    input  wire audio_wav,
    // 原视频流（与 x/y 同拍，来自 bundle_scanout 第二级寄存）
    input  wire [10:0] x,
    input  wire [9:0]  y,
    input  wire de,hs,vs,
    input  wire [7:0] r,g,b,
    // 仅由帧快照驱动；关闭频谱时，柱与底框一起隐藏，后台计算继续。
    output wire spec_hide,
    // 同一帧快照解出音量，首份状态前全音量，异常编码按16夹紧。
    output wire [4:0] vol_level,
    output reg  o_de,o_hs,o_vs,
    output reg  [7:0] o_r,o_g,o_b
);
    // ---------------- 状态字：mailbox 目标寄存器（最新收到）→ disp（帧边界快照） ----------------
    // mailbox 的 dst_data 在两次接收之间保持，无需再复制一份待显示状态。
    // pend_ok 记录复位后是否已收到状态，防止使用尚未初始化的 st_data。
    // vblank_start 拍把 st_data 整组拷进 disp，一帧内显示内容不变。
    reg [STATUS_W-1:0] disp;
    reg pend_ok,disp_ok;
    reg disp_ref_valid,disp_wav; reg [1:0] disp_slot;
    always @(posedge clk) begin
        if(rst) begin pend_ok<=1'b0;disp_ok<=1'b0;disp_ref_valid<=1'b0;end
        else begin
            if(st_valid) pend_ok<=1'b1;
            if(vblank_start) begin disp_ok<=pend_ok;disp_ref_valid<=ref_valid;end
        end
    end
    // 数据字段不复位：只有 *_ok 置位后才会被解释（先写后用）。
    always @(posedge clk) begin
        if(vblank_start) begin disp<=st_data;disp_slot<=ref_slot;disp_wav<=audio_wav && ref_valid;end
    end
    // 状态字布局（与 b3_control 中 osd_status 组装一致）：
    // L2：[0]handoff [3:1]menu_pos(0关闭，1..5菜单行) [4]AUTO [5]来源KEY
    // [12:6]周期BCD7 [19:13]倒计时BCD7 [27:20]装载码 [35:28]播放码 [36]倒计时有效
    // [38:37]DISP [42:39]亮度索引0..15 [47:43]音量0..16
    wire s_handoff=disp_ok && disp[0];
    wire [2:0] menu_pos=disp_ok ? disp[3:1] : 3'd0;
    wire s_open=menu_pos!=3'd0;
    wire s_auto=disp[4];
    wire s_key=disp[5];
    wire [7:0] s_per={1'b0,disp[12:6]};
    wire [7:0] s_cd={1'b0,disp[19:13]};
    wire [7:0] s_err=disp[27:20];
    wire [7:0] s_pc=disp[35:28];
    wire s_cd_ok=disp[36];
    wire [1:0] s_disp=disp_ok ? disp[38:37] : 2'b00;
    wire hide_stat=s_disp[0];
    assign spec_hide=s_disp[1];
    wire [4:0] raw_vol=disp_ok ? disp[47:43] : 5'd16;
    assign vol_level=raw_vol>5'd16 ? 5'd16 : raw_vol;
    // 字符码：ASCII-0x20。
    localparam [5:0] C_SP=6'h00,C_DASH=6'h0d,C_GT=6'h1e;
    function [5:0] dig; input [3:0] d; begin dig=6'h10+{2'b00,d}; end endfunction
    function [5:0] hex; input [3:0] d; begin hex=d<4'd10 ? 6'h10+{2'b00,d} : 6'h21+{2'b00,d}-6'd10; end endfunction
    function [5:0] asc; input [7:0] c; begin asc=c[5:0]-6'h20; end endfunction
    wire [5:0]  f_id=disp_ref_valid ? 6'h11+{4'd0,disp_slot} : C_DASH;
    wire [23:0] f_mode=!disp_ok ? {C_DASH,C_DASH,C_DASH,C_DASH} :
                       s_auto ? {asc("O"),asc("T"),asc("U"),asc("A")} : {C_SP,asc("N"),asc("A"),asc("M")};
    wire [11:0] f_per=s_handoff ? {dig(s_per[3:0]),dig(s_per[7:4])} : {C_DASH,C_DASH};
    wire [17:0] f_src=!disp_ok ? {C_DASH,C_DASH,C_DASH} : s_key ? {asc("Y"),asc("E"),asc("K")} : {C_SP,asc("W"),asc("S")};
    wire [11:0] f_cd=(s_handoff && s_auto && s_cd_ok) ? {dig(s_cd[3:0]),dig(s_cd[7:4])} : {C_DASH,C_DASH};
    wire [23:0] f_aud=disp_wav ? {C_SP,asc("V"),asc("A"),asc("W")} : {asc("E"),asc("N"),asc("O"),asc("T")};
    wire [11:0] f_err=disp_ok ? {hex(s_err[3:0]),hex(s_err[7:4])} : {C_DASH,C_DASH};
    wire [11:0] f_pc=disp_ok ? {hex(s_pc[3:0]),hex(s_pc[7:4])} : {C_DASH,C_DASH};
    wire [4:0] bri=disp_ok ? {1'b0,disp[42:39]}+5'd1 : 5'd16;
    wire [4:0] bri_ones=bri>=5'd10 ? bri-5'd10 : bri;
    wire [11:0] f_bri={dig(bri_ones[3:0]),dig({3'd0,bri>=5'd10})};
    wire [4:0] vol_ones=vol_level>=5'd10 ? vol_level-5'd10 : vol_level;
    wire [11:0] f_vol={dig(vol_ones[3:0]),dig({3'd0,vol_level>=5'd10})};
    // 增益与 disp 同沿快照，直接用 mailbox 已保持的状态；复位/首份状态前均为 16。
    reg [4:0] k_q;
    always @(posedge clk) begin
        if(rst) k_q<=5'd16;
        else if(vblank_start) k_q<=pend_ok ? {1'b0,st_data[42:39]}+5'd1 : 5'd16;
    end
    wire [23:0] f_disp=s_disp==2'd0 ? {C_SP,asc("L"),asc("L"),asc("A")} :
                         s_disp==2'd1 ? {asc("D"),asc("I"),asc("O"),asc("N")} :
                         s_disp==2'd2 ? {asc("P"),asc("S"),asc("O"),asc("N")} :
                                         {C_SP,asc("F"),asc("F"),asc("O")};

    // ---------------- S0（组合）→P1：坐标→格号→版面 ROM ----------------
    // 网格 x∈[192,896)、y∈[32,32+32*(1+MENU_ROWS))；列=x[10:4]-12，行=y[9:5]-1，格内点阵坐标=(x[3:1],y[4:1])。
    // 版面 ROM 地址 {row[2:0],col[5:0]} 由本拍坐标组合给出，ROM 输出寄存器即 P1 级；网格外地址无意义，由 p1_in 屏蔽。
    localparam [10:0] GX0=11'd192,GX1=11'd896;
    localparam [9:0]  GY0=10'd32,GY1=10'd32+10'd32*(MENU_ROWS+1);
    wire [6:0] col_raw=x[10:4]-7'd12;
    wire [4:0] row_raw=y[9:5]-5'd1;
    wire [7:0] t_q;   // {bg,is_field,code_or_index[5:0]}，P1 级（ROM 内部寄存）
    osd_text_rom #(.USE_EG_BRAM(USE_EG_BRAM)) u_text(.clk(clk),.addr({row_raw[2:0],col_raw[5:0]}),.q(t_q));
    reg p1_in; reg [2:0] p1_row; reg [3:0] p1_gy; reg [2:0] p1_gx;
    reg p1_de,p1_hs,p1_vs;
    // 五个选中箭头合成同一字段，字段字符总数保持 <=32；menu_pos 正好等于菜单行号。
    wire [5:0] f_sel=p1_row==menu_pos ? C_GT : C_SP;
    always @(posedge clk) begin
        if(rst) p1_in<=1'b0;
        else p1_in<=x>=GX0 && x<GX1 && y>=GY0 && y<GY1;
        p1_row<=row_raw[2:0];p1_gy<=y[4:1];p1_gx<=x[3:1];
        p1_de<=de;p1_hs<=hs;p1_vs<=vs;
    end
    // ---------------- P1（组合）→P2：字段选择→字库 ROM，可见/选中 ----------------
    wire [5:0] field_code;
    osd_field_sel u_field(.idx({1'b0,t_q[4:0]}),.f_aud(f_aud),.f_cd(f_cd),.f_err(f_err),.f_id(f_id),.f_mode(f_mode),
        .f_pc(f_pc),.f_per(f_per),.f_sel(f_sel),.f_bri(f_bri),.f_vol(f_vol),.f_disp(f_disp),.f_src(f_src),.code(field_code));
    wire [5:0] text_code=t_q[6] ? field_code : t_q[5:0];
    // 底框位已含“状态行 44 格 / 菜单行 20 格”；菜单行另需菜单打开。
    wire cell_vis=p1_in && t_q[7] && (p1_row==3'd0 ? !hide_stat : s_open);
    wire cell_sel=p1_row!=3'd0 && p1_row==menu_pos;
    wire [7:0] font_bits;   // P2 级（ROM 内部寄存）；字形行 = gy[3:1]（每个点阵行纵向重复两次）
    osd_font_rom #(.USE_EG_BRAM(USE_EG_BRAM)) u_font(.clk(clk),.addr({text_code,p1_gy[3:1]}),.q(font_bits));
    reg p2_in,p2_sel; reg [2:0] p2_gx;
    reg p2_de,p2_hs,p2_vs;
    // 三个 18x18 有符号宏，高位补 0；输入寄存=P1，输出寄存=P2。
    // 亮度只作用于进入 OSD 前的图像+频谱；P2 后才画文字，文字保持满亮度。
    wire signed [35:0] prod_r,prod_g,prod_b;
    spectrum_mul18 #(.USE_EG_DSP(USE_EG_DSP)) u_bright_r(.clk(clk),.a({10'd0,r}),.b({13'd0,k_q}),.p(prod_r));
    spectrum_mul18 #(.USE_EG_DSP(USE_EG_DSP)) u_bright_g(.clk(clk),.a({10'd0,g}),.b({13'd0,k_q}),.p(prod_g));
    spectrum_mul18 #(.USE_EG_DSP(USE_EG_DSP)) u_bright_b(.clk(clk),.a({10'd0,b}),.b({13'd0,k_q}),.p(prod_b));
    // (p*k+8)>>4；低四位只需取 bit3 作为舍入进位。p<=255、k<=16，无饱和溢出。
    wire [8:0] round_r={1'b0,prod_r[11:4]}+{8'd0,prod_r[3]};
    wire [8:0] round_g={1'b0,prod_g[11:4]}+{8'd0,prod_g[3]};
    wire [8:0] round_b={1'b0,prod_b[11:4]}+{8'd0,prod_b[3]};
    wire [7:0] p2_r=round_r[7:0],p2_g=round_g[7:0],p2_b=round_b[7:0];
    always @(posedge clk) begin
        if(rst) p2_in<=1'b0;
        else p2_in<=cell_vis;
        p2_sel<=cell_sel;p2_gx<=p1_gx;
        p2_de<=p1_de;p2_hs<=p1_hs;p2_vs<=p1_vs;
    end
    // ---------------- P2（组合）→O：取位、混合并寄存输出 ----------------
    // 前景：白色，当前菜单项黄色；背景：原像素 /4；其余原像素。
    wire p2_fg=p2_in && font_bits[3'd7-p2_gx];
    always @(posedge clk) begin
        if(rst) begin o_de<=1'b0;o_hs<=1'b0;o_vs<=1'b0;o_r<=8'd0;o_g<=8'd0;o_b<=8'd0;end
        else begin
            o_de<=p2_de;o_hs<=p2_hs;o_vs<=p2_vs;
            o_r<=p2_fg ? 8'hff : p2_in ? {2'b00,p2_r[7:2]} : p2_r;
            o_g<=p2_fg ? 8'hff : p2_in ? {2'b00,p2_g[7:2]} : p2_g;
            o_b<=p2_fg ? (p2_sel ? 8'h00 : 8'hff) : p2_in ? {2'b00,p2_b[7:2]} : p2_b;
        end
    end
endmodule
`default_nettype wire
