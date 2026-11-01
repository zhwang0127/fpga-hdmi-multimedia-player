`timescale 1ns/1ps
`default_nettype none
// 频谱四拍叠加器：所有端口都在pix域，输出可直接串到后续OSD，OSD应使用o_x/o_y。
// 第一拍只寄存输入，扫描RGB到本模块第一拍之间没有几何比较或颜色选择。
// P2算位置，P3比较，P4合色；全部视频信号同延迟。
// 柱高保存在 spectrum_bars_stream 的 BRAM 显示页，通过 h_addr/h_value 同步读口取得。
// P1 依据寄存坐标给出柱列号，P2 同拍取得该列高度，随后比较高度并混合前景色。
// BRAM 读延迟包含在四拍视频流水中；OSD 应连接本模块输出的坐标和视频信号。
module spectrum_overlay #(
    parameter integer X0=128, PITCH_LOG2=6, BAR_WIDTH=48,
    parameter integer BOTTOM=703, HEIGHT_SHIFT=1,
    parameter integer BG_X0=X0-8, BG_X1=X0+(15<<PITCH_LOG2)+BAR_WIDTH+7,
    parameter integer BG_Y0=BOTTOM-(63<<HEIGHT_SHIFT)-5, BG_Y1=BOTTOM+5,
    parameter [23:0] COLOR_LOW=24'h00ff00, COLOR_MID=24'hffff00, COLOR_HIGH=24'hff0000
)(
    input wire clk,rst,
    // h_addr 是组合输出（只由 P1 寄存器决定），接柱高 BRAM 读地址；h_data 为下一拍读数。
    output wire [3:0] h_addr,
    input wire [5:0] h_data,
    // pix 帧边界准静态开关：同时屏蔽柱和压暗框，不改变四拍流水或柱高读口。
    input wire hide,
    input wire bars_valid,
    input wire de,hs,vs,
    input wire [10:0] x,
    input wire [9:0] y,
    input wire [7:0] r,g,b,
    output reg o_de,o_hs,o_vs,
    output reg [10:0] o_x,
    output reg [9:0] o_y,
    output reg [7:0] o_r,o_g,o_b
);
    localparam integer PITCH=1<<PITCH_LOG2;
    localparam integer MAX_HEIGHT=63<<HEIGHT_SHIFT;
    localparam integer X_BASE=X0>>PITCH_LOG2;
    // 列号只用模16的4位；单独定义带位宽的常数，避免对 integer 参数取位段。
    localparam [3:0] X_BASE_MOD16=X_BASE&15;
    localparam integer X_LOW=X0&(PITCH-1);
    // 比较器实参显式扩展到 11 位，保证坐标及边界使用相同位宽。
    // 这里只支持可见光栅范围内的非负几何；超范围参数须先扩位和重验，不能靠截位使用。
    localparam [10:0] CMP_X_LOW=X_LOW,CMP_X0=X0,CMP_X1=X0+16*PITCH,CMP_WIDTH=BAR_WIDTH;
    localparam [10:0] CMP_BG_X0=BG_X0,CMP_BG_X1=BG_X1,CMP_BG_Y0=BG_Y0,CMP_BG_Y1=BG_Y1;
    localparam [10:0] CMP_BOTTOM=BOTTOM,CMP_TOP=BOTTOM-MAX_HEIGHT;
    localparam [10:0] CMP_LOW_COLOR=MAX_HEIGHT/3,CMP_MID_COLOR=2*MAX_HEIGHT/3;
    reg p1_de,p1_hs,p1_vs,p1_ok;
    reg [10:0] p1_x;
    reg [9:0] p1_y;
    reg [23:0] p1_rgb;
    reg p2_de,p2_hs,p2_vs,p2_box,p2_column;
    reg [10:0] p2_x;
    reg [9:0] p2_y,p2_dy;
    reg [23:0] p2_rgb;
    reg p3_de,p3_hs,p3_vs,p3_box;
    reg p3_high_gt,p3_high_eq,p3_middle_gt,p3_middle_eq,p3_low_gt;
    reg [10:0] p3_x;
    reg [9:0] p3_y;
    reg [23:0] p3_rgb,p3_color;
    wire p1_borrow;
    wire [3:0] p1_column_index;
    wire [10:0] x_offset=sub_x(p1_x);
    wire [9:0] y_bottom=sub_y(p1_y);
    // P2 拍的柱高直接是 BRAM 读数（BRAM 输出到 P3 比较寄存器之间只有 2/2/2 位比较）。
    wire [5:0] selected_h=h_data;
    (* keep *) wire pixel_hit=p3_high_gt || (p3_high_eq && (p3_middle_gt || (p3_middle_eq && p3_low_gt)));
    // 几何比较明确写成逐位布尔前缀，防止综合把“比较+条件选择”变成进位链再串LUT。
    // 常数减法同样展开借位；默认BOTTOM低6位全1，会直接化为低6位取反，不串减法器。
    function gt11;
        input [10:0] a,b;
        integer bit_index;
        reg equal_prefix,greater;
        begin
            equal_prefix=1;greater=0;
            for(bit_index=10;bit_index>=0;bit_index=bit_index-1) begin
                greater=greater | (equal_prefix && a[bit_index] && !b[bit_index]);
                equal_prefix=equal_prefix && (a[bit_index]~^b[bit_index]);
            end
            gt11=greater;
        end
    endfunction
    function [10:0] sub_x;
        input [10:0] value;
        integer bit_index;reg borrow;
        begin
            borrow=0;
            for(bit_index=0;bit_index<11;bit_index=bit_index+1) begin
                sub_x[bit_index]=value[bit_index]^((X0>>bit_index)&1)^borrow;
                borrow=(!value[bit_index] && ((X0>>bit_index)&1)) ||
                    (!(value[bit_index]^((X0>>bit_index)&1)) && borrow);
            end
        end
    endfunction
    function [9:0] sub_y;
        input [9:0] value;
        integer bit_index;reg borrow;
        begin
            borrow=0;
            for(bit_index=0;bit_index<10;bit_index=bit_index+1) begin
                sub_y[bit_index]=((BOTTOM>>bit_index)&1)^value[bit_index]^borrow;
                borrow=(!((BOTTOM>>bit_index)&1) && value[bit_index]) ||
                    (!(((BOTTOM>>bit_index)&1)^value[bit_index]) && borrow);
            end
        end
    endfunction
    // 列号由高位偏移和低位借位合成： ((p1_x[高4位]-X_BASE)&15)，借位时用 B 表（再减1）。
    // 这里直接算同一个 4 位模16差，只依赖 p1_x 的 4 个列位和（非对齐时）低位借位；
    // 默认 X0=128 时 borrow 恒 0、X_BASE=2，只剩 4 位减常数 2，没有乘法/动态切片。
    // 地址不在 P1 之前算：扫描输入到 P1 仍保持“只直连寄存”的约束。
    assign p1_borrow=gt11(CMP_X_LOW,{{(11-PITCH_LOG2){1'b0}},p1_x[PITCH_LOG2-1:0]});
    assign p1_column_index=p1_x[PITCH_LOG2+:4]-X_BASE_MOD16-{3'd0,p1_borrow};
    assign h_addr=p1_column_index;
    always @(posedge clk) begin
        // P1：这些数据入口只有直连寄存，不能在这里加扫描位置判断。
        p1_x<=x;p1_y<=y;p1_rgb<={r,g,b};
        if(rst) begin p1_de<=0;p1_hs<=0;p1_vs<=0;p1_ok<=0;end
        else begin p1_de<=de;p1_hs<=hs;p1_vs<=vs;p1_ok<=bars_valid;end
        // P2：坐标运算/边框判断与柱高 BRAM 读取（h_addr 在本沿采样）并行。
        p2_x<=p1_x;p2_y<=p1_y;p2_rgb<=p1_rgb;p2_dy<=y_bottom;
        if(rst) begin p2_de<=0;p2_hs<=0;p2_vs<=0;p2_box<=0;p2_column<=0;end
        else begin
            p2_de<=p1_de;p2_hs<=p1_hs;p2_vs<=p1_vs;
            p2_box<=p1_de && !hide && !gt11(CMP_BG_X0,p1_x) && !gt11(p1_x,CMP_BG_X1) && !gt11(CMP_BG_Y0,{1'b0,p1_y}) && !gt11({1'b0,p1_y},CMP_BG_Y1);
            p2_column<=p1_de && p1_ok && !hide && !gt11(CMP_X0,p1_x) && gt11(CMP_X1,p1_x) &&
                gt11(CMP_WIDTH,{{(11-PITCH_LOG2){1'b0}},x_offset[PITCH_LOG2-1:0]}) && !gt11({1'b0,p1_y},CMP_BOTTOM) && gt11({1'b0,p1_y},CMP_TOP);
        end
        // P3：BRAM 读数分2/2/2位比较；P4只合并条件，不把整条6位进位比较串在读口后。
        // column保证dy<MAX_HEIGHT，所以只取6位缩放值不截掉有效信息。
        p3_x<=p2_x;p3_y<=p2_y;p3_rgb<=p2_rgb;
        p3_color<=gt11(CMP_LOW_COLOR,{1'b0,p2_dy}) ? COLOR_LOW : gt11(CMP_MID_COLOR,{1'b0,p2_dy}) ? COLOR_MID : COLOR_HIGH;
        p3_middle_gt<=gt11({9'd0,selected_h[3:2]},{9'd0,p2_dy[HEIGHT_SHIFT+2+:2]});
        p3_middle_eq<=(selected_h[3:2]==p2_dy[HEIGHT_SHIFT+2+:2]);
        p3_low_gt<=gt11({9'd0,selected_h[1:0]},{9'd0,p2_dy[HEIGHT_SHIFT+:2]});
        if(rst) begin p3_de<=0;p3_hs<=0;p3_vs<=0;p3_box<=0;p3_high_gt<=0;p3_high_eq<=0;end
        else begin
            p3_de<=p2_de;p3_hs<=p2_hs;p3_vs<=p2_vs;p3_box<=p2_box;
            p3_high_gt<=p2_column && gt11({9'd0,selected_h[5:4]},{9'd0,p2_dy[HEIGHT_SHIFT+4+:2]});
            p3_high_eq<=p2_column && (selected_h[5:4]==p2_dy[HEIGHT_SHIFT+4+:2]);
        end
        // P4：只用上拍的命中/颜色选择，背景RGB逐通道右移1；消隐不改RGB数据内容。
        o_x<=p3_x;o_y<=p3_y;
        o_r<=pixel_hit ? p3_color[23:16] : p3_box ? {1'b0,p3_rgb[23:17]} : p3_rgb[23:16];
        o_g<=pixel_hit ? p3_color[15:8] : p3_box ? {1'b0,p3_rgb[15:9]} : p3_rgb[15:8];
        o_b<=pixel_hit ? p3_color[7:0] : p3_box ? {1'b0,p3_rgb[7:1]} : p3_rgb[7:0];
        if(rst) begin o_de<=0;o_hs<=0;o_vs<=0;end
        else begin o_de<=p3_de;o_hs<=p3_hs;o_vs<=p3_vs;end
    end
endmodule
`default_nettype wire
