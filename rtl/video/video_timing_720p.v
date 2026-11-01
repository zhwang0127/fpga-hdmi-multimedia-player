`timescale 1ns/1ps
// 功能：74.25 MHz 下的 1280x720@60Hz 光栅时序，不包含 PLL/TMDS/音频数据岛。
// 水平：有效1280 + 前沿110 + 同步40 + 后沿220 = 1650 周期。
// 垂直：有效720 + 前沿5 + 同步5 + 后沿20 = 750 行。
// 接口：rst 同步高有效；x/y 与 de、同步和起始标志描述同一个当前像素周期。
// Coordinate origin is the first active pixel. Positive H/V sync.
module video_timing_720p(input wire clk,input wire rst,
    output reg [10:0] x,output reg [9:0] y,
    output wire de,output wire hsync,output wire vsync,
    output wire frame_start,output wire vblank_start,
    // 由下一状态预算并寄存的 y 派生量，每拍恒等于对当前 y 的组合函数
    // （y_lt720==(y<720)、y_lt719==(y<719)、y_plus1==y+1、y_horizon==(y<718?y+2:y<720?719:1)），
    // 供 bundle_scanout 直接使用，避免在请求路径重复纵向比较和加法。
    output reg y_lt720,output reg y_lt719,output reg [9:0] y_plus1,output reg [9:0] y_horizon,
    // bundle_scanout 行描述符 row_desc(y)、row_desc(y+1)、row_desc(y+2)（10 位回绕），同样由下一状态寄存，
    // 每拍恒等于对当前 y 的组合函数。row_desc={2y mod 3, source_y, preview_slot, preview_y}，三个函数与 bundle_scanout 逐字相同。
    output reg [19:0] y_desc0,output reg [19:0] y_desc1,output reg [19:0] y_desc2,
    // 从下一水平位置寄存零点/行末条件，与当前x同拍，不增加请求等待拍。
    // cold_line保留rst的组合屏蔽语义；复位在两沿间撤销时也不能多维持一拍。
    output wire cold_line_qual,output reg row_edge_qual);
    reg x_zero_q;
    always @(posedge clk) begin
        // 水平派生量按 11 位计数器的完整取值范围计算，包含 2047 到 0 的回绕。
        x_zero_q<=rst || x==11'd1649 || x==11'd2047;
        row_edge_qual<=!rst && x==11'd1648;
    end
    assign cold_line_qual=x_zero_q && y_plus1==10'd1 && rst;
    function [8:0] source_y;input [9:0] yy;reg [10:0] w;
        begin w={1'b0,yy}<<1;source_y=w/11'd3;end
    endfunction
    function [1:0] preview_slot;input [9:0] yy;
        begin if(yy>=496) preview_slot=3;else if(yy>=352) preview_slot=2;
        else if(yy>=208) preview_slot=1;else preview_slot=0;end
    endfunction
    function [6:0] preview_y;input [9:0] yy;reg [9:0] base,d;
        begin case(preview_slot(yy)) 0:base=64;1:base=208;2:base=352;default:base=496;endcase
        d=yy-base;preview_y=(yy>=base && d<96) ? d[6:0] : 7'd0;end
    endfunction
    function [19:0] row_desc;input [9:0] yy;reg [10:0] w,m;
        begin w={1'b0,yy}<<1;m=w%11'd3;row_desc={m[1:0],source_y(yy),preview_slot(yy),preview_y(yy)};end
    endfunction
    wire [9:0] ny=rst ? 10'd0 : x==1649 ? (y==749 ? 10'd0 : y+1'b1) : y;
    always @(posedge clk) begin
        if(rst) begin x<=0;y<=0;end
        else if(x==1649) begin
            // 到达行尾才更新 y；最后一行结束后返回帧的首个有效像素。
            x<=0;
            if(y==749) y<=0;else y<=y+1'b1;
        end else x<=x+1'b1;
    end
    always @(posedge clk) begin
        y_lt720<=ny<10'd720;y_lt719<=ny<10'd719;y_plus1<=ny+10'd1;
        y_horizon<=ny<10'd718 ? ny+10'd2 : ny<10'd720 ? 10'd719 : 10'd1;
    end
    // 行内描述符不变，行末desc1→desc0、desc2→desc1，
    // 只有最新desc2增量计算。不能把行变化放到下一拍，否则请求描述符会与y错一拍。
    // source_y和相位按2n/3增量；preview仍按准确的目标行计算，保留10位回绕。
    wire [9:0] desc_new_row=y_plus1+10'd2;
    wire [1:0] desc_ph=y_desc2[19:18];wire [8:0] desc_src=y_desc2[17:9];
    wire [19:0] desc_step=desc_new_row==0 ? row_desc(10'd0) :
        {desc_ph==0 ? 2'd2 : desc_ph-2'd1,desc_ph!=0 ? desc_src+9'd1 : desc_src,
         preview_slot(desc_new_row),preview_y(desc_new_row)};
    always @(posedge clk) begin
        if(rst || (x==1649 && y==749)) begin
            y_desc0<=row_desc(10'd0);y_desc1<=row_desc(10'd1);y_desc2<=row_desc(10'd2);
        end else if(x==1649) begin
            y_desc0<=y_desc1;y_desc1<=y_desc2;y_desc2<=desc_step;
        end
    end
`ifdef FORMAL
    // 首个时钟沿后（无复位寄存器）各派生量恒等于对当前 y 的组合函数。
    reg f_v=1'b0;always @(posedge clk) f_v<=1'b1;
    // 复位后计数范围和帧边界条件保持一致。
    always @* if(f_v) begin
        assert(x<1650 && y<750);
        assert(x_zero_q==(x==0) && row_edge_qual==(x==1649));
        assert(cold_line_qual==(y_plus1==10'd1 && x==0 && !de));
    end
    always @* if(f_v) assert(y_lt720==(y<10'd720) && y_lt719==(y<10'd719) && y_plus1==y+10'd1 &&
        y_horizon==(y<10'd718 ? y+10'd2 : y<10'd720 ? 10'd719 : 10'd1) &&
        y_desc0==row_desc(y) && y_desc1==row_desc(y+10'd1) && y_desc2==row_desc(y+10'd2));
`endif
    // 有效区从 (0,0) 开始，de 使用与 y 同拍的 y_lt720，避免在像素输出路径重复宽比较。
    // 复位期间屏蔽 de；复位释放前，纵向计数及其派生条件已同步建立。
    assign de=!rst&&(x<1280)&&y_lt720;
    // 同步信号为正极性，比较采用左闭右开区间，避免多输出一个时钟/行。
    assign hsync=!rst&&(x>=1390)&&(x<1430);
    assign vsync=!rst&&(y>=725)&&(y<730);
    // 两个起始标志各占一个像素周期。消隐起点是帧交换的候选提交边界，
    // 还必须结合后续首行预取/音频准备状态，不可单凭此标志切换帧槽。
    assign frame_start=!rst&&(x==0)&&(y==0);
    assign vblank_start=!rst&&(x==0)&&(y==720);
endmodule
