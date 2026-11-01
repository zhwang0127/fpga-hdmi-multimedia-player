`timescale 1ns/1ps
`default_nettype none
// pix 域顺序频格聚合。core按0..255递增，bin_valid至少隔2拍（当前core固定13拍）。
// start清控制，done晚于最后bin；不能当作每拍连续输入的通用归约器。
// 只取 1..213，16 段最大值，不做宽能量累加。单个峰值寄存器复用，边界后置零。
// 数据没有复位：start 后的第一次 bin(0) 清峰值；new_valid 屏蔽未完成柱高。
module spectrum_bands #(parameter FLOOR=82)(
    input wire clk,rst,start,bin_valid,done,
    input wire [7:0] bin_idx,
    input wire [8:0] bin_lg,
    output reg [95:0] new_h,
    output reg new_valid,
    output wire column_valid,
    output wire [3:0] column_index,
    output wire [5:0] column_height
);
    reg [3:0] band;
    reg [7:0] peak,candidate_q;
    reg [7:0] boundary,next_boundary;
    wire [7:0] maximum;
    reg [7:0] completed_max,delta_q;
    reg [3:0] tag_q,tag_qq;
    reg [3:0] accepted_band,band_next;
    reg high_gt,high_eq,middle_gt,middle_eq,low_ge,pick_valid,accepted_boundary;
    reg complete_q,complete_qq,below_q,above_q;
    wire [5:0] height;
    genvar g;
    // core的re/im有16位表示，功率<=2^31，所以log编码最高位恒0，只需8位载荷。
    // 候选比较分为3/3/2位先寄存，再选最大，进位比较后不直接串数据选择。
    function gt3;
        input [2:0] a,b;
        begin gt3=(a[2]&&!b[2]) || ((a[2]~^b[2])&&a[1]&&!b[1]) ||
            ((a[2]~^b[2])&&(a[1]~^b[1])&&a[0]&&!b[0]);end
    endfunction
    function gt2;
        input [1:0] a,b;
        begin gt2=(a[1]&&!b[1]) || ((a[1]~^b[1])&&a[0]&&!b[0]);end
    endfunction
    function gt8;
        input [7:0] a,b;
        begin gt8=gt2(a[7:6],b[7:6]) || ((a[7:6]==b[7:6])&&gt2(a[5:4],b[5:4])) ||
            ((a[7:4]==b[7:4])&&gt2(a[3:2],b[3:2])) || ((a[7:2]==b[7:2])&&gt2(a[1:0],b[1:0]));end
    endfunction
    assign maximum=(high_gt || (high_eq && (middle_gt || (middle_eq && low_ge))))?candidate_q:peak;
    assign height=below_q?6'd0:(above_q?6'd63:delta_q[6:1]);
    // column 输出与 new_h 在同一沿更新，串行平滑直接接收该列结果。
    // new_h 同时保留整帧柱高，避免下游再次逐列选择。
    assign column_valid=complete_qq;
    assign column_index=tag_qq;
    assign column_height=height;
    // 固定切片写使能，避免 variable part-select 写口被综合成 96 位桶形移位器。
    generate for(g=0;g<16;g=g+1) begin : height_store
        always @(posedge clk)
            if(complete_qq && tag_qq==g)
                new_h[g*6+:6]<=height;
    end endgenerate
    generate if(FLOOR<0 || FLOOR>255) begin : invalid_floor
        spectrum_floor_must_be_0_to_255 u_error();
    end endgenerate
    always @* begin
        // 提前算下一段边界，数据只由当前band决定。匹配频格只驱动CE，
        // 避免bin_idx→边界比较→band加法→ROM再串到boundary的D端。
        case(band)
        0:next_boundary=2; 1:next_boundary=3; 2:next_boundary=4; 3:next_boundary=6;
        4:next_boundary=8; 5:next_boundary=11; 6:next_boundary=15; 7:next_boundary=21;
        8:next_boundary=29; 9:next_boundary=40; 10:next_boundary=55; 11:next_boundary=76;
        12:next_boundary=105; 13:next_boundary=145; default:next_boundary=213;
        endcase
    end
    always @(posedge clk) begin
        band_next<=band+1'b1;
        // 比较峰值、减 FLOOR、限幅分开三拍；不能把比较+进位链+多路选择串到同一拍。
        if(bin_valid) begin
            candidate_q<=bin_lg[7:0];accepted_band<=band;accepted_boundary<=(bin_idx==boundary);
            high_gt<=gt3(bin_lg[7:5],peak[7:5]);high_eq<=(bin_lg[7:5]==peak[7:5]);
            middle_gt<=gt3(bin_lg[4:2],peak[4:2]);middle_eq<=(bin_lg[4:2]==peak[4:2]);
            low_ge<=!gt2(peak[1:0],bin_lg[1:0]);
        end
        if(pick_valid && accepted_boundary) begin
            completed_max<=maximum; tag_q<=accepted_band;
        end
        delta_q<=completed_max-FLOOR;
        below_q<=!gt8(completed_max,FLOOR);
        above_q<=(FLOOR+126>255)?1'b0:!gt8(FLOOR+126,completed_max);
        tag_qq<=tag_q;
        if (bin_valid && bin_idx==0) peak<=0;
        else if (pick_valid) begin
            if (accepted_boundary) begin
                peak<=0;
            end else peak<=maximum;
        end
        if (rst) begin band<=0; boundary<=1; new_valid<=0; pick_valid<=0; complete_q<=0; complete_qq<=0; end
        else begin
            pick_valid<=bin_valid && bin_idx!=0 && !gt8(bin_idx,8'd213);
            complete_q<=pick_valid && accepted_boundary;
            complete_qq<=complete_q;
            new_valid<=done;
            if (start) begin band<=0; boundary<=1; end
            else if (bin_valid && bin_idx==boundary && band!=15) begin band<=band_next; boundary<=next_boundary; end
        end
    end
endmodule
`default_nettype wire
