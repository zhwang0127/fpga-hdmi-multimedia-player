`timescale 1ns/1ps
`default_nettype none
// sys 域 OSD 菜单控制，使用 ms_tick 进行按键计时和倒计时换算。
// K3 短按依次打开菜单、选择下一项、关闭菜单；长按不执行动作。
// K4 短按增加一档，长按 750ms 减少一档；菜单关闭时忽略 K4。
// MODE 请求切换手动/自动；PERIOD 从预设周期中选择，修改后重新计时。
// 其余菜单项控制显示方式、亮度和音量，状态由 b3_control 打包送往 pix 域。
// enable 生效前允许翻页但不修改参数；倒计时将 remain_ms 向上换算为秒并转为 BCD。
// 控制和状态同步复位，串行运算数据在开始计算时写入。
module osd_menu #(
    parameter integer ITEMS=5
)(
    input  wire clk,logic_rst,ms_tick,
    input  wire k3_short,k4_short,k4_long,
    input  wire enable,auto_on,
    input  wire [5:0] cfg_period,
    input  wire cfg_apply,
    input  wire [15:0] remain_ms,
    output reg  menu_open,
    output reg  [2:0] item,
    // BRIGHT 索引 0..15 对应显示 01..16；上电全亮，短/长按饱和，不自动连发。
    output reg  [3:0] bright_idx,
    // 音量00静音、01..16逐档，默认16；只变目标，pix样本域负责平滑增益。
    output reg  [4:0] vol_lvl,
    // DISP 是枚举开关：bit0 关状态行，bit1 关频谱；短按向前、长按向后，自然回绕。
    output reg  [1:0] disp_mode,
    output reg  auto_toggle_req,
    output reg  period_set_req,
    output reg  [5:0] period_new,
    output reg  [5:0] period_cur,
    output reg  src_key,
    output wire [7:0] period_bcd,
    output reg  [7:0] countdown_bcd,
    output reg  countdown_ok
);
    localparam [2:0] LAST=ITEMS-1;
    // 周期档位表：上一档/下一档（饱和）。
    function [5:0] step_up; input [5:0] p;
        begin step_up=p<6'd2 ? 6'd2 : p<6'd3 ? 6'd3 : p<6'd5 ? 6'd5 : p<6'd10 ? 6'd10 : p<6'd15 ? 6'd15 :
                      p<6'd20 ? 6'd20 : p<6'd30 ? 6'd30 : p<6'd45 ? 6'd45 : 6'd60; end endfunction
    function [5:0] step_down; input [5:0] p;
        begin step_down=p>6'd45 ? 6'd45 : p>6'd30 ? 6'd30 : p>6'd20 ? 6'd20 : p>6'd15 ? 6'd15 : p>6'd10 ? 6'd10 :
                        p>6'd5 ? 6'd5 : p>6'd3 ? 6'd3 : p>6'd2 ? 6'd2 : 6'd1; end endfunction
    wire up=menu_open && enable && k4_short;
    wire down=menu_open && enable && k4_long;
    wire [5:0] p_up=step_up(period_cur);
    wire [5:0] p_dn=step_down(period_cur);
    always @(posedge clk) begin
        if(logic_rst) begin
            menu_open<=1'b0;item<=3'd0;bright_idx<=4'd15;vol_lvl<=5'd16;auto_toggle_req<=1'b0;period_set_req<=1'b0;period_new<=6'd5;
            period_cur<=6'd5;src_key<=1'b0;disp_mode<=2'd0;
        end else begin
            auto_toggle_req<=1'b0;period_set_req<=1'b0;
            // 拨码周期在接管时生效一次（b3 cfg_apply）；KEY 改动前显示值跟随它。
            if(cfg_apply) period_cur<=cfg_period;
            if(k3_short) begin
                if(!menu_open) begin menu_open<=1'b1;item<=3'd0;end
                else if(item==LAST) menu_open<=1'b0;
                else item<=item+3'd1;
            end else if(up || down) begin
                case(item)
                    3'd0: if(up ? !auto_on : auto_on) auto_toggle_req<=1'b1;
                    3'd1: if((up ? p_up : p_dn)!=period_cur) begin
                              period_set_req<=1'b1;period_new<=up ? p_up : p_dn;period_cur<=up ? p_up : p_dn;src_key<=1'b1;
                          end
                    3'd2: disp_mode<=up ? disp_mode+2'd1 : disp_mode-2'd1;
                    3'd3: if(up ? bright_idx<4'd15 : bright_idx>4'd0)
                              bright_idx<=bright_idx+(up ? 4'd1 : 4'hf);
                    3'd4: if(up ? vol_lvl<5'd16 : vol_lvl>5'd0)
                              vol_lvl<=vol_lvl+(up ? 5'd1 : 5'h1f);
                    default: ;
                endcase
            end
        end
    end
    // 6位二进制转BCD，double-dabble 只校正个位。移入最后一位前最大31，十位<=3，
    // 所以不需十位加3。以移位和条件加法完成 0..63 的十进制转换。
    // 常量表和布尔译码用于减少菜单数据选择逻辑，串行毫秒换算保持固定调度。
    function [7:0] bcd6;
        input [5:0] value;
        reg [7:0] work_bcd;
        integer bit_index;
        begin
            work_bcd=8'd0;
            for(bit_index=5;bit_index>=0;bit_index=bit_index-1)begin
                if(work_bcd[3:0]>=4'd5)work_bcd[3:0]=work_bcd[3:0]+4'd3;
                work_bcd={work_bcd[6:0],value[bit_index]};
            end
            bcd6=work_bcd;
        end
    endfunction
    assign period_bcd=bcd6(period_cur);
    // 倒计时串行除法（ms→s，向上取整）。
    reg busy; reg [15:0] rem; reg [5:0] q;
    wire [5:0] sec=q+{5'd0,rem!=16'd0};
    // q 到 62 即停（余数异常大时只显示饱和值，不回绕）；正常 remain_ms≤60000，q≤60。
    // 1000=10'b1111101000：高6位非0，或低10位>=1000；显式前缀免去13位比较进位链。
    // q<62 时高 5 位不全为一，以此前缀条件限制串行换算计数。
    wire rem_ge1000=(|rem[15:10]) || ((&rem[9:5]) && (rem[4] || rem[3]));
    wire div_step=rem_ge1000 && !( &q[5:1]);
    always @(posedge clk) begin
        if(logic_rst) begin busy<=1'b0;countdown_ok<=1'b0;countdown_bcd<=8'd0;end
        else if(!busy) begin if(ms_tick) busy<=1'b1;end
        else if(!div_step) begin
            busy<=1'b0;countdown_ok<=1'b1;
            countdown_bcd<=bcd6(sec);
        end
    end
    always @(posedge clk) begin
        if(!busy) begin rem<=remain_ms;q<=6'd0;end
        else if(div_step) begin rem<=rem-16'd1000;q<=q+6'd1;end
    end
endmodule
`default_nettype wire
