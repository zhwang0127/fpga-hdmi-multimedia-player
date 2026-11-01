`timescale 1ns/1ps
`default_nettype none
// sys 域按键输入处理：两级同步，按 ms_tick 采样并消抖，输出按下、短按和长按事件。
// ACTIVE_LOW 选择按下电平；连续 DEBOUNCE_SAMPLES 次采样确认状态变化后更新 level_db。
// ms_tick 为 1ms 单周期使能；logic_rst 同步清除事件、消抖状态和计数器。
// 输入同步寄存器不复位并持续采样；复位后重新计满消抖样本数。
// LONG_MS=0 时仅输出 press；大于零时输出 short_rel 或 long_press，一次按键只产生其中一种。
// 长按计时从确认按下开始；确认释放与长按门限同拍时，优先输出 long_press。
// press、short_rel 和 long_press 均只保持一个 clk 周期。
module key_events #(
    parameter integer ACTIVE_LOW=1,
    parameter integer DEBOUNCE_SAMPLES=20,
    parameter integer LONG_MS=0
)(
    input wire clk,logic_rst,ms_tick,key_async,
    output reg press,short_rel,long_press,level_db
);
    function integer bits_for;
        input integer n; integer v;
        begin bits_for=1;v=n-1;while(v>1) begin bits_for=bits_for+1;v=v>>1;end end
    endfunction
    localparam integer DB_W=bits_for(DEBOUNCE_SAMPLES);
    localparam integer HOLD_W=bits_for(LONG_MS>0 ? LONG_MS : 1);
    localparam [DB_W-1:0] DB_LAST=DEBOUNCE_SAMPLES[DB_W-1:0]-1'b1;
    localparam [HOLD_W-1:0] HOLD_LAST=LONG_MS[HOLD_W-1:0]-1'b1;
    // 按键经两级同步进入本域；两级均禁止重定时，第二级使用 preserve 保留。
    // 首级允许工具映射到输入引脚寄存器，避免 keep 强制保留重复的 fabric 寄存器。
    (* allow_forward_retiming="false",
       allow_backward_retiming="false" *) reg key_s1;
    (* preserve="true", allow_forward_retiming="false",
       allow_backward_retiming="false" *) reg key_s2;
    reg [DB_W-1:0] different_samples;
    reg [HOLD_W-1:0] held_ms;
    reg long_sent;
    wire pressed=ACTIVE_LOW!=0 ? !key_s2 : key_s2;
    wire confirm_change=pressed!=level_db && different_samples==DB_LAST;
    wire long_due=LONG_MS>0 && level_db && !long_sent && held_ms==HOLD_LAST;
    always @(posedge clk) begin key_s1<=key_async;key_s2<=key_s1;end
    always @(posedge clk) begin
        if(logic_rst) begin
            press<=0;short_rel<=0;long_press<=0;level_db<=0;
            different_samples<=0;held_ms<=0;long_sent<=0;
        end else begin
            // 使能之间也清脉冲，事件只保持一个sys拍，而不是整整1ms。
            press<=0;short_rel<=0;long_press<=0;
            if(ms_tick) begin
                if(pressed==level_db) different_samples<=0;
                else if(confirm_change) begin
                    different_samples<=0;level_db<=pressed;
                    if(pressed) begin
                        if(LONG_MS==0) press<=1;
                        held_ms<=0;long_sent<=0;
                    end else if(LONG_MS>0 && !long_sent && !long_due) short_rel<=1;
                end else different_samples<=different_samples+1'b1;
                if(LONG_MS>0 && level_db && !long_sent) begin
                    if(long_due) begin long_press<=1;long_sent<=1;end
                    else held_ms<=held_ms+1'b1;
                end
            end
        end
    end
endmodule
`default_nettype wire
