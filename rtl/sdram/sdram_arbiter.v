`timescale 1ns/1ps
`default_nettype none

// SDRAM五级仲裁器：截止刷新>硬截止行事务>PCM缓存样本不足>普通行>后台加载。
// 纯组合：在eligible(请求存在且其长度不跨越任一适用截止窗口)中按固定
// 优先级选出唯一owner；数据通路BL8段进行中(burst_active)不发射新段——
// 实时请求可等当前段收尾，不允许抢占。窗口规则(方案§9.4"发突发前检查
// 其结束是否越过下一刷新或实时保留窗口")：刷新自身豁免；硬截止行/PCM
// 只受刷新窗口约束(它们本身占用实时保留窗口)；普通行/后台同时受刷新与
// 实时保留窗口约束。高优先级请求窗口不满足时等待，仍可选择能在窗口内完成的
// 低优先级请求。reserve_deadline=NO_DEADLINE哨兵时保留窗口恒不约束。
module sdram_arbiter #(
    parameter [15:0] NO_DEADLINE=16'hFFFF
)(
    input wire clk,arst,
    input wire req_refresh,req_line_hard,req_pcm,req_line_normal,req_background,
    input wire [15:0] len_refresh,len_line_hard,len_pcm,len_line_normal,len_background,
    input wire [15:0] refresh_deadline,   // 距下一刷新到期拍数(0=已到期)
    input wire [15:0] reserve_deadline,   // 距实时保留窗口开始的拍数(哨兵=无)
    input wire burst_active,              // 数据通路BL8段进行中：禁止新段
    output wire grant_valid,
    output wire [2:0] grant_owner         // 0=刷新 1=硬截止行 2=PCM 3=普通行 4=后台
);
    // 各级窗口可发性。
    wire hard_fits   = len_line_hard  <= refresh_deadline;
    wire pcm_fits    = len_pcm        <= refresh_deadline;
    wire normal_fits = len_line_normal<= refresh_deadline &&
                       len_line_normal<= reserve_deadline;
    wire bg_fits     = len_background <= refresh_deadline &&
                       len_background <= reserve_deadline;
    wire [4:0] eligible={req_background&&bg_fits, req_line_normal&&normal_fits,
                        req_pcm&&pcm_fits, req_line_hard&&hard_fits, req_refresh};
    // 固定优先级：位0(刷新)最高。
    reg [2:0] owner;
    always @(*) begin
        if(eligible[0]) owner=3'd0;
        else if(eligible[1]) owner=3'd1;
        else if(eligible[2]) owner=3'd2;
        else if(eligible[3]) owner=3'd3;
        else if(eligible[4]) owner=3'd4;
        else owner=3'd0;
    end
    assign grant_valid=!burst_active && (|eligible);
    assign grant_owner=owner;
    // clk仅为将来时序化预留；当前纯组合，复位不影响组合输出语义。
    wire unused_clk=clk;wire unused_arst=arst;wire unused_len_refresh=len_refresh;
endmodule
`default_nettype wire
