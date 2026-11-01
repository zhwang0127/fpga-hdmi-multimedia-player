`timescale 1ns/1ps
`default_nettype none

// SDRAM刷新调度器：初始化完成后按INTERVAL_CYCLES周期产生刷新需求。
// due置位后保持到真实服务沿(refresh_service)；服务才重载间隔计数。
// due期间继续计时的宽限不超过GRACE_CYCLES，超期拉高overdue并保持：
// 仲裁器据此不得再发起跨越下一刷新窗口的长事务(方案§9.2"累计延后
// 不可突破间隔上界")。相邻两次真实服务的拍距因此被约束在
// INTERVAL+GRACE之内。初始化未完成或复位期间不产生任何需求。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sdram_refresh_sched #(
    parameter INTERVAL_CYCLES=937,   // ≤7.5µs @125MHz
    parameter GRACE_CYCLES=937       // due后的服务宽限(一个间隔)
)(
    input wire clk,arst,
    input wire init_done,
    output wire refresh_due,
    output wire refresh_overdue,
    input wire refresh_service
);
    wire rst_logic;
    reg due,overdue;
    reg [10:0] interval_counter;
    reg [10:0] grace_counter;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // rst 由 reset_sync 以 arst 异步置位，!arst 冗余；去掉以免 rst_mem 释放链进入命令组合路径。
    wire enabled=!rst_logic;
    wire counting=enabled && init_done;
    wire interval_expired=counting && !due && interval_counter==INTERVAL_CYCLES[10:0]-11'd1;
    wire grace_expired=counting && due && grace_counter==GRACE_CYCLES[10:0]-11'd1;
    assign refresh_due=enabled && due;
    assign refresh_overdue=enabled && overdue;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            due<=0;overdue<=0;interval_counter<=0;grace_counter<=0;
        end else begin
            if(refresh_service && due) begin
                // 真实服务沿：清需求与超期，重载间隔计数重新计时。
                due<=0;overdue<=0;interval_counter<=0;grace_counter<=0;
            end else begin
                if(interval_expired) begin
                    due<=1;grace_counter<=0;
                end
                if(grace_expired) overdue<=1;
                // 间隔计数只在未到期时前进；due期间宽限计数前进。
                if(counting && !due) begin
                    if(interval_counter!=INTERVAL_CYCLES[10:0]-11'd1)
                        interval_counter<=interval_counter+11'd1;
                end
                if(counting && due) begin
                    if(grace_counter!=GRACE_CYCLES[10:0]-11'd1)
                        grace_counter<=grace_counter+11'd1;
                end
            end
        end
    end
endmodule
`default_nettype wire
