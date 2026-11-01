`timescale 1ns/1ps
`default_nettype none

// 连续锁定确认；失锁异步隔离，恢复先三级同步，再计满 QUAL_CYCLES。
// 不生成 PLL/时钟，不检测「LOCK 仍高但时钟停振」；此故障需独立时钟监测。
module pll_lock_guard #(
    parameter LOCK_COUNT = 1,
    parameter QUAL_CYCLES = 1000000
) (
    input wire sys_clk,
    input wire arst,
    input wire [LOCK_COUNT-1:0] locked_async,
    output wire reset_request,
    output wire qualified
);
    function integer count_width;
        input integer cycles;
        integer value;
        begin
            value = cycles-1; count_width = 0;
            while (value > 0) begin count_width = count_width+1; value = value >> 1; end
            if (count_width == 0) count_width = 1;
        end
    endfunction
    localparam COUNT_WIDTH = count_width(QUAL_CYCLES);
    localparam [COUNT_WIDTH-1:0] LAST_COUNT = QUAL_CYCLES-1;
    wire lock_fault = arst | !(&locked_async);
    wire qualification_reset;
    // 捕捉跨两个系统沿之间的短失锁，并拉长到三个系统沿同步释放。
    reset_sync u_fault_release(.clk(sys_clk), .arst(lock_fault), .rst(qualification_reset));
    reg [COUNT_WIDTH-1:0] count;
    reg qualified_reg;
    always @(posedge sys_clk or posedge qualification_reset) begin
        if (qualification_reset) begin count <= 0; qualified_reg <= 0; end
        else if (!qualified_reg) begin
            if (count == LAST_COUNT) qualified_reg <= 1;
            else count <= count+1'b1;
        end
    end
    // 原始失锁只参与异步置位/隔离；释放受三级同步与完整连续锁定计时窗口控制。
    assign reset_request = lock_fault | qualification_reset | !qualified_reg;
    assign qualified = !reset_request;
endmodule

`default_nettype wire
