`timescale 1ns/1ps
`default_nettype none

// 稳定总线mailbox：req/ack翻转位双级同步，数据整组锁存，不逐位同步。
// ack表示目标寄存器已锁存；目标消费和帧提交另行确认，不能混用。
// DATA_WIDTH>=1；唯一共同arst覆盖两端，禁止单端复位。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module cdc_mailbox #(
    parameter DATA_WIDTH = 256
) (
    input wire arst,
    input wire src_clk,
    input wire src_valid,
    output wire src_ready,
    input wire [DATA_WIDTH-1:0] src_data,
    output wire src_active,
    output wire src_busy,
    output reg src_done,
    input wire dst_clk,
    output wire dst_valid,
    input wire dst_ready,
    output wire [DATA_WIDTH-1:0] dst_data,
    output wire dst_active
);
    wire src_rst_logic;
    wire dst_rst_logic;
    wire src_rst, dst_rst;
    reset_sync u_src_reset(.clk(src_clk), .arst(arst), .rst(src_rst),.logic_rst(src_rst_logic));
    reset_sync u_dst_reset(.clk(dst_clk), .arst(arst), .rst(dst_rst),.logic_rst(dst_rst_logic));
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg req_toggle, ack_toggle;
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg req_d1, req_d2, ack_s1, ack_s2;
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg src_up_d1, src_up_d2, dst_up_s1, dst_up_s2;
    reg src_up, dst_up, busy_reg, valid_reg;
    // 数据寄存器禁止重定时；不加preserve，避免TD将IO迁移后的无用副本强制保留。
    (* allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg [DATA_WIDTH-1:0] held_bus, received_bus;

    // 新的两端up往返确认后才接受消息；停钟端未释放不能使用旧确认。
    // 业务门控使用本端同步后的 src_rst/dst_rst，arst 仅用于同步器的异步置位。
    // reset_sync 为异步置位：arst 上升时本端 rst 同时（经一个异步置位 clk-to-q）置 1，且在 arst
    // 释放后还保持 3 个本端时钟沿，故 !rst 已覆盖 !arst；arst 只进入 reset_sync 的异步置位输入。
    assign src_active = src_up && dst_up_s2 && !src_rst_logic;
    assign dst_active = dst_up && !dst_rst_logic;
    assign src_ready = src_active && !busy_reg;
    assign src_busy = !src_ready; // 含上电后的就绪确认，不仅代表在途消息。
    assign dst_valid = dst_active && valid_reg;
    assign dst_data = received_bus;
    // fire才代表这拍真的交接；只有valid而没有ready时，发送方要继续保持数据。
    wire src_fire = src_valid && src_ready;
    wire dst_fire = dst_valid && dst_ready;
    // 目标只有一个寄存器槽；可在消费旧消息的同沿锁存新的在途消息。
    // ack只说明目标寄存器装好了，不代表目标业务已经处理完这份消息。
    wire capture = dst_active && (req_d2 != ack_toggle) && (!valid_reg || dst_ready);

    // 数据寄存器不复位。源端只在接受握手时修改，直到锁存确认回到源域。
    // 源保持总线到目标锁存寄存器的跨域最大延迟必须在板级单独约束/检查。
    always @(posedge src_clk)
        if (src_fire) held_bus <= src_data;
    always @(posedge dst_clk)
        if (capture) received_bus <= held_bus;

    always @(posedge src_clk or posedge src_rst) begin
        if (src_rst) begin
            req_toggle <= 0; busy_reg <= 0; src_done <= 0;
            ack_s1 <= 0; ack_s2 <= 0;
            src_up <= 0; dst_up_s1 <= 0; dst_up_s2 <= 0;
        end else begin
            src_up <= 1;
            dst_up_s1 <= dst_up; dst_up_s2 <= dst_up_s1;
            ack_s1 <= ack_toggle; ack_s2 <= ack_s1;
            src_done <= 0;
            if (src_fire) begin
                req_toggle <= !req_toggle; busy_reg <= 1;
            end else if (src_active && busy_reg && ack_s2 == req_toggle) begin
                busy_reg <= 0; src_done <= 1;
            end
        end
    end
    always @(posedge dst_clk or posedge dst_rst) begin
        if (dst_rst) begin
            req_d1 <= 0; req_d2 <= 0; ack_toggle <= 0; valid_reg <= 0;
            src_up_d1 <= 0; src_up_d2 <= 0; dst_up <= 0;
        end else begin
            src_up_d1 <= src_up; src_up_d2 <= src_up_d1;
            dst_up <= src_up_d2;
            req_d1 <= req_toggle; req_d2 <= req_d1;
            if (capture) begin
                valid_reg <= 1; ack_toggle <= req_d2;
            end else if (dst_fire)
                valid_reg <= 0;
        end
    end
endmodule

`default_nettype wire
