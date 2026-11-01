`timescale 1ns/1ps
`default_nettype none
// 擦拭旧图写保护，运行在 sys 域。
// 显示成功响应把当前镜像切到新槽，但新 epoch 的行包仍可能读取旧槽。
// 本模块锁存上一显示槽，并从 writable_in 中屏蔽它，防止装载器覆盖擦拭数据。
// 保护持续到下一次成功响应：此时 display_read_owner 已确认上一 epoch 的行读取全部结束。
// 这种保守释放复用已有响应，不需要将 pix 域的擦拭结束事件再跨域。
// ack_fire 仅在 sys 域接受响应时置高一个周期；ack_ok 表示成功，ack_slot 为新显示槽。
// 首次响应只建立当前槽；后续成功响应同时更新当前槽和被保护的旧槽。
// writable_out 只清除可写位，不会把原本禁止写入的图像槽改为可写。
// 当前装载策略每槽仅装一次；本保护将显示引用的生命周期落实到写入许可。
// 接口仅支持单个在途换图；扩展并发描述符时需按 epoch 分别维护引用。
module wipe_old_guard(
    input  wire       clk,
    input  wire       rst,
    input  wire       ack_fire,
    input  wire       ack_ok,
    input  wire [1:0] ack_slot,
    input  wire [3:0] writable_in,
    output wire [3:0] writable_out,
    output wire [3:0] hold_mask        // 诊断：当前额外屏蔽的槽
);
    reg       cur_valid;   // 已有成功响应的显示槽（与 slot_owner mirror 同沿同值）
    reg [1:0] cur_slot;
    reg       prev_valid;  // 上一张显示图（擦拭旧图）仍受保护
    reg [1:0] prev_slot;
    assign hold_mask    = prev_valid ? (4'b0001 << prev_slot) : 4'd0;
    assign writable_out = writable_in & ~hold_mask;
    always @(posedge clk) begin
        if(rst) begin
            cur_valid<=1'b0;cur_slot<=2'd0;prev_valid<=1'b0;prev_slot<=2'd0;
        end else if(ack_fire && ack_ok) begin
            // 新响应：旧的“上一张”释放（对它的最后一次行读取已在本响应前结束），当前图降为“上一张”，新图成为当前。
            prev_valid<=cur_valid;prev_slot<=cur_slot;
            cur_valid<=1'b1;cur_slot<=ack_slot;
        end
    end
endmodule
`default_nettype wire
