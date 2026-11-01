`timescale 1ns/1ps
`default_nettype none

// 单个目标时钟域的复位同步器：高有效异步置位，三级上升沿同步后下降沿释放。
// arst 必须来自经过设计的复位源；PLL 锁定确认和跨域 flush 不由本模块完成。
// 上电时 arst 必须有效，以初始化同步链和各域的复位状态。
module reset_sync (
    input  wire clk,   // 被复位逻辑自身的时钟；每个时钟域单独实例化。
    input  wire arst,  // 高有效异步复位请求，物理脉宽须满足器件要求。
    output wire rst,   // 下降沿释放：仅作上升沿寄存器的异步SR或下级复位源。
    output wire logic_rst // 正沿第三级：组合业务条件/上升沿同步复位；禁止取前两级。
);
    // 三级同步寄存器使用保留和禁止重定时属性，以维持异步复位的同步释放结构。
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg [2:0] release_pipe;

    always @(posedge clk or posedge arst) begin
        if (arst)
            release_pipe <= 3'b111;
        else
            // 第三个上升沿后同步链清零，再由下降沿尾级撤销rst，给下一上升沿留下半周期。
            release_pipe <= {release_pipe[1:0], 1'b0};
    end

    // rst最后在下降沿释放，离下一次上升沿还有半周期，留出复位撤销余量。
    // 尾级只看第三级，别改接前两级；那两级还承担亚稳态隔离。
    // 停钟时仍靠异步置位拉高复位，不能换成只在时钟沿生效的写法。
    (* preserve = "true", allow_forward_retiming = "false",
       allow_backward_retiming = "false" *) reg release_tail;
    always @(negedge clk or posedge release_pipe[2]) begin
        if(release_pipe[2]) release_tail<=1'b1;
        else release_tail<=1'b0;
    end
    assign rst = release_tail;
    // 两路均经完整三级同步且异步置位。logic_rst早半拍撤销，但业务正沿采样值相同；
    // 不能用rst做长组合门控，否则负沿发起只剩半周期。logic_rst也不能替代异步SR，
    // 其同沿撤销会重新引入本域removal风险。实际 CQ 延迟和复位脉宽仍须满足器件时序要求。
    assign logic_rst = release_pipe[2];
endmodule

`default_nettype wire
