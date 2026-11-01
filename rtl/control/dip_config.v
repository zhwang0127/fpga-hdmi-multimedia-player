`timescale 1ns/1ps
`default_nettype none
// B3一次性启动周期配置，仅接SW1/SW2；SW3/SW4不参与业务或同步器。
// 原理图SW1/SW2为双掷到3V3/GND，经120R串联到FPGA；
// 高电平按逻辑1处理，不猜面板上哪一物理方向叫ON。{SW2,SW1}=00/01/10/11为1/5/10/30秒。
// 两位整组同步后连续20个1ms样本一致才锁存，防止分别锁到不同时刻。
// ready后不再跟随拨码，只有共同logic_rst才重读；不可用由USE_DIP=0显式选择，
// USE_DIP=0 时使用手动模式及 5 秒周期，不读取拨码。
module dip_config #(
    parameter integer USE_DIP=1,
    parameter integer SAMPLES=20
)(
    input wire clk,logic_rst,ms_tick,
    input wire [1:0] dip_async,
    output reg ready,
    output reg [5:0] period_s
);
    function integer bits_for;
        input integer n;integer v;
        begin bits_for=1;v=n-1;while(v>1)begin bits_for=bits_for+1;v=v>>1;end end
    endfunction
    localparam integer CW=bits_for(SAMPLES);
    localparam [CW-1:0] LAST=SAMPLES[CW-1:0]-1'b1;
    // 拨码经两级同步进入本域，两级均禁止重定时。
    // 首级允许映射到输入引脚寄存器，第二级保留独立同步级。
    (* allow_forward_retiming="false",allow_backward_retiming="false" *)
    reg [1:0] dip_s1;
    (* preserve="true",allow_forward_retiming="false",allow_backward_retiming="false" *)
    reg [1:0] dip_s2;
    reg [1:0] candidate;
    reg [CW-1:0] same_samples;
    always @(posedge clk)begin dip_s1<=dip_async;dip_s2<=dip_s1;end
    always @(posedge clk)begin
        if(logic_rst)begin ready<=0;period_s<=6'd5;candidate<=0;same_samples<=0;end
        else if(!ready)begin
            if(USE_DIP==0)begin ready<=1;period_s<=6'd5;end
            else if(ms_tick)begin
                if(dip_s2!=candidate)begin
                    candidate<=dip_s2;
                    // 当前变化样本也算第1个，不额外等第21次。
                    if(SAMPLES==1)begin
                        ready<=1;
                        case(dip_s2[1:0])0:period_s<=1;1:period_s<=5;2:period_s<=10;3:period_s<=30;endcase
                    end else same_samples<={{(CW-1){1'b0}},1'b1};
                end else if(same_samples==LAST)begin
                    ready<=1;
                    case(candidate[1:0])0:period_s<=1;1:period_s<=5;2:period_s<=10;3:period_s<=30;endcase
                end else same_samples<=same_samples+1'b1;
            end
        end
    end
endmodule
`default_nettype wire
