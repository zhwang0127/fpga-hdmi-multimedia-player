`timescale 1ns/1ps
`default_nettype none

// SDRAM 初始化序列：上电等待、PRECHARGE ALL、重复 AUTO REFRESH、设置模式寄存器及等待 tMRD。
// 模式为 CL3、顺序 BL8；完成后置 init_done，并通过 oe 将命令驱动权交给业务控制器。
// 命令只在单拍 cs_n=0 时发出，其余拍 DESELECT，cke 保持为一。
// 延时参数以控制器时钟周期计数；默认值覆盖 125MHz 下的保守时序，整机 100MHz 时仍满足等待时间。
module sdram_init_seq #(
    parameter POWER_WAIT_CYCLES=25000,   // ≥200µs @125MHz
    parameter REFRESH_COUNT=8,           // 至少8次AUTO REFRESH
    parameter T_RP_CYCLES=3,             // 预充恢复
    parameter T_RFC_CYCLES=8,            // 刷新间隔(tRC)
    parameter T_MRD_CYCLES=2             // MRS到下一命令
)(
    input wire clk,arst,
    output wire init_done,init_busy,
    output wire cmd_oe,                  // 命令驱动有效(初始化期间为1)
    output wire cmd_cs_n,cmd_ras_n,cmd_cas_n,cmd_we_n,cmd_cke,
    output wire [10:0] cmd_addr,
    output wire [1:0] cmd_ba
);
    wire rst_logic;
    localparam IDLE=0,POWER=1,PRE=2,RP=3,REF=4,RFC=5,MRS=6,MRD=7,DONE=8;
    reg [3:0] state;
    reg [15:0] power_counter;
    reg [3:0] refresh_left;
    reg [3:0] timing_counter;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // rst 由 reset_sync 以 arst 异步置位，!arst 冗余；去掉以免 rst_mem 释放链进入命令组合路径。
    wire enabled=!rst_logic;
    // 命令编码(SDR)：{ras,cas,we} 000=MRS 001=REF 010=PRE 011=ACT
    // 100=WRITE 101=READ 110/111=禁止；DESELECT由cs_n=1表达。
    // MRS模式寄存器值：A[2:0]=011(BL8) A[3]=0(顺序) A[6:4]=011(CL3) 其余0。
    localparam [10:0] MRS_VALUE=11'h033;
    // 单拍命令发射：状态进入命令态的首拍发命令，随后进入对应等待态。
    wire issue_pre=(state==PRE);
    wire issue_ref=(state==REF);
    wire issue_mrs=(state==MRS);
    assign cmd_cs_n=!(issue_pre || issue_ref || issue_mrs);
    assign cmd_ras_n=!(issue_pre || issue_ref || issue_mrs);
    assign cmd_cas_n=issue_mrs ? 1'b0 : (issue_ref ? 1'b0 : 1'b1);
    assign cmd_we_n=(issue_pre || issue_mrs) ? 1'b0 : 1'b1;
    assign cmd_cke=1'b1;
    // 初始化期间持有命令总线驱动权；done后交还控制器(oe=0)。
    assign cmd_oe=enabled && state!=DONE;
    assign cmd_addr=issue_pre ? 11'h400 :      // A10=1: ALL banks
                    issue_mrs ? MRS_VALUE : 11'h0;
    assign cmd_ba=2'b00;
    assign init_done=enabled && state==DONE;
    assign init_busy=enabled && state!=IDLE && state!=DONE;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;power_counter<=0;refresh_left<=0;timing_counter<=0;
        end else begin
            case(state)
                IDLE:begin
                    power_counter<=0;refresh_left<=REFRESH_COUNT[3:0];
                    state<=POWER;
                end
                POWER:begin
                    // 上电等待：不向器件发任何命令(cs_n=1)，cke=1。
                    if(power_counter==POWER_WAIT_CYCLES-1) begin
                        state<=PRE;timing_counter<=0;
                    end else power_counter<=power_counter+16'd1;
                end
                PRE:begin
                    // PRECHARGE ALL已在本拍发出；等待tRP。
                    state<=RP;timing_counter<=0;
                end
                RP:begin
                    if(timing_counter==T_RP_CYCLES-1) begin
                        state<=REF;timing_counter<=0;
                    end else timing_counter<=timing_counter+4'd1;
                end
                REF:begin
                    // AUTO REFRESH本拍发出；等待tRFC。
                    state<=RFC;timing_counter<=0;
                end
                RFC:begin
                    if(timing_counter==T_RFC_CYCLES-1) begin
                        timing_counter<=0;
                        if(refresh_left==4'd1) state<=MRS;
                        else begin refresh_left<=refresh_left-4'd1;state<=REF;end
                    end else timing_counter<=timing_counter+4'd1;
                end
                MRS:begin
                    // 模式寄存器已加载；等待tMRD后初始化完成。
                    state<=MRD;timing_counter<=0;
                end
                MRD:begin
                    if(timing_counter==T_MRD_CYCLES-1) state<=DONE;
                    else timing_counter<=timing_counter+4'd1;
                end
                DONE:state<=DONE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
