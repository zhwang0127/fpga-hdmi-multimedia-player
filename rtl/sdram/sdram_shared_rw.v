`timescale 1ns/1ps
`default_nettype none
// mem域共享控制器。显示段优先，后台限定单BL8；归属锁定直到真实PRE/done。
// reserve_cycles由上层提供距下一显示保留窗的拍数，FFFF代表无保留窗。
// 下层控制器负责实际刷新，本层依据 reserve_cycles 检查新段是否可进入保留窗口。
`ifndef SDRAM_PHY_RD_EXTRA_DEFAULT
`define SDRAM_PHY_RD_EXTRA_DEFAULT 0
`endif
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sdram_shared_rw #(
    parameter ENABLE_PCM=0,
    parameter [31:0] HARD_DISPLAY_CYCLES=1980,
    parameter POWER_WAIT_CYCLES=25000,REFRESH_COUNT=8,
    // 刷新参数以 mem 时钟周期计数；默认 937 拍对应 125MHz 下约 7.5µs。
    // 整机使用 100MHz，显式传入 REFRESH_INTERVAL=750、REFRESH_GRACE=750。
    parameter REFRESH_INTERVAL=937,REFRESH_GRACE=937,
    // 最长开行拍数（默认 10000=80µs@125MHz，与下级 sdram_stream_controller 默认相同，
    //   不传时行为逐位不变）；产品顶层传 8000（=80µs@100MHz）。
    parameter T_RAS_MAX=10000,
    parameter [15:0] WRITE_GUARD_CYCLES=16'd64,
    parameter PHY_RD_EXTRA=`SDRAM_PHY_RD_EXTRA_DEFAULT // 透传给控制器，见 sdram_stream_controller
)(
    input wire clk,arst,input wire [15:0] reserve_cycles,
    input wire line_valid,output wire line_ready,input wire [20:0] line_addr,
    input wire [3:0] line_count,input wire [31:0] line_budget,input wire line_cancel,
    output wire line_data_valid,input wire line_data_ready,output wire line_done,
    // PCM 客户端每次只允许一个 BL8。FIFO 已缓存样本不足时可优先于普通行，硬截止行始终优先。
    // 预算<=HARD_DISPLAY_CYCLES的行视为硬截止，保守预留完整行剩余工作量。
    input wire pcm_valid,pcm_urgent,output wire pcm_ready,input wire [20:0] pcm_addr,
    input wire [31:0] pcm_budget,input wire pcm_cancel,
    output wire pcm_data_valid,input wire pcm_data_ready,output wire pcm_done,
    input wire load_valid,output wire load_ready,input wire [20:0] load_addr,
    input wire [3:0] load_count,input wire [31:0] load_budget,input wire load_cancel,
    input wire load_data_valid,input wire [31:0] load_data,output wire load_beat,
    output wire load_done,output wire load_done_was_write,
    input wire [31:0] rd_data_in,
    output wire wr_data_valid,output wire [31:0] wr_data,
    output wire busy,output wire owner_write,
    output wire sd_cs_n,sd_ras_n,sd_cas_n,sd_we_n,sd_cke,
    output wire [10:0] sd_addr,output wire [1:0] sd_ba
);
    wire rst_logic;
    wire rst;reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // rst 由 reset_sync 以 arst 异步置位，!arst 冗余；去掉以免 rst_mem 释放链进入命令组合路径。
    wire enabled=!rst_logic;
    reg active,is_write,is_pcm;
    // 接受写请求时置位，真实done时清零，写数据和写拍消费共用这份条件。
    // 单独寄存是为了缩短PHY使能路径；取消后仍要供完已经接受的整段。
    (* keep = "true" *) reg write_active_q;
    // 与active同拍的条件副本，专门分担done/读数据有效的扇出。
    // keep防止综合又合并回去；两份寄存器必须始终保持等值。
    (* keep = "true" *) reg active_done_q;
    // 接受读请求时保存读请求的归属，减少 rd_ready 到控制器的译码层级。
    // done优先于launch，与active清零同沿；只选择原客户端ready，不提前接收数据。
    (* keep = "true" *) reg pcm_read_active_q,line_read_active_q;
    wire load_fits=load_count==4'd1 && reserve_cycles>=(ENABLE_PCM ? WRITE_GUARD_CYCLES+16'd16 : WRITE_GUARD_CYCLES);
    // 新仲裁选择及地址/计数/预算先寄存，再交给原控制器；不将urgent/已缓存样本数
    // 串入write_active_q→PHY写使能路径。grant从选定到真正ready保持不变，
    // 外部客户端只在该真正接受沿看到ready，不能把预选当作接受。
    localparam G_LINE=0,G_PCM=1,G_WRITE=2;
    reg grant_valid;reg [1:0] grant;
    reg [20:0] grant_addr;reg [3:0] grant_count;reg [31:0] grant_budget;
    // clamp先对每个源独立计算，优先级选择只走末级mux，不串入32位进位。
    // 数学恒等式f(sel?a:b)=sel?f(a):f(b)，接受沿和保守扣16拍完全不变。
    wire [31:0] reduced_line_budget=line_budget>32'd16 ? line_budget-32'd16 : 32'd0;
    wire [31:0] reduced_pcm_budget=pcm_budget>32'd16 ? pcm_budget-32'd16 : 32'd0;
    wire [31:0] reduced_load_budget=load_budget>32'd16 ? load_budget-32'd16 : 32'd0;
    wire line_hard=line_valid && line_budget<=HARD_DISPLAY_CYCLES;
    // reserve=0表示显示硬保留窗已经开始；即使 PCM 缓存样本不足也不得占用此窗口。
    // 与写口一致为预选到接受扣16拍余量。FFFF表示无硬保留窗。
    wire pcm_fits=reserve_cycles>=16'd80;
    wire choose_pcm=pcm_valid && pcm_fits && (!line_valid || (pcm_urgent && !line_hard));
    wire [1:0] next_grant=choose_pcm ? G_PCM : line_valid ? G_LINE : G_WRITE;
    wire pending_any=line_valid || (pcm_valid && pcm_fits) || (load_valid && load_fits);
    wire ctl_ready,ctl_done,ctl_beat,ctl_data_valid;
    wire launch,choose_valid;
    // 行段地址/计数本来已在display_row_bundle寄存。硬截止条件提前一拍
    // 计算，硬行可直通原读字段；不为每个32字段再叠加一个字段流水周期。
    reg hard_q;
    always @(posedge clk or posedge rst) if(rst) hard_q<=1;else hard_q<=line_budget<=HARD_DISPLAY_CYCLES;
    wire fast_line=ENABLE_PCM && hard_q && line_valid && !grant_valid;
    always @(posedge clk or posedge rst) begin
        if(rst) begin grant_valid<=0;grant<=G_LINE;grant_addr<=0;grant_count<=0;grant_budget<=0;end
        else if(ENABLE_PCM) begin
            if(!active && !grant_valid && pending_any && ctl_ready && !fast_line) begin
                grant_valid<=1;grant<=next_grant;
                grant_addr<=choose_pcm ? pcm_addr : line_valid ? line_addr : load_addr;
                grant_count<=choose_pcm ? 4'd1 : line_valid ? line_count : load_count;
                // 预选至接受最多包含一次已到期刷新；保守扣16拍，不增加客户端预算。
                // 只有ctl_ready时预选，不会把上电初始化等待算进在途请求。
                grant_budget<=choose_pcm ? reduced_pcm_budget : line_valid ? reduced_line_budget : reduced_load_budget;
            end
            if(launch && ctl_ready) grant_valid<=0;
        end
    end
    wire choose_write=ENABLE_PCM ? (grant_valid && grant==G_WRITE) : (!line_valid && load_valid && load_fits);
    assign choose_valid=ENABLE_PCM ? (grant_valid || fast_line) : (line_valid || (load_valid && load_fits));
    // 关闭 PCM 客户端时只在显示读和后台写之间选择，请求握手及段所有权规则相同。
    wire candidate_write=ENABLE_PCM ? grant_valid && grant==G_WRITE : !line_valid;
    wire [20:0] candidate_addr=ENABLE_PCM ? (grant_valid ? grant_addr : line_addr) : line_valid ? line_addr : load_addr;
    wire [3:0] candidate_count=ENABLE_PCM ? (grant_valid ? grant_count : line_count) : line_valid ? line_count : load_count;
    wire [31:0] candidate_budget=ENABLE_PCM ? (grant_valid ? grant_budget : line_budget) : line_valid ? line_budget : load_budget;

    assign launch=enabled && !active && choose_valid;
    assign line_ready=enabled && !active && ctl_ready && (ENABLE_PCM ? (fast_line || (grant_valid && grant==G_LINE)) : !choose_write);
    assign pcm_ready=ENABLE_PCM && enabled && !active && ctl_ready && grant_valid && grant==G_PCM;
    assign load_ready=enabled && !active && ctl_ready && choose_write && (!ENABLE_PCM || grant_valid);
    assign line_done=enabled && active_done_q && !is_write && !is_pcm && ctl_done;
    assign load_done=enabled && active_done_q && is_write && ctl_done;
    assign load_done_was_write=load_done;
    assign load_beat=enabled && write_active_q && ctl_beat;
    assign line_data_valid=enabled && active_done_q && !is_write && !is_pcm && ctl_data_valid;
    assign pcm_done=ENABLE_PCM && enabled && active_done_q && is_pcm && ctl_done;
    assign pcm_data_valid=ENABLE_PCM && enabled && active_done_q && is_pcm && ctl_data_valid;
    assign wr_data_valid=enabled && write_active_q && load_data_valid;
    assign wr_data=load_data;
    assign busy=enabled && active;
    assign owner_write=is_write;
    sdram_stream_controller #(.POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),.REFRESH_COUNT(REFRESH_COUNT),
        .REFRESH_INTERVAL(REFRESH_INTERVAL),.REFRESH_GRACE(REFRESH_GRACE),.T_RAS_MAX(T_RAS_MAX),.PHY_RD_EXTRA(PHY_RD_EXTRA)) u_memory(
        .clk(clk),.arst(arst),.seg_valid(launch),.seg_ready(ctl_ready),.seg_is_write(candidate_write),
        .seg_addr(candidate_addr),
        .seg_bl8_count(candidate_count),
        .seg_budget_cycles(candidate_budget),
        .seg_cancel(active && (is_write ? load_cancel : is_pcm ? pcm_cancel : line_cancel)),
        .wr_data_valid(wr_data_valid),.wr_data(wr_data),.wr_beat(ctl_beat),
        .rd_data_valid(ctl_data_valid),.rd_data_ready((pcm_read_active_q && pcm_data_ready) || (line_read_active_q && line_data_ready)),
        .rd_data_in(rd_data_in),.seg_done(ctl_done),.seg_done_was_write(),
        .sd_cs_n(sd_cs_n),.sd_ras_n(sd_ras_n),.sd_cas_n(sd_cas_n),.sd_we_n(sd_we_n),
        .sd_cke(sd_cke),.sd_addr(sd_addr),.sd_ba(sd_ba));
    always @(posedge clk or posedge rst) begin
        if(rst) begin active<=0;is_write<=0;is_pcm<=0;write_active_q<=0;active_done_q<=0;end
        else begin
            if(launch && ctl_ready) begin active<=1;is_pcm<=ENABLE_PCM && grant_valid && grant==G_PCM;is_write<=choose_write;write_active_q<=choose_write;active_done_q<=1'b1;end
            // 同拍结束与发起条件重叠时，done 优先更新所有权状态。
            // 正常协议要求段结束后才释放归属，防止完成响应被新请求覆盖。
            if(active && ctl_done) begin active<=0;write_active_q<=0;end
            if(active_done_q && ctl_done) active_done_q<=1'b0;
        end
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin pcm_read_active_q<=0;line_read_active_q<=0;end
        else begin
            if(launch && ctl_ready) begin
                pcm_read_active_q<=!choose_write && ENABLE_PCM && grant_valid && grant==G_PCM;
                line_read_active_q<=!choose_write && !(ENABLE_PCM && grant_valid && grant==G_PCM);
            end
            if(active_done_q && ctl_done) begin pcm_read_active_q<=0;line_read_active_q<=0;end
        end
    end
`ifdef FORMAL
    // 字段预选必须与锁定后的段所有者一致。
    always @* assert(write_active_q==(active && is_write));
    always @* assert(active_done_q==active);
    always @* assert(pcm_read_active_q==(active_done_q && !is_write && is_pcm));
    always @* assert(line_read_active_q==(active_done_q && !is_write && !is_pcm));
`endif
endmodule
`default_nettype wire
