`timescale 1ns/1ps
`default_nettype none
// sys 域播放控制接线层：同步按键事件、启动拨码配置及播放描述符。
// 首图成功响应后切换 handoff，由播放控制模块接管描述符；启动与播放请求必须互斥。
// rst/logic_rst 复用顶层 u_rst_sys，全部业务控制使用本域复位。
// SW1/SW2 选择轮播周期，上电固定手动，K2 切换自动模式。
// 拨码配置与首图响应到达顺序不限，配置就绪后 cfg_apply 只生效一次。
// EPOCH_W 默认 32 位、整机 8 位；连接同宽，一次仅允许一个在途换图请求。
module b3_control #(
    parameter integer EPOCH_W=32,
    parameter integer MS_CYCLES=50000,
    parameter integer USE_DIP=1,
    parameter integer DEBOUNCE_SAMPLES=20,
    // OSD_EN=0 时裁去 K3/K4 和菜单，播放控制模块使用基础按键及周期配置；
    // 1=K3/K4 菜单可改 AUTO/周期，并组装送 pix 域的 48 位 OSD 状态字。OSD_ITEMS 为菜单项数，默认 5 项，显示布局见 osd_overlay。
    parameter integer OSD_EN=0,
    parameter integer OSD_ITEMS=5
)(
    input wire clk,rst,logic_rst,key1,key2,
    // K3/K4（B1/C1，低有效，外部上拉；与 K1/K2 相同的两级同步+20ms 去抖），OSD_EN=0 时不接任何逻辑。
    input wire key3,key4,
    // 装载器错误码（tf_bmp_boot.error_code，sys 域寄存器），只进 OSD 状态字。
    input wire [7:0] media_error,
    input wire [1:0] switches,
    input wire [3:0] loaded,slot_reserved,
    input wire commit_valid,commit_ready,
    input wire [1:0] commit_slot,input wire [31:0] commit_gen,
    input wire boot_desc_valid,input wire [EPOCH_W+223:0] boot_desc_bus,
    input wire boot_ack_ready,
    input wire desc_ready,ack_valid,input wire [EPOCH_W+20:0] ack_bus,
    output wire desc_valid,ack_ready,output wire [EPOCH_W+223:0] desc_bus,
    output wire auto_mode,handoff,pc_fault,config_ready,
    output wire next_event,toggle_event,pc_desc_valid,pc_ack_ready,
    output wire [15:0] interval_frames,
    output wire interval_err,
    // 播放状态及计数观察输出；未连接时，对应统计逻辑可由综合工具删除。
    output wire [143:0] debug_status,
    // OSD 状态字（sys 域寄存/组合自本域寄存器，布局见 osd_overlay.v 注释）；OSD_EN=0 时恒 0。
    output wire [47:0] osd_status
);
    wire ms_tick;
    wire [5:0] cfg_period;
    reg cfg_applied;
    wire cfg_apply=handoff && config_ready && !cfg_applied;
    wire auto_toggle=toggle_event && cfg_applied;
    wire [EPOCH_W+223:0] pc_desc_bus;
    wire next_short,next_long,next_level,toggle_short,toggle_long,toggle_level;
    wire [1:0] cur_slot;
    wire [EPOCH_W-1:0] cur_epoch;wire [31:0] cur_gen;
    wire [15:0] last_frame,merged,rejected,no_target;
    wire [7:0] code;
    wire [15:0] remain_ms;
    // 送 play_ctrl 的 AUTO 切换/周期：OSD_EN=0 时就是原 auto_toggle/cfg_period/cfg_apply 三根线。
    wire play_auto_toggle;
    wire [5:0] play_period_s;
    wire play_period_set;
    key_events #(.ACTIVE_LOW(1),.DEBOUNCE_SAMPLES(DEBOUNCE_SAMPLES)) u_next(
        .clk(clk),.logic_rst(logic_rst),.ms_tick(ms_tick),.key_async(key1),
        .press(next_event),.short_rel(next_short),.long_press(next_long),.level_db(next_level));
    key_events #(.ACTIVE_LOW(1),.DEBOUNCE_SAMPLES(DEBOUNCE_SAMPLES)) u_toggle(
        .clk(clk),.logic_rst(logic_rst),.ms_tick(ms_tick),.key_async(key2),
        .press(toggle_event),.short_rel(toggle_short),.long_press(toggle_long),.level_db(toggle_level));
    dip_config #(.USE_DIP(USE_DIP),.SAMPLES(DEBOUNCE_SAMPLES)) u_config(
        .clk(clk),.logic_rst(logic_rst),.ms_tick(ms_tick),.dip_async(switches),
        .ready(config_ready),.period_s(cfg_period));
    always @(posedge clk)begin
        if(logic_rst)cfg_applied<=0;
        else if(cfg_apply)cfg_applied<=1;
    end
    generate if(OSD_EN!=0) begin:g_menu
        // K3：长按门限 1s（长按无动作，仅用于区分短按）；K4：长按 750ms=减一档。
        wire k3_press,k3_short,k3_long,k3_level,k4_press,k4_short,k4_long,k4_level;
        wire menu_open,menu_toggle,menu_period_set,src_key,cd_ok;
        wire [2:0] menu_item;
        wire [1:0] disp_mode;wire [3:0] bright_idx;wire [4:0] vol_lvl;
        // 菜单位置编码：0=关闭，1..5=菜单行号；两位 BCD 的十位<=6，省掉各自最高位，仍用原 48 位 CDC。
        wire [2:0] menu_pos=menu_open ? menu_item+3'd1 : 3'd0;
        wire [5:0] menu_period_new,menu_period_cur;
        wire [7:0] per_bcd,cd_bcd;
        key_events #(.ACTIVE_LOW(1),.DEBOUNCE_SAMPLES(DEBOUNCE_SAMPLES),.LONG_MS(1000)) u_k3(
            .clk(clk),.logic_rst(logic_rst),.ms_tick(ms_tick),.key_async(key3),
            .press(k3_press),.short_rel(k3_short),.long_press(k3_long),.level_db(k3_level));
        key_events #(.ACTIVE_LOW(1),.DEBOUNCE_SAMPLES(DEBOUNCE_SAMPLES),.LONG_MS(750)) u_k4(
            .clk(clk),.logic_rst(logic_rst),.ms_tick(ms_tick),.key_async(key4),
            .press(k4_press),.short_rel(k4_short),.long_press(k4_long),.level_db(k4_level));
        osd_menu #(.ITEMS(OSD_ITEMS)) u_menu(.clk(clk),.logic_rst(logic_rst),.ms_tick(ms_tick),
            .k3_short(k3_short),.k4_short(k4_short),.k4_long(k4_long),.enable(cfg_applied),.auto_on(auto_mode),
            .cfg_period(cfg_period),.cfg_apply(cfg_apply),.remain_ms(remain_ms),
            .menu_open(menu_open),.item(menu_item),.disp_mode(disp_mode),.bright_idx(bright_idx),.vol_lvl(vol_lvl),.auto_toggle_req(menu_toggle),.period_set_req(menu_period_set),
            .period_new(menu_period_new),.period_cur(menu_period_cur),.src_key(src_key),
            .period_bcd(per_bcd),.countdown_bcd(cd_bcd),.countdown_ok(cd_ok));
        // 同拍 K2 与菜单都要切换时只切一次（K2 优先，菜单请求作废），不会双切抵消。
        assign play_auto_toggle=auto_toggle || (menu_toggle && cfg_applied && !toggle_event);
        // 菜单改周期只在 cfg_applied 之后（enable）发生，与 cfg_apply 不会同拍。
        assign play_period_s=menu_period_set ? menu_period_new : cfg_period;
        assign play_period_set=cfg_apply || menu_period_set;
        assign osd_status={vol_lvl,bright_idx,disp_mode,cd_ok,code,media_error,cd_bcd[6:0],per_bcd[6:0],src_key,auto_mode,menu_pos,handoff};
    end else begin:g_no_menu
        assign play_auto_toggle=auto_toggle;
        assign play_period_s=cfg_period;
        assign play_period_set=cfg_apply;
        assign osd_status=48'd0;
    end endgenerate
    play_ctrl #(.EPOCH_W(EPOCH_W),.MS_CYCLES(MS_CYCLES)) u_play(
        .clk(clk),.rst(rst),.logic_rst(logic_rst),
        .ev_next(next_event && cfg_applied),.ev_auto_toggle(play_auto_toggle),.auto_inhibit(1'b0),
        .period_s(play_period_s),.period_set(play_period_set),.auto_init(1'b0),
        .boot_loaded(loaded),.slot_reserved(slot_reserved),.commit_valid(commit_valid),.commit_ready(commit_ready),
        .commit_slot(commit_slot),.commit_gen(commit_gen),.boot_ack_ready(boot_ack_ready),
        .ack_valid(ack_valid),.ack_bus(ack_bus),.desc_ready(desc_ready),
        .ms_tick(ms_tick),.pc_desc_valid(pc_desc_valid),.pc_ack_ready(pc_ack_ready),
        .handoff(handoff),.pc_fault(pc_fault),.pc_desc_bus(pc_desc_bus),.auto_on(auto_mode),
        .cur_slot(cur_slot),.cur_epoch(cur_epoch),.cur_gen(cur_gen),.last_frame(last_frame),
        .interval_frames(interval_frames),.interval_err(interval_err),.merged_cnt(merged),.reject_cnt(rejected),.no_target_cnt(no_target),.pc_code(code),
        .remain_ms(remain_ms));
    assign desc_valid=handoff ? pc_desc_valid : boot_desc_valid;
    assign desc_bus=handoff ? pc_desc_bus : boot_desc_bus;
    assign ack_ready=boot_ack_ready | pc_ack_ready;
    assign debug_status={cur_slot,{{(32-EPOCH_W){1'b0}},cur_epoch},cur_gen,last_frame,merged,rejected,no_target,code,
        next_short,next_long,next_level,toggle_short,toggle_long,toggle_level};
endmodule
`default_nettype wire
