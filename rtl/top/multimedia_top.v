`timescale 1ns/1ps
`default_nettype none
// 整机接线入口：TF读文件，SDRAM存图像/PCM，HDMI输出画面和声音。
// 首图完整装载并提交后才允许显示。
// 当前默认启用WAV、频谱和OSD；图片支持BMP24/RGB565，BMP32关闭。
// por负责上电复位；PLL失锁后，各时钟域也要重新复位并等待锁定。
module multimedia_top #(
    // 1启用BMP位深识别；实际允许哪些位深，还要看BMP32_EN。
    parameter EXT_BMP_FORMATS=1,
    // 1 允许 32 位 BMP；默认只接收 BMP24 和 RGB565。
    parameter BMP32_EN=0,
    parameter USE_EG_SDRAM=1,USE_EG_BRAM=1,
    parameter LOCK_QUAL_CYCLES=1000000,     // 50MHz 下 20ms（规格 §7 锁定确认连续 20ms）
    parameter VIDEO_TEST_PATTERN=0,         // 0=显示数据路径（默认），1=测试模式彩条
    parameter E2_WAV_EN=1,                 // 1装载真实WAV；播放条件仍由音频控制模块确认。
    parameter AUDIO_WAIT_CYCLES=35000000,
    parameter HDMI_AUDIO=1,                 // 1发送HDMI音频数据岛；0退回仅视频路径。
    parameter TF_POWER_CYCLES=50000,TF_POLL_CYCLES=50000, // sys域上电/初始化间隔，单位为 sys 时钟周期
    parameter B3_USE_DIP=1,B3_MS_CYCLES=50000, // 拨码禁用时手动5秒；50MHz下1ms为50000拍
    // 1使用现有18帧擦拭转场；0使用原硬切路径。
    parameter E1_WIPE_EN=1,
    // 频谱在 pix 域采样和计算；关闭时视频旁路，音频播放继续。
    // SPECTRUM_USE_EG_DSP 选择器件乘法宏或行为乘法，默认随 BRAM 配置选择。
    parameter SPECTRUM_EN=1, SPECTRUM_USE_EG_DSP=USE_EG_BRAM,
    // 1启用K3/K4菜单及OSD；状态经专用mailbox送到像素域。
    parameter OSD_EN=1,
    // 1 使用带上下文的 512B 分块文件服务；0 使用整文件服务接口。
    parameter P0_CONTEXT_ENABLE=1
)(
    input  wire clk_50m,                    // R7
    input wire key1,key2,                   // A2/B2；外部10k上拉，按下接地
    input wire key3,key4,                   // B1/C1：K3 菜单/下一项，K4 加/长按减；同 K1/K2 电路
    input wire [1:0] switches,              // 仅C8/C7：{SW2,SW1}周期；上电手动，K2切AUTO
    output wire auto_mode,                  // 独立用户LED0/A4，高电平亮；首图前兼作启动错误闪码（见下方 boot_led）
    output wire tf_cs_n,tf_sclk,tf_mosi,    // A12/A14/A13
    input  wire tf_miso,                    // B14
    output wire buzzer_n,                   // H11，低有效
    // HDMI_B 差分 P 端，LVDS33；N 端 D1/F2/G6/C2 由工具配对，具体引脚见 ADC。
    output wire [2:0] hdmi_d_p,             // D0/D1/D2 = G5/F1/E1
    output wire hdmi_clk_p                  // C3
);
    // sys 时钟经全局缓冲分发，减少大量寄存器负载造成的局部布线偏斜。
    // 50MHz 业务时钟经全局网分发，无分频、门控和相移；PLL 参考直接连接输入引脚。
    // 所有sys寄存器（含POR/复位同步器）共用此网，禁止留下两条逻辑sys时钟树。
    wire clk_sys;
    EG_LOGIC_BUFG u_sys_bufg(.i(clk_50m),.o(clk_sys));
    // 播放/显示 epoch 全链路采用 8 位；槽世代和 SD 初始化标识维持各自位宽。
    localparam integer EPOCH_W=8;
    // ---------------- 上电复位（sys 域） ----------------
    wire rst_word_logic;
    wire rst_pix_logic;
    wire rst_gear_logic;
    reg [15:0] por_shift = 16'hFFFF;
    always @(posedge clk_sys) por_shift <= {por_shift[14:0],1'b0};
    wire por = por_shift[15];

    // ---------------- 时钟 ----------------
    wire clk_mem,clk_device,mem_locked,clk_video_ref,clk_pix,clk_serial,clk_gear,clk_word,ref_locked,pix_locked;
    // 产品mem时钟为100MHz，器件时钟延后7.375ns，给SDRAM引脚留采样窗口。
    // 器件相位与寄存 PHY 及 td 约束共同确定读写采样窗口。
    pll_mem #(.DEVICE_CPHASE(6),.DEVICE_FPHASE(3)) u_pll_mem(.clk_ref(clk_50m),.reset(por),.clk_mem(clk_mem),.clk_device(clk_device),.locked(mem_locked));
    pll_video u_pll_video(.clk_ref(clk_50m),.reset(por),.clk_video_ref(clk_video_ref),.clk_pix(clk_pix),
        .clk_serial(clk_serial),.clk_gear(clk_gear),.clk_word(clk_word),
        .locked_ref(ref_locked),.locked_pix(pix_locked));

    // 三路锁定连续合格后才释放各域；任何失锁立即异步复位全部业务域（存储失锁=内容失效，规格 §7）。
    wire lock_reset_request,clocks_qualified;
    pll_lock_guard #(.LOCK_COUNT(3),.QUAL_CYCLES(LOCK_QUAL_CYCLES)) u_lock(
        .sys_clk(clk_sys),.arst(por),.locked_async({pix_locked,ref_locked,mem_locked}),
        .reset_request(lock_reset_request),.qualified(clocks_qualified));
    wire domain_arst_comb = por | lock_reset_request | !clocks_qualified;
    // 跨域复位请求先在 sys 域寄存，避免将组合锁定判据直接送到多个时钟域。
    // por 异步置位保证上电复位；失锁检测到业务复位最多增加一个 sys 周期（20ns）。
    reg domain_arst = 1'b1;
    always @(posedge clk_sys or posedge por)
        if (por) domain_arst <= 1'b1;
        else     domain_arst <= domain_arst_comb;
    wire rst_sys,rst_pix,rst_mem;wire rst_sys_logic;
    reset_sync u_rst_sys(.clk(clk_sys),.arst(domain_arst),.rst(rst_sys),.logic_rst(rst_sys_logic));
    reset_sync u_rst_pix(.clk(clk_pix),.arst(domain_arst),.rst(rst_pix),.logic_rst(rst_pix_logic));
    // HDMI word/gear 域各自异步置位、同步释放（同一 domain_arst，PLL2 失锁时与 pix 一起复位）。
    wire rst_word,rst_gear;
    reset_sync u_rst_word(.clk(clk_word),.arst(domain_arst),.rst(rst_word),.logic_rst(rst_word_logic));
    reset_sync u_rst_gear(.clk(clk_gear),.arst(domain_arst),.rst(rst_gear),.logic_rst(rst_gear_logic));
    // mem域用自己的复位同步器；业务逻辑不要直接使用sys域复位去门控数据。
    reset_sync u_rst_mem(.clk(clk_mem),.arst(domain_arst),.rst(rst_mem));

    // ---------------- 合封 SDRAM（mem 域） ----------------
    // 段客户端接显示行包。sdram_shared_rw = 显示段优先的共享层 + sdram_stream_controller
    // （上电初始化/刷新自主运行）。后台写口来自真实BMP装载器。
    // 共享层、控制器与 PHY 用同一个 rst_mem（复位后共同重新初始化）。
    wire sd_cs_n,sd_ras_n,sd_cas_n,sd_we_n,sd_cke,sd_wr_valid,sd_wr_beat;
    wire [10:0] sd_addr; wire [1:0] sd_ba; wire [31:0] phy_rd_data,sd_wr_data;
    wire line_valid,line_ready,line_cancel,line_data_valid,line_data_ready,line_done;
    wire [20:0] line_addr; wire [3:0] line_count; wire [31:0] line_budget;
    wire load_valid,load_ready,load_data_valid,load_done,load_done_w;
    wire [20:0] load_addr;wire [3:0] load_count;wire [31:0] load_budget,load_data;
    localparam AUDIO_WAV_ENABLE=E2_WAV_EN && HDMI_AUDIO;
    wire au_req_valid,au_req_ready,au_req_cancel,au_req_bank,au_rsp_valid,au_rsp_bank;
    wire [17:0] au_req_words;wire [15:0] au_req_token,au_rsp_token;wire [1:0] au_rsp_status;
    wire au_mem_valid,au_mem_ready,au_mem_cancel,au_mem_bank,au_mem_rsp_valid,au_mem_rsp_ready,au_mem_rsp_bank;
    wire [17:0] au_mem_words;wire [15:0] au_mem_token,au_mem_rsp_token;wire [1:0] au_mem_rsp_status;
    wire au_pcm_valid,au_pcm_ready,au_pcm_urgent,au_pcm_data_valid,au_pcm_data_ready,au_pcm_done;
    wire [20:0] au_pcm_addr;wire [31:0] au_pcm_budget;
    // PHY 在命令和写数据输出端寄存，并对读 DQ 采样，控制器读延迟设为 CL+2。
    // mem=100MHz 时刷新间隔和宽限各为 750 拍（7.5µs），最长开行为 8000 拍（80µs）。
    // 显示硬截止为 1980 拍，行截止为 2000 拍，后台写入保护为 64 拍。
    sdram_shared_rw #(.ENABLE_PCM(1),.PHY_RD_EXTRA(2),.REFRESH_INTERVAL(750),.REFRESH_GRACE(750),.T_RAS_MAX(8000)) u_sdram_ctrl(
        .clk(clk_mem),.arst(rst_mem),.reserve_cycles(16'hffff),
        .line_valid(line_valid),.line_ready(line_ready),.line_addr(line_addr),.line_count(line_count),
        .line_budget(line_budget),.line_cancel(line_cancel),
        .line_data_valid(line_data_valid),.line_data_ready(line_data_ready),.line_done(line_done),
        // PCM DMA 为第三个读客户端，在 FIFO 已缓存样本不足时提供紧急仲裁提示。
        .pcm_valid(au_pcm_valid),.pcm_urgent(au_pcm_urgent),.pcm_ready(au_pcm_ready),.pcm_addr(au_pcm_addr),.pcm_budget(au_pcm_budget),
        .pcm_cancel(1'b0),.pcm_data_valid(au_pcm_data_valid),.pcm_data_ready(au_pcm_data_ready),.pcm_done(au_pcm_done),
        // 唯一共享控制器：装载器不再输出第二套SDRAM命令。
        .load_valid(load_valid),.load_ready(load_ready),.load_addr(load_addr),.load_count(load_count),.load_budget(load_budget),.load_cancel(1'b0),
        .load_data_valid(load_data_valid),.load_data(load_data),.load_beat(sd_wr_beat),.load_done(load_done),.load_done_was_write(load_done_w),
        .rd_data_in(phy_rd_data),.wr_data_valid(sd_wr_valid),.wr_data(sd_wr_data),.busy(),.owner_write(),
        .sd_cs_n(sd_cs_n),.sd_ras_n(sd_ras_n),.sd_cas_n(sd_cas_n),.sd_we_n(sd_we_n),.sd_cke(sd_cke),
        .sd_addr(sd_addr),.sd_ba(sd_ba));
    // PHY 写拍用共享层的 load_beat（控制器实际消费的写拍），不用 wr_data_valid（只表示来源备好）。接受的BL8在取消后仍供满八拍。
    sdram_phy #(.USE_EG_SDRAM(USE_EG_SDRAM),.REGISTERED(1)) u_sdram_phy(
        .clk_mem(clk_mem),.clk_device(clk_device),.arst(rst_mem),
        .cs_n(sd_cs_n),.ras_n(sd_ras_n),.cas_n(sd_cas_n),.we_n(sd_we_n),.cke(sd_cke),
        .addr(sd_addr),.ba(sd_ba),.wr_beat(sd_wr_beat),.wr_data(sd_wr_data),.rd_data(phy_rd_data));

    // ---------------- 视频时序（pix 域） ----------------
    wire [10:0] vx; wire [9:0] vy; wire vde,vhs,vvs,vframe,vvblank;
    wire vy_lt720,vy_lt719; wire [9:0] vy_plus1,vy_horizon; wire [19:0] vy_desc0,vy_desc1,vy_desc2;
    // 水平条件来自同一实际光栅发生器，禁止悬空或固定条件。
    wire cold_line_qual,row_edge_qual;
    video_timing_720p u_video_timing(.clk(clk_pix),.rst(rst_pix_logic),.x(vx),.y(vy),.de(vde),
        .hsync(vhs),.vsync(vvs),.frame_start(vframe),.vblank_start(vvblank),
        .y_lt720(vy_lt720),.y_lt719(vy_lt719),.y_plus1(vy_plus1),.y_horizon(vy_horizon),
        .cold_line_qual(cold_line_qual),.row_edge_qual(row_edge_qual),.y_desc0(vy_desc0),.y_desc1(vy_desc1),.y_desc2(vy_desc2));

    // ---------------- 显示数据路径（规格 §9.3/§10.2） ----------------
    // 显示由三部分组成，跨域使用 cdc_mailbox 和双时钟行 RAM：
    //   display_read_owner（sys/pix）：显示引用与槽所有权；
    //   display_row_bundle（pix/mem）：行包预取，以及作为 SDRAM 段客户端；
    //   bundle_scanout（pix）：行调度与像素输出。
    // 双域模块按其契约共用同一个异步复位 domain_arst（cdc_mailbox 要求两端共同 arst）。
    // 缩略图读取关闭（layout=0，FETCH_THUMBNAILS=0）。E1_WIPE_EN=0 时为单图全屏：wipe=960、old 引用=new 引用；
    // E1_WIPE_EN=1 时 old 引用/分界来自下方 g_e1_wipe 的 wipe_ctrl。行包本来就同时取 old/new 两半各 640 字，
    // 所以擦拭不增加任何 SDRAM 读流量（old=new 时也是读两遍同一行）。
    wire disp_req_valid,disp_req_ready,disp_rel_valid,disp_rel_ready,disp_evt_valid,disp_evt_ready;
    wire [EPOCH_W+134:0] disp_req_bus; wire [EPOCH_W+118:0] disp_rel_bus; wire [EPOCH_W+136:0] disp_evt_bus; wire [EPOCH_W+46:0] disp_evt_primary;
    wire disp_bundle_fault,disp_scan_fault,disp_rd_bank,disp_prefetch_ready,disp_ref_valid,disp_ref_switch;
    wire [10:0] disp_rd_x; wire [31:0] disp_rd_data,disp_ref_gen;wire [EPOCH_W-1:0] disp_ref_epoch; wire [1:0] disp_ref_slot;
    wire boot_commit_valid,boot_commit_ready,boot_desc_valid,boot_desc_ready,boot_ack_ready;
    wire [1:0] boot_commit_slot; wire [31:0] boot_commit_gen; wire [EPOCH_W+223:0] boot_desc_bus;
    wire [3:0] media_reserved,media_writable;
    // disp_writable_raw 是 display_read_owner 原始 slot_writable；media_writable 是送装载器的最终值
    // （E1_WIPE_EN=1 时再屏蔽擦拭旧图槽，=0 时直连）。
    wire [3:0] disp_writable_raw;
    // 送 bundle_scanout 的旧图槽号与分界（=0 时 old=new、960）。
    // 行包 old_gen 接当前引用 gen，仅作标签匹配；同一 epoch 内保持不变。
    // 旧槽内容由写保护维持，避免为未参与寻址的字段复制跨域数据寄存器。
    wire [1:0] disp_old_slot; wire [10:0] disp_wipe_x;
    wire media_ack_valid;wire [EPOCH_W+20:0] media_ack_bus;
    wire [3:0] media_loaded;
    wire owner_desc_valid,owner_ack_ready;wire [EPOCH_W+223:0] owner_desc_bus;
    wire [5:0] wav_bank_states;wire [3:0] wav_bank_slots,wav_unavailable;wire [35:0] wav_bank_words;
    wire wav_pin_valid,wav_pin_bank,wav_retire_valid,wav_retire_bank,wav_want_valid,wav_bank_fault;
    wire [1:0] wav_release_bank,wav_want_slot,wav_ignore_present;
    wire display_desc_valid,display_desc_ready;wire [EPOCH_W+223:0] display_desc_bus;
    wire audio_commit_go;wire [19:0] audio_commit_data,ref_audio;
    // 启动错误闪码所需的 sys 域信号（tf_bmp_boot 的 done/error_code、b3 的 handoff、播放控制模块原 LED）。
    wire boot_done_w;wire [7:0] boot_error_w;wire b3_handoff,auto_mode_play;
    // OSD：b3 组装的 48 位 OSD 状态字（sys 域）；装载器错误码与启动闪码共用 boot_error_w（同一 tf_bmp_boot.error_code）。
    wire [47:0] osd_status_sys;
    // 媒体链与显示共用domain_arst，各端内部独立同步释放。
    // domain_arst 仅进入复位同步器，业务门控使用各端本域的同步复位。
    // 当前扫描器只读取主图，整机 WAV 配置关闭缩略图生成；
    // E2_WAV_EN=0 时启用带缩略图的图片装载配置。
    tf_bmp_boot #(.BMP32_EN(BMP32_EN),.EXT_BMP_FORMATS(EXT_BMP_FORMATS),.KEY_W(8),.ENABLE_THUMBNAILS(!E2_WAV_EN),.ENABLE_WAV(E2_WAV_EN),.ENABLE_AUDIO_CONTROL(AUDIO_WAV_ENABLE),.EPOCH_W(EPOCH_W),.USE_EG_BRAM(USE_EG_BRAM),.POWER_CYCLES(TF_POWER_CYCLES),.POLL_CYCLES(TF_POLL_CYCLES),
        .CONTEXT_ENABLE(P0_CONTEXT_ENABLE)) u_media(
        .clk(clk_sys),.mem_clk(clk_mem),.arst(domain_arst),.cancel(1'b0),
        .spi_cs_n(tf_cs_n),.spi_sclk(tf_sclk),.spi_mosi(tf_mosi),.spi_miso(tf_miso),
        .load_valid(load_valid),.load_ready(load_ready),.load_addr(load_addr),.load_count(load_count),.load_budget(load_budget),
        .load_data_valid(load_data_valid),.load_data(load_data),.load_beat(sd_wr_beat),.load_done(load_done),.load_done_was_write(load_done_w),
        .slot_reserved(media_reserved),.slot_writable(media_writable),
        .commit_valid(boot_commit_valid),.commit_ready(boot_commit_ready),.commit_slot(boot_commit_slot),.commit_gen(boot_commit_gen),
        .desc_valid(boot_desc_valid),.desc_ready(boot_desc_ready),.desc_bus(boot_desc_bus),
        .wav_current_valid(media_ack_valid && owner_ack_ready && media_ack_bus[EPOCH_W+20:EPOCH_W+18]==0),
        .wav_current_slot(media_ack_bus[1:0]),.wav_want_valid(wav_want_valid),.wav_want_slot(wav_want_slot),.wav_ignore_present(wav_ignore_present),
        .wav_pin_valid(wav_pin_valid),.wav_pin_bank(wav_pin_bank),.wav_retire_valid(wav_retire_valid),.wav_retire_bank(wav_retire_bank),.wav_release_bank(wav_release_bank),
        .wav_bank_states(wav_bank_states),.wav_bank_slots(wav_bank_slots),.wav_bank_words(wav_bank_words),.wav_bank_ready(),.wav_unavailable(wav_unavailable),.wav_fault(wav_bank_fault),
        .audio_req_valid(au_req_valid),.audio_req_ready(au_req_ready),.audio_req_cancel(au_req_cancel),.audio_req_bank(au_req_bank),.audio_req_words(au_req_words),.audio_req_token(au_req_token),
        .audio_rsp_valid(au_rsp_valid),.audio_rsp_status(au_rsp_status),.audio_rsp_bank(au_rsp_bank),.audio_rsp_token(au_rsp_token),
        .audio_mem_valid(au_mem_valid),.audio_mem_ready(au_mem_ready),.audio_mem_cancel(au_mem_cancel),.audio_mem_bank(au_mem_bank),.audio_mem_words(au_mem_words),.audio_mem_token(au_mem_token),
        .audio_mem_rsp_valid(au_mem_rsp_valid),.audio_mem_rsp_ready(au_mem_rsp_ready),.audio_mem_rsp_status(au_mem_rsp_status),.audio_mem_rsp_bank(au_mem_rsp_bank),.audio_mem_rsp_token(au_mem_rsp_token),
        .ack_valid(media_ack_valid),.ack_ready(boot_ack_ready),.ack_bus(media_ack_bus),.loaded(media_loaded),.failed(),.done(boot_done_w),.error_code(boot_error_w));
    // sys 域播放控制模块负责按键、周期配置及启动后的显示描述符交接。
    b3_control #(.EPOCH_W(EPOCH_W),.MS_CYCLES(B3_MS_CYCLES),.USE_DIP(B3_USE_DIP),.OSD_EN(OSD_EN),.OSD_ITEMS(5)) u_b3(
        .clk(clk_sys),.rst(rst_sys),.logic_rst(rst_sys_logic),.key1(key1),.key2(key2),.switches(switches),
        .key3(key3),.key4(key4),.media_error(boot_error_w),.osd_status(osd_status_sys),
        .loaded(media_loaded),.slot_reserved(media_reserved),.commit_valid(boot_commit_valid),.commit_ready(boot_commit_ready),
        .commit_slot(boot_commit_slot),.commit_gen(boot_commit_gen),
        .boot_desc_valid(boot_desc_valid),.boot_desc_bus(boot_desc_bus),.boot_ack_ready(boot_ack_ready),
        .desc_ready(boot_desc_ready),.ack_valid(media_ack_valid),.ack_bus(media_ack_bus),
        .desc_valid(owner_desc_valid),.desc_bus(owner_desc_bus),.ack_ready(owner_ack_ready),.auto_mode(auto_mode_play),
        .handoff(b3_handoff),.pc_fault(),.config_ready(),.next_event(),.toggle_event(),.pc_desc_valid(),.pc_ack_ready(),
        .interval_frames(),.interval_err(),.debug_status());
    generate if(AUDIO_WAV_ENABLE) begin:g_wav_control
        wav_audio_control #(.ENABLE(1),.EPOCH_W(EPOCH_W),.WAIT_CYCLES(AUDIO_WAIT_CYCLES)) u_control(
            .clk(clk_sys),.rst(rst_sys_logic),.loaded(media_loaded),.bank_states(wav_bank_states),.bank_slots(wav_bank_slots),.bank_words(wav_bank_words),.unavailable(wav_unavailable),
            .want_valid(wav_want_valid),.want_slot(wav_want_slot),.ignore_present(wav_ignore_present),
            .pin_valid(wav_pin_valid),.pin_bank(wav_pin_bank),.retire_valid(wav_retire_valid),.retire_bank(wav_retire_bank),.release_bank(wav_release_bank),
            .cmd_valid(au_req_valid),.cmd_ready(au_req_ready),.cmd_cancel(au_req_cancel),.cmd_bank(au_req_bank),.cmd_words(au_req_words),.cmd_token(au_req_token),
            .rsp_valid(au_rsp_valid),.rsp_status(au_rsp_status),.rsp_bank(au_rsp_bank),.rsp_token(au_rsp_token),
            .in_valid(owner_desc_valid),.in_ready(boot_desc_ready),.in_bus(owner_desc_bus),
            .out_valid(display_desc_valid),.out_ready(display_desc_ready),.out_bus(display_desc_bus),
            .ack_fire(media_ack_valid && owner_ack_ready),.ack_bus(media_ack_bus),.bank_fault(wav_bank_fault),.prefetch_late(),.stale_events(),.fault());
    end else begin:g_wav_control_off
        assign display_desc_valid=owner_desc_valid;assign display_desc_bus=owner_desc_bus;assign boot_desc_ready=display_desc_ready;
        assign wav_want_valid=0;assign wav_want_slot=0;assign wav_ignore_present=0;assign wav_pin_valid=0;assign wav_pin_bank=0;
        assign wav_retire_valid=0;assign wav_retire_bank=0;assign wav_release_bank=0;
        assign au_req_valid=0;assign au_req_cancel=0;assign au_req_bank=0;assign au_req_words=0;assign au_req_token=0;
    end endgenerate
    display_read_owner #(.EPOCH_W(EPOCH_W),.ENABLE_AUDIO_META(AUDIO_WAV_ENABLE)) u_disp_owner(.arst(domain_arst),.sys_clk(clk_sys),.px_clk(clk_pix),
        .slot_reserved(media_reserved),.slot_writable(disp_writable_raw),
        .commit_valid(boot_commit_valid),.commit_ready(boot_commit_ready),
        .commit_slot(boot_commit_slot),.commit_slot_gen(boot_commit_gen),
        .desc_valid(display_desc_valid),.desc_ready(display_desc_ready),.desc_bus(display_desc_bus),
        .ack_valid(media_ack_valid),.ack_ready(owner_ack_ready),.ack_bus(media_ack_bus),
        .vblank(vvblank),.prefetch_ready(disp_prefetch_ready),
        .ref_valid(disp_ref_valid),.ref_switch(disp_ref_switch),.ref_epoch(disp_ref_epoch),
        .ref_slot_gen(disp_ref_gen),.ref_slot(disp_ref_slot),.ref_frame(),
        .read_req_fire(disp_req_valid && disp_req_ready),.read_epoch(disp_req_bus[EPOCH_W+134:135]),
        .read_slot(disp_req_bus[100:99]),.read_slot_gen(disp_req_bus[98:67]),
        .read_source_y(disp_req_bus[66:58]),.read_bank({1'b0,disp_req_bus[16]}),
        .row_evt_fire(disp_evt_valid && disp_evt_ready),.row_evt_bus(disp_evt_primary),.guard_fault(),.audio_commit_go(audio_commit_go),.audio_commit_data(audio_commit_data),.ref_audio(ref_audio));
    // 整机只用 RGB 24 位：行RAM按 24 位真实实现（见 display_row_bundle LINE_BITS 注释）。
    display_row_bundle #(.EPOCH_W(EPOCH_W),.USE_EG_BRAM(USE_EG_BRAM),.LINE_BITS(24),.FETCH_THUMBNAILS(0)) u_disp_bundle(.arst(domain_arst),.px_clk(clk_pix),.mem_clk(clk_mem),
        .req_valid(disp_req_valid),.req_ready(disp_req_ready),.req_bus(disp_req_bus),
        .rel_valid(disp_rel_valid),.rel_ready(disp_rel_ready),.rel_bus(disp_rel_bus),
        .evt_valid(disp_evt_valid),.evt_ready(disp_evt_ready),.evt_bus(disp_evt_bus),.evt_primary(disp_evt_primary),
        .fault(disp_bundle_fault),.rd_bank(disp_rd_bank),.rd_x(disp_rd_x),.rd_data(disp_rd_data),
        .seg_valid(line_valid),.seg_ready(line_ready),.seg_addr(line_addr),.seg_count(line_count),
        .seg_budget(line_budget),.seg_cancel(line_cancel),
        .data_valid(line_data_valid),.data_ready(line_data_ready),.data_in(phy_rd_data),.seg_done(line_done));
    wire px_valid,px_hs,px_vs; wire [31:0] px_word;
    wire [10:0] px_x;wire [9:0] px_y;
    // ---------------- E1 18 帧擦拭（规格 §11：b(k)=ceil(960k/18)，x<b 取新图） ----------------
    // pix 域 wipe_ctrl：在 ref_switch 拍给出旧图（上一张引用）与 b(1)，之后每个 vblank_start 推进一帧，18 帧后恒 960。
    // sys 域 wipe_old_guard：成功响应后把“上一张显示图”继续标成不可写，直到下一次成功响应（届时对旧图的最后一次行读取已结束）。
    // 两者分别使用各自本域寄存信号；E1_WIPE_EN=0 时由 generate 裁去。
    generate if (E1_WIPE_EN!=0) begin : g_e1_wipe
        // wiping/hold_mask 是诊断口，产品不接（与 b3_control 的 debug_status 等同样处理，综合删除）。
        wipe_ctrl u_wipe(.clk(clk_pix),.rst(rst_pix_logic),.ref_valid(disp_ref_valid),.ref_switch(disp_ref_switch),
            .ref_slot(disp_ref_slot),.vblank_start(vvblank),
            .old_slot(disp_old_slot),.wipe_x(disp_wipe_x),.wiping());
        wipe_old_guard u_wipe_guard(.clk(clk_sys),.rst(rst_sys_logic),
            .ack_fire(media_ack_valid && owner_ack_ready),.ack_ok(media_ack_bus[EPOCH_W+20:EPOCH_W+18]==3'd0),
            .ack_slot(media_ack_bus[1:0]),.writable_in(disp_writable_raw),.writable_out(media_writable),.hold_mask());
    end else begin : g_e1_cut
        assign disp_old_slot=disp_ref_slot; assign disp_wipe_x=11'd960;
        assign media_writable=disp_writable_raw;
    end endgenerate
    bundle_scanout #(.EPOCH_W(EPOCH_W)) u_disp_scan(.px_clk(clk_pix),.arst(domain_arst),.timing_x(vx),.timing_y(vy),
        .timing_y_lt720(vy_lt720),.timing_y_lt719(vy_lt719),.timing_y_plus1(vy_plus1),.timing_horizon(vy_horizon),
        .timing_cold_line(cold_line_qual),.timing_row_edge(row_edge_qual),.timing_y_desc0(vy_desc0),.timing_y_desc1(vy_desc1),.timing_y_desc2(vy_desc2),.de(vde),
        .hsync(vhs),.vsync(vvs),.vblank_start(vvblank),
        .ref_valid(disp_ref_valid),.ref_switch(disp_ref_switch),.ref_epoch(disp_ref_epoch),
        .old_slot(disp_old_slot),.new_slot(disp_ref_slot),.old_gen(disp_ref_gen),.new_gen(disp_ref_gen),
        .thumb_gens(128'd0),.thumb_layout(1'b0),.wipe_x(disp_wipe_x),.prefetch_ready(disp_prefetch_ready),
        .req_valid(disp_req_valid),.req_ready(disp_req_ready),.req_bus(disp_req_bus),
        .rel_valid(disp_rel_valid),.rel_ready(disp_rel_ready),.rel_bus(disp_rel_bus),
        .evt_valid(disp_evt_valid),.evt_ready(disp_evt_ready),.evt_bus(disp_evt_bus),
        .bundle_fault(disp_bundle_fault),.fault(disp_scan_fault),.rd_bank(disp_rd_bank),.rd_x(disp_rd_x),
        .rd_data(disp_rd_data),.pixel_valid(px_valid),.pixel_word(px_word),.pixel_hsync(px_hs),.pixel_vsync(px_vs),
        .pixel_x(px_x),.pixel_y(px_y),.pixel_epoch(),.underrun_pulse(),.display_present());

    // ---------------- HDMI：视频编码、PCM 音频数据岛；HDMI_AUDIO=0 时仅输出 DVI 视频 ----------------
    // 视频源：VIDEO_TEST_PATTERN=0（默认）走显示数据路径（像素字 0x00RRGGBB，比原始时序晚 2 拍，
    // DE/HS/VS 同步延迟）；=1 为测试模式彩条（不经 SDRAM）。
    wire [7:0] tp_r,tp_g,tp_b; wire [9:0] sym0,sym1,sym2;
    colorbar_720p u_test_pattern(.x(vx),.de(vde),.r(tp_r),.g(tp_g),.b(tp_b));
    wire s_de = VIDEO_TEST_PATTERN ? vde : px_valid;
    wire s_hs = VIDEO_TEST_PATTERN ? vhs : px_hs;
    wire s_vs = VIDEO_TEST_PATTERN ? vvs : px_vs;
    wire [7:0] s_r = VIDEO_TEST_PATTERN ? tp_r : px_word[23:16];
    wire [7:0] s_g = VIDEO_TEST_PATTERN ? tp_g : px_word[15:8];
    wire [7:0] s_b = VIDEO_TEST_PATTERN ? tp_b : px_word[7:0];
    wire [10:0] s_x=VIDEO_TEST_PATTERN ? vx : px_x;
    wire [9:0] s_y=VIDEO_TEST_PATTERN ? vy : px_y;
    // 音频声明放在频谱使用之前。AUDIO_WAV_ENABLE时pipeline输出本身已含TONE回退，
    // 与HDMI所吃的最终PCM完全相同；这里取音量前原样样本，不增加全局commit/epoch/flush扇出。
    wire au_ce,tone_valid,di_accept;wire [15:0] tone_l,tone_r;
    wire wav_sample_valid;wire [15:0] wav_sample_l,wav_sample_r;
    wire sp_de,sp_hs,sp_vs;wire [7:0] sp_r,sp_g,sp_b;
    wire [10:0] sp_x;wire [9:0] sp_y;
    wire [4:0] vol_level;
    wire spec_hide; // OSD 帧快照送频谱显示条件，OSD 关闭时恒 0。
    generate if(SPECTRUM_EN!=0) begin : g_spectrum
        // 柱高经 BRAM 同步读口传递，叠加器给列号 height_addr，下一拍收 height_data。
        wire [3:0] height_addr;
        wire [5:0] height_data;
        wire heights_valid;
        spectrum_top #(.USE_EG_BRAM(USE_EG_BRAM),.USE_EG_DSP(SPECTRUM_USE_EG_DSP)) u_spectrum(
            .clk(clk_pix),.rst(rst_pix_logic),.vblank_start(vvblank),
            .in_valid(AUDIO_WAV_ENABLE ? wav_sample_valid : tone_valid),
            .in_l(AUDIO_WAV_ENABLE ? wav_sample_l : tone_l),.in_r(AUDIO_WAV_ENABLE ? wav_sample_r : tone_r),
            .view_addr(height_addr),.view_h(height_data),.bars_valid(heights_valid),.busy(),.sat_flag(),
            .bin_valid(),.bin_idx(),.bin_lg(),.debug_re(),.debug_im(),.done());
        spectrum_overlay u_overlay(.hide(spec_hide),.clk(clk_pix),.rst(rst_pix_logic),.h_addr(height_addr),.h_data(height_data),.bars_valid(heights_valid),
            .de(s_de),.hs(s_hs),.vs(s_vs),.x(s_x),.y(s_y),.r(s_r),.g(s_g),.b(s_b),
            .o_de(sp_de),.o_hs(sp_hs),.o_vs(sp_vs),.o_x(sp_x),.o_y(sp_y),.o_r(sp_r),.o_g(sp_g),.o_b(sp_b));
    end else begin : g_spectrum_bypass
        assign sp_de=s_de;assign sp_hs=s_hs;assign sp_vs=s_vs;
        assign sp_r=s_r;assign sp_g=s_g;assign sp_b=s_b;assign sp_x=s_x;assign sp_y=s_y;
    end endgenerate
    // OSD 插在 sp_* 与 o_* 之间，使用与 sp_* 同拍的 sp_x/sp_y，文字盖在频谱上。
    // ---------------- OSD（OSD_EN=1）：sys 状态字→唯一 sys→pix mailbox→帧边界快照→3 拍叠加流水（字库/版面各 1 块 BRAM9K） ----------------
    // v_* 送入 HDMI 输入寄存器；OSD_EN=0 时取 sp_*，=1 时取 osd_overlay 输出。
    // （视频整体晚 3 拍，DE/HS/VS/RGB 同延迟，行/场结构不变；HDMI 数据岛自己从 hs/vs 沿重建坐标）。
    wire v_de,v_hs,v_vs; wire [7:0] v_r,v_g,v_b;
    generate if (OSD_EN!=0) begin : g_osd
        // sys 源端：mailbox 一空闲就发送当前状态字（src_valid 仅在 src_ready 时为 1，握手往返约十几拍一次）。
        // 发送值在 mailbox 内 held_bus 保持到目标确认；目标端 overlay 只在 vblank 取最新值，故连续发送与“变化才发”显示相同。
        // mailbox 空闲时重发最新状态，不另存一份已发送副本。
        // 目标端在帧边界取整组快照；传输与显示之间不要求逐次状态变化一一对应。
        wire st_src_ready,st_dst_valid; wire [47:0] st_dst_data;
        wire st_send=st_src_ready;
        // OSD 状态使用与 slot_owner 相同的 cdc_mailbox 握手机制，
        // 共同 arst=domain_arst；目标端 dst_ready 恒 1（overlay 每拍可收，单槽不会丢最新值）。
        cdc_mailbox #(.DATA_WIDTH(48)) u_osd_status(
            .arst(domain_arst),.src_clk(clk_sys),.src_valid(st_send),.src_ready(st_src_ready),
            .src_data(osd_status_sys),.src_active(),.src_busy(),.src_done(),
            .dst_clk(clk_pix),.dst_valid(st_dst_valid),.dst_ready(1'b1),
            .dst_data(st_dst_data),.dst_active());
        osd_overlay #(.STATUS_W(48),.MENU_ROWS(5),.USE_EG_BRAM(USE_EG_BRAM)) u_overlay(.spec_hide(spec_hide),.vol_level(vol_level),.clk(clk_pix),.rst(rst_pix_logic),
            .st_valid(st_dst_valid),.st_data(st_dst_data),.vblank_start(vvblank),
            .ref_valid(disp_ref_valid),.ref_slot(disp_ref_slot),.audio_wav(AUDIO_WAV_ENABLE!=0 && ref_audio[19]),
            .x(sp_x),.y(sp_y),.de(sp_de),.hs(sp_hs),.vs(sp_vs),.r(sp_r),.g(sp_g),.b(sp_b),
            .o_de(v_de),.o_hs(v_hs),.o_vs(v_vs),.o_r(v_r),.o_g(v_g),.o_b(v_b));
    end else begin : g_no_osd
        assign spec_hide=1'b0;assign vol_level=5'd16;
        assign v_de=sp_de; assign v_hs=sp_hs; assign v_vs=sp_vs; assign v_r=sp_r; assign v_g=sp_g; assign v_b=sp_b;
    end endgenerate
    // HDMI 编码前将视频输入寄存一拍，缩短像素选择到编码器的组合路径。
    // DE/HS/VS 与 RGB 同步延迟，保持行场结构一致。
    reg o_de,o_hs,o_vs; reg [7:0] o_r,o_g,o_b;
    always @(posedge clk_pix) begin
        if(rst_pix_logic) begin o_de<=1'b0;o_hs<=1'b0;o_vs<=1'b0;o_r<=8'd0;o_g<=8'd0;o_b<=8'd0; end
        else begin o_de<=v_de;o_hs<=v_hs;o_vs<=v_vs;o_r<=v_r;o_g<=v_g;o_b<=v_b; end
    end
    // 三 lane 同一拍、同一时延：lane0=B+{VS,HS}，lane1=G，lane2=R（规格 §10.3）
    // DVI 编码器输出 dvi_sym*，由 HDMI_AUDIO 选择是否直接送入符号配对模块。
    wire [9:0] dvi_sym0,dvi_sym1,dvi_sym2;
    tmds_encoder u_enc0(.clk(clk_pix),.rst(rst_pix_logic),.de(o_de),.c({o_vs,o_hs}),.d(o_b),.q(dvi_sym0));
    tmds_encoder u_enc1(.clk(clk_pix),.rst(rst_pix_logic),.de(o_de),.c(2'b00),.d(o_g),.q(dvi_sym1));
    tmds_encoder u_enc2(.clk(clk_pix),.rst(rst_pix_logic),.de(o_de),.c(2'b00),.d(o_r),.q(dvi_sym2));
    // HDMI_AUDIO=0 时选择 DVI 符号；=1 时由 hdmi_data_island_tx 发送视频及音频数据岛。
    // 发送器从 HS/VS 边沿重建坐标，在消隐期调度 ACR、PCM 样本包、AVI 和 Audio InfoFrame。
    // 音源优先使用已提交的 WAV，缺失或失败时使用固定三角波回退；采样使能为 48kHz。
    // 三条 lane 延迟一致，未被选择的编码路径由综合工具裁去。
    audio_ce_48k u_au_ce(.clk(clk_pix),.rst(rst_pix_logic),.sample_ce(au_ce));
    audio_test_tone u_au_tone(.clk(clk_pix),.rst(rst_pix_logic),.ce(au_ce),.accept(di_accept),.mute(1'b0),.valid(tone_valid),.l(tone_l),.r(tone_r));
    // 两个FIFO的完整实例路径固定为u_audio/u_fifo_a、u_audio/u_fifo_b。
    wav_audio_pipeline #(.ENABLE(AUDIO_WAV_ENABLE),.USE_EG_BRAM(USE_EG_BRAM)) u_audio(
        .arst(domain_arst),.mem_clk(clk_mem),.pix_clk(clk_pix),
        .cmd_valid(au_mem_valid),.cmd_ready(au_mem_ready),.cmd_cancel(au_mem_cancel),.cmd_bank(au_mem_bank),.cmd_words(au_mem_words),.cmd_token(au_mem_token),
        .rsp_valid(au_mem_rsp_valid),.rsp_ready(au_mem_rsp_ready),.rsp_status(au_mem_rsp_status),.rsp_bank(au_mem_rsp_bank),.rsp_token(au_mem_rsp_token),
        .pcm_valid(au_pcm_valid),.pcm_ready(au_pcm_ready),.pcm_addr(au_pcm_addr),.pcm_urgent(au_pcm_urgent),.pcm_budget(au_pcm_budget),
        .pcm_data_valid(au_pcm_data_valid),.pcm_data(phy_rd_data),.pcm_done(au_pcm_done),.pcm_data_ready(au_pcm_data_ready),
        .commit_go(audio_commit_go),.next_audio(audio_commit_data),.audio(ref_audio),.ref_valid(disp_ref_valid),
        .sample_ce(au_ce),.sample_accept(di_accept),.tone_l(tone_l),.tone_r(tone_r),
        .sample_valid(wav_sample_valid),.sample_l(wav_sample_l),.sample_r(wav_sample_r),.underrun(),.eof(),.played(),.fault(),.level_a(),.level_b());
    // 只在菜单和HDMI音频均启用时插入音量；频谱仍连上游 wav/tone 样本，静音不影响柱。
    wire hdmi_sample_valid;wire [15:0] hdmi_sample_l,hdmi_sample_r;
    generate if(OSD_EN!=0 && HDMI_AUDIO!=0) begin : g_volume
        audio_volume #(.USE_EG_DSP(USE_EG_BRAM)) u_volume(.clk(clk_pix),.rst(rst_pix_logic),.level(vol_level),
            .in_valid(AUDIO_WAV_ENABLE ? wav_sample_valid : tone_valid),
            .in_l(AUDIO_WAV_ENABLE ? wav_sample_l : tone_l),.in_r(AUDIO_WAV_ENABLE ? wav_sample_r : tone_r),
            .out_valid(hdmi_sample_valid),.out_l(hdmi_sample_l),.out_r(hdmi_sample_r));
    end else begin : g_volume_bypass
        assign hdmi_sample_valid=AUDIO_WAV_ENABLE ? wav_sample_valid : tone_valid;
        assign hdmi_sample_l=AUDIO_WAV_ENABLE ? wav_sample_l : tone_l;
        assign hdmi_sample_r=AUDIO_WAV_ENABLE ? wav_sample_r : tone_r;
    end endgenerate
    generate if (HDMI_AUDIO!=0) begin : g_hdmi_audio
        wire di_drop,di_locked;
        hdmi_data_island_tx u_hdmi_di(.clk(clk_pix),.rst(rst_pix_logic),.enable(1'b1),
            .de(o_de),.hs(o_hs),.vs(o_vs),.r(o_r),.g(o_g),.b(o_b),
            .s_valid(hdmi_sample_valid),.s_l(hdmi_sample_l),.s_r(hdmi_sample_r),.s_accept(di_accept),
            .sym0(sym0),.sym1(sym1),.sym2(sym2),.drop(di_drop),.locked(di_locked));
    end else begin : g_hdmi_dvi
        assign di_accept=1'b0;
        assign sym0=dvi_sym0; assign sym1=dvi_sym1; assign sym2=dvi_sym2;
    end endgenerate
    wire [79:0] tmds_word; wire tmds_tog;
    tmds_pair_pack u_pair(.clk_pix(clk_pix),.rst_pix(rst_pix_logic),.s0(sym0),.s1(sym1),.s2(sym2),
        .clk_word(clk_word),.rst_word(rst_word_logic),.word(tmds_word),.word_tog(tmds_tog));
    tmds_gearbox20_phy u_hdmi_phy(.clk_gear(clk_gear),.rst_gear(rst_gear_logic),.clk_serial(clk_serial),
        .word(tmds_word),.word_tog(tmds_tog),.pad_p({hdmi_clk_p,hdmi_d_p}));

    // ---------------- LED0（A4）首图前启动错误闪码 ----------------
    // 首图成功响应后（b3 handoff=1），LED0 显示 play_ctrl 的自动模式状态，K2 切换模式。
    // 首图前若 tf_bmp_boot 已放弃（done=1）且仍无有效显示引用，则按错误类别闪 N 下：
    //   N=1 卡初始化失败（error_code=0x1x，已含 CMD0 排空重试与 3 次整体重试）
    //   N=2 FAT32 挂载失败（0x2x）  N=3 找不到 MEDIA 目录（0x3x）  N=4 首图显示响应失败（0x4x）
    //   N=5 装载隔离/看门狗（0xEx） N=6 启动结束但四张 BMP 都没装成（error_code=0）
    // 每下亮 200ms、灭 200ms，N 下之后再灭 1.2s，循环。仍在启动/重试中（done=0）时保持熄灭。
    // 闪码计数和状态全部运行在 sys 域，直接驱动独立状态 LED。
    localparam integer LED_SLOT_CYCLES=10000000;   // 50MHz 下 200ms
    reg [23:0] led_div;      // 200ms 时隙分频
    reg [4:0] led_slot;      // 当前时隙号：0..2N-1 为闪烁段，2N..2N+5 熄灭（N=6 时最大 17，需 5 位）
    reg [2:0] led_code;      // 当前闪码 N（0=不闪）
    reg led_blink;           // 闪码输出寄存
    always @(posedge clk_sys) begin
        if(rst_sys_logic) begin led_div<=0;led_slot<=0;led_code<=0;led_blink<=0;end
        else begin
            if(!boot_done_w || b3_handoff) led_code<=3'd0;
            else case(boot_error_w[7:4])
                4'h1:led_code<=3'd1;
                4'h2:led_code<=3'd2;
                4'h3:led_code<=3'd3;
                4'h4:led_code<=3'd4;
                4'he:led_code<=3'd5;
                default:led_code<=boot_error_w==8'd0 ? 3'd6 : 3'd5;
            endcase
            if(led_code==0) begin led_div<=0;led_slot<=0;led_blink<=0;end
            else begin
                if(led_div==LED_SLOT_CYCLES-1) begin
                    led_div<=0;
                    led_slot<=(led_slot>={1'b0,led_code,1'b0}+5'd5) ? 5'd0 : led_slot+5'd1;
                end else led_div<=led_div+24'd1;
                led_blink<=led_slot<{1'b0,led_code,1'b0} && !led_slot[0];
            end
        end
    end
    assign auto_mode = b3_handoff ? auto_mode_play : led_blink;

    // ---------------- 其余板级输出：未使用的蜂鸣器保持静音 ----------------
    assign buzzer_n = 1'b1;   // 低有效蜂鸣器保持静音。
endmodule
`default_nettype wire
