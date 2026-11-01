`timescale 1ns/1ps
`default_nettype none
// 原生FAT服务与文件提交协调器的实际连接。装载未排空/提交前禁止初始化、挂载及扫描插入。
// ENABLE_THUMBNAILS 默认启用，整机 WAV 配置关闭；文件提交仍需 CRC 和源文件流结束确认。
module fat32_bmp_loader #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    // TIME_W 只传给SD/FAT32服务的预算计数器（默认 32 位）。
    parameter EXT_BMP_FORMATS=0,
    parameter BMP32_EN=1,parameter integer KEY_W=32,parameter integer TIME_W=32,parameter integer FILE_LEN_W=32,parameter integer CARD_SECTOR_W=33,parameter ENABLE_THUMBNAILS=1,ENABLE_WAV=0,ENABLE_AUDIO_CONTROL=0,EXTERNAL_MEMORY=0,USE_EG_BRAM=1,POWER_CYCLES=50000,POLL_CYCLES=50000,INIT_TIMEOUT_CYCLES=50000000,READ_WAIT_CYCLES=5000000,DRAIN_CYCLES=500000,POWER_WAIT_CYCLES=25000,REFRESH_COUNT=8,REFRESH_INTERVAL=937,REFRESH_GRACE=937,
    parameter integer CONTEXT_ENABLE=0,EPOCH_W=12)(
    input wire audio_req_valid,output wire audio_req_ready,input wire audio_req_cancel,audio_req_bank,
    input wire [17:0] audio_req_words,input wire [15:0] audio_req_token,
    output wire audio_rsp_valid,output wire [1:0] audio_rsp_status,output wire audio_rsp_bank,output wire [15:0] audio_rsp_token,
    output wire audio_mem_valid,input wire audio_mem_ready,output wire audio_mem_cancel,audio_mem_bank,
    output wire [17:0] audio_mem_words,output wire [15:0] audio_mem_token,
    input wire audio_mem_rsp_valid,output wire audio_mem_rsp_ready,input wire [1:0] audio_mem_rsp_status,
    input wire audio_mem_rsp_bank,input wire [15:0] audio_mem_rsp_token,
    input wire  clk,
    input wire  mem_clk,
    input wire  arst,
    input wire  init_valid,
    output wire  init_ready,
    input wire [KEY_W-1:0] init_epoch,
    input wire  init_abort_valid,
    output wire  init_abort_ready,
    input wire [KEY_W-1:0] init_abort_epoch,
    output wire  init_done_valid,
    input wire  init_done_ready,
    output wire [KEY_W-1:0] init_done_epoch,
    output wire [31:0] init_done_register,
    output wire [3:0] init_done_code,
    output wire [5:0] init_done_command,
    output wire [7:0] init_done_r1,
    output wire  card_qualified,
    output wire  card_block_addressed,
    output wire  media_fault,
    output wire  card_initialized,
    output wire [31:0] card_last_lba,
    output wire [32:0] card_sector_count,
    input wire  mount_valid,
    output wire  mount_ready,
    input wire [KEY_W-1:0] mount_session,
    input wire [KEY_W-1:0] mount_seq,
    input wire [31:0] mount_budget_cycles,
    input wire  mount_cancel_valid,
    output wire  mount_cancel_ready,
    input wire [KEY_W-1:0] mount_cancel_session,
    input wire [KEY_W-1:0] mount_cancel_seq,
    output wire  mount_busy,
    output wire  mount_result_valid,
    input wire  mount_result_ready,
    output wire [KEY_W-1:0] mount_result_session,
    output wire [KEY_W-1:0] mount_result_seq,
    output wire [31:0] mount_result_elapsed,
    output wire [3:0] mount_result_code,
    output wire [7:0] mount_result_detail,
    output wire  mounted,
    output wire [32:0] partition_end,
    output wire [32:0] volume_end,
    output wire [31:0] fat_lba,
    output wire [31:0] data_lba,
    output wire [31:0] root_lba,
    output wire [31:0] fat_sectors,
    output wire [31:0] root_cluster,
    output wire [27:0] cluster_count,
    output wire [2:0] cluster_shift,
    output wire [3:0] active_fat,
    input wire  scan_valid,
    output wire  scan_ready,
    input wire [KEY_W-1:0] scan_session,
    input wire [KEY_W-1:0] scan_seq,
    input wire [31:0] scan_budget_cycles,
    input wire [87:0] scan_target_name,
    input wire [31:0] scan_start_cluster,
    input wire  scan_want_dir,
    input wire [5:0] scan_sector_limit,
    input wire  scan_cancel_valid,
    output wire  scan_cancel_ready,
    input wire [KEY_W-1:0] scan_cancel_session,
    input wire [KEY_W-1:0] scan_cancel_seq,
    output wire  scan_busy,
    output wire  scan_result_valid,
    input wire  scan_result_ready,
    output wire [KEY_W-1:0] scan_result_session,
    output wire [KEY_W-1:0] scan_result_seq,
    output wire [31:0] scan_result_elapsed,
    output wire [3:0] scan_result_code,
    output wire [7:0] scan_result_detail,
    output wire [31:0] hit_cluster,
    output wire [31:0] hit_size,
    output wire  spi_cs_n,
    output wire  spi_sclk,
    output wire  spi_mosi,
    input wire  spi_miso,
    input wire  req_valid,
    output wire  req_ready,
    input wire [KEY_W-1:0] req_session,
    input wire [KEY_W-1:0] req_seq,
    input wire [31:0] req_cluster,
    input wire [31:0] req_length,
    input wire [31:0] req_slot_gen,
    input wire [1:0] req_slot,
    input wire req_wav,input wire [1:0] wav_bank_writable,
    input wire [3:0] slot_writable,
    input wire [31:0] req_src_budget,
    input wire [31:0] req_mem_budget,
    input wire  req_crc_enable,
    input wire [31:0] req_crc,
    input wire  cancel_valid,
    input wire [KEY_W-1:0] cancel_session,
    input wire [KEY_W-1:0] cancel_seq,
    output wire  busy,
    output wire  quarantined,
    output wire [3:0] slot_reserved,
    output wire [3:0] slot_valid,
    output wire  commit_valid,
    input wire  commit_ready,
    output wire [KEY_W-1:0] active_session,
    output wire [KEY_W-1:0] active_seq,
    output wire [31:0] active_slot_gen,
    output wire [31:0] actual_crc,
    output wire [1:0] active_slot,
    output wire  result_valid,
    input wire  result_ready,
    output wire [7:0] result_code,
    output wire [18:0] written_words,
    output wire [13:0] thumbnail_words,
    // EXTERNAL_MEMORY=1时接产品唯一共享控制器（mem_clk域）；禁止并接命令总线。
    // 已接受BL8在取消时仍连续供满八拍；完成须来自真实PRE/done，不撤销在途所有权。
    output wire load_valid,input wire load_ready,output wire [20:0] load_addr,
    output wire [3:0] load_count,output wire [31:0] load_budget,
    input wire load_beat,load_done,load_done_was_write,
    output wire  wr_data_valid,
    output wire [31:0] wr_data,
    output wire  sd_cs_n,
    output wire  sd_ras_n,
    output wire  sd_cas_n,
    output wire  sd_we_n,
    output wire  sd_cke,
    output wire [10:0] sd_addr,
    output wire [1:0] sd_ba
);
    wire  file_valid;
    wire  file_ready;
    wire [KEY_W-1:0] file_session;
    wire [KEY_W-1:0] file_seq;
    wire [31:0] file_budget_cycles;
    wire [31:0] file_start_cluster;
    wire [31:0] file_size;
    wire  file_cancel_valid;
    wire  file_cancel_ready;
    wire [KEY_W-1:0] file_cancel_session;
    wire [KEY_W-1:0] file_cancel_seq;
    wire  file_busy;
    wire  file_result_valid;
    wire  file_result_ready;
    wire [KEY_W-1:0] file_result_session;
    wire [KEY_W-1:0] file_result_seq;
    wire [31:0] file_result_elapsed;
    wire [31:0] file_result_count;
    wire [3:0] file_result_code;
    wire [7:0] file_result_detail;
    wire  file_sink_abort_req;
    wire  file_sink_abort_ack;
    wire [KEY_W-1:0] file_sink_abort_session;
    wire [KEY_W-1:0] file_sink_abort_seq;
    wire [KEY_W-1:0] file_sink_ack_session;
    wire [KEY_W-1:0] file_sink_ack_seq;
    wire  out_valid;
    wire  out_ready;
    wire [7:0] out_data;
    wire  out_last;
    wire [1:0] out_user;
    wire [KEY_W-1:0] out_session;
    wire [KEY_W-1:0] out_seq;
    wire  dst_stop_req;
    wire  dst_stop_ack;

    // CONTEXT_ENABLE=0 直接服务整文件请求；=1 保存本次文件请求的参数，并按块读取和装载。
    // 中间事件用于恢复下一块，只有最终事件交给文件提交器。
    wire legacy_file_ready,legacy_file_cancel_ready,legacy_file_busy,legacy_file_result_valid;
    wire [KEY_W-1:0] legacy_file_result_session,legacy_file_result_seq;
    wire [31:0] legacy_file_result_count;
    wire [3:0] legacy_file_result_code;
    reg ctx_lease;
    reg [KEY_W-1:0] ctx_session_q,ctx_seq_q;
    reg [31:0] ctx_cluster_q,ctx_size_q,ctx_budget_q;
    wire ctx_busy,ctx_quiescent;
    wire [3:0] ctx_record_valid,ctx_accepted;
    wire ctx_owner_valid,ctx_owner_ready;
    // 产品只接固定的第0号客户端，这里不需要动态安装目标路由。
    assign ctx_owner_ready=1'b1;
    wire [1:0] ctx_owner_ctx;
    wire [KEY_W-1:0] ctx_owner_session,ctx_owner_seq,ctx_owner_ticket;
    wire [EPOCH_W-1:0] ctx_owner_epoch;
    wire ctx_receipt_valid,ctx_receipt_ready;
    wire [KEY_W-1:0] ctx_receipt_ticket;
    wire ctx_event_valid,ctx_event_ready,ctx_event_more;
    wire [1:0] ctx_event_ctx;
    wire [KEY_W-1:0] ctx_event_session,ctx_event_seq,ctx_event_ticket;
    wire [EPOCH_W-1:0] ctx_event_epoch;
    wire [3:0] ctx_event_code;
    wire [7:0] ctx_event_detail;
    wire [31:0] ctx_event_count,ctx_event_quantum_count;
    wire [EPOCH_W-1:0] ctx_media_epoch=init_epoch;
    wire ctx_final_event=ctx_event_valid && (!ctx_event_more || ctx_event_code!=0);
    assign ctx_event_ready=CONTEXT_ENABLE!=0 && (ctx_final_event ? file_result_ready : 1'b1);
    // 挂载还没收尾时不能抢先收文件，否则上层看到的装载起点会提前。
    // init/scan等控制请求排队时，也先把它们处理完。
    wire ctx_geometry_ready=mounted && !mount_busy && !init_valid && !scan_valid;
    assign file_ready=CONTEXT_ENABLE!=0 ? ctx_geometry_ready && !ctx_lease && !ctx_busy : legacy_file_ready;
    assign file_cancel_ready=CONTEXT_ENABLE!=0 ? ctx_lease : legacy_file_cancel_ready;
    assign file_busy=CONTEXT_ENABLE!=0 ? ctx_busy : legacy_file_busy;
    assign file_result_valid=CONTEXT_ENABLE!=0 ? ctx_final_event : legacy_file_result_valid;
    assign file_result_session=CONTEXT_ENABLE!=0 ? ctx_event_session : legacy_file_result_session;
    assign file_result_seq=CONTEXT_ENABLE!=0 ? ctx_event_seq : legacy_file_result_seq;
    assign file_result_count=CONTEXT_ENABLE!=0 ? ctx_event_count : legacy_file_result_count;
    assign file_result_code=CONTEXT_ENABLE!=0 ? ctx_event_code : legacy_file_result_code;
    // 复位约束：domain_arst只允许进复位同步器；本层用于保存文件任务的寄存器使用同步后的 rst，
    // 与u_storage/u_load内部的reset_sync同型，不新增任何异步复位业务锥。
    wire ctx_rst,ctx_rst_logic;
    reset_sync u_ctx_reset(.clk(clk),.arst(arst),.rst(ctx_rst),.logic_rst(ctx_rst_logic));
    always @(posedge clk) begin
        if(ctx_rst) begin ctx_lease<=0;ctx_session_q<=0;ctx_seq_q<=0;ctx_cluster_q<=0;ctx_size_q<=0;ctx_budget_q<=0;end
        else begin
            if(CONTEXT_ENABLE!=0 && file_valid && file_ready) begin
                ctx_lease<=1;ctx_session_q<=file_session;ctx_seq_q<=file_seq;
                ctx_cluster_q<=file_start_cluster;ctx_size_q<=file_size;ctx_budget_q<=file_budget_cycles;
            end
            if(CONTEXT_ENABLE!=0 && ctx_final_event && ctx_event_ready) ctx_lease<=0;
        end
    end
    // 此通道只传当前文件任务的 ticket；session/epoch 保存在任务寄存器中，任务结束前不能切换请求方。
    wire quantum_receipt_valid,quantum_receipt_ready;
    wire [KEY_W-1:0] quantum_receipt_ticket;
    cdc_mailbox #(.DATA_WIDTH(KEY_W)) u_quantum_receipt(
        .src_clk(mem_clk),.dst_clk(clk),.arst(arst),
        .src_valid(quantum_receipt_valid),.src_ready(quantum_receipt_ready),.src_data(quantum_receipt_ticket),
        .src_active(),.src_busy(),.src_done(),
        .dst_valid(ctx_receipt_valid),.dst_ready(ctx_receipt_ready),.dst_data(ctx_receipt_ticket),.dst_active());
    wire init_ready_raw,mount_ready_raw,scan_ready_raw,load_req_ready;
    // 控制请求优先，排队的文件valid不能阻塞完成初始化/挂载所需的前置请求。
    wire control_pending=init_valid || mount_valid || scan_valid;
    assign req_ready=load_req_ready && !control_pending;
    assign init_ready=init_ready_raw && !busy;
    assign mount_ready=mount_ready_raw && !busy;
    assign scan_ready=scan_ready_raw && !busy;
    fat32_storage_services #(.KEY_W(KEY_W),.TIME_W(TIME_W),.FILE_LEN_W(FILE_LEN_W),.CARD_SECTOR_W(CARD_SECTOR_W),
        .CHUNK_ENABLE(CONTEXT_ENABLE),.CONTEXT_ENABLE(CONTEXT_ENABLE),.EPOCH_W(EPOCH_W),
        .USE_EG_BRAM(USE_EG_BRAM),.POWER_CYCLES(POWER_CYCLES),.POLL_CYCLES(POLL_CYCLES),
        .INIT_TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES),.READ_WAIT_CYCLES(READ_WAIT_CYCLES)) u_storage(
        .clk(clk),
        .dst_clk(mem_clk),
        .arst(arst),
        .init_valid(init_valid && !busy),
        .init_ready(init_ready_raw),
        .init_epoch(init_epoch),
        .init_abort_valid(init_abort_valid),
        .init_abort_ready(init_abort_ready),
        .init_abort_epoch(init_abort_epoch),
        .init_done_valid(init_done_valid),
        .init_done_ready(init_done_ready),
        .init_done_epoch(init_done_epoch),
        .init_done_register(init_done_register),
        .init_done_code(init_done_code),
        .init_done_command(init_done_command),
        .init_done_r1(init_done_r1),
        .card_qualified(card_qualified),
        .card_block_addressed(card_block_addressed),
        .media_fault(media_fault),
        .card_initialized(card_initialized),
        .card_last_lba(card_last_lba),
        .card_sector_count(card_sector_count),
        .mount_valid(mount_valid && !busy),
        .mount_ready(mount_ready_raw),
        .mount_session(mount_session),
        .mount_seq(mount_seq),
        .mount_budget_cycles(mount_budget_cycles),
        .mount_cancel_valid(mount_cancel_valid),
        .mount_cancel_ready(mount_cancel_ready),
        .mount_cancel_session(mount_cancel_session),
        .mount_cancel_seq(mount_cancel_seq),
        .mount_busy(mount_busy),
        .mount_result_valid(mount_result_valid),
        .mount_result_ready(mount_result_ready),
        .mount_result_session(mount_result_session),
        .mount_result_seq(mount_result_seq),
        .mount_result_elapsed(mount_result_elapsed),
        .mount_result_code(mount_result_code),
        .mount_result_detail(mount_result_detail),
        .mounted(mounted),
        .partition_end(partition_end),
        .volume_end(volume_end),
        .fat_lba(fat_lba),
        .data_lba(data_lba),
        .root_lba(root_lba),
        .fat_sectors(fat_sectors),
        .root_cluster(root_cluster),
        .cluster_count(cluster_count),
        .cluster_shift(cluster_shift),
        .active_fat(active_fat),
        .scan_valid(scan_valid && !busy),
        .scan_ready(scan_ready_raw),
        .scan_session(scan_session),
        .scan_seq(scan_seq),
        .scan_budget_cycles(scan_budget_cycles),
        .scan_target_name(scan_target_name),
        .scan_start_cluster(scan_start_cluster),
        .scan_want_dir(scan_want_dir),
        .scan_sector_limit(scan_sector_limit),
        .scan_cancel_valid(scan_cancel_valid),
        .scan_cancel_ready(scan_cancel_ready),
        .scan_cancel_session(scan_cancel_session),
        .scan_cancel_seq(scan_cancel_seq),
        .scan_busy(scan_busy),
        .scan_result_valid(scan_result_valid),
        .scan_result_ready(scan_result_ready),
        .scan_result_session(scan_result_session),
        .scan_result_seq(scan_result_seq),
        .scan_result_elapsed(scan_result_elapsed),
        .scan_result_code(scan_result_code),
        .scan_result_detail(scan_result_detail),
        .hit_cluster(hit_cluster),
        .hit_size(hit_size),
        .file_valid(CONTEXT_ENABLE==0 && file_valid),
        .file_ready(legacy_file_ready),
        .file_session(file_session),
        .file_seq(file_seq),
        .file_budget_cycles(file_budget_cycles),
        .file_start_cluster(file_start_cluster),
        .file_size(file_size),
        // 上下文模式由 ctx_limit 提供 512B 分块长度；直连整文件模式禁用 CHUNK_ENABLE。
        // 未使用的分块输入显式连接常量，序号连接当前文件键。
        .file_chunked(1'b0),.file_start_sector(6'd0),.file_chunk_limit(13'd512),
        .file_start_extent_seq(file_seq),
        .file_cancel_valid(CONTEXT_ENABLE==0 && file_cancel_valid),
        .file_cancel_ready(legacy_file_cancel_ready),
        .file_cancel_session(file_cancel_session),
        .file_cancel_seq(file_cancel_seq),
        .file_busy(legacy_file_busy),
        .file_result_valid(legacy_file_result_valid),
        .file_result_ready(CONTEXT_ENABLE==0 && file_result_ready),
        .file_result_session(legacy_file_result_session),
        .file_result_seq(legacy_file_result_seq),
        .file_result_elapsed(file_result_elapsed),
        .file_result_count(legacy_file_result_count),
        .file_result_code(legacy_file_result_code),
        .file_result_detail(file_result_detail),
        .file_sink_abort_req(file_sink_abort_req),
        .file_sink_abort_ack(file_sink_abort_ack),
        .file_sink_abort_session(file_sink_abort_session),
        .file_sink_abort_seq(file_sink_abort_seq),
        .file_sink_ack_session(file_sink_ack_session),
        .file_sink_ack_seq(file_sink_ack_seq),
        .ctx_service_enabled(1'b1),.quiesce(init_valid || mount_valid),.media_epoch(ctx_media_epoch),
        .ctx_valid({3'd0,ctx_lease}),.demand({3'd0,ctx_lease}),.credit({3'd0,ctx_lease}),
        .critical_mask(4'd0),.low_mask(4'd0),.necessary_mask({3'd0,ctx_lease}),.background_mask(4'd0),
        .cancel_mask({3'd0,file_cancel_valid}),.deadline_mask(4'd0),
        .ctx_session({{(3*KEY_W){1'b0}},ctx_session_q}),.ctx_seq({{(3*KEY_W){1'b0}},ctx_seq_q}),
        .ctx_epoch({4{ctx_media_epoch}}),
        .ctx_cluster({96'd0,ctx_cluster_q}),.ctx_size({96'd0,ctx_size_q}),.ctx_budget({96'd0,ctx_budget_q}),
        .ctx_limit({39'd0,13'd512}),.ctx_busy(ctx_busy),.quiescent(ctx_quiescent),
        .record_valid(ctx_record_valid),.ctx_accepted(ctx_accepted),
        .owner_valid(ctx_owner_valid),.owner_ready(ctx_owner_ready),.owner_ctx(ctx_owner_ctx),
        .owner_session(ctx_owner_session),.owner_seq(ctx_owner_seq),
        .owner_ticket(ctx_owner_ticket),.owner_epoch(ctx_owner_epoch),
        .installed_valid(1'b1),.installed_ctx(ctx_owner_ctx),
        .installed_session(ctx_owner_session),.installed_seq(ctx_owner_seq),
        .installed_ticket(ctx_owner_ticket),.installed_epoch(ctx_owner_epoch),
        .receipt_valid(ctx_receipt_valid),.receipt_ready(ctx_receipt_ready),.receipt_ctx(2'd0),
        .receipt_session(ctx_owner_session),.receipt_seq(ctx_owner_seq),
        .receipt_ticket(ctx_receipt_ticket),.receipt_epoch(ctx_owner_epoch),
        .sink_ack_ctx(ctx_owner_ctx),.sink_ack_ticket(ctx_owner_ticket),.sink_ack_epoch(ctx_owner_epoch),
        .sink_abort_ctx(),.sink_abort_ticket(),.sink_abort_epoch(),
        .event_valid(ctx_event_valid),.event_ready(ctx_event_ready),.event_ctx(ctx_event_ctx),
        .event_session(ctx_event_session),.event_seq(ctx_event_seq),.event_ticket(ctx_event_ticket),
        .event_epoch(ctx_event_epoch),.event_code(ctx_event_code),.event_detail(ctx_event_detail),
        .event_more(ctx_event_more),.event_count(ctx_event_count),.event_quantum_count(ctx_event_quantum_count),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data(out_data),
        .out_last(out_last),
        .out_user(out_user),
        .out_session(out_session),
        .out_seq(out_seq),
        .dst_stop_req(dst_stop_req),
        .dst_stop_ack(dst_stop_ack),
        .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso));
    bmp_file_commit #(.BMP32_EN(BMP32_EN),.EXT_BMP_FORMATS(EXT_BMP_FORMATS),.KEY_W(KEY_W),.ENABLE_THUMBNAILS(ENABLE_THUMBNAILS),.ENABLE_WAV(ENABLE_WAV),.ENABLE_AUDIO_CONTROL(ENABLE_AUDIO_CONTROL),.EXTERNAL_MEMORY(EXTERNAL_MEMORY),.DRAIN_CYCLES(DRAIN_CYCLES),.POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),.REFRESH_COUNT(REFRESH_COUNT),
        .REFRESH_INTERVAL(REFRESH_INTERVAL),.REFRESH_GRACE(REFRESH_GRACE)) u_load(
                .audio_req_valid(audio_req_valid),.audio_req_ready(audio_req_ready),.audio_req_cancel(audio_req_cancel),.audio_req_bank(audio_req_bank),
        .audio_req_words(audio_req_words),.audio_req_token(audio_req_token),
        .audio_rsp_valid(audio_rsp_valid),.audio_rsp_status(audio_rsp_status),.audio_rsp_bank(audio_rsp_bank),.audio_rsp_token(audio_rsp_token),
        .audio_mem_valid(audio_mem_valid),.audio_mem_ready(audio_mem_ready),.audio_mem_cancel(audio_mem_cancel),.audio_mem_bank(audio_mem_bank),
        .audio_mem_words(audio_mem_words),.audio_mem_token(audio_mem_token),
        .audio_mem_rsp_valid(audio_mem_rsp_valid),.audio_mem_rsp_ready(audio_mem_rsp_ready),.audio_mem_rsp_status(audio_mem_rsp_status),
        .audio_mem_rsp_bank(audio_mem_rsp_bank),.audio_mem_rsp_token(audio_mem_rsp_token),
.clk(clk),
        .mem_clk(mem_clk),
        .arst(arst),
        .req_valid(req_valid && !control_pending),
        .req_ready(load_req_ready),
        .req_session(req_session),
        .req_seq(req_seq),
        .req_cluster(req_cluster),
        .req_length(req_length),
        .req_slot_gen(req_slot_gen),
        .req_slot(req_slot),.req_wav(req_wav),.wav_bank_writable(wav_bank_writable),
        .slot_writable(slot_writable),
        .req_src_budget(req_src_budget),
        .req_mem_budget(req_mem_budget),
        .req_crc_enable(req_crc_enable),
        .req_crc(req_crc),
        .cancel_valid(cancel_valid),
        .cancel_session(cancel_session),
        .cancel_seq(cancel_seq),
        .busy(busy),
        .quarantined(quarantined),
        .slot_reserved(slot_reserved),
        .slot_valid(slot_valid),
        .commit_valid(commit_valid),
        .commit_ready(commit_ready),
        .active_session(active_session),
        .active_seq(active_seq),
        .active_slot_gen(active_slot_gen),
        .actual_crc(actual_crc),
        .active_slot(active_slot),
        .result_valid(result_valid),
        .result_ready(result_ready),
        .result_code(result_code),
        .written_words(written_words),
        .thumbnail_words(thumbnail_words),
        .file_valid(file_valid),
        .file_ready(file_ready),
        .file_session(file_session),
        .file_seq(file_seq),
        .file_start_cluster(file_start_cluster),
        .file_size(file_size),
        .file_budget_cycles(file_budget_cycles),
        .file_cancel_valid(file_cancel_valid),
        .file_cancel_ready(file_cancel_ready),
        .file_cancel_session(file_cancel_session),
        .file_cancel_seq(file_cancel_seq),
        .file_busy(file_busy),
        .file_result_valid(file_result_valid),
        .file_result_ready(file_result_ready),
        .file_result_session(file_result_session),
        .file_result_seq(file_result_seq),
        .file_result_count(file_result_count),
        .file_result_code(file_result_code),
        .file_sink_abort_req(file_sink_abort_req),
        .file_sink_abort_session(file_sink_abort_session),
        .file_sink_abort_seq(file_sink_abort_seq),
        .file_sink_abort_ack(file_sink_abort_ack),
        .file_sink_ack_session(file_sink_ack_session),
        .file_sink_ack_seq(file_sink_ack_seq),
        .quantum_receipt_valid(quantum_receipt_valid),.quantum_receipt_ready(quantum_receipt_ready),
        .quantum_receipt_ticket(quantum_receipt_ticket),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data(out_data),
        .out_last(out_last),
        .out_user(out_user),
        .out_session(out_session),
        .out_seq(out_seq),
        .dst_stop_req(dst_stop_req),
        .dst_stop_ack(dst_stop_ack),
        .load_valid(load_valid),.load_ready(load_ready),.load_addr(load_addr),
        .load_count(load_count),.load_budget(load_budget),.load_beat(load_beat),
        .load_done(load_done),.load_done_was_write(load_done_was_write),
        .wr_data_valid(wr_data_valid),
        .wr_data(wr_data),
        .sd_cs_n(sd_cs_n),
        .sd_ras_n(sd_ras_n),
        .sd_cas_n(sd_cas_n),
        .sd_we_n(sd_we_n),
        .sd_cke(sd_cke),
        .sd_addr(sd_addr),
        .sd_ba(sd_ba));
endmodule
`default_nettype wire
