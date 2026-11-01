`timescale 1ns/1ps
`default_nettype none

// FAT32存储服务顶层：一个共享extent实例同时服务挂载闭环与目录扫描控制模块。
// 挂载事务空闲时才把extent通路借给扫描；初始化独占期两者都被拒绝。
// dst域字节消费与停止确认仍由上层真实客户端驱动；内部无假ready/假ack。
module fat32_storage_services #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32,
    // 挂载/链主控内部预算计数位宽；默认32，产品由tf_bmp_boot传26。
    parameter integer TIME_W=32,
    // FILE_LEN_W 给链主控（默认32）；CARD_SECTOR_W 给CSD容量上限（默认33=不加额外上限）。
    parameter integer FILE_LEN_W=32,parameter integer CARD_SECTOR_W=33,
    // 上下文接口默认关闭，关闭时忽略分块恢复字段并使用整文件服务。
    parameter integer CHUNK_ENABLE=0,AUTO_CHUNK=0,AUTO_CHUNK_BYTES=4096,
    parameter integer CONTEXT_ENABLE=0,EPOCH_W=12,
    parameter USE_EG_BRAM=1,POWER_CYCLES=50000,POLL_CYCLES=50000,
    parameter INIT_TIMEOUT_CYCLES=50000000,READ_WAIT_CYCLES=5000000
)(
    input wire clk,dst_clk,arst,
    input wire init_valid,output wire init_ready,input wire [KEY_W-1:0] init_epoch,
    input wire init_abort_valid,output wire init_abort_ready,input wire [KEY_W-1:0] init_abort_epoch,
    output wire init_done_valid,input wire init_done_ready,
    output wire [KEY_W-1:0] init_done_epoch, output wire [31:0] init_done_register,
    output wire [3:0] init_done_code,output wire [5:0] init_done_command,
    output wire [7:0] init_done_r1,
    output wire card_qualified,card_block_addressed,media_fault,card_initialized,
    output wire [31:0] card_last_lba,output wire [32:0] card_sector_count,

    input wire mount_valid,output wire mount_ready,
    input wire [KEY_W-1:0] mount_session,mount_seq, input wire [31:0] mount_budget_cycles,
    input wire mount_cancel_valid,output wire mount_cancel_ready,
    input wire [KEY_W-1:0] mount_cancel_session,mount_cancel_seq,
    output wire mount_busy,mount_result_valid,input wire mount_result_ready,
    output wire [KEY_W-1:0] mount_result_session,mount_result_seq, output wire [31:0] mount_result_elapsed,
    output wire [3:0] mount_result_code,output wire [7:0] mount_result_detail,
    output wire mounted,
    output wire [32:0] partition_end,volume_end,
    output wire [31:0] fat_lba,data_lba,root_lba,fat_sectors,root_cluster,
    output wire [27:0] cluster_count,
    output wire [2:0] cluster_shift,output wire [3:0] active_fat,

    input wire scan_valid,output wire scan_ready,
    input wire [KEY_W-1:0] scan_session,scan_seq, input wire [31:0] scan_budget_cycles,
    input wire [87:0] scan_target_name,input wire [31:0] scan_start_cluster,
    input wire scan_want_dir,input wire [5:0] scan_sector_limit,
    input wire scan_cancel_valid,output wire scan_cancel_ready,
    input wire [KEY_W-1:0] scan_cancel_session,scan_cancel_seq,
    output wire scan_busy,scan_result_valid,input wire scan_result_ready,
    output wire [KEY_W-1:0] scan_result_session,scan_result_seq, output wire [31:0] scan_result_elapsed,
    output wire [3:0] scan_result_code,output wire [7:0] scan_result_detail,
    output wire [31:0] hit_cluster,hit_size,

    input wire file_valid,output wire file_ready,
    input wire [KEY_W-1:0] file_session,file_seq, input wire [31:0] file_budget_cycles,
    input wire [31:0] file_start_cluster,file_size,
    input wire file_chunked,input wire [5:0] file_start_sector,input wire [12:0] file_chunk_limit,
    input wire [KEY_W-1:0] file_start_extent_seq,
    input wire file_cancel_valid,output wire file_cancel_ready,
    input wire [KEY_W-1:0] file_cancel_session,file_cancel_seq,
    output wire file_busy,file_result_valid,input wire file_result_ready,
    output wire [KEY_W-1:0] file_result_session,file_result_seq, output wire [31:0] file_result_elapsed,
    output wire [31:0] file_result_count,
    output wire [3:0] file_result_code,output wire [7:0] file_result_detail,
    output wire file_result_more,output wire [31:0] file_checkpoint_cluster,
    output wire [5:0] file_checkpoint_sector,output wire [31:0] file_checkpoint_remaining,
    output wire [KEY_W-1:0] file_checkpoint_extent_seq,
    output wire file_sink_abort_req,input wire file_sink_abort_ack,
    output wire [KEY_W-1:0] file_sink_abort_session,file_sink_abort_seq,
    input wire [KEY_W-1:0] file_sink_ack_session,file_sink_ack_seq,

    input wire ctx_service_enabled,quiesce,
    input wire [EPOCH_W-1:0] media_epoch,
    input wire [3:0] ctx_valid,demand,credit,critical_mask,low_mask,necessary_mask,background_mask,
    input wire [3:0] cancel_mask,deadline_mask,
    input wire [4*KEY_W-1:0] ctx_session,ctx_seq,
    input wire [4*EPOCH_W-1:0] ctx_epoch,
    input wire [127:0] ctx_cluster,ctx_size,ctx_budget,
    input wire [51:0] ctx_limit,
    output wire ctx_busy,quiescent,
    output wire [3:0] record_valid,ctx_accepted,
    output wire owner_valid,input wire owner_ready,
    output wire [1:0] owner_ctx,
    output wire [KEY_W-1:0] owner_session,owner_seq,owner_ticket,
    output wire [EPOCH_W-1:0] owner_epoch,
    input wire installed_valid,
    input wire [1:0] installed_ctx,
    input wire [KEY_W-1:0] installed_session,installed_seq,installed_ticket,
    input wire [EPOCH_W-1:0] installed_epoch,
    input wire receipt_valid,output wire receipt_ready,
    input wire [1:0] receipt_ctx,
    input wire [KEY_W-1:0] receipt_session,receipt_seq,receipt_ticket,
    input wire [EPOCH_W-1:0] receipt_epoch,
    input wire [1:0] sink_ack_ctx,
    input wire [KEY_W-1:0] sink_ack_ticket,
    input wire [EPOCH_W-1:0] sink_ack_epoch,
    output wire [1:0] sink_abort_ctx,
    output wire [KEY_W-1:0] sink_abort_ticket,
    output wire [EPOCH_W-1:0] sink_abort_epoch,
    output wire event_valid,input wire event_ready,
    output wire [1:0] event_ctx,
    output wire [KEY_W-1:0] event_session,event_seq,event_ticket,
    output wire [EPOCH_W-1:0] event_epoch,
    output wire [3:0] event_code,output wire [7:0] event_detail,
    output wire event_more,
    output wire [31:0] event_count,event_quantum_count,

    output wire out_valid,input wire out_ready,output wire [7:0] out_data,output wire out_last,
    output wire [1:0] out_user,output wire [KEY_W-1:0] out_session,out_seq,
    output wire dst_stop_req,input wire dst_stop_ack,
    output wire spi_cs_n,spi_sclk,spi_mosi,input wire spi_miso
);
    // 服务转发总线：目录与文件控制模块按完整extent事务仲裁，共享挂载器内唯一extent。
    wire svc_req_valid,svc_req_ready,svc_cancel_valid,svc_cancel_ready;
    wire svc_result_valid,svc_result_ready;
    wire svc_tap_valid,svc_tap_sector_last;
    wire [7:0] svc_tap_data;wire [8:0] svc_tap_index;
    wire [KEY_W-1:0] svc_req_session,svc_req_seq; wire [31:0] svc_req_lba,svc_req_budget_cycles;
    wire [KEY_W-1:0] svc_cancel_session,svc_cancel_seq;
    wire [KEY_W-1:0] svc_result_session,svc_result_seq; wire [31:0] svc_result_count,svc_result_elapsed;
    wire [KEY_W-1:0] svc_tap_session,svc_tap_seq;
    wire [3:0] svc_req_sectors,svc_result_code;
    wire [12:0] svc_req_byte_count;wire [1:0] svc_req_user;
    wire [7:0] svc_result_detail;
    wire ext_result_valid,ext_result_ready;
    wire [KEY_W-1:0] ext_result_session,ext_result_seq; wire [31:0] ext_result_count,ext_result_elapsed;
    wire [3:0] ext_result_code;wire [7:0] ext_result_detail;
    wire mount_init_ready,mount_ready_int;

    wire chain_req_valid,chain_ext_cancel_valid,chain_ext_result_ready;
    wire [KEY_W-1:0] chain_req_session,chain_req_seq; wire [31:0] chain_req_lba,chain_req_budget;
    wire [KEY_W-1:0] chain_cancel_session,chain_cancel_seq;
    wire [3:0] chain_req_sectors;wire [12:0] chain_req_bytes;wire [1:0] chain_req_user;

    // 合一链控制模块内部以单FSM保证目录/文件两种事务互斥，svc通路直通转发：
    // 无需服务授予/所有者跟踪，挂载器仍只在IDLE授予extent。
    assign svc_req_valid=chain_req_valid;
    assign svc_req_session=chain_req_session;
    assign svc_req_seq=chain_req_seq;
    assign svc_req_lba=chain_req_lba;
    assign svc_req_budget_cycles=chain_req_budget;
    assign svc_req_sectors=chain_req_sectors;
    assign svc_req_byte_count=chain_req_bytes;
    assign svc_req_user=chain_req_user;
    assign svc_cancel_valid=chain_ext_cancel_valid;
    assign svc_cancel_session=chain_cancel_session;
    assign svc_cancel_seq=chain_cancel_seq;
    assign svc_result_ready=chain_ext_result_ready;

    // 初始化或重新挂载不能插入目录/文件的多extent事务间隙。
    assign init_ready=mount_init_ready && !scan_busy && !file_busy;
    assign mount_ready=mount_ready_int && !scan_busy && !file_busy;

    fat32_mount_storage #(.KEY_W(KEY_W),.TIME_W(TIME_W),.CARD_SECTOR_W(CARD_SECTOR_W),.USE_EG_BRAM(USE_EG_BRAM),.POWER_CYCLES(POWER_CYCLES),
        .POLL_CYCLES(POLL_CYCLES),.INIT_TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES),
        .READ_WAIT_CYCLES(READ_WAIT_CYCLES)) u_mount(
        .clk(clk),.dst_clk(dst_clk),.arst(arst),
        .init_valid(init_valid && !scan_busy && !file_busy),.init_ready(mount_init_ready),.init_epoch(init_epoch),
        .init_abort_valid(init_abort_valid),.init_abort_ready(init_abort_ready),
        .init_abort_epoch(init_abort_epoch),.init_done_valid(init_done_valid),
        .init_done_ready(init_done_ready),.init_done_epoch(init_done_epoch),
        .init_done_register(init_done_register),.init_done_code(init_done_code),
        .init_done_command(init_done_command),.init_done_r1(init_done_r1),
        .card_qualified(card_qualified),.card_block_addressed(card_block_addressed),
        .media_fault(media_fault),.card_initialized(card_initialized),
        .card_last_lba(card_last_lba),.card_sector_count(card_sector_count),
        .mount_valid(mount_valid && !scan_busy && !file_busy),.mount_ready(mount_ready_int),
        .mount_session(mount_session),.mount_seq(mount_seq),
        .mount_budget_cycles(mount_budget_cycles),
        .cancel_valid(mount_cancel_valid),.cancel_ready(mount_cancel_ready),
        .cancel_session(mount_cancel_session),.cancel_seq(mount_cancel_seq),
        .mount_busy(mount_busy),.mount_result_valid(mount_result_valid),
        .mount_result_ready(mount_result_ready),.mount_result_session(mount_result_session),
        .mount_result_seq(mount_result_seq),.mount_result_elapsed(mount_result_elapsed),
        .mount_result_code(mount_result_code),.mount_result_detail(mount_result_detail),
        .mounted(mounted),.partition_end(partition_end),.volume_end(volume_end),
        .fat_lba(fat_lba),.data_lba(data_lba),.root_lba(root_lba),
        .fat_sectors(fat_sectors),.root_cluster(root_cluster),
        .cluster_count(cluster_count),.cluster_shift(cluster_shift),.active_fat(active_fat),
        .svc_req_valid(svc_req_valid),.svc_req_ready(svc_req_ready),
        .svc_req_session(svc_req_session),.svc_req_seq(svc_req_seq),.svc_req_lba(svc_req_lba),
        .svc_req_budget_cycles(svc_req_budget_cycles),.svc_req_sectors(svc_req_sectors),
        .svc_req_byte_count(svc_req_byte_count),.svc_req_user(svc_req_user),
        .svc_cancel_valid(svc_cancel_valid),.svc_cancel_ready(svc_cancel_ready),
        .svc_cancel_session(svc_cancel_session),.svc_cancel_seq(svc_cancel_seq),
        .svc_result_valid(svc_result_valid),.svc_result_ready(svc_result_ready),
        .svc_result_session(svc_result_session),.svc_result_seq(svc_result_seq),
        .svc_result_count(svc_result_count),.svc_result_elapsed(svc_result_elapsed),
        .svc_result_code(svc_result_code),.svc_result_detail(svc_result_detail),
        .svc_tap_valid(svc_tap_valid),.svc_tap_sector_last(svc_tap_sector_last),
        .svc_tap_data(svc_tap_data),.svc_tap_index(svc_tap_index),
        .svc_tap_session(svc_tap_session),.svc_tap_seq(svc_tap_seq),
        .out_valid(out_valid),.out_ready(out_ready),.out_data(out_data),.out_last(out_last),
        .out_user(out_user),.out_session(out_session),.out_seq(out_seq),
        .dst_stop_req(dst_stop_req),.dst_stop_ack(dst_stop_ack),
        .spi_cs_n(spi_cs_n),.spi_sclk(spi_sclk),.spi_mosi(spi_mosi),.spi_miso(spi_miso)
    );

    wire [31:0] chain_calc_cur_cluster,chain_calc_fat_value;
    wire [5:0] chain_calc_sector_off;
    wire [38:0] chain_cluster_lba;
    wire [32:0] chain_fat_entry_lba;
    wire chain_cluster_lba_overflow,chain_fat_entry_lba_overflow;
    wire [27:0] chain_max_cluster;
    wire chain_entry_eoc,chain_entry_bad_reserved,chain_entry_in_range;
    // AUTO小量复用同一份总预算/游标/事务键；不额外套一层整文件状态副本。
    // 产品上下文模式已接入单客户端；记录有四槽，不代表已有四路格式解析器。
    fat32_chain_master #(.KEY_W(KEY_W),.TIME_W(TIME_W),.FILE_LEN_W(FILE_LEN_W),
        .CHUNK_ENABLE(CHUNK_ENABLE || AUTO_CHUNK || CONTEXT_ENABLE),.AUTO_CONTINUE(AUTO_CHUNK),
        .CONTEXT_ENABLE(CONTEXT_ENABLE),.EPOCH_W(EPOCH_W),.USE_EG_BRAM(USE_EG_BRAM)) u_chain(
        .clk(clk),.arst(arst),
        // 链控制模块使用IDLE本地命令槽提前接受，再等待 svc_req_ready 完成下游请求交接。
        // 初始化与链请求同拍到达时优先初始化；挂载或初始化期间不接受链请求，
        // 避免文件和目录事务使用正在更新的几何信息。
        .scan_geometry_valid(mounted && !mount_busy && !init_valid),
        .file_geometry_valid(mounted && !mount_busy && !init_valid),
        .fat_lba(fat_lba),.data_lba(data_lba),.root_cluster(root_cluster),
        .cluster_count(cluster_count),.cluster_shift(cluster_shift),
        .scan_valid(scan_valid),.scan_ready(scan_ready),
        .scan_session(scan_session),.scan_seq(scan_seq),
        .scan_budget_cycles(scan_budget_cycles),.scan_target_name(scan_target_name),
        .scan_start_cluster(scan_start_cluster),.scan_want_dir(scan_want_dir),
        .scan_sector_limit(scan_sector_limit),
        .scan_cancel_valid(scan_cancel_valid),.scan_cancel_ready(scan_cancel_ready),
        .scan_cancel_session(scan_cancel_session),.scan_cancel_seq(scan_cancel_seq),
        .scan_busy(scan_busy),.scan_result_valid(scan_result_valid),
        .scan_result_ready(scan_result_ready),.scan_result_session(scan_result_session),
        .scan_result_seq(scan_result_seq),.scan_result_elapsed(scan_result_elapsed),
        .scan_result_code(scan_result_code),.scan_result_detail(scan_result_detail),
        .hit_cluster(hit_cluster),.hit_size(hit_size),
        .file_valid(file_valid),.file_ready(file_ready),
        .file_session(file_session),.file_seq(file_seq),
        .file_budget_cycles(file_budget_cycles),.file_start_cluster(file_start_cluster),
        .file_size(file_size),
        .file_start_extent_seq(AUTO_CHUNK?file_seq:file_start_extent_seq),.file_checkpoint_extent_seq(file_checkpoint_extent_seq),
        .file_chunked(AUTO_CHUNK?1'b1:file_chunked),.file_start_sector(AUTO_CHUNK?6'd0:file_start_sector),
        // 分块长度接口为 13 位，AUTO_CHUNK_BYTES 显式选择低 13 位后连接。
        .file_chunk_limit(AUTO_CHUNK?AUTO_CHUNK_BYTES[12:0]:file_chunk_limit),.file_yield_valid(),.file_yield_ready(1'b1),
        .file_result_more(file_result_more),.file_checkpoint_cluster(file_checkpoint_cluster),
        .file_checkpoint_sector(file_checkpoint_sector),.file_checkpoint_remaining(file_checkpoint_remaining),
        .file_cancel_valid(file_cancel_valid),.file_cancel_ready(file_cancel_ready),
        .file_cancel_session(file_cancel_session),.file_cancel_seq(file_cancel_seq),
        .sink_abort_req(file_sink_abort_req),.sink_abort_ack(file_sink_abort_ack),
        .sink_abort_session(file_sink_abort_session),.sink_abort_seq(file_sink_abort_seq),
        .sink_ack_session(file_sink_ack_session),.sink_ack_seq(file_sink_ack_seq),
        .file_busy(file_busy),.file_result_valid(file_result_valid),
        .file_result_ready(file_result_ready),.file_result_session(file_result_session),
        .file_result_seq(file_result_seq),.file_result_elapsed(file_result_elapsed),
        .file_result_count(file_result_count),
        .file_result_code(file_result_code),.file_result_detail(file_result_detail),
        .req_valid(chain_req_valid),.req_ready(svc_req_ready),
        .req_session(chain_req_session),.req_seq(chain_req_seq),.req_lba(chain_req_lba),
        .req_budget_cycles(chain_req_budget),.req_sectors(chain_req_sectors),
        .req_byte_count(chain_req_bytes),.req_user(chain_req_user),
        .ext_cancel_valid(chain_ext_cancel_valid),.ext_cancel_ready(svc_cancel_ready),
        .ext_cancel_session(chain_cancel_session),.ext_cancel_seq(chain_cancel_seq),
        .ext_result_valid(svc_result_valid),.ext_result_ready(chain_ext_result_ready),
        .ext_result_session(svc_result_session),.ext_result_seq(svc_result_seq),
        .ext_result_count(svc_result_count),.ext_result_elapsed(svc_result_elapsed),
        .ext_result_code(svc_result_code),.ext_result_detail(svc_result_detail),
        .tap_valid(svc_tap_valid),.tap_data(svc_tap_data),.tap_index(svc_tap_index),
        .tap_sector_last(svc_tap_sector_last),.tap_session(svc_tap_session),.tap_seq(svc_tap_seq),
        .calc_cur_cluster(chain_calc_cur_cluster),.calc_fat_value(chain_calc_fat_value),
        .calc_cluster_sector_off(chain_calc_sector_off),
        .calc_cluster_lba(chain_cluster_lba),
        .calc_fat_entry_lba(chain_fat_entry_lba),
        .calc_cluster_lba_overflow(chain_cluster_lba_overflow),
        .calc_fat_entry_lba_overflow(chain_fat_entry_lba_overflow),
        .calc_max_cluster(chain_max_cluster),
        .calc_entry_eoc(chain_entry_eoc),
        .calc_entry_bad_reserved(chain_entry_bad_reserved),
        .calc_entry_in_range(chain_entry_in_range),
        .ctx_service_enabled(ctx_service_enabled),.quiesce(quiesce),.media_epoch(media_epoch),
        .ctx_valid(ctx_valid),.demand(demand),.credit(credit),.critical_mask(critical_mask),
        .low_mask(low_mask),.necessary_mask(necessary_mask),.background_mask(background_mask),
        .cancel_mask(cancel_mask),.deadline_mask(deadline_mask),
        .ctx_session(ctx_session),.ctx_seq(ctx_seq),.ctx_epoch(ctx_epoch),
        .ctx_cluster(ctx_cluster),.ctx_size(ctx_size),.ctx_budget(ctx_budget),.ctx_limit(ctx_limit),
        .ctx_busy(ctx_busy),.quiescent(quiescent),.record_valid(record_valid),.ctx_accepted(ctx_accepted),
        .owner_valid(owner_valid),.owner_ready(owner_ready),.owner_ctx(owner_ctx),
        .owner_session(owner_session),.owner_seq(owner_seq),.owner_ticket(owner_ticket),.owner_epoch(owner_epoch),
        .installed_valid(installed_valid),.installed_ctx(installed_ctx),
        .installed_session(installed_session),.installed_seq(installed_seq),
        .installed_ticket(installed_ticket),.installed_epoch(installed_epoch),
        .receipt_valid(receipt_valid),.receipt_ready(receipt_ready),.receipt_ctx(receipt_ctx),
        .receipt_session(receipt_session),.receipt_seq(receipt_seq),
        .receipt_ticket(receipt_ticket),.receipt_epoch(receipt_epoch),
        .sink_ack_ctx(sink_ack_ctx),.sink_ack_ticket(sink_ack_ticket),.sink_ack_epoch(sink_ack_epoch),
        .sink_abort_ctx(sink_abort_ctx),.sink_abort_ticket(sink_abort_ticket),.sink_abort_epoch(sink_abort_epoch),
        .event_valid(event_valid),.event_ready(event_ready),.event_ctx(event_ctx),
        .event_session(event_session),.event_seq(event_seq),.event_ticket(event_ticket),
        .event_epoch(event_epoch),.event_code(event_code),.event_detail(event_detail),
        .event_more(event_more),.event_count(event_count),.event_quantum_count(event_quantum_count)
    );
    fat32_chain_calc u_chain_calc(
        .cur_cluster(chain_calc_cur_cluster),.cluster_count(cluster_count),
        .base_fat_lba(fat_lba),.base_data_lba(data_lba),.fat_value(chain_calc_fat_value),
        .cluster_shift(cluster_shift),.cluster_sector_off(chain_calc_sector_off),
        .cluster_lba(chain_cluster_lba),.fat_entry_lba(chain_fat_entry_lba),
        .cluster_lba_overflow(chain_cluster_lba_overflow),
        .fat_entry_lba_overflow(chain_fat_entry_lba_overflow),
        .max_cluster(chain_max_cluster),
        .entry_eoc(chain_entry_eoc),.entry_bad_reserved(chain_entry_bad_reserved),
        .entry_in_range(chain_entry_in_range)
    );
    // 结果字段由共享extent广播，valid/tap只发给锁定的服务所有者。
    assign ext_result_session=svc_result_session;assign ext_result_seq=svc_result_seq;
    assign ext_result_count=svc_result_count;assign ext_result_elapsed=svc_result_elapsed;
    assign ext_result_code=svc_result_code;assign ext_result_detail=svc_result_detail;
endmodule
`default_nettype wire
