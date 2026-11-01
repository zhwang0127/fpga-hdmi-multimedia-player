`timescale 1ns/1ps
`default_nettype none

// 真实SD挂载闭环：CMD9条件后的sector 0 -> 可选分区引导扇区 -> FAT32数值几何。
// 引导扇区仍走extent的CRC、重试、CDC和真实目标接收器；本模块不实现目录或文件读取。
// dst域字节消费与停止确认作为端口交给上层真实客户端驱动；内部不内置假接收器，
// 否则综合器会把跨域FIFO折叠成组合直通，破坏CDC结构。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module fat32_mount_storage #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    // TIME_W 为内部预算、已用和剩余周期计数的位宽，默认 32 位；
    // 产品由 tf_bmp_boot 传入 counter_width(SYS_BUDGET+1)=26，该宽度按构造恰好容纳预算值本身。
    // 所有比较/减法都在 TIME_W 位内完成；截止判定仍是逐周期精确的，预算值不变。
    parameter integer KEY_W=32,
    parameter integer TIME_W=32,
    // CSD容量上限位数，透传到 sd_csd_reader（默认 33 位）。
    parameter integer CARD_SECTOR_W=33,
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
    input wire cancel_valid,output wire cancel_ready,
    input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire mount_busy,output wire mount_result_valid,input wire mount_result_ready,
    output wire [KEY_W-1:0] mount_result_session,mount_result_seq, output wire [31:0] mount_result_elapsed,
    output wire [3:0] mount_result_code,output wire [7:0] mount_result_detail,
    output reg mounted,
    output wire [32:0] partition_end,volume_end,
    output wire [31:0] fat_lba,data_lba,root_lba,fat_sectors,root_cluster,
    output wire [27:0] cluster_count,
    output wire [2:0] cluster_shift,output wire [3:0] active_fat,
    output wire out_valid,input wire out_ready,output wire [7:0] out_data,output wire out_last,
    output wire [1:0] out_user,output wire [KEY_W-1:0] out_session,out_seq,
    output wire dst_stop_req,input wire dst_stop_ack,
    // 扫描服务转发：挂载完全空闲时把extent通路借给目录扫描控制模块。
    // 挂载/初始化事务期间svc_req_ready恒0，挂载行为不变；服务结果原样转交上层。
    input wire svc_req_valid,output wire svc_req_ready,
    input wire [KEY_W-1:0] svc_req_session,svc_req_seq, input wire [31:0] svc_req_lba,svc_req_budget_cycles,
    input wire [3:0] svc_req_sectors,input wire [12:0] svc_req_byte_count,
    input wire [1:0] svc_req_user,
    input wire svc_cancel_valid,output wire svc_cancel_ready,
    input wire [KEY_W-1:0] svc_cancel_session,svc_cancel_seq,
    output wire svc_result_valid,input wire svc_result_ready,
    output wire [KEY_W-1:0] svc_result_session,svc_result_seq, output wire [31:0] svc_result_count,svc_result_elapsed,
    output wire [3:0] svc_result_code,output wire [7:0] svc_result_detail,
    output wire svc_tap_valid,svc_tap_sector_last,
    output wire [7:0] svc_tap_data,output wire [8:0] svc_tap_index,
    output wire [KEY_W-1:0] svc_tap_session,svc_tap_seq,
    output wire spi_cs_n,spi_sclk,spi_mosi,input wire spi_miso
);
    wire rst_logic;
    localparam IDLE=0,READ0_START=1,READ0_WAIT=2,READ1_START=3,READ1_WAIT=4,
               GEO_START=5,GEO_WAIT=6,RESULT=7;
    localparam OK=0,EXTENT=1,BOOT=2,GEOMETRY=3,CANCELED=8,DEADLINE=10,PROTOCOL=11;
    reg [2:0] state;
    reg [3:0] status;
    reg [7:0] detail;
    reg [KEY_W-1:0] session,seq,internal_seq; reg [TIME_W-1:0] budget,elapsed; reg [31:0] partition_lba_reg;
    // 缓存当前elapsed对应的到期/剩余量，避免宽减法/比较串进extent与parser CE。
    // 原截止是budget==0或elapsed==budget-1（不是>=），必须保留完整32位回绕语义。
    reg deadline_q;reg [TIME_W-1:0] budget_penultimate,remaining_q;
    reg [32:0] partition_sectors_reg;
    reg extent_done;
    reg [3:0] extent_code_reg;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    wire active=state!=IDLE && state!=RESULT;
    wire cancel_match=active && cancel_valid && cancel_session==session && cancel_seq==seq;
    wire cancel_now=enabled && cancel_match && status==OK;
    wire deadline_now=enabled && active && status==OK && deadline_q;
    wire [TIME_W-1:0] remaining=remaining_q;


    wire extent_init_ready,extent_req_ready,extent_busy,extent_result_valid;
    wire [KEY_W-1:0] extent_result_session,extent_result_seq; wire [31:0] extent_result_count,extent_result_elapsed;
    wire [3:0] extent_result_code;
    wire [7:0] extent_result_detail;
    wire [2:0] extent_bridge_reason;
    wire [5:0] extent_attempts;wire [4:0] extent_retries;
    wire extent_card_fault;
    wire [KEY_W-1:0] out_session_int,out_seq_int;wire [1:0] out_user_int;
    wire [10:0] src_level,dst_level;
    wire tap_valid,tap_sector_last,tap_last;
    wire [7:0] tap_data;wire [8:0] tap_index;
    wire [KEY_W-1:0] tap_session,tap_seq;
    wire context_available,context_wr_enable,context_rd_enable;
    wire [7:0] context_wr_addr,context_rd_addr,context_wr_data,context_rd_data;

    wire read_start=state==READ0_START || state==READ1_START;
    wire read_wait=state==READ0_WAIT || state==READ1_WAIT;
    wire parser_start_ready,parser_byte_ready,parser_result_valid;
    wire [2:0] parser_result_code;wire parser_is_mbr;
    wire [31:0] parser_partition_lba;wire [32:0] parser_partition_sectors;
    wire [15:0] parser_signature,parser_bps,parser_reserved;
    wire [7:0] parser_spc,parser_fats;
    wire [15:0] parser_root_entries,parser_total16,parser_fat16,parser_version,parser_flags;
    wire [31:0] parser_total,parser_fat_size,parser_root;
    wire extent_req_valid_mnt=enabled && read_start && parser_start_ready &&
                              !cancel_now && !deadline_now && remaining!=0;
    // 服务借用只在挂载IDLE、无初始化且extent可接受时授予；svc_inflight覆盖整个服务事务。
    reg svc_inflight;
    wire svc_grant=enabled && state==IDLE && !init_valid && extent_req_ready && !svc_inflight;
    assign svc_req_ready=svc_grant;
    wire svc_fire=svc_req_valid && svc_req_ready;
    wire svc_cancel_go=svc_inflight && svc_cancel_valid;
    assign svc_cancel_ready=svc_inflight;
    wire extent_req_valid=extent_req_valid_mnt || svc_fire;
    wire extent_req_fire=extent_req_valid && extent_req_ready;
    // extent也借给扫描/文件服务，不能把BPB解析器错开成COLLECT。
    // read_start限定真实挂载两种读状态，仍与extent接受同一sys正沿
    // 启动；不把ready常量化，也不改变服务请求与响应握手。
    wire parser_start_valid=extent_req_fire && read_start;
    wire tap_match=tap_session==session && tap_seq==internal_seq;
    wire tap_protocol=read_wait && tap_valid && (!tap_match || !parser_byte_ready);
    // tap_protocol只在本拍锁存status，禁止组合进入abort/取消：经parser enabled→byte_ready
    // 或经extent取消→p_ready→tap_valid都会回到tap_protocol自身形成环；下游停止推迟一拍。
    wire parser_abort=read_wait && (status!=OK || cancel_now || deadline_now ||
                      (extent_result_valid && extent_result_code!=OK));
    wire read_complete=(extent_done || extent_result_valid) && parser_result_valid;
    wire parser_result_ready=read_wait && status==OK && !cancel_now && !deadline_now &&
                             read_complete;
    // 服务事务结果直通上层；挂载读结果仍由本地等待状态消费。
    wire extent_result_ready=svc_inflight ? svc_result_ready : read_wait;
    assign svc_result_valid=svc_inflight && extent_result_valid;
    assign svc_result_session=extent_result_session;assign svc_result_seq=extent_result_seq;
    assign svc_result_count=extent_result_count;assign svc_result_elapsed=extent_result_elapsed;
    assign svc_result_code=extent_result_code;assign svc_result_detail=extent_result_detail;
    // tap观察口直通转发：扫描控制模块自行按键和状态门控字节。
    assign svc_tap_valid=tap_valid;assign svc_tap_data=tap_data;assign svc_tap_index=tap_index;
    assign svc_tap_sector_last=tap_sector_last;assign svc_tap_session=tap_session;
    assign svc_tap_seq=tap_seq;
    // 取消只看已锁存status：tap_protocol当拍进取消会经extent p_ready→tap_valid闭环。
    wire extent_cancel=read_wait && (status!=OK || cancel_now || deadline_now);

    // dst域消费与停止确认走模块端口：由上层真实客户端驱动。
    // 内部不产生假ready/假ack，跨域FIFO结构对综合器保持不可折叠。

    wire geo_ready,geo_busy,geo_result_valid;
    wire [KEY_W-1:0] geo_result_session,geo_result_seq; wire [31:0] geo_result_elapsed;
    wire [3:0] geo_result_code;
    wire geo_start=state==GEO_START && status==OK && !cancel_now && !deadline_now &&
                   remaining!=0 && context_available && geo_ready;
    wire context_claim=geo_busy || geo_start;
    wire geo_cancel_valid=state==GEO_WAIT && (status!=OK || cancel_now || deadline_now);
    wire geo_result_ready=state==GEO_WAIT && !cancel_now && !deadline_now;

    assign init_ready=extent_init_ready && state==IDLE;
    assign mount_ready=enabled && state==IDLE && !init_valid && card_qualified &&
                       extent_req_ready && context_available && geo_ready;
    assign cancel_ready=enabled && active && !cancel_match ? 1'b0 :
                        enabled && active && status==OK;
    assign mount_busy=enabled && state!=IDLE;
    assign mount_result_valid=enabled && state==RESULT;
    assign mount_result_session=session;assign mount_result_seq=seq;
    assign mount_result_elapsed=elapsed;assign mount_result_code=status;
    assign mount_result_detail=detail;
    assign out_session=out_session_int;assign out_seq=out_seq_int;assign out_user=out_user_int;

    sd_extent_stream #(.KEY_W(KEY_W),.TIME_W(TIME_W),.CARD_SECTOR_W(CARD_SECTOR_W),.USE_EG_BRAM(USE_EG_BRAM),.SHARE_CONTEXT(1),
        .POWER_CYCLES(POWER_CYCLES),.POLL_CYCLES(POLL_CYCLES),
        .INIT_TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES),.READ_WAIT_CYCLES(READ_WAIT_CYCLES),
        .EXACT_LENGTH(1)) u_extent(
        .clk(clk),.dst_clk(dst_clk),.arst(arst),
        .init_valid(init_valid && state==IDLE),.init_ready(extent_init_ready),.init_epoch(init_epoch),
        .init_abort_valid(init_abort_valid),.init_abort_ready(init_abort_ready),.init_abort_epoch(init_abort_epoch),
        .init_done_valid(init_done_valid),.init_done_ready(init_done_ready),
        .init_done_epoch(init_done_epoch),.init_done_register(init_done_register),
        .init_done_code(init_done_code),.init_done_command(init_done_command),.init_done_r1(init_done_r1),
        .card_qualified(card_qualified),.card_block_addressed(card_block_addressed),
        .media_fault(media_fault),.card_initialized(card_initialized),
        .card_last_lba(card_last_lba),.card_sector_count(card_sector_count),
        .req_valid(extent_req_valid),.req_ready(extent_req_ready),
        .req_session(svc_fire ? svc_req_session : session),
        .req_seq(svc_fire ? svc_req_seq : internal_seq),
        .req_lba(svc_fire ? svc_req_lba : (state==READ0_START ? 32'd0 : partition_lba_reg)),
        .req_budget_cycles(svc_fire ? svc_req_budget_cycles : remaining),
        .req_sectors(svc_fire ? svc_req_sectors : 4'd1),
        .req_byte_count(svc_fire ? svc_req_byte_count : 13'd512),
        .req_user(svc_fire ? svc_req_user : 2'b00),
        .cancel_valid(extent_cancel || svc_cancel_go),
        .cancel_session(svc_cancel_go ? svc_cancel_session : session),
        .cancel_seq(svc_cancel_go ? svc_cancel_seq : internal_seq),
        .busy(extent_busy),.result_valid(extent_result_valid),.result_ready(extent_result_ready),
        .result_session(extent_result_session),.result_seq(extent_result_seq),
        .result_count(extent_result_count),.result_elapsed(extent_result_elapsed),
        .result_code(extent_result_code),.result_detail(extent_result_detail),
        .result_bridge_reason(extent_bridge_reason),.result_attempts(extent_attempts),
        .result_retries(extent_retries),.result_card_fault(extent_card_fault),
        .out_valid(out_valid),.out_ready(out_ready),.out_data(out_data),.out_last(out_last),
        .out_session(out_session_int),.out_seq(out_seq_int),.out_user(out_user_int),
        .dst_stop_req(dst_stop_req),.dst_stop_ack(dst_stop_ack),
        .dst_ack_session(out_session_int),.dst_ack_seq(out_seq_int),
        .src_level(src_level),.dst_level(dst_level),
        .tap_valid(tap_valid),.tap_data(tap_data),.tap_index(tap_index),
        .tap_sector_last(tap_sector_last),.tap_last(tap_last),
        .tap_session(tap_session),.tap_seq(tap_seq),
        .spi_cs_n(spi_cs_n),.spi_sclk(spi_sclk),.spi_mosi(spi_mosi),.spi_miso(spi_miso),
        .context_claim(context_claim),.context_available(context_available),
        .context_wr_enable(context_wr_enable),.context_rd_enable(context_rd_enable),
        .context_wr_addr(context_wr_addr),.context_rd_addr(context_rd_addr),
        .context_wr_data(context_wr_data),.context_rd_data(context_rd_data)
    );

    // 引导扇区就地比对（IN_PLACE=1），只存后级要用的字段，供几何校验和挂载结果生成使用。
    fat32_boot_parser #(.IN_PLACE(1)) u_parser(
        .clk(clk),.arst(arst),.abort(parser_abort),
        .start_valid(parser_start_valid),.start_ready(parser_start_ready),
        .byte_valid(tap_valid && read_wait && tap_match),.byte_ready(parser_byte_ready),
        .byte_data(tap_data),.byte_index(tap_index),.byte_last(tap_sector_last),
        .result_valid(parser_result_valid),.result_ready(parser_result_ready),
        .result_code(parser_result_code),.result_is_mbr(parser_is_mbr),
        .partition_lba(parser_partition_lba),.partition_sectors(parser_partition_sectors),
        .signature(parser_signature),.bytes_per_sector(parser_bps),
        .reserved_sectors(parser_reserved),.sectors_per_cluster(parser_spc),.num_fats(parser_fats),
        .root_entry_count(parser_root_entries),.total_sectors16(parser_total16),
        .fat_sectors16(parser_fat16),.fs_version(parser_version),.ext_flags(parser_flags),
        .total_sectors(parser_total),.fat_sectors(parser_fat_size),.root_cluster(parser_root)
    );

    fat32_geometry #(.KEY_W(KEY_W),.TIME_W(TIME_W),.USE_EG_BRAM(USE_EG_BRAM),.EXTERNAL_CONTEXT_RAM(1)) u_geometry(
        .clk(clk),.arst(arst),.req_valid(state==GEO_START && status==OK &&
             !cancel_now && !deadline_now && remaining!=0 && context_available),
        .req_ready(geo_ready),.req_session(session),.req_seq(seq),.req_budget_cycles(remaining),
        .req_partition_lba(partition_lba_reg),.req_partition_sectors(partition_sectors_reg),
        .req_card_sectors(card_sector_count),.req_signature(parser_signature),
        .req_bytes_per_sector(parser_bps),.req_reserved_sectors(parser_reserved),
        .req_sectors_per_cluster(parser_spc),.req_num_fats(parser_fats),
        .req_root_entry_count(parser_root_entries),.req_total_sectors16(parser_total16),
        .req_fat_sectors16(parser_fat16),.req_fs_version(parser_version),
        .req_total_sectors(parser_total),.req_fat_sectors(parser_fat_size),
        .req_root_cluster(parser_root),.req_ext_flags(parser_flags),
        .cancel_valid(geo_cancel_valid),.cancel_ready(),
        .cancel_session(session),.cancel_seq(seq),.busy(geo_busy),
        .result_valid(geo_result_valid),.result_ready(geo_result_ready),
        .result_session(geo_result_session),.result_seq(geo_result_seq),
        .result_code(geo_result_code),.result_elapsed(geo_result_elapsed),
        .partition_end(partition_end),.volume_end(volume_end),
        .fat_lba(fat_lba),.data_lba(data_lba),.root_lba(root_lba),
        .fat_sectors(fat_sectors),.root_cluster(root_cluster),
        .cluster_count(cluster_count),.cluster_shift(cluster_shift),.active_fat(active_fat),
        .context_ram_wr_enable(context_wr_enable),.context_ram_rd_enable(context_rd_enable),
        .context_ram_wr_addr(context_wr_addr),.context_ram_rd_addr(context_rd_addr),
        .context_ram_wr_data(context_wr_data),.context_ram_rd_data(context_rd_data)
    );

    // IDLE 每拍准备当前描述符及预算，接受沿锁存同一份输入；离开 IDLE 后随事务计时更新。
    // RESULT 保持最后结果，下次请求前重新准备；elapsed 回绕时按 budget-elapsed 的模值重建剩余量。
    always @(posedge clk or posedge rst) begin
        if(rst) begin deadline_q<=1;budget_penultimate<={{(TIME_W-1){1'b1}},1'b0};remaining_q<=0;end
        else if(state==IDLE) begin
            deadline_q<=mount_budget_cycles<=32'd1;
            budget_penultimate<=mount_budget_cycles[TIME_W-1:0]-2'd2;remaining_q<=mount_budget_cycles[TIME_W-1:0];
        end else if(active) begin
            deadline_q<=budget==0 || elapsed==budget_penultimate;
            if(elapsed=={TIME_W{1'b1}}) remaining_q<=budget; // TIME_W位回绕，与TIME_W位 max(budget-elapsed,0) 一致
            else if(remaining_q!=0) remaining_q<=remaining_q-1'b1;
        end
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;status<=OK;detail<=0;session<=0;seq<=0;budget<=0;elapsed<=0;
            internal_seq<=0;partition_lba_reg<=0;partition_sectors_reg<=0;
            extent_done<=0;extent_code_reg<=0;mounted<=0;svc_inflight<=0;
        end else begin
            // 服务借用从授予沿保持到该服务extent结果被上层消费。
            if(svc_fire) svc_inflight<=1;
            else if(svc_inflight && extent_result_valid && extent_result_ready) svc_inflight<=0;
            if(!card_qualified || media_fault) mounted<=0;
            if(init_valid && init_ready) mounted<=0;
            if(active) elapsed<=elapsed+1'b1;
            case(state)
                IDLE:if(mount_valid && mount_ready) begin
                    session<=mount_session;seq<=mount_seq;internal_seq<=mount_seq;
                    budget<=mount_budget_cycles[TIME_W-1:0];elapsed<=0;status<=OK;detail<=0;
                    partition_lba_reg<=0;partition_sectors_reg<=0;extent_done<=0;
                    mounted<=0;
                    if(mount_budget_cycles==0) begin status<=DEADLINE;state<=RESULT;end
                    else state<=READ0_START;
                end
                READ0_START,READ1_START:begin
                    if(cancel_now) begin status<=CANCELED;detail<=state;state<=RESULT;end
                    else if(deadline_now) begin status<=DEADLINE;detail<=state;state<=RESULT;end
                    else if(extent_req_fire) begin
                        extent_done<=0;extent_code_reg<=0;
                        state<=state==READ0_START ? READ0_WAIT : READ1_WAIT;
                    end
                end
                READ0_WAIT,READ1_WAIT:begin
                    if(cancel_now) begin status<=CANCELED;detail<=state;end
                    else if(deadline_now) begin status<=DEADLINE;detail<=state;end
                    else if(tap_protocol) begin status<=PROTOCOL;detail<=8'h01;end
                    if(extent_result_valid && extent_result_ready) begin
                        extent_done<=1;extent_code_reg<=extent_result_code;
                        if(extent_result_session!=session || extent_result_seq!=internal_seq) begin
                            status<=PROTOCOL;detail<=8'h02;
                        end else if(extent_result_code!=OK && status==OK && !cancel_now && !deadline_now) begin
                            status<=EXTENT;detail<={4'd0,extent_result_code};
                        end
                    end
                    if(status!=OK && extent_done) state<=RESULT;
                    else if(status==OK && extent_result_valid && extent_result_code!=OK &&
                            !cancel_now && !deadline_now) begin
                        status<=EXTENT;detail<={4'd0,extent_result_code};state<=RESULT;
                    end else if(read_complete && status==OK && !cancel_now && !deadline_now) begin
                        if(parser_result_code!=0) begin
                            status<=BOOT;detail<={5'd0,parser_result_code};state<=RESULT;
                        end else if(state==READ0_WAIT && parser_is_mbr) begin
                            partition_lba_reg<=parser_partition_lba;
                            partition_sectors_reg<=parser_partition_sectors;
                            internal_seq<=seq+1'b1;extent_done<=0;state<=READ1_START;
                        end else if(state==READ1_WAIT && parser_is_mbr) begin
                            status<=BOOT;detail<=8'h08;state<=RESULT;
                        end else begin
                            partition_lba_reg<=state==READ0_WAIT ? 32'd0 : partition_lba_reg;
                            partition_sectors_reg<=state==READ0_WAIT ? card_sector_count : partition_sectors_reg;
                            extent_done<=0;state<=GEO_START;
                        end
                    end
                end
                GEO_START:begin
                    if(cancel_now) begin status<=CANCELED;detail<=GEO_START;state<=RESULT;end
                    else if(deadline_now) begin status<=DEADLINE;detail<=GEO_START;state<=RESULT;end
                    else if(geo_start) state<=GEO_WAIT;
                end
                GEO_WAIT:begin
                    if(cancel_now) begin status<=CANCELED;detail<=GEO_WAIT;end
                    else if(deadline_now) begin status<=DEADLINE;detail<=GEO_WAIT;end
                    if(geo_result_valid && geo_result_ready) begin
                        if(geo_result_session!=session || geo_result_seq!=seq) begin
                            status<=PROTOCOL;detail<=8'h03;mounted<=0;
                        end else if(status==OK && geo_result_code!=0) begin
                            status<=GEOMETRY;detail<={4'd0,geo_result_code};mounted<=0;
                        end else if(status==OK) mounted<=1;
                        state<=RESULT;
                    end
                end
                RESULT:if(mount_result_valid && mount_result_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
