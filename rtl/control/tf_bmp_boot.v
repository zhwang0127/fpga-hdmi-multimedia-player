`timescale 1ns/1ps
`default_nettype none
// 冷启动控制模块：初始化TF卡、挂载FAT32、找MEDIA，再依次装载四个图片ID。
// clk域管流程，mem_clk域实际写SDRAM，两域通过mailbox和字节桥交接。
// 首图真正提交后才发显示描述符；失败图片按失败类型重试或跳过。
// 卡失格时先恢复初始化/挂载，不把临时I/O故障直接当成永久坏文件。
// 重试次数用完就停止；HDMI时基本身不跟着读卡流程停下来。
module tf_bmp_boot #(
    parameter EXT_BMP_FORMATS=0,
    parameter BMP32_EN=1,
    parameter integer EPOCH_W=32,
    // SD/FAT32 整条装载链的 session/seq/init epoch 键宽，整机为 8 位。
    // KEY_W 逐层传到 fat32_bmp_loader 以下实例，所有事务接口必须同宽。
    // 每条通道仅一个在途事务，取消后等实际排空才复用键，防止回绕后旧消息命中新作业。
    parameter integer KEY_W=8,
    // 默认允许缩略图；整机 WAV 配置关闭未用于显示的缩略图生成。
    parameter ENABLE_THUMBNAILS=1,ENABLE_WAV=0,ENABLE_AUDIO_CONTROL=0,
    parameter USE_EG_BRAM=1,POWER_CYCLES=50000,POLL_CYCLES=50000,
    parameter INIT_TIMEOUT_CYCLES=50000000,READ_WAIT_CYCLES=5000000,
    // SYS_BUDGET 按 sys=50MHz 计数，MEM_BUDGET 按 mem=100MHz 计数，默认均为 1 秒。
    parameter SYS_BUDGET=50000000,MEM_BUDGET=100000000,
    // 扇区读取消后须等待令牌并排空整块；sys 域 150ms 排空期限覆盖 100ms 令牌等待和数据尾部。
    // 到期仍未确认停止时隔离该事务，避免提前复用仍在途的资源。
    parameter DRAIN_CYCLES=7500000,
    // 卡初始化/挂载/目录失败的重试次数与间隔（50MHz 下 5000000 拍=100ms）。
    parameter RETRY_LIMIT=3,RETRY_WAIT_CYCLES=5000000,
    // TF卡容量上限 = 2^CARD_SECTOR_W 个512B扇区。27 -> 64GiB（2^27扇区）。
    // 更大的SDHC/SDXC卡在CMD9/CSD阶段判 CAPACITY，按“初始化失败→100ms 间隔重试 3 次→LED 闪码”处理，不截断容量。
    parameter integer CARD_SECTOR_W=27,
    parameter integer CONTEXT_ENABLE=0
)(
    input wire audio_req_valid,output wire audio_req_ready,input wire audio_req_cancel,audio_req_bank,
    input wire [17:0] audio_req_words,input wire [15:0] audio_req_token,
    output wire audio_rsp_valid,output wire [1:0] audio_rsp_status,output wire audio_rsp_bank,output wire [15:0] audio_rsp_token,
    output wire audio_mem_valid,input wire audio_mem_ready,output wire audio_mem_cancel,audio_mem_bank,
    output wire [17:0] audio_mem_words,output wire [15:0] audio_mem_token,
    input wire audio_mem_rsp_valid,output wire audio_mem_rsp_ready,input wire [1:0] audio_mem_rsp_status,
    input wire audio_mem_rsp_bank,input wire [15:0] audio_mem_rsp_token,
    input wire clk,mem_clk,arst,cancel,
    output wire spi_cs_n,spi_sclk,spi_mosi,input wire spi_miso,
    output wire load_valid,input wire load_ready,output wire [20:0] load_addr,
    output wire [3:0] load_count,output wire [31:0] load_budget,
    output wire load_data_valid,output wire [31:0] load_data,
    input wire load_beat,load_done,load_done_was_write,
    output wire [3:0] slot_reserved,input wire [3:0] slot_writable,
    output wire commit_valid,input wire commit_ready,
    output wire [1:0] commit_slot,output wire [31:0] commit_gen,
    output wire desc_valid,input wire desc_ready,output wire [EPOCH_W+223:0] desc_bus,
    input wire ack_valid,output wire ack_ready,input wire [EPOCH_W+20:0] ack_bus,
    // sys 域音频控制模块接口：当前图只用于预取规划；pin/retire/release才改变读引用。
    input wire wav_current_valid,input wire [1:0] wav_current_slot,
    input wire wav_want_valid,input wire [1:0] wav_want_slot,
    input wire [1:0] wav_ignore_present,
    input wire wav_pin_valid,wav_pin_bank,wav_retire_valid,wav_retire_bank,
    input wire [1:0] wav_release_bank,
    output wire [5:0] wav_bank_states,output wire [3:0] wav_bank_slots,
    output wire [35:0] wav_bank_words,output wire [1:0] wav_bank_ready,
    output wire [3:0] wav_unavailable,output wire wav_fault,
    output reg [3:0] loaded,failed,output reg done,output reg [7:0] error_code
);
    // 恢复流程全部在clk域；下面这些状态不跨时钟域直接使用。
    // init_epoch_q给每轮初始化换一个标签，回绕时跳过0和首次启动的1。
    // retry_left记录还可重试几次，age在重试间隔和等待响应时复用。
    // recovering区分首次开机和运行中恢复，resume_state记住恢复后回哪里。
    // io_streak数连续I/O失败；任意文件成功都会把它清零。
    // bmp_io_retried限制同一图片的额外重试，防止坏卡一直占住流程。
    reg [7:0] init_epoch_q;
    wire [7:0] init_epoch_next=(init_epoch_q==8'hff) ? 8'd2 : init_epoch_q+8'd1;
    reg [1:0] retry_left,io_streak;
    reg recovering,card_dead,bmp_io_retried;
    reg [4:0] resume_state;
    wire media_fault_w;
    wire rst_logic;
    localparam INIT=0,INIT_WAIT=1,MOUNT=2,MOUNT_WAIT=3,DIR=4,DIR_WAIT=5,
        SCAN=6,SCAN_WAIT=7,LOAD=8,LOAD_WAIT=9,TABLE=10,DESC=11,ACK=12,NEXT=13,STOP=14,WSCAN=15,WSCAN_WAIT=16,WLOAD=17,WLOAD_WAIT=18,
        RETRY_WAIT=19;
    // 状态使用 5 位编码，包含重试等待状态；非法状态进入 STOP。
    reg [4:0] state;
    reg [1:0] index,first_slot;
    reg [31:0] media_cluster,file_cluster;
    // 目录项长度保存低 24 位及超限标志，达到 16MiB 时向提交层发送 32'hffffffff。
    // 提交层按完整长度拒绝超限描述符，避免截断后把不合法文件当成较短的合法文件。
    reg [23:0] file_size24; reg file_too_long;
    wire [31:0] file_size=file_too_long ? 32'hffffffff : {8'd0,file_size24};
    // 计数位宽须能表示阈值本身，防止达到截止值前回绕。
    function integer counter_width;
        input integer value;
        integer v;
        begin v=value-1;counter_width=1;while(v>1) begin v=v>>1;counter_width=counter_width+1;end end
    endfunction
    // age 只在 TABLE/DESC/ACK/RETRY_WAIT 计数，到 SYS_BUDGET-1（看门狗）或 RETRY_WAIT_CYCLES-1 即离开这些状态并清零；
    // 多留1位余量（+2），保证 >= 比较在任何状态顺序下都不会因回绕失效。产品 50,000,000 -> 26位。
    localparam integer AGE_W=counter_width((SYS_BUDGET>RETRY_WAIT_CYCLES ? SYS_BUDGET : RETRY_WAIT_CYCLES)+2);
    // SD/FAT32 服务内部预算计数位宽：容纳 SYS_BUDGET 本身（挂载/目录/文件读取预算全部 = SYS_BUDGET）。产品26位。
    localparam integer TIME_W=counter_width(SYS_BUDGET+1);
    // 链主控文件字节计数24位（小于 16MiB）；超长在 tf_bmp_boot/提交层/链主控三处都显式拒绝。
    localparam integer FILE_LEN_W=24;
    reg [AGE_W-1:0] age;
    reg shown,committed;
    wire rst;reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    wire init_ready,init_done_valid,mount_ready,mount_result_valid,scan_ready,scan_result_valid;
    wire [3:0] init_code,mount_code,scan_code;
    wire [31:0] root_cluster,hit_cluster,hit_size;
    wire req_ready,result_valid,quarantined;
    wire [7:0] result_code;
    wire [31:0] active_gen;
    wire [1:0] active_slot;
    wire raw_commit;
    // 四图启动流程结束后，WAV 作业复用同一 FAT 服务，文件请求与图片装载串行执行。
    reg wav_processing_q;
    wire wav_processing=ENABLE_WAV && wav_processing_q;
    reg [1:0] wav_job_slot;reg wav_job_bank;
    // WAV 文件键宽为 KEY_W，seq=0 由提交器判为非法描述符，分配时须跳过零，
    // 所以回绕必须跳过0：8位时在64..127内循环（127之后回到64），与BMP键4..7/16..19、
    // 首次挂载/目录2/3、恢复挂载/目录128..255（见下）互不重叠。KEY_W=32 时从 64 开始分配序号，较窄编号回绕时跳过零。
    reg [KEY_W-1:0] wav_sequence;reg first_wave_attempted,wav_return_first;
    localparam [KEY_W-1:0] WAV_KEY_FIRST=64,WAV_KEY_LAST=127,BMP_SCAN_BASE=4,BMP_LOAD_BASE=16;
    wire [KEY_W-1:0] wav_sequence_next=(KEY_W<32 && wav_sequence==WAV_KEY_LAST) ? WAV_KEY_FIRST : wav_sequence+1'b1;
    // 恢复阶段的挂载和目录请求由 epoch 及末位类型区分。
    // KEY_W=32 时保留完整初始化 epoch；较窄键使用可容纳的 epoch 低位，并与文件请求键分区。
    wire [KEY_W-1:0] mount_seq_key,dir_seq_key;
    // BMP键：目录扫描4..7、装载16..19。
    wire [KEY_W-1:0] bmp_scan_key=BMP_SCAN_BASE+{{(KEY_W-2){1'b0}},index};
    wire [KEY_W-1:0] bmp_load_key=BMP_LOAD_BASE+{{(KEY_W-2){1'b0}},index};
    // 键宽只支持 8 位或 32 位，其他配置由参数检查直接拒绝。
    generate if(KEY_W!=8 && KEY_W!=32) begin:g_invalid_key_width
        KEY_W_MUST_BE_8_OR_32 invalid_configuration();
    end endgenerate
    generate if(KEY_W<32) begin:g_short_keys
        assign mount_seq_key=init_epoch_q==1 ? {{(KEY_W-2){1'b0}},2'd2} : {1'b1,init_epoch_q[KEY_W-3:0],1'b0};
        assign dir_seq_key  =init_epoch_q==1 ? {{(KEY_W-2){1'b0}},2'd3} : {1'b1,init_epoch_q[KEY_W-3:0],1'b1};
    end else begin:g_full_keys
        assign mount_seq_key=init_epoch_q==1 ? 32'd2 : {1'b1,22'd0,init_epoch_q,1'b0};
        assign dir_seq_key  =init_epoch_q==1 ? 32'd3 : {1'b1,22'd0,init_epoch_q,1'b1};
    end endgenerate
    wire wav_job_valid,wav_job_ready,wav_job_bank_next;
    wire [1:0] wav_job_slot_next;
    wire [18:0] file_written_words;
    wire wav_done=(state==WSCAN_WAIT && scan_result_valid && scan_code!=0) ||
                  (state==WLOAD_WAIT && result_valid);
    wire wav_good=state==WLOAD_WAIT && result_valid && result_code==0 && committed;
    // I/O 型失败：失败当拍卡已 media_fault（扇区读 R1/令牌错误/超时）。缺文件、坏格式、取消都不置 media_fault。
    // 只要卡未判定失效且这不是第 3 次连续 I/O 失败，就按“暂时”处理：不写永久 unavailable，恢复后重装。
    wire io_transient=media_fault_w && !card_dead && io_streak!=2'd2;
    wire wav_transient=wav_done && !wav_good && io_transient;
    // WAV 服务期间卡失格：先恢复（重新初始化+挂载+找目录）再接下一个 WAV 作业。
    wire need_recover=media_fault_w && !card_dead;
    assign wav_job_ready=ENABLE_WAV && !rst_logic && (state==STOP && done && !need_recover || ENABLE_AUDIO_CONTROL && state==TABLE && !shown && !first_wave_attempted && commit_ready && slot_reserved==0) && !quarantined;
    wire [1:0] wav_write_permission={wav_bank_states[5:3]==3'd1,wav_bank_states[2:0]==3'd1};
    wav_bank_manager u_wav_banks(.clk(clk),.rst(rst_logic),.enable(ENABLE_WAV && (done || ENABLE_AUDIO_CONTROL && !shown && loaded[first_slot])),.loaded(loaded),
        .current_valid(wav_current_valid),.current_slot(wav_current_slot),
        .want_valid(wav_want_valid),.want_slot(wav_want_slot),.ignore_present(ENABLE_AUDIO_CONTROL ? wav_ignore_present : 2'd0),
        .pin_valid(wav_pin_valid),.pin_bank(wav_pin_bank),.retire_valid(wav_retire_valid),.retire_bank(wav_retire_bank),
        .release_bank(wav_release_bank),.job_valid(wav_job_valid),.job_ready(wav_job_ready),
        .job_bank(wav_job_bank_next),.job_slot(wav_job_slot_next),.job_done(wav_done),.job_ok(wav_good),.job_transient(wav_transient),
        .job_words(file_written_words[17:0]),.bank_states(wav_bank_states),.bank_slots(wav_bank_slots),
        .bank_words(wav_bank_words),.bank_ready(wav_bank_ready),.unavailable(wav_unavailable),.fault(wav_fault));
    // 目录API把第0个字符放低字节；Verilog字符串字面量恰好相反，显式转换。
    function [87:0] name_le;input [87:0] text;integer j;
        begin for(j=0;j<11;j=j+1) name_le[j*8 +:8]=text[87-j*8 -:8];end
    endfunction
    wire [87:0] bmp_name=index==0?name_le("P0001   BMP"):index==1?name_le("P0002   BMP"):
                           index==2?name_le("P0003   BMP"):name_le("P0004   BMP");
    wire [87:0] wav_name=wav_job_slot==0?name_le("P0001   WAV"):wav_job_slot==1?name_le("P0002   WAV"):
                           wav_job_slot==2?name_le("P0003   WAV"):name_le("P0004   WAV");
    wire [87:0] file_name=wav_processing ? wav_name : bmp_name;
    // 请求键在每个ID中唯一，后续文件内部extent序号由FAT层管理。
    wire [KEY_W-1:0] load_seq=wav_processing ? wav_sequence : bmp_load_key;
    assign commit_valid=raw_commit && !wav_processing;assign commit_slot=active_slot;assign commit_gen=active_gen;
    assign desc_valid=!rst_logic && state==DESC;
    assign desc_bus={{{(EPOCH_W-1){1'b0}},1'b1},32'd0,32'd1,first_slot,first_slot,32'd0,124'd0};
    assign ack_ready=!rst_logic && state==ACK;
    // cancel只取消当前文件；装载器将取消保持到生产者和物理写均排空。
    // load_cancel不直通物理控制器：已接受BL8始终供满八拍，避免违反控制器contract A。
    fat32_bmp_loader #(.BMP32_EN(BMP32_EN),.EXT_BMP_FORMATS(EXT_BMP_FORMATS),.KEY_W(KEY_W),.TIME_W(TIME_W),.FILE_LEN_W(FILE_LEN_W),.CARD_SECTOR_W(CARD_SECTOR_W),.ENABLE_THUMBNAILS(ENABLE_THUMBNAILS),.ENABLE_WAV(ENABLE_WAV),.ENABLE_AUDIO_CONTROL(ENABLE_AUDIO_CONTROL),.EXTERNAL_MEMORY(1),.USE_EG_BRAM(USE_EG_BRAM),
        .CONTEXT_ENABLE(CONTEXT_ENABLE),.EPOCH_W(EPOCH_W),
        .POWER_CYCLES(POWER_CYCLES),.POLL_CYCLES(POLL_CYCLES),
        .INIT_TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES),.READ_WAIT_CYCLES(READ_WAIT_CYCLES),
        .DRAIN_CYCLES(DRAIN_CYCLES)) u_loader(
                .audio_req_valid(audio_req_valid),.audio_req_ready(audio_req_ready),.audio_req_cancel(audio_req_cancel),.audio_req_bank(audio_req_bank),
        .audio_req_words(audio_req_words),.audio_req_token(audio_req_token),
        .audio_rsp_valid(audio_rsp_valid),.audio_rsp_status(audio_rsp_status),.audio_rsp_bank(audio_rsp_bank),.audio_rsp_token(audio_rsp_token),
        .audio_mem_valid(audio_mem_valid),.audio_mem_ready(audio_mem_ready),.audio_mem_cancel(audio_mem_cancel),.audio_mem_bank(audio_mem_bank),
        .audio_mem_words(audio_mem_words),.audio_mem_token(audio_mem_token),
        .audio_mem_rsp_valid(audio_mem_rsp_valid),.audio_mem_rsp_ready(audio_mem_rsp_ready),.audio_mem_rsp_status(audio_mem_rsp_status),
        .audio_mem_rsp_bank(audio_mem_rsp_bank),.audio_mem_rsp_token(audio_mem_rsp_token),
.clk(clk),.mem_clk(mem_clk),.arst(arst),
        .init_valid(!rst_logic && state==INIT),.init_ready(init_ready),.init_epoch(init_epoch_q),
        .init_abort_valid(1'b0),.init_abort_epoch(0),.init_abort_ready(),
        .init_done_valid(init_done_valid),.init_done_ready(state==INIT_WAIT),.init_done_code(init_code),
        .init_done_epoch(),.init_done_register(),.init_done_command(),.init_done_r1(),
        .card_qualified(),.card_block_addressed(),.media_fault(media_fault_w),.card_initialized(),.card_last_lba(),.card_sector_count(),
        .mount_valid(!rst_logic && state==MOUNT),.mount_ready(mount_ready),.mount_session(1),.mount_seq(mount_seq_key),
        .mount_budget_cycles(SYS_BUDGET),.mount_cancel_valid(1'b0),.mount_cancel_session(0),.mount_cancel_seq(0),
        .mount_cancel_ready(),.mount_busy(),.mount_result_valid(mount_result_valid),.mount_result_ready(state==MOUNT_WAIT),
        .mount_result_session(),.mount_result_seq(),.mount_result_elapsed(),.mount_result_code(mount_code),.mount_result_detail(),
        .mounted(),.partition_end(),.volume_end(),.fat_lba(),.data_lba(),.root_lba(),.fat_sectors(),
        .root_cluster(root_cluster),.cluster_count(),.cluster_shift(),.active_fat(),
        .scan_valid(!rst_logic && (state==DIR || state==SCAN || (ENABLE_WAV && state==WSCAN))),.scan_ready(scan_ready),
        .scan_session(1),.scan_seq(wav_processing ? wav_sequence : state==DIR ? dir_seq_key : bmp_scan_key),.scan_budget_cycles(SYS_BUDGET),
        .scan_target_name(state==DIR?name_le("MEDIA      "):file_name),.scan_start_cluster(state==DIR?root_cluster:media_cluster),
        .scan_want_dir(state==DIR),.scan_sector_limit(6'd32),
        .scan_cancel_valid(1'b0),.scan_cancel_session(0),.scan_cancel_seq(0),.scan_cancel_ready(),
        .scan_busy(),.scan_result_valid(scan_result_valid),.scan_result_ready(state==DIR_WAIT || state==SCAN_WAIT || (ENABLE_WAV && state==WSCAN_WAIT)),
        .scan_result_session(),.scan_result_seq(),.scan_result_elapsed(),.scan_result_code(scan_code),.scan_result_detail(),
        .hit_cluster(hit_cluster),.hit_size(hit_size),
        .spi_cs_n(spi_cs_n),.spi_sclk(spi_sclk),.spi_mosi(spi_mosi),.spi_miso(spi_miso),
        .req_valid(!rst_logic && (state==LOAD || (ENABLE_WAV && state==WLOAD))),.req_ready(req_ready),.req_session(1),.req_seq(load_seq),
        .req_cluster(file_cluster),.req_length(file_size),.req_slot_gen(32'd1),.req_slot(wav_processing ? {1'b0,wav_job_bank} : index),.slot_writable(slot_writable),
        .req_wav(wav_processing),.wav_bank_writable(wav_write_permission),
        .req_src_budget(SYS_BUDGET),.req_mem_budget(MEM_BUDGET),.req_crc_enable(1'b0),.req_crc(32'd0),
        .cancel_valid(cancel),.cancel_session(1),.cancel_seq(load_seq),.busy(),.quarantined(quarantined),
        .slot_reserved(slot_reserved),.slot_valid(),.commit_valid(raw_commit),.commit_ready(wav_processing ? 1'b1 : commit_ready),
        .active_session(),.active_seq(),.active_slot_gen(active_gen),.active_slot(active_slot),.actual_crc(),
        .result_valid(result_valid),.result_ready(state==LOAD_WAIT || (ENABLE_WAV && state==WLOAD_WAIT)),.result_code(result_code),.written_words(file_written_words),.thumbnail_words(),
        .load_valid(load_valid),.load_ready(load_ready),.load_addr(load_addr),.load_count(load_count),.load_budget(load_budget),
        .load_beat(load_beat),.load_done(load_done),.load_done_was_write(load_done_was_write),
        .wr_data_valid(load_data_valid),.wr_data(load_data),
        .sd_cs_n(),.sd_ras_n(),.sd_cas_n(),.sd_we_n(),.sd_cke(),.sd_addr(),.sd_ba());
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=INIT;wav_processing_q<=0;first_wave_attempted<=0;wav_return_first<=0;wav_job_slot<=0;wav_job_bank<=0;wav_sequence<=WAV_KEY_FIRST;index<=0;first_slot<=0;media_cluster<=0;file_cluster<=0;file_size24<=0;file_too_long<=0;
            shown<=0;committed<=0;loaded<=0;failed<=0;done<=0;error_code<=0;age<=0;
            init_epoch_q<=8'd1;retry_left<=RETRY_LIMIT[1:0];recovering<=0;card_dead<=0;
            bmp_io_retried<=0;resume_state<=STOP;io_streak<=0;
        end else begin
            if(quarantined || wav_done) wav_processing_q<=0;
            else if(wav_job_valid && wav_job_ready) wav_processing_q<=1;
            if(raw_commit && (wav_processing || commit_ready)) committed<=1;
            // 连续 I/O 失败计数：WAV/BMP 文件 I/O 失败时加一（第 3 次判卡失效）；任一文件成功清零。
            if((wav_done && !wav_good && media_fault_w && !card_dead) ||
               ((state==SCAN_WAIT && scan_result_valid && scan_code!=0 || state==LOAD_WAIT && result_valid && !(result_code==0 && committed)) && media_fault_w && !card_dead)) begin
                if(io_streak==2'd2) card_dead<=1;else io_streak<=io_streak+2'd1;
            end else if(wav_good || (state==LOAD_WAIT && result_valid && result_code==0 && committed)) io_streak<=0;
            // 只给首次引用的表/描述符/响应等待设置控制模块看门狗；文件和控制事务自带预算。
            if(state==TABLE || state==DESC || state==ACK || state==RETRY_WAIT) age<=age+1'b1;else age<=0;
            if(quarantined) begin state<=STOP;error_code<=8'he1;done<=1;end
            else if((state==TABLE || state==DESC || state==ACK) && age>=SYS_BUDGET-1) begin
                state<=STOP;error_code<=8'he2;done<=1;
            end else case(state)
                INIT:if(init_ready) state<=INIT_WAIT;
                INIT_WAIT:if(init_done_valid) begin
                    if(init_code==0) state<=MOUNT;
                    else if(retry_left!=0) begin retry_left<=retry_left-2'd1;state<=RETRY_WAIT;end
                    else if(recovering) begin recovering<=0;card_dead<=1;state<=resume_state;end
                    else begin state<=STOP;error_code<=8'h10|{4'd0,init_code};done<=1;end
                end
                MOUNT:if(mount_ready) state<=MOUNT_WAIT;
                MOUNT_WAIT:if(mount_result_valid) begin
                    if(mount_code==0) state<=DIR;
                    else if(retry_left!=0) begin retry_left<=retry_left-2'd1;state<=RETRY_WAIT;end
                    else if(recovering) begin recovering<=0;card_dead<=1;state<=resume_state;end
                    else begin state<=STOP;error_code<=8'h20|{4'd0,mount_code};done<=1;end
                end
                DIR:if(scan_ready) state<=DIR_WAIT;
                DIR_WAIT:if(scan_result_valid) begin
                    // 恢复成功：回到被打断处；首次启动照旧开始扫描 P0001。
                    if(scan_code==0) begin media_cluster<=hit_cluster;state<=recovering ? resume_state : SCAN;recovering<=0;end
                    else if(retry_left!=0) begin retry_left<=retry_left-2'd1;state<=RETRY_WAIT;end
                    else if(recovering) begin recovering<=0;card_dead<=1;state<=resume_state;end
                    else begin state<=STOP;error_code<=8'h30|{4'd0,scan_code};done<=1;end
                end
                // 装 BMP 前卡已失格：先恢复，成功后回到 SCAN 同一 ID。
                SCAN:if(need_recover) begin
                        recovering<=1;resume_state<=SCAN;retry_left<=RETRY_LIMIT[1:0];init_epoch_q<=init_epoch_next;state<=INIT;
                    end else if(scan_ready) state<=SCAN_WAIT;
                // BMP 扫描/装载 I/O 失败：同一 ID 只再试一次（回 SCAN 触发恢复），再失败才记 failed。
                SCAN_WAIT:if(scan_result_valid) begin
                    if(scan_code==0) begin file_cluster<=hit_cluster;file_size24<=hit_size[23:0];file_too_long<=|hit_size[31:24];committed<=0;state<=LOAD;end
                    else if(io_transient && !bmp_io_retried) begin bmp_io_retried<=1;state<=SCAN;end
                    else begin failed[index]<=1;state<=NEXT;end
                end
                LOAD:if(req_ready) state<=LOAD_WAIT;
                LOAD_WAIT:if(result_valid) begin
                    if(result_code==0 && committed) begin
                        loaded[index]<=1;
                        if(!shown) begin first_slot<=index;state<=TABLE;end else state<=NEXT;
                    end else if(io_transient && !bmp_io_retried) begin bmp_io_retried<=1;state<=SCAN;end
                    else begin failed[index]<=1;state<=NEXT;end
                end
                // commit_ready 恢复表示上一条表消息已被像素端接收，此时可继续描述符交接。
                TABLE:if(commit_ready && slot_reserved==0) begin
                    if(ENABLE_AUDIO_CONTROL && !first_wave_attempted) begin
                        if(wav_job_valid && wav_job_ready) begin
                            wav_job_slot<=wav_job_slot_next;wav_job_bank<=wav_job_bank_next;
                            wav_sequence<=wav_sequence_next;wav_return_first<=1;committed<=0;state<=WSCAN;
                        end
                    end else state<=DESC;
                end
                DESC:if(desc_ready) state<=ACK;
                ACK:if(ack_valid) begin
                    if(ack_bus[EPOCH_W+20:EPOCH_W+18]==0 && ack_bus[EPOCH_W+17:18]==1 && ack_bus[1:0]==first_slot) begin shown<=1;state<=NEXT;end
                    else begin state<=STOP;error_code<=8'h40|{5'd0,ack_bus[EPOCH_W+20:EPOCH_W+18]};done<=1;end
                end
                NEXT:begin bmp_io_retried<=0;if(index==3) begin state<=STOP;done<=1;end else begin index<=index+2'd1;state<=SCAN;end end
                // 启动结束后的 WAV 服务：卡失格先恢复，恢复期间不接新 WAV 作业（wav_job_ready 已门控）。
                STOP:if(ENABLE_WAV && done && need_recover) begin
                    recovering<=1;resume_state<=STOP;retry_left<=RETRY_LIMIT[1:0];init_epoch_q<=init_epoch_next;state<=INIT;
                end else if(wav_job_valid && wav_job_ready) begin
                    wav_job_slot<=wav_job_slot_next;wav_job_bank<=wav_job_bank_next;
                    wav_sequence<=wav_sequence_next;wav_return_first<=0;committed<=0;state<=WSCAN;
                end
                WSCAN:if(ENABLE_WAV) begin if(scan_ready) state<=WSCAN_WAIT;end else state<=STOP;
                WSCAN_WAIT:if(scan_result_valid) begin
                    if(scan_code==0) begin file_cluster<=hit_cluster;file_size24<=hit_size[23:0];file_too_long<=|hit_size[31:24];state<=WLOAD;end
                    else begin
                        if(wav_return_first) begin first_wave_attempted<=1;state<=TABLE;end else state<=STOP;
                    end
                end
                WLOAD:if(req_ready) state<=WLOAD_WAIT;
                WLOAD_WAIT:if(result_valid) begin
                    if(wav_return_first) begin first_wave_attempted<=1;state<=TABLE;end else state<=STOP;
                end
                // 两次尝试之间等待约 100ms，再用新 epoch 重新初始化。
                RETRY_WAIT:if(age>=RETRY_WAIT_CYCLES-1) begin init_epoch_q<=init_epoch_next;state<=INIT;end
                default:state<=STOP;
            endcase
        end
    end
endmodule
`default_nettype wire
