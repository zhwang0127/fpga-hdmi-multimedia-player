`timescale 1ns/1ps
`default_nettype none
// FAT 文件接口与 BMP/WAV 装载提交；clk 为系统时钟域，mem_clk 为 SDRAM 时钟域。
// file result/sink abort使用文件(session,seq)；out_*使用内部extent键，不可混为同一seq。
// slot_writable来自引用控制模块；只允许独占无读引用槽。slot_valid只在commit握手置位。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module bmp_file_commit #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter EXT_BMP_FORMATS=0,
    parameter BMP32_EN=1,
    parameter integer KEY_W=32,
    // 默认生成缩略图；关闭时完成条件为 307200 个主图字且缩略图字数为零。
    parameter ENABLE_THUMBNAILS=1,
    parameter ENABLE_WAV=0,ENABLE_AUDIO_CONTROL=0,EXTERNAL_MEMORY=0,DRAIN_CYCLES=500000,POWER_WAIT_CYCLES=25000,REFRESH_COUNT=8,
    parameter REFRESH_INTERVAL=937,REFRESH_GRACE=937
)(
    // u_cmd/u_event 同时传送音频控制，跨域实例路径与逐网约束配套。
    input wire audio_req_valid,output wire audio_req_ready,input wire audio_req_cancel,audio_req_bank,
    input wire [17:0] audio_req_words,input wire [15:0] audio_req_token,
    output wire audio_rsp_valid,output wire [1:0] audio_rsp_status,output wire audio_rsp_bank,output wire [15:0] audio_rsp_token,
    output wire audio_mem_valid,input wire audio_mem_ready,output wire audio_mem_cancel,audio_mem_bank,
    output wire [17:0] audio_mem_words,output wire [15:0] audio_mem_token,
    input wire audio_mem_rsp_valid,output wire audio_mem_rsp_ready,input wire [1:0] audio_mem_rsp_status,
    input wire audio_mem_rsp_bank,input wire [15:0] audio_mem_rsp_token,
    input wire clk,mem_clk,arst,
    input wire req_valid,output wire req_ready,
    input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_cluster,req_length,req_slot_gen,
    input wire [1:0] req_slot,input wire [3:0] slot_writable,
    // ENABLE_WAV=1时复用原文件事务与CDC；WAV音频缓冲区许可独立于图像槽引用。
    input wire req_wav,input wire [1:0] wav_bank_writable,
    input wire [31:0] req_src_budget,req_mem_budget,
    input wire req_crc_enable,input wire [31:0] req_crc,
    input wire cancel_valid,input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire busy,quarantined,output wire [3:0] slot_reserved,slot_valid,
    output wire commit_valid,input wire commit_ready,
    output wire [KEY_W-1:0] active_session,active_seq, output wire [31:0] active_slot_gen,actual_crc,
    output wire [1:0] active_slot,
    output wire result_valid,input wire result_ready,output wire [7:0] result_code,
    output wire [18:0] written_words,output wire [13:0] thumbnail_words,
    // 接fat32_storage_services的真实系统域文件请求/取消/结果/消费者排空握手。
    output wire file_valid,input wire file_ready,
    output wire [KEY_W-1:0] file_session,file_seq, output wire [31:0] file_start_cluster,file_size,file_budget_cycles,
    output wire file_cancel_valid,input wire file_cancel_ready,
    output wire [KEY_W-1:0] file_cancel_session,file_cancel_seq,
    input wire file_busy,file_result_valid,output wire file_result_ready,
    input wire [KEY_W-1:0] file_result_session,file_result_seq, input wire [31:0] file_result_count,input wire [3:0] file_result_code,
    input wire file_sink_abort_req,input wire [KEY_W-1:0] file_sink_abort_session,file_sink_abort_seq,
    output wire file_sink_abort_ack,output wire [KEY_W-1:0] file_sink_ack_session,file_sink_ack_seq,
    // mem_clk域量响应；当前量的解析流水、待发写和在途写都排空后才有效。
    output wire quantum_receipt_valid,input wire quantum_receipt_ready,
    output wire [KEY_W-1:0] quantum_receipt_ticket,
    // 已有file_byte_cdc目标域接口；metadata被消费但不送像素/CRC。
    input wire out_valid,output wire out_ready,input wire [7:0] out_data,input wire out_last,
    input wire [1:0] out_user,input wire [KEY_W-1:0] out_session,out_seq,
    input wire dst_stop_req,output wire dst_stop_ack,
    // EXTERNAL_MEMORY=1时接产品唯一共享控制器（mem_clk域）；禁止并接命令总线。
    // 已接受BL8在取消时仍连续供满八拍；完成须来自真实PRE/done，不撤销在途所有权。
    output wire load_valid,input wire load_ready,output wire [20:0] load_addr,
    output wire [3:0] load_count,output wire [31:0] load_budget,
    input wire load_beat,load_done,load_done_was_write,
    output wire wr_data_valid,output wire [31:0] wr_data,
    output wire sd_cs_n,sd_ras_n,sd_cas_n,sd_we_n,sd_cke,
    output wire [10:0] sd_addr,output wire [1:0] sd_ba
);
    generate if(ENABLE_AUDIO_CONTROL && !ENABLE_WAV) begin:g_invalid_audio_configuration
        AUDIO_CONTROL_REQUIRES_WAV invalid_configuration();
    end endgenerate
    wire rst_logic;
    wire mrst_logic;
    localparam IDLE=0,RUN=1,DRAIN=2,READY=3,RESULT=4;
    localparam OK=0,DESCRIPTOR=1,SOURCE=2,MEDIA=3,CRC=4,PROTOCOL=5,CANCELED=6;
    reg [2:0] phase;reg [7:0] first_code;reg [31:0] length,expected_crc,slot_gen;
    reg check_crc,source_seen,source_good,mem_seen,cmd_delivered,abort_sent,cancel_sent;
    reg quarantine;
    // 计数位宽 = 恰好容纳“阈值本身”的位数（counter_width(P+1)）。
    // 计数器在达到阈值前递增，到达后离开计数状态或置位完成标志
    // 位宽需覆盖阈值本身，避免计数到达上限前发生回绕。
    function integer counter_width;
        input integer value;
        integer v;
        begin v=value-1;counter_width=1;while(v>1) begin v=v>>1;counter_width=counter_width+1;end end
    endfunction
    // drain_age 到 DRAIN_CYCLES-1 即置 quarantine 并停止递增，最大值≤DRAIN_CYCLES-1。
    localparam integer DRAIN_W=counter_width(DRAIN_CYCLES+1); // 产品7,500,000 -> 23位
    reg [DRAIN_W-1:0] drain_age;reg [3:0] valid_slots;
    wire rst,mrst;reset_sync u_sys_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    reset_sync u_mem_reset(.clk(mem_clk),.arst(arst),.rst(mrst),.logic_rst(mrst_logic));
    // 业务门控只使用本端reset_sync输出；该rst异步置位、三级同步释放，
    // 已覆盖arst有效区间。原始arst仅进入复位链，避免sys复位直接穿过mem数据/握手锥。
    wire enabled=!rst_logic;
    wire tx_ready,tx_ready_valid,tx_busy,tx_abort;wire [2:0] tx_reason;
    wire active_phase=phase==RUN || phase==DRAIN || phase==READY;
    wire cancel_match=cancel_valid && active_phase && cancel_session==active_session && cancel_seq==active_seq;
    wire source_match=file_result_session==active_session && file_result_seq==active_seq;
    wire source_abort_match=active_phase && file_sink_abort_req && file_sink_abort_session==active_session && file_sink_abort_seq==active_seq;
    // 命令/事件/取消三个mailbox按KEY_W定宽，CDC结构不变（仍整包保持+翻转位双级同步）。
    // 命令总线自低到高：slot[1:0]、mem预算[33:2]、长度[65:34]、seq字段[66+SEQF_W-1:66]、session、
    // 再往上是 WAV/取消位与音频位。启用音频控制时 seq 字段至少占 16 位，
    // 其低 16 位用于传递音频 token。
    // 文件命令使用低 KEY_W 位，高位补零，音频和文件载荷由类型位区分。
    localparam integer SEQF_W=(ENABLE_AUDIO_CONTROL!=0 && KEY_W<16) ? 16 : KEY_W;
    localparam integer CMD_SEQ_LO=66;
    localparam integer CMD_SES_LO=CMD_SEQ_LO+SEQF_W;
    localparam integer CMD_TOP=CMD_SES_LO+KEY_W;          // WAV/取消位位置
    localparam CMD_W=CMD_TOP+(ENABLE_WAV!=0)+(ENABLE_AUDIO_CONTROL!=0);
    // 事件总线：{session,seq,count[32],crc[32],written[19],thumb[14],ok,detail[8],stream_code[8]}，
    // detail[8]（[15:8]）字段保留位置但恒送0。sys 侧从不读文件事件的 detail
    // （完成判定只看 ok[16]、stream_code[7:0]、count/crc/written/thumb），但开启音频控制时 [15:0] 同时是音频响应 token，
    // 导致综合无法删掉 mem 侧 BMP 头/块/WAV 解析器只作诊断的 detail 逻辑。各装载器仍单独提供 result_detail 输出。
    // 低114位布局不变，键字段在[113+2*KEY_W:114]；音频位在最高位。
    localparam integer EVT_KEY_LO=114;
    localparam EVENT_W=EVT_KEY_LO+2*KEY_W+(ENABLE_AUDIO_CONTROL!=0);
    wire cmd_ready,cmd_done;wire cmd_valid,cmd_take;wire [CMD_W-1:0] cmd_bus;
    wire [CMD_W-1:0] cmd_input;
    wire req_fire,cmd_fire;
    wire cmd_is_audio,event_is_audio;
    wire audio_request=ENABLE_AUDIO_CONTROL && audio_req_valid;
    wire audio_response=ENABLE_AUDIO_CONTROL && audio_mem_rsp_valid;
    wire file_event_send;
    wire req_is_wav=ENABLE_WAV && req_wav;
    wire cmd_is_wav;
    reg sys_wav_q,mem_wav_q;
    wire sys_is_wav=ENABLE_WAV && sys_wav_q;
    wire mem_is_wav=ENABLE_WAV && mem_wav_q;
    wire event_src_ready,event_valid;
    wire [EVENT_W-1:0] event_bus;
    generate if(ENABLE_AUDIO_CONTROL) begin:g_audio_command
        // 132bit：最高位区分音频，次高位在音频时是CANCEL、文件时是WAV类型。
        // 音频token复用seq[15:0]，words复用length[17:0]，bank复用slot[0]。
        // 其余95位保持文件值，避免把本来常量的字段随音频清0而制造FF/门控；
        // 类型为音频时文件控制模块不消费这些位。mailbox仍只在真实接受沿锁整包。
        // seq字段：文件命令=零扩展的req_seq；音频命令=低16位换成token（高位保留req_seq零扩展值，
        // 不随音频清0，避免把常量位变成门控）。
        wire [SEQF_W-1:0] req_seq_field=req_seq;
        wire [SEQF_W-1:0] cmd_seq_field;
        if(SEQF_W>16) begin:g_seq_wide
            assign cmd_seq_field={req_seq_field[SEQF_W-1:16],audio_request ? audio_req_token : req_seq_field[15:0]};
        end else begin:g_seq_16
            assign cmd_seq_field=audio_request ? audio_req_token : req_seq_field;
        end
        assign cmd_input={audio_request,audio_request ? audio_req_cancel : req_is_wav,
            req_session,cmd_seq_field,
            req_length[31:18],audio_request ? audio_req_words : req_length[17:0],
            req_mem_budget,req_slot[1],audio_request ? audio_req_bank : req_slot[0]};
        assign cmd_is_audio=cmd_bus[CMD_TOP+1];assign cmd_is_wav=cmd_bus[CMD_TOP] && !cmd_is_audio;
        assign event_is_audio=event_bus[EVT_KEY_LO+2*KEY_W];
        assign audio_mem_cancel=cmd_bus[CMD_TOP];assign audio_mem_token=cmd_bus[CMD_SEQ_LO+15:CMD_SEQ_LO];
        assign audio_mem_words=cmd_bus[51:34];assign audio_mem_bank=cmd_bus[0];
        assign audio_rsp_status=event_bus[18:17];assign audio_rsp_bank=event_bus[16];assign audio_rsp_token=event_bus[15:0];
    end else begin:g_file_command
        if(ENABLE_WAV) begin:g_wav
            assign cmd_input={req_is_wav,req_session,req_seq,req_length,req_mem_budget,req_slot};
            assign cmd_is_wav=cmd_bus[CMD_TOP];
        end else begin:g_bmp
            assign cmd_input={req_session,req_seq,req_length,req_mem_budget,req_slot};assign cmd_is_wav=1'b0;
        end
        assign cmd_is_audio=1'b0;assign event_is_audio=1'b0;
        assign audio_mem_cancel=1'b0;assign audio_mem_token=0;assign audio_mem_words=0;assign audio_mem_bank=0;
        assign audio_rsp_status=0;assign audio_rsp_bank=0;assign audio_rsp_token=0;
    end endgenerate
    wire m_enabled;reg m_active;
    assign audio_req_ready=ENABLE_AUDIO_CONTROL && enabled && cmd_ready && !quarantine;
    assign audio_mem_valid=ENABLE_AUDIO_CONTROL && m_enabled && cmd_valid && cmd_is_audio && !m_active;
    assign audio_rsp_valid=ENABLE_AUDIO_CONTROL && event_valid && event_is_audio;
    assign audio_mem_rsp_ready=ENABLE_AUDIO_CONTROL && event_src_ready;
    always @(posedge clk or posedge rst) if(rst) sys_wav_q<=0;else if(req_fire) sys_wav_q<=req_is_wav;
    always @(posedge mem_clk or posedge mrst) if(mrst) mem_wav_q<=0;else if(cmd_fire) mem_wav_q<=cmd_is_wav;
    wire abort_ready,abort_valid;
    wire source_error=(source_seen && !source_good) || source_abort_match;
    
    // mem_seen与这些快照在同一个实际event_fire沿生效；不增加完成等待拍。
    // 媒体事件只由当前command的真实终结发送，下一command前总线保持；判定仍用原字段。
    // 避免received/两sink的加法和比较经failure/cancel进入prefetch/FAT解析使能。
    // 请求沿只清mem_seen使旧快照失效，不重复清快照寄存器，避免req_fire长链进入其CE。
    reg [31:0] received_count_q,sink_count_q;
    reg media_good_q,crc_good_q;
    reg [18:0] file_written_q;reg [13:0] file_thumb_q;reg [31:0] file_crc_q;
    wire [31:0] event_received=event_bus[113:82];
    wire [31:0] received=received_count_q;
    // WAV子模块独立核PCM声明长度、实际PCM字节数和物理BL8接受数；此处还核源文件流的完整性和结束结果。
    wire media_good_live=event_bus[16] && event_bus[7:0]==0 && event_received==length &&
        (sys_is_wav ? (event_bus[49:31]>=19'd48000 && event_bus[49:31]<=19'd240000 && event_bus[30:17]==0) :
         (event_bus[49:31]==19'd307200 && event_bus[30:17]==(ENABLE_THUMBNAILS ? 14'd12288 : 14'd0)));
    wire media_good=media_good_q;
    wire sink_error=mem_seen && !media_good;
    // event_fire当沿核的是本次事件；file_crc_q/actual_crc在该沿之后才更新。
    // 使用旧快照会在C共享音频邮箱模式下把第一份合法CRC文件误判为失败。
    // 对外actual_crc仍保留文件快照，避免随后音频响应覆盖其可观察值。
    wire crc_good_live=!check_crc || event_bus[81:50]==expected_crc;
    wire crc_good=crc_good_q;
    wire crc_error=mem_seen && media_good && !crc_good;
    wire failure=first_code!=OK || source_error || sink_error || crc_error || cancel_match;
    wire abort_send=active_phase && (failure || tx_abort) && cmd_delivered && !abort_sent && !mem_seen;wire [2*KEY_W-1:0] abort_bus;
    
    wire event_match=event_bus[EVT_KEY_LO+2*KEY_W-1:EVT_KEY_LO]=={active_session,active_seq};
    wire event_fire=event_valid && !event_is_audio && event_match && (phase==RUN || phase==DRAIN);
    wire source_fire=file_result_valid && file_result_ready && source_match;

    assign actual_crc=ENABLE_AUDIO_CONTROL ? file_crc_q : event_bus[81:50];
    assign written_words=mem_seen ? (ENABLE_AUDIO_CONTROL ? file_written_q : event_bus[49:31]) : 19'd0;
    assign thumbnail_words=mem_seen ? (ENABLE_AUDIO_CONTROL ? file_thumb_q : event_bus[30:17]) : 14'd0;

    assign req_ready=enabled && phase==IDLE && !quarantine && !cancel_valid &&
                     tx_ready && cmd_ready && !audio_request && file_ready && (req_is_wav ? wav_bank_writable[req_slot[0]] : slot_writable[req_slot]);
    assign req_fire=req_valid && req_ready;
    assign file_valid=req_fire;
    assign file_session=req_session;assign file_seq=req_seq;assign file_start_cluster=req_cluster;
    assign file_size=req_length;assign file_budget_cycles=req_src_budget;
    assign file_cancel_valid=enabled && (phase==RUN || phase==DRAIN || phase==READY) &&
        (failure || tx_abort) && !source_seen && !cancel_sent;
    assign file_cancel_session=active_session;assign file_cancel_seq=active_seq;
    assign file_result_ready=enabled && (phase==RUN || phase==DRAIN || phase==READY);
    assign file_sink_abort_ack=enabled && source_abort_match && mem_seen;
    assign file_sink_ack_session=active_session;assign file_sink_ack_seq=active_seq;
    assign busy=phase!=IDLE || quarantine;
    assign quarantined=quarantine;
    assign slot_reserved=busy && !sys_is_wav ? (4'b0001<<active_slot) : 4'd0;
    assign slot_valid=valid_slots;
    assign active_slot_gen=slot_gen;
    assign commit_valid=enabled && phase==READY && tx_ready_valid && !failure && !quarantine;
    assign result_valid=enabled && phase==RESULT;
    assign result_code=first_code;
    // src_done还需mem侧CRC/完整文件计数；sink精确计数分别校核，不能用总和掩盖互抵。
    load_txn #(.KEY_W(KEY_W)) u_txn(.clk(clk),.rst(rst_logic),.req_valid(req_fire),.req_ready(tx_ready),
        .req_session(req_session),.req_seq(req_seq),.req_slot(req_slot),
        // BMP计数单位仍为写入字；WAV的计数单位是“已在子模块独立核完字节数、
        // 声明长度和实际写入数的一个完整音频缓冲区”。期望1在请求沿锁存，不能事后
        // 把未知PCM长度或实际完成计数反喂成期望来掩盖少写/多写。
        .expected_src_count(req_length),.expected_sink_count(req_is_wav ? 32'd1 : (ENABLE_THUMBNAILS ? 32'd319488 : 32'd307200)),
        .src_done(source_seen && source_good && mem_seen && media_good && crc_good),
        .src_error(source_error || crc_error || first_code==DESCRIPTOR),
        .src_session(active_session),.src_seq(active_seq),.src_count(received),
        .sink_done(mem_seen),.sink_error(sink_error),.sink_session(active_session),.sink_seq(active_seq),
        .sink_count(sys_is_wav ? (media_good ? 32'd1 : 32'd0) : sink_count_q),
        .cancel(cancel_match),.cancel_session(active_session),.cancel_seq(active_seq),
        // file result 确认源文件流结束，mem event 确认两路已接受的 SDRAM 写段均已排空。
        .abort_ack(phase==DRAIN && source_seen && mem_seen && !file_busy),
        .ack_session(active_session),.ack_seq(active_seq),
        .ready_valid(tx_ready_valid),.ready_accept(commit_valid && commit_ready),
        .busy(tx_busy),.abort_req(tx_abort),.active_session(active_session),.active_seq(active_seq),
        .active_slot(active_slot),.error_reason(tx_reason));
    cdc_mailbox #(.DATA_WIDTH(CMD_W)) u_command(.arst(arst),.src_clk(clk),
        .src_valid(req_fire || (audio_request && audio_req_ready)),.src_ready(cmd_ready),.src_data(cmd_input),
        .src_done(cmd_done),.src_active(),.src_busy(),.dst_clk(mem_clk),
        .dst_valid(cmd_valid),.dst_ready(cmd_take),.dst_data(cmd_bus),.dst_active());
    cdc_mailbox #(.DATA_WIDTH(2*KEY_W)) u_abort(.arst(arst),.src_clk(clk),
        .src_valid(abort_send),
        .src_ready(abort_ready),.src_data({active_session,active_seq}),.src_done(),.src_active(),.src_busy(),
        .dst_clk(mem_clk),.dst_valid(abort_valid),.dst_ready(1'b1),.dst_data(abort_bus),.dst_active());
    wire event_send;wire [EVENT_W-1:0] event_data;
    cdc_mailbox #(.DATA_WIDTH(EVENT_W)) u_event(.arst(arst),.src_clk(mem_clk),
        .src_valid(event_send),.src_ready(event_src_ready),.src_data(event_data),
        .src_done(),.src_active(),.src_busy(),.dst_clk(clk),.dst_valid(event_valid),
        .dst_ready(1'b1),.dst_data(event_bus),.dst_active());
    // 所有关键结果在本域锁存，事件不同拍也不会丢失；同沿取消优先屏蔽commit。
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            phase<=IDLE;first_code<=0;length<=0;expected_crc<=0;check_crc<=0;slot_gen<=0;
            source_seen<=0;source_good<=0;mem_seen<=0;cmd_delivered<=0;abort_sent<=0;cancel_sent<=0;
            received_count_q<=0;sink_count_q<=0;media_good_q<=0;crc_good_q<=0;file_written_q<=0;file_thumb_q<=0;file_crc_q<=0;
            drain_age<=0;quarantine<=0;valid_slots<=0;
        end else begin
            if(cmd_done) cmd_delivered<=1;
            if(file_cancel_valid && file_cancel_ready) cancel_sent<=1;
            if(abort_send && abort_ready) abort_sent<=1;
            if(source_fire) begin source_seen<=1;source_good<=file_result_code==0 && file_result_count==length;end
            if(event_fire) begin
                file_written_q<=event_bus[49:31];file_thumb_q<=event_bus[30:17];file_crc_q<=event_bus[81:50];
                mem_seen<=1;received_count_q<=event_received;
                sink_count_q<={13'd0,event_bus[49:31]}+{18'd0,event_bus[30:17]};
                media_good_q<=media_good_live;crc_good_q<=crc_good_live;
            end
            case(phase)
                IDLE:if(req_fire) begin
                    phase<=RUN;length<=req_length;expected_crc<=req_crc;check_crc<=req_crc_enable;slot_gen<=req_slot_gen;
                    source_seen<=0;source_good<=0;mem_seen<=0;cmd_delivered<=0;abort_sent<=0;cancel_sent<=0;
                    first_code<=((req_is_wav ? (req_length<12 || req_length>32'hf0000) : (req_length<54 || req_length>((EXT_BMP_FORMATS && BMP32_EN)?32'h12d000:32'he2000))) || req_session==0 || req_seq==0 || req_slot_gen==0)?DESCRIPTOR:OK;
                    if(!req_is_wav) valid_slots[req_slot]<=0;drain_age<=0;
                end
                RUN,READY:begin
                    if(failure || tx_abort) begin
                        phase<=DRAIN;drain_age<=0;
                        if(first_code==OK) first_code<=cancel_match?CANCELED:(source_error?SOURCE:(crc_error?CRC:MEDIA));
                    end else if(tx_ready_valid) begin
                        phase<=READY;
                        if(commit_valid && commit_ready) begin if(!sys_is_wav) valid_slots[active_slot]<=1;phase<=RESULT;end
                    end
                end
                DRAIN:begin
                    if(!quarantine && !tx_busy && source_seen && mem_seen && !file_busy) phase<=RESULT;
                    else if(!quarantine && drain_age>=DRAIN_CYCLES-1) quarantine<=1;
                    else if(!quarantine) drain_age<=drain_age+1'b1;
                end
                RESULT:if(result_valid && result_ready && !quarantine) phase<=IDLE;
                default:phase<=DRAIN;
            endcase
        end
    end
    // mem域按extent键校验整条native字节口；metadata占序号，不占文件CRC/像素计数。
    reg m_stop;reg [KEY_W-1:0] m_session,m_seq,m_expected_seq; reg [31:0] m_length,m_count;
    // 与 m_length 同步锁存的末载荷下标，避免末字校验先穿32位减法。
    reg [31:0] m_last_index;
    // 当前计数的末字/范围条件同拍寄存；只在已接受的有效payload后前进。
    // 范围在最后一个合法字节后清零，后续多字节仍走原bad_now拒绝/排空。
    reg [31:0] m_penultimate_index;
    reg m_at_last,m_in_range;
    reg [7:0] stream_code;
    wire m_start_ready,m_byte_ready,m_result,m_ok;
    wire m_start_ready_na;   // u_media 的 abort=0 余因子 start_ready（见下方 cmd_take）
    // 媒体在“本层送入的 abort=0”时的 ready（m_byte_ready==m_byte_ready_na&&!(m_active&&stop_now)）。
    wire m_byte_ready_na;wire [7:0] m_detail;
    wire [18:0] m_written;wire [13:0] m_thumb;wire [31:0] m_crc;
    assign m_enabled=!mrst_logic;
    // cmd_take 仅在本地空闲且目标消费者可接受时成立。
    // 本地空闲已排除当前事务停止条件，无需把取消键比较再串入新命令准入路径。
    assign cmd_take=m_enabled && !m_active && (cmd_is_audio ? audio_mem_ready : m_start_ready_na);
    assign cmd_fire=cmd_valid && cmd_take && !cmd_is_audio;
    // 取消键匹配提前一拍计算，避免 session/seq 宽比较直接串入装载器的停止路径。
    // abort_hit_old 比较当前事务键，abort_hit_cmd 比较本拍命令携带的新键。
    // abort_cmd_fire_q 指示上一拍是否接受新命令，据此选择匹配结果，再与 m_active 合并。
    // 取消的实际生效晚一拍；在取消前已接受的写段仍须排空，最终响应受停止握手约束。
    // 取消确认完成后才能复用事务键，防止旧取消命中新作业。
    reg abort_hit_old,abort_hit_cmd,abort_cmd_fire_q;
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin abort_hit_old<=0;abort_hit_cmd<=0;abort_cmd_fire_q<=0;end
        else begin
            abort_hit_old<=abort_valid && abort_bus=={m_session,m_seq};
            abort_hit_cmd<=abort_valid && abort_bus=={cmd_bus[CMD_SES_LO+KEY_W-1:CMD_SES_LO],cmd_bus[CMD_SEQ_LO+KEY_W-1:CMD_SEQ_LO]};
            abort_cmd_fire_q<=cmd_fire;
        end
    end
    wire m_abort_msg=m_active && (abort_cmd_fire_q ? abort_hit_cmd : abort_hit_old);
    // 一字节入口寄存器。先从真实bridge接受并锁存数据/字段检查结果，
    // 下一拍才交给CRC/媒体。无直通、无同沿pop+push，避免新字段撞上旧计数/seq。
    // SPI字节间隔远大于两拍，但deadline仍逐拍计时；这里不延长任何预算。
    // m_active外仍直接排空metadata；payload必须等command建立本次所有权才能捕获。
    reg byte_pending;
    // 字节类型、事务键和边界条件随本地入口数据一起锁存。
    reg bad_pending_q,bmp_pending_q,wav_pending_q;
    reg [7:0] pending_data;
    reg [1:0] pending_user;
    reg pending_last,pending_key_bad,pending_last_bad;
    wire incoming_key_bad=out_session!=m_session || out_seq!=m_expected_seq || out_user==2'b10;
    wire incoming_full_last=out_user[1] && out_last;
    wire incoming_last_bad=out_user[0] && (incoming_full_last!=m_at_last || !m_in_range);
    wire full_last=pending_user[1] && pending_last;
    wire bad_now=m_active && bad_pending_q;
    wire stop_now=m_stop || m_abort_msg || dst_stop_req || bad_now;
    // 取消/flush仍同拍禁止入口接受；旧buffer丢弃后，必须等媒体实际在途段排空
    // 与event响应使m_active清零才能dst_stop_ack，不能把buffer空当作写完成。
    assign out_ready=m_enabled && (m_active ? (!m_stop && !m_abort_msg && !dst_stop_req &&
        !m_result && !byte_pending) : !out_user[0]);
    // pending_ready 排除停止、取消和目标停流；字节有效性与消费者就绪在本地合并。
    // 错误字节可直接进入排空路径，正常媒体字节须等待消费者接受。
    wire pending_ready=m_enabled && m_active && !m_result && !m_stop && !m_abort_msg && !dst_stop_req &&
        (bad_now || !pending_user[0] || m_byte_ready_na);
    wire pending_fire=byte_pending && pending_ready;
    wire payload_fire=pending_fire && pending_user[0] && !bad_now;
    wire byte_capture=m_active && out_valid && out_ready;
    // cancel/终结优先清除本地副本；未消费字节不进入CRC/计数，也不会泄露给下次请求。
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin
            byte_pending<=0;bad_pending_q<=0;bmp_pending_q<=0;wav_pending_q<=0;pending_data<=0;pending_user<=0;pending_last<=0;
            pending_key_bad<=0;pending_last_bad<=0;
        end else if(!m_active || m_stop || m_abort_msg || dst_stop_req || m_result) begin
            byte_pending<=0;bad_pending_q<=0;bmp_pending_q<=0;wav_pending_q<=0;
        end else begin
            if(pending_fire) begin byte_pending<=0;bad_pending_q<=0;bmp_pending_q<=0;wav_pending_q<=0;end
            if(byte_capture) begin
                byte_pending<=1;pending_data<=out_data;
                bad_pending_q<=incoming_key_bad || incoming_last_bad;
                bmp_pending_q<=out_user[0] && !mem_is_wav;wav_pending_q<=out_user[0] && mem_is_wav;pending_user<=out_user;pending_last<=out_last;
                pending_key_bad<=incoming_key_bad;pending_last_bad<=incoming_last_bad;
            end
        end
    end
    // 末字交接只记录边界；解析流水、待发写和已接受写都排空后才响应。
    reg [7:0] quantum_writes;
    reg quantum_pending;
    reg [KEY_W-1:0] quantum_ticket;
    wire quantum_boundary=pending_fire && pending_user[0] && pending_last && !bad_now;
    wire b_quantum_idle,w_quantum_idle;
    wire media_quantum_idle=mem_is_wav ? w_quantum_idle : b_quantum_idle;
    assign quantum_receipt_valid=quantum_pending && !byte_pending && media_quantum_idle &&
        quantum_writes==0 && !load_valid && !m_stop && !m_abort_msg && !dst_stop_req;
    assign quantum_receipt_ticket=quantum_ticket;
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin quantum_writes<=0;quantum_pending<=0;quantum_ticket<=0;end
        else begin
            case({load_valid && load_ready,load_done && load_done_was_write})
                2'b10:quantum_writes<=quantum_writes+1'b1;
                2'b01:quantum_writes<=quantum_writes-1'b1;
                default:quantum_writes<=quantum_writes;
            endcase
            if(cmd_fire) begin quantum_writes<=0;quantum_pending<=0;end
            else if(m_stop || m_abort_msg || dst_stop_req) quantum_pending<=0;
            else begin
                // 这里传文件任务键m_seq；链控制模块按同一约定匹配，不能改成字节extent键。
                if(quantum_boundary) begin quantum_pending<=1;quantum_ticket<=m_seq;end
                if(quantum_receipt_valid && quantum_receipt_ready) quantum_pending<=0;
            end
        end
    end
    assign dst_stop_ack=m_enabled && dst_stop_req && !m_active;
    crc32_ieee u_crc(.clk(mem_clk),.rst(mrst_logic),.clear(cmd_fire),.data_valid(payload_fire),.data(pending_data),.crc(m_crc));
    // BMP和WAV共用唯一文件来源和外部后台写口，命令握手后数据去向保持锁定，直至完成事件被接收。
    wire b_start_ready,b_start_ready_na,b_byte_ready,b_byte_ready_na,b_result,b_ok;
    wire [7:0] b_detail;wire [18:0] b_written;wire [13:0] b_thumb;
    wire b_load_valid,b_wr_valid;wire [20:0] b_load_addr;wire [3:0] b_load_count;
    wire [31:0] b_load_budget,b_wr_data;
    wire w_start_ready,w_start_ready_na,w_byte_ready,w_byte_ready_na,w_result,w_ok;
    wire [7:0] w_detail;wire [18:0] w_written;
    wire w_load_valid,w_wr_valid;wire [20:0] w_load_addr;wire [3:0] w_load_count;
    wire [31:0] w_load_budget,w_wr_data;
    assign m_start_ready=cmd_is_wav ? w_start_ready : b_start_ready;
    assign m_start_ready_na=cmd_is_wav ? w_start_ready_na : b_start_ready_na;
    assign m_byte_ready=mem_is_wav ? w_byte_ready : b_byte_ready;
    assign m_byte_ready_na=mem_is_wav ? w_byte_ready_na : b_byte_ready_na;
    assign m_result=mem_is_wav ? w_result : b_result;
    assign m_ok=mem_is_wav ? w_ok : b_ok;
    assign m_detail=mem_is_wav ? w_detail : b_detail;
    assign m_written=mem_is_wav ? w_written : b_written;
    assign m_thumb=mem_is_wav ? 14'd0 : b_thumb;
    assign load_valid=mem_is_wav ? w_load_valid : b_load_valid;
    assign load_addr=mem_is_wav ? w_load_addr : b_load_addr;
    assign load_count=mem_is_wav ? w_load_count : b_load_count;
    assign load_budget=mem_is_wav ? w_load_budget : b_load_budget;
    assign wr_data_valid=mem_is_wav ? w_wr_valid : b_wr_valid;
    assign wr_data=mem_is_wav ? w_wr_data : b_wr_data;
    bmp_thumbnail_sdram_loader #(.EXT_FORMATS(EXT_BMP_FORMATS),.ALLOW_32(BMP32_EN),.ENABLE_THUMBNAILS(ENABLE_THUMBNAILS),.EXTERNAL_MEMORY(EXTERNAL_MEMORY),.POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),.REFRESH_COUNT(REFRESH_COUNT),
        .REFRESH_INTERVAL(REFRESH_INTERVAL),.REFRESH_GRACE(REFRESH_GRACE)) u_media(
        .clk(mem_clk),.arst(arst),.start_valid(cmd_fire && !cmd_is_wav),.start_ready(b_start_ready),.start_ready_na(b_start_ready_na),
        .file_length(cmd_bus[65:34]),.budget_cycles(cmd_bus[33:2]),.slot(cmd_bus[1:0]),.abort(m_active && stop_now && !mem_is_wav),
        // byte_valid 只表示本地暂存中存在该事务的字节，取消由独立 abort 端口送入消费者。
        // abort 有效时消费者不产生实际字节握手，因而无需把宽取消判据再串入 byte_valid 路径。
        .byte_valid(m_active && bmp_pending_q && !m_result),.byte_ready(b_byte_ready),.byte_ready_na(b_byte_ready_na),
        .byte_data(pending_data),.byte_last(full_last),.result_valid(b_result),.result_ready(!audio_response && event_src_ready && m_active && !mem_is_wav),
        .result_ok(b_ok),.result_detail(b_detail),.out_written_words(b_written),.out_thumbnail_words(b_thumb),
        .quantum_idle(b_quantum_idle),
        .load_valid(b_load_valid),.load_ready(load_ready && !mem_is_wav),.load_addr(b_load_addr),
        .load_count(b_load_count),.load_budget(b_load_budget),.load_beat(load_beat && !mem_is_wav),
        .load_done(load_done && !mem_is_wav),.load_done_was_write(load_done_was_write),
        .wr_data_valid(b_wr_valid),.wr_data(b_wr_data),.sd_cs_n(sd_cs_n),.sd_ras_n(sd_ras_n),
        .sd_cas_n(sd_cas_n),.sd_we_n(sd_we_n),.sd_cke(sd_cke),.sd_addr(sd_addr),.sd_ba(sd_ba));
    generate if(ENABLE_WAV) begin:g_wav
        if(!EXTERNAL_MEMORY) begin:invalid_wav_memory WAV_requires_shared_external_memory u_error();end
        wav_sdram_loader u_wav(.clk(mem_clk),.arst(arst),
            .start_valid(cmd_fire && cmd_is_wav),.start_ready(w_start_ready),.start_ready_na(w_start_ready_na),
            .file_length(cmd_bus[65:34]),.budget_cycles(cmd_bus[33:2]),.bank(cmd_bus[0]),
            .abort(m_active && stop_now && mem_is_wav),
            .byte_valid(m_active && wav_pending_q && !m_result),
            .byte_ready(w_byte_ready),.byte_ready_na(w_byte_ready_na),.byte_data(pending_data),.byte_last(full_last),
            .result_valid(w_result),.result_ready(!audio_response && event_src_ready && m_active && mem_is_wav),
            .result_ok(w_ok),.result_detail(w_detail),.out_written_words(w_written),
            .quantum_idle(w_quantum_idle),
            .load_valid(w_load_valid),.load_ready(load_ready && mem_is_wav),.load_addr(w_load_addr),
            .load_count(w_load_count),.load_budget(w_load_budget),.wr_data_valid(w_wr_valid),.wr_data(w_wr_data),
            .load_beat(load_beat && mem_is_wav),.load_done(load_done && mem_is_wav),.load_done_was_write(load_done_was_write));
    end else begin:g_no_wav
        assign {w_start_ready,w_start_ready_na,w_byte_ready,w_byte_ready_na,w_result,w_ok}=6'd0;
        assign w_detail=0;assign w_written=0;assign w_load_valid=0;assign w_wr_valid=0;
        assign w_quantum_idle=1'b1;
        assign w_load_addr=0;assign w_load_count=0;assign w_load_budget=0;assign w_wr_data=0;
    end endgenerate
    assign file_event_send=m_active && m_result;
    assign event_send=audio_response || file_event_send;
    generate if(ENABLE_AUDIO_CONTROL) begin:g_audio_event
        // 音频响应只覆盖低19位；上部文件字段在type=1时不由文件完成逻辑消费。
        // 文件结果快照仍仅在event_fire(type=0)锁存，CRC/计数不被音频事件改写。
        wire [EVT_KEY_LO+2*KEY_W-1:0] file_payload={m_session,m_seq,m_count,m_crc,m_written,m_thumb,m_ok,8'd0,stream_code};
        assign event_data={audio_response,file_payload[EVT_KEY_LO+2*KEY_W-1:19],
            audio_response ? {audio_mem_rsp_status,audio_mem_rsp_bank,audio_mem_rsp_token} : file_payload[18:0]};
    end else begin:g_file_event
        assign event_data={m_session,m_seq,m_count,m_crc,m_written,m_thumb,m_ok,8'd0,stream_code};
    end endgenerate
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin m_active<=0;m_stop<=0;m_session<=0;m_seq<=0;m_expected_seq<=0;m_length<=0;m_last_index<=32'hffffffff;m_penultimate_index<=32'hfffffffe;m_at_last<=0;m_in_range<=0;m_count<=0;stream_code<=0;end
        else begin
            if(cmd_fire) begin
                m_active<=1;m_stop<=0;m_session<=cmd_bus[CMD_SES_LO+KEY_W-1:CMD_SES_LO];m_seq<=cmd_bus[CMD_SEQ_LO+KEY_W-1:CMD_SEQ_LO];
                m_expected_seq<=cmd_bus[CMD_SEQ_LO+KEY_W-1:CMD_SEQ_LO];m_length<=cmd_bus[65:34];m_last_index<=cmd_bus[65:34]-32'd1;m_penultimate_index<=cmd_bus[65:34]-32'd2;
                m_at_last<=cmd_bus[65:34]==32'd1;m_in_range<=cmd_bus[65:34]!=0;m_count<=0;stream_code<=0;
            end
            if(m_active && (m_abort_msg || dst_stop_req || bad_now)) begin m_stop<=1;if(bad_now && stream_code==0) stream_code<=1;end
            if(payload_fire) begin
                m_count<=m_count+32'd1;m_at_last<=m_count==m_penultimate_index;
                if(m_at_last) m_in_range<=0;
            end
            if(pending_fire && !bad_now && pending_last) begin
                m_expected_seq<=m_expected_seq+1'b1;
                // KEY_W=32 时 extent 序号用尽即停流（stream_code=2）；KEY_W<32（产品8位）
                // 与fat32_chain_master的internal_seq同样按2^KEY_W模回绕，最大编号后继续回绕：
                // 一个文件约0.9MB/4KB≈230个extent，加FAT读后会跨过255，必须允许回绕。
                // 一次只传输一个 extent；开始下一个 extent 前，上一段字节必须已处理完毕或被拒绝。
                if(KEY_W>=32 && m_expected_seq=={KEY_W{1'b1}} && !full_last) begin m_stop<=1;stream_code<=2;end
            end
            if(file_event_send && !audio_response && event_src_ready) m_active<=0;
        end
    end
endmodule
`default_nettype wire
