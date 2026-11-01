`timescale 1ns/1ps
`default_nettype none

// 一个状态机共用目录扫描和文件读取，底下只接一套真实extent通路。
// 目录按短文件名查项；文件沿FAT链读取，并裁掉最后扇区里的填充字节。
// 已经向下游发送字节后发生错误时，须等下游停止并处理完在途操作，才能返回空闲状态。
// CONTEXT_ENABLE启用时，只有在分块数据处理和写入全部完成后才保存游标并切换上下文。
module fat32_chain_master #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    // TIME_W 为内部预算、已用和剩余周期计数的位宽，默认 32 位；
    // 产品由 tf_bmp_boot 传入 counter_width(SYS_BUDGET+1)=26，该宽度按构造恰好容纳预算值本身。
    // 所有比较/减法都在 TIME_W 位内完成；截止判定仍是逐周期精确的，预算值不变。
    parameter integer KEY_W=32,
    parameter integer TIME_W=32,
    // FILE_LEN_W = 文件剩余/已送/簇内剩余字节计数位宽。默认 32 位；产品传24（小于 16MiB）。
    // file_size 高位非0时在 IDLE 显式拒绝为 FILE_LIMIT(detail=02)，不会截断后读一个较短的文件。
    parameter integer FILE_LEN_W=32,
    // 上下文模式默认关闭；启用后可在 512B 边界保存恢复点并分块返回。
    // 不是取消：已消费文件字节、FAT链位置和解析器状态由调用方保持，下一量接着读。
    parameter integer CHUNK_ENABLE=0,AUTO_CONTINUE=0,
    // CONTEXT_ENABLE只在YIELD边界增加保存/恢复/换槽；默认0不实例化记录RAM。
    parameter integer CONTEXT_ENABLE=0,EPOCH_W=12,USE_EG_BRAM=1
)(
    input wire clk,arst,
    input wire scan_geometry_valid,file_geometry_valid,
    input wire [31:0] fat_lba,data_lba,root_cluster,
    input wire [27:0] cluster_count,input wire [2:0] cluster_shift,

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
    input wire file_chunked,input wire [5:0] file_start_sector,
    input wire [12:0] file_chunk_limit,
    input wire [KEY_W-1:0] file_start_extent_seq,
    // AUTO_CONTINUE产品在此沿交接小量，不重新锁存整文件状态或复位extent序号。
    output wire file_yield_valid,input wire file_yield_ready,
    input wire file_cancel_valid,output wire file_cancel_ready,
    input wire [KEY_W-1:0] file_cancel_session,file_cancel_seq,

    output wire sink_abort_req,input wire sink_abort_ack,
    output wire [KEY_W-1:0] sink_abort_session,sink_abort_seq,
    input wire [KEY_W-1:0] sink_ack_session,sink_ack_seq,

    output wire file_busy,file_result_valid,input wire file_result_ready,
    output wire [KEY_W-1:0] file_result_session,file_result_seq, output wire [31:0] file_result_elapsed,
    output wire [31:0] file_result_count,
    output wire [3:0] file_result_code,output wire [7:0] file_result_detail,
    // 仅result有效且code=0时可使用；more=1才允许用cluster/sector/remaining继续。
    // checkpoint不含任何在途SPI/extent：跨簇边界先校验FAT并移到下一簇后才发布。
    output wire file_result_more,output wire [31:0] file_checkpoint_cluster,
    output wire [5:0] file_checkpoint_sector,output wire [31:0] file_checkpoint_remaining,
    output wire [KEY_W-1:0] file_checkpoint_extent_seq,

    output wire req_valid,input wire req_ready,
    output wire [KEY_W-1:0] req_session,req_seq, output wire [31:0] req_lba,req_budget_cycles,
    output wire [3:0] req_sectors,output wire [12:0] req_byte_count,
    output wire [1:0] req_user,
    output wire ext_cancel_valid,input wire ext_cancel_ready,
    output wire [KEY_W-1:0] ext_cancel_session,ext_cancel_seq,
    input wire ext_result_valid,output wire ext_result_ready,
    input wire [KEY_W-1:0] ext_result_session,ext_result_seq, input wire [31:0] ext_result_count,ext_result_elapsed,
    input wire [3:0] ext_result_code,input wire [7:0] ext_result_detail,
    input wire tap_valid,input wire [7:0] tap_data,input wire [8:0] tap_index,
    input wire tap_sector_last,input wire [KEY_W-1:0] tap_session,tap_seq,

    // 共享链计算：导出当前簇/簇内偏移与FAT表项值，由services层单一实例计算。
    output wire [31:0] calc_cur_cluster,calc_fat_value,
    output wire [5:0] calc_cluster_sector_off,
    input wire [38:0] calc_cluster_lba,
    input wire [32:0] calc_fat_entry_lba,
    input wire calc_cluster_lba_overflow,calc_fat_entry_lba_overflow,
    input wire [27:0] calc_max_cluster,
    input wire calc_entry_eoc,calc_entry_bad_reserved,calc_entry_in_range,

    // 四上下文模式：所有信号与clk同域，完整量键包含起始extent ticket。
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
    output wire [31:0] event_count,event_quantum_count
);
    wire rst_logic;
    localparam IDLE=0,DATA_REQ=1,DATA_WAIT=2,FAT_REQ=3,FAT_WAIT=4,SINK_WAIT=5,RESULT=6,ADDR_STAGE=7,YIELD=8,
        CTX_LOAD_CMD=9,CTX_LOAD=10,CTX_LOAD_DONE=11,CTX_PREP=12,
        CTX_OWNER_SEND=13,CTX_OWNER_WAIT=14,CTX_RETIRE=15,
        CTX_SAVE_CMD=16,CTX_SAVE=17,CTX_SAVE_DONE=18,
        CTX_DROP_CMD=19,CTX_DROP_DONE=20,CTX_EVENT=21;
    // 目录和文件结果使用各自的状态码：目录侧LIMIT=6/LOOP=5/DUP=9，文件侧LIMIT=5/
    // DESCRIPTOR=9；DUP与DESCRIPTOR同值9且模式互斥，LIMIT按判定点区分常量。
    localparam OK=0,EXTENT=1,CHAIN=4,LOOP=5,NOT_FOUND=7,CANCELED=8,DUP=9,DESCRIPTOR=9,
               SCAN_LIMIT=6,FILE_LIMIT=5,DEADLINE=10,PROTOCOL=11;
    localparam integer STATE_W=CONTEXT_ENABLE!=0?5:AUTO_CONTINUE!=0?4:3;
    reg [STATE_W-1:0] state;
    // 每条extent请求增加一个地址准备拍，隔开簇减法/移位/加法和下游范围判定。
    // 此拍计入真实deadline，取消仍优先；尚未发布req_valid，不需要伪造后端完成。
    reg address_for_fat;
    reg [38:0] data_lba_cached;
    reg [32:0] fat_lba_cached;
    reg data_overflow_cached,fat_overflow_cached;
    reg [3:0] status;
    reg [7:0] detail;
    reg scan_mode;
    // session/seq认整份文件，internal_seq认当前extent，不能互相替换。
    // 整机使用 8 位事务编号；上一段传输结束后才分配下一编号，避免回绕后的编号匹配到迟到数据。
    // 消费者的m_expected_seq也按同样位宽递增，恢复时必须延续它。
    reg [KEY_W-1:0] session,seq,internal_seq; reg [TIME_W-1:0] budget,elapsed;
    reg [31:0] cur_cluster; reg [FILE_LEN_W-1:0] bytes_remaining,total_delivered,cluster_bytes_left;
    // 缩窄计数器前先保存原始长度的超限条件，不能等高位丢了再检查。
    reg ctx_size_too_long;
    wire [31:0] first_size=CONTEXT_ENABLE!=0 ? bytes_remaining : file_size;
    wire file_too_long=CONTEXT_ENABLE!=0 ? ctx_size_too_long :
        ((FILE_LEN_W<32) && ((file_size>>FILE_LEN_W)!=0));
    reg [6:0] sectors_left;
    reg [5:0] sector_off;
    reg payload_started;
    // 目录模式专有：项数上限按扇区累计、簇步数上限防环、目标名/类型。
    reg [5:0] sector_total;
    // 连续预计算扇区批次及本次传输后的累计扇区数，由现有ADDR_STAGE留出稳定拍。
    // 将簇内范围计算与 extent 接受、解析器使能分开。
    // 6位累计最多63、批次最多8，完整候选最多71，必须保留第7位，禁止截断成6位。
    reg [3:0] scan_batch_q;
    reg [6:0] scan_sector_sum_q;
    reg [8:0] cluster_steps;
    reg [87:0] target_name;
    reg want_dir;
    reg extent_done;

    // ctx_ticket沿用文件任务seq；ctx_quantum_base才是本量开始前的累计字节数。
    reg [1:0] ctx_slot,ctx_rr;
    reg [3:0] ctx_blocked;
    reg [EPOCH_W-1:0] ctx_epoch_q;
    reg [KEY_W-1:0] ctx_ticket;
    reg [31:0] ctx_quantum_base;
    reg ctx_receipt_seen;
    wire [3:0] ctx_leased=ctx_valid & ~ctx_blocked;
    wire [3:0] ctx_retire=(ctx_leased & (cancel_mask|deadline_mask)) |
        (record_valid & ~ctx_valid) | (quiesce ? (record_valid|ctx_leased) : 4'b0);
    wire [3:0] ctx_eligible=ctx_retire |
        ((ctx_service_enabled && !quiesce) ? (ctx_leased & demand & credit) : 4'b0);
    // 先挑优先级最高的一组，再从上次结束位置往后轮询，避免同级总抢同一槽。
    reg [3:0] ctx_priority_set;
    reg [1:0] ctx_chosen;
    reg ctx_found;
    integer ctx_n,ctx_k;
    always @* begin
        if(|ctx_retire) ctx_priority_set=ctx_retire;
        else if(|(ctx_eligible&critical_mask)) ctx_priority_set=ctx_eligible&critical_mask;
        else if(|(ctx_eligible&low_mask)) ctx_priority_set=ctx_eligible&low_mask;
        else if(|(ctx_eligible&necessary_mask)) ctx_priority_set=ctx_eligible&necessary_mask;
        else if(|(ctx_eligible&~background_mask)) ctx_priority_set=ctx_eligible&~background_mask;
        else ctx_priority_set=ctx_eligible;
        ctx_chosen=ctx_rr;ctx_found=0;
        for(ctx_n=0;ctx_n<4;ctx_n=ctx_n+1) begin
            ctx_k=(ctx_rr+ctx_n)&3;
            if(!ctx_found && ctx_priority_set[ctx_k]) begin ctx_chosen=ctx_k;ctx_found=1;end
        end
    end
    wire [KEY_W-1:0] ctx_client_session=ctx_session[ctx_slot*KEY_W+:KEY_W];
    wire [KEY_W-1:0] ctx_client_seq=ctx_seq[ctx_slot*KEY_W+:KEY_W];
    wire [EPOCH_W-1:0] ctx_client_epoch=ctx_epoch[ctx_slot*EPOCH_W+:EPOCH_W];
    wire ctx_identity_bad=session!=ctx_client_session || seq!=ctx_client_seq ||
        ctx_epoch_q!=ctx_client_epoch || ctx_epoch_q!=media_epoch;
    wire ctx_abort=quiesce || cancel_mask[ctx_slot] || deadline_mask[ctx_slot] || !ctx_valid[ctx_slot];
    wire [3:0] ctx_forced_code=ctx_identity_bad && !quiesce ? PROTOCOL :
        (deadline_mask[ctx_slot] ? DEADLINE : CANCELED);
    wire ctx_installed_match={installed_ctx,installed_session,installed_seq,installed_epoch,installed_ticket}==
        {ctx_slot,session,seq,ctx_epoch_q,ctx_ticket};
    wire ctx_receipt_match={receipt_ctx,receipt_session,receipt_seq,receipt_epoch,receipt_ticket}==
        {ctx_slot,session,seq,ctx_epoch_q,ctx_ticket};
    wire ctx_sink_key_match={sink_ack_ctx,sink_ack_epoch,sink_ack_ticket}==
        {ctx_slot,ctx_epoch_q,ctx_ticket};
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    wire address_phase=state==ADDR_STAGE;
    wire request_phase=state==DATA_REQ || state==FAT_REQ;
    wire wait_phase=state==DATA_WAIT || state==FAT_WAIT;
    wire data_phase=state==DATA_REQ || state==DATA_WAIT;
    wire active=state!=IDLE && state!=RESULT;
    // 取消按当前模式的完整键匹配；两种事务的键空间由上层分配，互不串扰。
    // 上下文模式的目录扫描仍由本状态机执行，取消源、键和状态码按 scan_mode 选择。
    // 目录扫描不持有文件上下文，不能借用文件槽的取消条件。
    wire [KEY_W-1:0] eff_cancel_session=(CONTEXT_ENABLE!=0 && !scan_mode) ? session : scan_mode ? scan_cancel_session : file_cancel_session;
    wire [KEY_W-1:0] eff_cancel_seq=(CONTEXT_ENABLE!=0 && !scan_mode) ? seq : scan_mode ? scan_cancel_seq : file_cancel_seq;
    wire eff_cancel_valid=(CONTEXT_ENABLE!=0 && !scan_mode) ? ctx_abort : scan_mode ? scan_cancel_valid : file_cancel_valid;
    wire cancel_match=active && eff_cancel_valid &&
                      eff_cancel_session==session && eff_cancel_seq==seq;
    wire cancel_now=enabled && cancel_match && status==OK;
    wire [3:0] cancel_status=(CONTEXT_ENABLE!=0 && !scan_mode) ? ctx_forced_code : CANCELED;
    // YIELD 等待消费者排空时仍消耗父文件预算，截止条件在本地单独计算。
    // 传输状态分别使用地址、请求和等待条件，避免 YIELD 译码进入高扇出的传输控制路径。
    wire deadline_timer_hit=budget==0 || elapsed==budget-1'b1;
    wire deadline_transfer_now=enabled && (address_phase || request_phase || wait_phase) &&
                               status==OK && deadline_timer_hit;
    wire yield_deadline_now=enabled && AUTO_CONTINUE!=0 && state==YIELD &&
                            status==OK && deadline_timer_hit;
    wire [TIME_W-1:0] remaining_cycles=budget>elapsed ? budget-elapsed : {TIME_W{1'b0}};
    wire [TIME_W-1:0] remaining=budget>elapsed ? budget-elapsed : {TIME_W{1'b0}};

    wire [27:0] max_cluster=calc_max_cluster;
    wire start_in_range=file_start_cluster[31:28]==0 && file_start_cluster>=32'd2 &&
                        file_start_cluster[27:0]<=max_cluster;
    wire scan_start_in_range=scan_start_cluster[31:28]==0 && scan_start_cluster>=32'd2 &&
                             {4'd0,scan_start_cluster[27:0]}<={4'd0,max_cluster};
    wire [32:0] cluster_bytes=33'd512<<cluster_shift;
    // 对非零长度，ceil(size/(512<<shift))<=cluster_count 等价于
    // floor((size-1)/512)>>shift < cluster_count，避免再做33位+1后范围判断。
    // 八个shift候选并行、常量右移，cluster_shift仅在末级选择比较结果，
    // 各项以固定移位计算，末级只选择比较结果。保留全部32位长度/28位簇数；
    // size=0仍无条件通过此范围检查（其descriptor分支在原位置另行判定）。
    // 长度判据与 IDLE 接受条件并行计算，取消及排空由独立状态处理。
    wire [31:0] last_file_byte=first_size-32'd1;
    wire [22:0] last_file_sector=last_file_byte[31:9];
    wire [7:0] expected_range_by_shift;
    genvar range_shift;
    generate for(range_shift=0;range_shift<8;range_shift=range_shift+1) begin:g_expected_range
        assign expected_range_by_shift[range_shift]=
            ({5'd0,last_file_sector} >> range_shift) < cluster_count;
    end endgenerate
    wire expected_in_range=first_size==0 || expected_range_by_shift[cluster_shift];
`ifdef FORMAL
    // 参考表达式保留完整 33 位范围计算，用于检查越界判据及第 33 位进位。
    wire [32:0] original_expected_clusters=first_size==0 ? 33'd0 :
        (({1'b0,first_size}-33'd1)>>(9+cluster_shift))+33'd1;
    always @* assert(expected_in_range==(original_expected_clusters<={5'd0,cluster_count}));
`endif
    wire chunk_start=CONTEXT_ENABLE!=0 || (CHUNK_ENABLE!=0 && file_chunked);
    wire [5:0] start_sector=CONTEXT_ENABLE!=0 ? sector_off : chunk_start ? file_start_sector : 6'd0;
    wire [31:0] first_room=cluster_bytes[31:0]-{17'd0,start_sector,9'd0};
    wire [31:0] first_cluster_bytes=first_size>first_room ? first_room : first_size;
    wire [32:0] first_sectors_wide=({1'b0,first_cluster_bytes}+33'd511)>>9;
    wire ctx_start_in_range=cur_cluster[31:28]==0 && cur_cluster>=32'd2 &&
                            cur_cluster[27:0]<=max_cluster;
    reg chunked_q;reg [12:0] chunk_left,chunk_limit_q;
    wire [3:0] ordinary_batch=sectors_left>7'd8 ? 4'd8 : sectors_left[3:0];
    wire [3:0] chunk_sectors=chunk_left[12:9];
    wire [3:0] batch=(CHUNK_ENABLE!=0 && chunked_q && chunk_sectors<ordinary_batch) ?
                      chunk_sectors : ordinary_batch;
    wire [12:0] batch_capacity={batch,9'd0};
    wire [12:0] batch_bytes=cluster_bytes_left>{19'd0,batch_capacity} ?
                            batch_capacity : cluster_bytes_left[12:0];
    wire final_payload_batch=bytes_remaining=={19'd0,batch_bytes};
    // 链LBA与FAT表项判定由共享单元完成；本模块只提供当前所有者输入。
    wire [38:0] data_lba_all=data_lba_cached;
    wire [32:0] fat_lba_all=fat_lba_cached;
    wire data_lba_overflow=data_overflow_cached;
    wire fat_lba_overflow=fat_overflow_cached;
    // 目录模式批次：每簇最多64扇区(7位)，extent一次最多8扇区且绝不跨簇。
    wire [6:0] spc=7'd1<<cluster_shift;
    wire [6:0] cluster_left=spc-{1'b0,sector_off};
    wire [3:0] scan_batch_live=cluster_left>7'd8 ? 4'd8 : cluster_left[3:0];
    wire [3:0] scan_batch=scan_batch_q;
    // limit仍取当前输入，允许调用者收紧范围；只缓存完整算术候选，不缓存最终范围检查结果。
    wire sector_would_overflow=scan_sector_sum_q>{1'b0,scan_sector_limit};
    wire step_would_overflow=cluster_steps==9'd511;

    wire m_start_ready,m_byte_ready,m_result_valid,m_result_pending,m_chain_end;
    wire m_code;
    wire [1:0] m_hits;
    wire [31:0] m_first_cluster,m_first_size;
    wire f_start_ready,f_byte_ready,f_result_valid,f_result_pending,f_code;
    wire [31:0] f_value;
    wire req_valid_now=enabled && request_phase &&
                       (scan_mode ? m_start_ready : 1'b1) &&
                       (scan_mode ? scan_geometry_valid : file_geometry_valid) && req_ready &&
                       !cancel_now && !deadline_transfer_now && remaining_cycles!=0 &&
                       !(data_phase ? data_lba_overflow : fat_lba_overflow) &&
                       !(scan_mode && data_phase && sector_would_overflow);
    wire req_fire=req_valid_now && req_ready;
    wire tap_match=tap_session==session && tap_seq==internal_seq;
    wire fat_tap_valid=tap_valid && state==FAT_WAIT && tap_match;
    wire m_gated_valid=tap_valid && state==DATA_WAIT && tap_match && scan_mode;
    wire tap_protocol=wait_phase && tap_valid &&
                      (!tap_match || ((scan_mode ? state==DATA_WAIT && !m_byte_ready :
                                                          state==FAT_WAIT && !f_byte_ready)));
    // tap_protocol只在本拍锁存status，禁止组合进入abort/取消：经子模块 enabled→byte_ready
    // 或经extent取消→p_ready→tap_valid都会回到tap_protocol自身形成环；下游停止推迟一拍。
    wire parse_abort=wait_phase && (status!=OK || cancel_now || deadline_transfer_now ||
                     (ext_result_valid && ext_result_code!=OK));
    // 结果寄存器保存已完成的 extent；接收前统一检查错误、取消及超时状态。
    // 将这些条件直接用于本层接收控制，避免经子模块 abort/ready 再反馈形成过长的组合路径。
    wire transfer_result_permit=status==OK && !cancel_now && !deadline_transfer_now &&
                       (!ext_result_valid || ext_result_code==OK);
    wire read_complete=(extent_done || ext_result_valid) && f_result_pending;
    wire m_read_complete=(extent_done || ext_result_valid) && m_result_pending;
    wire m_result_ready_int=state==DATA_WAIT && scan_mode && transfer_result_permit && m_read_complete;
    wire f_result_ready_int=state==FAT_WAIT && transfer_result_permit && read_complete;
    wire ext_match=ext_result_session==session && ext_result_seq==internal_seq;
    wire ext_ok=!ext_result_valid || (ext_match && ext_result_code==OK);
    wire [27:0] fat_next=f_value[27:0];
    wire eoc=calc_entry_eoc;
    wire bad_reserved=calc_entry_bad_reserved;
    wire next_in_range=calc_entry_in_range;
    wire sink_ack_match=sink_abort_ack && sink_ack_session==session && sink_ack_seq==seq &&
        (CONTEXT_ENABLE==0 || ctx_sink_key_match);
    // 目录模式链结束码：一次命中OK、未命中NOT_FOUND；必须显式4位。
    wire [3:0] chain_finish_status=m_hits==2'd1 ? OK : NOT_FOUND;
    // 错误收尾：文件数据已开始传输时先等待下游停止确认，目录模式直接收尾。
    // 状态宽度由 STATE_W 根据上下文和自动分块配置选择，错误收尾表达式须使用完整位宽。
    // 目录扫描没有上下文记录，错误须进入 RESULT 并由 scan_result 返回。
    // 文件服务则先保存恢复点，再经上下文事件端口发布分块或最终结果。
    wire [STATE_W-1:0] error_state_next=CONTEXT_ENABLE!=0 ?
        (scan_mode ? RESULT :
         ((payload_started || total_delivered!=0) ? SINK_WAIT : CTX_DROP_CMD)) :
        (!scan_mode && payload_started) ? SINK_WAIT : RESULT;

    // IDLE自身就是一个完整命令描述符槽，可先锁存命令，再等待下游 extent 服务。
    // ready只声明本地实际存储容量，接受沿仍真实锁session/seq/预算/簇与长度，绝非固定READY。
    // 后续ADDR_STAGE与REQ等待继续逐拍计入原budget，真正extent请求仍须req_ready才接受。
    // 本地命令槽占用后须等事务结束才能接受下一命令，下游忙时最多预存一个命令。
    assign scan_ready=enabled && state==IDLE && scan_geometry_valid;
    assign file_ready=enabled && CONTEXT_ENABLE==0 && state==IDLE && file_geometry_valid &&
                       !scan_valid;
    assign scan_cancel_ready=enabled && active && scan_mode && cancel_match && status==OK;
    assign file_cancel_ready=enabled && active && !scan_mode && cancel_match && status==OK;
    assign scan_busy=enabled && state!=IDLE;
    assign file_busy=(enabled && state!=IDLE) ||
        (CONTEXT_ENABLE!=0 && ((|record_valid) || (|ctx_leased)));
    assign scan_result_valid=enabled && state==RESULT && scan_mode;
    assign file_result_valid=enabled && CONTEXT_ENABLE==0 && state==RESULT && !scan_mode;
    assign scan_result_session=session;assign scan_result_seq=seq;
    assign scan_result_elapsed=elapsed;assign scan_result_code=status;
    assign scan_result_detail=detail;
    assign file_result_session=session;assign file_result_seq=seq;
    assign file_result_elapsed=elapsed;assign file_result_count=total_delivered;
    assign file_result_code=status;assign file_result_detail=detail;
    assign file_result_more=CHUNK_ENABLE!=0 && chunked_q && status==OK && bytes_remaining!=0;
    assign file_checkpoint_cluster=cur_cluster;
    assign file_checkpoint_sector=sector_off;
    assign file_checkpoint_remaining=bytes_remaining;
    assign file_checkpoint_extent_seq=internal_seq;
    assign file_yield_valid=enabled && CONTEXT_ENABLE==0 && AUTO_CONTINUE!=0 && state==YIELD;
    assign hit_cluster=m_first_cluster;assign hit_size=m_first_size;
    assign calc_cur_cluster=cur_cluster;
    assign calc_fat_value=f_value;
    assign calc_cluster_sector_off=sector_off;
    assign req_valid=req_valid_now;
    assign req_session=session;assign req_seq=internal_seq;
    assign req_lba=data_phase ? data_lba_all[31:0] : fat_lba_all[31:0];
    assign req_budget_cycles=remaining_cycles;
    assign req_sectors=data_phase ? (scan_mode ? scan_batch : batch) : 4'd1;
    assign req_byte_count=data_phase ? (scan_mode ? {scan_batch,9'd0} : batch_bytes) : 13'd512;
    assign req_user=data_phase && !scan_mode ? {final_payload_batch,1'b1} : 2'b00;
    // 取消只看已锁存status：tap_protocol当拍进取消会经extent p_ready→tap_valid闭环。
    assign ext_cancel_valid=wait_phase && (status!=OK || cancel_now || deadline_transfer_now);
    assign ext_cancel_session=session;assign ext_cancel_seq=internal_seq;
    assign ext_result_ready=wait_phase;
    assign sink_abort_req=enabled && state==SINK_WAIT && !scan_mode;
    assign sink_abort_session=session;assign sink_abort_seq=seq;
    assign sink_abort_ctx=CONTEXT_ENABLE!=0 ? ctx_slot : 2'd0;
    assign sink_abort_ticket=CONTEXT_ENABLE!=0 ? ctx_ticket : {KEY_W{1'b0}};
    assign sink_abort_epoch=CONTEXT_ENABLE!=0 ? ctx_epoch_q : {EPOCH_W{1'b0}};

    assign ctx_busy=CONTEXT_ENABLE!=0 && (state!=IDLE || (|record_valid) || (|ctx_leased));
    assign quiescent=CONTEXT_ENABLE!=0 && state==IDLE && !(|record_valid) && !(|ctx_leased) && !rst;
    assign ctx_accepted=(CONTEXT_ENABLE!=0 && state==CTX_OWNER_WAIT && installed_valid &&
        ctx_installed_match && !ctx_abort && !ctx_identity_bad) ? (4'b1<<ctx_slot) : 4'b0;
    assign owner_valid=CONTEXT_ENABLE!=0 && state==CTX_OWNER_SEND && !rst;
    assign owner_ctx=CONTEXT_ENABLE!=0 ? ctx_slot : 2'd0;
    assign owner_session=CONTEXT_ENABLE!=0 ? session : {KEY_W{1'b0}};
    assign owner_seq=CONTEXT_ENABLE!=0 ? seq : {KEY_W{1'b0}};
    assign owner_epoch=CONTEXT_ENABLE!=0 ? ctx_epoch_q : {EPOCH_W{1'b0}};
    assign owner_ticket=CONTEXT_ENABLE!=0 ? ctx_ticket : {KEY_W{1'b0}};
    assign receipt_ready=CONTEXT_ENABLE!=0 && !rst &&
        (state==ADDR_STAGE || request_phase || wait_phase || state==YIELD ||
         state==SINK_WAIT || state==CTX_RETIRE);
    assign event_valid=CONTEXT_ENABLE!=0 && state==CTX_EVENT && !rst;
    assign event_ctx=CONTEXT_ENABLE!=0 ? ctx_slot : 2'd0;
    assign event_session=CONTEXT_ENABLE!=0 ? session : {KEY_W{1'b0}};
    assign event_seq=CONTEXT_ENABLE!=0 ? seq : {KEY_W{1'b0}};
    assign event_epoch=CONTEXT_ENABLE!=0 ? ctx_epoch_q : {EPOCH_W{1'b0}};
    assign event_ticket=CONTEXT_ENABLE!=0 ? ctx_ticket : {KEY_W{1'b0}};
    assign event_code=CONTEXT_ENABLE!=0 ? status : 4'd0;
    assign event_detail=CONTEXT_ENABLE!=0 ? detail : 8'd0;
    assign event_more=CONTEXT_ENABLE!=0 && status==OK && bytes_remaining!=0;
    assign event_count=CONTEXT_ENABLE!=0 ? total_delivered : 32'd0;
    assign event_quantum_count=CONTEXT_ENABLE!=0 && status==OK ? total_delivered-ctx_quantum_base : 32'd0;

    wire ctx_ram_cmd_ready,ctx_ram_wr_ready,ctx_ram_rd_valid,ctx_ram_done;
    wire [4:0] ctx_ram_index;
    wire [31:0] ctx_ram_data;
    wire [1:0] ctx_ram_code;
    wire ctx_ram_cmd_valid=state==CTX_LOAD_CMD || state==CTX_SAVE_CMD || state==CTX_DROP_CMD;
    wire [1:0] ctx_ram_op=state==CTX_LOAD_CMD ? 2'd1 : state==CTX_DROP_CMD ? 2'd2 : 2'd0;
    reg [31:0] ctx_save_word;
    always @* begin
        case(ctx_ram_index)
            // 字6保存internal_seq（量/extent连续序号，每个量边界+1，恢复后必须接着涨），
            // 不能存ctx_ticket：ticket恒等于字1的文件任务seq，恢复时由seq回填；
            // 若把文件任务ticket装回internal_seq，第二个量的out_seq会回退，下游键校验即中止。
            0:ctx_save_word=session;1:ctx_save_word=seq;2:ctx_save_word=ctx_epoch_q;
            3:ctx_save_word=cur_cluster;4:ctx_save_word=bytes_remaining;
            5:ctx_save_word={26'd0,sector_off};6:ctx_save_word=internal_seq;
            7:ctx_save_word=total_delivered;default:ctx_save_word=0;
        endcase
    end
    generate if(CONTEXT_ENABLE!=0) begin:g_context_ram
        // 到达LOAD/SAVE/DROP前外部事务已经结束，本控制模块不会再并发第二个RAM命令。
        fat32_file_context_ram #(.USE_EG_BRAM(USE_EG_BRAM)) u_contexts(
            .clk(clk),.rst(rst),.cmd_valid(ctx_ram_cmd_valid && !rst),.cmd_ready(ctx_ram_cmd_ready),
            .cmd_op(ctx_ram_op),.cmd_slot(ctx_slot),.pinned_mask(4'b0),
            .abort(1'b0),.wr_valid(state==CTX_SAVE && !rst),.wr_ready(ctx_ram_wr_ready),.wr_data(ctx_save_word),
            .rd_valid(ctx_ram_rd_valid),.rd_ready(state==CTX_LOAD && !rst),.rd_data(ctx_ram_data),
            .word_index(ctx_ram_index),.rd_last(),.done_valid(ctx_ram_done),
            .done_ready((state==CTX_LOAD_DONE || state==CTX_SAVE_DONE || state==CTX_DROP_DONE) && !rst),
            .done_op(),.done_slot(),.done_code(ctx_ram_code),.valid_mask(record_valid),.busy());
    end else begin:g_no_context_ram
        assign ctx_ram_cmd_ready=0;assign ctx_ram_wr_ready=0;assign ctx_ram_rd_valid=0;
        assign ctx_ram_done=0;assign ctx_ram_index=0;assign ctx_ram_data=0;assign ctx_ram_code=0;
        assign record_valid=0;
    end endgenerate

    fat32_dirent_match u_match(
        .clk(clk),.arst(arst),.abort(parse_abort),
        .start_valid(scan_mode && state==DATA_REQ && req_fire),.start_ready(m_start_ready),
        .byte_valid(m_gated_valid),.byte_ready(m_byte_ready),
        .byte_data(tap_data),.byte_index(tap_index),.byte_last(tap_sector_last),
        .target_name(target_name),.want_dir(want_dir),
        .sector_count(scan_batch),
        .chain_reset(state==IDLE && scan_valid && scan_ready),
        .result_valid(m_result_valid),
        .result_pending(m_result_pending),
        .result_ready(m_result_ready_int),
        .result_code(m_code),
        .first_hit_valid(),.first_is_dir(),
        .first_hit_cluster(m_first_cluster),.first_hit_size(m_first_size),
        .hit_count(m_hits),.chain_terminated(m_chain_end)
    );
    fat32_fat_entry u_fat(
        .clk(clk),.arst(arst),.abort(parse_abort),
        .start_valid(state==FAT_REQ && req_fire),.start_ready(f_start_ready),
        .byte_valid(fat_tap_valid),.byte_ready(f_byte_ready),
        .byte_data(tap_data),.byte_index(tap_index),.byte_last(tap_sector_last),
        .query_cluster(cur_cluster),
        .result_valid(f_result_valid),.result_pending(f_result_pending),.result_ready(f_result_ready_int),
        .result_code(f_code),.entry_value(f_value)
    );

    // 挂载后的几何配置由services保持到目录/文件事务结束，与请求时保存的 LBA 范围配套。
    // sector_off/sector_total在离开WAIT的沿更新，随后ADDR_STAGE再给一拍，REQ看到新快照。
    // 连续寄存不附加deadline/取消组合CE；有效请求/目录等待期才使用batch，空闲数据无契约。
    // 范围准备阶段计入 elapsed，取消和排空仍优先处理。
    always @(posedge clk or posedge rst) begin
        if(rst) begin scan_batch_q<=0;scan_sector_sum_q<=0;end
        else begin
            scan_batch_q<=scan_batch_live;
            scan_sector_sum_q<={1'b0,sector_total}+{3'd0,scan_batch_live};
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;status<=OK;detail<=0;session<=0;seq<=0;budget<=0;elapsed<=0;
            internal_seq<=0;cur_cluster<=0;bytes_remaining<=0;total_delivered<=0;
            cluster_bytes_left<=0;sectors_left<=0;sector_off<=0;extent_done<=0;
            payload_started<=0;scan_mode<=0;sector_total<=0;cluster_steps<=0;
            chunked_q<=0;chunk_left<=0;chunk_limit_q<=0;
            target_name<=0;want_dir<=0;address_for_fat<=0;
            data_lba_cached<=0;fat_lba_cached<=0;data_overflow_cached<=0;fat_overflow_cached<=0;
            ctx_slot<=0;ctx_rr<=0;ctx_blocked<=0;ctx_epoch_q<=0;ctx_ticket<=0;
            ctx_quantum_base<=0;ctx_receipt_seen<=0;ctx_size_too_long<=0;
        end else begin
            ctx_blocked<=ctx_blocked & ctx_valid;
            if(CONTEXT_ENABLE!=0 && receipt_ready && receipt_valid && ctx_receipt_match) ctx_receipt_seen<=1;
            if(address_phase || request_phase || wait_phase ||
               (CONTEXT_ENABLE==0 && AUTO_CONTINUE!=0 && state==YIELD)) elapsed<=elapsed+1'b1;
            case(state)
                IDLE:begin
                    if(CONTEXT_ENABLE!=0 && !scan_valid) begin
                        if(ctx_found) begin
                            ctx_slot<=ctx_chosen;ctx_receipt_seen<=0;
                            // 已保存的恢复点只来自通过检查的文件，不能继承上一文件任务的失败位。
                            ctx_size_too_long<=!record_valid[ctx_chosen] && FILE_LEN_W<32 &&
                                ((ctx_size[ctx_chosen*32+:32]>>FILE_LEN_W)!=0);
                            scan_mode<=0;status<=OK;detail<=0;elapsed<=0;
                            budget<=ctx_budget[ctx_chosen*32+:32];
                            chunked_q<=1;chunk_left<=ctx_limit[ctx_chosen*13+:13];
                            chunk_limit_q<=ctx_limit[ctx_chosen*13+:13];
                            payload_started<=0;extent_done<=0;address_for_fat<=0;
                            if(record_valid[ctx_chosen]) state<=CTX_LOAD_CMD;
                            else begin
                                session<=ctx_session[ctx_chosen*KEY_W+:KEY_W];
                                seq<=ctx_seq[ctx_chosen*KEY_W+:KEY_W];
                                internal_seq<=ctx_seq[ctx_chosen*KEY_W+:KEY_W];
                                ctx_ticket<=ctx_seq[ctx_chosen*KEY_W+:KEY_W];
                                ctx_epoch_q<=ctx_epoch[ctx_chosen*EPOCH_W+:EPOCH_W];
                                cur_cluster<=ctx_cluster[ctx_chosen*32+:32];
                                bytes_remaining<=ctx_size[ctx_chosen*32+:32];
                                sector_off<=0;total_delivered<=0;ctx_quantum_base<=0;
                                state<=CTX_PREP;
                            end
                        end
                    end else if(scan_valid && scan_ready) begin
                        scan_mode<=1;
                        chunked_q<=0;
                        session<=scan_session;seq<=scan_seq;internal_seq<=scan_seq;
                        budget<=scan_budget_cycles[TIME_W-1:0];elapsed<=0;status<=OK;detail<=0;
                        target_name<=scan_target_name;want_dir<=scan_want_dir;
                        cur_cluster<=scan_start_cluster;sector_off<=0;
                        sector_total<=0;cluster_steps<=0;extent_done<=0;
                        if(scan_budget_cycles==0) begin status<=DEADLINE;state<=RESULT;end
                        else if(!scan_start_in_range) begin status<=CHAIN;detail<=8'h01;state<=RESULT;end
                        else begin state<=ADDR_STAGE;address_for_fat<=0;end
                    end else if(file_valid && file_ready) begin
                        scan_mode<=0;
                        session<=file_session;seq<=file_seq;
                        // 文件事务键不变，extent序号跨小量连续；下游提交器也校验这条序列。
                        internal_seq<=chunk_start ? file_start_extent_seq : file_seq;
                        budget<=file_budget_cycles[TIME_W-1:0];elapsed<=0;status<=OK;detail<=0;
                        cur_cluster<=file_start_cluster;bytes_remaining<=file_size;
                        total_delivered<=0;sector_off<=start_sector;extent_done<=0;payload_started<=0;
                        chunked_q<=chunk_start;chunk_left<=chunk_start ? file_chunk_limit : 13'd0;
                        chunk_limit_q<=chunk_start ? file_chunk_limit : 13'd0;
                        cluster_bytes_left<=first_cluster_bytes;
                        sectors_left<=first_sectors_wide[6:0];
                        if(file_budget_cycles==0) begin status<=DEADLINE;state<=RESULT;end
                        else if(chunk_start && (file_chunk_limit<13'd512 || file_chunk_limit>13'd4096 ||
                                file_chunk_limit[8:0]!=0 || {1'b0,start_sector}>=(7'd1<<cluster_shift))) begin
                            status<=DESCRIPTOR;detail<=8'h20;state<=RESULT;end
                        else if(file_size==0) begin
                            if(file_start_cluster==0) state<=RESULT;
                            else begin status<=DESCRIPTOR;detail<=8'h01;state<=RESULT;end
                        end else if(!start_in_range) begin status<=CHAIN;detail<=8'h01;state<=RESULT;end
                        else if(!expected_in_range) begin status<=FILE_LIMIT;detail<=8'h01;state<=RESULT;end
                        else if(file_too_long) begin status<=FILE_LIMIT;detail<=8'h02;state<=RESULT;end // 达到或超过 16MiB 时显式拒绝。
                        else begin state<=ADDR_STAGE;address_for_fat<=0;end
                    end
                end
                ADDR_STAGE:begin
                    // 配置在挂载事务完成后由services持有；当前簇/偏移在此拍已稳定。
                    // 缓存完整39/33位及溢出标志，绝不以截断消除越界检查。
                    if(cancel_now) begin status<=cancel_status;detail<=address_for_fat?FAT_REQ:DATA_REQ;
                        state<=error_state_next;end
                    else if(deadline_transfer_now) begin status<=DEADLINE;detail<=address_for_fat?FAT_REQ:DATA_REQ;
                        state<=error_state_next;end
                    else begin
                        data_lba_cached<=calc_cluster_lba;fat_lba_cached<=calc_fat_entry_lba;
                        data_overflow_cached<=calc_cluster_lba_overflow;
                        fat_overflow_cached<=calc_fat_entry_lba_overflow;
                        state<=address_for_fat?FAT_REQ:DATA_REQ;
                    end
                end
                DATA_REQ,FAT_REQ:begin
                    if(cancel_now) begin status<=cancel_status;detail<=state;
                        state<=error_state_next;end
                    else if(deadline_transfer_now) begin status<=DEADLINE;detail<=state;
                        state<=error_state_next;end
                    else if(scan_mode && data_phase && sector_would_overflow) begin
                        status<=SCAN_LIMIT;detail<=8'h01;state<=RESULT;end
                    else if((data_phase && data_lba_overflow) || (!data_phase && fat_lba_overflow)) begin
                        status<=CHAIN;detail<=8'h02;state<=error_state_next;end
                    else if(state==FAT_REQ && !f_start_ready) begin
                        status<=PROTOCOL;detail<=8'h01;state<=error_state_next;end
                    else if(req_fire) begin
                        extent_done<=0;
                        if(state==DATA_REQ) begin
                            if(scan_mode) sector_total<=sector_total+{2'd0,scan_batch};
                            else payload_started<=1;
                        end
                        state<=state==DATA_REQ ? DATA_WAIT : FAT_WAIT;
                    end
                end
                DATA_WAIT:begin
                    if(cancel_now) begin status<=cancel_status;detail<=state;end
                    else if(deadline_transfer_now) begin status<=DEADLINE;detail<=state;end
                    else if(tap_protocol) begin status<=PROTOCOL;detail<=8'h02;end
                    if(ext_result_valid && ext_result_ready) begin
                        extent_done<=1;
                        if(scan_mode) begin
                            // 目录模式：取消/期限已先置status，ext错误码不覆盖；
                            // 键不匹配与错误码都立刻标记，等待 extent 在途传输结束后收尾。
                            if(ext_result_session!=session || ext_result_seq!=internal_seq) begin
                                status<=PROTOCOL;detail<=8'h02;end
                            else if(ext_result_code!=OK && status==OK) begin
                                status<=EXTENT;detail<={4'd0,ext_result_code};end
                        end else begin
                            if(cancel_now || status!=OK) state<=SINK_WAIT;
                            else if(deadline_transfer_now) begin status<=DEADLINE;detail<=state;state<=SINK_WAIT;end
                            else if(tap_protocol) begin status<=PROTOCOL;detail<=8'h02;state<=SINK_WAIT;end
                            else if(!ext_match) begin status<=PROTOCOL;detail<=8'h03;state<=SINK_WAIT;end
                            else if(ext_result_code!=OK) begin status<=EXTENT;detail<={4'd0,ext_result_code};state<=SINK_WAIT;end
                            else if(ext_result_count!={19'd0,batch_bytes}) begin
                                status<=PROTOCOL;detail<=8'h04;state<=SINK_WAIT;end
                            else begin
                                bytes_remaining<=bytes_remaining-{19'd0,batch_bytes};
                                total_delivered<=total_delivered+{19'd0,batch_bytes};
                                cluster_bytes_left<=cluster_bytes_left-{19'd0,batch_bytes};
                                if(CHUNK_ENABLE!=0 && chunked_q) chunk_left<=chunk_left-batch_bytes;
                                if(sectors_left>{3'd0,batch}) begin
                                    sectors_left<=sectors_left-{3'd0,batch};
                                    sector_off<=sector_off+{2'd0,batch};
                                    internal_seq<=internal_seq+1'b1;
                                    // 必须在结果已消费、字节计数匹配后停；不能在req接受时预报成功。
                                    if(CHUNK_ENABLE!=0 && chunked_q && chunk_left==batch_bytes &&
                                       bytes_remaining>{19'd0,batch_bytes}) state<=(AUTO_CONTINUE!=0 || CONTEXT_ENABLE!=0)?YIELD:RESULT;
                                    else begin state<=ADDR_STAGE;address_for_fat<=0;end
                                end else begin
                                    sectors_left<=0;internal_seq<=internal_seq+1'b1;begin state<=ADDR_STAGE;address_for_fat<=1;end
                                end
                            end
                        end
                    end
                    if(scan_mode) begin
                        if(status!=OK) begin
                            if(extent_done) state<=RESULT;
                        end else if(m_read_complete && transfer_result_permit) begin
                            if(m_code) begin status<=PROTOCOL;detail<=8'h03;state<=RESULT;end
                            else if(m_hits==2'd2) begin
                                status<=DUP;detail<=8'h01;state<=RESULT;end
                            else if(m_chain_end) begin
                                status<=chain_finish_status;detail<=8'h02;state<=RESULT;end
                            else if({1'b0,sector_off}+{3'd0,scan_batch}<spc) begin
                                sector_off<=sector_off+{2'd0,scan_batch};
                                extent_done<=0;internal_seq<=internal_seq+1'b1;
                                begin state<=ADDR_STAGE;address_for_fat<=0;end
                            end else begin
                                extent_done<=0;internal_seq<=internal_seq+1'b1;
                                begin state<=ADDR_STAGE;address_for_fat<=1;end
                            end
                        end
                    end
                end
                FAT_WAIT:begin
                    if(cancel_now) begin status<=cancel_status;detail<=state;end
                    else if(deadline_transfer_now) begin status<=DEADLINE;detail<=state;end
                    else if(tap_protocol) begin status<=PROTOCOL;detail<=8'h05;end
                    if(ext_result_valid && ext_result_ready) begin
                        extent_done<=1;
                        // 取消、期限及协议错误优先于同沿返回的 extent 结果；
                        // 后者不得覆盖本周期已经确定的整文件事务状态。
                        if(!cancel_now && !deadline_transfer_now && !tap_protocol && status==OK) begin
                            if(!ext_match) begin status<=PROTOCOL;detail<=8'h06;end
                            else if(ext_result_code!=OK) begin status<=EXTENT;detail<={4'd0,ext_result_code};end
                        end
                    end
                    if(status!=OK || cancel_now || deadline_transfer_now || tap_protocol) begin
                        if(extent_done || (ext_result_valid && ext_result_ready))
                            state<=error_state_next;
                    end else if(read_complete && ext_ok && !cancel_now && !deadline_transfer_now) begin
                        if(f_code) begin status<=PROTOCOL;detail<=8'h07;state<=error_state_next;end
                        else if(scan_mode) begin
                            // 目录链语义：坏簇/EOC/范围按目录遍历判定，步数上限防环。
                            if(bad_reserved) begin status<=CHAIN;detail<=8'h03;state<=RESULT;end
                            else if(eoc) begin
                                status<=chain_finish_status;detail<=8'h04;state<=RESULT;end
                            else if(!next_in_range) begin status<=CHAIN;detail<=8'h05;state<=RESULT;end
                            else if(step_would_overflow) begin
                                status<=LOOP;detail<=8'h01;state<=RESULT;end
                            else begin
                                cur_cluster<={4'd0,fat_next};sector_off<=0;
                                cluster_steps<=cluster_steps+9'd1;
                                extent_done<=0;internal_seq<=internal_seq+1'b1;begin state<=ADDR_STAGE;address_for_fat<=0;end
                            end
                        end else begin
                            if(bad_reserved) begin status<=CHAIN;detail<=8'h03;state<=SINK_WAIT;end
                            else if(bytes_remaining==0) begin
                                if(eoc) state<=CONTEXT_ENABLE!=0 ? CTX_RETIRE : RESULT;
                                else begin status<=CHAIN;detail<=8'h04;state<=SINK_WAIT;end
                            end else if(eoc) begin status<=CHAIN;detail<=8'h05;state<=SINK_WAIT;end
                            else if(!next_in_range) begin status<=CHAIN;detail<=8'h06;state<=SINK_WAIT;end
                            else begin
                                cur_cluster<={4'd0,fat_next};sector_off<=0;
                                cluster_bytes_left<=bytes_remaining>{31'd0,cluster_bytes} ?
                                                    cluster_bytes[31:0] : bytes_remaining;
                                sectors_left<=((bytes_remaining>{31'd0,cluster_bytes} ?
                                              cluster_bytes : {1'b0,bytes_remaining})+33'd511)>>9;
                                extent_done<=0;internal_seq<=internal_seq+1'b1;
                                // 量尾恰在簇尾：此时FAT已真实返回合法next，恢复点才能跨过去。
                                if(CHUNK_ENABLE!=0 && chunked_q && chunk_left==0) state<=(AUTO_CONTINUE!=0 || CONTEXT_ENABLE!=0)?YIELD:RESULT;
                                else begin state<=ADDR_STAGE;address_for_fat<=0;end
                            end
                        end
                    end
                end
                YIELD:begin
                    // 上下文模式在此等待消费者确认本块处理完成；AUTO 模式继续按事务预算和交接状态推进。
                    if(cancel_now)begin status<=cancel_status;detail<=8'h21;state<=error_state_next;end
                    else if(CONTEXT_ENABLE==0 && yield_deadline_now)begin status<=DEADLINE;detail<=8'h21;state<=error_state_next;end
                    else if(CONTEXT_ENABLE!=0) begin
                        if(ctx_receipt_seen || (receipt_valid && receipt_ready && ctx_receipt_match))
                            state<=CTX_SAVE_CMD;
                    end else if(file_yield_valid && file_yield_ready)begin
                        chunk_left<=chunk_limit_q;extent_done<=0;address_for_fat<=0;state<=ADDR_STAGE;
                    end
                end
                SINK_WAIT:if(sink_ack_match) begin
                    if(CONTEXT_ENABLE!=0) begin total_delivered<=ctx_quantum_base;state<=CTX_DROP_CMD;end
                    else state<=RESULT;
                end
                RESULT:if((scan_mode ? scan_result_valid && scan_result_ready :
                                    file_result_valid && file_result_ready)) state<=IDLE;

                CTX_LOAD_CMD:if(ctx_ram_cmd_valid && ctx_ram_cmd_ready) state<=CTX_LOAD;
                // RAM的读有效到来才接记录；顺序和下面的保存字段必须一一对应。
                CTX_LOAD:if(ctx_ram_rd_valid) begin
                    case(ctx_ram_index)
                        0:session<=ctx_ram_data;1:seq<=ctx_ram_data;2:ctx_epoch_q<=ctx_ram_data;
                        3:cur_cluster<=ctx_ram_data;4:bytes_remaining<=ctx_ram_data;
                        5:sector_off<=ctx_ram_data[5:0];
                        // 字6是保存的internal_seq；文件任务ticket在字1已装入seq，这里直接回填，
                        // 保持ticket==文件任务seq的不变量，分块完成确认和换槽匹配才不依赖被改写的序号。
                        6:begin internal_seq<=ctx_ram_data;ctx_ticket<=seq;end
                        7:begin total_delivered<=ctx_ram_data;ctx_quantum_base<=ctx_ram_data;end
                    endcase
                    if(ctx_ram_index==31) state<=CTX_LOAD_DONE;
                end
                CTX_LOAD_DONE:if(ctx_ram_done) begin
                    if(ctx_ram_code!=0) begin status<=PROTOCOL;state<=CTX_DROP_CMD;end
                    else state<=CTX_PREP;
                end
                CTX_PREP:begin
                    cluster_bytes_left<=first_cluster_bytes;
                    sectors_left<=first_sectors_wide[6:0];
                    chunk_left<=ctx_limit[ctx_slot*13+:13];
                    chunk_limit_q<=ctx_limit[ctx_slot*13+:13];
                    state<=CTX_OWNER_SEND;
                end
                CTX_OWNER_SEND:if(owner_valid && owner_ready) state<=CTX_OWNER_WAIT;
                CTX_OWNER_WAIT:if(installed_valid && ctx_installed_match) begin
                    if(ctx_identity_bad || ctx_abort) begin
                        status<=ctx_forced_code;state<=error_state_next;
                    end else if(budget==0) begin status<=DEADLINE;state<=error_state_next;
                    end else if(!ctx_start_in_range) begin
                        status<=CHAIN;detail<=8'h01;state<=error_state_next;
                    end else if(!expected_in_range) begin
                        status<=FILE_LIMIT;detail<=8'h01;state<=error_state_next;
                    end else if(file_too_long) begin
                        status<=FILE_LIMIT;detail<=8'h02;state<=error_state_next;
                    end else if(chunk_limit_q<13'd512 || chunk_limit_q>13'd4096 ||
                                chunk_limit_q[8:0]!=0 || {1'b0,sector_off}>=(7'd1<<cluster_shift)) begin
                        status<=DESCRIPTOR;detail<=8'h20;state<=error_state_next;
                    end else if(bytes_remaining==0) begin
                        if(cur_cluster==0) state<=CTX_DROP_CMD;
                        else begin status<=DESCRIPTOR;detail<=8'h01;state<=error_state_next;end
                    end else begin state<=ADDR_STAGE;address_for_fat<=0;end
                end
                CTX_RETIRE:begin
                    if(cancel_now) begin status<=cancel_status;state<=error_state_next;end
                    else if(ctx_receipt_seen || (receipt_valid && receipt_ready && ctx_receipt_match))
                        state<=CTX_DROP_CMD;
                end
                CTX_SAVE_CMD:if(ctx_ram_cmd_valid && ctx_ram_cmd_ready) state<=CTX_SAVE;
                CTX_SAVE:if(ctx_ram_wr_ready && ctx_ram_index==31) state<=CTX_SAVE_DONE;
                CTX_SAVE_DONE:if(ctx_ram_done) begin
                    if(ctx_ram_code==0) state<=CTX_EVENT;
                    else begin status<=PROTOCOL;state<=error_state_next;end
                end
                CTX_DROP_CMD:if(ctx_ram_cmd_valid && ctx_ram_cmd_ready) state<=CTX_DROP_DONE;
                CTX_DROP_DONE:if(ctx_ram_done) begin
                    if(ctx_ram_code!=0) status<=PROTOCOL;
                    state<=CTX_EVENT;
                end
                // 对方接走事件后才换槽；最终事件还会挡住未重新拉低valid的旧文件任务。
                CTX_EVENT:if(event_valid && event_ready) begin
                    if(!event_more) ctx_blocked[ctx_slot]<=1;
                    ctx_rr<=ctx_slot+1'b1;state<=IDLE;
                end
                default:begin status<=PROTOCOL;detail<=8'hff;state<=error_state_next;end
            endcase
        end
    end
endmodule
`default_nettype wire
