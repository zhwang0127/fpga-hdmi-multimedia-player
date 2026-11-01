`timescale 1ns/1ps
`default_nettype none
// 标准BMP字节流 -> 同一个RGB产生点 -> 主图tile与中心缩略图 -> 唯一SDRAM控制器。
// pixel_valid/ready与R字节同边沿接受，不以简单ready相与驱动两个独立的valid消费者。
// 这里只给出未提交图/缩略图的联合本地结果；CRC、FAT stop_ack及READY引用控制模块仍在外层。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module bmp_thumbnail_sdram_loader #(
    // 缩略图生成默认开启，整机 WAV 配置显式关闭。
    // 关闭时只等待主图真实结果，thumbnail_words明确为0，不伪造缩略图ready/done。
    parameter EXT_FORMATS=0,
    parameter ALLOW_32=1,
    parameter ENABLE_THUMBNAILS=1,
    parameter EXTERNAL_MEMORY=0,POWER_WAIT_CYCLES=25000,REFRESH_COUNT=8,
    parameter REFRESH_INTERVAL=937,REFRESH_GRACE=937
)(
    input wire clk,arst,
    input wire start_valid,output wire start_ready,
    input wire [31:0] file_length,budget_cycles,input wire [1:0] slot,input wire abort,
    input wire byte_valid,output wire byte_ready,input wire [7:0] byte_data,input wire byte_last,
    // byte_ready 在本模块顶层 abort=0 时的取值（cancel_pending/peer_stop/timed_out 仍计入），
    // 恒有 byte_ready==byte_ready_na&&!abort。提交层的接受式本身已含 !stop_now（即 abort=0），
    // 用它消去重复的取消项；不能单独当接受信号。
    output wire byte_ready_na,
    // start_ready 在 abort=0 时的取值，恒有 start_ready==start_ready_na&&!abort。
    // 提交层只在 !m_active 时接受命令，而它送进来的 abort=m_active&&stop_now 此时恒为 0，故可直接用本信号；
    // 不能在 abort 可能为 1 的场合单独当接受信号。
    output wire start_ready_na,
    output wire result_valid,input wire result_ready,output wire result_ok,
    output wire [7:0] result_detail,output wire [18:0] out_written_words,
    output wire [13:0] out_thumbnail_words,
    // EXTERNAL_MEMORY=1时接产品唯一共享控制器（mem_clk域）；禁止并接命令总线。
    // 已接受BL8在取消时仍连续供满八拍；完成须来自真实PRE/done，不撤销在途所有权。
    // 分块边界只排空能独立推进的写，未满BL8的像素仍留给下一块。
    output wire quantum_idle,
    output wire load_valid,input wire load_ready,output wire [20:0] load_addr,
    output wire [3:0] load_count,output wire [31:0] load_budget,
    input wire load_beat,load_done,load_done_was_write,
    output wire wr_data_valid,output wire [31:0] wr_data,
    output wire sd_cs_n,sd_ras_n,sd_cas_n,sd_we_n,sd_cke,
    output wire [10:0] sd_addr,output wire [1:0] sd_ba
);
    wire rst_logic;
    localparam IDLE=0,HEADER=1,LOAD_START=2,LOAD=3,RESULT=4;
    reg [2:0] phase;reg [31:0] flen;reg [31:0] remaining;reg [1:0] target_slot;
    // 父期限用递推标志；子预算提前寄存remaining-1，避免父比较/减法再串子到期判断。
    // 两份计数同沿推进，LOAD_START传下去的值仍严格扣除该启动拍，不增加等待拍。
    reg timed_out_q;reg [31:0] remaining_less_one;
    // remaining>=2的递推镜像，在递减消费点上与!timed_out逐拍一致；独立寄存器
    // 拆掉timed_out_q单线到32位计数器组的高扇出布线。粘滞自反馈仅防综合
    // 把它当反极性副本合并回原寄存器，域内取值与无粘滞写法相同。
    reg rem_ge2_q;
    reg [7:0] detail,first_failure;reg ok,cancel_pending,load_entered,peer_stop,failure_valid;
    wire rst;reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 业务门控只使用本端reset_sync输出；该rst异步置位、三级同步释放，
    // 已覆盖arst有效区间。原始arst仅进入复位链，避免sys复位直接穿过mem数据/握手锥。
    wire enabled=!rst_logic;
    wire head_ready,head_byte_ready,head_result,head_ok,head_bottom_up;
    wire [7:0] head_detail;wire [31:0] head_offset;
    wire [1:0] head_format;wire [6:0] head_consumed;
    wire timed_out=(phase==HEADER || phase==LOAD_START) && timed_out_q;
    // timed_out_q 的取消族副本。原单线同时驱动计数器守卫（已由 rem_ge2_q
    // 分担）、detail/child_le_one_q 以及 local_abort→两个子装载的 abort 锥；
    // grant 预选后 mem 域布线紧张，local_abort 分量换用独立粘滞镜像分摊。
    // 与 timed_out_q 同初值同转移逐拍等值，粘滞自反馈防综合合并。
    reg timed_out_ab_q;
    wire timed_out_ab=(phase==HEADER || phase==LOAD_START) && timed_out_ab_q;
    // 子预算本身饱和为零，直接从寄存器送出，避免phase/timeout先门控32位再进子检查。
    // 有效joint_start沿仍恰为max(remaining-1,0)，已耗尽绝不回绕成超大预算。
    // 其它phase的无效预算允许不同；两个子模块仅在真实start握手沿取预算。
    wire [31:0] child_budget=remaining_less_one;
    // 预算宽比较留在本级标志D，避免remaining_less_one再经两子模块起始状态串行传播。
    // child_budget在真实joint_start恰为max(remaining-1,0)，故zero==remaining<=1，
    // le_one==remaining<=2；先比较完整32位再缓存，不截断/增加预算或准备拍。
    reg child_le_one_q;
    reg launch_descriptor_bad_q;
    // 与launch_descriptor_bad_q同一采样沿（HEADER且头检结果有效）寄存offset==54。
    // 该沿bf_off四个字节早已落入raw（字节10..13，结果在idx53），之后header回IDLE且无pending，
    // raw 保持到 LOAD_START 的 joint_start 接受沿，缓存标志与该沿描述符字段对应。
    reg launch_offset_is_54_q;
    wire [12:0] launch_offset_room=flen[12:0]^13'h1000;
    wire [31:0] source_pixel_bytes=head_format==2?32'd614400:ALLOW_32 && head_format==1?32'd1228800:32'd921600;
    wire [31:0] source_room=flen-source_pixel_bytes;
    // 关闭32bit时只剩两个固定像素区，完整长度带检查后直接比较低13位可用偏移。
    // 565的0x96000低13位为0，24bit的0xE1000低13位0x1000需翻转bit12；上界/下界仍全检。
    // ALLOW_32=1保留三格式选择表达式，不让裁剪配置再综合三选一32位减法。
    wire launch_descriptor_bad_live=EXT_FORMATS?
          (ALLOW_32 ?
           (head_offset<{25'd0,head_consumed} || head_offset>4096 || flen<source_pixel_bytes ||
            source_room>4096 || head_offset>source_room) :
           (head_offset<{25'd0,head_consumed} || head_offset>4096 ||
            (head_format==2 ? (flen[31:13]!=19'd75 || flen[12:0]>13'd4096) :
                             (flen<32'he1000 || flen>32'he2000)) ||
            head_offset[12:0]>(head_format==2 ? flen[12:0] : launch_offset_room))):
          (head_offset<32'd54 || head_offset>32'd4096 ||
           flen<32'he1000 || flen>32'he2000 || head_offset[12:0]>launch_offset_room);
    wire local_abort=abort || cancel_pending || timed_out_ab;
    wire load_abort=abort || cancel_pending || peer_stop;
    wire tile_start_ready,tile_byte_ready,tile_result,tile_ok;
    wire thumb_start_ready,thumb_pixel_ready,thumb_result,thumb_ok;
    wire tap_valid_core,thumb_pixel_ready_core;   // 共用 load_abort 的核心握手
    wire [7:0] tile_detail,thumb_detail;wire [18:0] tile_written;wire [13:0] thumb_written;
    wire joint_start=phase==LOAD_START && tile_start_ready && (!ENABLE_THUMBNAILS || thumb_start_ready);
    wire retire_load=phase==RESULT && result_ready;
    wire tap_valid,tap_last;wire [23:0] tap_rgb;
    wire t_valid,t_ready,t_data_valid,t_beat,t_done,h_valid,h_ready,h_data_valid,h_beat,h_done;
    wire t_intent,h_intent;
    wire [20:0] t_addr,h_addr;wire [3:0] t_count,h_count;wire [31:0] t_budget,h_budget,t_data,h_data;
    wire m_valid,m_ready,m_beat,m_done,m_done_w;wire [20:0] m_addr;
    wire [3:0] m_count;wire [31:0] m_budget;
    // 一个真实的本地请求槽。子端先在本地握手转交所有权，再向唯一后端排队。
    // 下游valid只由寄存的槽/owner驱动，不能再把实时abort/预算/子valid串到bank许可。
    // 取消到来时，已接受的请求仍须真实后端接受并供满BL8、等待PRE/done；不伪造done。
    // 子端REQUEST/WAIT_WRITE均逐拍扣同一文件预算，排队时间计入原deadline，不延长期限。
    reg request_pending,request_owner,busy,owner,prefer_thumb;
    // 末字能同拍产生子请求；请求意向、本地槽和后台在途都必须为空。
    assign quantum_idle=enabled && phase!=LOAD_START && !t_intent && !h_intent &&
        !request_pending && !busy;
    reg [20:0] request_addr;reg [3:0] request_count;reg [31:0] request_budget;
    // 共同load_abort只影响真实接受；槽选择在queue_take时必有!load_abort。
    // 用两支原意向展开选择，取消不再重复进入地址/owner选择锥；不能以意向伪造请求。
    wire choose_thumb=h_intent && (!t_intent || prefer_thumb);
    wire queue_ready=enabled && !busy && !request_pending;
    wire queue_take=queue_ready && !load_abort && (t_intent || h_intent);
    // request_pending 和 busy 由异步复位清零，置位时已包含活动条件。
    // 请求与写数据许可直接使用这两个标志；空闲态的 queue_ready/start_ready/result_valid 仍需复位门控。
    assign m_valid=!busy && request_pending;
    assign m_addr=request_addr;assign m_count=request_count;assign m_budget=request_budget;
    // 两路请求按 prefer_thumb 和对方请求意向决定 ready。
    // 实际接受仍须本路 valid 与 ready 同时成立，并由 queue_take 锁存 owner 和地址。
    // 取消阻止新请求入槽，已经入槽的请求继续等待后端完成。
    assign t_ready=queue_ready && (!prefer_thumb || !h_intent);
    assign h_ready=queue_ready && (prefer_thumb || !t_intent);
    assign wr_data_valid=busy && (owner?h_data_valid:t_data_valid);   // enabled 冗余（busy 由 rst 异步清零）
    assign wr_data=owner?h_data:t_data;
    assign t_beat=busy && !owner && m_beat;
    assign h_beat=busy && owner && m_beat;
    assign t_done=busy && !owner && m_done;
    assign h_done=busy && owner && m_done;
    // 父模块回到 IDLE 前，头检查、主图及已启用的缩略图消费者均须确认处理完毕且在途写段已完成。
    // 因此启动条件由父级空闲状态给出，避免将子级 ready 反压串回文件命令形成组合环路。
    assign start_ready_na=enabled && phase==IDLE;
    assign start_ready=start_ready_na && !abort;   // 取消时不接受新装载
    // LOAD 时 timed_out 恒假，真实消费者byte_ready/pixel_valid已包含同一load_abort，
    // LOAD 入口使用消费者的字节和像素接受条件，取消由共同 load_abort 屏蔽。
    // 按顶层 abort 香农分解。local_abort/load_abort 去掉 abort 后分别为
    // cancel_pending||timed_out 与 cancel_pending||peer_stop；子模块 ready=核心&&!各自abort。
    wire head_byte_ready_core,tile_byte_ready_core;
    (* keep *) wire byte_ready_na_w=enabled &&
        ((phase==HEADER && !cancel_pending && !timed_out && !peer_stop && head_byte_ready_core) ||
         (phase==LOAD && !cancel_pending && !peer_stop && tile_byte_ready_core));
    assign byte_ready_na=byte_ready_na_w;
    assign byte_ready=byte_ready_na_w && !abort;   // 取消时不接受新字节
    assign result_valid=enabled && phase==RESULT;
    assign result_ok=ok;
    assign result_detail=detail;
    assign out_written_words=load_entered?tile_written:19'd0;
    assign out_thumbnail_words=load_entered?thumb_written:14'd0;
    bmp_header_check #(.EXT_FORMATS(EXT_FORMATS),.ALLOW_32(ALLOW_32)) u_header(.clk(clk),.arst(arst),
        .start_valid(start_valid && start_ready),.start_ready(head_ready),.file_length(file_length),
        .byte_valid(byte_valid && phase==HEADER && !local_abort),.byte_ready(head_byte_ready),.byte_ready_core(head_byte_ready_core),
        .byte_data(byte_data),.byte_last(byte_last),.abort(local_abort && phase==HEADER),
        .result_valid(head_result),.result_ready(phase==HEADER),.result_ok(head_ok),
        .result_detail(head_detail),.out_offset(head_offset),.out_bottom_up(head_bottom_up),.out_pixel_format(head_format),.out_header_consumed(head_consumed));
    // 无缩略图消费者时停用旁路观察口；主图字节/BL8仍走真实握手。
    wire tile_tap_ready=!ENABLE_THUMBNAILS || thumb_pixel_ready;
    wire tile_tap_ready_core=!ENABLE_THUMBNAILS || thumb_pixel_ready_core;
    bmp_tile_loader #(.EXT_FORMATS(EXT_FORMATS),.ALLOW_32(ALLOW_32),.ENABLE_PIXEL_TAP(ENABLE_THUMBNAILS),.USE_LAUNCH_FLAGS(1)) u_tile(.clk(clk),.arst(arst),
        .start_valid(joint_start),.start_ready(tile_start_ready),
        // 头检不限制目录长度上限；必须保留32bit交给tile完整描述符范围检查。
        .file_length(flen),.pixel_offset(head_offset),.bottom_up(head_bottom_up),.slot(target_slot),
        .pixel_format(head_format),.header_consumed(head_consumed),
        .budget_cycles(child_budget),.abort(load_abort),
        .launch_budget_zero(timed_out_q),.launch_budget_le_one(child_le_one_q),
        .launch_descriptor_bad(launch_descriptor_bad_q),.launch_offset_is_54(launch_offset_is_54_q),
        .byte_valid(byte_valid && phase==LOAD),.byte_ready(tile_byte_ready),.byte_ready_core(tile_byte_ready_core),
        .byte_data(byte_data),.byte_last(byte_last),
        // tile 与缩略图共用 load_abort，交叉连接两侧去掉该项的核心握手供余因子使用。
        .pixel_valid(tap_valid),.pixel_ready(tile_tap_ready),.pixel_rgb(tap_rgb),.pixel_last(tap_last),
        .pixel_valid_core(tap_valid_core),.pixel_ready_core(tile_tap_ready_core),
        .seg_valid(t_valid),.seg_ready(t_ready),.seg_intent(t_intent),.seg_addr(t_addr),.seg_bl8_count(t_count),.seg_budget_cycles(t_budget),
        .wr_data_valid(t_data_valid),.wr_data(t_data),.wr_beat(t_beat),
        .seg_done(t_done),.seg_done_was_write(m_done_w),
        .result_valid(tile_result),.result_ready(retire_load),.result_ok(tile_ok),
        .result_detail(tile_detail),.out_written_words(tile_written));
    generate if(ENABLE_THUMBNAILS) begin:g_thumbnail
    thumbnail_sampler #(.USE_LAUNCH_FLAGS(1)) u_thumbnail(.clk(clk),.arst(arst),
        .start_valid(joint_start),.start_ready(thumb_start_ready),
        .bottom_up(head_bottom_up),.slot(target_slot),.budget_cycles(child_budget),.abort(load_abort),
        .launch_budget_zero(timed_out_q),.launch_budget_le_one(child_le_one_q),
        .pixel_valid(tap_valid),.pixel_ready(thumb_pixel_ready),.pixel_rgb(tap_rgb),.pixel_last(tap_last),
        .pixel_valid_core(tap_valid_core),.pixel_ready_core(thumb_pixel_ready_core),
        .seg_valid(h_valid),.seg_ready(h_ready),.seg_intent(h_intent),.seg_addr(h_addr),.seg_bl8_count(h_count),.seg_budget_cycles(h_budget),
        .wr_data_valid(h_data_valid),.wr_data(h_data),.wr_beat(h_beat),
        .seg_done(h_done),.seg_done_was_write(m_done_w),
        .result_valid(thumb_result),.result_ready(retire_load),.result_ok(thumb_ok),
        .result_detail(thumb_detail),.out_written_words(thumb_written));
    end else begin:g_no_thumbnail
        // 功能停用，既不申请写段，也不声明任何完成。上层控制模块仅根据主图结果判断装载是否完成。
        assign thumb_start_ready=1'b0;assign thumb_pixel_ready=1'b0;assign thumb_pixel_ready_core=1'b0;
        assign thumb_result=1'b0;assign thumb_ok=1'b0;assign thumb_detail=8'd0;assign thumb_written=14'd0;
        assign h_valid=1'b0;assign h_intent=1'b0;assign h_data_valid=1'b0;
        assign h_addr=21'd0;assign h_count=4'd0;assign h_budget=32'd0;assign h_data=32'd0;
    end endgenerate
    generate if(EXTERNAL_MEMORY) begin: g_shared
        assign load_valid=m_valid;assign m_ready=load_ready;
        assign load_addr=m_addr;assign load_count=m_count;assign load_budget=m_budget;
        assign m_beat=load_beat;assign m_done=load_done;assign m_done_w=load_done_was_write;
        // 外部配置没有第二套命令后端；未使用的兼容输出保持物理空闲。
        assign {sd_cs_n,sd_ras_n,sd_cas_n,sd_we_n,sd_cke}=5'b11110;
        assign sd_addr=11'd0;assign sd_ba=2'd0;
    end else begin: g_standalone
        assign load_valid=1'b0;assign load_addr=21'd0;assign load_count=4'd0;assign load_budget=32'd0;
    sdram_controller #(.POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),.REFRESH_COUNT(REFRESH_COUNT),
        .REFRESH_INTERVAL(REFRESH_INTERVAL),.REFRESH_GRACE(REFRESH_GRACE)) u_memory(
        .clk(clk),.arst(arst),.seg_valid(m_valid),.seg_ready(m_ready),.seg_is_write(1'b1),
        .seg_addr(m_addr),.seg_bl8_count(m_count),.seg_budget_cycles(m_budget),
        .wr_data_valid(wr_data_valid),.wr_data(wr_data),.wr_beat(m_beat),
        .rd_data_valid(),.rd_data_ready(1'b1),.rd_data_in(32'd0),
        .seg_done(m_done),.seg_done_was_write(m_done_w),.seg_cancel(1'b0),
        .sd_cs_n(sd_cs_n),.sd_ras_n(sd_ras_n),.sd_cas_n(sd_cas_n),.sd_we_n(sd_we_n),
        .sd_cke(sd_cke),.sd_addr(sd_addr),.sd_ba(sd_ba));
    end endgenerate
    // 槽接受、后端接受和后端完成三个边界独立；不能把本地槽空当作后端完成。
    // request_*在排队期间稳定，owner在物理接受后直到真实done稳定。
    // 头检查结果接受沿缓存描述符，下一拍 LOAD_START 向消费者发起同一份描述符。
    always @(posedge clk or posedge rst) begin
        if(rst) begin child_le_one_q<=1;launch_descriptor_bad_q<=0;launch_offset_is_54_q<=0;end
        else begin
            if(start_valid && start_ready) child_le_one_q<=budget_cycles<=32'd2;
            else if((phase==HEADER || phase==LOAD_START) && !timed_out)
                child_le_one_q<=remaining<=32'd3;
            if(phase==HEADER && head_result) launch_descriptor_bad_q<=launch_descriptor_bad_live;
            if(phase==HEADER && head_result) launch_offset_is_54_q<=head_offset==(EXT_FORMATS?{25'd0,head_consumed}:32'd54);
        end
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            request_pending<=0;request_owner<=0;busy<=0;owner<=0;prefer_thumb<=0;
            request_addr<=0;request_count<=0;request_budget<=0;
        end else begin
            if(queue_take) begin
                request_pending<=1;request_owner<=choose_thumb;
                request_addr<=choose_thumb?h_addr:t_addr;
                request_count<=choose_thumb?h_count:t_count;
                request_budget<=choose_thumb?h_budget:t_budget;
            end
            if(m_valid && m_ready) begin
                request_pending<=0;busy<=1;owner<=request_owner;prefer_thumb<=!request_owner;
            end
            if(busy && m_done) busy<=0;
        end
    end
    // 缩略图错误1/2与主图共用取消/期限码，其它缩略图错误保留0xC0来源位。
    wire [7:0] thumb_failure=(thumb_detail==1 || thumb_detail==2)?
        {1'b1,thumb_detail[6:0]}:{2'b11,thumb_detail[5:0]};
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            phase<=IDLE;flen<=0;remaining<=0;timed_out_q<=1;timed_out_ab_q<=1;remaining_less_one<=0;
            rem_ge2_q<=0;
            target_slot<=0;detail<=0;ok<=0;
            cancel_pending<=0;load_entered<=0;peer_stop<=0;failure_valid<=0;first_failure<=0;
        end else begin
            if(abort && phase!=IDLE && phase!=RESULT) cancel_pending<=1;
            if((phase==HEADER || phase==LOAD_START) && rem_ge2_q) begin
                remaining<=remaining-32'd1;
                if(remaining_less_one!=0) remaining_less_one<=remaining_less_one-32'd1;
            end
            if((phase==HEADER || phase==LOAD_START) && !timed_out)
                timed_out_q<=remaining==32'd2;
            if((phase==HEADER || phase==LOAD_START) && !timed_out_ab_q)
                timed_out_ab_q<=timed_out_ab_q || (remaining==32'd2);
            if((phase==HEADER || phase==LOAD_START) && rem_ge2_q)
                rem_ge2_q<=rem_ge2_q && (remaining>=32'd3);
            case(phase)
                IDLE:if(start_valid && start_ready) begin
                    flen<=file_length;remaining<=budget_cycles;
                    remaining_less_one<=budget_cycles==0?32'd0:budget_cycles-32'd1;
                    timed_out_q<=budget_cycles<=32'd1;timed_out_ab_q<=budget_cycles<=32'd1;rem_ge2_q<=budget_cycles>=32'd2;target_slot<=slot;
                    phase<=HEADER;ok<=0;cancel_pending<=0;load_entered<=0;peer_stop<=0;failure_valid<=0;
                end
                HEADER:if(head_result) begin
                    if(head_ok && !local_abort) phase<=LOAD_START;
                    else begin detail<=(abort || cancel_pending)?8'h81:(timed_out?8'h82:{1'b0,head_detail[6:0]});ok<=0;phase<=RESULT;end
                end
                LOAD_START:if(joint_start) begin load_entered<=1;phase<=LOAD;end
                LOAD:begin
                    if((tile_result && !tile_ok) || (thumb_result && !thumb_ok)) begin
                        peer_stop<=1;
                        if(!failure_valid) begin
                            failure_valid<=1;first_failure<=(tile_result && !tile_ok)?{1'b1,tile_detail[6:0]}:thumb_failure;
                        end
                    end
                    // 一支先失败先锁原因，另一支须排空或本地停止；不能先返回再撤销owner。
                    if(tile_result && (!ENABLE_THUMBNAILS || thumb_result)) begin
                        ok<=tile_ok && (!ENABLE_THUMBNAILS || thumb_ok) && !load_abort;
                        if(failure_valid) detail<=first_failure;
                        else if(!tile_ok) detail<={1'b1,tile_detail[6:0]};
                        else if(ENABLE_THUMBNAILS && !thumb_ok) detail<=thumb_failure;
                        else detail<=load_abort?8'h81:8'd0;
                        phase<=RESULT;
                    end
                end
                RESULT:if(result_valid && result_ready) phase<=IDLE;
                default:phase<=IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
