`timescale 1ns/1ps
`default_nettype none

// 读引用控制模块：四只读图槽的所有权与播放epoch原子提交。
// 拥有像素域槽{valid,gen}表与显示引用{epoch,slot,gen}；装载commit经mailbox
// 入表，(224+EPOCH_W)bit播放描述符经mailbox进入像素域，仅在vblank且行预取就绪的
// 单一边界换引用。分别检查 reserved 位、epoch、旧槽及新槽，拒绝与当前状态不符的描述符；
// commit结果{status,epoch,frame,slot}经mailbox回sys。
// 描述符布局：epoch[EPOCH_W+223:224] old_gen[223:192] new_gen[191:160]
// old_slot[159:158] new_slot[157:156] 下游字段[155:124]；音频关闭时保留[123:0]=0。
// ENABLE_AUDIO_META时[19]为存在位、[18]为PCM 缓冲区、[17:0]为样本数，其余保留位仍必须为0。
// slot_writable在sys域屏蔽装载占用槽(含quarantine经busy保持)、显示引用槽
// 与已提交未决描述符的目标槽；提交沿即屏蔽目标，不等像素域换引用。
// 一次只允许一个在途描述符：sys收到commit响应前不再接受新描述符。
// 擦拭/PCM等其它引用种类由各自模块持有，本模块只拥有显示引用。
// EPOCH_W 为播放/显示标识位宽；默认 32 位、整机 8 位，整条连接必须同宽。
// 仅改变epoch字段，slot_gen/SD初始化标识保持32位；握手仍限制单个在途换图。
module slot_owner #(parameter integer EPOCH_W=32,parameter ENABLE_AUDIO_META=0)(
    input wire arst,
    input wire sys_clk,input wire px_clk,
    // 装载占用位图(sys)：来自bmp_file_commit，busy含quarantine保持。
    input wire [3:0] slot_reserved,
    output wire [3:0] slot_writable,
    // 装载commit消费(sys)：{slot,gen}表更新经mailbox发往像素域。
    input wire commit_valid,output wire commit_ready,
    input wire [1:0] commit_slot,input wire [31:0] commit_slot_gen,
    // 播放描述符(sys)：(224+EPOCH_W)bit稳定总线；reserved[123:0]非零在像素域拒绝。
    input wire desc_valid,output wire desc_ready,input wire [EPOCH_W+223:0] desc_bus,
    // commit响应(sys)：{status[EPOCH_W+20:EPOCH_W+18],epoch[EPOCH_W+17:18],frame[17:2],slot[1:0]}。
    output wire ack_valid,input wire ack_ready,output wire [EPOCH_W+20:0] ack_bus,
    // 像素域边界：vblank为候选提交边界，prefetch_ready为行预取就绪
    // 换引用时 ref_switch 输出一个周期。
    input wire vblank,input wire prefetch_ready,
    output wire ref_valid,ref_switch,
    output wire [EPOCH_W-1:0] ref_epoch,output wire [31:0] ref_slot_gen,
    output wire [1:0] ref_slot,
    output wire [15:0] ref_frame,
    // 描述符低 20 位为{PCM有效,音频缓冲区,样本数18}；与成功的 commit 在同一沿更新。
    output wire audio_commit_go,output wire [19:0] audio_commit_data,ref_audio
);
    localparam OK=3'd0,OLD_FENCE=3'd1,NEW_FENCE=3'd2,EPOCH_FENCE=3'd3,RESERVED=3'd4;
    wire sys_rst,px_rst;
    reset_sync u_sys_reset(.clk(sys_clk),.arst(arst),.rst(sys_rst));
    reset_sync u_px_reset(.clk(px_clk),.arst(arst),.rst(px_rst));

    // M1 描述符 sys->px；M2 表更新 sys->px；M3 commit响应 px->sys。
    wire m1_src_ready,m1_dst_valid;
    wire [EPOCH_W+223:0] m1_dst_data;
    wire m2_src_ready,m2_dst_valid;
    wire [33:0] m2_dst_data;
    wire m3_src_ready,m3_dst_valid;
    wire [EPOCH_W+20:0] m3_dst_data;
    reg desc_pending;
    reg [1:0] desc_target;
    reg mirror_valid;
    reg [1:0] mirror_slot;
    // 像素域表/引用/未决状态。
    reg [3:0] table_valid;
    reg [31:0] table_gen0,table_gen1,table_gen2,table_gen3;
    reg ref_on,ref_switch_reg,ack_pending;
    reg [EPOCH_W-1:0] ref_epoch_reg;reg [31:0] ref_gen_reg;
    reg [1:0] ref_slot_reg;
    reg [15:0] frame_reg;
    reg [EPOCH_W+223:0] pend_bus;reg [19:0] audio_reg;
    reg pend_valid;
    // 响应epoch与引用epoch同沿同值：成功都写d_epoch，拒绝都保持旧引用。
    // mailbox接受时仍快照完整载荷；复用这一组FF，去掉重复epoch FF及选择器。
    reg [2:0] ack_status;reg [15:0] ack_frame;reg [1:0] ack_slot;
    wire [EPOCH_W+20:0] ack_data={ack_status,ref_epoch_reg,ack_frame,ack_slot};

    // 未决响应期间不再接新描述符；发送沿立即在sys域屏蔽目标槽。
    assign desc_ready=m1_src_ready && !desc_pending;
    // 描述符一接收就预留目标槽，不等屏幕真正换过去才禁止重写。
    wire desc_fire=desc_valid && desc_ready;
    wire ack_fire=m3_dst_valid && ack_ready;
    assign ack_valid=m3_dst_valid;
    assign ack_bus=m3_dst_data;
    assign commit_ready=m2_src_ready;
    // 引用镜像是提交成功的显示槽；拒绝不改变引用，只解除未决屏蔽。
    wire ack_commit_ok=m3_dst_data[EPOCH_W+20:EPOCH_W+18]==OK;
    assign slot_writable={
        !slot_reserved[3] && !(mirror_valid && mirror_slot==2'd3) && !(desc_pending && desc_target==2'd3),
        !slot_reserved[2] && !(mirror_valid && mirror_slot==2'd2) && !(desc_pending && desc_target==2'd2),
        !slot_reserved[1] && !(mirror_valid && mirror_slot==2'd1) && !(desc_pending && desc_target==2'd1),
        !slot_reserved[0] && !(mirror_valid && mirror_slot==2'd0) && !(desc_pending && desc_target==2'd0)
    };
    cdc_mailbox #(.DATA_WIDTH(EPOCH_W+224)) u_desc(
        .arst(arst),.src_clk(sys_clk),.src_valid(desc_fire),.src_ready(m1_src_ready),
        .src_data(desc_bus),.src_active(),.src_busy(),.src_done(),
        .dst_clk(px_clk),.dst_valid(m1_dst_valid),.dst_ready(!pend_valid),
        .dst_data(m1_dst_data),.dst_active());
    cdc_mailbox #(.DATA_WIDTH(34)) u_table(
        .arst(arst),.src_clk(sys_clk),.src_valid(commit_valid && m2_src_ready),.src_ready(m2_src_ready),
        .src_data({commit_slot,commit_slot_gen}),.src_active(),.src_busy(),.src_done(),
        .dst_clk(px_clk),.dst_valid(m2_dst_valid),.dst_ready(1'b1),
        .dst_data(m2_dst_data),.dst_active());
    cdc_mailbox #(.DATA_WIDTH(EPOCH_W+21)) u_ack(
        .arst(arst),.src_clk(px_clk),.src_valid(ack_pending),.src_ready(m3_src_ready),
        .src_data(ack_data),.src_active(),.src_busy(),.src_done(),
        .dst_clk(sys_clk),.dst_valid(m3_dst_valid),.dst_ready(ack_ready),
        .dst_data(m3_dst_data),.dst_active());

    wire [EPOCH_W-1:0] d_epoch=pend_bus[EPOCH_W+223:224];
    wire [31:0] d_old_gen=pend_bus[223:192];
    wire [31:0] d_new_gen=pend_bus[191:160];
    wire [1:0] d_old_slot=pend_bus[159:158];
    wire [1:0] d_new_slot=pend_bus[157:156];
    // 长度范围与“无音频却带非零载荷”均在接收描述符时缓存。
    // 两者与pend_bus同沿更新，提交拍直接使用预先缓存的字段条件；存在位及其他保留位
    // 仍使用原pending载荷。全0复位对应长度越界=1、无音频非零=0，不增加提交延迟。
    reg audio_length_bad_q;
    reg audio_absent_nonzero_q;
    wire incoming_audio_length_bad=m1_dst_data[17:0]<48000 || m1_dst_data[17:0]>240000;
    wire audio_metadata_bad=pend_bus[19] ? audio_length_bad_q : audio_absent_nonzero_q;
    wire pend_reserved_bad=ENABLE_AUDIO_META ? ((|pend_bus[123:20]) || audio_metadata_bad) : |pend_bus[123:0];
    wire [31:0] d_new_gen_table=d_new_slot==2'd0 ? table_gen0 : d_new_slot==2'd1 ? table_gen1 :
                               d_new_slot==2'd2 ? table_gen2 : table_gen3;
    // 首次提交前引用无效，不检查旧槽是否匹配；epoch从1起按位宽回绕。
    wire fence_reserved=!pend_reserved_bad;
    wire fence_old=!ref_on || (ref_slot_reg==d_old_slot && ref_gen_reg==d_old_gen);
    wire fence_new=table_valid[d_new_slot] && d_new_gen_table==d_new_gen;
    // 模加到EPOCH_W位；一次只发一个描述符，旧图的行读取全部结束后才归还响应。
    wire [EPOCH_W-1:0] next_epoch=ref_epoch_reg+1'b1;
    wire fence_epoch=d_epoch==next_epoch;
    wire [2:0] verdict=!fence_reserved ? RESERVED : !fence_epoch ? EPOCH_FENCE :
                       !fence_old ? OLD_FENCE : !fence_new ? NEW_FENCE : OK;
    wire commit_go=vblank && prefetch_ready && pend_valid && !ack_pending;
    assign ref_valid=ref_on;
    assign ref_switch=ref_switch_reg;
    assign ref_epoch=ref_epoch_reg;
    assign ref_slot=ref_slot_reg;
    assign ref_slot_gen=ref_gen_reg;
    assign ref_frame=frame_reg;
    // 这是边界沿之前的同域条件，先禁止旧FIFO弹出，再同时提交显示/配音。
    assign audio_commit_go=ENABLE_AUDIO_META && commit_go && verdict==OK;
    assign audio_commit_data=ENABLE_AUDIO_META ? pend_bus[19:0] : 20'd0;
    assign ref_audio=ENABLE_AUDIO_META ? audio_reg : 20'd0;

    always @(posedge px_clk or posedge px_rst) begin
        if(px_rst) begin
            table_valid<=0;table_gen0<=0;table_gen1<=0;table_gen2<=0;table_gen3<=0;
            ref_on<=0;ref_switch_reg<=0;ack_pending<=0;ref_epoch_reg<=0;ref_gen_reg<=0;
            ref_slot_reg<=0;frame_reg<=0;audio_reg<=0;audio_length_bad_q<=1;audio_absent_nonzero_q<=0;pend_bus<=0;pend_valid<=0;ack_status<=0;ack_frame<=0;ack_slot<=0;
        end else begin
            ref_switch_reg<=0;
            if(m2_dst_valid) begin
                table_valid[m2_dst_data[33:32]]<=1'b1;
                case(m2_dst_data[33:32])
                    2'd0:table_gen0<=m2_dst_data[31:0];
                    2'd1:table_gen1<=m2_dst_data[31:0];
                    2'd2:table_gen2<=m2_dst_data[31:0];
                    default:table_gen3<=m2_dst_data[31:0];
                endcase
            end
            if(vblank) frame_reg<=frame_reg+16'd1;
            if(m1_dst_valid && !pend_valid) begin
                pend_bus<=m1_dst_data;audio_length_bad_q<=incoming_audio_length_bad;audio_absent_nonzero_q<=|m1_dst_data[18:0];pend_valid<=1'b1;
            end
            if(commit_go) begin
                pend_valid<=1'b0;
                // 响应帧号是提交边界开启的新帧；拒绝时epoch报当前引用值。
                ack_status<=verdict;ack_frame<=frame_reg+16'd1;ack_slot<=d_new_slot;
                ack_pending<=1'b1;
                if(verdict==OK) begin
                    ref_on<=1'b1;ref_epoch_reg<=d_epoch;ref_slot_reg<=d_new_slot;
                    ref_gen_reg<=d_new_gen;ref_switch_reg<=1'b1;audio_reg<=pend_bus[19:0];
                end
            end
            if(ack_pending && m3_src_ready) ack_pending<=0;
        end
    end

    // ---- sys域：未决目标屏蔽与显示槽镜像 ----
    always @(posedge sys_clk or posedge sys_rst) begin
        if(sys_rst) begin
            desc_pending<=0;desc_target<=0;mirror_valid<=0;mirror_slot<=0;
        end else begin
            if(desc_fire) begin desc_pending<=1'b1;desc_target<=desc_bus[157:156];end
            // 响应被消费(任意status)即解除未决屏蔽；成功才更新显示槽镜像。
            if(ack_fire) begin
                desc_pending<=0;
                if(ack_commit_ok) begin mirror_valid<=1'b1;mirror_slot<=m3_dst_data[1:0];end
            end
        end
    end
endmodule
`default_nettype wire
