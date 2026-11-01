`timescale 1ns/1ps
`default_nettype none

// 显示引用控制模块+在途行读取保护。每个已接受的行请求（包括跨域排队）
// 保留完整(45+EPOCH_W)位标签，只有收到标签匹配的排空事件，才清除对应的在途记录。
// 换引用后，旧 epoch 的行读取全部结束后，通过 mailbox 向 sys 提交 EPOCH_W 位完成确认。
// 成功响应必须取得同 epoch 的行读取完成确认才能消费，因此slot_owner镜像一直保留旧槽。
// 行读取失败时，须等待 SDRAM 段完成后才能发布事件。
// 显示描述符同沿优先于新装载；已有装载保留槽禁止新描述符准入。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
// EPOCH_W 为播放/显示标识位宽；默认 32 位、整机 8 位，整条连接必须同宽。
// 仅改变epoch字段，slot_gen/SD初始化标识保持32位；握手仍限制单个在途换图。
module display_read_owner #(parameter integer EPOCH_W=32,parameter ENABLE_AUDIO_META=0)(
    input wire arst,sys_clk,px_clk,
    input wire [3:0] slot_reserved,output wire [3:0] slot_writable,
    input wire commit_valid,output wire commit_ready,
    input wire [1:0] commit_slot,input wire [31:0] commit_slot_gen,
    input wire desc_valid,output wire desc_ready,input wire [EPOCH_W+223:0] desc_bus,
    output wire ack_valid,input wire ack_ready,output wire [EPOCH_W+20:0] ack_bus,
    input wire vblank,prefetch_ready,
    output wire ref_valid,ref_switch,output wire [EPOCH_W-1:0] ref_epoch,output wire [31:0] ref_slot_gen,
    output wire [1:0] ref_slot,output wire [15:0] ref_frame,
    // px域观察已握手的行请求及已消费的行读取完成事件，不是原始valid。
    input wire read_req_fire,input wire [EPOCH_W-1:0] read_epoch,input wire [31:0] read_slot_gen,
    input wire [1:0] read_slot,read_bank,input wire [8:0] read_source_y,
    input wire row_evt_fire,input wire [EPOCH_W+46:0] row_evt_bus,
    output wire guard_fault,
    output wire audio_commit_go,output wire [19:0] audio_commit_data,ref_audio
);
    wire srst_logic;
    wire srst,prst;
    reset_sync u_sys_reset(.clk(sys_clk),.arst(arst),.rst(srst),.logic_rst(srst_logic));
    reset_sync u_px_reset(.clk(px_clk),.arst(arst),.rst(prst));
    wire raw_desc_ready,raw_ack_valid,raw_ack_ready;
    wire [3:0] raw_writable;
    wire [EPOCH_W+20:0] raw_ack;
    wire target_free=!slot_reserved[desc_bus[157:156]];
    assign desc_ready=raw_desc_ready && target_free && !guard_fault;
    wire desc_fire=desc_valid && desc_ready;
    // 组合屏蔽，使装载器在同一sys采样沿看到描述符的目标已经不可写。
    assign slot_writable=guard_fault ? 4'd0 :
        raw_writable & ~(desc_fire ? (4'b0001<<desc_bus[157:156]) : 4'd0);
    slot_owner #(.EPOCH_W(EPOCH_W),.ENABLE_AUDIO_META(ENABLE_AUDIO_META)) u_owner(.arst(arst),.sys_clk(sys_clk),.px_clk(px_clk),
        .slot_reserved(slot_reserved),.slot_writable(raw_writable),
        .commit_valid(commit_valid),.commit_ready(commit_ready),
        .commit_slot(commit_slot),.commit_slot_gen(commit_slot_gen),
        .desc_valid(desc_fire),.desc_ready(raw_desc_ready),.desc_bus(desc_bus),
        .ack_valid(raw_ack_valid),.ack_ready(raw_ack_ready),.ack_bus(raw_ack),
        .vblank(vblank),.prefetch_ready(prefetch_ready),
        .ref_valid(ref_valid),.ref_switch(ref_switch),.ref_epoch(ref_epoch),
        .ref_slot_gen(ref_slot_gen),.ref_slot(ref_slot),.ref_frame(ref_frame),
        .audio_commit_go(audio_commit_go),.audio_commit_data(audio_commit_data),.ref_audio(ref_audio));

    reg [3:0] pending;
    reg [EPOCH_W+44:0] tag0,tag1,tag2,tag3;
    wire [1:0] eb=row_evt_bus[EPOCH_W+44:EPOCH_W+43];
    wire [EPOCH_W+44:0] event_tag=eb==0 ? tag0 : eb==1 ? tag1 : eb==2 ? tag2 : tag3;
    wire event_match=pending[eb] && event_tag==row_evt_bus[EPOCH_W+44:0];
    wire event_bad=row_evt_fire && (!event_match || row_evt_bus[EPOCH_W+46:EPOCH_W+45]==2'd3);
    // 为每个行缓存区预先计算重复申请条件，再由本拍 read_bank 选择。
    // 同拍收到匹配的行读取完成事件时可清除 pending，新请求因此能在同一行缓存区继续登记。
    wire [3:0] rb_pre;
    genvar gb;
    generate for(gb=0;gb<4;gb=gb+1) begin: g_rb
        assign rb_pre[gb]=pending[gb] && !(row_evt_fire && event_match && eb==gb);
    end endgenerate
    wire request_bad=read_req_fire && rb_pre[read_bank];
    reg wait_old,token_pending,px_fault;
    reg [EPOCH_W-1:0] switch_epoch;
    wire old_pending=(pending[0] && tag0[EPOCH_W+42:43]!=switch_epoch) ||
                     (pending[1] && tag1[EPOCH_W+42:43]!=switch_epoch) ||
                     (pending[2] && tag2[EPOCH_W+42:43]!=switch_epoch) ||
                     (pending[3] && tag3[EPOCH_W+42:43]!=switch_epoch);
    (* keep *) wire of_base=event_bad || (ref_switch && (wait_old || token_pending));
    (* keep *) wire [3:0] of_req={of_base||rb_pre[3],of_base||rb_pre[2],of_base||rb_pre[1],of_base||rb_pre[0]};
    wire observe_fault=read_req_fire ? of_req[read_bank] : of_base;
`ifdef FORMAL
    always @* assert(observe_fault==(event_bad || request_bad || (ref_switch && (wait_old || token_pending))));
`endif
    wire token_src_ready,token_dst_valid,token_dst_ready;
    wire [EPOCH_W-1:0] token_dst;
    cdc_mailbox #(.DATA_WIDTH(EPOCH_W)) u_retired(.arst(arst),
        .src_clk(px_clk),.src_valid(token_pending && !px_fault && !observe_fault),
        .src_ready(token_src_ready),.src_data(switch_epoch),
        .src_active(),.src_busy(),.src_done(),
        .dst_clk(sys_clk),.dst_valid(token_dst_valid),.dst_ready(token_dst_ready),
        .dst_data(token_dst),.dst_active());
    // 标签数据不复位，以减少宽数据寄存器的复位扇出。
    // pending 由复位清零，标签仅在 pending=1 时参与匹配；请求接受沿同时写标签并置 pending。
    // 因而任何可观察的标签均已写入，复位期间的无效数据不会影响读取完成和所有权判断。
    always @(posedge px_clk) begin
        if(read_req_fire && !request_bad) begin
            case(read_bank)
                0:tag0<={read_bank,read_epoch,read_slot,read_slot_gen,read_source_y};
                1:tag1<={read_bank,read_epoch,read_slot,read_slot_gen,read_source_y};
                2:tag2<={read_bank,read_epoch,read_slot,read_slot_gen,read_source_y};
                3:tag3<={read_bank,read_epoch,read_slot,read_slot_gen,read_source_y};
            endcase
        end
    end
    always @(posedge px_clk or posedge prst) begin
        if(prst) begin
            pending<=0;
            wait_old<=0;token_pending<=0;switch_epoch<=0;px_fault<=0;
        end else begin
            if(observe_fault) px_fault<=1;
            if(row_evt_fire && event_match && !event_bad) pending[eb]<=1'b0;
            if(read_req_fire && !request_bad) pending[read_bank]<=1'b1;
            if(ref_switch) begin wait_old<=1'b1;switch_epoch<=ref_epoch;end
            if(wait_old && !old_pending && !ref_switch && !px_fault && !observe_fault &&
               !(read_req_fire && read_epoch!=switch_epoch)) begin
                wait_old<=1'b0;token_pending<=1'b1;
            end
            if(token_pending && token_src_ready && !px_fault && !observe_fault) token_pending<=0;
        end
    end
    reg token_valid,sys_fault;
    reg [EPOCH_W-1:0] retired_epoch;
    (* preserve="true", allow_forward_retiming="false", allow_backward_retiming="false" *)
    reg fault_s1,fault_s2;
    wire success=raw_ack[EPOCH_W+20:EPOCH_W+18]==3'd0;
    wire proved=token_valid && retired_epoch==raw_ack[EPOCH_W+17:18];
    // 只用 sys 域同步复位 srst（异步置位，覆盖 arst 有效期并多保持 3 拍），arst 仅用于异步置位。
    assign guard_fault=sys_fault || fault_s2 || srst_logic;
    assign ack_valid=raw_ack_valid && (!success || proved) && !guard_fault;
    assign ack_bus=raw_ack;
    assign raw_ack_ready=ack_ready && (!success || proved) && !guard_fault;
    wire ack_fire=ack_valid && ack_ready;
    assign token_dst_ready=!token_valid;
    always @(posedge sys_clk or posedge srst) begin
        if(srst) begin
            token_valid<=0;retired_epoch<=0;sys_fault<=0;fault_s1<=0;fault_s2<=0;
        end else begin
            fault_s1<=px_fault;fault_s2<=fault_s1;
            if(token_dst_valid && token_dst_ready) begin token_valid<=1;retired_epoch<=token_dst;end
            if(ack_fire && success) token_valid<=0;
            if(raw_ack_valid && success && token_valid && !proved) sys_fault<=1;
        end
    end
endmodule
`default_nettype wire
