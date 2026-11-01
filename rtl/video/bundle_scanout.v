`timescale 1ns/1ps
`default_nettype none
// Pixel-domain coordinator for display_row_bundle; callers retain all source slots.
// Each output row owns a full bundle, including repeated main source rows.
// EPOCH_W 为播放/显示标识位宽；默认 32 位、整机 8 位，整条连接必须同宽。
// 仅改变epoch字段，slot_gen/SD初始化标识保持32位；握手仍限制单个在途换图。
module bundle_scanout #(parameter integer EPOCH_W=32,parameter ROW_DEADLINE_MEM=16'd2000)(
    input wire px_clk,arst,
    input wire [10:0] timing_x,input wire [9:0] timing_y,
    // video_timing_720p 寄存的 y 派生量（与 timing_y 同拍有效）：y<720、y<719、y+1、horizon。
    input wire timing_y_lt720,timing_y_lt719,input wire [9:0] timing_y_plus1,timing_horizon,
    // video_timing_720p 寄存的行描述符 row_desc(y)、row_desc(y+1)、row_desc(y+2)（同拍有效，见 video_timing_720p）。
    input wire [19:0] timing_y_desc0,timing_y_desc1,timing_y_desc2,
    // 实际video_timing的同拍条件；必须与timing_x/y/de成套连接。
    // 以次态前置水平比较，减少x到行调度/描述符CE的扇出，不延迟事件或请求。
    input wire timing_cold_line,timing_row_edge,
    input wire de,hsync,vsync,vblank_start,
    input wire ref_valid,ref_switch,input wire [EPOCH_W-1:0] ref_epoch,
    input wire [1:0] old_slot,new_slot,input wire [31:0] old_gen,new_gen,
    input wire [127:0] thumb_gens,
    input wire thumb_layout,input wire [10:0] wipe_x,
    output wire prefetch_ready,
    output wire req_valid,input wire req_ready,output wire [EPOCH_W+134:0] req_bus,
    output wire rel_valid,input wire rel_ready,output wire [EPOCH_W+118:0] rel_bus,
    input wire evt_valid,output wire evt_ready,input wire [EPOCH_W+136:0] evt_bus,
    input wire bundle_fault,output wire fault,
    output wire rd_bank,output wire [10:0] rd_x,input wire [31:0] rd_data,
    output wire pixel_valid,output wire [31:0] pixel_word,
    output wire pixel_hsync,pixel_vsync,output wire [10:0] pixel_x,
    output wire [9:0] pixel_y,output wire [EPOCH_W-1:0] pixel_epoch,
    output wire underrun_pulse,output wire display_present
);
    wire rst;reset_sync u_reset(.clk(px_clk),.arst(arst),.rst(rst));
    localparam FREE=3'd0,PEND=3'd1,READY=3'd2,DISP=3'd3,RET=3'd4;
    reg [2:0] state[0:1];reg [EPOCH_W+134:0] tag[0:1];reg [9:0] scan_y[0:1];
    reg [1:0] stale;reg active,have,bank,sticky,bad,under;
    reg [EPOCH_W-1:0] epoch;reg [31:0] og,ng;reg [1:0] os,ns;reg [127:0] tg;
    reg layout;reg [10:0] boundary;reg [9:0] next_row;
    reg release_hold;reg [EPOCH_W+118:0] release_tag;
    integer k;
    function [8:0] source_y;input [9:0] yy;reg [10:0] w;
        begin w={1'b0,yy}<<1;source_y=w/11'd3;end
    endfunction
    function [1:0] preview_slot;input [9:0] yy;
        begin if(yy>=496) preview_slot=3;else if(yy>=352) preview_slot=2;
        else if(yy>=208) preview_slot=1;else preview_slot=0;end
    endfunction
    function [6:0] preview_y;input [9:0] yy;reg [9:0] base,d;
        begin case(preview_slot(yy)) 0:base=64;1:base=208;2:base=352;default:base=496;endcase
        d=yy-base;preview_y=(yy>=base && d<96) ? d[6:0] : 7'd0;end
    endfunction
    // 行描述符 row_desc(n)={2n mod 3, source_y(n), preview_slot(n), preview_y(n)}。
    // timing_y+1 使用光栅发生器提供的 timing_y_desc1，next_row 使用随行号更新的 nr_d。
    // 各来源均提前保存描述符，请求末级仅由 pick_next 选择，避免行号比较后再串联地址计算。
    // nr_d 必须对应 next_row，sd[k] 必须对应 scan_y[k]，与行号在同一沿更新。
    function [19:0] row_desc;input [9:0] yy;reg [10:0] w,m;
        begin w={1'b0,yy}<<1;m=w%11'd3;row_desc={m[1:0],source_y(yy),preview_slot(yy),preview_y(yy)};end
    endfunction
    localparam [19:0] DESC0=20'd0;   // row_desc(0)：相位 0、source_y 0、slot 0、preview_y 0（0<64）
    reg [19:0] nr_d;reg [19:0] sd[0:1];
    // After an underrun, skip missed output rows instead of perpetually refilling old rows.
    // 所有派生量与当前y同拍有效（由实际video_timing_720p寄存输出）。
    // 10位p1=y+1：p1==1/750/0分别等价y==0/749/1023，含模回绕。
    // pick_next受lt720保护，此时next_row<=y恰好等价next_row<p1，不现场重做y比较。
    // 行号、行缓存编号与描述符必须来自同一拍的光栅及预取状态。
    wire cold_line=timing_cold_line;
    // 行调度使用光栅发生器同步提供的纵向条件和下一行号，避免重复组合比较。
    wire pick_next=timing_y_lt720 && !cold_line && ((next_row[9:5]<timing_y_plus1[9:5]) || (next_row[9:5]==timing_y_plus1[9:5] && next_row[4:0]<timing_y_plus1[4:0]));
    wire [9:0] pick_row=pick_next ? timing_y_plus1 : next_row;
    wire nb=pick_row[0];
    wire [19:0] d_pick=pick_next ? timing_y_desc1 : nr_d;
    wire [1:0] ts=d_pick[8:7];
    function [31:0] tgen_of;input [1:0] t;input [127:0] g;
        begin tgen_of=t==0 ? g[31:0] : t==1 ? g[63:32] : t==2 ? g[95:64] : g[127:96]; end
    endfunction
    wire [31:0] tgen=pick_next ? tgen_of(timing_y_desc1[8:7],tg) : tgen_of(nr_d[8:7],tg);
    wire [9:0] horizon=timing_horizon;
    // 按 pick_row 的两个来源分别计算行范围条件：
    //   pick_next 时 pick_row=timing_y+1 且 timing_y_lt720：pick_row<720 ⇔ timing_y<719，且此时 pick_row<=horizon 恒成立；
    //   否则 pick_row=next_row，条件原样。
    wire pick_ok=pick_next ? timing_y_lt719 : (next_row<10'd720 && next_row<=horizon);
    // 两路候选分别计算申请条件，pick_next 仅作末级选择。
    // 公共条件包括引用有效、无故障、无释放等待和请求端可接受；行缓存区必须空闲。
    wire issue_com=active && !ref_switch && !bad && !bundle_fault && !release_hold && req_ready;
    (* keep *) wire issue_a=issue_com && state[timing_y_plus1[0]]==FREE && timing_y_lt719;
    (* keep *) wire issue_b=issue_com && state[next_row[0]]==FREE && next_row<10'd720 && next_row<=horizon;
    wire issue=pick_next ? issue_a : issue_b;
`ifdef FORMAL
    always @* assert(issue==(active && !ref_switch && !bad && !bundle_fault && !release_hold && state[nb]==FREE && pick_ok && req_ready));
`endif
    assign req_valid=issue;
    assign req_bus={epoch,os,og,ns,ng,d_pick[17:9],ts,tgen,d_pick[6:0],nb,ROW_DEADLINE_MEM};
    assign rel_valid=release_hold && !bad && !bundle_fault;assign rel_bus=release_tag;
    assign evt_ready=!bad && !bundle_fault;
    wire eb=evt_bus[16];wire [1:0] kind=evt_bus[EPOCH_W+136:EPOCH_W+135];
    wire event_match=state[eb]==PEND && evt_bus[EPOCH_W+134:0]==tag[eb];
    assign fault=bad || bundle_fault;assign prefetch_ready=!ref_valid || sticky;
    assign underrun_pulse=under;assign display_present=have;
    // Coordinate-select before the one shared synchronous RAM read.
    wire [10:0] origin=layout ? 11'd0 : 11'd160;
    wire main_de=de && timing_x>=origin && timing_x<origin+11'd960;
    reg [9:0] sx;reg [1:0] phase;
    always @(posedge px_clk or posedge rst) begin
        if(rst) begin sx<=0;phase<=0;end
        else if(!main_de) begin sx<=0;phase<=0;end
        else begin
            if(phase!=0 && sx<639) sx<=sx+1'b1;
            phase<=phase==2 ? 0 : phase+1'b1;
        end
    end
    wire [1:0] ps=preview_slot(timing_y);
    wire [9:0] top=ps==0 ? 10'd64 : ps==1 ? 10'd208 : ps==2 ? 10'd352 : 10'd496;
    wire thumb_de=de && layout && timing_x>=1056 && timing_x<1184 && timing_y>=top && timing_y<top+96;
    wire select_new=(timing_x-origin)<boundary;
    assign rd_bank=bank;
    assign rd_x=thumb_de ? 11'd1280+(timing_x-11'd1056) :
        main_de ? (select_new ? 11'd640 : 11'd0)+{1'b0,sx} : 11'd0;
    reg de1,de2,vis1,vis2,hs1,hs2,vs1,vs2,badrow1,badrow2;
    reg [10:0] x1,x2;reg [9:0] y1,y2;reg [EPOCH_W-1:0] ep1,ep2;reg [31:0] word_q;
    always @(posedge px_clk or posedge rst) begin
        if(rst) begin badrow1<=0;badrow2<=0;de1<=0;de2<=0;vis1<=0;vis2<=0;hs1<=0;hs2<=0;vs1<=0;vs2<=0;
            x1<=0;x2<=0;y1<=0;y2<=0;ep1<=0;ep2<=0;word_q<=0;end
        else begin
            badrow1<=de && (!have || fault);badrow2<=badrow1;
            de1<=de;de2<=de1;vis1<=have && !fault && (main_de || thumb_de);vis2<=vis1;
            hs1<=hsync;hs2<=hs1;vs1<=vsync;vs2<=vs1;
            x1<=timing_x;x2<=x1;y1<=timing_y;y2<=y1;ep1<=epoch;ep2<=ep1;word_q<=rd_data;
        end
    end
    assign pixel_valid=de2;
    assign pixel_word=badrow2 ? (x2<11'd32 ? 32'h00ff4040 : 32'h00303030) : vis2 ? word_q : 32'd0;
    assign pixel_hsync=hs2;assign pixel_vsync=vs2;assign pixel_x=x2;assign pixel_y=y2;assign pixel_epoch=ep2;
    wire row_edge=timing_row_edge;
    // 复用与y同拍的派生寄存器，不从y现场再加1、选行、比较。
    // 749帧回绕和10位1023回绕都保留；want_bank在749的750和0之间低位相同。
    // timing_y_plus1/lt719 必须与 timing_y 同拍，光栅发生器与扫描器应配套连接。
    wire [9:0] want_y=timing_y_plus1==10'd750 ? 10'd0 : timing_y_plus1;
    wire want_bank=timing_y_plus1[0];
    wire want_visible=timing_y_plus1==10'd750 || timing_y_plus1==10'd0 || timing_y_lt719;
    wire next_before_want=timing_y_plus1!=10'd750 && next_row<timing_y_plus1;
    wire want_ready=state[want_bank]==READY && !stale[want_bank] &&
        (timing_y_plus1==10'd750 ? scan_y[want_bank]==0 : scan_y[want_bank]==timing_y_plus1);
    // P-PIX：row_desc(next_row+1)。source_y/相位由 nr_d 增量：2(n+1)=2n+2，相位 p→(p+2) mod 3，p>=1 时 source_y+1
    //   （source_y 使用 9 位模值）；n=1023 回绕到 0 单列。preview_* 直接由 next_row+1 计算（只含常数比较与减法）。
    wire [9:0] nr_p1=next_row+1'b1;
    wire [1:0] nr_ph=nr_d[19:18];wire [8:0] nr_src=nr_d[17:9];
    wire [19:0] d_nr1=(next_row==10'h3FF) ? DESC0 :
        {(nr_ph==2'd0) ? 2'd2 : nr_ph-2'd1,(nr_ph!=2'd0) ? nr_src+9'd1 : nr_src,preview_slot(nr_p1),preview_y(nr_p1)};
    // nr_d 的 D：逐句照抄下方 always 中 next_row 的全部赋值（条件与优先级相同），每处换成被赋值的描述符。
    reg [19:0] nr_d_nx;
    always @(*) begin
        nr_d_nx=nr_d;
        if(evt_valid && evt_ready) begin
            // 事件类型为 3 或标签不匹配时，不根据该事件更新行描述符。
            if(kind==3 || !event_match) ;
            else if(kind==0) ;
            else if(!stale[eb] && !ref_switch && scan_y[eb]<next_row)
                nr_d_nx=timing_y_lt720 && scan_y[eb]<timing_y ? timing_y_desc0 : sd[eb];
        end
        if(issue) nr_d_nx=pick_next ? timing_y_desc2 : d_nr1;
        if(row_edge && want_visible && active && !ref_switch && !fault) begin
            if(want_ready) ;
            else if(next_before_want && state[want_bank]==FREE) nr_d_nx=(timing_y_plus1==10'd750) ? DESC0 : timing_y_desc1;
        end
        if(row_edge && (timing_y_lt720 && !timing_y_lt719)) nr_d_nx=DESC0;
        if(ref_valid && (!active || ref_switch)) begin
            if(active && (de || timing_y_lt720)) ;
            else nr_d_nx=DESC0;
        end
    end
    always @(posedge px_clk or posedge rst)
        if(rst) begin nr_d<=DESC0;sd[0]<=DESC0;sd[1]<=DESC0;end
        else begin
            nr_d<=nr_d_nx;
            if(issue) sd[nb]<=d_pick;   // 与 scan_y[nb]<=pick_row 同条件
        end
`ifdef FORMAL
    always @* assert(nr_d==row_desc(next_row) && sd[0]==row_desc(scan_y[0]) && sd[1]==row_desc(scan_y[1]));
    always @* assert(d_pick==row_desc(pick_row) && d_nr1==row_desc(next_row+10'd1));
`endif
    always @(posedge px_clk or posedge rst) begin
        if(rst) begin
            for(k=0;k<2;k=k+1) begin state[k]<=FREE;tag[k]<=0;scan_y[k]<=0;end
            stale<=0;active<=0;have<=0;bank<=0;sticky<=0;bad<=0;under<=0;
            epoch<=0;og<=0;ng<=0;os<=0;ns<=0;tg<=0;layout<=0;boundary<=960;
            next_row<=0;release_hold<=0;release_tag<=0;
        end else begin
            under<=0;
            if(evt_valid && evt_ready) begin
                if(!event_match || kind==3) bad<=1;
                else if(kind==0) begin
                    state[eb]<=READY;
                    if(!stale[eb] && !ref_switch && scan_y[eb]==0) sticky<=1;
                end else begin
                    state[eb]<=FREE;stale[eb]<=0;
                    // Failed bundles are already released by the prefetcher.
                    if(!stale[eb] && !ref_switch && scan_y[eb]<next_row)
                        next_row<=timing_y_lt720 && scan_y[eb]<timing_y ? timing_y : scan_y[eb];
                end
            end
            if(!release_hold && !fault) begin
                if(state[0]==RET || (stale[0] && state[0]==READY)) begin
                    release_hold<=1;release_tag<=tag[0][EPOCH_W+134:16];state[0]<=RET;
                end else if(state[1]==RET || (stale[1] && state[1]==READY)) begin
                    release_hold<=1;release_tag<=tag[1][EPOCH_W+134:16];state[1]<=RET;
                end
            end
            if(rel_valid && rel_ready) begin
                state[release_tag[0]]<=FREE;stale[release_tag[0]]<=0;release_hold<=0;
            end
            if(issue) begin state[nb]<=PEND;tag[nb]<=req_bus;scan_y[nb]<=pick_row;stale[nb]<=0;next_row<=pick_row+1'b1;end
            // At x1282, both pipeline stages have consumed the final active read.
            if(timing_x==1282 && timing_y_lt720 && have) begin state[bank]<=RET;have<=0;end
            if(active && !have && !de && timing_x==0 && timing_y_plus1==10'd1 &&
                state[0]==READY && scan_y[0]==0 && !stale[0] && !fault) begin
                state[0]<=DISP;bank<=0;have<=1;
            end
            if(row_edge && want_visible && active && !ref_switch && !fault) begin
                if(want_ready) begin
                    state[want_bank]<=DISP;bank<=want_bank;have<=1;
                end else begin
                    have<=0;under<=1;
                    if(state[want_bank]==READY) stale[want_bank]<=1;
                    if(next_before_want && state[want_bank]==FREE) next_row<=want_y;
                end
            end
            if(row_edge && (timing_y_lt720 && !timing_y_lt719)) begin next_row<=0;have<=0;
                for(k=0;k<2;k=k+1) if(state[k]==READY || state[k]==PEND) stale[k]<=1;
            end
            // Layout and wipe boundary are frame snapshots, never live pixel controls.
            if(vblank_start) begin layout<=thumb_layout;boundary<=wipe_x>960 ? 11'd960 : wipe_x;end
            if(ref_valid && (!active || ref_switch)) begin
                if(active && (de || timing_y_lt720)) bad<=1;
                else begin
                    active<=1;epoch<=ref_epoch;os<=old_slot;og<=old_gen;ns<=new_slot;ng<=new_gen;tg<=thumb_gens;
                    layout<=thumb_layout;boundary<=wipe_x>960 ? 11'd960 : wipe_x;
                    sticky<=0;have<=0;next_row<=0;
                    for(k=0;k<2;k=k+1) if(state[k]!=FREE) stale[k]<=1;
                end
            end
        end
    end
endmodule
`default_nettype wire
