`timescale 1ns/1ps
`default_nettype none
// sys 域播放控制模块，将按键及定时事件转换为换图描述符。
// 首图成功响应前，仅记录装载提交的槽号和 gen；首图响应须与 loaded、gen_valid、epoch 和 slot 匹配。
// 完成启动交接后才发送换图请求；一次只允许一个在途请求，期间额外 NEXT 事件合并计数，不逐次排队。
// 关闭自动模式只清除尚未发送的自动请求，已经置 valid 的请求保持到接受或超时。
// 成功响应后更新当前显示引用；请求被拒绝时保留当前引用，重新计满轮播周期再发下一次请求。
// period_s 范围为 1..60 秒，超出时限幅；启动时锁存配置，运行中仅在 period_set 时更新。
// MS_CYCLES 定义 1ms 的时钟拍数，整机为 50000；LEAD_MS 定义换图请求的提前毫秒数，整机为 8。
// 握手和故障输出使用 rst 隔离，业务计数由 logic_rst 同步复位；数据总线不复位。
module play_ctrl #(
    parameter integer EPOCH_W=32,
    parameter integer MS_CYCLES=50000,
    parameter integer PC_BUDGET=50000000,
    parameter integer LEAD_MS=8
)(
    input wire clk,rst,logic_rst,
    input wire ev_next,ev_auto_toggle,auto_inhibit,
    input wire [5:0] period_s,input wire period_set,auto_init,
    input wire [3:0] boot_loaded,slot_reserved,
    input wire commit_valid,commit_ready,
    input wire [1:0] commit_slot,input wire [31:0] commit_gen,
    input wire boot_ack_ready,ack_valid,input wire [EPOCH_W+20:0] ack_bus,
    input wire desc_ready,
    output wire ms_tick,
    output reg pc_desc_valid,pc_ack_ready,handoff,pc_fault,
    output reg [EPOCH_W+223:0] pc_desc_bus,
    output reg auto_on,
    output reg [1:0] cur_slot,
    output reg [EPOCH_W-1:0] cur_epoch,
    output reg [31:0] cur_gen,
    output reg [15:0] last_frame,interval_frames,
    output reg interval_err,
    output reg [15:0] merged_cnt,reject_cnt,no_target_cnt,
    output reg [7:0] pc_code,
    // OSD 只读观察口：当前周期剩余毫秒=period_ms-elapsed_ms，纯组合，不参与任何业务判断；
    // 产品 OSD_EN=0 时无负载被综合删除。
    output wire [15:0] remain_ms
);
    function integer bits_for;
        input integer n;integer v;
        begin bits_for=1;v=n-1;while(v>1)begin bits_for=bits_for+1;v=v>>1;end end
    endfunction
    function [5:0] clamp_period;
        input [5:0] value;
        begin clamp_period=value==0 ? 6'd1 : value>60 ? 6'd60 : value;end
    endfunction
    localparam integer TICK_W=bits_for(MS_CYCLES),AGE_W=bits_for(PC_BUDGET);
    localparam [TICK_W-1:0] TICK_LAST=MS_CYCLES[TICK_W-1:0]-1'b1;
    localparam [AGE_W-1:0] AGE_LAST=PC_BUDGET[AGE_W-1:0]-1'b1;
    localparam [2:0] OBSERVE=0,IDLE=1,ISSUE=2,WAIT_ACK=3,FAULT=4;
    reg [2:0] st;
    reg [TICK_W-1:0] ms_div;
    reg [AGE_W-1:0] age;
    reg [5:0] period_q;
    reg [15:0] elapsed_ms,issued_frames,pending_frames;
    reg pending,pending_auto,auto_fired,issued_auto;
    // 这里只做诊断，不参与选槽、描述符握手或AUTO到期。sys域没有当前像素帧号，
    // ack_bus只在响应有效时才有意义；不能把旧ACK当作K2时刻的当前帧。
    // 1ms tick按3/50累加得到标称60Hz参考，无新时钟/CDC。phase保留余数，
    // 不用16ms或17ms整除造成长期漂移；帧计数和真实ACK一样按16位模回绕。
    reg [15:0] diagnostic_frame,auto_ref_frame,issued_ref_frame;
    reg [5:0] diagnostic_phase;
    reg auto_ref_from_enable,issued_ref_from_enable;
    reg [3:0] gen_valid;
    reg [1:0] first_slot;
    reg [31:0] gen0,gen1,gen2,gen3;
    wire [1:0] slot1=cur_slot+2'd1,slot2=cur_slot+2'd2,slot3=cur_slot+2'd3;
    wire [3:0] eligible=boot_loaded & gen_valid & ~slot_reserved & ~(4'b0001<<cur_slot);
    wire [1:0] target=eligible[slot1] ? slot1 : eligible[slot2] ? slot2 : slot3;
    wire [31:0] target_gen=target==0 ? gen0 : target==1 ? gen1 : target==2 ? gen2 : gen3;
    wire [31:0] ack_gen=ack_bus[1:0]==0 ? gen0 : ack_bus[1:0]==1 ? gen1 : ack_bus[1:0]==2 ? gen2 : gen3;
    wire [15:0] period_ms={10'd0,period_q}*16'd1000;
    wire [15:0] frame_goal={10'd0,period_q}*16'd60;
    wire [15:0] fire_ms=period_ms-LEAD_MS[15:0];
    // OSD 观察口用 1000=1024-16-8 计算周期毫秒数，避免乘减表达式占用 DSP。
    // period_q 最大 63，结果不超过 63000，可由 16 位完整表示；此值仅用于显示剩余时间。
    wire [15:0] period_ext={10'd0,period_q};
    wire [15:0] period_ms_osd=(period_ext<<10)-(period_ext<<4)-(period_ext<<3);
    assign remain_ms=period_ms_osd-elapsed_ms;
    wire cancel_auto=auto_inhibit || (ev_auto_toggle && auto_on);
    wire enable_auto=ev_auto_toggle && !auto_on && !auto_inhibit;
    wire auto_due=auto_on && !auto_fired && ms_tick && elapsed_ms==fire_ms-16'd1;
    wire commit_fire=commit_valid && commit_ready;
    wire boot_take=!handoff && boot_ack_ready && ack_valid && ack_bus[EPOCH_W+20:EPOCH_W+18]==0 &&
        ack_bus[EPOCH_W+17:18]=={{(EPOCH_W-1){1'b0}},1'b1} && ack_bus[1:0]==first_slot &&
        gen_valid[ack_bus[1:0]] && boot_loaded[ack_bus[1:0]];
    wire consume_pending=handoff && st==IDLE && pending && !(cancel_auto && pending_auto);
    wire launch=consume_pending && (|eligible) && commit_ready;
    wire ack_fire=pc_ack_ready && ack_valid;
    wire ack_success=ack_bus[EPOCH_W+20:EPOCH_W+18]==0;
    // 显式截到EPOCH_W位，255→0时描述符与ACK期望必须使用同一个模加结果。
    wire [EPOCH_W-1:0] next_epoch=cur_epoch+1'b1;
    wire ack_matches=ack_bus[EPOCH_W+17:18]==next_epoch && ack_bus[1:0]==pc_desc_bus[157:156];
    wire timeout_now=(st==ISSUE || st==WAIT_ACK) && age==AGE_LAST && !ack_fire;
    // 首周期的参考在K2开启沿冻结；发布时另存一份，避免在途再次切模式移动旧请求起点。
    // 参考计数在每个成功ACK校准成真实帧号，K2时锁住估算当前帧。首个ACK仍
    // 使用实际像素域帧号作模减，不能只按sys等待时间判定从而漏掉错误ACK帧号。
    // 未跨域采集K2瞬间像素帧相位，估计量化/ACK传输误差由原±1帧门槛约束；
    // 第一次之后继续直接从上次真实成功ACK作模减。
    // last_frame始终保留真实成功ACK，不能拿sys估算值覆盖显示协议状态。
    wire [6:0] diagnostic_phase_next={1'b0,diagnostic_phase}+7'd3;
    wire diagnostic_rebase=boot_take || (ack_fire && ack_success && ack_matches);
    wire [15:0] enable_frame_delta=ack_bus[17:2]-issued_ref_frame;
    wire [15:0] ack_frame_delta=ack_bus[17:2]-last_frame;
    wire [15:0] frame_delta=issued_auto && issued_ref_from_enable ? enable_frame_delta : ack_frame_delta;
    wire [15:0] frame_error=frame_delta>issued_frames ? frame_delta-issued_frames : issued_frames-frame_delta;
    assign ms_tick=!logic_rst && ms_div==TICK_LAST;

    // 有效/就绪直接出寄存器。FAULT期间二者都为0，只有共同复位才重新接管。
    always @(posedge clk or posedge rst) begin
        if(rst) begin st<=OBSERVE;handoff<=0;pc_desc_valid<=0;pc_ack_ready<=0;pc_fault<=0;end
        else if(timeout_now || (ack_fire && ack_success && !ack_matches)) begin
            st<=FAULT;pc_desc_valid<=0;pc_ack_ready<=0;pc_fault<=1;
        end else case(st)
            OBSERVE:if(boot_take) begin st<=IDLE;handoff<=1;end
            IDLE:if(launch) begin st<=ISSUE;pc_desc_valid<=1;end
            ISSUE:if(desc_ready) begin st<=WAIT_ACK;pc_desc_valid<=0;pc_ack_ready<=1;end
            WAIT_ACK:if(ack_fire) begin st<=IDLE;pc_ack_ready<=0;end
            default:begin pc_desc_valid<=0;pc_ack_ready<=0;end
        endcase
    end
    // 大数据寄存器不需要异步SR。gen_valid/握手状态禁止使用未写gen或描述符内容。
    always @(posedge clk) begin
        if(commit_fire) begin
            case(commit_slot)
                0:gen0<=commit_gen;1:gen1<=commit_gen;2:gen2<=commit_gen;3:gen3<=commit_gen;
            endcase
        end
        if(launch) pc_desc_bus<={next_epoch,cur_gen,target_gen,cur_slot,target,32'd0,124'd0};
    end
    always @(posedge clk) begin
        if(logic_rst) begin
            ms_div<=0;age<=0;period_q<=clamp_period(period_s);elapsed_ms<=0;pending_frames<=0;
            pending<=0;pending_auto<=0;auto_fired<=0;issued_auto<=0;issued_frames<=0;
            gen_valid<=0;first_slot<=0;auto_on<=auto_init;
            cur_slot<=0;cur_epoch<=0;cur_gen<=0;last_frame<=0;interval_frames<=0;interval_err<=0;
            merged_cnt<=0;reject_cnt<=0;no_target_cnt<=0;pc_code<=0;
        end else begin
            if(ms_tick) ms_div<=0;else ms_div<=ms_div+1'b1;
            if(st==ISSUE || st==WAIT_ACK) begin if(age!=AGE_LAST) age<=age+1'b1;end
            else age<=0;
            if(commit_fire) begin
                gen_valid[commit_slot]<=1;
                if(gen_valid==0) first_slot<=commit_slot;
            end
            if(!handoff) begin
                auto_on<=auto_init;period_q<=clamp_period(period_s);elapsed_ms<=0;
                pending<=0;pending_auto<=0;auto_fired<=0;
                if(boot_take) begin
                    cur_slot<=ack_bus[1:0];cur_gen<=ack_gen;cur_epoch<=ack_bus[EPOCH_W+17:18];last_frame<=ack_bus[17:2];
                end
            end else if(st!=FAULT) begin
                if(auto_inhibit) auto_on<=0;
                else if(ev_auto_toggle) auto_on<=!auto_on;
                if(period_set) period_q<=clamp_period(period_s);
                if(ms_tick && auto_on && !auto_fired && !auto_due) elapsed_ms<=elapsed_ms+16'd1;
                if(cancel_auto || ev_auto_toggle || period_set) begin elapsed_ms<=0;auto_fired<=0;end
                // 同拍优先级：OFF/inhibit > NEXT > 到时。已有手动pending不被AUTO OFF清掉。
                if(cancel_auto) begin if(pending_auto) begin pending<=0;pending_auto<=0;end end
                else if(ev_next || auto_due) begin
                    if(st==IDLE && !pending) begin pending<=1;pending_auto<=!ev_next;pending_frames<=frame_goal;end
                    else if(merged_cnt!=16'hffff) merged_cnt<=merged_cnt+16'd1;
                    if(ev_next && pending) pending_auto<=0;
                    if(auto_due) auto_fired<=1;
                end
                if(consume_pending && (!(|eligible) || commit_ready)) begin
                    pending<=0;pending_auto<=0;
                    if(!(|eligible) && no_target_cnt!=16'hffff) no_target_cnt<=no_target_cnt+16'd1;
                    // 暂时只有一张图不等于永久停播；下一完整周期形成一个新的自动请求。
                    if(!(|eligible) && pending_auto) begin elapsed_ms<=0;auto_fired<=0;end
                end
                // 表mailbox背压期间的PERIOD修改只影响后续周期，不能改写已排队请求的诊断目标。
                if(launch) begin
                    issued_auto<=pending_auto;issued_frames<=pending_frames;age<=0;
                end
                if(ack_fire) begin
                    elapsed_ms<=0;auto_fired<=0;
                    if(!ack_success) begin if(reject_cnt!=16'hffff) reject_cnt<=reject_cnt+16'd1;end
                    else if(ack_matches) begin
                        cur_slot<=ack_bus[1:0];cur_gen<=pc_desc_bus[191:160];cur_epoch<=ack_bus[EPOCH_W+17:18];
                        // 公开interval_frames仍保留两次真实ACK之间的物理帧差，兼容
                        // 原诊断接口；首周期可能包含手动停留，不能拿它直接判AUTO周期。
                        last_frame<=ack_bus[17:2];interval_frames<=ack_frame_delta;
                        interval_err<=issued_auto && frame_error>16'd1;
                    end
                end
                if(timeout_now) pc_code<=8'h01;
                else if(ack_fire && ack_success && !ack_matches) pc_code<=8'h02;
            end else begin pending<=0;pending_auto<=0;end
        end
    end
    // 诊断寄存器独立成一个同步过程，与播放状态和业务计数独立更新。
    // 数据寄存器的使能与事件标志分别计算，在同一采样沿更新。
    always @(posedge clk) begin
        if(logic_rst) begin
            diagnostic_frame<=0;diagnostic_phase<=0;auto_ref_frame<=0;
            issued_ref_frame<=0;auto_ref_from_enable<=0;issued_ref_from_enable<=0;
        end else begin
            if(ms_tick) begin
                if(diagnostic_phase_next>=7'd50) begin
                    diagnostic_phase<=diagnostic_phase_next[5:0]-6'd50;
                    diagnostic_frame<=diagnostic_frame+16'd1;
                end else diagnostic_phase<=diagnostic_phase_next[5:0];
            end
            // 成功 ACK 携带实际提交帧号；收到后重新校准诊断计数并清除毫秒余数。
            // ACK 存在跨域传输延迟，诊断相位仅供观察，不用于驱动像素时序。
            if(diagnostic_rebase) begin diagnostic_frame<=ack_bus[17:2];diagnostic_phase<=0;end
            if(handoff && st!=FAULT) begin
                if(launch) begin
                    issued_ref_from_enable<=auto_ref_from_enable;
                    issued_ref_frame<=auto_ref_frame;
                end
                if(ack_fire && ack_success && ack_matches) auto_ref_from_enable<=0;
                // K2 OFF→ON同时重置参考和 elapsed_ms；同拍启用优先于成功ACK。
                // 已发布的issued_*不受这次重开影响，不能把旧请求挪到新起点。
                if(enable_auto) begin
                    // 开启与ACK同拍时直接用该真实ACK；否则用已校准的自由运行参考。
                    auto_ref_frame<=diagnostic_rebase ? ack_bus[17:2] : diagnostic_frame;
                    auto_ref_from_enable<=1;
                end
            end
        end
    end
endmodule
`default_nettype wire
