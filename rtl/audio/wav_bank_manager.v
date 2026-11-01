`timescale 1ns/1ps
`default_nettype none
// sys 域 PCM 双缓冲管理，装载结果须匹配当前作业，完整解析和写入成功后才置 READY。
// PIN 保留读引用并禁止覆盖；停止使用时先进入 DRAINING，等待 DMA 请求结束及 FIFO 清空。
// 仅收到 release 才转为 FREE，允许装入下一段配音，固定延时不能代替完成确认。
// pin/retire/release 由 wav_audio_control 驱动，依次表示保留、开始停止排空和释放缓冲区。
module wav_bank_manager(
    input wire clk,rst,enable,input wire [3:0] loaded,
    input wire current_valid,input wire [1:0] current_slot,
    input wire want_valid,input wire [1:0] want_slot,
    input wire [1:0] ignore_present, // 同ID新epoch只忽略旧读引用，允许另一音频缓冲区准备新副本。
    input wire pin_valid,pin_bank,retire_valid,retire_bank,input wire [1:0] release_bank,
    output wire job_valid,input wire job_ready,output wire job_bank,output wire [1:0] job_slot,
    input wire job_done,job_ok,input wire [17:0] job_words,
    // job_transient=本次失败是卡 I/O 错误（控制模块随后会重新初始化卡再重装）。
    // 为 1 时音频缓冲区照常回 FREE，但不写永久 unavailable；缺文件/坏格式/长度不合法仍为 0，照旧永久回退测试音。
    input wire job_transient,
    output wire [5:0] bank_states,output wire [3:0] bank_slots,
    output wire [35:0] bank_words,output wire [1:0] bank_ready,
    output reg [3:0] unavailable,output reg fault
);
    localparam FREE=0,LOADING=1,READY=2,PLAYING=3,DRAINING=4;
    reg [2:0] s0,s1;
    reg [1:0] id0,id1,current;
    reg [17:0] n0,n1;
    reg busy,active_bank;reg [1:0] active_id;
    function [1:0] following;
        input [1:0] cur;input [3:0] mask;integer j;reg found;reg [1:0] candidate;
        begin following=cur;found=0;
            for(j=1;j<=4;j=j+1) begin candidate=cur+j;
                if(!found && mask[candidate]) begin following=candidate;found=1;end
            end
        end
    endfunction
    wire [1:0] effective_current=current_valid ? current_slot : current;
    wire current_present=(s0!=FREE && id0==effective_current) || (s1!=FREE && id1==effective_current);
    wire [1:0] desired=want_valid ? want_slot :
        (!current_present && !unavailable[effective_current] && loaded[effective_current]) ? effective_current : following(effective_current,loaded);
    wire present=(s0!=FREE && id0==desired && !ignore_present[0]) || (s1!=FREE && id1==desired && !ignore_present[1]);
    // 只有 READY、未被读取且不属于当前图片的缓冲区，才可用于装载其他配音；PLAYING 和 DRAINING 状态禁止覆盖。
    wire free0=(s0==FREE || (s0==READY && id0!=effective_current)) && !(pin_valid && !pin_bank);
    wire free1=(s1==FREE || (s1==READY && id1!=effective_current)) && !(pin_valid && pin_bank);
    assign job_valid=enable && !rst && !busy && loaded[desired] && !unavailable[desired] && !present && (free0 || free1);
    assign job_bank=!free0;
    assign job_slot=desired;
    assign bank_states={s1,s0};assign bank_slots={id1,id0};assign bank_words={n1,n0};
    assign bank_ready={s1==READY || s1==PLAYING,s0==READY || s0==PLAYING};
    // PIN 与新装载同时发生时优先保留原缓冲区；配音长度仍须单独检查。
    wire job_accepted_ok=job_ok && job_words>=48000 && job_words<=240000;
    always @(posedge clk) begin
        if(rst) begin
            s0<=FREE;s1<=FREE;id0<=0;id1<=0;n0<=0;n1<=0;current<=0;
            busy<=0;active_bank<=0;active_id<=0;unavailable<=0;fault<=0;
        end else begin
            if(current_valid) current<=current_slot;
            if(release_bank[0]) begin if(s0==DRAINING) begin s0<=FREE;n0<=0;end else fault<=1;end
            if(release_bank[1]) begin if(s1==DRAINING) begin s1<=FREE;n1<=0;end else fault<=1;end
            if(retire_valid) begin
                if(retire_bank) begin if(s1==PLAYING) s1<=DRAINING;else fault<=1;end
                else begin if(s0==PLAYING) s0<=DRAINING;else fault<=1;end
            end
            if(pin_valid) begin
                if(pin_bank) begin if(s1==READY) s1<=PLAYING;else fault<=1;end
                else begin if(s0==READY) s0<=PLAYING;else fault<=1;end
            end
            if(job_valid && job_ready) begin
                busy<=1;active_bank<=job_bank;active_id<=job_slot;
                if(job_bank) begin s1<=LOADING;id1<=job_slot;n1<=0;end
                else begin s0<=LOADING;id0<=job_slot;n0<=0;end
            end
            if(job_done) begin
                if(!busy) fault<=1;
                else begin
                    busy<=0;
                    if(!job_accepted_ok && !job_transient) unavailable[active_id]<=1;
                    if(active_bank) begin s1<=job_accepted_ok ? READY : FREE;n1<=job_accepted_ok ? job_words : 0;end
                    else begin s0<=job_accepted_ok ? READY : FREE;n0<=job_accepted_ok ? job_words : 0;end
                end
            end
        end
    end
`ifdef FORMAL
    reg f_past_valid=0;
    always @(posedge clk) f_past_valid<=1;
    always @* if(!rst) begin
        assert(!job_valid || (job_bank ? (s1==FREE || s1==READY) : (s0==FREE || s0==READY)));
        assert(!busy || (active_bank ? s1==LOADING : s0==LOADING));
        assert(!bank_ready[0] || (n0>=48000 && n0<=240000));
        assert(!bank_ready[1] || (n1>=48000 && n1<=240000));
    end
    // 音频缓冲区读引用须依次经过 retire 和 release 才能释放。
    // 即使作业、ACK、PIN 和释放同拍到达，也依据已锁存的引用状态决定释放条件。
    always @(posedge clk) if(f_past_valid && !rst && !$past(rst)) begin
        if($past(s0)==PLAYING && !$past(retire_valid && !retire_bank)) assert(s0==PLAYING);
        if($past(s1)==PLAYING && !$past(retire_valid && retire_bank)) assert(s1==PLAYING);
        if($past(s0)==DRAINING && !$past(release_bank[0])) assert(s0==DRAINING);
        if($past(s1)==DRAINING && !$past(release_bank[1])) assert(s1==DRAINING);
    end
`endif
endmodule
`default_nettype wire
