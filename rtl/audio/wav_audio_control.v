`timescale 1ns/1ps
`default_nettype none
// sys 域音频缓冲区控制：准备新配音，并在换图后停止和释放旧配音缓冲区。
// PIN 保留缓冲区，防止装载器覆盖；START 启动预取，FIFO 缓存至少 512 个样本后置 ARMED。
// 新显示描述符确认提交后，旧缓冲区进入停止流程，等待其 DMA 在途请求结束并清空 FIFO。
// 收到缓冲区编号和 token 均匹配的 CANCEL 完成响应后，才发出 release 允许重新装载。
// 迟到或不匹配的响应不释放任何缓冲区；控制消息和响应通过外层文件 mailbox 传递。
// 新配音的缓冲区编号及样本数随显示描述符提交，与画面在同一边界切换。
module wav_audio_control #(
    parameter ENABLE=1,parameter integer EPOCH_W=8,parameter [31:0] WAIT_CYCLES=35000000
)(
    input wire clk,rst,input wire [3:0] loaded,
    // 音频缓冲区控制模块sys域粘滞fault：置位后不再arm新音频，描述符走known_bad回退测试音；
    // 已准备的缓冲区仍按停止流程等待 DMA 和 FIFO 排空；mem 域错误通过响应传递。
    input wire bank_fault,
    input wire [5:0] bank_states,input wire [3:0] bank_slots,input wire [35:0] bank_words,
    input wire [3:0] unavailable,
    output wire want_valid,output wire [1:0] want_slot,output wire [1:0] ignore_present,
    output wire pin_valid,pin_bank,retire_valid,retire_bank,output reg [1:0] release_bank,
    output wire cmd_valid,input wire cmd_ready,output reg cmd_cancel,cmd_bank,
    output reg [17:0] cmd_words,output reg [15:0] cmd_token,
    input wire rsp_valid,input wire [1:0] rsp_status,input wire rsp_bank,input wire [15:0] rsp_token,
    input wire in_valid,output wire in_ready,input wire [EPOCH_W+223:0] in_bus,
    output wire out_valid,input wire out_ready,output wire [EPOCH_W+223:0] out_bus,
    input wire ack_fire,input wire [EPOCH_W+20:0] ack_bus,
    output wire [31:0] prefetch_late,output wire [15:0] stale_events,output reg fault
);
    localparam READY=2,PLAYING=3;
    reg [1:0] owned,armed,retiring,played_bank,retire_needed;
    reg [15:0] token0,token1,next_token;
    reg current_valid,active_valid,active_bank;
    reg [1:0] current_slot;
    reg pending,sent_audio,sent_bank;reg [1:0] sent_slot;reg [EPOCH_W-1:0] sent_epoch;
    reg [31:0] wait_count;
    // late_count 统计预取等待达到时限的次数，stale_count 统计不匹配的响应。
    // 统计输出未连接时，对应计数逻辑可由综合工具删除。
    reg [31:0] late_count;
    reg [15:0] stale_count;
    assign prefetch_late=late_count;assign stale_events=stale_count;
    function [1:0] following;input [1:0] slot;input [3:0] mask;
        integer j;reg found;reg [1:0] candidate;
        begin following=slot;found=0;
            for(j=1;j<=4;j=j+1) begin candidate=slot+j;
                if(!found && mask[candidate]) begin following=candidate;found=1;end
            end
        end
    endfunction
    wire [1:0] target=in_bus[157:156];
    wire [1:0] desired=pending ? sent_slot : in_valid ? target : following(current_slot,loaded);
    assign want_valid=ENABLE && !rst;assign want_slot=desired;
    // 单图循环/显式同ID换epoch也需要新音频缓冲区的第0样本，不能重用已经消费的FIFO。
    // 仅在同ID新描述符准备期间忽略旧播放音频缓冲区的“已存在”，允许另一音频缓冲区装同ID副本。
    assign ignore_present=ENABLE && in_valid && !pending && active_valid && current_slot==target ?
        (active_bank ? 2'b10 : 2'b01) : 2'b00;
    wire need0=bank_slots[1:0]==desired || (current_valid && bank_slots[1:0]==current_slot);
    wire need1=bank_slots[3:2]==desired || (current_valid && bank_slots[3:2]==current_slot);
    wire can_pin0=bank_states[2:0]==READY && !owned[0] && need0 && bank_words[17:0]>=48000 && bank_words[17:0]<=240000;
    wire can_pin1=bank_states[5:3]==READY && !owned[1] && need1 && bank_words[35:18]>=48000 && bank_words[35:18]<=240000;
    assign pin_valid=ENABLE && !rst && !fault && (can_pin0 || can_pin1);
    assign pin_bank=!can_pin0;
    wire retire0=retire_needed[0] && owned[0] && !retiring[0] && bank_states[2:0]==PLAYING;
    wire retire1=retire_needed[1] && owned[1] && !retiring[1] && bank_states[5:3]==PLAYING;
    assign retire_valid=ENABLE && !rst && (retire0 || retire1);
    assign retire_bank=!retire0;
    wire fresh0=owned[0] && armed[0] && !played_bank[0] && !retiring[0] && !retire_needed[0] && bank_slots[1:0]==target;
    wire fresh1=owned[1] && armed[1] && !played_bank[1] && !retiring[1] && !retire_needed[1] && bank_slots[3:2]==target;
    wire prepared=(fresh0 || fresh1) && !fault;
    wire prepared_bank=!fresh0;
    wire known_bad=unavailable[target] || fault || bank_fault;
    wire timed_out=WAIT_CYCLES==0 || wait_count>=WAIT_CYCLES;
    wire permit=!ENABLE || prepared || known_bad || timed_out;
    wire use_audio=ENABLE && prepared && !known_bad;
    wire [19:0] metadata=use_audio ? {1'b1,prepared_bank,prepared_bank ? bank_words[35:18] : bank_words[17:0]} : 20'd0;
    assign out_bus=ENABLE ? {in_bus[EPOCH_W+223:20],metadata} : in_bus;
    assign out_valid=in_valid && !pending && permit && !rst;
    assign in_ready=out_ready && !pending && permit && !rst;
    wire desc_fire=out_valid && out_ready;
    localparam Q_IDLE=0,Q_SEND=1,Q_WAIT=2;
    reg [1:0] command_state;
    assign cmd_valid=ENABLE && !rst && command_state==Q_SEND;
    wire choose_cancel=retiring[0] || retiring[1];
    wire choose_bank=choose_cancel ? !retiring[0] : !(owned[0] && !armed[0] && !played_bank[0] && !retiring[0] && !retire_needed[0]);
    wire need_start=(owned[0] && !armed[0] && !played_bank[0] && !retiring[0] && !retire_needed[0]) ||
                    (owned[1] && !armed[1] && !played_bank[1] && !retiring[1] && !retire_needed[1]);
    wire response_match=command_state==Q_WAIT && rsp_bank==cmd_bank && rsp_token==cmd_token;
    wire ack_match=pending && ack_bus[1:0]==sent_slot &&
        (ack_bus[EPOCH_W+20:EPOCH_W+18]!=0 || ack_bus[EPOCH_W+17:18]==sent_epoch);
    always @(posedge clk) begin
        release_bank<=0;
        if(rst) begin
            owned<=0;armed<=0;retiring<=0;played_bank<=0;retire_needed<=0;
            token0<=0;token1<=0;next_token<=1;current_valid<=0;current_slot<=0;active_valid<=0;active_bank<=0;
            pending<=0;sent_audio<=0;sent_bank<=0;sent_slot<=0;sent_epoch<=0;
            wait_count<=0;late_count<=0;stale_count<=0;fault<=0;
            command_state<=Q_IDLE;cmd_cancel<=0;cmd_bank<=0;cmd_words<=0;cmd_token<=0;
        end else if(ENABLE) begin
            if(!in_valid || pending || desc_fire) wait_count<=0;
            else if(!permit && wait_count<WAIT_CYCLES) begin
                wait_count<=wait_count+1'b1;
                if(wait_count==WAIT_CYCLES-1) late_count<=late_count+1'b1;
            end
            if(pin_valid) begin
                owned[pin_bank]<=1;armed[pin_bank]<=0;played_bank[pin_bank]<=0;
                if(pin_bank) token1<=next_token;else token0<=next_token;
                next_token<=next_token==16'hffff ? 16'd1 : next_token+1'b1;
            end
            // 停止预取当前画面和待切换画面都不再需要的配音，随后按停止流程释放缓冲区。
            // 本模块不取消正在进行的文件装载；装载成功后，缓冲区进入 READY。
            // READY 缓冲区若不属于当前画面，且未被预取或播放占用，可用于装载其他图片的配音。
            if(owned[0] && !need0 && !(pending && sent_audio && !sent_bank)) retire_needed[0]<=1;
            if(owned[1] && !need1 && !(pending && sent_audio && sent_bank)) retire_needed[1]<=1;
            if(retire_valid) begin retiring[retire_bank]<=1;retire_needed[retire_bank]<=0;end
            if(desc_fire) begin
                pending<=1;sent_audio<=use_audio;sent_bank<=prepared_bank;
                sent_slot<=target;sent_epoch<=in_bus[EPOCH_W+223:224];
            end
            if(ack_fire) begin
                if(!ack_match) stale_count<=stale_count+1'b1;
                else begin
                    pending<=0;
                    if(ack_bus[EPOCH_W+20:EPOCH_W+18]==0 && ack_bus[1:0]==sent_slot) begin
                        current_valid<=1;current_slot<=sent_slot;
                        if(active_valid && (!sent_audio || sent_bank!=active_bank)) retire_needed[active_bank]<=1;
                        active_valid<=sent_audio;active_bank<=sent_bank;
                        if(sent_audio) played_bank[sent_bank]<=1;
                    end
                end
            end
            case(command_state)
                Q_IDLE:if(choose_cancel || need_start) begin
                    cmd_cancel<=choose_cancel;cmd_bank<=choose_bank;
                    cmd_words<=choose_bank ? bank_words[35:18] : bank_words[17:0];
                    cmd_token<=choose_bank ? token1 : token0;command_state<=Q_SEND;
                end
                Q_SEND:if(cmd_valid && cmd_ready) command_state<=Q_WAIT;
                Q_WAIT:if(rsp_valid && response_match) begin
                    command_state<=Q_IDLE;
                    if(cmd_cancel && rsp_status==1) begin
                        release_bank[cmd_bank]<=1;owned[cmd_bank]<=0;armed[cmd_bank]<=0;
                        played_bank[cmd_bank]<=0;retiring[cmd_bank]<=0;retire_needed[cmd_bank]<=0;
                    end else if(!cmd_cancel && rsp_status==0) begin
                        if(!retiring[cmd_bank] && !retire_needed[cmd_bank]) armed[cmd_bank]<=1;
                    end else begin fault<=1;retire_needed[cmd_bank]<=1;end
                end
                default:begin command_state<=Q_IDLE;fault<=1;end
            endcase
            if(rsp_valid && !response_match) stale_count<=stale_count+1'b1;
        end
    end
endmodule
`default_nettype wire
