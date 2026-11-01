`timescale 1ns/1ps
`default_nettype none

// 连续extent预读：两逻辑512B扇区缓冲区共用1024x8 RAM，发布前等待完整backend成功结果。
// backend必须是独占、单在途、带键的真实sd_spi_media_port；本模块不执行卡初始化，也不修改卡可用状态。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_sector_prefetch #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    // TIME_W 为内部预算、已用和剩余周期计数的位宽，默认 32 位；
    // 产品由 tf_bmp_boot 传入 counter_width(SYS_BUDGET+1)=26，该宽度按构造恰好容纳预算值本身。
    // 所有比较/减法都在 TIME_W 位内完成；截止判定仍是逐周期精确的，预算值不变。
    parameter integer KEY_W=32,parameter integer TIME_W=32,parameter USE_EG_BRAM=1,parameter [KEY_W-1:0] FIRST_ATTEMPT=1)(
    input wire clk,arst,
    input wire req_valid,output wire req_ready,
    input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_lba,req_budget_cycles,
    input wire [3:0] req_sectors,
    input wire stop_req,input wire [KEY_W-1:0] stop_session,stop_seq,
    output wire stop_ack,output wire [KEY_W-1:0] stop_ack_session,stop_ack_seq,
    output wire busy,
    output wire data_valid,input wire data_ready,output wire [7:0] data,
    output wire [8:0] data_index,output wire [2:0] data_sector,
    output wire data_sector_last,data_last,output wire [KEY_W-1:0] data_session,data_seq,
    output wire result_valid,input wire result_ready,
    output wire [KEY_W-1:0] result_session,result_seq, output wire [31:0] result_elapsed,
    output wire [3:0] result_code,output wire [7:0] result_detail,
    output wire [12:0] result_delivered,
    output wire [5:0] result_attempts,output wire [4:0] result_retries,
    output wire result_card_fault,
    output wire b_req_valid,input wire b_req_ready,
    output wire [KEY_W-1:0] b_req_session,b_req_seq, output wire [31:0] b_req_lba,
    output wire b_cancel_valid,input wire b_cancel_ready,
    output wire [KEY_W-1:0] b_cancel_session,b_cancel_seq,
    input wire b_data_valid,output wire b_data_ready,input wire [7:0] b_data,
    input wire [8:0] b_data_index,input wire b_data_last,
    input wire [KEY_W-1:0] b_data_session,b_data_seq,
    input wire b_result_valid,output wire b_result_ready,
    input wire [KEY_W-1:0] b_result_session,b_result_seq,
    input wire [3:0] b_result_code,input wire [7:0] b_result_detail,
    input wire [9:0] b_result_delivered,input wire b_result_card_fault
);
    wire rst_logic;
    localparam IDLE=0,RUN=1,COMPLETE=2;
    localparam F_IDLE=0,F_ISSUE=1,F_RECV=2,F_RESULT=3;
    localparam C_IDLE=0,C_READ=1,C_OUTPUT=2;
    localparam OK=0,CRC_ERROR=7,CANCELED=8,DESCRIPTOR=9,DEADLINE=10,PROTOCOL=11,GENERATION=12;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    reg [1:0] state,fetch_state,consume_state;
    reg [KEY_W-1:0] session_key,seq_key,attempt_key,next_attempt; reg [31:0] base_lba; reg [TIME_W-1:0] budget,remaining_budget;
    reg [3:0] sectors,next_sector,status;
    reg [2:0] sector0,sector1;
    reg [1:0] full,retry_count;
    reg fill_bank,read_bank,key_valid,stopped,aborting,backend_active,cancel_sent,exhausted,card_fault;
    reg [9:0] received;
    reg [8:0] read_index;
    reg [12:0] delivered;
    reg [5:0] attempts;
    reg [4:0] retries;
    reg [7:0] detail;
    wire stop_matches=key_valid && stop_session==session_key && stop_seq==seq_key;
    wire stop_now=enabled && stop_req && stop_matches && !stopped;
    // 完整32位模倒数：remaining_budget恒为budget-elapsed，不饱和、不缩位。
    // 原elapsed==budget-1恰等于remaining_budget==1，截止/重试/取消沿都不变。
    // 对外elapsed仍给原逐拍值；父级不接统计输出时，budget及输出减法可裁除。
    wire [TIME_W-1:0] elapsed=budget-remaining_budget;
    wire deadline_now=enabled && state==RUN && !aborting && remaining_budget==1;
    wire abort_now=stop_now || deadline_now;
    assign req_ready=enabled && state==IDLE && !stop_req;
    assign busy=state!=IDLE;
    // 仅确认本extent生产者的实际停流；不包含FAT其他客户端/初始化/SDRAM与FIFO清空。
    assign stop_ack=enabled && stop_req && stop_matches && stopped &&
                    (state==COMPLETE || state==IDLE) && !backend_active && fetch_state==F_IDLE && full==0;
    assign stop_ack_session=session_key;assign stop_ack_seq=seq_key;
    assign b_req_valid=enabled && state==RUN && fetch_state==F_ISSUE && !aborting && !abort_now;
    assign b_req_session=session_key;assign b_req_seq=next_attempt;
    assign b_req_lba=base_lba+{28'd0,next_sector};
    wire b_req_fire=b_req_valid && b_req_ready;
    // 每次实际卡请求分配新尝试编号（KEY_W位，产品8位回绕跳0），不能把旧尝试返回解释为重试成功。
    assign b_cancel_valid=enabled && backend_active && aborting && !cancel_sent;
    assign b_cancel_session=session_key;assign b_cancel_seq=attempt_key;
    assign b_data_ready=enabled && backend_active;
    assign b_result_ready=enabled && backend_active;
    wire b_data_fire=b_data_valid && b_data_ready;
    wire b_result_fire=b_result_valid && b_result_ready;
    wire result_matches=b_result_session==session_key && b_result_seq==attempt_key;
    wire data_bad=b_data_fire && !aborting &&
                  (fetch_state!=F_RECV || received>=512 || b_data_index!=received[8:0] ||
                   b_data_last!=(received==511) || b_data_session!=session_key || b_data_seq!=attempt_key);
    wire success_bad=b_result_code==OK && (fetch_state!=F_RESULT || received!=512 ||
                                          b_result_delivered!=512 || b_result_card_fault);
    wire can_retry=b_result_code==CRC_ERROR && !b_result_card_fault && received==0 &&
                   b_result_delivered==0 && retry_count<2 && !exhausted;
    wire result_bad=b_result_fire && !aborting &&
                    (!result_matches || success_bad || (b_result_code==CRC_ERROR &&
                     (received!=0 || b_result_delivered!=0)));
    wire fatal_result=b_result_fire && !aborting && result_matches &&
                       (success_bad || (b_result_code!=OK && !can_retry));
    wire protocol_now=data_bad || result_bad;
    wire isolate=aborting || abort_now || protocol_now || fatal_result;
    assign data_valid=enabled && state==RUN && consume_state==C_OUTPUT && !isolate;
    assign data_index=read_index;
    assign data_sector=read_bank ? sector1 : sector0;
    assign data_sector_last=read_index==511;
    assign data_last=data_sector_last && ({1'b0,data_sector}+4'd1)==sectors;
    assign data_session=session_key;assign data_seq=seq_key;
    wire data_fire=data_valid && data_ready;
    assign result_valid=enabled && state==COMPLETE && !stop_now;
    assign result_session=session_key;assign result_seq=seq_key;assign result_elapsed=elapsed;
    assign result_code=status;assign result_detail=detail;assign result_delivered=delivered;
    assign result_attempts=attempts;assign result_retries=retries;assign result_card_fault=card_fault;
    wire [32:0] req_end={1'b0,req_lba}+{29'd0,req_sectors}-33'd1;
    wire ram_write=b_data_fire && !isolate;
    // 地址只来自本域正沿寄存器；下降沿保持，RAM上升沿读到的地址/拍数不变。
    // 不复位此镜像：复位期间读使能已屏蔽，释放前已有下降沿捕获有效地址。
    // 避免把下降沿释放的本域rst接到同沿寄存器SR而产生新的恢复时间短路径。
    reg [9:0] bank_read_address_guard;
    always @(negedge clk) bank_read_address_guard<={read_bank,read_index};
    fifo_dual_clock_ram #(.DATA_WIDTH(8),.ADDR_WIDTH(10),.USE_EG_BRAM(USE_EG_BRAM),
                         .RAM_IMPLEMENT("9K")) u_banks(
        .wr_clk(clk),.wr_enable(ram_write),.wr_addr({fill_bank,received[8:0]}),.wr_data(b_data),
        .rd_clk(clk),.rd_enable(enabled && state==RUN && consume_state==C_READ && !isolate),
        .rd_addr(bank_read_address_guard),.rd_data(data));
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;fetch_state<=F_IDLE;consume_state<=C_IDLE;
            session_key<=0;seq_key<=0;base_lba<=0;budget<=0;remaining_budget<=0;attempt_key<=0;next_attempt<=FIRST_ATTEMPT;
            sectors<=0;next_sector<=0;status<=OK;sector0<=0;sector1<=0;full<=0;retry_count<=0;
            fill_bank<=0;read_bank<=0;key_valid<=0;stopped<=0;aborting<=0;backend_active<=0;
            cancel_sent<=0;exhausted<=0;card_fault<=0;received<=0;read_index<=0;delivered<=0;
            attempts<=0;retries<=0;detail<=0;
        end else begin
            if(state==IDLE && req_valid && req_ready) begin
                session_key<=req_session;seq_key<=req_seq;base_lba<=req_lba;budget<=req_budget_cycles[TIME_W-1:0];
                remaining_budget<=req_budget_cycles[TIME_W-1:0];sectors<=req_sectors;next_sector<=0;status<=OK;full<=0;retry_count<=0;
                fill_bank<=0;read_bank<=0;key_valid<=1;stopped<=0;aborting<=0;card_fault<=0;
                received<=0;read_index<=0;delivered<=0;attempts<=0;retries<=0;detail<=0;
                fetch_state<=F_IDLE;consume_state<=C_IDLE;state<=RUN;
                if(req_sectors==0 || req_sectors>8 || req_end[32]) begin status<=DESCRIPTOR;state<=COMPLETE;end
                else if(req_budget_cycles[TIME_W-1:0]==0) begin status<=DEADLINE;state<=COMPLETE;end
                else if(exhausted) begin status<=GENERATION;state<=COMPLETE;end
            end
            if(state==COMPLETE && result_valid && result_ready) state<=IDLE;
            if(state==RUN) begin
                if(!aborting) remaining_budget<=remaining_budget-1'b1;
                if(!aborting) begin
                    case(fetch_state)
                        F_IDLE:if(next_sector<sectors && !full[fill_bank]) begin
                            if(exhausted) begin aborting<=1;status<=GENERATION;end
                            else begin received<=0;fetch_state<=F_ISSUE;end
                        end
                        F_ISSUE:if(b_req_fire) begin
                            backend_active<=1;attempt_key<=next_attempt;
                            // KEY_W=32 时编号用尽后返回 GENERATION；较窄编号回绕并跳过零。
                            // 每次请求仅允许一个在途尝试，取消后等匹配结果完成才分配下一编号。
                            if(KEY_W<32) next_attempt<=(next_attempt=={KEY_W{1'b1}}) ? {{(KEY_W-1){1'b0}},1'b1} : next_attempt+1'b1;
                            else begin
                                next_attempt<=next_attempt+1'b1;
                                if(next_attempt=={KEY_W{1'b1}}) exhausted<=1;
                            end
                            attempts<=attempts+6'd1;cancel_sent<=0;received<=0;fetch_state<=F_RECV;
                        end
                        F_RECV:if(b_data_fire && !data_bad) begin
                            received<=received+10'd1;
                            if(received==511) fetch_state<=F_RESULT;
                        end
                        default:begin end
                    endcase
                    case(consume_state)
                        C_IDLE:if(full[read_bank]) begin read_index<=0;consume_state<=C_READ;end
                        C_READ:consume_state<=C_OUTPUT;
                        C_OUTPUT:if(data_fire) begin
                            delivered<=delivered+13'd1;
                            if(read_index==511) begin
                                full[read_bank]<=0;read_bank<=!read_bank;consume_state<=C_IDLE;
                            end else begin read_index<=read_index+9'd1;consume_state<=C_READ;end
                        end
                        default:consume_state<=C_IDLE;
                    endcase
                    if(next_sector==sectors && fetch_state==F_IDLE && full==0 && consume_state==C_IDLE)
                        state<=COMPLETE;
                end else if(!backend_active) begin fetch_state<=F_IDLE;state<=COMPLETE;end
            end
            if(b_cancel_valid && b_cancel_ready) cancel_sent<=1;
            if(b_result_fire && result_matches) begin
                backend_active<=0;fetch_state<=F_IDLE;card_fault<=card_fault || b_result_card_fault;detail<=b_result_detail;
                if(!isolate && b_result_code==OK) begin
                    full[fill_bank]<=1;
                    if(fill_bank) sector1<=next_sector[2:0];else sector0<=next_sector[2:0];
                    fill_bank<=!fill_bank;next_sector<=next_sector+4'd1;retry_count<=0;
                end else if(!isolate && can_retry) begin
                    retry_count<=retry_count+2'd1;retries<=retries+5'd1;
                    received<=0;fetch_state<=F_ISSUE;
                end
            end
            // 协议错误/致命结果即刻隔离所有扇区缓冲区，但未完成的实际卡事务仍要取消收尾。
            if(state==RUN && (protocol_now || fatal_result || (fetch_state==F_IDLE && !aborting && exhausted && next_sector<sectors))) begin
                aborting<=1;full<=0;consume_state<=C_IDLE;
                status<=protocol_now ? PROTOCOL : (exhausted && !fatal_result ? GENERATION : b_result_code);
            end
            if(deadline_now) begin aborting<=1;status<=DEADLINE;full<=0;consume_state<=C_IDLE;end
            if(stop_now) begin
                stopped<=1;aborting<=1;status<=CANCELED;full<=0;consume_state<=C_IDLE;
            end
            // ISSUE尚未被接受时没有实际卡所有权；已接受则只由匹配结果握手清active。
            if(isolate && !backend_active && !b_req_fire) fetch_state<=F_IDLE;
        end
    end
endmodule
`default_nettype wire
