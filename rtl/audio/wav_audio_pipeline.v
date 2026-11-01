`timescale 1ns/1ps
`default_nettype none
// mem/pix音频执行端。控制和响应由外层文件 mailbox 运送，音频数据通过各音频缓冲区的异步 FIFO 跨域。
// 每音频缓冲区独立2048x32 FIFO和PCM DMA，两个DMA按单BL8合并成共享层的第三客户端。
// sys只有取得ARMED(写端确认 FIFO 已缓存至少 512 个样本)才能发带音频元数据的显示描述符；CANCEL响应
// 必须等待 DMA 在途读请求结束以及 FIFO flush_done。token由sys所有者分配，本端原样回送。
// pix在显示实际commit_go同沿重置PCM样本计数，阻止该沿弹出旧音频缓冲区；EOF持续输出零。
module wav_audio_pipeline #(
    parameter ENABLE=1,USE_EG_BRAM=1
)(
    input wire arst,mem_clk,pix_clk,
    input wire cmd_valid,output wire cmd_ready,input wire cmd_cancel,cmd_bank,
    input wire [17:0] cmd_words,input wire [15:0] cmd_token,
    output wire rsp_valid,input wire rsp_ready,output reg [1:0] rsp_status,
    output reg rsp_bank,output reg [15:0] rsp_token,
    output wire pcm_valid,input wire pcm_ready,output wire [20:0] pcm_addr,
    output wire pcm_urgent,output wire [31:0] pcm_budget,
    input wire pcm_data_valid,input wire [31:0] pcm_data,input wire pcm_done,
    output wire pcm_data_ready,
    // commit_go与next_audio来自像素域显示描述符提交逻辑；audio为已提交元数据。
    input wire commit_go,input wire [19:0] next_audio,audio,input wire ref_valid,
    input wire sample_ce,sample_accept,input wire [15:0] tone_l,tone_r,
    output wire sample_valid,output wire [15:0] sample_l,sample_r,
    output wire underrun,eof,output wire [17:0] played,
    output wire fault,output wire [11:0] level_a,level_b
);
    wire mrst,mrst_logic,prst,prst_logic;
    reset_sync u_mem_reset(.clk(mem_clk),.arst(arst),.rst(mrst),.logic_rst(mrst_logic));
    reset_sync u_pix_reset(.clk(pix_clk),.arst(arst),.rst(prst),.logic_rst(prst_logic));
    wire enabled_m=ENABLE && !mrst_logic;
    reg start_a,start_b,cancel_a,cancel_b,session_a,session_b;
    reg [17:0] length_a,length_b;
    wire va,vb,ra,rb,ua,ub,da,db,ca,cb,oa,ob,busy_a,busy_b;
    wire [20:0] addr_a,addr_b;wire [31:0] data_a,data_b;
    wire wr_a,wr_b,wr_ready_a,wr_ready_b,flush_a,flush_b,flushed_a,flushed_b;
    wire active_a,active_b,flush_ready_a,flush_ready_b;
    wire read_valid_a,read_valid_b,read_ready_a,read_ready_b;wire [31:0] read_data_a,read_data_b;
    reg direct_flush_a,direct_flush_b;
    wire [11:0] read_level_a,read_level_b;
    reg select_valid,select_bank,read_active,read_bank,round_robin;
    // 新选择先寄存；发出后保持到共享层真实接受，done之前归属不能变化。
    assign pcm_valid=enabled_m && select_valid && (select_bank ? vb : va);
    assign pcm_addr=ENABLE ? (select_bank ? addr_b : addr_a) : 21'd0;
    assign pcm_urgent=ENABLE && (select_bank ? ub : ua);
    assign pcm_budget=32'd100000;
    assign pcm_data_ready=ENABLE ? (read_bank ? wr_ready_b : wr_ready_a) : 1'b1;
    assign ra=pcm_ready && pcm_valid && !select_bank;
    assign rb=pcm_ready && pcm_valid && select_bank;
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin select_valid<=0;select_bank<=0;read_active<=0;read_bank<=0;round_robin<=0;end
        else begin
            if(enabled_m && !read_active && !select_valid && (va || vb)) begin
                select_valid<=1;
                select_bank<=!va ? 1'b1 : !vb ? 1'b0 : ua!=ub ? ub : round_robin;
            end
            if(pcm_valid && pcm_ready) begin select_valid<=0;read_active<=1;read_bank<=select_bank;end
            if(read_active && pcm_done) begin read_active<=0;round_robin<=!read_bank;end
        end
    end
    pcm_dma u_dma_a(.clk(mem_clk),.rst(mrst_logic || !ENABLE),.start(start_a),
        .base_addr(21'h140000),.nwords(length_a),.cancel(cancel_a),
        .req_valid(va),.req_ready(ra),.req_addr(addr_a),.req_count(),
        .rd_valid(pcm_data_valid && read_active && !read_bank),.rd_data(pcm_data),.req_done(pcm_done && read_active && !read_bank),
        .fifo_wr_valid(wr_a),.fifo_wr_data(data_a),.fifo_wr_ready(wr_ready_a),.fifo_wr_level(level_a),
        .fifo_flush_req(flush_a),.fifo_flush_done(flushed_a),.urgent(ua),.fetch_done(da),.cancel_done(ca),.overflow(oa),.busy(busy_a));
    pcm_dma u_dma_b(.clk(mem_clk),.rst(mrst_logic || !ENABLE),.start(start_b),
        .base_addr(21'h180000),.nwords(length_b),.cancel(cancel_b),
        .req_valid(vb),.req_ready(rb),.req_addr(addr_b),.req_count(),
        .rd_valid(pcm_data_valid && read_active && read_bank),.rd_data(pcm_data),.req_done(pcm_done && read_active && read_bank),
        .fifo_wr_valid(wr_b),.fifo_wr_data(data_b),.fifo_wr_ready(wr_ready_b),.fifo_wr_level(level_b),
        .fifo_flush_req(flush_b),.fifo_flush_done(flushed_b),.urgent(ub),.fetch_done(db),.cancel_done(cb),.overflow(ob),.busy(busy_b));
    // 两个音频FIFO单独启用写端缓存样本计数流水，报告读指针再保守一拍；精确容量和读接口不变。
    // FULL_LOOKAHEAD只提前缓存下一写Gray指针，满标志仍在原沿生效，不增加握手延迟。
    // 预装阶段未收到ARMED就没有显示引用，实际ARM阈值仍至少512；DMA 发请求前的剩余空间判断更保守。
    // FIFO 实例路径与逐网 CDC 约束配套，修改层次名时须同步更新约束。
    async_fifo #(.DATA_WIDTH(32),.ADDR_WIDTH(11),.USE_EG_BRAM(USE_EG_BRAM),.PIPE_WR_LEVEL(1),.FULL_LOOKAHEAD(1),.READ_PRECOMPARE(1),.LEVEL_LOOKAHEAD(1)) u_fifo_a(
        .arst(arst),.wr_clk(mem_clk),.wr_valid(wr_a && ENABLE),.wr_data(data_a),.wr_ready(wr_ready_a),.wr_active(active_a),.wr_level(level_a),
        .flush_req(flush_a || direct_flush_a),.flush_ready(flush_ready_a),.flush_done(flushed_a),.flush_busy(),
        .rd_clk(pix_clk),.rd_ready(read_ready_a),.rd_valid(read_valid_a),.rd_data(read_data_a),.rd_active(),.rd_level(read_level_a));
    async_fifo #(.DATA_WIDTH(32),.ADDR_WIDTH(11),.USE_EG_BRAM(USE_EG_BRAM),.PIPE_WR_LEVEL(1),.FULL_LOOKAHEAD(1),.READ_PRECOMPARE(1),.LEVEL_LOOKAHEAD(1)) u_fifo_b(
        .arst(arst),.wr_clk(mem_clk),.wr_valid(wr_b && ENABLE),.wr_data(data_b),.wr_ready(wr_ready_b),.wr_active(active_b),.wr_level(level_b),
        .flush_req(flush_b || direct_flush_b),.flush_ready(flush_ready_b),.flush_done(flushed_b),.flush_busy(),
        .rd_clk(pix_clk),.rd_ready(read_ready_b),.rd_valid(read_valid_b),.rd_data(read_data_b),.rd_active(),.rd_level(read_level_b));
    localparam C_IDLE=0,C_ARM=1,C_CANCEL=2,C_RESPONSE=3;
    reg [1:0] control_state;reg control_bank,control_direct;
    reg protocol_fault;
    // 缓存样本数达到 512 的条件先寄存，再交给音频准备状态机使用。
    // 只在C_ARM采样已选音频缓冲区，其他状态清0，防止用上一缓冲区的样本数判断新配音已经准备好。
    // sys取得ARMED之前不会提交引用该新音频缓冲区的显示描述符，因此预装阶段没有读出；
    // FIFO 已缓存样本数只增不减，晚一拍给响应仍保证至少有 512 个样本，不降低启动阈值。
    reg arm_level_ready_q;
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) arm_level_ready_q<=0;
        else arm_level_ready_q<=control_state==C_ARM && (control_bank ? level_b>=512 : level_a>=512);
    end
    // 常量范围用“最高一个不同位”的前缀并行判断，避免18位比较进位链串入响应状态。
    // 每项高位都等于边界常量，当前位才取相反值；所有项相或恰好为<48000或>240000。
    // 等于两端仍合法，任意18位输入成立；不提前采样、不加拍，取消分支优先级不变。
    wire selected_session=cmd_bank ? session_b : session_a;
    wire selected_active=cmd_bank ? active_b : active_a;
    assign cmd_ready=enabled_m && control_state==C_IDLE &&
        (cmd_cancel || selected_active);
    assign rsp_valid=enabled_m && control_state==C_RESPONSE;
    assign fault=oa || ob || protocol_fault;
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin
            start_a<=0;start_b<=0;cancel_a<=0;cancel_b<=0;session_a<=0;session_b<=0;
            length_a<=0;length_b<=0;direct_flush_a<=0;direct_flush_b<=0;
            control_state<=C_IDLE;control_bank<=0;control_direct<=0;
            rsp_status<=0;rsp_bank<=0;rsp_token<=0;protocol_fault<=0;
        end else begin
            start_a<=0;start_b<=0;cancel_a<=0;cancel_b<=0;
            case(control_state)
                C_IDLE:if(cmd_valid && cmd_ready) begin
                    control_bank<=cmd_bank;rsp_bank<=cmd_bank;rsp_token<=cmd_token;
                    if(cmd_cancel) begin
                        control_direct<=!selected_session;control_state<=C_CANCEL;
                        if(cmd_bank) begin
                            if(selected_session) cancel_b<=1;else direct_flush_b<=1;
                        end else begin
                            if(selected_session) cancel_a<=1;else direct_flush_a<=1;
                        end
                    end else if(((cmd_words[17:16]==2'd0 && !cmd_words[15]) ||
                        (cmd_words[17:14]==4'd2 && !cmd_words[13]) ||
                        (cmd_words[17:13]==5'd5 && !cmd_words[12]) ||
                        (cmd_words[17:12]==6'd11 && !cmd_words[11]) ||
                        (cmd_words[17:10]==8'd46 && !cmd_words[9]) ||
                        (cmd_words[17:9]==9'd93 && !cmd_words[8]) ||
                        (cmd_words[17:8]==10'd187 && !cmd_words[7])) ||
                        ((cmd_words[17:15]==3'd7 && cmd_words[14]) ||
                        (cmd_words[17:13]==5'd29 && cmd_words[12]) ||
                        (cmd_words[17:11]==7'd117 && cmd_words[10]) ||
                        (cmd_words[17:10]==8'd234 && cmd_words[9]) ||
                        (cmd_words[17:7]==11'd1875 && cmd_words[6]) ||
                        (cmd_words[17:6]==12'd3750 && cmd_words[5]) ||
                        (cmd_words[17:5]==13'd7500 && cmd_words[4]) ||
                        (cmd_words[17:4]==14'd15000 && cmd_words[3]) ||
                        (cmd_words[17:3]==15'd30000 && cmd_words[2]) ||
                        (cmd_words[17:2]==16'd60000 && cmd_words[1]) ||
                        (cmd_words[17:1]==17'd120000 && cmd_words[0])) || cmd_token==0 || selected_session || protocol_fault) begin
                        rsp_status<=2;control_state<=C_RESPONSE;protocol_fault<=1;
                    end else begin
                        if(cmd_bank) begin length_b<=cmd_words;start_b<=1;session_b<=1;end
                        else begin length_a<=cmd_words;start_a<=1;session_a<=1;end
                        control_state<=C_ARM;
                    end
                end
                C_ARM:if(control_bank ? ob : oa) begin rsp_status<=2;control_state<=C_RESPONSE;end
                    else if(arm_level_ready_q) begin rsp_status<=0;control_state<=C_RESPONSE;end
                C_CANCEL:if(control_direct ? (control_bank ? flushed_b : flushed_a) : (control_bank ? cb : ca)) begin
                    if(control_bank) begin session_b<=0;direct_flush_b<=0;end
                    else begin session_a<=0;direct_flush_a<=0;end
                    rsp_status<=1;control_state<=C_RESPONSE;
                end
                C_RESPONSE:if(rsp_ready) control_state<=C_IDLE;
                default:begin protocol_fault<=1;control_state<=C_IDLE;end
            endcase
        end
    end
    wire fifo_pop,pcm_ce,pcm_out_valid;wire [15:0] pcm_l,pcm_r;
    pcm_sample_out u_samples(.clk(pix_clk),.rst(prst_logic || !ENABLE),
        .start(commit_go && ENABLE),.n_play(next_audio[17:0]),.play(ENABLE && ref_valid && audio[19]),
        .sample_ce(sample_ce && sample_accept),.fifo_rd_valid(audio[18] ? read_valid_b : read_valid_a),
        .fifo_rd_data(audio[18] ? read_data_b : read_data_a),.fifo_rd_ready(fifo_pop),
        .out_ce(pcm_ce),.out_l(pcm_l),.out_r(pcm_r),.out_valid(pcm_out_valid),.underrun(underrun),.eof(eof),.played(played));
    assign read_ready_a=ENABLE && !prst_logic && !audio[18] && fifo_pop;
    assign read_ready_b=ENABLE && !prst_logic && audio[18] && fifo_pop;
    reg sample_tick_q;reg [15:0] tone_l_q,tone_r_q;
    // PCM模块输出晚一拍，提示音同样锁存旧相位值。即便commit与CE重合，仍发一个零样本，
    // 保持48k时刻连续；start禁止旧FIFO弹出，下一次CE才读新音频缓冲区的第0样本。
    always @(posedge pix_clk) begin
        if(prst_logic) begin sample_tick_q<=0;tone_l_q<=0;tone_r_q<=0;end
        else begin
            sample_tick_q<=ENABLE && sample_ce && sample_accept;
            if(sample_ce && sample_accept) begin tone_l_q<=tone_l;tone_r_q<=tone_r;end
        end
    end
    assign sample_valid=sample_tick_q;
    assign sample_l=audio[19] && ref_valid ? pcm_l : tone_l_q;
    assign sample_r=audio[19] && ref_valid ? pcm_r : tone_r_q;
endmodule
`default_nettype wire
