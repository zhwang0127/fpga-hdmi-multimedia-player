`timescale 1ns/1ps
`default_nettype none
// mem域整文件WAV→PCM 缓冲区；只有文件尾检查通过且所有已接受写段完成后才报告成功。
// 复用流式wav_parser，PCM头/尾chunk均不写入SDRAM。32字节缓存组成单BL8，
// 小端L16/R16原样组为{R16,L16}；最后不足32字节时补零，reported words不含补零。
// 请求一旦挂出即保持到ready，取消/超期也不能撤回或截断已接受的八拍。
// 本模块只写尚未交给播放端的音频缓冲区；sys 域的 wav_bank_manager 防止覆盖已占用或正在播放的缓冲区。
module wav_sdram_loader(
    input wire clk,arst,start_valid,abort,
    output wire start_ready,start_ready_na,
    input wire [31:0] file_length,budget_cycles,input wire bank,
    input wire byte_valid,input wire [7:0] byte_data,input wire byte_last,
    output wire byte_ready,byte_ready_na,
    output wire result_valid,input wire result_ready,output wire result_ok,
    output wire [7:0] result_detail,output wire [18:0] out_written_words,
    output wire quantum_idle,
    output wire load_valid,input wire load_ready,output wire [20:0] load_addr,
    output wire [3:0] load_count,output wire [31:0] load_budget,
    output wire wr_data_valid,output wire [31:0] wr_data,
    input wire load_beat,load_done,load_done_was_write
);
    localparam IDLE=0,FILL=1,ISSUE=2,WRITE=3,RESULT=4,VALIDATE=5;
    reg [2:0] state;
    wire rst,rst_logic;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    reg [31:0] remaining;
    reg stopped,parsed,parser_good,data_end;
    reg [7:0] detail;
    reg [19:0] pcm_bytes;
    reg [18:0] physical_words;
    reg [5:0] fill_bytes;
    reg [2:0] beat_index;
    // 独立四位计数区分第7、8、9拍，完成条件使用实际写拍数，不依赖索引回绕。
    // 索引回到0不等于八拍写完，实际写拍数要单独核对。
    reg [3:0] burst_beats;
    reg [20:0] address;
    // 每字完整写入，避免TD对变量字节写RAM产生未驱动的部分写端。
    // 未复位暂存数据在完整32B收齐之前绝不发布，条件与地址寄存器仍正常复位。
    // 八字FIFO式移位暂存：收满一个字移入高端，低端始终是下个待读字。
    // 数据阵列无复位/初值，发布前必须真实覆盖八字，控制状态和握手条件独立保存。
    reg [255:0] word_ring;
    reg [31:0] assembly_word;
    // 到期标志与 remaining 同步更新，表示剩余预算为零。
    // 停止和字节接受条件直接使用该标志，减少宽计数比较的扇出。
    reg rem_zero_q;
    wire expired=state!=IDLE && state!=RESULT && rem_zero_q;
    wire stopping=stopped || abort || expired;
    wire parser_start_ready,parser_byte_ready,pcm_valid,pcm_last,parser_result,parser_ok;
    // 输入字节先进入单字节缓冲，上游 ready 只取决于该缓冲是否空闲，不直接依赖 chunk 检查、
    // PCM 组包和取消逻辑；解析器读取缓冲时接受字节，上游 CRC 在字节进入缓冲时更新。
    reg input_pending;reg [7:0] input_data;reg input_last;
    // abort 同拍禁止接收新字节、生成 PCM 写入及报告装载成功。
    // 缓冲中已接收的字节可继续交给解析器，但取消的任务不会报告成功，上层不会将缓冲区置为 READY。
    wire parser_input_valid=input_pending && !stopped && !parsed;
    wire parser_input_fire=parser_input_valid && parser_byte_ready;
    // 输入暂存和 PCM 均已处理完、待写数据均已写入后，才能确认本块完成；尾包补零还能自动发写，必须等待补零数据写入 SDRAM。
    // 普通块末的半包继续保留，仅在文件末尾补零，不能为结束当前分块而在文件中途补零。
    assign quantum_idle=!rst_logic && !input_pending && !pcm_valid &&
        (state==IDLE || state==RESULT || state==VALIDATE ||
         (state==FILL && !(data_end && fill_bytes!=0)));
    always @(posedge clk or posedge rst) begin
        if(rst) begin input_pending<=0;input_data<=0;input_last<=0;end
        else if(state==IDLE || state==RESULT || stopping || parser_result) input_pending<=0;
        else begin
            if(parser_input_fire) input_pending<=0;
            if(byte_valid && byte_ready) begin input_pending<=1;input_data<=byte_data;input_last<=byte_last;end
        end
    end
    wire [7:0] pcm_data,parser_detail;
    wire [31:0] data_length;
    // 解析器的 abort=0 条件单独计算；abort=1 时 byte_valid 已被禁止，
    // 因而解析器不会看到PCM有效；本层真实byte_ready仍显式与!abort相与。
    wire pcm_ready=state==FILL && !data_end && !stopped && !expired;
    assign start_ready_na=!rst_logic && state==IDLE && parser_start_ready;
    assign start_ready=start_ready_na && !abort;
    wire start_fire=start_valid && start_ready;
    // 地址在独立时序块中保存，以免与控制状态重编码合并；仅在装载起始沿更新。
    always @(posedge clk or posedge rst) begin
        if(rst) address<=0;
        else if(state==IDLE && start_fire) address<=bank ? 21'h180000 : 21'h140000;
        else if(state==WRITE && load_done) address<=address+21'd8;
    end
    // 镜像更新沿与remaining完全相同：装载拍看预算是否为0，递减拍看是否减到1。
    // 用自身作递减守卫，remaining 及其到期镜像均以该标志决定是否继续递减。
    always @(posedge clk or posedge rst) begin
        if(rst) rem_zero_q<=1;
        else if(state==IDLE && start_fire) rem_zero_q<=budget_cycles==32'd0;
        else if(state!=IDLE && state!=RESULT && !rem_zero_q) rem_zero_q<=remaining==32'd1;
    end
    // na为外部abort=0的余因子；本地粘滞停止及预算仍计入，不能忽略它们继续收字节。
    assign byte_ready_na=!rst_logic && state!=IDLE && !parsed && !stopped && !expired && !input_pending;
    assign byte_ready=byte_ready_na && !abort;
    wav_parser #(.PRODUCT_LOGIC_RESET(1)) u_parser(
        .clk(clk),.arst(arst),.start_valid(start_fire),.start_ready(parser_start_ready),
        .file_length(file_length),.abort(stopped),
        .byte_valid(parser_input_valid),
        .byte_ready(parser_byte_ready),.byte_data(input_data),.byte_last(input_last),
        .pcm_valid(pcm_valid),.pcm_ready(pcm_ready),.pcm_data(pcm_data),.pcm_last(pcm_last),
        .result_valid(parser_result),.result_ready(!parsed),.result_ok(parser_ok),
        .result_detail(parser_detail),.out_offset(),.out_length(data_length));
    assign load_valid=!rst_logic && state==ISSUE;
    assign load_addr=address;
    assign load_count=4'd1;
    // 已挂出请求的预算固定饱和到至少1；本模块期限独立逐拍计时，超时后报告本次装载失败，该缓冲区的内容不能交给播放端。
    // 物理层不得因本地超期停供八字；预算仍使用接受文件时的剩余窗口。
    reg [31:0] segment_budget;
    assign load_budget=segment_budget;
    assign wr_data_valid=!rst_logic && state==WRITE;
    // 请求发布至写段完成期间，队列头保持为当前写数据。
    // load_valid 和 wr_data_valid 无效时，接收端不得使用地址或数据载荷。
    assign wr_data=word_ring[31:0];
    assign result_valid=!rst_logic && state==RESULT;
    // result_valid才有结果意义；空闲时不要只看result_ok就把音频缓冲区交给播放。
    assign result_ok=detail==0;
    assign result_detail=detail;
    assign out_written_words={1'b0,pcm_bytes[19:2]};
    wire put_pcm=pcm_valid && pcm_ready && !abort;
    wire pad=state==FILL && data_end && fill_bytes!=0 && !stopping;
    wire [7:0] put_byte=pad ? 8'd0 : pcm_data;
    wire [18:0] padded_words={2'd0,data_length[19:5]}*19'd8 +
        (data_length[4:0]!=0 ? 19'd8 : 19'd0);
    reg [18:0] padded_words_q;
    reg counts_good_q;
    // 完整字条件已保证fill_bytes非零，pad分支无需再比较全部六位。
    // put_pcm 和 pad 同拍响应取消或超时，停止后不再写入新数据。
    wire ring_push=!rst_logic && fill_bytes[1:0]==3 && state==FILL && !stopping && (data_end || pcm_valid);
    wire ring_rotate=state==WRITE && load_beat && beat_index!=7;
    // 大加法与两组计数比较分开寄存；最终额外VALIDATE拍只消费稳定快照。
    always @(posedge clk) begin
        padded_words_q<=padded_words;
        counts_good_q<=data_end && pcm_bytes==data_length[19:0] && physical_words==padded_words_q;
        // 两种操作共享低224位的字移位，只在最高字选择新组包或循环队列头。
        // 若异常输入让两条件同拍为真，按 ring_push 优先处理。
        if(ring_rotate || ring_push) begin
            word_ring<={ring_push ? {put_byte,assembly_word[31:8]} : word_ring[31:0],word_ring[255:32]};
        end
    end

    // 错误码在独立时序块中保存，优先级为写段异常、最终检查、解析错误、取消或超时。
    // 启动沿清零，完成前保持首次有效错误；同拍覆盖顺序由此优先级决定。
    wire write_fault=state==WRITE && ((load_beat && burst_beats>=8) ||
        (load_done && (!load_done_was_write || (load_beat ? burst_beats!=7 : burst_beats!=8))));
    always @(posedge clk or posedge rst) begin
        if(rst) detail<=0;
        else if(state==IDLE && start_fire) detail<=0;
        else if(write_fault) detail<=8'he4;
        else if(state==VALIDATE && !stopping && !counts_good_q) detail<=8'he3;
        else if(parser_result && !parsed && !parser_ok && detail==0) detail<=parser_detail;
        else if(state!=IDLE && state!=RESULT && (abort || expired) && detail==0)
            detail<=abort ? 8'he1 : 8'he2;
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;remaining<=0;stopped<=0;parsed<=0;parser_good<=0;data_end<=0;
            pcm_bytes<=0;physical_words<=0;fill_bytes<=0;beat_index<=0;burst_beats<=0;
            segment_budget<=0;assembly_word<=0;
        end else begin
            if(state!=IDLE && state!=RESULT && !rem_zero_q) remaining<=remaining-32'd1;
            if(state!=IDLE && state!=RESULT && (abort || expired)) begin
                stopped<=1;
            end
            if(parser_result && !parsed) begin
                parsed<=1;parser_good<=parser_ok;
                
            end
            if(put_pcm) begin pcm_bytes<=pcm_bytes+20'd1;if(pcm_last) data_end<=1;end
            if(put_pcm || pad) begin
                assembly_word<={put_byte,assembly_word[31:8]};
                if(fill_bytes==31) begin
                    fill_bytes<=0;beat_index<=0;burst_beats<=0;
                    // 第八个完整字移入后，队列低端就是第一字；ISSUE时才发布请求。
                    segment_budget<=remaining>1 ? remaining-1 : 1;
                end else fill_bytes<=fill_bytes+6'd1;
            end
            case(state)
                IDLE:if(start_fire) begin
                    state<=FILL;remaining<=budget_cycles;stopped<=0;parsed<=0;parser_good<=0;
                    data_end<=0;pcm_bytes<=0;physical_words<=0;fill_bytes<=0;beat_index<=0;burst_beats<=0;
                    // PCM 缓冲区使用固定字地址，与 mem_map.vh 的 PCM_A/PCM_B 字节基址右移两位对应。
                end
                FILL:begin
                    // 本状态按是否收满、是否还需补零选择继续或发起写段。
                    if(stopping || (parsed && !parser_good)) begin
                        fill_bytes<=0;
                        if(parsed) state<=RESULT;
                    end else if(fill_bytes==31 && (pcm_valid || data_end)) begin
                        state<=ISSUE;
                    end else if(parsed && fill_bytes==0 && (data_end || !pcm_valid)) begin
                        // 本分支已知FILL且未停止，!put_pcm可用等值余因子。
                        state<=VALIDATE;
                    end
                end
                VALIDATE:begin
                    
                    state<=RESULT;
                end
                ISSUE:if(load_ready) state<=WRITE;
                WRITE:begin
                    if(load_beat) begin
                        physical_words<=physical_words+19'd1;
                        if(burst_beats<15) burst_beats<=burst_beats+1'b1;
                        if(burst_beats>=8) begin stopped<=1;end
                        if(beat_index!=7) begin beat_index<=beat_index+3'd1;end
                    end
                    if(load_done) begin
                        // 写段完成沿若同时接受最后一拍，旧计数应为7；否则应已为8。
                        // 末级只选择两个相等判据，不让共享后台的load_beat再穿过加法器。
                        if(!load_done_was_write || (load_beat ? burst_beats!=7 : burst_beats!=8)) begin stopped<=1;end
                        state<=FILL;
                    end
                end
                RESULT:if(result_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
`ifdef FORMAL
    // 无引用条件的本地装载器只生成单BL8；在途仍持续供数，不被abort截断。
    always @* begin
        assert(!load_valid || load_count==1);
        assert(!wr_data_valid || state==WRITE);
        assert(!result_valid || state==RESULT);
    end
`endif
endmodule
`default_nettype wire
