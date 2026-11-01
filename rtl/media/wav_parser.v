`timescale 1ns/1ps
`default_nettype none
// WAV整文件流式解析：共享32位组包器，fmt按字节比较，避免保存宽字段副本。
// PCM输出仅为暂存数据；尾部 chunk 和整文件检查成功前，装载控制模块不得提交音频缓冲区。
// abort 只停止本模块解析；上层还须等待源文件读取停止及在途数据处理完毕。
module wav_parser #(parameter PRODUCT_LOGIC_RESET=0)(
    input wire clk,arst,
    input wire start_valid,output wire start_ready,input wire [31:0] file_length,
    input wire abort,
    input wire byte_valid,output wire byte_ready,input wire [7:0] byte_data,
    input wire byte_last,
    output wire pcm_valid,input wire pcm_ready,output wire [7:0] pcm_data,
    output wire pcm_last,
    output wire result_valid,input wire result_ready,
    output wire result_ok,output wire [7:0] result_detail,
    output wire [31:0] out_offset,out_length
);
    localparam IDLE=0,RIFF=1,BOUNDARY=2,CHDR=3,CHECK=4,BODY=5,PAD=6,TAIL=7,RESULT=8,CHUNK_PAD=9,CHUNK_SUM=10,CHUNK_RANGE=11,RIFF_SUM=12,RIFF_RANGE=13,BOUND_CHECK=14;
    localparam OK=0,MAGIC=1,TRUNC=2,RANGE=3,FMT=4,ORDER=5,DUP=6,
               HEADER_LIMIT=7,DATA_LENGTH=8,FILE_LIMIT=9,PROTOCOL=10,CANCEL=11;
    reg [3:0] state;
    reg [19:0] pos,flen,riff_end,body_end,data_offset,data_length;
    // pos_p8_q 与 pos 同步推进，保存 pos+8，供下一个 chunk 头的完整边界比较使用。
    reg [19:0] pos_p8_q;
    reg [31:0] word_buf;
    // 文件不足1MiB，高12位只需保存非零条件；所有越界拒绝仍保留。
    reg [19:0] chunk_size_low; reg chunk_size_big;
    wire [31:0] chunk_size={11'd0,chunk_size_big,chunk_size_low};
    reg chunk_fmt_tag,chunk_data_tag;
    reg [4:0] idx;
    reg seen_fmt,seen_data,fmt_bad,is_fmt,is_data,odd;
    reg [7:0] detail;
    // 产品模式使用只读检查流水；文件字节口在这些状态背压，不改变接受的字节顺序。
    // 范围只和20位文件位置比较：保留低20位与粘滞溢出，高位非零时仍按超限处理。
    reg [20:0] chunk_compact_q,end_compact_q,riff_compact_q;
    wire rst,rst_logic;
    // 使用 CHUNK_PAD 准备拍，CHECK只读稳定判据，不增加阶段或字节接受延迟。
    reg fmt_short_q,data_bad_q,size_zero_q,size_one_q;
    wire fmt_short=PRODUCT_LOGIC_RESET ? fmt_short_q : chunk_size<32'd16;
    wire data_bad=PRODUCT_LOGIC_RESET ? data_bad_q : (chunk_size<32'd192000 || chunk_size>32'd960000 || chunk_size[1:0]!=0);
    wire size_zero=PRODUCT_LOGIC_RESET ? size_zero_q : chunk_size==0;
    wire size_one=PRODUCT_LOGIC_RESET ? size_one_q : chunk_size==1;
    always @(posedge clk or posedge rst) begin
        if(rst) begin fmt_short_q<=1;data_bad_q<=1;size_zero_q<=1;size_one_q<=0;end
        else if(PRODUCT_LOGIC_RESET && state==CHUNK_PAD) begin
            fmt_short_q<=chunk_size<32'd16;
            data_bad_q<=chunk_size<32'd192000 || chunk_size>32'd960000 || chunk_size[1:0]!=0;
            size_zero_q<=chunk_size==0;size_one_q<=chunk_size==1;
        end
    end
    wire [20:0] chunk_low_sum={1'b0,chunk_size[19:0]}+chunk_size[0];
    wire [20:0] end_low_sum={1'b0,pos}+{1'b0,chunk_compact_q[19:0]};
    wire [20:0] riff_low_sum={1'b0,word_buf[19:0]}+21'd8;
    // BOUNDARY本来就是背压准备拍；把稳定边界判据写入寄存器，下一拍仅选状态。
    reg boundary_range_bad_q,boundary_end_q,boundary_file_end_q,boundary_header_q;
    reg range_bad_q,header_bad_q;
    reg [19:0] file_penultimate_q,body_sum_q,body_penultimate_q;
    reg file_last_q,body_last_q;
    wire byte_fire=byte_valid && byte_ready;
    // 连续字节也能逐拍保持“本字节为末字节”。接受n后寄存n==end-2，
    // 下一字节n+1的比较已备好，避免20位加法再比较后进入所有字段的CE。
    always @(posedge clk or posedge rst) begin
        if(rst) begin file_penultimate_q<=0;file_last_q<=0;body_last_q<=0;end
        else if(start_valid && start_ready) begin
            file_penultimate_q<=file_length[19:0]-20'd2;file_last_q<=file_length==1;body_last_q<=0;
        end else begin
            if(byte_fire) begin file_last_q<=pos==file_penultimate_q;body_last_q<=pos==body_penultimate_q;end
            if(PRODUCT_LOGIC_RESET && state==CHECK) body_last_q<=size_one;
        end
    end
    wire current_file_last=PRODUCT_LOGIC_RESET ? file_last_q : pos+20'd1==flen;
    wire current_body_last=PRODUCT_LOGIC_RESET ? body_last_q : pos+20'd1==body_end;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 产品接入只让原始arst进入reset_sync异步置位端；业务使用正沿三级条件，
    // PRODUCT_LOGIC_RESET=1 时业务门控使用 rst_logic，并将范围检查分阶段寄存；=0 使用直接检查接口。
    wire enabled=PRODUCT_LOGIC_RESET ? !rst_logic : !arst && !rst;
    assign start_ready=enabled && state==IDLE;
    assign byte_ready=enabled && !abort &&
        (state==RIFF || state==CHDR || state==PAD || state==TAIL ||
         (state==BODY && (!is_data || pcm_ready)));
    assign pcm_valid=enabled && !abort && state==BODY && is_data && byte_valid;
    assign pcm_data=byte_data;
    assign pcm_last=current_body_last;
    assign result_valid=enabled && state==RESULT;
    assign result_ok=detail==OK;
    assign result_detail=detail;
    assign out_offset={12'd0,data_offset};
    assign out_length={12'd0,data_length};
    wire [31:0] assembled={byte_data,word_buf[31:8]};
    // 高位非零直接越界，低20位加pos和pad用21位保留进位。
    // 同时检查 33 位和是否越过整文件边界；不可丢掉chunk_size_big后只比较低位。
    wire [20:0] padded_end={1'b0,pos}+{1'b0,chunk_size_low}+{20'd0,chunk_size_low[0]};
    wire check_range_bad=PRODUCT_LOGIC_RESET ? range_bad_q : (chunk_size_big || padded_end>{1'b0,riff_end});
    wire check_header_bad=PRODUCT_LOGIC_RESET ? header_bad_q :
        !seen_data && ((chunk_data_tag && pos>20'd4096) ||
                      (!chunk_data_tag && (chunk_size_big || padded_end>21'd4088)));
    wire [32:0] riff_total={1'b0,assembled}+33'd8;
    function [7:0] magic_byte;
        input [3:0] n;
        begin case(n)
            0:magic_byte=8'h52;1:magic_byte=8'h49;2:magic_byte=8'h46;3:magic_byte=8'h46;
            8:magic_byte=8'h57;9:magic_byte=8'h41;10:magic_byte=8'h56;
            default:magic_byte=8'h45;
        endcase end
    endfunction
    function [7:0] fmt_byte;
        input [3:0] n;
        // PCM1/2ch/48000/192000/align4/16bit的小端常量，未知扩展字节跳过。
        begin case(n)
            0:fmt_byte=1;2:fmt_byte=2;4:fmt_byte=8'h80;5:fmt_byte=8'hbb;
            8:fmt_byte=0;9:fmt_byte=8'hee;10:fmt_byte=2;12:fmt_byte=4;14:fmt_byte=16;
            default:fmt_byte=0;
        endcase end
    endfunction
    // 每次接受字节时预先保存下一索引的常量字节；本字节比较只读FF。
    // CHECK/新文件沿准备首字节，暂停时同idx一起保持；无新增字节或握手拍。
    reg [7:0] magic_expected_q,fmt_expected_q;
    wire [7:0] magic_expected=PRODUCT_LOGIC_RESET ? magic_expected_q : magic_byte(idx[3:0]);
    wire [7:0] fmt_expected=PRODUCT_LOGIC_RESET ? fmt_expected_q : fmt_byte(idx[3:0]);
    always @(posedge clk or posedge rst) begin
        if(rst) begin magic_expected_q<=8'h52;fmt_expected_q<=8'd1;end
        else begin
            if(start_valid && start_ready) magic_expected_q<=8'h52;
            else if(state==RIFF && byte_fire) magic_expected_q<=magic_byte(idx[3:0]+4'd1);
            if(state==CHECK) fmt_expected_q<=8'd1;
            else if(state==BODY && is_fmt && idx<16 && byte_fire) fmt_expected_q<=fmt_byte(idx[3:0]+4'd1);
        end
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;pos<=0;pos_p8_q<=20'd8;flen<=0;riff_end<=0;body_end<=0;idx<=0;
            word_buf<=0;chunk_fmt_tag<=0;chunk_data_tag<=0;chunk_size_low<=0;chunk_size_big<=0;seen_fmt<=0;seen_data<=0;
            fmt_bad<=0;is_fmt<=0;is_data<=0;odd<=0;detail<=OK;
            data_offset<=0;data_length<=0;chunk_compact_q<=0;end_compact_q<=0;riff_compact_q<=0;
            boundary_range_bad_q<=0;boundary_end_q<=0;boundary_file_end_q<=0;boundary_header_q<=0;range_bad_q<=0;header_bad_q<=0;body_sum_q<=0;body_penultimate_q<=0;
        end else if(abort && state!=IDLE && state!=RESULT) begin
            detail<=CANCEL;state<=RESULT;
        end else begin
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    pos<=0;pos_p8_q<=20'd8;idx<=0;word_buf<=0;seen_fmt<=0;seen_data<=0;
                    data_offset<=0;data_length<=0;detail<=OK;
                    flen<=file_length[19:0];
                    if(file_length>32'hf0000) begin detail<=FILE_LIMIT;state<=RESULT;end
                    else if(file_length<32'd12) begin detail<=TRUNC;state<=RESULT;end
                    else state<=RIFF;
                end
                BOUNDARY:if(PRODUCT_LOGIC_RESET) begin
                    boundary_range_bad_q<=pos>riff_end || riff_end-pos<20'd8;
                    boundary_end_q<=pos==riff_end;boundary_file_end_q<=pos==flen;
                    boundary_header_q<=!seen_data && pos_p8_q>20'd4096;state<=BOUND_CHECK;
                end else begin
                    idx<=0;word_buf<=0;
                    if(pos==riff_end) begin
                        if(!seen_data) begin detail<=ORDER;state<=RESULT;end
                        else if(pos==flen) state<=RESULT;
                        else state<=TAIL; // RIFF外字节仍消费到目录EOF，供上游CRC覆盖。
                    end else if(pos>riff_end || riff_end-pos<20'd8) begin
                        detail<=RANGE;state<=RESULT;
                    end else if(!seen_data && pos+20'd8>20'd4096) begin
                        detail<=HEADER_LIMIT;state<=RESULT;
                    end else state<=CHDR;
                end
                BOUND_CHECK:if(PRODUCT_LOGIC_RESET) begin
                    idx<=0;word_buf<=0;
                    if(boundary_end_q) begin
                        if(!seen_data) begin detail<=ORDER;state<=RESULT;end
                        else if(boundary_file_end_q) state<=RESULT;
                        else state<=TAIL; // RIFF外字节仍消费到目录EOF，供上游CRC覆盖。
                    end else if(boundary_range_bad_q) begin
                        detail<=RANGE;state<=RESULT;
                    end else if(boundary_header_q) begin
                        detail<=HEADER_LIMIT;state<=RESULT;
                    end else state<=CHDR;
                end
                CHUNK_PAD:if(PRODUCT_LOGIC_RESET) begin
                    chunk_compact_q<={((|chunk_size[31:20]) || chunk_low_sum[20]),chunk_low_sum[19:0]};body_sum_q<=pos+chunk_size[19:0];state<=CHUNK_SUM;
                end
                CHUNK_SUM:if(PRODUCT_LOGIC_RESET) begin
                    end_compact_q<={(chunk_compact_q[20] || end_low_sum[20]),end_low_sum[19:0]};state<=CHUNK_RANGE;
                end
                CHUNK_RANGE:if(PRODUCT_LOGIC_RESET) begin
                    body_penultimate_q<=body_sum_q-20'd2;
                    range_bad_q<=end_compact_q>{13'd0,riff_end};
                    header_bad_q<=!seen_data && ((chunk_data_tag && pos>20'd4096) ||
                        (!chunk_data_tag && end_compact_q>33'd4088));
                    state<=CHECK;
                end
                RIFF_SUM:if(PRODUCT_LOGIC_RESET) begin
                    riff_compact_q<={((|word_buf[31:20]) || riff_low_sum[20]),riff_low_sum[19:0]};state<=RIFF_RANGE;
                end
                RIFF_RANGE:if(PRODUCT_LOGIC_RESET) begin
                    if(riff_compact_q>{13'd0,flen} || riff_compact_q<33'd12) begin detail<=RANGE;state<=RESULT;end
                    else begin riff_end<=riff_compact_q[19:0];state<=RIFF;end
                end
                CHECK:begin
                    is_fmt<=chunk_fmt_tag;is_data<=chunk_data_tag;
                    body_end<=PRODUCT_LOGIC_RESET ? body_sum_q : pos+chunk_size[19:0];odd<=chunk_size[0];idx<=0;fmt_bad<=0;
                    if(check_range_bad) begin detail<=RANGE;state<=RESULT;end
                    else if(chunk_fmt_tag && seen_fmt) begin detail<=DUP;state<=RESULT;end
                    else if(chunk_data_tag && seen_data) begin detail<=DUP;state<=RESULT;end
                    else if(chunk_fmt_tag && fmt_short) begin detail<=FMT;state<=RESULT;end
                    else if(chunk_data_tag && !seen_fmt) begin detail<=ORDER;state<=RESULT;end
                    else if(check_header_bad) begin
                        detail<=HEADER_LIMIT;state<=RESULT;
                    end else if(chunk_data_tag &&
                            data_bad) begin
                        detail<=DATA_LENGTH;state<=RESULT;
                    end else begin
                        if(chunk_data_tag) begin
                            seen_data<=1;data_offset<=pos;data_length<=chunk_size[19:0];
                        end
                        if(size_zero) state<=BOUNDARY;
                        else state<=BODY;
                    end
                end
                RESULT:if(result_valid && result_ready) state<=IDLE;
                default:if(byte_valid && byte_ready) begin
                    pos<=pos+20'd1;
                    pos_p8_q<=pos_p8_q+20'd1;
                    // last 与预期文件末尾不一致时进入错误状态；字段寄存器可在同拍更新，
                    // 但不会发布有效解析结果。
                    case(state)
                        RIFF:begin
                            word_buf<=assembled;idx<=idx+4'd1;
                            if((idx<4 || idx>=8) && byte_data!=magic_expected) begin
                                detail<=MAGIC;state<=RESULT;
                            end else if(idx==7) begin
                                if(PRODUCT_LOGIC_RESET) state<=RIFF_SUM;
                                else if(riff_total>{13'd0,flen} || riff_total<33'd12) begin
                                    detail<=RANGE;state<=RESULT;
                                end else riff_end<=riff_total[19:0];
                            end else if(idx==11) state<=BOUNDARY;
                        end
                        CHDR:begin
                            word_buf<=assembled;idx<=idx+4'd1;
                            if(idx==3) begin chunk_fmt_tag<=assembled==32'h20746d66;chunk_data_tag<=assembled==32'h61746164;end
                            if(idx==7) begin chunk_size_low<=assembled[19:0];chunk_size_big<=|assembled[31:20];state<=PRODUCT_LOGIC_RESET ? CHUNK_PAD : CHECK;end
                        end
                        BODY:begin
                            if(is_fmt && idx<16) idx<=idx+5'd1;
                            if(is_fmt && idx<16 && byte_data!=fmt_expected)
                                fmt_bad<=1;
                            if(current_body_last) begin
                                if(is_fmt && (fmt_bad || (idx<16 && byte_data!=fmt_expected))) begin
                                    detail<=FMT;state<=RESULT;
                                end else begin
                                    if(is_fmt) seen_fmt<=1;
                                    state<=odd?PAD:BOUNDARY;
                                end
                            end
                        end
                        PAD:state<=BOUNDARY;
                        TAIL:if(byte_last) state<=RESULT;
                        default:begin detail<=PROTOCOL;state<=RESULT;end
                    endcase
                    // last必须与目录长度一致；缺失last也不能以计数掩盖协议缺陷。
                    if(byte_last!=current_file_last) begin
                        detail<=byte_last?TRUNC:PROTOCOL;state<=RESULT;
                    end
                end
            endcase
        end
    end
endmodule
`default_nettype wire
