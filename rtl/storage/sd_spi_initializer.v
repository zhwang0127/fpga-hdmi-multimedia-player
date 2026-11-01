`timescale 1ns/1ps
`default_nettype none

// sys域初始化客户端。start只在已独占SPI业务口后接受；不直接切换总线所有者。
// 正常路径：上电等待/80时钟/FLUSH排空+CMD0（非0x01或无回应最多重试 CMD0_TRIES 次）/CMD8/CMD59/CMD55+ACMD41/CMD58/[CMD16]。
// 超时/取消不直接清busy：真正收回SPI、CS高尾8时钟、再次停止后才发布带epoch结果。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_spi_initializer #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32,
    parameter POWER_CYCLES=50000,
    parameter POLL_CYCLES=50000,
    parameter TIMEOUT_CYCLES=50000000,
    parameter R1_BYTES=16,
    // CMD0 前排空 + CMD0 重试（见下方 FLUSH 说明）。
    parameter CMD0_TRIES=16,        // CMD0 最多发送次数（含第一次）
    parameter FLUSH_FF_RUN=16,      // 连续收到这么多个 0xFF 才认为卡已空闲
    parameter FLUSH_RETRY_MIN=600,  // 重试时至少排空的字节数（>512B块+2B CRC+令牌余量）
    parameter FLUSH_MAX=2048        // 单次排空上限字节数（400kHz 下约 41ms），超过也照发 CMD0
) (
    input wire clk,arst,
    input wire start_valid,output wire start_ready,input wire [KEY_W-1:0] start_epoch,
    input wire abort_valid,output wire abort_ready,input wire [KEY_W-1:0] abort_epoch,
    output wire done_valid,input wire done_ready,
    output wire [KEY_W-1:0] done_epoch, output wire [31:0] done_register,
    output wire [3:0] done_code,
    output wire [5:0] done_command,
    output wire [7:0] done_r1,
    output reg qualified,block_addressed,
    output wire busy,
    output reg spi_cs_n,
    output wire spi_tx_valid,input wire spi_tx_ready,
    output reg [7:0] spi_tx_data,output wire [15:0] spi_tx_half_period,
    input wire spi_rx_valid,output wire spi_rx_ready,input wire [7:0] spi_rx_data,
    output wire spi_stop_req,input wire spi_stop_ack
);
    wire rst_logic;
    localparam IDLE=0,POWER=1,CLOCK80=2,COMMAND=3,R1=4,EXTENSION=5,
               EVALUATE=6,STOP=7,TAIL=8,FINAL_STOP=9,PAUSE=10,RESULT=11,FLUSH=12;
    localparam OP0=0,OP8=1,OP59=2,OP55=3,OP41=4,OP58=5,OP16=6;
    localparam OK=0,HARD_TIMEOUT=1,R1_TIMEOUT=2,R1_ERROR=3,ECHO_ERROR=4,OCR_ERROR=5,CANCELED=8;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    reg [3:0] state,status;
    reg [2:0] operation,next_operation,command_index;
    reg [KEY_W-1:0] epoch; reg [31:0] extension; // extension 是R7/OCR数据移位寄存器，必须保留32位
    // 计数位宽 = 恰好容纳“阈值本身”的位数（counter_width(P+1)）。
    // 计数器在达到阈值前递增，到达后离开计数状态或置位完成标志
    // 位宽需覆盖阈值本身，避免计数到达上限前发生回绕。
    function integer counter_width;
        input integer value;
        integer v;
        begin v=value-1;counter_width=1;while(v>1) begin v=v>>1;counter_width=counter_width+1;end end
    endfunction
    // elapsed 在 finishing 置位（含 elapsed==TIMEOUT_CYCLES-1 硬超时）后停止计数，最大值≤TIMEOUT_CYCLES。
    localparam integer ELAPSED_W=counter_width(TIMEOUT_CYCLES+1);              // 产品50,000,000 -> 26位
    // delay_count 只服务 POWER / PAUSE 两个固定协议等待，到 X_CYCLES-1 即清零离开，延时以 clk 周期计数。
    localparam integer DELAY_W=counter_width((POWER_CYCLES>POLL_CYCLES ? POWER_CYCLES : POLL_CYCLES)+1); // 产品50,000 -> 16位
    reg [ELAPSED_W-1:0] elapsed; reg [DELAY_W-1:0] delay_count;
    reg [6:0] command_crc;
    reg [7:0] r1,response_count;
    reg [3:0] startup_bytes;
    reg [1:0] extension_index;
    reg issued,version2,finishing,aborted,pause_next;
    // FLUSH 在每次 CMD0 前保持 CS 低并发送 0xFF，排空未断电卡可能残留的读事务数据。
    // flush_count 统计排空字节，ff_run 统计连续 0xFF，cmd0_tries 统计已发送的 CMD0 次数。
    reg [11:0] flush_count;
    reg [4:0] ff_run;
    reg [3:0] cmd0_tries;
    // 第一次只要求连续 FLUSH_FF_RUN 个 0xFF（正常卡第一时间满足，几乎不增加启动时间）；
    // 重试时另要求至少排空 FLUSH_RETRY_MIN 字节，跨过可能含长串 0xFF 的残留数据块。
    wire [11:0] flush_min=cmd0_tries==0 ? 12'd0 : FLUSH_RETRY_MIN[11:0];
    wire cmd0_retry_left=cmd0_tries<CMD0_TRIES-1;
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    assign start_ready=enabled && state==IDLE && !abort_valid;
    assign abort_ready=enabled && state!=IDLE && !aborted && abort_epoch==epoch;
    wire abort_now=abort_valid && abort_ready;
    assign done_valid=enabled && state==RESULT && !abort_now;
    assign done_epoch=epoch;
    assign done_register=extension; // 最新R7/OCR；成功时保留OCR，不因CMD16擦除。
    assign done_code=status;
    assign done_r1=r1;
    assign busy=state!=IDLE;
    wire serial_state=state==CLOCK80 || state==COMMAND || state==R1 || state==EXTENSION || state==TAIL || state==FLUSH;
    assign spi_tx_valid=enabled && serial_state && !issued && !abort_now;
    assign spi_tx_half_period=16'd63; // 50MHz标称下396825.4Hz，直到初始化实际收尾后才换速。
    assign spi_rx_ready=enabled && (serial_state || state==STOP || state==FINAL_STOP);
    assign spi_stop_req=enabled && (state==STOP || state==FINAL_STOP);
    wire tx_fire=spi_tx_valid && spi_tx_ready;
    wire rx_fire=spi_rx_valid && spi_rx_ready;
    reg [5:0] command;
    reg [31:0] argument;
    assign done_command=command;
    always @* begin
        command=0;argument=0;
        case(operation)
            OP0:command=0;
            OP8:begin command=8;argument=32'h000001aa;end
            OP59:begin command=59;argument=1;end
            OP55:command=55;
            OP41:begin command=41;argument=version2 ? 32'h40000000 : 0;end
            OP58:command=58;
            OP16:begin command=16;argument=512;end
            default:command=0;
        endcase
        spi_tx_data=8'hff;
        if(state==COMMAND) case(command_index)
            0:spi_tx_data={2'b01,command};
            1:spi_tx_data=argument[31:24];
            2:spi_tx_data=argument[23:16];
            3:spi_tx_data=argument[15:8];
            4:spi_tx_data=argument[7:0];
            default:spi_tx_data={command_crc,1'b1};
        endcase
    end
    function [6:0] crc7_byte;
        input [6:0] previous;input [7:0] value;
        reg [6:0] remainder;integer k;
        begin
            remainder=previous;
            for(k=7;k>=0;k=k-1)
                if(remainder[6]^value[k]) remainder={remainder[5:0],1'b0}^7'h09;
                else remainder={remainder[5:0],1'b0};
            crc7_byte=remainder;
        end
    endfunction
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;status<=OK;operation<=OP0;next_operation<=OP0;command_index<=0;
            epoch<=0;elapsed<=0;delay_count<=0;extension<=0;command_crc<=0;r1<=8'hff;response_count<=0;
            startup_bytes<=0;extension_index<=0;issued<=0;version2<=0;finishing<=0;aborted<=0;pause_next<=0;
            qualified<=0;block_addressed<=0;spi_cs_n<=1;flush_count<=0;ff_run<=0;cmd0_tries<=0;
        end else begin
            if(state!=IDLE && state!=RESULT && !finishing) elapsed<=elapsed+1'b1;
            if(tx_fire) issued<=1;
            if(rx_fire && serial_state) issued<=0;
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    epoch<=start_epoch;elapsed<=0;delay_count<=0;status<=OK;qualified<=0;block_addressed<=0;
                    extension<=0;version2<=0;finishing<=0;aborted<=0;pause_next<=0;
                    operation<=OP0;next_operation<=OP0;command_index<=0;command_crc<=0;r1<=8'hff;
                    response_count<=0;startup_bytes<=0;issued<=0;spi_cs_n<=1;state<=POWER;
                    flush_count<=0;ff_run<=0;cmd0_tries<=0;
                end
                POWER:if(delay_count==POWER_CYCLES-1) begin delay_count<=0;state<=CLOCK80;end
                      else delay_count<=delay_count+1'b1;
                CLOCK80:if(rx_fire) begin
                    if(startup_bytes==9) begin next_operation<=OP0;state<=STOP;end
                    else startup_bytes<=startup_bytes+4'd1;
                end
                COMMAND:begin
                    if(tx_fire && command_index<5) command_crc<=crc7_byte(command_crc,spi_tx_data);
                    if(rx_fire) begin
                        if(command_index==5) begin response_count<=0;state<=R1;end
                        else command_index<=command_index+3'd1;
                    end
                end
                R1:if(rx_fire) begin
                    if(!spi_rx_data[7]) begin
                        r1<=spi_rx_data;
                        if((operation==OP8 && spi_rx_data==1) || (operation==OP58 && spi_rx_data==0)) begin
                            extension<=0;extension_index<=0;state<=EXTENSION;
                        end else state<=EVALUATE;
                    end else if(response_count==R1_BYTES-1) begin
                        // CMD0 无回应（卡仍在送残留块/令牌等待全是0xFF）：还有次数就重试，不收尾。
                        if(operation==OP0 && cmd0_retry_left) begin
                            cmd0_tries<=cmd0_tries+4'd1;next_operation<=OP0;pause_next<=0;state<=STOP;
                        end else begin status<=R1_TIMEOUT;finishing<=1;state<=STOP;end
                    end
                    else response_count<=response_count+8'd1;
                end
                EXTENSION:if(rx_fire) begin
                    extension<={extension[23:0],spi_rx_data};
                    if(extension_index==3) state<=EVALUATE;
                    else extension_index<=extension_index+2'd1;
                end
                EVALUATE:begin
                    state<=STOP;pause_next<=0;
                    case(operation)
                        // CMD0 回应不是 0x01（可能把残留数据字节当成了 R1）：还有次数就重发 CMD0。
                        OP0:if(r1==1) next_operation<=OP8;
                            else if(cmd0_retry_left) begin cmd0_tries<=cmd0_tries+4'd1;next_operation<=OP0;end
                            else begin status<=R1_ERROR;finishing<=1;end
                        OP8:if(r1==8'h05) begin version2<=0;next_operation<=OP59;end
                            else if(r1==1) begin
                                if(extension==32'h000001aa) begin version2<=1;next_operation<=OP59;end
                                else begin status<=ECHO_ERROR;finishing<=1;end
                            end else begin status<=R1_ERROR;finishing<=1;end
                        OP59:if(r1==1) next_operation<=OP55;
                             else begin status<=R1_ERROR;finishing<=1;end
                        OP55:if(r1==1) next_operation<=OP41;
                             else begin status<=R1_ERROR;finishing<=1;end
                        OP41:if(r1==1) begin next_operation<=OP55;pause_next<=1;end
                             else if(r1==0) next_operation<=OP58;
                             else begin status<=R1_ERROR;finishing<=1;end
                        OP58:begin
                            if(r1!=0) begin status<=R1_ERROR;finishing<=1;end
                            else if(!extension[31] || (extension & 32'h00300000)==0 || (!version2 && extension[30])) begin
                                status<=OCR_ERROR;finishing<=1;
                            end else if(version2 && extension[30]) begin block_addressed<=1;finishing<=1;end
                            else begin block_addressed<=0;next_operation<=OP16;end
                        end
                        OP16:begin finishing<=1;if(r1!=0) status<=R1_ERROR;end
                        default:begin status<=R1_ERROR;finishing<=1;end
                    endcase
                end
                STOP:if(spi_stop_ack) begin spi_cs_n<=1;issued<=0;state<=TAIL;end
                TAIL:if(rx_fire) state<=FINAL_STOP;
                FINAL_STOP:if(spi_stop_ack) begin
                    if(finishing) begin qualified<=status==OK;state<=RESULT;end
                    else if(pause_next) begin delay_count<=0;state<=PAUSE;end
                    else begin
                        operation<=next_operation;command_index<=0;command_crc<=0;response_count<=0;
                        // CMD0 先进 FLUSH（同一次 CS 低内排空后紧接 CMD0），其余命令照旧直接发。
                        flush_count<=0;ff_run<=0;
                        spi_cs_n<=0;state<=next_operation==OP0 ? FLUSH : COMMAND;
                    end
                end
                // 排空：满足“连续 FLUSH_FF_RUN 个 0xFF 且已排空 flush_min 字节”或达到 FLUSH_MAX 即发 CMD0。
                FLUSH:if(rx_fire) begin
                    flush_count<=flush_count+12'd1;
                    if(spi_rx_data==8'hff) begin if(ff_run!=5'd31) ff_run<=ff_run+5'd1;end
                    else ff_run<=0;
                    if((spi_rx_data==8'hff && {7'd0,ff_run}+12'd1>=FLUSH_FF_RUN && flush_count+12'd1>=flush_min) ||
                       flush_count+12'd1>=FLUSH_MAX)
                        state<=COMMAND;
                end
                PAUSE:if(delay_count==POLL_CYCLES-1) begin
                    operation<=next_operation;command_index<=0;command_crc<=0;response_count<=0;
                    spi_cs_n<=0;state<=COMMAND;
                end else delay_count<=delay_count+1'b1;
                RESULT:if(done_valid && done_ready) state<=IDLE;
                default:begin qualified<=0;status<=R1_ERROR;finishing<=1;state<=STOP;end
            endcase
            // 1s准入硬截止优先于未完成的成功；当前字节+CS尾时钟的实际排空另计。
            if(state!=IDLE && state!=RESULT && !finishing && elapsed==TIMEOUT_CYCLES-1) begin
                qualified<=0;status<=HARD_TIMEOUT;finishing<=1;
            end
            if(abort_now) begin
                aborted<=1;qualified<=0;status<=CANCELED;finishing<=1;
            end
            // 收尾是单调推进的协议阶段。已发送的尾时钟不能因取消/截止再发送一次。
            // 同拍保留TAIL的返回握手、STOP的停止确认；PAUSE已完成尾时钟，仅再确认停止。
            if(abort_now || (state!=IDLE && state!=RESULT && !finishing && elapsed==TIMEOUT_CYCLES-1)) begin
                case(state)
                    RESULT:state<=RESULT;
                    TAIL:state<=rx_fire ? FINAL_STOP : TAIL;
                    FINAL_STOP:begin
                        spi_cs_n<=1;operation<=operation;
                        state<=spi_stop_ack ? RESULT : FINAL_STOP;
                    end
                    PAUSE:begin spi_cs_n<=1;operation<=operation;state<=FINAL_STOP;end
                    STOP:state<=spi_stop_ack ? TAIL : STOP;
                    default:state<=STOP;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
