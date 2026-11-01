`timescale 1ns/1ps
`default_nettype none

// CMD9条件客户端：初始化成功后独占同一SPI业务口，读取16B CSD和线上CRC16。
// 容量先校验再发布；任何失败均由媒体控制模块要求完整重初始化。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_csd_reader #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32,
    // 容量上限 = 2^CARD_SECTOR_W 个扇区。默认 33 位可覆盖 CSD v2 的完整编码范围（最多 2^32 个扇区）。
    // 产品传27（64GiB）：v2 C_SIZE>=0x20000（(C_SIZE+1)*1024 > 2^27）在 CHECK 判 CAPACITY，与 v1 超限同一结果码，
    // 媒体控制模块把它当作初始化失败（重试后进入启动错误闪码状态）；同时 last_lba 高位恒0，下游比较器的高位由综合常量消除。
    // v1(SDSC) 最大 4153344 扇区 < 2^22，任何 CARD_SECTOR_W>=23 都不受影响。
    parameter integer CARD_SECTOR_W=33,
    parameter WAIT_CYCLES=5000000, TIMEOUT_CYCLES=50000000, R1_BYTES=16
)(
    input wire clk,arst,
    input wire start_valid,output wire start_ready,input wire [KEY_W-1:0] start_epoch,
    input wire block_addressed,
    input wire abort_valid,output wire abort_ready,input wire [KEY_W-1:0] abort_epoch,
    output wire done_valid,input wire done_ready,output wire [KEY_W-1:0] done_epoch,
    output wire [3:0] done_code,output wire [7:0] done_r1,
    output wire [31:0] last_lba,output wire [32:0] sector_count,
    output reg spi_cs_n,
    output wire spi_tx_valid,input wire spi_tx_ready,output wire [7:0] spi_tx_data,
    output wire [15:0] spi_tx_half_period,
    input wire spi_rx_valid,output wire spi_rx_ready,input wire [7:0] spi_rx_data,
    output wire spi_stop_req,input wire spi_stop_ack
);
    wire rst_logic;
    localparam IDLE=0,READY=1,COMMAND=2,R1=3,TOKEN=4,BLOCK=5,
               CHECK=6,SIZE=7,STOP=8,TAIL=9,FINAL_STOP=10,RESULT=11;
    localparam OK=0,READY_TIMEOUT=2,R1_TIMEOUT=3,R1_ERROR=4,TOKEN_TIMEOUT=5,
               TOKEN_ERROR=6,CRC_ERROR=7,CANCELED=8,FORMAT=9,CAPACITY=10,DEADLINE=11;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    reg [3:0] state,status;
    function integer counter_width;
        input integer value;
        integer v;
        begin v=value-1;counter_width=1;while(v>1) begin v=v>>1;counter_width=counter_width+1;end end
    endfunction
    localparam TIME_WIDTH=counter_width(TIMEOUT_CYCLES),R1_WIDTH=counter_width(R1_BYTES);
    reg [KEY_W-1:0] epoch;
    reg [TIME_WIDTH-1:0] elapsed;
    // 最大12位C_SIZE与最大11次移位共23位，完整容纳v1公式的所有编码值。
    reg [22:0] v1_limit;
    reg addressed,issued,aborted;
    reg [2:0] command_index;
    reg [R1_WIDTH-1:0] response_count;
    reg [7:0] r1,crc_high;
    reg [4:0] byte_index;
    reg [1:0] structure;
    reg [3:0] read_bl_len;
    reg [4:0] shifts;
    reg [2:0] size_mult;
    reg [11:0] size_v1;
    reg [21:0] size_v2;
    reg end_bit,format_bad;
    assign start_ready=enabled && state==IDLE && !abort_valid;
    assign abort_ready=enabled && state!=IDLE && !aborted && abort_epoch==epoch;
    wire abort_now=abort_valid && abort_ready;
    assign done_valid=enabled && state==RESULT && !abort_now;
    assign done_epoch=epoch;assign done_code=status;assign done_r1=r1;
    // inclusive上界完整覆盖2^32扇区；数量必须扩为33位后加1。
    // CARD_SECTOR_W<32 时只把 C_SIZE 低 (CARD_SECTOR_W-10) 位接到 last_lba：超限值已在 CHECK 判 CAPACITY，
    // 而失败结果里的容量本来就无效（下游只在 done_code==OK 时采用），所以这里不会把大卡“截断”成可用的小卡。
    wire [21:0] size_v2_cap=(CARD_SECTOR_W>=32) ? size_v2 : (size_v2 & ((22'd1<<(CARD_SECTOR_W-10))-22'd1));
    assign last_lba=addressed ? {size_v2_cap,10'h3ff} : {9'd0,v1_limit};
    assign sector_count={1'b0,last_lba}+33'd1;
    wire serial_state=state==READY || state==COMMAND || state==R1 || state==TOKEN || state==BLOCK || state==TAIL;
    assign spi_tx_valid=enabled && serial_state && !issued && !abort_now;
    assign spi_rx_ready=enabled && (serial_state || state==STOP || state==FINAL_STOP);
    assign spi_stop_req=enabled && (state==STOP || state==FINAL_STOP);
    assign spi_tx_half_period=16'd1;
    // 固定 CMD9 字节为 49 00 00 00 00 af，末字节含 CRC7，无需动态命令 CRC 组合器。
    assign spi_tx_data=state!=COMMAND ? 8'hff : command_index==0 ? 8'h49 : command_index==5 ? 8'haf : 8'h00;
    wire tx_fire=spi_tx_valid && spi_tx_ready,rx_fire=spi_rx_valid && spi_rx_ready;
    wire [15:0] crc;
    crc16_sd u_crc(.clk(clk),.rst(rst_logic),.clear(state!=BLOCK),
        .data_valid(rx_fire && state==BLOCK && byte_index<16),.data(spi_rx_data),.crc(crc));
    wire timed_out=state!=IDLE && state!=RESULT && status==OK && elapsed==TIMEOUT_CYCLES-1;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;status<=OK;epoch<=0;elapsed<=0;v1_limit<=0;
            addressed<=0;issued<=0;aborted<=0;command_index<=0;response_count<=0;r1<=8'hff;
            crc_high<=0;byte_index<=0;structure<=0;read_bl_len<=0;size_mult<=0;
            size_v1<=0;size_v2<=0;end_bit<=0;format_bad<=0;shifts<=0;spi_cs_n<=1;
        end else begin
            if(tx_fire) issued<=1;
            if(rx_fire) issued<=0;
            if(state!=IDLE && state!=RESULT && status==OK) elapsed<=elapsed+32'd1;
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    epoch<=start_epoch;addressed<=block_addressed;status<=OK;aborted<=0;issued<=0;
                    elapsed<=0;v1_limit<=0;command_index<=0;response_count<=0;r1<=8'hff;format_bad<=0;
                    spi_cs_n<=0;state<=READY;
                end
                READY:if(rx_fire && spi_rx_data==8'hff) state<=COMMAND;
                    // READY是首阶段，复用总截止计数器，避免另建23位等待计数器。
                    else if(elapsed==WAIT_CYCLES-1) begin status<=READY_TIMEOUT;state<=STOP;end
                COMMAND:if(rx_fire) begin
                    if(command_index==5) state<=R1;
                    else command_index<=command_index+3'd1;
                end
                R1:if(rx_fire) begin
                    if(!spi_rx_data[7]) begin
                        r1<=spi_rx_data;
                        if(spi_rx_data==0) begin state<=TOKEN;response_count<=0;end
                        else begin status<=R1_ERROR;state<=STOP;end
                    end else if(response_count==R1_BYTES-1) begin status<=R1_TIMEOUT;state<=STOP;end
                    else response_count<=response_count+8'd1;
                end
                TOKEN:if(rx_fire && spi_rx_data==8'hfe) begin byte_index<=0;state<=BLOCK;end
                    else if(rx_fire && spi_rx_data!=8'hff) begin status<=TOKEN_ERROR;state<=STOP;end
                    // 数据token不是NCR：等待按时间截止，不能用16字节窗口拒绝慢卡。
                    // 复用从CMD9阶段开始的计时器；默认100ms内完成READY/命令/token。
                    else if(elapsed==WAIT_CYCLES-1) begin status<=TOKEN_TIMEOUT;state<=STOP;end
                    else if(rx_fire) response_count<=response_count+1'b1;
                BLOCK:if(rx_fire) begin
                    // 只保留容量所需字段，不建128位整寄存器影子，不复制512B扇区缓冲区。
                    case(byte_index)
                        0:begin structure<=spi_rx_data[7:6];format_bad<=|spi_rx_data[5:0];end
                        5:begin read_bl_len<=spi_rx_data[3:0];if(!spi_rx_data[6]) format_bad<=1;end // CCC class2读块
                        6:begin
                            size_v1[11:10]<=spi_rx_data[1:0];
                            if(spi_rx_data[3:2]!=0 || (addressed ? spi_rx_data[7:5]!=0 : !spi_rx_data[7])) format_bad<=1;
                        end
                        7:begin
                            size_v1[9:2]<=spi_rx_data;size_v2[21:16]<=spi_rx_data[5:0];
                            if(addressed && spi_rx_data[7:6]!=0) format_bad<=1;
                        end
                        8:begin size_v1[1:0]<=spi_rx_data[7:6];size_v2[15:8]<=spi_rx_data;end
                        9:begin size_mult[2:1]<=spi_rx_data[1:0];size_v2[7:0]<=spi_rx_data;end
                        10:size_mult[0]<=spi_rx_data[7];
                        15:end_bit<=spi_rx_data[0];
                        16:crc_high<=spi_rx_data;
                        17:begin
                            state<=CHECK;
                            if({crc_high,spi_rx_data}!=crc) status<=CRC_ERROR;
                        end
                    endcase
                    byte_index<=byte_index+5'd1;
                end
                CHECK:begin
                    if(status!=OK) state<=STOP;
                    else if(format_bad || !end_bit || (addressed ? structure!=1 || read_bl_len!=9 : structure!=0 || read_bl_len<9 || read_bl_len>11)) begin
                        status<=FORMAT;state<=STOP;
                    end else if(addressed) begin
                        // (C_SIZE+1)*1024-1等价拼接，全范围包含ffffffff，不先截断数量。
                        // C_SIZE+1 > 2^(CARD_SECTOR_W-10) 即容量 > 2^CARD_SECTOR_W 扇区 -> CAPACITY（默认33不触发）。
                        if(CARD_SECTOR_W<32 && (size_v2>>(CARD_SECTOR_W-10))!=0) status<=CAPACITY;
                        state<=STOP;
                    end else begin
                        // v1数量=(C_SIZE+1)*2^(READ_BL_LEN+C_SIZE_MULT-7)。串行移位节省桶形器。
                        v1_limit<={11'd0,size_v1};shifts<={1'b0,read_bl_len}+{2'b0,size_mult}-5'd7;state<=SIZE;
                    end
                end
                SIZE:if(shifts!=0) begin v1_limit<={v1_limit[21:0],1'b1};shifts<=shifts-5'd1;end
                    else begin
                        if(v1_limit>23'd4153343) status<=CAPACITY; // 规范v1最大用户区，23位覆盖完整计算。
                        state<=STOP;
                    end
                STOP:if(spi_stop_ack) begin spi_cs_n<=1;issued<=0;state<=TAIL;end
                TAIL:if(rx_fire) state<=FINAL_STOP;
                FINAL_STOP:if(spi_stop_ack) state<=RESULT;
                RESULT:if(done_valid && done_ready) state<=IDLE;
                default:begin state<=IDLE;spi_cs_n<=1;end
            endcase
            // 超时/取消不能重复尾时钟，也不能跳过实际stop_ack。失败容量在结果中无效。
            if(timed_out || abort_now) begin
                status<=abort_now ? CANCELED : DEADLINE;
                if(abort_now) aborted<=1;
                case(state)
                    RESULT:state<=RESULT;
                    TAIL:state<=rx_fire ? FINAL_STOP : TAIL;
                    FINAL_STOP:state<=spi_stop_ack ? RESULT : FINAL_STOP;
                    default:state<=STOP;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
