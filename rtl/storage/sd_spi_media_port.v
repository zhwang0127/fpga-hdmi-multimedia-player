`timescale 1ns/1ps
`default_nettype none

// 初始化及 CSD 读取结束、SPI 停止后，才允许扇区读取模块使用总线。
// 初始化请求由上层FPGA启动/RESCAN控制模块发出。本层不高频自动探卡，也不含FAT32。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_spi_media_port #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    parameter integer KEY_W=32,
    // CSD容量上限位数（默认 33 位，不加额外上限）。
    parameter integer CARD_SECTOR_W=33,
    parameter USE_EG_BRAM=1,SHARE_CONTEXT=0,POWER_CYCLES=50000,POLL_CYCLES=50000,
    parameter INIT_TIMEOUT_CYCLES=50000000,READ_WAIT_CYCLES=5000000
) (
    input wire clk,arst,
    input wire init_valid,output wire init_ready,input wire [KEY_W-1:0] init_epoch,
    input wire init_abort_valid,output wire init_abort_ready,input wire [KEY_W-1:0] init_abort_epoch,
    output wire init_done_valid,input wire init_done_ready,
    output wire [KEY_W-1:0] init_done_epoch, output wire [31:0] init_done_register,
    output wire [3:0] init_done_code,output wire [5:0] init_done_command,output wire [7:0] init_done_r1,
    output wire card_qualified,card_block_addressed,media_fault,
    output wire card_initialized,output wire [31:0] card_last_lba,output wire [32:0] card_sector_count,
    input wire req_valid,output wire req_ready,input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_lba,
    input wire cancel_valid,output wire cancel_ready,input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire data_valid,input wire data_ready,output wire [7:0] data,output wire [8:0] data_index,
    output wire data_last,output wire [KEY_W-1:0] data_session,data_seq,
    output wire result_valid,input wire result_ready,output wire [KEY_W-1:0] result_session,result_seq,
    output wire [3:0] result_code,output wire [7:0] result_detail,output wire [9:0] result_delivered,
    output wire result_card_fault,
    output wire spi_cs_n,spi_sclk,spi_mosi,input wire spi_miso,
    input wire context_claim,output wire context_available,
    input wire context_wr_enable,context_rd_enable,
    input wire [7:0] context_wr_addr,context_rd_addr,context_wr_data,
    output wire [7:0] context_rd_data
);
    wire rst_logic;
    localparam OFFLINE=0,WAIT_INIT=1,START_INIT=2,RUN_INIT=3,RELEASE_INIT=4,
               WAIT_READ=5,REQUALIFY=6,READABLE=7,RELEASE_READ=8,
               START_CSD=9,RUN_CSD=10,REPORT_INIT=11;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    reg [3:0] phase;
    reg [KEY_W-1:0] pending_epoch;
    reg init_ok,addressed,initialized;
    reg [3:0] init_code;
    wire [31:0] i_register;
    wire [5:0] i_command;
    wire [7:0] i_r1;
    wire [7:0] c_r1;                       // CSD 客户端的 R1 响应字节。
    wire i_grant,r_grant,i_release_ready,r_release_ready,i_busy,r_busy,i_start_ready;
    wire i_done_valid,i_qualified,i_addressed,r_req_ready,r_cancel_ready,r_data_valid,r_result_valid;
    wire [3:0] i_code;
    assign init_ready=enabled && (phase==OFFLINE || (phase==READABLE && !r_busy));
    wire take_init=init_valid && init_ready;
    wire i_abort_ready,c_abort_ready;
    wire held_abort=enabled && phase==REPORT_INIT && init_code!=8 && init_abort_epoch==pending_epoch;
    wire abort_now=init_abort_valid && held_abort;
    assign init_abort_ready=phase==RUN_INIT ? i_abort_ready : phase==RUN_CSD ? c_abort_ready : held_abort;
    assign init_done_valid=enabled && phase==REPORT_INIT && !abort_now;
    assign init_done_epoch=pending_epoch;
    // 两协议客户端结束当前事务后保留诊断寄存器，到下一次start才改写；不复制整组快照。
    assign init_done_register=i_register;
    assign init_done_code=init_code;
    assign init_done_command=initialized ? 6'd9 : i_command;
    assign init_done_r1=initialized ? c_r1 : i_r1;
    assign card_initialized=enabled && initialized;
    assign card_sector_count={1'b0,card_last_lba}+33'd1;
    assign card_qualified=enabled && phase==READABLE && r_grant && init_ok && !media_fault;
    assign card_block_addressed=addressed;
    // 初始化与新的读请求同沿时优先接受初始化；在途读/待取结果必须先取消并消费。
    assign req_ready=card_qualified && !init_valid && r_req_ready;
    assign cancel_ready=enabled && phase==READABLE && r_grant && r_cancel_ready;
    assign data_valid=enabled && phase==READABLE && r_grant && r_data_valid;
    assign result_valid=enabled && phase==READABLE && r_grant && r_result_valid;
    wire init_claim=phase==WAIT_INIT || phase==START_INIT || phase==RUN_INIT ||
                    phase==START_CSD || phase==RUN_CSD || phase==REPORT_INIT;
    wire read_claim=phase==WAIT_READ || phase==REQUALIFY || phase==READABLE;
    wire i_cs,r_cs,i_tx_valid,r_tx_valid,i_tx_ready,r_tx_ready,i_rx_valid,r_rx_valid,i_rx_ready,r_rx_ready;
    wire i_stop,r_stop,i_ack,r_ack;
    wire [7:0] i_tx,r_tx,i_rx,r_rx;
    wire [15:0] i_half,r_half;
    wire p_tx_valid,p_tx_ready,p_rx_valid,p_rx_ready,p_stop,p_ack,p_busy;
    wire [7:0] p_tx,p_rx;
    wire [15:0] p_half;
    wire [1:0] p_level;
    // CSD在初始化所有权内串行运行；切换只发生在初始化器已经真实停止后。
    wire csd_selected=phase==START_CSD || phase==RUN_CSD;
    wire j_cs,j_tx_valid,j_tx_ready,j_rx_valid,j_rx_ready,j_stop,j_ack;
    wire [7:0] j_tx,j_rx;
    wire [15:0] j_half;
    wire c_cs,c_tx_valid,c_rx_ready,c_stop,c_start_ready,c_done_valid;
    wire [7:0] c_tx;
    wire [15:0] c_half;
    wire [3:0] c_code;
    assign i_cs=csd_selected ? c_cs : j_cs;
    assign i_tx_valid=csd_selected ? c_tx_valid : j_tx_valid;
    assign i_tx=csd_selected ? c_tx : j_tx;
    assign i_half=csd_selected ? c_half : j_half;
    assign i_rx_ready=csd_selected ? c_rx_ready : j_rx_ready;
    assign i_stop=csd_selected ? c_stop : j_stop;
    assign j_tx_ready=!csd_selected && i_tx_ready;
    assign j_rx_valid=!csd_selected && i_rx_valid;
    assign j_rx=i_rx;assign j_ack=!csd_selected && i_ack;
    sd_csd_reader #(.KEY_W(KEY_W),.CARD_SECTOR_W(CARD_SECTOR_W),.WAIT_CYCLES(READ_WAIT_CYCLES),.TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES)) u_csd(
        .clk(clk),.arst(arst),.start_valid(enabled && phase==START_CSD && i_grant),.start_ready(c_start_ready),
        .start_epoch(pending_epoch),.block_addressed(addressed),
        .abort_valid(init_abort_valid && phase==RUN_CSD),.abort_ready(c_abort_ready),.abort_epoch(init_abort_epoch),
        .done_valid(c_done_valid),.done_ready(phase==RUN_CSD),.done_epoch(),.done_code(c_code),.done_r1(c_r1),
        .last_lba(card_last_lba),.sector_count(),.spi_cs_n(c_cs),.spi_tx_valid(c_tx_valid),
        .spi_tx_ready(csd_selected && i_tx_ready),.spi_tx_data(c_tx),.spi_tx_half_period(c_half),
        .spi_rx_valid(csd_selected && i_rx_valid),.spi_rx_ready(c_rx_ready),.spi_rx_data(i_rx),
        .spi_stop_req(c_stop),.spi_stop_ack(csd_selected && i_ack));
    sd_spi_initializer #(.KEY_W(KEY_W),.POWER_CYCLES(POWER_CYCLES),.POLL_CYCLES(POLL_CYCLES),
                         .TIMEOUT_CYCLES(INIT_TIMEOUT_CYCLES)) u_init(
        .clk(clk),.arst(arst),.start_valid(enabled && phase==START_INIT && i_grant),.start_ready(i_start_ready),
        .start_epoch(pending_epoch),.abort_valid(init_abort_valid && phase==RUN_INIT),.abort_ready(i_abort_ready),
        .abort_epoch(init_abort_epoch),.done_valid(i_done_valid),.done_ready(phase==RUN_INIT),
        .done_epoch(),.done_register(i_register),.done_code(i_code),
        .done_command(i_command),.done_r1(i_r1),.qualified(i_qualified),.block_addressed(i_addressed),
        .busy(i_busy),.spi_cs_n(j_cs),.spi_tx_valid(j_tx_valid),.spi_tx_ready(j_tx_ready),.spi_tx_data(j_tx),
        .spi_tx_half_period(j_half),.spi_rx_valid(j_rx_valid),.spi_rx_ready(j_rx_ready),.spi_rx_data(j_rx),
        .spi_stop_req(j_stop),.spi_stop_ack(j_ack));
    sd_spi_sector_reader #(.KEY_W(KEY_W),.USE_EG_BRAM(USE_EG_BRAM),.SHARE_CONTEXT(SHARE_CONTEXT),.WAIT_CYCLES(READ_WAIT_CYCLES)) u_reader(
        .clk(clk),.arst(arst),.card_qualified(enabled && init_ok && r_grant),
        .card_requalified(phase==REQUALIFY && r_grant),.media_fault(media_fault),
        .req_valid(req_valid && card_qualified && !init_valid),.req_ready(r_req_ready),
        .req_session(req_session),.req_seq(req_seq),.req_lba(req_lba),.req_block_addressed(addressed),.card_last_lba(card_last_lba),
        .cancel_valid(cancel_valid && phase==READABLE && r_grant),.cancel_ready(r_cancel_ready),
        .cancel_session(cancel_session),.cancel_seq(cancel_seq),.data_valid(r_data_valid),.data_ready(data_ready && phase==READABLE),
        .data(data),.data_index(data_index),.data_last(data_last),.data_session(data_session),.data_seq(data_seq),
        .result_valid(r_result_valid),.result_ready(result_ready && phase==READABLE),
        .result_session(result_session),.result_seq(result_seq),.result_code(result_code),
        .result_detail(result_detail),.result_delivered(result_delivered),.result_card_fault(result_card_fault),
        .context_claim(context_claim),.context_available(context_available),
        .context_wr_enable(context_wr_enable),.context_rd_enable(context_rd_enable),
        .context_wr_addr(context_wr_addr),.context_rd_addr(context_rd_addr),
        .context_wr_data(context_wr_data),.context_rd_data(context_rd_data),
        .busy(r_busy),.spi_cs_n(r_cs),.spi_tx_valid(r_tx_valid),.spi_tx_ready(r_tx_ready),.spi_tx_data(r_tx),
        .spi_tx_half_period(r_half),.spi_rx_valid(r_rx_valid),.spi_rx_ready(r_rx_ready),.spi_rx_data(r_rx),
        .spi_stop_req(r_stop),.spi_stop_ack(r_ack));
    sd_spi_port_arbiter u_arbiter(
        .clk(clk),.arst(arst),.init_claim(init_claim),.read_claim(read_claim),.init_grant(i_grant),.read_grant(r_grant),
        .init_release_valid(phase==RELEASE_INIT),.read_release_valid(phase==RELEASE_READ),
        .init_release_ready(i_release_ready),.read_release_ready(r_release_ready),.init_cs_n(i_cs),.read_cs_n(r_cs),
        .init_tx_valid(i_tx_valid),.read_tx_valid(r_tx_valid),.init_tx_ready(i_tx_ready),.read_tx_ready(r_tx_ready),
        .init_tx_data(i_tx),.read_tx_data(r_tx),.init_half_period(i_half),.read_half_period(r_half),
        .init_rx_valid(i_rx_valid),.read_rx_valid(r_rx_valid),.init_rx_ready(i_rx_ready),.read_rx_ready(r_rx_ready),
        .init_rx_data(i_rx),.read_rx_data(r_rx),.init_stop_req(i_stop),.read_stop_req(r_stop),.init_stop_ack(i_ack),.read_stop_ack(r_ack),
        .spi_cs_n(spi_cs_n),.spi_tx_valid(p_tx_valid),.spi_tx_ready(p_tx_ready),.spi_tx_data(p_tx),.spi_half_period(p_half),
        .spi_rx_valid(p_rx_valid),.spi_rx_ready(p_rx_ready),.spi_rx_data(p_rx),.spi_stop_req(p_stop),.spi_stop_ack(p_ack),
        .spi_busy(p_busy),.spi_rx_level(p_level));
    spi_byte_master u_spi(.clk(clk),.arst(arst),.tx_valid(p_tx_valid),.tx_ready(p_tx_ready),.tx_data(p_tx),
        .tx_half_period(p_half),.rx_valid(p_rx_valid),.rx_ready(p_rx_ready),.rx_data(p_rx),.rx_level(p_level),
        .stop_req(p_stop),.stop_ack(p_ack),.busy(p_busy),.spi_miso(spi_miso),.spi_sclk(spi_sclk),.spi_mosi(spi_mosi));
    always @(posedge clk or posedge rst) begin
        if(rst) begin phase<=OFFLINE;pending_epoch<=0;init_ok<=0;addressed<=0;initialized<=0;
            init_code<=0;end
        else begin
            case(phase)
                OFFLINE:if(take_init) begin pending_epoch<=init_epoch;init_ok<=0;initialized<=0;phase<=WAIT_INIT;end
                WAIT_INIT:if(i_grant) phase<=START_INIT;
                START_INIT:if(i_grant && i_start_ready) phase<=RUN_INIT;
                RUN_INIT:if(i_done_valid) begin
                    initialized<=i_code==0 && i_qualified;addressed<=i_addressed;
                    init_code<=i_code;
                    phase<=i_code==0 && i_qualified ? START_CSD : REPORT_INIT;
                end
                START_CSD:if(i_grant && c_start_ready) phase<=RUN_CSD;
                RUN_CSD:if(c_done_valid) begin
                    init_ok<=c_code==0;phase<=REPORT_INIT;
                    case(c_code)
                        0:init_code<=0;
                        7:init_code<=11;
                        8:init_code<=8;
                        9:init_code<=12;
                        10:init_code<=13;
                        11:init_code<=14;
                        5,6:init_code<=10;
                        default:init_code<=9;
                    endcase
                end
                REPORT_INIT:begin
                    if(init_done_valid && init_done_ready) phase<=RELEASE_INIT;
                    if(abort_now) begin init_code<=8;init_ok<=0;end
                end
                RELEASE_INIT:if(i_release_ready) phase<=init_ok ? WAIT_READ : OFFLINE;
                WAIT_READ:if(r_grant) phase<=REQUALIFY;
                REQUALIFY:phase<=READABLE;
                READABLE:if(take_init) begin pending_epoch<=init_epoch;init_ok<=0;initialized<=0;phase<=RELEASE_READ;end
                RELEASE_READ:if(r_release_ready) phase<=WAIT_INIT;
                default:begin phase<=OFFLINE;init_ok<=0;end
            endcase
        end
    end
endmodule
`default_nettype wire
