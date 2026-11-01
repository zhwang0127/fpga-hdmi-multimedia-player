`timescale 1ns/1ps
`default_nettype none

// 初始化/读取两客户端独占同一SPI字节引擎；交接通过实际stop_ack，不能定时猜空。
// claim保持到grant；release必须valid/ready握手，只有CS高/无字节/无结果时才接受。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sd_spi_port_arbiter(
    input wire clk,arst,
    input wire init_claim,read_claim,output wire init_grant,read_grant,
    input wire init_release_valid,read_release_valid,output wire init_release_ready,read_release_ready,
    input wire init_cs_n,read_cs_n,
    input wire init_tx_valid,read_tx_valid,output wire init_tx_ready,read_tx_ready,
    input wire [7:0] init_tx_data,read_tx_data,
    input wire [15:0] init_half_period,read_half_period,
    output wire init_rx_valid,read_rx_valid,input wire init_rx_ready,read_rx_ready,
    output wire [7:0] init_rx_data,read_rx_data,
    input wire init_stop_req,read_stop_req,output wire init_stop_ack,read_stop_ack,
    output reg spi_cs_n,
    output wire spi_tx_valid,input wire spi_tx_ready,output wire [7:0] spi_tx_data,
    output wire [15:0] spi_half_period,
    input wire spi_rx_valid,output wire spi_rx_ready,input wire [7:0] spi_rx_data,
    output wire spi_stop_req,input wire spi_stop_ack,
    input wire spi_busy,input wire [1:0] spi_rx_level
);
    wire rst_logic;
    localparam NONE=0,INIT=1,READ=2,DRAIN=3;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    reg [1:0] owner;
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    assign init_grant=enabled && owner==INIT;
    assign read_grant=enabled && owner==READ;
    wire desired_cs=(owner==INIT) ? init_cs_n : (owner==READ ? read_cs_n : 1'b1);
    wire cs_settled=desired_cs==spi_cs_n;
    wire physically_empty=!spi_busy && spi_rx_level==0;
    assign init_release_ready=init_grant && init_cs_n && spi_cs_n && !init_tx_valid && physically_empty;
    assign read_release_ready=read_grant && read_cs_n && spi_cs_n && !read_tx_valid && physically_empty;
    wire release_owner=(init_release_valid && init_release_ready) || (read_release_valid && read_release_ready);
    wire selected_tx=(owner==INIT) ? init_tx_valid : read_tx_valid;
    assign spi_tx_valid=enabled && (init_grant || read_grant) && cs_settled && !release_owner && selected_tx;
    assign spi_tx_data=init_grant ? init_tx_data : read_tx_data;
    assign spi_half_period=init_grant ? init_half_period : read_half_period;
    assign init_tx_ready=init_grant && cs_settled && !release_owner && spi_tx_ready;
    assign read_tx_ready=read_grant && cs_settled && !release_owner && spi_tx_ready;
    assign init_rx_valid=init_grant && spi_rx_valid;
    assign read_rx_valid=read_grant && spi_rx_valid;
    assign init_rx_data=spi_rx_data;
    assign read_rx_data=spi_rx_data;
    assign spi_rx_ready=enabled && (init_grant ? init_rx_ready : (read_grant ? read_rx_ready : 1'b1));
    assign spi_stop_req=enabled && (init_grant ? init_stop_req : (read_grant ? read_stop_req : 1'b1));
    assign init_stop_ack=init_grant && spi_stop_ack;
    assign read_stop_ack=read_grant && spi_stop_ack;
    always @(posedge clk or posedge rst) begin
        if(rst) begin owner<=NONE;spi_cs_n<=1;end
        else begin
            // 客户端必须先停流再改CS；仲裁器另挡住仍在途的物理片选变化。
            if(physically_empty) spi_cs_n<=desired_cs;
            case(owner)
                NONE:if(spi_stop_ack && physically_empty && spi_cs_n) begin
                    if(init_claim) owner<=INIT; // 同时请求时初始化优先，只在空闲仲裁。
                    else if(read_claim) owner<=READ;
                end
                INIT,READ:if(release_owner) owner<=DRAIN;
                DRAIN:if(spi_stop_ack && physically_empty && spi_cs_n) owner<=NONE;
                default:owner<=DRAIN;
            endcase
        end
    end
endmodule
`default_nettype wire
