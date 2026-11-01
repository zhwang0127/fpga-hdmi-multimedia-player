`timescale 1ns/1ps
`default_nettype none

// 全部逻辑由clk驱动，SCLK只是寄存器输出；Mode0，MSB先发，全双工。
// 两字节接收信用含一条在途，启动后不因rx反压暂停半字节。
// stop只停止字节引擎，不代替SD的CMD12/忙等待/CS收尾或完整事务ack。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module spi_byte_master(
    input wire clk, arst,
    input wire tx_valid, output wire tx_ready,
    input wire [7:0] tx_data,
    // 每半SCLK的系统周期数，接受时锁存；0规整为1，16bit无符号。
    input wire [15:0] tx_half_period,
    output wire rx_valid, input wire rx_ready, output wire [7:0] rx_data,
    output wire [1:0] rx_level,
    input wire stop_req, output wire stop_ack, output wire busy,
    input wire spi_miso,
    (* allow_forward_retiming="false", allow_backward_retiming="false" *) output reg spi_sclk,
    (* allow_forward_retiming="false", allow_backward_retiming="false" *) output reg spi_mosi
);
    wire rst_logic;
    wire rst;
    reset_sync u_reset(.clk(clk), .arst(arst), .rst(rst),.logic_rst(rst_logic));
    reg active, stop_pending;
    reg [15:0] half_period, remaining;
    reg [2:0] bit_index;
    reg [7:0] tx_shift, rx_shift;
    reg [7:0] response0, response1;
    reg write_slot, read_slot;
    reg [1:0] queued;
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    assign busy=active;
    assign rx_valid=enabled && queued!=0;
    assign rx_data=read_slot ? response1 : response0;
    assign rx_level=queued;
    wire pop=rx_valid && rx_ready;
    wire final_fall=active && remaining==16'd1 && spi_sclk && bit_index==3'd0;
    // 边界同时产生本字节结果；还需为下一在途字节保留一个结果槽。
    wire can_chain=(queued==0 || (queued==1 && pop));
    assign tx_ready=enabled && !stop_req && !stop_pending &&
                    ((!active && (queued<2 || pop)) || (final_fall && can_chain));
    wire take=tx_valid && tx_ready;
    assign stop_ack=enabled && (stop_pending || stop_req) && !active && queued==0;
    wire [15:0] new_half_period=tx_half_period==0 ? 16'd1 : tx_half_period;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            active<=0;stop_pending<=0;half_period<=1;remaining<=0;bit_index<=7;
            tx_shift<=0;rx_shift<=0;spi_sclk<=0;spi_mosi<=1;
            queued<=0;write_slot<=0;read_slot<=0;
            // 不依赖接收数据复位；有效计数隔离旧内容，减少无效复位负担。
        end else begin
            if(stop_req) stop_pending<=1;
            else if(!active && queued==0) stop_pending<=0;
            if(pop) read_slot<=!read_slot;
            if(final_fall) begin
                write_slot<=!write_slot;
            end
            case({final_fall,pop})
                2'b10: queued<=queued+2'd1;
                2'b01: queued<=queued-2'd1;
                default: queued<=queued;
            endcase
            if(active) begin
                if(remaining==16'd1) begin
                    remaining<=half_period;
                    if(!spi_sclk) begin
                        spi_sclk<=1;
                        rx_shift<={rx_shift[6:0],spi_miso};
                    end else begin
                        spi_sclk<=0;
                        if(bit_index==3'd0) begin
                            active<=0;remaining<=0;spi_mosi<=1;
                        end else begin
                            bit_index<=bit_index-3'd1;
                            tx_shift<={tx_shift[6:0],1'b1};
                            spi_mosi<=tx_shift[6];
                        end
                    end
                end else remaining<=remaining-16'd1;
            end
            // 同沿最后下降沿与新请求可连续拼接；新字节首位预置于低电平阶段。
            if(take) begin
                active<=1;half_period<=new_half_period;remaining<=new_half_period;
                bit_index<=7;tx_shift<=tx_data;rx_shift<=0;spi_mosi<=tx_data[7];
            end
        end
    end
    // 数据槽不复位，计数隔离旧内容；上升沿采样条件避免隐含的负沿rst生成业务CE。
    always @(posedge clk) begin
        if(!rst_logic && final_fall) begin
            if(write_slot) response1<=rx_shift;else response0<=rx_shift;
        end
    end
endmodule

`default_nettype wire
