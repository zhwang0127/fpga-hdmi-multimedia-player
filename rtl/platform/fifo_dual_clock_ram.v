`timescale 1ns/1ps
`default_nettype none

// 平台边界：一拍同步读、读使能为0时保持输出，不初始化、不清空存储阵列。
// 默认使用 TD RAM 原语；USE_EG_BRAM=0 选择相同读写契约的行为 RAM。
module fifo_dual_clock_ram #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 11,
    parameter USE_EG_BRAM = 1,
    parameter RAM_IMPLEMENT = "32K",
    parameter PHYSICAL_BYTE = 0
) (
    input wire wr_clk, wr_enable,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire [DATA_WIDTH-1:0] wr_data,
    input wire rd_clk, rd_enable,
    input wire [ADDR_WIDTH-1:0] rd_addr,
    output wire [DATA_WIDTH-1:0] rd_data
);
    localparam DEPTH = 1 << ADDR_WIDTH;
    generate
        if (USE_EG_BRAM && PHYSICAL_BYTE) begin : physical_byte
            // 显式使用双端口 RAM，保证该配置的读写端口和字节写使能。
            // 固定1024x8场景直接定义DP8K物理形状；其余配置使用通用器件 RAM 宏。
            if (DATA_WIDTH!=8 || ADDR_WIDTH!=10) begin : invalid_shape
                invalid_physical_byte_ram_shape u_error();
            end
            wire [8:0] byte_data;
            assign rd_data=byte_data[7:0];
            EG_PHY_BRAM #(.MODE("DP8K"),.DATA_WIDTH_A("9"),.DATA_WIDTH_B("9"),
                .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
                .RESETMODE("SYNC"),.OCEAMUX("1"),.OCEBMUX("1"),.RSTAMUX("0"),.RSTBMUX("0"),.WEBMUX("0")) u_ram(
                .clka(wr_clk),.cea(wr_enable),.wea(wr_enable),.addra({wr_addr,3'b111}),.dia({1'b0,wr_data}),
                .csa(3'b111),.ocea(1'b1),.rsta(1'b0),.doa(),
                .clkb(rd_clk),.ceb(rd_enable),.web(1'b0),.addrb({rd_addr,3'b111}),.dib(9'd0),
                .csb(3'b111),.oceb(1'b1),.rstb(1'b0),.dob(byte_data));
        end else if (USE_EG_BRAM) begin : anlogic
            // 原生 RAM 的端口和宽度配置集中在此，业务模块使用统一的读写接口。
            // NOREG 指没有额外输出寄存器；BRAM 的固有同步读延迟仍为一拍。
            EG_LOGIC_BRAM #(
                .DATA_WIDTH_A(DATA_WIDTH), .DATA_WIDTH_B(DATA_WIDTH),
                .ADDR_WIDTH_A(ADDR_WIDTH), .ADDR_WIDTH_B(ADDR_WIDTH),
                .DATA_DEPTH_A(DEPTH), .DATA_DEPTH_B(DEPTH),
                .BYTE_ENABLE(0), .MODE("DP"),
                .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
                .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
                .RESETMODE("SYNC"), .INIT_FILE("NONE"), .FILL_ALL("NONE"),
                .IMPLEMENT(RAM_IMPLEMENT)
            ) u_ram (
                .clka(wr_clk), .cea(wr_enable), .wea(wr_enable),
                .addra(wr_addr), .dia(wr_data), .bea(1'b1),
                .rsta(1'b0), .ocea(1'b1), .doa(),
                .clkb(rd_clk), .ceb(rd_enable), .web(1'b0),
                .addrb(rd_addr), .dib({DATA_WIDTH{1'b0}}), .beb(1'b1),
                .rstb(1'b0), .oceb(1'b1), .dob(rd_data)
            );
        end else begin : portable
            // 行为 RAM：写端按 wr_clk 更新，读端按 rd_clk 同步读取，未使能时保持输出。
            reg [DATA_WIDTH-1:0] memory [0:DEPTH-1];
            reg [DATA_WIDTH-1:0] read_register;
            always @(posedge wr_clk)
                if (wr_enable) memory[wr_addr] <= wr_data;
            always @(posedge rd_clk)
                if (rd_enable) read_register <= memory[rd_addr];
            assign rd_data = read_register;
        end
    endgenerate
endmodule

`default_nettype wire
