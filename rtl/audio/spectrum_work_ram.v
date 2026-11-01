`timescale 1ns/1ps
`default_nettype none
// 频谱专用同步工作 RAM，容量 512×18 位，读延迟一拍，存储阵列不复位。
// 原生分支使用 PDPW8K 模式，dia/dib 各承载 9 位写数据，doa/dob 拼接为 18 位读数据。
// 地址低 4 位为宽度及字节选择，写完整 18 位时置全一。
// 全部端口运行在 pix 上升沿，FFT 调度器先写后读，并避免同地址读写冲突。
// USE_EG_BRAM=0 选择同一读写时序的行为 RAM。
module spectrum_work_ram #(parameter USE_EG_BRAM=1)(
    input wire clk,wr_enable,
    input wire [8:0] wr_addr,rd_addr,
    input wire [17:0] wr_data,
    output wire [17:0] rd_data
);
    generate if(USE_EG_BRAM) begin : anlogic
        EG_PHY_BRAM #(.MODE("PDPW8K"),.DATA_WIDTH_A("18"),.DATA_WIDTH_B("18"),
            .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.RESETMODE("SYNC"),
            .CEAMUX("SIG"),.CEBMUX("1"),.OCEAMUX("1"),.OCEBMUX("1"),
            .RSTAMUX("0"),.RSTBMUX("0"),.WEAMUX("1"),.WEBMUX("0"),
            .CSA0("1"),.CSA1("1"),.CSA2("1"),.CSB0("1"),.CSB1("1"),.CSB2("1")) u_ram(
            .clka(clk),.cea(wr_enable),.wea(1'b1),.addra({wr_addr,4'b1111}),
            .dia(wr_data[8:0]),.dib(wr_data[17:9]),.csa(3'b111),.rsta(1'b0),.ocea(1'b1),
            .clkb(clk),.ceb(1'b1),.web(1'b0),.addrb({rd_addr,4'b1111}),
            .csb(3'b111),.rstb(1'b0),.oceb(1'b1),.doa(rd_data[8:0]),.dob(rd_data[17:9]));
    end else begin : portable
        reg [17:0] memory[0:511];
        reg [17:0] q;
        always @(posedge clk) begin
            if(wr_enable) memory[wr_addr]<=wr_data;
            q<=memory[rd_addr];
        end
        assign rd_data=q;
    end endgenerate
endmodule
`default_nettype wire
