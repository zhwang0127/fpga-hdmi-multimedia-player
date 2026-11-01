`timescale 1ns/1ps
`default_nettype none
// 合封 SDRAM 的平台边界：接收控制器命令及实际写拍，连接 EG 器件原语。
// clk_mem 驱动控制器和本层寄存器，clk_device 直接送 SDRAM；两者相位由顶层 PLL 配置。
// REGISTERED=1 时命令/地址/写数据寄存一拍，读 DQ 再采样一拍，控制器须设 PHY_RD_EXTRA=2。
// REGISTERED=0 为组合接口；整机显式启用寄存接口。
// arst 异步使命令进入 NOP 并关闭 FPGA DQ 驱动，同步释放期间继续保持 NOP 和 DQ 高阻。
// 控制器、PHY 及外围有效槽须共同复位，重新初始化前不得发布在途 BL8 的数据。
// chip_* 是原语侧逻辑节点；合封器件的固定控制连线不替代复位期间的 NOP 命令及 DQ 高阻控制。
// USE_EG_SDRAM=1 使用器件原语；=0 留出接口供行为模型连接。
`ifndef SDRAM_PHY_REG_DEFAULT
`define SDRAM_PHY_REG_DEFAULT 0
`endif
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sdram_phy #(
    parameter USE_EG_SDRAM=1,
    parameter REGISTERED=`SDRAM_PHY_REG_DEFAULT
)(
    input wire clk_mem,clk_device,arst,
    input wire cs_n,ras_n,cas_n,we_n,cke,
    input wire [10:0] addr,input wire [1:0] ba,
    input wire wr_beat,input wire [31:0] wr_data,
    output wire [31:0] rd_data
);
    // 异步申请立即关总线，三级同步释放；控制器须用同一个arst共同重启初始化。
    // 暂停/复位不能保留半个BL8作为有效槽，外围所有引用/有效标签也必须清掉。
    wire rst_logic;
    wire rst;
    reset_sync u_reset(.clk(clk_mem),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    wire enabled=!rst_logic;
    wire chip_cs_n,chip_ras_n,chip_cas_n,chip_we_n,chip_cke;
    wire [10:0] chip_addr;wire [1:0] chip_ba;wire [3:0] chip_dm;wire chip_write_enable;
    wire [31:0] chip_dq;
    generate if(REGISTERED) begin: reg_io
        // 寄存级：arst 异步置 NOP；rst（同步释放期）同步保持 NOP。复位期间的器件输出由寄存器保持。
        reg q_cs_n,q_ras_n,q_cas_n,q_we_n,q_cke;reg [10:0] q_addr;reg [1:0] q_ba;reg [3:0] q_dm;
        reg [31:0] q_wdata,q_rdata;
        // 每个 DQ 位各有一个驱动使能寄存器，便于与写数据寄存器一起映射到器件接口。
        // q_oe 与 q_wdata 使用一致的时钟、使能和复位：arst 清零，复位释放期间保持，正常时装入 wr_beat。
        // 不使用 keep 阻止接口寄存器合并和打包；实际映射由器件工具决定。
        reg [31:0] q_oe;
        // 地址和 bank 与写数据使用相同的控制方式：arst 异步清零，rst 期间保持，正常拍更新。
        // 这样避免在地址输出前增加复位多路选择；命令线在复位期间仍保持 NOP。
        always @(posedge clk_mem or posedge arst)
            if(arst) begin
                q_cs_n<=1;q_ras_n<=1;q_cas_n<=1;q_we_n<=1;q_cke<=0;q_addr<=0;q_ba<=0;q_dm<=4'b1111;q_wdata<=0;
            end else if(rst_logic) begin
                q_cs_n<=1;q_ras_n<=1;q_cas_n<=1;q_we_n<=1;q_cke<=0;q_dm<=4'b1111;
            end else begin
                q_cs_n<=cs_n;q_ras_n<=ras_n;q_cas_n<=cas_n;q_we_n<=we_n;q_cke<=cke;q_addr<=addr;q_ba<=ba;
                q_dm<=4'b0000;q_wdata<=wr_data;
            end
        always @(posedge clk_mem or posedge arst)
            if(arst) q_oe<=32'd0;
            else if(!rst_logic) q_oe<={32{wr_beat}};
`ifdef FORMAL
        // 参考寄存器按相同写拍和复位条件更新，检查各 DQ 驱动位是否一致。
        reg f_q_we;
        always @(posedge clk_mem or posedge arst)
            if(arst) f_q_we<=0; else if(rst) f_q_we<=0; else f_q_we<=wr_beat;
        always @* assert(q_oe=={32{f_q_we}});
        // 参考地址寄存器在 rst 期间清零，用于检查异步清零后保持方式的有效输出。
        reg [10:0] f_q_addr;reg [1:0] f_q_ba;
        always @(posedge clk_mem or posedge arst)
            if(arst) begin f_q_addr<=0;f_q_ba<=0; end
            else if(rst) begin f_q_addr<=0;f_q_ba<=0; end
            else begin f_q_addr<=addr;f_q_ba<=ba; end
        always @* begin assert(q_addr==f_q_addr); assert(q_ba==f_q_ba); end
        // f_pr 表示上一个 clk_mem 沿处于 arst/rst。此时芯片看到的是 NOP/inhibit（cs_n=ras_n=cas_n=we_n=1），
        // 不存在依赖 addr/ba 的命令；f_pr=0（含 rst 释放后的第一拍）时 chip 地址/bank 等于上一沿的 addr/ba 输入。
        reg f_pr;reg [10:0] f_pa;reg [1:0] f_pb;
        always @(posedge clk_mem or posedge arst)
            if(arst) begin f_pr<=1;f_pa<=0;f_pb<=0; end
            else begin f_pr<=rst;f_pa<=addr;f_pb<=ba; end
        always @* begin
            assert(!rst || (q_cs_n && q_ras_n && q_cas_n && q_we_n));
            assert(!f_pr || (q_cs_n && q_ras_n && q_cas_n && q_we_n));
            assert(f_pr || (q_addr==f_pa && q_ba==f_pb));
        end
`endif
        // 读采样：每个 clk_mem 沿寄存 DQ，不复位（数据只在控制器 rd_data_valid 那拍被消费）。
        always @(posedge clk_mem) q_rdata<=chip_dq;
        assign chip_cs_n=q_cs_n;assign chip_ras_n=q_ras_n;assign chip_cas_n=q_cas_n;assign chip_we_n=q_we_n;
        assign chip_cke=q_cke;assign chip_addr=q_addr;assign chip_ba=q_ba;assign chip_dm=q_dm;
        // chip_write_enable 汇总 DQ 驱动条件；q_oe 各位使用相同的写拍和复位条件。
        assign chip_write_enable=q_oe[0];
        genvar gi;
        for(gi=0;gi<32;gi=gi+1) begin: g_dq
            assign chip_dq[gi]=q_oe[gi] ? q_wdata[gi] : 1'bz;
        end
        assign rd_data=q_rdata;
    end else begin: comb_io
        assign chip_cs_n=enabled ? cs_n : 1'b1;
        assign chip_ras_n=enabled ? ras_n : 1'b1;
        assign chip_cas_n=enabled ? cas_n : 1'b1;
        assign chip_we_n=enabled ? we_n : 1'b1;
        assign chip_cke=enabled && cke;
        assign chip_addr=enabled ? addr : 11'd0;
        assign chip_ba=enabled ? ba : 2'd0;
        // 业务路径只做完整32bit/BL8写；复位请求屏蔽四字节（非已确认的die侧隔离）。
        assign chip_dm=enabled ? 4'b0000 : 4'b1111;
        assign chip_write_enable=enabled && wr_beat;
        // 首字与 WRITE 许可同沿给出，其余 7 字连续提供，上游必须满足完整 BL8 写入。
        assign chip_dq=chip_write_enable ? wr_data : 32'bz;
        // 组合分支直接提供器件读返回，控制器按 CL3 取得数据。
        assign rd_data=chip_dq;
    end endgenerate
    generate if(USE_EG_SDRAM) begin: anlogic
        // 原生分支连接合封 SDRAM 器件原语，业务控制器通过本层的命令、写拍和读数据接口访问。
        EG_PHY_SDRAM_2M_32 u_sdram(
            .clk(clk_device),.ras_n(chip_ras_n),.cas_n(chip_cas_n),.we_n(chip_we_n),
            .addr(chip_addr),.ba(chip_ba),.dq(chip_dq),.cs_n(chip_cs_n),
            .dm0(chip_dm[0]),.dm1(chip_dm[1]),.dm2(chip_dm[2]),.dm3(chip_dm[3]),.cke(chip_cke));
    end endgenerate
    // 行为模型可连接私有 chip_* 节点；这些节点的读写时序与原语接口一致。
endmodule
`default_nettype wire
