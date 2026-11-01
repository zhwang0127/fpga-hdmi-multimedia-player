`timescale 1ns/1ps
// 功能：计算 SD 数据块的 CRC16；不处理命令 CRC7，也不把数据 token 计入。
// 接口：rst/clear 同步高有效且优先于 data_valid；每个有效周期接受一个字节。
// 输出：末字节寄存更新后读取 crc，再由 SD 接收器与卡片返回的 CRC 字节比较。
// SD data CRC16: non-reflected polynomial 1021, initial 0000, no final XOR.
module crc16_sd(input wire clk,input wire rst,input wire clear,
                 input wire data_valid,input wire [7:0] data,output wire [15:0] crc);
    reg [15:0] state;
    reg [15:0] next_state;
    integer bit_index;
    always @* begin
        // 非反射算法：数据异或到高八位，按最高位决定左移后的多项式反馈。
        // 八轮组合展开完成一个字节；与 CRC32 的移位方向和初值不同。
        next_state=state ^ {data,8'd0};
        for(bit_index=0;bit_index<8;bit_index=bit_index+1)
            if(next_state[15]) next_state=(next_state<<1)^16'h1021;
            else next_state=next_state<<1;
    end
    always @(posedge clk) begin
        // clear 表示新数据块开始，空拍保持状态；clear 同拍数据被丢弃。
        if(rst || clear) state<=16'h0000;
        else if(data_valid) state<=next_state;
    end
    assign crc=state; // SD 数据 CRC16 无最终异或，初始/空块结果为 0。
endmodule
