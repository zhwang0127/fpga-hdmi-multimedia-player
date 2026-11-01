`timescale 1ns/1ps
// 功能：一拍接受一个字节的 IEEE CRC32，用于文件/媒体内容完整性检查。
// 接口：rst/clear 同步高有效，优先于 data_valid；clear 同拍的数据不计入。
// 输出：crc 是已异或的当前前缀校验值；空数据块为 0，不额外输出完成脉冲。
// 调用方按实际接受的字节数判定块尾，在末字节寄存更新后读取结果。
// Reflected IEEE CRC-32: init FFFFFFFF, polynomial EDB88320, final XOR FFFFFFFF.
module crc32_ieee(input wire clk,input wire rst,input wire clear,
                  input wire data_valid,input wire [7:0] data,output wire [31:0] crc);
    reg [31:0] state; // 保存未做最终异或的 CRC 状态。
    reg [31:0] next_state;
    integer bit_index;
    always @* begin
        // 反射算法从字节低位开始：先异或到低八位，再进行八轮右移反馈。
        // for 循环是组合逻辑展开，不是每个比特额外占一个时钟周期。
        next_state=state ^ {24'd0,data};
        for(bit_index=0;bit_index<8;bit_index=bit_index+1)
            if(next_state[0]) next_state=(next_state>>1)^32'hEDB88320;
            else next_state=next_state>>1;
    end
    always @(posedge clk) begin
        // 没有 data_valid 的空拍保留 state；没有背压，valid 的每个上升沿都接受。
        if(rst || clear) state<=32'hFFFFFFFF;
        else if(data_valid) state<=next_state;
    end
    assign crc=state^32'hFFFFFFFF; // 输出最终异或；不要将 crc 反馈回内部递推。
endmodule
