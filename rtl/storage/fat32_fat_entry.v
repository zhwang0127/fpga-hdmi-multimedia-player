`timescale 1ns/1ps
`default_nettype none

// FAT32表项提取器：一次消费一个完整512字节FAT扇区。
// 按查询簇号定位4字节小端表项（项内偏移=簇号低7位*4），流式锁存不做RAM复制。
// 只负责提取原始32位值；EOC/坏簇/范围判定由目录扫描控制模块完成。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module fat32_fat_entry(
    input wire clk,arst,abort,
    input wire start_valid,output wire start_ready,
    input wire byte_valid,output wire byte_ready,
    input wire [7:0] byte_data,input wire [8:0] byte_index,input wire byte_last,
    input wire [31:0] query_cluster,
    output wire result_valid,input wire result_ready,
    // 本地结果槽条件；控制模块另检查真实取消/期限/extent错误才消费。
    // 不移除本域复位隔离，原result_valid仍同拍被abort遮蔽。
    output wire result_pending,
    output wire result_code,
    output wire [31:0] entry_value
);
    wire rst_logic;
    localparam IDLE=0,COLLECT=1,RESULT=2;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic && !abort;
    reg [1:0] state;
    reg [8:0] expected;
    reg stream_bad;
    reg [31:0] value;
    reg code_reg;
    wire fire=byte_valid && byte_ready;
    // 表项4字节对齐：低7位簇号给出0..508的字节偏移。
    wire [9:0] entry_offset={query_cluster[6:0],2'b00};
    wire final_stream_bad=stream_bad || byte_index!=expected ||
                          byte_index!=9'd511 || !byte_last;
    assign start_ready=enabled && state==IDLE;
    assign byte_ready=enabled && state==COLLECT;
    assign result_valid=enabled && state==RESULT;
    assign result_pending=!rst_logic && state==RESULT;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;expected<=0;stream_bad<=0;value<=0;code_reg<=0;
        end else begin
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    state<=COLLECT;expected<=0;stream_bad<=0;
                end
                COLLECT:begin
                    if(fire) begin
                        expected<=expected+9'd1;
                        if(byte_index!=expected) stream_bad<=1;
                        // 四个9位相等比较锁定项内4字节；偏移上限511在9位内。
                        if(byte_index==entry_offset[8:0]) value[7:0]<=byte_data;
                        if(byte_index==entry_offset[8:0]+9'd1) value[15:8]<=byte_data;
                        if(byte_index==entry_offset[8:0]+9'd2) value[23:16]<=byte_data;
                        if(byte_index==entry_offset[8:0]+9'd3) value[31:24]<=byte_data;
                        // 完成沿锁存流校验结果；RESULT保持期间不再依赖当前端口值。
                        if(byte_index==9'd511) begin
                            code_reg<=final_stream_bad;state<=RESULT;
                        end
                    end
                end
                RESULT:if(result_valid && result_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
            // abort同拍遮蔽ready/result，沿上撤销本地收集；已锁存项值保留无妨。
            if(abort) begin state<=IDLE;stream_bad<=0;code_reg<=0;end
        end
    end
    assign result_code=code_reg;
    assign entry_value=value;
endmodule
`default_nettype wire
