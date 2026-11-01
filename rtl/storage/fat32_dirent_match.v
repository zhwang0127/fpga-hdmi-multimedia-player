`timescale 1ns/1ps
`default_nettype none

// FAT32目录项流式匹配器：一次消费一个extent批次的全部目录扇区(1-8个)。
// 每个扇区按字节索引直接解码32字节目录项，不复制窗口RAM；目标名为11字节8.3格式。
// 跳过删除项(0xE5)、LFN(0x0F)、卷标(attr[3])；任一扇区名字首字节0x00即链终止。
// 重复命中同一目标名时hits饱和为2，调用方据此按DUP拒绝；chain_end后不再计数。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module fat32_dirent_match(
    input wire clk,arst,abort,
    input wire start_valid,output wire start_ready,
    input wire byte_valid,output wire byte_ready,
    input wire [7:0] byte_data,input wire [8:0] byte_index,input wire byte_last,
    input wire [87:0] target_name,input wire want_dir,
    input wire [3:0] sector_count,
    // chain_reset由扫描控制模块在事务接受沿发出：清上一链的命中累计，防止跨事务泄漏。
    input wire chain_reset,
    output wire result_valid,input wire result_ready,
    // 只表示本地结果槽已满，保留本域复位隔离但不串外部abort。
    // 控制模块使用前必须显式检查取消/期限/extent错误；公共result_valid语义不变。
    output wire result_pending,
    output wire result_code,
    output wire first_hit_valid,first_is_dir,
    output wire [31:0] first_hit_cluster,first_hit_size,
    output wire [1:0] hit_count,output wire chain_terminated
);
    wire rst_logic;
    localparam IDLE=0,COLLECT=1,RESULT=2;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic && !abort;
    reg [1:0] state;
    reg [8:0] expected;
    reg [3:0] sectors_left;
    reg stream_bad;
    reg name_match;
    reg [7:0] name_first,attr;
    reg [15:0] cluster_hi,cluster_lo;
    reg [31:0] size_bytes;
    reg first_valid,is_dir;
    reg [31:0] first_cluster,first_size;
    reg [1:0] hits;
    reg chain_end;
    reg code_reg;
    wire fire=byte_valid && byte_ready;
    wire [4:0] offset=byte_index[4:0];
    // 有效命中条件：非删除、非LFN、非卷标、名字全等且类型匹配。
    wire candidate=name_first!=8'h00 && name_first!=8'he5 &&
                   attr!=8'h0f && !attr[3] && attr[4]==want_dir;
    wire final_stream_bad=stream_bad || byte_index!=expected ||
                          byte_index!=9'd511 || !byte_last;
    assign start_ready=enabled && state==IDLE;
    assign byte_ready=enabled && state==COLLECT;
    assign result_valid=enabled && state==RESULT;
    assign result_pending=!rst_logic && state==RESULT;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;expected<=0;sectors_left<=0;stream_bad<=0;name_match<=0;
            name_first<=0;attr<=0;cluster_hi<=0;cluster_lo<=0;size_bytes<=0;
            first_valid<=0;is_dir<=0;first_cluster<=0;first_size<=0;
            hits<=0;chain_end<=0;code_reg<=0;
        end else begin
            case(state)
                IDLE:begin
                    // 命中累计只在控制模块发出chain_reset时清除：扇区间保持用于
                    // 全链DUP检查，不跨扫描事务泄漏。chain_reset与首个start不同沿。
                    if(chain_reset) begin
                        first_valid<=0;is_dir<=0;first_cluster<=0;first_size<=0;
                        hits<=0;chain_end<=0;
                    end
                    if(start_valid && start_ready) begin
                        state<=COLLECT;expected<=0;stream_bad<=0;name_match<=1;
                        name_first<=0;attr<=0;cluster_hi<=0;cluster_lo<=0;size_bytes<=0;
                        sectors_left<=sector_count==4'd0 ? 4'd1 : sector_count;
                    end
                end
                COLLECT:begin
                    if(fire) begin
                        expected<=expected+9'd1;
                        if(byte_index!=expected) stream_bad<=1;
                        case(offset)
                            // 名字11字节流式比较：每字节一个8位相等项。
                            5'd0:begin name_first<=byte_data;name_match<=name_match && byte_data==target_name[7:0];end
                            5'd1:name_match<=name_match && byte_data==target_name[15:8];
                            5'd2:name_match<=name_match && byte_data==target_name[23:16];
                            5'd3:name_match<=name_match && byte_data==target_name[31:24];
                            5'd4:name_match<=name_match && byte_data==target_name[39:32];
                            5'd5:name_match<=name_match && byte_data==target_name[47:40];
                            5'd6:name_match<=name_match && byte_data==target_name[55:48];
                            5'd7:name_match<=name_match && byte_data==target_name[63:56];
                            5'd8:name_match<=name_match && byte_data==target_name[71:64];
                            5'd9:name_match<=name_match && byte_data==target_name[79:72];
                            5'd10:name_match<=name_match && byte_data==target_name[87:80];
                            5'd11:attr<=byte_data;
                            // FAT32首簇高16位在项内字节20-21，低16位在26-27。
                            5'd20:cluster_hi[7:0]<=byte_data;
                            5'd21:cluster_hi[15:8]<=byte_data;
                            5'd26:cluster_lo[7:0]<=byte_data;
                            5'd27:cluster_lo[15:8]<=byte_data;
                            5'd28:size_bytes[7:0]<=byte_data;
                            5'd29:size_bytes[15:8]<=byte_data;
                            5'd30:size_bytes[23:16]<=byte_data;
                            5'd31:begin
                                size_bytes[31:24]<=byte_data;
                                // 项完成沿统一判定；先判链终止再判命中。
                                if(name_first==8'h00) chain_end<=1;
                                else if(!chain_end && candidate && name_match) begin
                                    if(!first_valid) begin
                                        first_valid<=1;is_dir<=attr[4];
                                        first_cluster<={cluster_hi,cluster_lo};
                                        first_size<=size_bytes;
                                        hits<=2'd1;
                                    end else hits<=2'd2;
                                end
                                name_match<=1;
                            end
                        endcase
                        // 批次内逐扇区推进：扇区末重置期望索引；末扇区后锁存并进入RESULT。
                        if(byte_index==9'd511 && byte_last) begin
                            if(sectors_left>4'd1) begin
                                sectors_left<=sectors_left-4'd1;expected<=0;
                            end else begin
                                code_reg<=final_stream_bad;state<=RESULT;
                            end
                        end
                    end
                end
                RESULT:if(result_valid && result_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
            // abort同拍遮蔽ready/result，沿上撤销本地收集；命中累计保留给全链DUP检查。
            if(abort) begin state<=IDLE;stream_bad<=0;code_reg<=0;end
        end
    end
    assign result_code=code_reg;
    assign first_hit_valid=first_valid;assign first_is_dir=is_dir;
    assign first_hit_cluster=first_cluster;assign first_hit_size=first_size;
    assign hit_count=hits;assign chain_terminated=chain_end;
endmodule
`default_nettype wire
