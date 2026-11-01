`timescale 1ns/1ps
`default_nettype none

// 几何控制模块内部的64位上下文运算器；八个字节构成一个完整数值。
// 没有卡请求、事务键或生产者停止确认。业务所有权/完整键由父模块维护。
// RAM不复位：父模块须在每次业务请求重新写入所有输入/常量，再使用派生值。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module fat32_context_alu #(parameter USE_EG_BRAM=1,EXTERNAL_CONTEXT_RAM=0)(
    input wire clk,arst,abort,
    input wire command_valid,output wire command_ready,
    input wire [1:0] command_op,
    input wire [4:0] command_a,command_b,command_dst,
    output wire done_valid,input wire done_ready,
    output wire busy,output reg carry,zero,
    output wire context_ready,
    input wire context_write,input wire [7:0] context_address,
    input wire [7:0] context_wdata,
    input wire context_read,output wire context_rvalid,output wire [7:0] context_rdata,
    output wire context_ram_wr_enable,context_ram_rd_enable,
    output wire [7:0] context_ram_wr_addr,context_ram_rd_addr,context_ram_wr_data,
    input wire [7:0] context_ram_rd_data
);
    wire rst_logic;
    localparam IDLE=0,READ_A=1,TAKE_A=2,READ_B=3,TAKE_B=4,WRITE=6,DONE=7;
    localparam ADD=0,SUB=1,LEFT=2,RIGHT=3;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic && !abort;
    reg [2:0] state,byte_index;
    reg [1:0] operation;
    reg [4:0] a_index,b_index,dst_index;
    reg read_pending;
    reg [7:0] left;
    wire subtract=operation==SUB;
    wire shifting=operation[1];
    wire [8:0] digit={1'b0,left}+{1'b0,(context_rdata ^ {8{subtract}})}+{8'd0,carry};
    assign command_ready=enabled && state==IDLE;
    assign context_ready=command_ready && !command_valid;
    assign busy=state!=IDLE;
    assign done_valid=enabled && state==DONE;
    assign context_rvalid=enabled && read_pending;
    wire internal_write=state==WRITE;
    wire internal_read=state==READ_A || state==READ_B;
    wire wr_enable=enabled && (internal_write || (context_ready && context_write));
    wire [7:0] wr_address=internal_write ? {dst_index,byte_index} : context_address;
    wire [7:0] wr_data=internal_write ? left : context_wdata;
    // 外部读写请求同拍有效时写优先，避免依赖 RAM 的同址碰撞行为。
    wire rd_enable=enabled && (internal_read || (context_ready && context_read && !context_write));
    wire [7:0] rd_address=state==READ_A ? {a_index,byte_index} :
                          state==READ_B ? {b_index,byte_index} : context_address;
    // 外置模式仅用于已取得排他所有权、同clk一拍同步读的父级共享RAM。
    // 模式关闭时不使用上下文输入，也不发布 RAM 请求。
    assign context_ram_wr_enable=EXTERNAL_CONTEXT_RAM && wr_enable;
    assign context_ram_rd_enable=EXTERNAL_CONTEXT_RAM && rd_enable;
    assign context_ram_wr_addr=EXTERNAL_CONTEXT_RAM ? wr_address : 8'd0;
    assign context_ram_rd_addr=EXTERNAL_CONTEXT_RAM ? rd_address : 8'd0;
    assign context_ram_wr_data=EXTERNAL_CONTEXT_RAM ? wr_data : 8'd0;
    generate if(EXTERNAL_CONTEXT_RAM) begin : shared_context
        assign context_rdata=context_ram_rd_data;
    end else begin : local_context
        // 使用双端口 RAM，保持同钟读写所需的端口配置。
        fifo_dual_clock_ram #(.DATA_WIDTH(8),.ADDR_WIDTH(10),.USE_EG_BRAM(USE_EG_BRAM),
                             .RAM_IMPLEMENT("9K"),.PHYSICAL_BYTE(1)) u_context(
            .wr_clk(clk),.wr_enable(wr_enable),.wr_addr({2'd0,wr_address}),.wr_data(wr_data),
            .rd_clk(clk),.rd_enable(rd_enable),.rd_addr({2'd0,rd_address}),.rd_data(context_rdata));
    end endgenerate
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;operation<=ADD;a_index<=0;b_index<=0;dst_index<=0;
            byte_index<=0;left<=0;carry<=0;zero<=1;read_pending<=0;
        end else begin
            read_pending<=context_ready && context_read && !context_write;
            case(state)
                IDLE:if(command_valid && command_ready) begin
                    operation<=command_op;a_index<=command_a;b_index<=command_b;dst_index<=command_dst;
                    byte_index<=command_op==RIGHT ? 3'd7 : 3'd0;carry<=command_op==SUB;zero<=1;state<=READ_A;
                end
                READ_A:state<=TAKE_A;
                TAKE_A:begin
                    if(shifting) begin
                        // 右移先高字节、左移先低字节，跨字节的位必须真实传递。
                        if(operation==RIGHT) begin
                            left<={byte_index==7 ? 1'b0 : carry,context_rdata[7:1]};carry<=context_rdata[0];
                        end else begin
                            left<={context_rdata[6:0],byte_index==0 ? 1'b0 : carry};carry<=context_rdata[7];
                        end
                        state<=WRITE;
                    end else begin left<=context_rdata;state<=READ_B;end
                end
                READ_B:state<=TAKE_B;
                TAKE_B:begin left<=digit[7:0];carry<=digit[8];state<=WRITE;end
                WRITE:begin
                    zero<=zero && left==0;
                    if((operation==RIGHT && byte_index==0) || (operation!=RIGHT && byte_index==7)) state<=DONE;
                    else begin byte_index<=operation==RIGHT ? byte_index-3'd1 : byte_index+3'd1;state<=READ_A;end
                end
                DONE:if(done_valid && done_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
            // 只丢弃封闭本地运算；写使能同拍遮蔽。未写完的上下文不可被业务复用。
            if(abort) begin state<=IDLE;read_pending<=0;end
        end
    end
endmodule
`default_nettype wire
