`timescale 1ns/1ps
`default_nettype none

// 保存四个文件上下文，每槽 32 个 32 位字，共 128 字节；文件调度和超时由 fat32_chain_master 处理。
// 所有接口运行在 clk 域，rst 同步高有效；valid 与 ready 同时有效的上升沿接受命令或数据。
// 命令 op 为 0 保存、1 恢复、2 作废单槽、3 作废全部。
// 保存开始时清除槽有效位，收到全部 32 字后才重新置位；中途取消的记录不能恢复。
// RAM 数据不复位，以 valid_mask 判断记录是否完整；读取采用一拍同步延迟，等待接收时保持 data/index/last。
// pinned_mask 标识仍被 SPI、字节桥或数据处理模块使用的槽，对这些槽的访问返回 BUSY。
// 上层在本模块空闲时改变槽占用；只有源端与数据处理模块均确认停止并完成在途操作，才能清除 pin。
// 本模块的 done 仅表示记录传送结束，abort 仅取消当前记录传送，两者都不清空外部 FIFO。
module fat32_file_context_ram #(
    parameter USE_EG_BRAM=1
)(
    input wire clk,rst,
    input wire cmd_valid,output wire cmd_ready,
    input wire [1:0] cmd_op,cmd_slot,
    input wire [3:0] pinned_mask,
    input wire abort,
    input wire wr_valid,output wire wr_ready,input wire [31:0] wr_data,
    output wire rd_valid,input wire rd_ready,output wire [31:0] rd_data,
    output wire [4:0] word_index,output wire rd_last,
    output wire done_valid,input wire done_ready,
    output wire [1:0] done_op,done_slot,done_code,
    output reg [3:0] valid_mask,
    output wire busy
);
    localparam [2:0] IDLE=0,STORE=1,READ_ISSUE=2,READ_OUT=3,DONE=4;
    localparam [1:0] OK=0,EMPTY=1,CANCELED=2,BUSY=3;
    reg [2:0] state;
    reg [1:0] op_q,slot_q,code_q;
    reg [4:0] index_q;
    wire active=state==STORE || state==READ_ISSUE || state==READ_OUT;
    wire [6:0] address={slot_q,index_q};
    wire ram_write=wr_valid && wr_ready;
    wire ram_read=state==READ_ISSUE && !rst && !abort;
    assign busy=state!=IDLE;
    assign cmd_ready=state==IDLE && !rst;
    assign wr_ready=state==STORE && !rst && !abort;
    assign rd_valid=state==READ_OUT && !rst && !abort;
    assign word_index=index_q;
    assign rd_last=index_q==31;
    assign done_valid=state==DONE && !rst;
    assign done_op=op_q;
    assign done_slot=slot_q;
    assign done_code=code_q;

    fifo_dual_clock_ram #(.DATA_WIDTH(32),.ADDR_WIDTH(7),
        .USE_EG_BRAM(USE_EG_BRAM),.RAM_IMPLEMENT("9K")) u_records(
        .wr_clk(clk),.wr_enable(ram_write),.wr_addr(address),.wr_data(wr_data),
        .rd_clk(clk),.rd_enable(ram_read),.rd_addr(address),.rd_data(rd_data));

    always @(posedge clk) begin
        if(rst) begin
            state<=IDLE;op_q<=0;slot_q<=0;code_q<=OK;index_q<=0;valid_mask<=0;
        end else if(active && abort) begin
            // STORE从命令接受起已失效，READ不修改记录；取消优先于同沿字交接。
            state<=DONE;code_q<=CANCELED;
        end else case(state)
            IDLE: if(cmd_valid && cmd_ready) begin
                op_q<=cmd_op;slot_q<=cmd_slot;index_q<=0;code_q<=OK;
                if((cmd_op==3 && |pinned_mask) ||
                   (cmd_op!=3 && pinned_mask[cmd_slot])) begin
                    state<=DONE;code_q<=BUSY;
                end else case(cmd_op)
                    0: begin valid_mask[cmd_slot]<=0;state<=STORE;end
                    1: if(valid_mask[cmd_slot]) state<=READ_ISSUE;
                       else begin state<=DONE;code_q<=EMPTY;end
                    2: begin valid_mask[cmd_slot]<=0;state<=DONE;end
                    3: begin valid_mask<=0;state<=DONE;end
                endcase
            end
            STORE: if(ram_write) begin
                if(index_q==31) begin valid_mask[slot_q]<=1;state<=DONE;end
                else index_q<=index_q+1'b1;
            end
            READ_ISSUE: state<=READ_OUT;
            READ_OUT: if(rd_valid && rd_ready) begin
                if(index_q==31) state<=DONE;
                else begin index_q<=index_q+1'b1;state<=READ_ISSUE;end
            end
            DONE: if(done_valid && done_ready) state<=IDLE;
            default: begin state<=IDLE;valid_mask<=0;end
        endcase
    end
endmodule

`default_nettype wire
