`timescale 1ns/1ps
`default_nettype none

// BPB数值几何条件：输入必须来自已完整读取并CRC通过的引导扇区。
// 本模块不发SD请求、不解析MBR、不提供文件生产者stop_ack。
// 所有地址/乘积先在64位中计算，边界通过后才发布32位LBA。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module fat32_geometry #(
    // 会话、序号和初始化 epoch 的键宽；默认 32 位，整机由 tf_bmp_boot 传入 8 位。
    // TIME_W 为内部预算、已用和剩余周期计数的位宽，默认 32 位；
    // 产品由 tf_bmp_boot 传入 counter_width(SYS_BUDGET+1)=26，该宽度按构造恰好容纳预算值本身。
    // 所有比较/减法都在 TIME_W 位内完成；截止判定仍是逐周期精确的，预算值不变。
    parameter integer KEY_W=32,parameter integer TIME_W=32,parameter USE_EG_BRAM=1,EXTERNAL_CONTEXT_RAM=0)(
    input wire clk,arst,
    input wire req_valid,output wire req_ready,
    input wire [KEY_W-1:0] req_session,req_seq, input wire [31:0] req_budget_cycles,
    input wire [31:0] req_partition_lba,
    input wire [32:0] req_partition_sectors,req_card_sectors,
    input wire [15:0] req_signature,req_bytes_per_sector,req_reserved_sectors,
    input wire [7:0] req_sectors_per_cluster,req_num_fats,
    input wire [15:0] req_root_entry_count,req_total_sectors16,req_fat_sectors16,req_fs_version,
    input wire [31:0] req_total_sectors,req_fat_sectors,req_root_cluster,
    input wire [15:0] req_ext_flags,
    input wire cancel_valid,output wire cancel_ready,
    input wire [KEY_W-1:0] cancel_session,cancel_seq,
    output wire busy,result_valid,input wire result_ready,
    output wire [KEY_W-1:0] result_session,result_seq,
    output wire [3:0] result_code,output wire [31:0] result_elapsed,
    output wire [32:0] partition_end,volume_end,
    output wire [31:0] fat_lba,data_lba,root_lba,
    output reg [31:0] fat_sectors,root_cluster,
    output wire [27:0] cluster_count,
    output reg [2:0] cluster_shift,output reg [3:0] active_fat,
    output wire context_ram_wr_enable,context_ram_rd_enable,
    output wire [7:0] context_ram_wr_addr,context_ram_rd_addr,context_ram_wr_data,
    input wire [7:0] context_ram_rd_data
);
    wire rst_logic;
    localparam IDLE=0,PART_CHECK=1,PART_LIMIT=2,PROFILE=3,VOL_CHECK=4,VOL_LIMIT=5,
               META_LOOP=6,META_VALIDATE=7,DATA_CHECK=8,DIVIDE=9,TYPE_LOW=10,
               TYPE_HIGH=11,FAT_COUNT=12,FAT_CAP=13,ACTIVE=14,FAT_LOOP=15,
               FAT_START=16,FAT_SUM=17,FAT_END=18,ROOT_RANGE=19,ROOT_LIMIT=20,
               ROOT_CHECK=21,ROOT_SHIFT=22,ROOT_ADD=23,ROOT_START=24,
               ROOT_SUM=25,ROOT_END=26,RESULT=27,COMMAND=28,SEED=29,PUBLISH=30,TAKE=31;
    localparam C_BASE=0,C_PART=1,C_CARD=2,C_TOTAL=3,C_RESERVED=4,C_FATSIZE=5,C_ROOT=6,
               C_PARTEND=7,C_VOLEND=8,C_META=9,C_DATA=10,C_COUNT=11,C_FATCAP=12,
               C_FATSTART=13,C_ROOTSTART=14,C_ONE=15,C_TWO=16,C_MIN=17,C_MAX=18,
               C_SCRATCH=19,C_ZERO=20,C_DATALBA=21,C_SPC=22;
    localparam OK=0,SIGNATURE=1,ADDRESS=2,GEOMETRY=3,FAT32_FIELDS=4,
               DATA_REGION=5,FAT_TYPE=6,FAT_TOO_SHORT=7,CANCELED=8,
               ACTIVE_FAT=9,CLUSTER_RANGE=10,DEADLINE=11;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic;
    reg [4:0] state,return_state;
    reg [3:0] status,seed_row,publish_row;
    reg [2:0] seed_byte,publish_byte;
    reg [KEY_W-1:0] session,seq; reg [TIME_W-1:0] budget,elapsed; reg [31:0] base,total;
    reg [32:0] part_count,card_count;
    reg [15:0] reserved;
    reg [7:0] num_fats,remaining;
    reg signature_bad,geometry_bad,fields_bad,canceled;
    reg operation_valid;
    reg [1:0] operation;
    reg [4:0] operand_a,operand_b,destination;
    wire operation_ready,operation_done,carry,zero,context_ready,context_rvalid;
    wire [7:0] context_rdata;
    wire cancel_now=cancel_valid && cancel_ready;
    assign req_ready=enabled && state==IDLE && !cancel_valid;
    assign cancel_ready=enabled && state!=IDLE && !canceled &&
                        cancel_session==session && cancel_seq==seq;
    assign busy=enabled && state!=IDLE;
    assign result_valid=enabled && state==RESULT && !cancel_now;
    assign result_session=session;assign result_seq=seq;
    assign result_code=status;assign result_elapsed=elapsed;
    wire supported_spc=req_sectors_per_cluster==1 || req_sectors_per_cluster==2 ||
                       req_sectors_per_cluster==4 || req_sectors_per_cluster==8 ||
                       req_sectors_per_cluster==16 || req_sectors_per_cluster==32 ||
                       req_sectors_per_cluster==64;
    // 指令只装载上下文索引；不会在64个数据位上复制大范围状态选择器。
    task start_alu;
        input [4:0] lhs,rhs,dst;
        input [1:0] kind;
        input [4:0] next_state;
        begin
            operand_a<=lhs;operand_b<=rhs;destination<=dst;operation<=kind;operation_valid<=1;
            return_state<=next_state;state<=COMMAND;
        end
    endtask
    task fail;
        input [3:0] code;
        begin status<=code;state<=RESULT;end
    endtask
    reg [4:0] seed_index,publish_index;
    // 每格明确64位含零扩展，整个选择范围都有驱动，避免越界选择补齐悬空网。
    wire [1023:0] seed_image={64'd0,{56'd0,(8'd1<<cluster_shift)},64'd0,64'h0fffffee,
        64'd65524,64'd2,64'd1,{25'd0,fat_sectors,7'd0},{48'd0,reserved},
        {32'd0,root_cluster},{32'd0,fat_sectors},{48'd0,reserved},{32'd0,total},
        {31'd0,card_count},{31'd0,part_count},{32'd0,base}};
    wire [7:0] seed_data=seed_image[{seed_row,seed_byte,3'd0}+:8];
    // 按低字节先发布26字节，统一移位寄存器代替逐字段/逐字节宽装载选择。
    // 正在计算时不发布有效结果，旧片段只在26字节全部覆盖后才可使用。
    reg [207:0] publication;
    assign partition_end=publication[32:0];assign volume_end=publication[72:40];
    assign fat_lba=publication[111:80];assign data_lba=publication[143:112];
    assign cluster_count=publication[171:144];assign root_lba=publication[207:176];
    always @* begin
        seed_index=C_ZERO;
        case(seed_row)
            0:seed_index=C_BASE;1:seed_index=C_PART;2:seed_index=C_CARD;
            3:seed_index=C_TOTAL;4:seed_index=C_RESERVED;5:seed_index=C_FATSIZE;6:seed_index=C_ROOT;
            7:seed_index=C_META;8:seed_index=C_FATCAP;9:seed_index=C_ONE;10:seed_index=C_TWO;
            11:seed_index=C_MIN;12:seed_index=C_MAX;14:seed_index=C_SPC;
            default:seed_index=C_ZERO;
        endcase
        publish_index=C_ROOTSTART;
        case(publish_row)
            0:publish_index=C_PARTEND;1:publish_index=C_VOLEND;2:publish_index=C_FATSTART;
            3:publish_index=C_DATALBA;4:publish_index=C_COUNT;
            default:publish_index=C_ROOTSTART;
        endcase
    end
    wire core_abort=state==RESULT || cancel_now || (state!=IDLE && elapsed==budget-1'b1);
    fat32_context_alu #(.USE_EG_BRAM(USE_EG_BRAM),.EXTERNAL_CONTEXT_RAM(EXTERNAL_CONTEXT_RAM)) u_alu(
        .clk(clk),.arst(arst),.abort(core_abort),.command_valid(operation_valid),.command_ready(operation_ready),
        .command_op(operation),.command_a(operand_a),.command_b(operand_b),.command_dst(destination),
        .done_valid(operation_done),.done_ready(1'b1),.busy(),.carry(carry),.zero(zero),
        .context_ready(context_ready),.context_write(state==SEED),
        .context_address(state==SEED ? {seed_index,seed_byte} : {publish_index,publish_byte}),
        .context_wdata(seed_data),.context_read(state==PUBLISH),
        .context_rvalid(context_rvalid),.context_rdata(context_rdata),
        .context_ram_wr_enable(context_ram_wr_enable),.context_ram_rd_enable(context_ram_rd_enable),
        .context_ram_wr_addr(context_ram_wr_addr),.context_ram_rd_addr(context_ram_rd_addr),
        .context_ram_wr_data(context_ram_wr_data),.context_ram_rd_data(context_ram_rd_data));
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;return_state<=IDLE;status<=OK;seed_row<=0;seed_byte<=0;publish_row<=0;publish_byte<=0;
            session<=0;seq<=0;budget<=0;elapsed<=0;base<=0;total<=0;
            part_count<=0;card_count<=0;reserved<=0;num_fats<=0;remaining<=0;
            signature_bad<=0;geometry_bad<=0;fields_bad<=0;canceled<=0;
            operation_valid<=0;operation<=0;operand_a<=0;operand_b<=0;destination<=0;
            publication<=0;fat_sectors<=0;root_cluster<=0;cluster_shift<=0;active_fat<=0;
        end else begin
            if(state!=IDLE && state!=RESULT) elapsed<=elapsed+1'b1;
            if(operation_valid && operation_ready) operation_valid<=0;
            case(state)
                IDLE:if(req_valid && req_ready) begin
                    session<=req_session;seq<=req_seq;budget<=req_budget_cycles[TIME_W-1:0];elapsed<=0;
                    status<=OK;canceled<=0;base<=req_partition_lba;part_count<=req_partition_sectors;
                    card_count<=req_card_sectors;total<=req_total_sectors;reserved<=req_reserved_sectors;
                    num_fats<=req_num_fats;fat_sectors<=req_fat_sectors;root_cluster<=req_root_cluster;
                    signature_bad<=req_signature!=16'haa55;
                    geometry_bad<=req_bytes_per_sector!=512 || !supported_spc ||
                                  req_reserved_sectors==0 || req_num_fats==0;
                    fields_bad<=req_root_entry_count!=0 || req_total_sectors16!=0 ||
                                req_fat_sectors16!=0 || req_fs_version!=0 || req_fat_sectors==0;
                    active_fat<=req_ext_flags[7] ? req_ext_flags[3:0] : 4'd0;
                    case(req_sectors_per_cluster)
                        2:cluster_shift<=1;4:cluster_shift<=2;8:cluster_shift<=3;
                        16:cluster_shift<=4;32:cluster_shift<=5;64:cluster_shift<=6;
                        default:cluster_shift<=0;
                    endcase
                    seed_row<=0;seed_byte<=0;state<=SEED;operation_valid<=0;
                    if(req_budget_cycles==0) fail(DEADLINE);
                end
                SEED:if(context_ready) begin
                    seed_byte<=seed_byte+3'd1;
                    if(seed_byte==7) begin
                        if(seed_row==14) start_alu(C_BASE,C_PART,C_PARTEND,0,PART_CHECK);
                        else seed_row<=seed_row+4'd1;
                    end
                end
                COMMAND:if(operation_done) state<=return_state;
                PART_CHECK:begin
                    if(signature_bad) fail(SIGNATURE);
                    else if(part_count==0 || card_count==0 || (card_count[32] && |card_count[31:0])) fail(ADDRESS);
                    else start_alu(C_CARD,C_PARTEND,C_SCRATCH,1,PART_LIMIT);
                end
                PART_LIMIT:begin
                    // 完整64位bound-end减法最终carry=1时表示未越界。
                    if(!carry) fail(ADDRESS);
                    else state<=PROFILE;
                end
                PROFILE:begin
                    if(geometry_bad) fail(GEOMETRY);
                    else if(fields_bad) fail(FAT32_FIELDS);
                    else start_alu(C_BASE,C_TOTAL,C_VOLEND,0,VOL_CHECK);
                end
                VOL_CHECK:begin
                    if(total==0) fail(ADDRESS);
                    else start_alu(C_PARTEND,C_VOLEND,C_SCRATCH,1,VOL_LIMIT);
                end
                VOL_LIMIT:begin
                    if(!carry) fail(ADDRESS);
                    else begin remaining<=num_fats;state<=META_LOOP;end
                end
                META_LOOP:begin
                    if(remaining!=0) begin
                        remaining<=remaining-8'd1;start_alu(C_META,C_FATSIZE,C_META,0,META_LOOP);
                    end else start_alu(C_TOTAL,C_META,C_DATA,1,META_VALIDATE);
                end
                META_VALIDATE:begin
                    // total-metadata需无借位且非零；数据扇区数和 metadata 均须能由 32 位表示。
                    if(!carry || zero) fail(DATA_REGION);
                    else start_alu(C_BASE,C_META,C_DATALBA,0,DATA_CHECK);
                end
                DATA_CHECK:begin
                    // metadata<total且base+total<=卡尾<=2^32，data_start 在此前已检查为 32 位范围。
                    remaining<={5'd0,cluster_shift};start_alu(C_DATA,C_ZERO,C_COUNT,0,DIVIDE);
                end
                DIVIDE:begin
                    if(remaining!=0) begin remaining<=remaining-8'd1;start_alu(C_COUNT,C_ZERO,C_COUNT,3,DIVIDE);end
                    else start_alu(C_MIN,C_COUNT,C_SCRATCH,1,TYPE_LOW);
                end
                TYPE_LOW:begin
                    // 65524-count有借位才是FAT32；卷标签不参与。
                    if(carry) fail(FAT_TYPE);
                    else start_alu(C_MAX,C_COUNT,C_SCRATCH,1,TYPE_HIGH);
                end
                TYPE_HIGH:begin
                    if(!carry) fail(FAT_TYPE);
                    else start_alu(C_COUNT,C_TWO,C_SCRATCH,0,FAT_COUNT);
                end
                FAT_COUNT:start_alu(C_FATCAP,C_SCRATCH,C_SCRATCH,1,FAT_CAP);
                FAT_CAP:begin
                    if(!carry) fail(FAT_TOO_SHORT);
                    else state<=ACTIVE;
                end
                ACTIVE:begin
                    if({4'd0,active_fat}>=num_fats) fail(ACTIVE_FAT);
                    else begin
                        remaining<={4'd0,active_fat};start_alu(C_BASE,C_RESERVED,C_FATSTART,0,FAT_LOOP);
                    end
                end
                FAT_LOOP:begin
                    if(remaining!=0) begin
                        remaining<=remaining-8'd1;start_alu(C_FATSTART,C_FATSIZE,C_FATSTART,0,FAT_LOOP);
                    end else start_alu(C_VOLEND,C_FATSTART,C_SCRATCH,1,FAT_START);
                end
                FAT_START:begin
                    if(!carry || zero) fail(ADDRESS);
                    else start_alu(C_FATSTART,C_FATSIZE,C_SCRATCH,0,FAT_SUM);
                end
                FAT_SUM:start_alu(C_VOLEND,C_SCRATCH,C_SCRATCH,1,FAT_END);
                FAT_END:begin
                    // 前序检查已满足卷尾<=分区尾<=卡尾，不重复实例化三套宽比较器。
                    if(!carry) fail(ADDRESS);
                    else state<=ROOT_RANGE;
                end
                ROOT_RANGE:begin
                    if(root_cluster<2) fail(CLUSTER_RANGE);
                    else start_alu(C_COUNT,C_ONE,C_SCRATCH,0,ROOT_LIMIT);
                end
                ROOT_LIMIT:start_alu(C_SCRATCH,C_ROOT,C_SCRATCH,1,ROOT_CHECK);
                ROOT_CHECK:begin
                    // max_cluster=count+1已经避开保留值域，完整32位root在缩位前比较。
                    if(!carry) fail(CLUSTER_RANGE);
                    else begin
                        remaining<={5'd0,cluster_shift};start_alu(C_ROOT,C_TWO,C_ROOTSTART,1,ROOT_SHIFT);
                    end
                end
                ROOT_SHIFT:begin
                    if(remaining!=0) begin remaining<=remaining-8'd1;start_alu(C_ROOTSTART,C_ZERO,C_ROOTSTART,2,ROOT_SHIFT);end
                    else start_alu(C_DATALBA,C_ROOTSTART,C_ROOTSTART,0,ROOT_ADD);
                end
                ROOT_ADD:start_alu(C_VOLEND,C_ROOTSTART,C_SCRATCH,1,ROOT_START);
                ROOT_START:begin
                    if(!carry || zero) fail(ADDRESS);
                    else start_alu(C_ROOTSTART,C_SPC,C_SCRATCH,0,ROOT_SUM);
                end
                ROOT_SUM:start_alu(C_VOLEND,C_SCRATCH,C_SCRATCH,1,ROOT_END);
                ROOT_END:begin
                    if(!carry) fail(ADDRESS);
                    else begin publish_row<=0;publish_byte<=0;state<=PUBLISH;end
                end
                PUBLISH:if(context_ready) state<=TAKE;
                TAKE:if(context_rvalid) begin
                    // 缩位只发生于全部 64 位几何范围检查通过后的发布阶段。
                    publication<={context_rdata,publication[207:8]};
                    if((publish_row<2 && publish_byte==4) || (publish_row>=2 && publish_byte==3)) begin
                        publish_byte<=0;
                        if(publish_row==5) state<=RESULT;
                        else begin publish_row<=publish_row+4'd1;state<=PUBLISH;end
                    end else begin publish_byte<=publish_byte+3'd1;state<=PUBLISH;end
                end
                RESULT:if(result_valid && result_ready) state<=IDLE;
                default:begin state<=IDLE;status<=GEOMETRY;end
            endcase
            // 只取消封闭的本地计算；不能作为外部SD/文件生产者的停流确认。
            // 同键取消优先于待取正常结果，完整session/seq不可截断。
            if(state!=IDLE && state!=RESULT && elapsed==budget-1'b1) begin fail(DEADLINE);operation_valid<=0;end
            if(cancel_now) begin status<=CANCELED;canceled<=1;state<=RESULT;operation_valid<=0;end
        end
    end
endmodule
`default_nettype wire
