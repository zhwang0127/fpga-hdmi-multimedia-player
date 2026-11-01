`timescale 1ns/1ps
`default_nettype none

// BMP头部校验器：默认流式消费54字节(BITMAPFILEHEADER+BITMAPINFOHEADER)，
// EXT_FORMATS=1还允许32bit BI_RGB及16bit BI_BITFIELDS；565再消费12B标准掩码，共66B。
// 图片尺寸固定为 640×480，允许正负高度。格式/已消费长度只在成功结果后有效，源流保持原字节。
// 按方案§8.4冻结规则逐字段校验并输出像素区起始位移与文件长度界。
// 全部范围检查用64位(头长/位移/像素区组合)，拒绝时给出独立detail编码。
// 不做像素解码、tile映射、方向归一化或装载——那是tile装载器的职责。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
// 头字段只在字节真实握手时收集，检查使用本地已接收的字段和条件标志。
// 完成结果携带位深、像素偏移及方向，消费者仅在头检查成功后使用。
module bmp_header_check #(parameter EXT_FORMATS=0,parameter ALLOW_32=1)(
    input wire clk,arst,
    input wire start_valid,output wire start_ready,
    input wire [31:0] file_length,       // 目录给出的文件长度(字节)
    input wire byte_valid,output wire byte_ready,
    // byte_ready 去掉 abort 项的核心条件（恒有 byte_ready==byte_ready_core&&!abort）。
    // 只供父级给提交层算“同一取消已门控 byte_valid 时”的无取消 ready；不能当接受信号。
    output wire byte_ready_core,
    input wire [7:0] byte_data,
    input wire byte_last,                // 文件末字节(头部不足时触发截断拒绝)
    input wire abort,                    // 本地取消；不得代替源文件的真实停止确认
    output wire result_valid,input wire result_ready,
    output wire result_ok,output wire [7:0] result_detail,
    output wire [31:0] out_offset,       // 校验通过的bfOffBits(像素区起始)
    output wire [1:0] out_pixel_format,output wire [6:0] out_header_consumed,
    output wire out_bottom_up           // 成功描述符：正高自底向上，负高自顶向下
);
    wire rst_logic;
    localparam IDLE=0,COLLECT=1,RESULT=2;
    localparam OK=0,MAGIC=1,TRUNC=2,DIB_SIZE=3,WIDTH=4,HEIGHT=5,
               PLANES=6,BPP=7,COMPRESS=8,OFFBITS=9,FILESIZE=10,PIXELSPAN=11,
               IMAGELIMIT=12,INPUT_PROTOCOL=13,CANCEL=14;
    reg [1:0] state;
    localparam integer IDX_W=EXT_FORMATS?7:6;
    reg [IDX_W-1:0] idx;                        // 默认0..53；565到65
    reg [31:0] f_length;
    reg [7:0] magic0_raw,magic1_raw;      // 原始字段不复位，仅在对应有效条件成立时使用。
    // 需要真实算术或公开输出的字段保留完整32位raw。
    reg [31:0] bf_size_raw,bf_off_raw,dib_size_raw,image_size_raw;
    // width/height/planes/bpp/compression只参与“是否等于固定格式常量”的判定，
    // 改存逐字节相等标志（定义见下方字段段落）；height另存符号位供方向输出。
    reg [3:0] width_byte_ok;              // 640=0x00000280逐字节
    reg [3:0] height_pos_byte_ok;         // +480=0x000001E0逐字节
    reg [3:0] height_neg_byte_ok;         // -480=0xFFFFFE20逐字节
    reg height_sign_raw;                  // bmp_height[31]，即第25字节bit7
    reg [1:0] planes_byte_ok;             // 1=0x0001逐字节
    reg [1:0] bpp_byte_ok;                // 24=0x0018逐字节
    reg [3:0] compression_byte_ok;        // BI_RGB=0逐字节
    reg [7:0] detail;
    reg [7:0] verdict_cached;
    reg [32:0] header_end_cached,span_cached;
    reg [7:0] verdict_r;                  // 头字段的组合校验结果。
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 共用原三级同步条件，只复制末级负沿释放，禁止每组另起异步同步链。
    // 这样所有字段组和控制状态在同一个负沿释放；停钟/arst重新置位时仍异步隔离。
    // 每组最多32个字段位，避免原header复位副本一次驱动约65个物理SR终点。
    // preserve/禁止重定时沿用reset_sync属性；实际是否保留及recovery仍以TD报告为准。
    wire [8:0] field_rst;
    genvar reset_group;
    generate for(reset_group=0;reset_group<9;reset_group=reset_group+1) begin:g_field_reset
        (* preserve = "true", allow_forward_retiming = "false",
           allow_backward_retiming = "false" *) reg release_tail;
        always @(negedge clk or posedge rst_logic) begin
            if(rst_logic) release_tail<=1'b1;
            else release_tail<=1'b0;
        end
        assign field_rst[reset_group]=release_tail;
    end endgenerate
    // 业务门控只使用本端reset_sync输出；该rst异步置位、三级同步释放，
    // 已覆盖arst有效区间。原始arst仅进入复位链，避免sys复位直接穿过mem数据/握手锥。
    wire enabled=!rst_logic;
    assign start_ready=enabled && !abort && state==IDLE;
    assign byte_ready=enabled && !abort && state==COLLECT;
    assign byte_ready_core=enabled && state==COLLECT;
    assign result_valid=enabled && state==RESULT;
    assign result_ok=detail==OK;
    assign result_detail=detail;
    // out_offset/out_bottom_up 读取已写入寄存器的 raw 字段。
    // 父模块仅在头检查成功及后续 LOAD_START 接受沿使用它们；错误或收集中不使用描述符。
    reg [1:0] format_raw;
    reg [10:0] mask_ok;
    wire source_565=EXT_FORMATS && format_raw==2;
    wire [IDX_W-1:0] last_header=source_565?7'd65:7'd53;
    assign out_pixel_format=EXT_FORMATS?format_raw:2'd0;
    assign out_header_consumed=source_565?7'd66:7'd54;
    assign out_offset=bf_off_raw;
    assign out_bottom_up=!height_sign_raw;
    // 字段锁存映射(小端字节序)。
    // 标准BITMAPINFOHEADER顺序：size,width,height,planes,bpp,compression。
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;idx<=0;detail<=INPUT_PROTOCOL;
        end else if(abort && state==COLLECT) begin detail<=CANCEL;state<=RESULT;
        end else begin
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    idx<=0;detail<=OK;
                    state<=COLLECT;
                end
                COLLECT:if(byte_valid && byte_ready) begin
                    if(byte_last && idx!=last_header) begin
                        detail<=TRUNC;state<=RESULT;
                    end else if(idx==last_header) begin
                        // 基本头或565掩码齐：此前缓存检查优先，最后掩码字节用本沿真实输入。
                        detail<=verdict_cached!=OK?verdict_cached:
                            (source_565 && (mask_ok!==11'h7ff || byte_data!==8'd0 ||
                              bf_off_raw<66 || (dib_size_raw!=40 && dib_size_raw<52)))?8'd15:OK;
                        state<=RESULT;
                    end else idx<=idx+6'd1;
                end
                RESULT:if(result_valid && result_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
    // 真实字节握手先写入 staged_* 本地暂存，宽字段寄存器由本地有效标签驱动。
    // 依赖字段的检查放在其落入 raw 后的下一个字节采样点，直接读取稳定 raw，避免跨级前递。
    // 结果在 idx53 实际接受沿发布；取消只阻止新接受，不撤销已经进入暂存的字节。
    wire header_byte_take=state==COLLECT && byte_valid && byte_ready;
    reg [7:0] staged_data;reg [IDX_W-1:0] staged_idx;
    reg staged_valid;
    // magic原本不复位；保留未写入寄存器magic直到下一个真实时钟，即使此时停钟/arst。
    // field_pending单独用原异步SR：字段复位时也清零，不让负沿尾级进入业务D/E。
    reg field_pending;
    always @(posedge clk) begin
        staged_valid<=header_byte_take;
        if(header_byte_take) begin staged_data<=byte_data;staged_idx<=idx;end
    end
    always @(posedge clk or posedge rst) begin
        if(rst) field_pending<=1'b0;
        else field_pending<=header_byte_take;
    end
    always @(posedge clk or posedge field_rst[0]) begin
        if(field_rst[0]) f_length<=0;
        else if(start_valid && start_ready) f_length<=file_length;
    end
    // 下面四个完整字段（bf_size/bf_off/dib_size/image_size）与Z完全相同，只是删掉前递视图。
    always @(posedge clk or posedge field_rst[1]) begin
        if(field_rst[1]) bf_size_raw<=0;
        else if(field_pending) case(staged_idx)
            6'd2:bf_size_raw[7:0]<=staged_data;
            6'd3:bf_size_raw[15:8]<=staged_data;
            6'd4:bf_size_raw[23:16]<=staged_data;
            6'd5:bf_size_raw[31:24]<=staged_data;
            default:;
        endcase
    end
    always @(posedge clk or posedge field_rst[2]) begin
        if(field_rst[2]) bf_off_raw<=0;
        else if(field_pending) case(staged_idx)
            6'd10:bf_off_raw[7:0]<=staged_data;
            6'd11:bf_off_raw[15:8]<=staged_data;
            6'd12:bf_off_raw[23:16]<=staged_data;
            6'd13:bf_off_raw[31:24]<=staged_data;
            default:;
        endcase
    end
    always @(posedge clk or posedge field_rst[3]) begin
        if(field_rst[3]) dib_size_raw<=0;
        else if(field_pending) case(staged_idx)
            6'd14:dib_size_raw[7:0]<=staged_data;
            6'd15:dib_size_raw[15:8]<=staged_data;
            6'd16:dib_size_raw[23:16]<=staged_data;
            6'd17:dib_size_raw[31:24]<=staged_data;
            default:;
        endcase
    end
    // 固定格式字段的逐字节相等标志。
    // 固定值字段使用逐字节匹配标志，表示对应字段是否等于要求的常量。
    // 复位初值按字段清零后的比较结果设置，标志仅在 field_pending 对应字节写入寄存器时更新。
    always @(posedge clk or posedge field_rst[4]) begin
        if(field_rst[4]) width_byte_ok<=4'b1100;
        else if(field_pending) case(staged_idx)
            6'd18:width_byte_ok[0]<=staged_data==8'h80;
            6'd19:width_byte_ok[1]<=staged_data==8'h02;
            6'd20:width_byte_ok[2]<=staged_data==8'h00;
            6'd21:width_byte_ok[3]<=staged_data==8'h00;
            default:;
        endcase
    end
    always @(posedge clk or posedge field_rst[5]) begin
        if(field_rst[5]) begin
            height_pos_byte_ok<=4'b1100;height_neg_byte_ok<=4'b0000;height_sign_raw<=1'b0;
        end else if(field_pending) case(staged_idx)
            6'd22:begin height_pos_byte_ok[0]<=staged_data==8'he0;height_neg_byte_ok[0]<=staged_data==8'h20;end
            6'd23:begin height_pos_byte_ok[1]<=staged_data==8'h01;height_neg_byte_ok[1]<=staged_data==8'hfe;end
            6'd24:begin height_pos_byte_ok[2]<=staged_data==8'h00;height_neg_byte_ok[2]<=staged_data==8'hff;end
            6'd25:begin
                height_pos_byte_ok[3]<=staged_data==8'h00;height_neg_byte_ok[3]<=staged_data==8'hff;
                height_sign_raw<=staged_data[7];
            end
            default:;
        endcase
    end
    always @(posedge clk or posedge field_rst[6]) begin
        if(field_rst[6]) begin planes_byte_ok<=2'b10;bpp_byte_ok<=2'b10;format_raw<=0;end
        else if(field_pending) case(staged_idx)
            6'd26:planes_byte_ok[0]<=staged_data==8'h01;
            6'd27:planes_byte_ok[1]<=staged_data==8'h00;
            6'd28:begin bpp_byte_ok[0]<=staged_data==8'h18 || (EXT_FORMATS && (staged_data==8'h10 || (ALLOW_32 && staged_data==8'h20)));
                format_raw<=staged_data==8'h10 ? 2'd2 : (ALLOW_32 && staged_data==8'h20) ? 2'd1 : 2'd0;end
            6'd29:bpp_byte_ok[1]<=staged_data==8'h00;
            default:;
        endcase
    end
    always @(posedge clk or posedge field_rst[7]) begin
        if(field_rst[7]) compression_byte_ok<=4'b1111;
        else if(field_pending) case(staged_idx)
            6'd30:compression_byte_ok[0]<=staged_data==(source_565?8'h03:8'h00);
            6'd31:compression_byte_ok[1]<=staged_data==8'h00;
            6'd32:compression_byte_ok[2]<=staged_data==8'h00;
            6'd33:compression_byte_ok[3]<=staged_data==8'h00;
            default:;
        endcase
    end
    // image_size当前不参与判定/输出（综合会对称删除），保持Z原样以免改动第9复位组。
    always @(posedge clk or posedge field_rst[8]) begin
        if(field_rst[8]) image_size_raw<=0;
        else if(field_pending) case(staged_idx)
            6'd34:image_size_raw[7:0]<=staged_data;
            6'd35:image_size_raw[15:8]<=staged_data;
            6'd36:image_size_raw[23:16]<=staged_data;
            6'd37:image_size_raw[31:24]<=staged_data;
            default:;
        endcase
    end
    // magic 字段按接受字节写入；未写入或含未知位时，!== 比较将其判为 MAGIC 错误。
    always @(posedge clk) begin
        if(staged_valid) begin
            if(staged_idx==6'd0) magic0_raw<=staged_data;
            if(staged_idx==6'd1) magic1_raw<=staged_data;
        end
    end
    // 长度边界缓存仅在对应字段已落入 raw 后采样：像素偏移在 idx15，DIB 长度在 idx19，压缩字段在 idx35。
    // 字节先进入 staged 暂存，下一拍写入 raw；采样点至少晚于字段末字节两次接受。
    // 完整头在发布结果前必经这些采样点，缓存无需复位；取消及截断由控制模块优先处理。
    always @(posedge clk) begin
        if(!rst_logic && state==COLLECT) begin
            if(idx==(EXT_FORMATS?7'd35:7'd15)) span_cached<={1'b0,bf_off_raw}+
                (EXT_FORMATS && format_raw==2?33'd614400:EXT_FORMATS && format_raw==1?33'd1228800:33'd921600);
            if(idx==6'd19) header_end_cached<={1'b0,dib_size_raw}+33'd14;
            if(idx==(EXT_FORMATS?7'd37:7'd35)) verdict_cached<=verdict_r;
        end
    end
    // 565的三个标准颜色掩码从byte54开始；最后byte65同沿判0，避免旧值旁路。
    // 只增加11个局部标志，不在父/子各建完整32位mask副本。
    always @(posedge clk) if(EXT_FORMATS && header_byte_take)begin
        case(idx)
            54:mask_ok[0]<=byte_data==0;55:mask_ok[1]<=byte_data==8'hf8;
            56:mask_ok[2]<=byte_data==0;57:mask_ok[3]<=byte_data==0;
            58:mask_ok[4]<=byte_data==8'he0;59:mask_ok[5]<=byte_data==8'h07;
            60:mask_ok[6]<=byte_data==0;61:mask_ok[7]<=byte_data==0;
            62:mask_ok[8]<=byte_data==8'h1f;63:mask_ok[9]<=byte_data==0;
            64:mask_ok[10]<=byte_data==0;
            default:;
        endcase
    end
    // 判定只读raw/标志/已缓存算术，不读staged_*，所以没有staged_idx到判定的长锥。
    // 33位足以容纳任意32位DIB/offset加固定正数，不截断溢出；bf_size/f_length范围算术原样保留。
    wire [63:0] flen64={32'd0,f_length};
    wire [63:0] bfsz64={32'd0,bf_size_raw};
    wire width_ok=&width_byte_ok;
    wire height_ok=(&height_pos_byte_ok) || (&height_neg_byte_ok);
    wire planes_ok=&planes_byte_ok;
    wire bpp_ok=&bpp_byte_ok;
    wire compression_ok=&compression_byte_ok;
    always @(*) begin
        verdict_r=OK;
        if({magic1_raw,magic0_raw}!==16'h4d42) verdict_r=MAGIC;
        else if(dib_size_raw<32'd40) verdict_r=DIB_SIZE;
        else if(!width_ok) verdict_r=WIDTH;
        else if(!height_ok) verdict_r=HEIGHT;
        else if(!planes_ok) verdict_r=PLANES;
        else if(!bpp_ok) verdict_r=BPP;
        else if(!compression_ok) verdict_r=COMPRESS;
        else if({32'd0,bf_off_raw}<{31'd0,header_end_cached}) verdict_r=OFFBITS;
        else if({31'd0,span_cached}>flen64) verdict_r=PIXELSPAN;
        else if(bf_size_raw!=32'd0 && (bfsz64<{31'd0,span_cached} || bfsz64>flen64))
            verdict_r=FILESIZE;
    end
endmodule
`default_nettype wire
