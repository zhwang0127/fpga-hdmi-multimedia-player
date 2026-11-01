`timescale 1ns/1ps
`default_nettype none
// 固定640x480 BMP像素装载：默认24bit，EXT_FORMATS=1再接32bit BGRX与16bit RGB565。
// 接收已通过头检的描述符与header_consumed之后的原始文件字节；不转换或重算源文件CRC。
// 8像素缓冲全部就绪后才请求BL8，保证真实SDR连续写拍不会被字节源饿死。
// 本模块只写未提交槽；本地成功仍须由控制模块联合整文件CRC/source_done提交。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module bmp_tile_loader #(parameter EXT_FORMATS=0,parameter ALLOW_32=1,parameter ENABLE_PIXEL_TAP=0,parameter USE_LAUNCH_FLAGS=0)(
    input wire clk,arst,
    input wire start_valid,output wire start_ready,
    input wire [31:0] file_length,pixel_offset,
    input wire bottom_up,input wire [1:0] slot,
    input wire [1:0] pixel_format,input wire [6:0] header_consumed,
    input wire [31:0] budget_cycles,input wire abort,
    // 联合父模块在 HEADER/LOAD_START 准备拍缓存完整描述符及预算判据。
    // 仅USE_LAUNCH_FLAGS=1采用，且只在真实start接受沿使用；默认独立接口仍现场全检。
    input wire launch_budget_zero,launch_budget_le_one,launch_descriptor_bad,
    // 父模块在头检成功时缓存 pixel_offset==54，避免启动路径再次进行宽比较。
    // USE_LAUNCH_FLAGS=1 时采用该标志，且必须与 start 握手沿的 pixel_offset 一致。
    // 独立接口 USE_LAUNCH_FLAGS=0 忽略标志，在本地直接比较。
    input wire launch_offset_is_54,
    input wire byte_valid,output wire byte_ready,
    // byte_ready 去掉 abort 项后的核心条件，恒有 byte_ready==byte_ready_core&&!abort。
    // 只给父级/提交层在“已由同一取消门控 byte_valid”的场合消去重复取消项，不能当接受信号。
    output wire byte_ready_core,
    input wire [7:0] byte_data,input wire byte_last,
    // 可选同源RGB握手：只在合法R字节上发布；默认关闭，启用时提供同源 RGB 接受事件。
    output wire pixel_valid,input wire pixel_ready,output wire [23:0] pixel_rgb,output wire pixel_last,
    // pixel_valid_core 为 pixel_valid 去掉 abort 项（pixel_valid==core&&!abort）。
    // pixel_ready_core 输入为下游 pixel_ready 去掉同一 abort 项的核心条件：联合父接缩略图的
    // pixel_ready_core（两者共用 load_abort）；独立调用者必须直接接 pixel_ready。
    // pixel_ready 保留为接口兼容输入；内部使用 pixel_ready_core 形成共享取消条件下的握手。
    // 上层需保证 RGB 消费者与本模块使用同一取消源。
    output wire pixel_valid_core,input wire pixel_ready_core,
    output wire seg_valid,input wire seg_ready,output wire [20:0] seg_addr,
    // 仅供同父双子仲裁的请求意向，尚未计共同abort，绝不能当真实valid接受。
    // 独立调用者可不连接；真实握手始终仍是seg_valid && seg_ready。
    output wire seg_intent,
    output wire [3:0] seg_bl8_count,output wire [31:0] seg_budget_cycles,
    output wire wr_data_valid,output wire [31:0] wr_data,input wire wr_beat,
    input wire seg_done,seg_done_was_write,
    output wire result_valid,input wire result_ready,
    output wire result_ok,output wire [7:0] result_detail,
    output wire [18:0] out_written_words
);
    wire rst_logic;
    localparam IDLE=0,SKIP=1,FILL=2,REQUEST=3,WAIT_WRITE=4,TAIL=5,RESULT=6;
    localparam OK=0,CANCEL=1,DEADLINE=2,TRUNC=3,PROTOCOL=4,DESCRIPTOR=5,WRITE_COUNT=6;
    reg [2:0] state;reg [7:0] detail;
    // 32bit整图1228800B超过1MiB，源位置必须21位；像素数/SDRAM布局仍是307200个32bit字。
    // 基础 BMP24 路径使用 20 位位置和长度计数。
    // 扩展格式使用覆盖允许文件长度的 POS_W，描述符先检查完整长度再使用窄计数。
    localparam integer POS_W=(EXT_FORMATS && ALLOW_32)?21:20;
    reg [POS_W-1:0] pos,flen;reg [12:0] offset;reg [31:0] remaining;
    // 预算接受沿先算“是否已到最后一个准入周期”，之后每拍递推。
    // 原elapsed>=budget-1仍在同一沿截止；不能把32位减法/比较串到byte_ready和bank许可。
    // 计数在到期后饱和，WAIT_WRITE仍按真实写拍排空，不因本地期限截断BL8。
    reg deadline_q;
    // 末字关系与pos同拍递推，避免flen-1的减法/比较经过两消费者握手扇出。
    // pos只在真实字节接受沿前进，反压/写突发期间保持；位置按 20 位模运算递增。
    reg [POS_W-1:0] flen_penultimate;
    reg at_eof;
    // 缓存当前pos+1==offset，切开pos加法/比较到状态CE的路径。
    // 起始沿缓存offset-2，之后真实字节接受时只比较旧pos；完整20位模运算
    // 包括非法描述符/回绕也保持身份，不借用头检或缩减原边界检查。
    reg [POS_W-1:0] skip_before_last;reg skip_last_q;
    reg direction;reg [1:0] target_slot,channel;
    reg [9:0] x;reg [8:0] row;
    reg [7:0] blue,green,red;
    reg [1:0] format_q;
    wire [6:0] consumed=EXT_FORMATS?header_consumed:7'd54;
    wire [31:0] input_pixels=EXT_FORMATS && pixel_format==2?32'd614400:
                             EXT_FORMATS && ALLOW_32 && pixel_format==1?32'd1228800:32'd921600;
    wire [31:0] format_room=file_length-input_pixels;
    wire pixel_end=EXT_FORMATS?(format_q==2?channel==1:ALLOW_32 && format_q==1?channel==3:channel==2):channel==2;
    // channel=3 仅在启用 32 位格式时消费 alpha 字节。
    wire pixel_complete=EXT_FORMATS?pixel_end:(channel!=0 && channel!=1);
    // 565按高位复制扩展到8bit；32bit在alpha字节接受沿输出先前收齐的B/G/R，忽略alpha值。
    // 两者仍只产生一个RGB接受点，缩略图与主图不会重复消费或错一像素。
    wire [23:0] assembled_rgb=EXT_FORMATS && format_q==2?
        {{byte_data[7:3],byte_data[7:5]},{byte_data[2:0],blue[7:5],byte_data[2:1]},{blue[4:0],blue[4:2]}}:
        EXT_FORMATS && ALLOW_32 && format_q==1?{red,green,blue}:{byte_data,green,blue};
    // 像素缓存由 8 个 24 位 RGB888 字组成，写 SDRAM 时高 8 位补零。
    reg [191:0] pixels;reg [3:0] beats;
    // 缓存真实WAIT_WRITE且未供满8字的条件，避免state/beat比较每拍串到bank和pad。
    // 本地请求接受沿即打开；父槽排队时数据保持，真实WRITE命令仍同沿消费第0字。
    // 只有真实第8次wr_beat或真实seg_done关闭，取消/到期不撤销已接受BL8供数。
    reg write_supply_q;
    reg [18:0] parsed,written;
    // parsed_full_q 表示已解析 307200 个像素，pos_eq_flen_q 表示已消费完整文件。
    // 两者在启动及实际字节接受沿按下一计数值更新，供 WAIT_WRITE 结束时选择继续、尾部或结果状态。
    // pos+1==flen 直接复用 at_eof，避免再增加一条宽加法比较链。
    reg parsed_full_q,pos_eq_flen_q;
    // parsed_last_q 与像素计数同步更新，始终表示 parsed==307199。
    // 最后像素条件提前寄存，供缩略图消费者使用，避免比较计数后再进入握手路径。
    reg parsed_last_q;
    reg [20:0] address;
    wire rst;reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 业务门控只使用本端reset_sync输出；该rst异步置位、三级同步释放，
    // 已覆盖arst有效区间。原始arst仅进入复位链，避免sys复位直接穿过mem数据/握手锥。
    wire enabled=!rst_logic;
    wire active=state!=IDLE && state!=RESULT;
    wire expired=active && deadline_q;
    wire [12:0] offset_room=file_length[12:0]^13'h1000;
    wire descriptor_bad_live=EXT_FORMATS?
                        (pixel_format==3 || (!ALLOW_32 && pixel_format==1) || (header_consumed!=54 && header_consumed!=66) ||
                         pixel_offset<{25'd0,consumed} || pixel_offset>4096 ||
                         file_length<input_pixels || format_room>4096 || pixel_offset>format_room):
                        (pixel_offset<32'd54 || pixel_offset>32'd4096 ||
                         file_length<32'he1000 || file_length>32'he2000 ||
                         pixel_offset[12:0]>offset_room);
    wire descriptor_bad=USE_LAUNCH_FLAGS ? launch_descriptor_bad : descriptor_bad_live;
    wire budget_zero=USE_LAUNCH_FLAGS ? launch_budget_zero : budget_cycles==0;
    wire budget_le_one=USE_LAUNCH_FLAGS ? launch_budget_le_one : budget_cycles<=32'd1;
    // 起始状态选择SKIP/FILL的偏移判据；联合父用寄存标志切断raw→state组合锥。
    wire offset_is_54=USE_LAUNCH_FLAGS ? launch_offset_is_54 : pixel_offset=={25'd0,consumed};
    wire [8:0] y=direction ? 9'd479-row : row;
    // 槽跨度1280个tile，恰为64行×20列；先拼接slot与tile行，再乘20。
    // 保留完整9bit行回绕与21bit字地址；这里只用组合恒等式，不增等待拍。
    wire [7:0] global_tile_row={target_slot,y[8:3]};
    wire [10:0] row_times_five={1'b0,global_tile_row,2'b0}+{3'd0,global_tile_row};
    wire [12:0] global_tile={row_times_five,2'b0}+{8'd0,x[9:5]};
    wire [20:0] next_word_address={global_tile,y[2:0],x[4:0]};

    assign start_ready=enabled && state==IDLE;
    wire eof_correct=byte_last==at_eof;
    // 状态和错误码分别计算取消前后的候选值，在末级按 abort 选择。
    // 数据寄存器仍只由真实字节或写拍握手推进，候选状态不产生额外接受事件。
    wire core_ok=!expired && detail==OK;
    assign pixel_valid_core=ENABLE_PIXEL_TAP && core_ok && state==FILL && pixel_end && byte_valid && eof_correct;
    assign pixel_valid=pixel_valid_core && !abort;   // 取消时屏蔽像素有效
    assign pixel_rgb=ENABLE_PIXEL_TAP ? assembled_rgb : 24'd0;
    assign pixel_last=ENABLE_PIXEL_TAP && parsed_last_q;   // 原 parsed==19'd307199，用预寄存标志
    // 两消费者以同一个R字节接受边沿前进；错last必须能先拒绝，不能被缩略图背压锁死。
    assign byte_ready_core=core_ok && (state==SKIP || state==FILL || state==TAIL) &&
        (state!=FILL || !pixel_end || !ENABLE_PIXEL_TAP || pixel_ready_core || !eof_correct);
    assign byte_ready=byte_ready_core && !abort;
    // 真实字节接受的 abort=0 余因子；数据寄存器 CE 只在末级与 !abort 相与。
    (* keep *) wire byte_take_core=(state==SKIP || state==FILL || state==TAIL) && byte_valid && byte_ready_core;
    wire byte_take=byte_take_core && !abort;          // ==在SKIP/FILL/TAIL且byte_valid&&byte_ready
    assign seg_intent=!expired && detail==OK && state==REQUEST;   // enabled 冗余（state==REQUEST 蕴含）
    assign seg_valid=seg_intent && !abort;
    assign seg_addr=address;
    assign seg_bl8_count=4'd1;
    // 已接受的突发必须排空：本地期限不截断物理写。控制器段预算固定大于一条BL8。
    assign seg_budget_cycles=32'd4096;
    assign wr_data_valid=write_supply_q;   // enabled 冗余（write_supply_q 由 rst 异步清零）
    // 写段第 b 拍读取固定缓存的第 beats[2:0] 个 RGB888 像素，扩展为 32 位写数据。
    // 数据仅在 wr_data_valid 有效时使用，物理缓存不随写拍移动。
    reg [23:0] rd_word;
    always @* begin
        case(beats[2:0])
            3'd0: rd_word=pixels[  0 +: 24];
            3'd1: rd_word=pixels[ 24 +: 24];
            3'd2: rd_word=pixels[ 48 +: 24];
            3'd3: rd_word=pixels[ 72 +: 24];
            3'd4: rd_word=pixels[ 96 +: 24];
            3'd5: rd_word=pixels[120 +: 24];
            3'd6: rd_word=pixels[144 +: 24];
            default: rd_word=pixels[168 +: 24];
        endcase
    end
    assign wr_data={8'h00,rd_word};
    assign result_valid=enabled && state==RESULT;
    assign result_ok=detail==OK;
    assign result_detail=detail;
    assign out_written_words=written;
    always @(posedge clk or posedge rst) begin
        if(rst) write_supply_q<=0;
        else case(state)
            REQUEST:write_supply_q<=seg_valid && seg_ready;
            WAIT_WRITE:if(seg_done || (wr_beat && wr_data_valid && beats==4'd7)) write_supply_q<=0;
            default:write_supply_q<=0;
        endcase
    end
    // 分别计算 abort=0 和 abort=1 的状态候选，在末级按取消输入选择。
    // 每份候选使用显式 always @* 读取全部组合输入，确保敏感表完整。
    // 数据写入仍由实际字节或写拍握手推进；WAIT_WRITE 必须供满 BL8 并等待完成。
    reg [2:0] state_c0,state_c1;reg sn_c0,sn_c1;
    // abort=0 的余因子
        always @* begin
            sn_c0=1'b0 || expired || detail!=OK;
            state_c0=state;
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    if(descriptor_bad || budget_zero) state_c0=RESULT;
                    else state_c0=offset_is_54 ? FILL : SKIP;
                end
                SKIP:if(byte_valid) begin
                    if(!eof_correct) state_c0=RESULT;
                    else if(skip_last_q) state_c0=FILL;
                end
                FILL:if(byte_valid &&
                        (!pixel_end || !ENABLE_PIXEL_TAP || (pixel_ready_core && !1'b0) || !eof_correct)) begin
                    if(!eof_correct) state_c0=RESULT;
                    else if(pixel_complete && x[2:0]==7) state_c0=REQUEST;
                end
                TAIL:if(byte_valid && (!eof_correct || byte_last)) state_c0=RESULT;
                REQUEST:if(seg_ready) state_c0=WAIT_WRITE;
                WAIT_WRITE:if(seg_done) begin
                    // 本块是 wr_beat=0 的余因子：原 (beats!=8 && !(beats==7 && wr_beat && wr_data_valid))
                    // 在 wr_beat=0 时化为 beats!=8；wr_beat=1 的差异只在 beat_hit 拍，由末级覆盖（见 state 寄存器处）。
                    if(!seg_done_was_write || beats!=8 || sn_c0)
                        state_c0=RESULT;
                    else if(parsed_full_q) state_c0=pos_eq_flen_q ? RESULT : TAIL;
                    else state_c0=FILL;
                end
                RESULT:if(result_valid && result_ready) state_c0=IDLE;
                default:;
            endcase
            // 原末级：active 且非 WAIT_WRITE 时 stop_now 优先转 RESULT。
            if(active && state!=WAIT_WRITE && sn_c0) state_c0=RESULT;
        end
    // abort=1 的余因子
        always @* begin
            sn_c1=1'b1 || expired || detail!=OK;
            state_c1=state;
            case(state)
                IDLE:if(start_valid && start_ready) begin
                    if(descriptor_bad || budget_zero) state_c1=RESULT;
                    else state_c1=offset_is_54 ? FILL : SKIP;
                end
                SKIP:if(byte_valid) begin
                    if(!eof_correct) state_c1=RESULT;
                    else if(skip_last_q) state_c1=FILL;
                end
                FILL:if(byte_valid &&
                        (!pixel_end || !ENABLE_PIXEL_TAP || (pixel_ready_core && !1'b1) || !eof_correct)) begin
                    if(!eof_correct) state_c1=RESULT;
                    else if(pixel_complete && x[2:0]==7) state_c1=REQUEST;
                end
                TAIL:if(byte_valid && (!eof_correct || byte_last)) state_c1=RESULT;
                REQUEST:if(seg_ready) state_c1=WAIT_WRITE;
                WAIT_WRITE:if(seg_done) begin
                    // wr_beat=0 余因子，说明同上。
                    if(!seg_done_was_write || beats!=8 || sn_c1)
                        state_c1=RESULT;
                    else if(parsed_full_q) state_c1=pos_eq_flen_q ? RESULT : TAIL;
                    else state_c1=FILL;
                end
                RESULT:if(result_valid && result_ready) state_c1=IDLE;
                default:;
            endcase
            // 原末级：active 且非 WAIT_WRITE 时 stop_now 优先转 RESULT。
            if(active && state!=WAIT_WRITE && sn_c1) state_c1=RESULT;
        end
    (* keep *) wire [2:0] state_a0=state_c0;
    (* keep *) wire [2:0] state_a1=state_c1;
    // 末次写拍与 seg_done 可同拍到达；beat_hit 预先表示已处于第八拍完成边界。
    // 分别形成 wr_beat=0 和写满八拍的状态/错误码候选，再在末级选择。
    // 这样将晚到的实际写拍与宽状态译码分开，同时保留取消、截止和写数错误的优先级。
    (* keep *) wire beat_hit=state==WAIT_WRITE && seg_done && seg_done_was_write && beats==4'd7 && wr_data_valid;
    // 命中拍、未取消/未到期且 detail==OK 时的正常续行目标。
    wire [2:0] state_hit_go=parsed_full_q ? (pos_eq_flen_q ? RESULT : TAIL) : FILL;   // 用预寄存标志
    (* keep *) wire [2:0] state_hit=abort ? RESULT : ((expired || detail!=OK) ? RESULT : state_hit_go);
    always @(posedge clk or posedge rst) begin
        if(rst) state<=IDLE;
        else state<=(wr_beat && beat_hit) ? state_hit : (abort ? state_a1 : state_a0);
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            parsed_full_q<=0;pos_eq_flen_q<=1;   // 复位 parsed=0、pos=flen=0
            parsed_last_q<=0;   // 复位 parsed=0，0!=307199
            pos<=0;flen<=0;offset<=0;remaining<=0;deadline_q<=1;flen_penultimate<={POS_W{1'b1}}-1'b1;at_eof<=0;skip_before_last<={POS_W{1'b1}}-1'b1;skip_last_q<=0;
            direction<=0;target_slot<=0;channel<=0;x<=0;row<=0;blue<=0;green<=0;red<=0;format_q<=0;
            beats<=0;parsed<=0;written<=0;address<=0;
        end else begin
            if(active && !expired) begin
                remaining<=remaining-32'd1;
                // 本沿之前剩2，下一沿之前恰好剩1；零/一预算在描述符接受沿处理。
                deadline_q<=remaining==32'd2;
            end
            // 数据只认真实接受事件；状态候选绝不驱动它们，坏last不推进RGB坐标。
            if(state==WAIT_WRITE && wr_beat && wr_data_valid) begin
                beats<=beats+4'd1;written<=written+19'd1;
            end
            if(state==IDLE && start_valid && start_ready) begin
                pos<=consumed;flen<=file_length[POS_W-1:0];
                format_q<=EXT_FORMATS?pixel_format:2'd0;offset<=pixel_offset[12:0];
                skip_before_last<={7'd0,pixel_offset[12:0]}-20'd2;
                skip_last_q<=pixel_offset[12:0]==({6'd0,consumed}+13'd1);
                flen_penultimate<=file_length[POS_W-1:0]-2;
                at_eof<=file_length[POS_W-1:0]==({{(POS_W-7){1'b0}},consumed}+1'b1);
                remaining<=budget_cycles;deadline_q<=budget_le_one;
                direction<=bottom_up;target_slot<=slot;channel<=0;x<=0;row<=0;
                parsed<=0;written<=0;beats<=0;
                parsed_full_q<=0;pos_eq_flen_q<=file_length[POS_W-1:0]==consumed;   // pos<=54
                parsed_last_q<=0;   // 启动沿 parsed<=0
            end
            if(byte_take) begin
                pos<=pos+20'd1;at_eof<=pos==flen_penultimate;skip_last_q<=pos==skip_before_last;
                pos_eq_flen_q<=at_eof;   // pos+1==flen 恰为当前 at_eof
                if(state==FILL && eof_correct) begin
                    if(!pixel_complete) begin
                        if(channel==0)begin blue<=byte_data;if(x[2:0]==0)address<=next_word_address;end
                        if(channel==1)green<=byte_data;
                        if(channel==2)red<=byte_data;
                        channel<=channel+1'b1;
                    end else begin
                        channel<=0;parsed<=parsed+19'd1;
                        parsed_full_q<=parsed==19'd307199;   // parsed+1==307200
                        parsed_last_q<=parsed==19'd307198;   // parsed+1==307199
                        if(x==639) begin x<=0;row<=row+9'd1;end else x<=x+10'd1;
                    end
                end
            end
            if(state==REQUEST && seg_valid && seg_ready) beats<=0;
        end
    end
    // 实际像素接受和 SDRAM 写拍分别由 rgb_take、write_take 表示。
    // WAIT_WRITE 中即使收到取消也必须供满已接受的 BL8；两类事件按状态互斥。
    // 物理像素缓存按本地预写规则更新。
    wire rgb_take_core=state==FILL && pixel_end && byte_valid && byte_ready_core && eof_correct;
    wire rgb_take=rgb_take_core && !abort;
    wire write_take=state==WAIT_WRITE && wr_beat && wr_data_valid;
    // 8 个 RGB888 像素按 x[2:0] 分别写入固定槽位，写拍只更新 beats/written，不移动整组数据。
    // 640 是 8 的倍数，进入 REQUEST 前全部槽位均已覆盖，无需每次装载或写后清空。
    integer pw;
    always @(posedge clk or posedge rst) begin
        if(rst) pixels<=0;
        // 常量下标展开：第 k 字只在本拍样本序号为 k 时装入，每字一个 CE，不经可变移位网络。
        // 当前像素槽在 FILL/pixel_end 期间可反复写入，x 只在实际字节握手后前进。
        // 接受当拍写定最终 RGB；进入 REQUEST 前，8 个槽均已由有效像素覆盖。
        // 因此写使能只需本地状态和通道条件，不必串接上游字节反压及取消比较。
        // 若 EOF 错误或取消使事务失败，缓存不再发送；下次装载会从 x=0 重新覆盖。
        else if(state==FILL && pixel_end)
            for(pw=0;pw<8;pw=pw+1)
                // 索引范围为 0..7，比较时两侧均使用 3 位。
                if(x[2:0]==pw[2:0]) pixels[pw*24 +: 24]<=assembled_rgb;
    end
`ifdef FORMAL_AE_PIXELS
    reg [255:0] f_old_pixels;
    always @(posedge clk or posedge rst) begin
        if(rst) f_old_pixels<=0;
        else if(state==IDLE && start_valid && start_ready) f_old_pixels<=0;
        else if(write_take) f_old_pixels<={32'd0,f_old_pixels[255:32]};
        else if(rgb_take) f_old_pixels<={{8'h00,assembled_rgb},f_old_pixels[255:32]};
    end
    integer fj;
    // 新存储按原 32 位字格式展开（高 8 位补 0），便于与影子逐字比较。
    reg [255:0] f_new_view;
    always @* for(fj=0;fj<8;fj=fj+1) f_new_view[fj*32 +: 32]={8'h00,pixels[fj*24 +: 24]};
    always @* begin
        if(wr_data_valid) assert(wr_data==f_old_pixels[31:0]);
        if(write_supply_q) assert(state==WAIT_WRITE && beats<4'd8);
        // 32 位格式的 channel=3 对应 alpha 字节，参考状态须计入该通道。
        if(!EXT_FORMATS) assert(channel!=2'd3);
        if(state==SKIP) assert(x[2:0]==3'd0 && f_old_pixels==256'd0);
        if(state==REQUEST) assert(x[2:0]==3'd0 && f_old_pixels==f_new_view);
        if(state==WAIT_WRITE) assert(x[2:0]==3'd0 && beats<=4'd8);
        for(fj=0;fj<8;fj=fj+1) begin
            if(state==FILL) assert(f_old_pixels[fj*32 +: 32]==
                (fj>=8-x[2:0] ? f_new_view[(fj-(8-x[2:0]))*32 +: 32] : 32'd0));
            if(state==WAIT_WRITE) assert(f_old_pixels[fj*32 +: 32]==
                (fj+beats<8 ? f_new_view[(fj+beats)*32 +: 32] : 32'd0));
        end
    end
`endif
    // 错误码与控制状态分别更新，取消、到期和写数错误保持明确优先级。
    wire write_count_bad=state==WAIT_WRITE && seg_done && (!seg_done_was_write || beats!=8);
    // 分别计算 abort=0 和 abort=1 的错误码候选，在末级选择。
    // 显式 always @* 包含全部组合输入，错误优先级由各分支的覆盖顺序决定。
    reg [7:0] detail_c0,detail_c1;
    // abort=0 的余因子
        always @* begin
            detail_c0=detail;
            if(state==IDLE && start_valid && start_ready)
                detail_c0=descriptor_bad ? DESCRIPTOR : budget_zero ? DEADLINE : OK;
            else if(write_count_bad) detail_c0=WRITE_COUNT;
            else if(active && detail==OK && (1'b0 || expired)) detail_c0=1'b0 ? CANCEL : DEADLINE;
            else if(detail==OK && byte_valid &&
                    (state==SKIP || state==FILL || state==TAIL) && byte_last!=at_eof)
                detail_c0=byte_last ? TRUNC : PROTOCOL;
        end
    // abort=1 的余因子
        always @* begin
            detail_c1=detail;
            if(state==IDLE && start_valid && start_ready)
                detail_c1=descriptor_bad ? DESCRIPTOR : budget_zero ? DEADLINE : OK;
            else if(write_count_bad) detail_c1=WRITE_COUNT;
            else if(active && detail==OK && (1'b1 || expired)) detail_c1=1'b1 ? CANCEL : DEADLINE;
            else if(detail==OK && byte_valid &&
                    (state==SKIP || state==FILL || state==TAIL) && byte_last!=at_eof)
                detail_c1=byte_last ? TRUNC : PROTOCOL;
        end
    (* keep *) wire [7:0] detail_a0=detail_c0;
    (* keep *) wire [7:0] detail_a1=detail_c1;
    // 第 8 拍与 done 同拍（wr_beat&&beat_hit）时的 detail：不是写数错误，只保留取消/到期的首次锁存。
    (* keep *) wire [7:0] detail_hit=abort ? (detail==OK ? CANCEL : detail) :
                                        ((detail==OK && expired) ? DEADLINE : detail);
    always @(posedge clk or posedge rst) begin
        if(rst) detail<=OK;
        else detail<=(wr_beat && beat_hit) ? detail_hit : (abort ? detail_a1 : detail_a0);
    end
`ifdef FORMAL
    always @* if(!rst) begin
        assert(skip_before_last==({{(POS_W-13){1'b0}},offset}-{{(POS_W-2){1'b0}},2'd2}));
        assert(skip_last_q==((pos+1'b1)=={{(POS_W-13){1'b0}},offset}));
        assert(parsed_last_q==(parsed==19'd307199));
    end
`endif
endmodule
`default_nettype wire
