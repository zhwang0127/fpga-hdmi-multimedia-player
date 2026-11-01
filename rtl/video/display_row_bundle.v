`timescale 1ns/1ps
`default_nettype none
// 双图640+640+缩略图128，一次完整标签请求/一次完成发布；2x2048x32行RAM。
// payload={epoch[EPOCH_W],old_slot2,old_gen32,new_slot2,new_gen32,y9,thumb_slot2,
//          thumb_gen32,thumb_y7,bank1,deadline16}，共(135+EPOCH_W)bit。
// 所有来源引用须由调用方保留到使用结束；主引用可接display_read_owner 的行读取完成确认。
// LINE_BITS 选择行 RAM 的物理位宽（24 或 32）。整机只使用 RGB 低 24 位。
// 24 位配置不存储高 8 位，读出时补零；显式位宽用于确定 RAM 端口及 byte-enable 映射。
// rst 异步置位、同步释放，业务使能使用本域复位。
// EPOCH_W 默认 32 位、整机 8 位，全部连接同宽；槽世代保持 32 位，一次仅一个在途换图。
module display_row_bundle #(parameter integer EPOCH_W=32,parameter USE_EG_BRAM=1,parameter LINE_BITS=32,parameter FETCH_THUMBNAILS=1)(
    input wire arst,px_clk,mem_clk,
    input wire req_valid,output wire req_ready,input wire [EPOCH_W+134:0] req_bus,
    input wire rel_valid,output wire rel_ready,input wire [EPOCH_W+118:0] rel_bus,
    output wire evt_valid,input wire evt_ready,output wire [EPOCH_W+136:0] evt_bus,
    output wire [EPOCH_W+46:0] evt_primary,output wire fault,
    input wire rd_bank,input wire [10:0] rd_x,output wire [31:0] rd_data,
    output wire seg_valid,input wire seg_ready,output wire [20:0] seg_addr,
    output wire [3:0] seg_count,output wire [31:0] seg_budget,output wire seg_cancel,
    input wire data_valid,output wire data_ready,input wire [31:0] data_in,input wire seg_done
);
    // 当前产品thumb_layout=0：只取40个主图段，不触碰未装载缩略图槽。
    // 全缩略图配置默认仍44段，调用方必须持有相应缩略图引用；本参数不取消装载成功计数。
    wire prst_logic;
    localparam [5:0] LAST_SEGMENT=FETCH_THUMBNAILS?6'd43:6'd39;
    wire prst,mrst;reset_sync u_px_reset(.clk(px_clk),.arst(arst),.rst(prst),.logic_rst(prst_logic));
    reset_sync u_mem_reset(.clk(mem_clk),.arst(arst),.rst(mrst));
    reg [1:0] powned,pfetched;reg [EPOCH_W+118:0] ptag0,ptag1;reg pfault,pwait;
    wire req_src_ready,req_dst_valid;wire [EPOCH_W+134:0] request;
    wire rel_src_ready,rel_dst_valid;wire [EPOCH_W+118:0] release_tag;
    wire evt_src_ready,evt_dst_valid;wire [EPOCH_W+136:0] event_data;
    reg [1:0] busy,fetched;reg [EPOCH_W+118:0] tag0,tag1;
    reg filling,draining,live,event_pending;reg [1:0] event_kind;
    reg [EPOCH_W+134:0] current,event_payload;reg [5:0] segment,words;
    reg [15:0] age;
    // 剩余预算逐拍递推，恒等于(current[15:0]-age)的16位模值。
    // age 记录本次请求经过的周期数，可用于诊断。
    // 从请求沿直接装deadline，filling沿减1，保留原回绕和截止拍，不增加等待。
    reg [15:0] remaining_budget;
    // mem 域提前寄存命令地址、行内段号及请求条件，减少段选择到 SDRAM 命令的组合深度。
    // 寄存候选必须与当前标签和段计数同拍对应；地址在实际申请和续段时更新。
    reg geom_bad_q,age_eq_q;reg [20:0] seg_addr_q;
    // fl_q、fin_q提前缓存取消路径要用的条件，避免把段计数比较串到控制器。
    // 它们跟原状态同拍更新；不能为了省寄存器就直接删掉。
    reg fl_q,fin_q;
    // 段尾和字尾提前寄存，事件发布只检查本地完成条件。
    reg seg43_q,w32_q;
    // 段19/39边界与缩略图区条件随segment同拍递推，地址候选末级只看本地FF。
    // 边界条件直接参与下一段地址选择，避免在加法前重复译码 segment。
    reg seg19_q,seg39_q,seg_thumb_q;
    // 当前真实段申请和每个行RAM的写条件单独寄存，按原次态同沿递推。
    // 不缓存data_valid，不新增写拍或数据/地址延迟；实际写仍须本拍真实data_valid。
    // 复位、取消、到期和最后一段按原沿失效，不能拿这些条件伪造ready/done。
    reg seg_issue_q,line0_write_q,line1_write_q;
    wire rb=request[16],lb=release_tag[0],cb=current[16];
    wire req_accept=req_dst_valid && !filling && !draining && !event_pending && !busy[rb];
    wire rel_accept=rel_dst_valid && busy[lb] && fetched[lb] && release_tag==(lb ? tag1 : tag0);
    assign req_ready=req_src_ready && !pwait && !powned[req_bus[16]] && !pfault && !prst_logic;
    assign rel_ready=rel_src_ready && powned[rel_bus[0]] && pfetched[rel_bus[0]] &&
        rel_bus==(rel_bus[0] ? ptag1 : ptag0) && !pfault && !prst_logic;
    assign evt_valid=evt_dst_valid;assign evt_bus=event_data;assign fault=pfault;
    assign evt_primary={event_data[EPOCH_W+136:EPOCH_W+135],1'b0,event_data[16],event_data[EPOCH_W+134:135],
        event_data[100:99],event_data[98:67],event_data[66:58]};
    cdc_mailbox #(.DATA_WIDTH(EPOCH_W+135)) u_req(.arst(arst),.src_clk(px_clk),
        .src_valid(req_valid && req_ready),.src_ready(req_src_ready),.src_data(req_bus),
        .src_active(),.src_busy(),.src_done(),.dst_clk(mem_clk),.dst_valid(req_dst_valid),
        .dst_ready(req_accept),.dst_data(request),.dst_active());
    cdc_mailbox #(.DATA_WIDTH(EPOCH_W+119)) u_rel(.arst(arst),.src_clk(px_clk),
        .src_valid(rel_valid && rel_ready),.src_ready(rel_src_ready),.src_data(rel_bus),
        .src_active(),.src_busy(),.src_done(),.dst_clk(mem_clk),.dst_valid(rel_dst_valid),
        .dst_ready(rel_accept),.dst_data(release_tag),.dst_active());
    cdc_mailbox #(.DATA_WIDTH(EPOCH_W+137)) u_evt(.arst(arst),.src_clk(mem_clk),
        .src_valid(event_pending),.src_ready(evt_src_ready),.src_data({event_kind,event_payload}),
        .src_active(),.src_busy(),.src_done(),.dst_clk(px_clk),.dst_valid(evt_dst_valid),
        .dst_ready(evt_ready),.dst_data(event_data),.dst_active());
    wire eb=event_data[16];wire ematch=powned[eb] && event_data[EPOCH_W+134:16]==(eb ? ptag1 : ptag0);
    always @(posedge px_clk or posedge prst) begin
        if(prst) begin powned<=0;pfetched<=0;ptag0<=0;ptag1<=0;pfault<=0;pwait<=0;end
        else begin
            if(req_valid && req_ready) begin
                pwait<=1;
                powned[req_bus[16]]<=1;pfetched[req_bus[16]]<=0;
                if(req_bus[16]) ptag1<=req_bus[EPOCH_W+134:16];else ptag0<=req_bus[EPOCH_W+134:16];
            end
            if(evt_valid && evt_ready) begin
                pwait<=0;
                if(!ematch || event_data[EPOCH_W+136:EPOCH_W+135]==3) pfault<=1;
                else if(event_data[EPOCH_W+136:EPOCH_W+135]==0) pfetched[eb]<=1;
                else begin powned[eb]<=0;pfetched[eb]<=0;end
            end
            if(rel_valid && rel_ready) begin powned[rel_bus[0]]<=0;pfetched[rel_bus[0]]<=0;end
        end
    end
    function geom_bad;input [EPOCH_W+134:0] c;
        begin geom_bad=c[66:58]>=9'd480 || c[23:17]>=7'd96; end
    endfunction
    function [20:0] seg_addr_of;input [EPOCH_W+134:0] c;input [5:0] s;
        reg [1:0] slot;reg [5:0] idx;reg [10:0] trow,tl;reg [20:0] b;
        begin
            slot=s<20 ? c[134:133] : c[100:99];
            idx=s<20 ? s : s-6'd20;
            trow={5'd0,c[66:61]};
            tl=(trow<<4)+(trow<<2)+{5'd0,idx};
            b=(slot<<18)+(slot<<16);
            seg_addr_of=s<40 ? b+{2'd0,tl,c[60:58],5'd0} :
                21'h1c0000+{5'd0,c[57:56],14'd0}+{7'd0,c[23:17],7'd0}+{10'd0,(s-6'd40),5'd0};
        end
    endfunction
    wire invalid_geometry=geom_bad_q;
    wire hit=filling && age_eq_q;
    wire complete=live && seg_done;
    wire success=complete && fin_q;   // fin_q==(segment==43&&words==32)
    assign seg_valid=seg_issue_q;
    assign seg_count=4'd4;assign seg_budget={16'd0,remaining_budget};
    assign seg_cancel=draining || (fl_q && age_eq_q && !(seg_done && fin_q));
    assign data_ready=live && (filling || draining);
    // 行 RAM 仅在有效装载、未到期且有数据返回时写入。
    // 使用提前寄存的 fl_q 和 age_eq_q 形成写条件，减少计数比较到 RAM 使能的组合路径。
    wire write_fire=(line0_write_q || line1_write_q) && data_valid;
    assign seg_addr=seg_addr_q;
    // 下一拍 current/segment/age（与下方 always 的赋值优先级逐条对应；req_accept 时 filling=0，二者互斥）。
    wire [EPOCH_W+134:0] current_nx=req_accept ? request : current;
    wire seg_step=seg_done && fl_q && w32_q && !seg43_q;   // = complete&&filling&&words==32&&segment!=43
    // seg_addr_q 的 D 端三路并行预算后末级选择，去掉 seg_step/segment_nx 译码进加法链。
    //   seg_step   → seg_addr_of(current, segment+1)
    //   req_accept → seg_addr_of(request, 0)
    //   否则       → 保持 seg_addr_q（seg_addr_q==seg_addr_of(current,segment)）
    // seg_step 含 filling、req_accept 含 !filling，二者组合互斥，以此选择标签载入或行内续段的地址。
    // 行内步进利用分区地址的线性关系：seg_addr_of 在每个区段内对 segment 线性——主图区（s<20、20<=s<40）
    // 每段 +0x100（tl 最大 63*20+19=1279，不溢出 11 位），缩略图区（s>=40）每段 +0x20——所以
    //   s==19 → seg_addr_of(current,20)，s==39 → seg_addr_of(current,40)（两者只依赖 current 寄存器），
    //   其余 s<40 → seg_addr_q+0x100，s>=40 → seg_addr_q+0x20（寄存器直接进加法）。
    // segment 不超过 43，s==63 的回绕不可达。
    wire [20:0] seg_addr_j20=seg_addr_of(current,6'd20);
    wire [20:0] seg_addr_j40=seg_addr_of(current,6'd40);
    wire [20:0] seg_addr_step=seg19_q ? seg_addr_j20 : seg39_q ? seg_addr_j40 :
                              !seg_thumb_q ? seg_addr_q+21'h100 : seg_addr_q+21'h20;
    wire [20:0] seg_addr_req=seg_addr_of(request,6'd0);
    wire [20:0] seg_addr_d=seg_step ? seg_addr_step : req_accept ? seg_addr_req : seg_addr_q;
    wire age_eq_nx=req_accept ? request[15:0]==16'd0 : filling ? remaining_budget==16'd1 : remaining_budget==16'd0;
    // 预先计算行填充与段传输状态的下一拍条件：
    //   fl_q 的 D = 下一拍 filling&&live。下一拍 filling=1 只有两种情况：req_accept（此时 live 清 0，乘积为 0），
    //   或本拍 filling 且未被 complete 结束/hit 取消/几何非法清掉；此时 live 下一拍为 1 当且仅当本拍未 complete，
    //   且已 live 或本拍段被接受（seg_valid&&seg_ready，在 filling&&!hit&&!invalid_geometry 下等于 !live&&seg_ready）。
    //   draining 时 filling 为 0，其完成事件不会改变该条件。
    wire fl_nx=filling && !hit && !invalid_geometry && !complete && (live || seg_ready);
    //   fin_q 的 D = 下一拍 segment==43&&words==32。words 次态 = seg_step?0 : write_fire?words+1 : req_accept?0 : words；
    //   segment 次态 = seg_step?segment+1 : req_accept?0 : segment。seg_step 要求 segment!=43，故不必单列。
    wire fin_nx=!req_accept && seg43_q && (write_fire ? words==6'd31 : w32_q);
    //   段和字数的下一拍条件：segment 次态==43 ⇔ seg_step&&segment==42 或 (!seg_step&&!req_accept&&segment==43)；
    //   words 次态==32 ⇔ !seg_step && (write_fire ? words==31 : !req_accept && words==32)（write_fire 与 req_accept 互斥：前者要 filling）。
    wire seg43_nx=seg_step ? segment==(LAST_SEGMENT-6'd1) : !req_accept && seg43_q;
    wire w32_nx=!seg_step && (write_fire ? words==6'd31 : !req_accept && w32_q);
    // 新条件自身的D也必须评估；下列filling/live次态按原always的赋值覆盖顺序展开。
    wire filling_nx=req_accept ? 1'b1 :
        filling && !(complete && (words!=6'd32 || segment==LAST_SEGMENT)) && !(hit && !success) && !invalid_geometry;
    wire live_a=req_accept ? 1'b0 : (seg_valid && seg_ready) ? 1'b1 : live;
    wire live_b=(complete && filling) ? 1'b0 : live_a;
    wire live_c=(hit && !success && !(live && !seg_done)) ? 1'b0 : live_b;
    wire live_nx=(draining && complete) ? 1'b0 : live_c;
    // 新请求与行内续段并行形成申请条件，避免展开live完整覆盖优先级。
    // 只有 seg_step 完成了非末段，或尚无live且后端没接受当前段时，下一拍才可申请。
    // 所有完成/取消/到期沿不变；request的零期限/非法几何仍立即拒绝。
    wire issue_nx=req_accept ? (request[15:0]!=0 && !geom_bad(request)) :
        filling && !hit && !invalid_geometry && !age_eq_nx &&
        (seg_step || (!live && !seg_ready));
    wire seg19_nx=seg_step ? segment==6'd18 : !req_accept && seg19_q;
    wire seg39_nx=seg_step ? segment==6'd38 : !req_accept && seg39_q;
    // 完整6位模递推也保留63回绕，虽然生产LAST_SEGMENT只为39或43。
    wire seg_thumb_nx=seg_step ? (segment>=6'd39 && segment!=6'd63) : !req_accept && seg_thumb_q;
    wire write0_nx=fl_nx && !age_eq_nx && !current_nx[16];
    wire write1_nx=fl_nx && !age_eq_nx && current_nx[16];
`ifdef FORMAL
    // 段地址随行标签和段号更新。
    wire [5:0] segment_nx=seg_step ? segment+1'b1 : req_accept ? 6'd0 : segment;
    always @* assert(segment<=LAST_SEGMENT);
    always @* assert(seg_addr_d==seg_addr_of(current_nx,segment_nx));
    always @* assert(remaining_budget==current[15:0]-age);
    always @* assert(geom_bad_q==geom_bad(current) && age_eq_q==(age==current[15:0]) &&
        seg_addr_q==seg_addr_of(current,segment));
    // 寄存条件由本拍的请求、续段和完成事件推导，并与主状态同步更新。
    wire [5:0] words_nx=seg_step ? 6'd0 : write_fire ? words+1'b1 : req_accept ? 6'd0 : words;
    always @* assert(fl_nx==(filling_nx && live_nx) && fin_nx==(segment_nx==LAST_SEGMENT && words_nx==6'd32));
    always @* assert(!(draining && filling));
    always @* assert(fl_q==(filling && live) && fin_q==(segment==LAST_SEGMENT && words==6'd32));
    always @* assert(seg43_q==(segment==LAST_SEGMENT) && w32_q==(words==6'd32));
    always @* assert(seg43_nx==(segment_nx==LAST_SEGMENT) && w32_nx==(words_nx==6'd32));
    always @* assert(write_fire==(filling && !hit && data_valid && data_ready) && seg_step==(complete && filling && words==6'd32 && segment!=LAST_SEGMENT));
    always @* assert(seg_cancel==(draining || (hit && live && !success)));
`endif
`ifdef FORMAL
    // 段边界条件由当前段号决定。
    always @* begin
        assert(seg19_q==(segment==6'd19));
        assert(seg39_q==(segment==6'd39));
        assert(seg_thumb_q==(segment>=6'd40));
        assert(issue_nx==(filling_nx && !live_nx && !age_eq_nx && !geom_bad(current_nx)));
    end
    // 寄存请求及两行 RAM 写条件与本拍接受条件对应。
    always @* begin
        assert(seg_issue_q==(filling && !live && !hit && !invalid_geometry));
        assert(line0_write_q==(fl_q && !age_eq_q && !cb));
        assert(line1_write_q==(fl_q && !age_eq_q && cb));
    end
`endif
    wire [10:0] offset={segment,words[4:0]};wire [LINE_BITS-1:0] data0,data1;
    generate if (LINE_BITS!=32 && LINE_BITS!=24) begin : bad_line_bits invalid_line_bits u_error(); end endgenerate
    // offset仅由正沿segment/words寄存器产生；在半周前保持本拍写地址。
    // 两行共用此镜像，写使能/数据/事务拍数仍沿用原接口。复位清除写条件，
    // 首次有效写之前已有下降沿捕获；镜像无需复位，避免同沿rst释放的恢复短路径。
    (* keep = "true" *) reg [10:0] line_write_address_guard;
    always @(negedge mem_clk) line_write_address_guard<=offset;
    // 第二行 RAM 的地址 bit8 使用同沿独立副本，降低半周期地址路径扇出。
    // keep 防止副本被重新合并；地址捕获沿和写条件与其余位一致。
    (* keep = "true" *) reg line1_address_bit8_guard;
    always @(negedge mem_clk) line1_address_bit8_guard<=offset[8];
    fifo_dual_clock_ram #(.DATA_WIDTH(LINE_BITS),.ADDR_WIDTH(11),.USE_EG_BRAM(USE_EG_BRAM),.RAM_IMPLEMENT("9K")) u_line0(
        .wr_clk(mem_clk),.wr_enable(data_valid && line0_write_q),.wr_addr(line_write_address_guard),.wr_data(data_in[LINE_BITS-1:0]),
        .rd_clk(px_clk),.rd_enable(1'b1),.rd_addr(rd_x),.rd_data(data0));
    fifo_dual_clock_ram #(.DATA_WIDTH(LINE_BITS),.ADDR_WIDTH(11),.USE_EG_BRAM(USE_EG_BRAM),.RAM_IMPLEMENT("9K")) u_line1(
        .wr_clk(mem_clk),.wr_enable(data_valid && line1_write_q),.wr_addr({line_write_address_guard[10:9],line1_address_bit8_guard,line_write_address_guard[7:0]}),.wr_data(data_in[LINE_BITS-1:0]),
        .rd_clk(px_clk),.rd_enable(1'b1),.rd_addr(rd_x),.rd_data(data1));
    wire [LINE_BITS-1:0] line_q=rd_bank ? data1 : data0;
    generate if (LINE_BITS==32) begin : full assign rd_data=line_q; end
    else begin : rgb assign rd_data={8'd0,line_q}; end endgenerate
    always @(posedge mem_clk or posedge mrst) begin
        if(mrst) begin
            busy<=0;fetched<=0;tag0<=0;tag1<=0;filling<=0;draining<=0;live<=0;
            event_pending<=0;event_kind<=0;current<=0;event_payload<=0;segment<=0;words<=0;age<=0;remaining_budget<=0;
            // 复位值 = 对 current=0、segment=0、age=0 的函数值。
            geom_bad_q<=0;age_eq_q<=1;seg_addr_q<=0;
            fl_q<=0;fin_q<=0;   // = filling&&live、segment==43&&words==32 在复位值下的值
            seg43_q<=0;w32_q<=0;seg19_q<=0;seg39_q<=0;seg_thumb_q<=0;seg_issue_q<=0;line0_write_q<=0;line1_write_q<=0;
        end else begin
            seg19_q<=seg19_nx;seg39_q<=seg39_nx;seg_thumb_q<=seg_thumb_nx;
            seg_issue_q<=issue_nx;line0_write_q<=write0_nx;line1_write_q<=write1_nx;
            geom_bad_q<=geom_bad(current_nx);age_eq_q<=age_eq_nx;seg_addr_q<=seg_addr_d;
            fl_q<=fl_nx;fin_q<=fin_nx;seg43_q<=seg43_nx;w32_q<=w32_nx;
            if(filling) begin age<=age+1'b1;remaining_budget<=remaining_budget-16'd1;end
            if(req_accept) begin
                current<=request;filling<=1;age<=0;remaining_budget<=request[15:0];segment<=0;words<=0;live<=0;
                busy[rb]<=1;fetched[rb]<=0;
                if(rb) tag1<=request[EPOCH_W+134:16];else tag0<=request[EPOCH_W+134:16];

            end
            if(seg_valid && seg_ready) live<=1;
            if(write_fire) words<=words+1'b1;
            if(complete && filling) begin
                live<=0;
                if(!w32_q) begin
                    filling<=0;busy[cb]<=0;fetched[cb]<=0;
                    event_pending<=1;event_kind<=2;event_payload<=current;
                end else if(seg43_q) begin
                    filling<=0;fetched[cb]<=1;
                    event_pending<=1;event_kind<=0;event_payload<=current;
                end else begin segment<=segment+1'b1;words<=0;end
            end
            if(hit && !success) begin
                filling<=0;
                if(live && !seg_done) draining<=1;
                else begin
                    live<=0;busy[cb]<=0;fetched[cb]<=0;
                    event_pending<=1;event_kind<=1;event_payload<=current;
                end
            end
            if(draining && complete) begin
                draining<=0;live<=0;busy[cb]<=0;fetched[cb]<=0;
                event_pending<=1;event_kind<=1;event_payload<=current;
            end
            if(filling && invalid_geometry) begin
                filling<=0;busy[cb]<=0;fetched[cb]<=0;
                event_pending<=1;event_kind<=2;event_payload<=current;
            end
            if(rel_accept) begin busy[lb]<=0;fetched[lb]<=0;end
            if(event_pending && evt_src_ready) event_pending<=0;
        end
    end
endmodule
`default_nettype wire
