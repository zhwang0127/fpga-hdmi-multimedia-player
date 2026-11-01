`timescale 1ns/1ps
`default_nettype none

// SDRAM控制器集成顶层：初始化→刷新调度→五级仲裁→命令许可→BL8突发。
// 地址映射(方案§9.1)：21bit word地址 col=addr[7:0] bank=addr[9:8] row=addr[20:10]，
// 每tile(32word)恰好一个bank-row页。读/写段请求(首word地址+BL8条数)经
// 仲裁（突发段在途时锁定）与许可（bank_timing 判据）后由内置突发引擎发射。
// 刷新需求接仲裁最高级；REF命令由控制模块直接发（突发引擎只处理
// 数据方向）。命令引脚多路：初始化期间init_seq驱动，之后控制模块驱动。
// 不含行缓冲所有权、SDRAM写端提交协议或HDMI侧读仲裁。
`ifndef SDRAM_PHY_RD_EXTRA_DEFAULT
`define SDRAM_PHY_RD_EXTRA_DEFAULT 0
`endif
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sdram_stream_controller #(
    parameter POWER_WAIT_CYCLES=25000,REFRESH_COUNT=8,
    parameter T_RCD=3,T_RP=3,T_RAS=6,T_RRD=2,T_RFC=8,
    parameter READ_TO_PRE=11,WRITE_TO_PRE=10,T_RAS_MAX=10000,
    parameter REFRESH_INTERVAL=937,REFRESH_GRACE=937,
    parameter CAS_LATENCY=3,T_WR=2,
    // 控制器发 READ 到 rd_data_in 出现该字之间、CL 之外的额外拍数。
    // 0=组合 PHY；寄存 PHY（sdram_phy REGISTERED=1）为 2（命令寄存 1 + 读采样 1）。
    // 写方向不需要参数：PHY 把命令和写数据一起延后一拍，器件侧仍是“首字同 WRITE 沿”。
    parameter PHY_RD_EXTRA=`SDRAM_PHY_RD_EXTRA_DEFAULT
)(
    input wire clk,arst,
    // 读写段请求(经外部仲裁后的单一在途段；方向+首word地址+BL8条数)。
    input wire seg_valid,output wire seg_ready,
    input wire seg_is_write,
    input wire [20:0] seg_addr,          // word地址：col[7:0] bank[9:8] row[20:10]
    input wire [3:0] seg_bl8_count,
    input wire [31:0] seg_budget_cycles,
    // 数据流(直通burst_engine)。
    input wire wr_data_valid,input wire [31:0] wr_data,
    output wire wr_beat,
    output wire rd_data_valid,input wire rd_data_ready,input wire [31:0] rd_data_in,
    // 段完成/取消。
    output wire seg_done,output wire seg_done_was_write,
    input wire seg_cancel,
    // SDRAM物理引脚(接EG_PHY_SDRAM_2M_32)。
    output wire sd_cs_n,sd_ras_n,sd_cas_n,sd_we_n,sd_cke,
    output wire [10:0] sd_addr,output wire [1:0] sd_ba
);
    wire rst_logic;
    wire pipe_rst_logic;
    wire init_done;
    wire ref_due,ref_overdue,ref_service;
    // 初始化序列器。
    wire i_cs_n,i_ras_n,i_cas_n,i_we_n,i_cke;
    wire [10:0] i_addr;wire [1:0] i_ba;wire i_oe;
    sdram_init_seq #(.POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),.REFRESH_COUNT(REFRESH_COUNT),
        .T_RP_CYCLES(T_RP),.T_RFC_CYCLES(T_RFC),.T_MRD_CYCLES(T_WR)) u_init(
        .clk(clk),.arst(arst),.init_done(init_done),.init_busy(),
        .cmd_oe(i_oe),.cmd_cs_n(i_cs_n),.cmd_ras_n(i_ras_n),.cmd_cas_n(i_cas_n),
        .cmd_we_n(i_we_n),.cmd_cke(i_cke),.cmd_addr(i_addr),.cmd_ba(i_ba)
    );
    // 刷新调度。
    sdram_refresh_sched #(.INTERVAL_CYCLES(REFRESH_INTERVAL),.GRACE_CYCLES(REFRESH_GRACE)) u_ref(
        .clk(clk),.arst(arst),.init_done(init_done),
        .refresh_due(ref_due),.refresh_overdue(ref_overdue),.refresh_service(ref_service)
    );
    // 命令许可引擎。
    wire [2:0] bt_kind;wire [1:0] bt_bank;wire [10:0] bt_row;
    wire bt_grant,bt_req_valid;wire [3:0] bank_open;wire row_close_needed;wire [1:0] row_close_bank,open_oldest;
    // seg_addr_r 的 D 端（下一拍 bank/row），供 bank_timing 预先寄存许可判据（PREDECODE=1）。
    wire [20:0] seg_addr_nx;
    // 装载选择单独给 bank_timing（PREDECODE=2），在许可寄存器 D 端最后一级才选候选。
    wire seg_load;
    wire [4:0] bt_allow;
    sdram_bank_timing #(.T_RCD(T_RCD),.T_RP(T_RP),.T_RAS(T_RAS),.T_RRD(T_RRD),
        .T_RFC(T_RFC),.READ_TO_PRE(READ_TO_PRE),.WRITE_TO_PRE(WRITE_TO_PRE),
        .T_RAS_MAX(T_RAS_MAX),.PREDECODE(2)) u_bt(
        .clk(clk),.arst(arst),
        .req_valid(bt_req_valid),.req_kind(bt_kind),.req_bank(bt_bank),.req_row(bt_row),
        .req_bank_next(seg_addr_nx[9:8]),.req_row_next(seg_addr_nx[20:10]),
        .req_ld(seg_load),.req_bank_ld(seg_addr[9:8]),.req_row_ld(seg_addr[20:10]),
        .req_grant(bt_grant),.allow_use(bt_allow),.allow_next(),.bank_open(bank_open),
        .row_close_needed(row_close_needed),.row_close_bank(row_close_bank),
        .open_bank_oldest(open_oldest)
    );
    // 五级仲裁：截止刷新>硬截止行>PCM缓存样本不足>普通行>后台。
    // 本顶层把"段请求"视为普通行(级3)；行/PCM/后台由上层调度器再分层接入。
    localparam [15:0] NO_DL=16'hFFFF;
    wire arb_valid;wire [2:0] arb_owner;
    wire [15:0] ref_deadline_view=ref_due ? 16'd0 : NO_DL;
    localparam S_IDLE=0,S_ACT=1,S_SEG=2,S_REF=3,S_PRE=4,S_DONE=5;
    reg [2:0] state;
    wire rst;
    wire enabled=!rst_logic;
    wire burst_busy=enabled && state==S_SEG;
    reg seg_valid_hold;
    wire seg_pending=enabled && state==S_IDLE && seg_valid_hold;
    reg [3:0] seg_cnt;
    localparam integer RD_LAT=CAS_LATENCY+PHY_RD_EXTRA;
    wire [15:0] seg_len_est={12'd0,seg_cnt}*(RD_LAT+9)+16'd2;
    sdram_arbiter #(.NO_DEADLINE(NO_DL)) u_arb(
        .clk(clk),.arst(arst),
        .req_refresh(ref_due),.req_line_hard(1'b0),.req_pcm(1'b0),
        .req_line_normal(seg_pending && !burst_busy),.req_background(1'b0),
        .len_refresh(16'd8),.len_line_hard(16'd0),.len_pcm(16'd0),
        .len_line_normal(seg_len_est),.len_background(16'd0),
        .refresh_deadline(ref_deadline_view),.reserve_deadline(NO_DL),
        .burst_active(burst_busy),.grant_valid(arb_valid),.grant_owner(arb_owner)
    );
    // 刷新到期视图：从refresh_sched观察剩余拍数(简化为due时0、否则满窗)。

    // 段请求缓存(控制模块)：仲裁获准后拆地址发ACT+交给burst_engine。

    reg seg_rw;reg [20:0] seg_addr_r;reg [31:0] seg_budget;
    reg [31:0] seg_elapsed;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 业务门控使用 reset_sync 的本域复位，异步 arst 仅进入同步器置位端。
    wire [7:0] seg_col=seg_addr_r[7:0];
    wire [1:0] seg_bank=seg_addr_r[9:8];
    wire [10:0] seg_row=seg_addr_r[20:10];
    // 与下方控制模块 S_IDLE 装载条件逐字相同（seg_ready 已含 !ref_due）。
    assign seg_load=state==S_IDLE && !ref_due && seg_valid && seg_ready;
    assign seg_addr_nx=seg_load ? seg_addr : seg_addr_r;
    // burst_engine段在途。
    wire be_done,be_done_w;wire be_cmd_valid,be_cmd_w;wire [1:0] be_bank;
    wire [7:0] be_col;

    // 段长度估计(供仲裁窗口检查)：每BL8约1+8拍+CAS余量。

    // 命令发射到bank_timing：ACT(ref控制模块)/ACT(段)/burst的READ/WRITE/PRE。
    wire ref_cmd_fire=bt_req_valid && bt_kind==3'd0 && bt_grant;
    wire act_cmd_fire=bt_req_valid && bt_kind==3'd1 && bt_grant;
    wire be_cmd_fire=be_cmd_valid && bt_kind==3'd2 && bt_grant;
    // bank/row 在段请求接受沿锁存，直到预充电和完成响应之前保持稳定。
    // 数据引擎直接使用同一地址，不另存一份 bank/row。
    wire [1:0] bt_bank_r=seg_bank;
    wire [10:0] bt_row_r=seg_row;
    reg [2:0] bt_kind_r;
    always @(*) case(state)
        S_SEG:bt_kind_r=3'd2;
        S_ACT:bt_kind_r=3'd1;
        S_PRE:bt_kind_r=3'd4;
        default:bt_kind_r=3'd0;
    endcase
    assign bt_req_valid=be_cmd_valid || state==S_REF || state==S_ACT || state==S_PRE;
    // burst_engine的READ/WRITE编码在kind=2(K_READ)——写方向如何区分？
    // bank_timing对READ/WRITE判据相同(行命中+tRCD)，此处kind=2代表读写通用。
    // 引脚方向由burst_engine的cmd_is_write直接驱动。
    assign bt_kind=bt_kind_r;assign bt_bank=bt_bank_r;assign bt_row=bt_row_r;
    // (bt_req_valid 的 assign 在下方always后)
    // REF服务脉冲：REF命令发射沿反馈给refresh_sched。
    reg ref_service_r;
    always @(posedge clk or posedge rst)
        if(rst) ref_service_r<=0;
        else ref_service_r<=ref_cmd_fire;
    assign ref_service=ref_service_r;
    // burst_engine实例。
    // 内置突发引擎处理连续读写请求。
    // 读消费者连续ready；CAS支持1..3，BL8连续命令间距8拍，不跨段。
    reg be_started,act_fired;
    wire pipe_seg_valid=act_fired && !be_started;
    wire pipe_seg_ready;
    localparam pipe_IDLE=0,pipe_CMD=1,pipe_WDATA=2,pipe_RWAIT=3,pipe_RDATA=4,pipe_DRAIN=5,pipe_DONE=6;
    localparam pipe_OK=0,pipe_CANCELED=1,pipe_DEADLINE=2;
    reg [2:0] pipe_state;
    reg [1:0] pipe_status;
    reg pipe_is_write;
    reg [3:0] pipe_bl8_left;
    reg [7:0] pipe_col_next;          // 下一条BL8命令的列地址(A10=0手动预充)
    reg [2:0] pipe_beat_cnt;         // 0..7写/读数据拍
    // 读延迟等待计数到 RD_LAT-1；RD_LAT<=4 时使用 2 位计数，否则使用 3 位。
    localparam integer CLW=(RD_LAT>4) ? 3 : 2;
    reg [CLW-1:0] pipe_cl_cnt;
    reg [3:0] pipe_twr_cnt;          // 末写数据后的tWR
    // 剩余预算从N倒数：当前值为 1 的周期禁止发出新命令。
    // 不保存两份32bit计数，也不在命令组合链现场做budget-1减法。
    reg [31:0] pipe_remaining;
    reg pipe_cancel_seen;
    reg pipe_write_issued; // 这段是否真的发过WRITE；没发过就不需要假装排空8字。
    reg pipe_deadline_seen; // 锁存到期状态，直到本段结束前都禁止发出新命令。
    reg pipe_next_read_issued;
    // pipe_remaining==1 预先寄存（下一值==1），去掉 32 位比较进 be_cmd_valid/引脚的组合链。
    reg pipe_rem1;
    generate if(RD_LAT<1 || RD_LAT>7) begin: bad_rd_lat invalid_read_latency u_error(); end endgenerate
    // 下一条 READ 在本 BL8 第 (7-RD_LAT) 个数据拍发出：命令间距仍恰为 8 拍（器件侧连续 BL8）。
    localparam [2:0] pipe_PIPE_BEAT=7-RD_LAT;
    // 读尾提前PRE（仅 PHY_RD_EXTRA>0 启用）：本段最后一条 READ 已发出（bl8_left==0）后，
    // 控制模块不等数据排空即请求 PRE；PRE 何时真正发出仍由 bank_timing 的 READ_TO_PRE（READ 起 CL+BL8 保守）把关，
    // 命令与数据在 PHY 中统一延后，器件侧间隔不变。seg_done 仍等数据引擎回到 IDLE（全部数据字已被下游接受）。
    wire pipe_rst;
    reset_sync pipe_u_reset(.clk(clk),.arst(arst),.rst(pipe_rst),.logic_rst(pipe_rst_logic));
    wire pipe_enabled=!pipe_rst_logic; // 同上，pipe_rst 已含 arst。
    // 寄存 PHY 启用时，将预充电许可检查与尾部数据排空并行，缩短每个读段的收尾时间。
    localparam EARLY_RD_PRE=(PHY_RD_EXTRA>0);
    wire be_rd_tail=pipe_enabled && !pipe_is_write && pipe_bl8_left==0 &&
        (pipe_state==pipe_RWAIT || pipe_state==pipe_RDATA || pipe_state==pipe_DONE);
    wire pipe_pipe_window=!pipe_is_write && pipe_state==pipe_RDATA && pipe_beat_cnt==pipe_PIPE_BEAT;
    wire pipe_active=pipe_state!=pipe_IDLE && pipe_state!=pipe_DONE;
    wire pipe_deadline_now=pipe_active && pipe_rem1;
    // 下一拍 pipe_remaining：IDLE 装载 seg_budget（此时 pipe_active=0），有效期内减 1，否则保持。
    wire pipe_load=pipe_state==pipe_IDLE && pipe_seg_valid && pipe_seg_ready;
    wire pipe_rem1_nx=pipe_load ? seg_budget==32'd1 : pipe_active ? pipe_remaining==32'd2 : pipe_remaining==32'd1;
`ifdef FORMAL
    always @* assert(pipe_rem1==(pipe_remaining==32'd1));
`endif
    // 取消立即停止新BL8命令；已发射的突发按节拍排空(写满8拍/读满CL+8拍)。
    wire pipe_stop_new_cmd=pipe_cancel_seen || pipe_deadline_seen || pipe_deadline_now || seg_cancel;
    assign pipe_seg_ready=pipe_enabled && pipe_state==pipe_IDLE;
    // 命令准入的本地状态、取消、截止和剩余 BL8 条件提前寄存为 be_ok_q。
    // 本拍再与外部 seg_cancel 及写数据就绪合并，避免完整状态判断串入命令输出路径。
    wire be_ok_now=pipe_enabled && (pipe_state==pipe_CMD || pipe_pipe_window) && !pipe_cancel_seen && !pipe_deadline_seen &&
        !pipe_deadline_now && pipe_bl8_left!=0;
    reg be_ok_q;
    // 写拍侧使用独立的同值许可寄存器，将输出使能负载与管线次态逻辑分开。
    // keep 保留该物理副本，以减少晚到发射条件的扇出。
    (* keep = "true" *) reg wr_cmd_q,wr_data_q;
    assign be_cmd_valid=be_ok_q && !seg_cancel && (!pipe_is_write || wr_data_valid);
    assign be_cmd_w=pipe_is_write;
    assign be_bank=seg_bank;assign be_col=pipe_col_next;
    // 首字必须跟WRITE命令同沿。只有命令真正获准，上游才弹出这一个字。
    // READ的提前发射窗不会碰写数据；写源必须提前备齐连续8字。
    // 写许可按控制模块当前状态选择；单拍取消后，管线和控制模块可能暂时处于不同阶段。
    // wr_cmd_q有效且外层wr_data_valid为1时，请求有效性由 be_cmd_valid 保证；
    // 同构reset_sync和wr_cmd_q异步清零也保证bank启用，输出条件直接使用本地活动状态。
    wire wr_bank_allow=state==S_SEG ? bt_allow[2] : state==S_ACT ? bt_allow[1] :
        state==S_PRE ? bt_allow[3] : bt_allow[0];
    (* keep = "true" *) wire wr_cmd_gate=
        wr_cmd_q && !seg_cancel && wr_bank_allow;
    assign wr_beat=wr_data_valid && (wr_data_q || wr_cmd_gate);
    // rd_data_valid 使用读流水状态的寄存条件，直接驱动行缓存的数据接受条件。
    reg rdv_q;
    assign rd_data_valid=rdv_q;
    assign be_done=pipe_enabled && pipe_state==pipe_DONE;
    assign be_done_w=pipe_is_write;

    // 分别计算本拍发射命令和未发射命令的管线次态，pf_now 在末级选择。
    // 两份次态使用当前寄存器值，分别推导状态、计数、命令许可和读数据有效条件。
    wire pf_now=be_cmd_valid && bt_grant;
    localparam integer PNW=3+2+1+4+8+3+CLW+4+32+5;
    wire [2*PNW-1:0] pn_vec;wire [1:0] pn_ok,pn_ok_nocancel,pn_rdv;
    genvar gp;
    // gp=0/1 计算取消输入对应的下一状态；gp=2/3 仅导出取消为0的许可候选。
    // 无用的其他次态在综合中删除，取消输入只在 be_ok_d 的最后一级参与。
    generate for(gp=0;gp<4;gp=gp+1) begin: g_pn
        wire pf=(gp%2==1);
        wire cancel_now=(gp<2) ? seg_cancel : 1'b0;
        wire stop_new_cmd=pipe_cancel_seen || pipe_deadline_seen || pipe_deadline_now || cancel_now;
        wire wrb=pipe_enabled && wr_data_valid && (pipe_state==pipe_WDATA || (pipe_state==pipe_CMD && pipe_is_write && pf));
        reg [2:0] n_state;reg [1:0] n_status;reg n_is_write;reg [3:0] n_bl8_left;reg [7:0] n_col_next;reg [2:0] n_beat_cnt;
        reg [CLW-1:0] n_cl_cnt;reg [3:0] n_twr_cnt;reg [31:0] n_remaining;
        reg n_cancel_seen,n_write_issued,n_deadline_seen,n_next_read_issued,n_rem1;
        always @(*) begin
            n_state=pipe_state;n_status=pipe_status;n_is_write=pipe_is_write;n_bl8_left=pipe_bl8_left;n_col_next=pipe_col_next;n_beat_cnt=pipe_beat_cnt;n_cl_cnt=pipe_cl_cnt;n_twr_cnt=pipe_twr_cnt;n_remaining=pipe_remaining;n_cancel_seen=pipe_cancel_seen;n_write_issued=pipe_write_issued;n_deadline_seen=pipe_deadline_seen;n_next_read_issued=pipe_next_read_issued;n_rem1=pipe_rem1;
                n_rem1=pipe_rem1_nx;
                // 四个字节并行递减，借位只看本沿旧的低位是否全0；不串32bit减法。
                // 到期后停止新命令并完成已接受的突发；deadline_seen 保持到期状态，
                // 防止计数回绕后重新发送命令。
                if(pipe_active) begin
                    n_remaining[7:0]=pipe_remaining[7:0]-8'd1;
                    n_remaining[15:8]=pipe_remaining[15:8]-(pipe_remaining[7:0]==0);
                    n_remaining[23:16]=pipe_remaining[23:16]-(pipe_remaining[15:0]==0);
                    n_remaining[31:24]=pipe_remaining[31:24]-(pipe_remaining[23:0]==0);
                end
                if(cancel_now && pipe_active) begin n_cancel_seen=1;n_status=pipe_CANCELED;end
                if(pipe_deadline_now) begin n_deadline_seen=1;n_status=pipe_DEADLINE;end
                case(pipe_state)
                    pipe_IDLE:if(pipe_seg_valid && pipe_seg_ready) begin
                        n_is_write=seg_rw;
                        n_col_next=seg_col;
                        n_bl8_left=seg_cnt;n_remaining=seg_budget;
                        n_status=pipe_OK;n_cancel_seen=0;n_next_read_issued=0;n_write_issued=0;n_deadline_seen=0;
                        n_beat_cnt=0;n_cl_cnt=0;n_twr_cnt=0;
                        if(seg_budget==0 || seg_cnt==0) begin
                            n_status=pipe_DEADLINE;n_state=pipe_DONE;end
                        else n_state=pipe_CMD;
                    end
                    pipe_CMD:begin
                        if((pipe_deadline_seen || pipe_deadline_now) && pipe_bl8_left!=0) begin n_status=pipe_DEADLINE;if(pipe_is_write && pipe_write_issued) begin n_twr_cnt=0;n_state=pipe_DRAIN;end else n_state=pipe_DONE;end
                        else if(stop_new_cmd && pipe_bl8_left!=0) begin if(pipe_status!=pipe_DEADLINE) n_status=pipe_CANCELED;if(pipe_is_write && pipe_write_issued) begin n_twr_cnt=0;n_state=pipe_DRAIN;end else n_state=pipe_DONE;end
                        // 命令发射沿由外层(bank_timing grant)与本模块cmd_valid共同成立；
                        // 本模块在cmd_valid拍后推进。无grant时停留等待。
                        else if(pf) begin
                            if(pipe_is_write) n_write_issued=1;
                            n_bl8_left=pipe_bl8_left-4'd1;
                            n_col_next=pipe_col_next+8'd8;
                            // WRITE沿已经送完第0字；接下来WDATA只送第1到第7字。
                            n_beat_cnt=pipe_is_write ? 3'd1 : 3'd0;n_cl_cnt=0;
                            n_state=pipe_is_write ? pipe_WDATA : pipe_RWAIT;
                        end
                    end
                    // 写：这里送命令之后的7字；上游必须先备齐8字再启动，不能中途暂停——每拍写入完整 32 位数据，
                    // PCM尾部补零由供给端完成，本引擎只数满8拍。
                    pipe_WDATA:begin
                        if(wrb) begin
                            if(pipe_beat_cnt==3'd7) begin
                                // 第8写拍：还有BL8且无停止条件则回CMD发下一条；
                                // 段完成或已取消/超期则进tWR排空后收尾。
                                if(pipe_bl8_left!=0 && !stop_new_cmd)
                                    n_state=pipe_CMD;
                                else begin n_twr_cnt=0;n_state=pipe_DRAIN;end
                            end else n_beat_cnt=pipe_beat_cnt+3'd1;
                        end else if(pipe_deadline_now) begin
                            n_status=pipe_DEADLINE;n_state=pipe_DRAIN;   // 不截断：DRAIN等满剩余拍
                        end
                    end
                    pipe_RWAIT:begin
                        // 物理读不能暂停；截止/取消只停新命令，已接受CAS+8拍保持返回。
                        if(pipe_cl_cnt==RD_LAT-1) begin n_cl_cnt=0;n_beat_cnt=0;n_state=pipe_RDATA;end
                        else n_cl_cnt=pipe_cl_cnt+1'b1;
                    end
                    pipe_RDATA:begin
                        // 当前BL8第(7-CL)数据拍发下一READ，命令间距恰8拍。
                        // 若本沿未获许可，当前BL8后回CMD重新等CAS，禁止猜测发射。
                        if(pipe_pipe_window && pf) begin
                            n_next_read_issued=1'b1;n_bl8_left=pipe_bl8_left-4'd1;n_col_next=pipe_col_next+8'd8;
                        end
                        // 消费端必须连续ready：这是不可背压的真实SDR采样流。
                        if(pipe_beat_cnt==3'd7) begin
                            if(pipe_next_read_issued) begin n_beat_cnt=0;n_next_read_issued=0;end
                            else if(pipe_bl8_left!=0 && !stop_new_cmd) n_state=pipe_CMD;
                            else n_state=pipe_DONE;
                        end else n_beat_cnt=pipe_beat_cnt+1'b1;
                    end
                    // 排空：写方向等满8数据拍+tWR；读方向等CL+8拍；随后完成。
                    pipe_DRAIN:begin
                        if(pipe_is_write) begin
                            if(pipe_beat_cnt!=3'd7) n_beat_cnt=pipe_beat_cnt+3'd1;
                            else if(pipe_twr_cnt!=T_WR-1) n_twr_cnt=pipe_twr_cnt+3'd1;
                            else n_state=pipe_DONE;
                        end else begin
                            if(pipe_cl_cnt!=RD_LAT-1) n_cl_cnt=pipe_cl_cnt+1'b1;
                            else if(pipe_beat_cnt!=3'd7) n_beat_cnt=pipe_beat_cnt+3'd1;
                            else n_state=pipe_DONE;
                        end
                    end
                    pipe_DONE:if(be_done) n_state=pipe_IDLE;
                    default:n_state=pipe_IDLE;
                endcase
        end
        wire ok_candidate=(n_state==pipe_CMD || (!n_is_write && n_state==pipe_RDATA && n_beat_cnt==pipe_PIPE_BEAT)) &&
            !n_cancel_seen && !n_deadline_seen && !(n_state!=pipe_IDLE && n_state!=pipe_DONE && n_rem1) && n_bl8_left!=0;
        if(gp<2) begin: real_next
        assign pn_vec[PNW*gp+:PNW]={n_state,n_status,n_is_write,n_bl8_left,n_col_next,n_beat_cnt,n_cl_cnt,n_twr_cnt,n_remaining,
            n_cancel_seen,n_write_issued,n_deadline_seen,n_next_read_issued,n_rem1};
        // be_ok_now / rd_data_valid 在下一拍的值（pipe_enabled 下一拍为 1：pipe_rst 只会异步拉高，届时两寄存器也被清 0）。
        assign pn_ok[gp]=ok_candidate;
        assign pn_rdv[gp]=n_state==pipe_RDATA;
        end else begin: cancel_free_permission
            assign pn_ok_nocancel[gp-2]=ok_candidate;
        end
    end endgenerate
    // active 时出现取消，下一拍 cancel_seen 必为1，许可必为0；IDLE装载和DONE退出
    // 不受 seg_cancel 影响。这里是精确的末级分解，不能把掩码写成单独 !seg_cancel。
    // 直接列出下一拍能够发命令的四种来源，
    // 直接按各状态转移来源计算下一拍许可。IDLE装载必须独立处理，因为它会清旧取消/截止标志；
    // 活动状态继续检查剩余预算、已锁存的停止状态和当前取消信号。
    wire ok_load=pipe_load && seg_budget!=0 && seg_budget!=1 && seg_cnt!=0;
    wire ok_continue=!pipe_cancel_seen && !pipe_deadline_seen && !pipe_deadline_now &&
        pipe_remaining!=2 && !seg_cancel;
    wire [2:0] read_beat_nx=pipe_beat_cnt+3'd1;
    wire read_left_nx=(pipe_pipe_window && pf_now) ? pipe_bl8_left!=1 : pipe_bl8_left!=0;
    wire read_window_nx=!pipe_is_write &&
        ((pipe_beat_cnt!=7 && read_beat_nx==pipe_PIPE_BEAT) ||
         (pipe_beat_cnt==7 && pipe_next_read_issued && pipe_PIPE_BEAT==0));
    wire read_return_cmd=pipe_beat_cnt==7 && !pipe_next_read_issued && pipe_bl8_left!=0;
    wire ok_active=(pipe_state==pipe_CMD && !pf_now && pipe_bl8_left!=0) ||
        (pipe_state==pipe_WDATA && pipe_enabled && wr_data_valid && pipe_beat_cnt==7 && pipe_bl8_left!=0) ||
        (pipe_state==pipe_RWAIT && !pipe_is_write && pipe_cl_cnt==RD_LAT-1 && pipe_PIPE_BEAT==0 && pipe_bl8_left!=0) ||
        (pipe_state==pipe_RDATA && read_left_nx && (read_window_nx || read_return_cmd));
    wire be_ok_d=ok_load || (ok_active && ok_continue);
    // wr_cmd_q为下一拍CMD+写方向+许可；be_ok为1且方向为写时，许可不可能来自读流水窗。
    wire wr_cmd_d=be_ok_d && (pipe_state==pipe_IDLE ? seg_rw : pipe_is_write);
    wire wr_data_d=(pipe_state==pipe_CMD && pf_now && pipe_is_write &&
                   !(pipe_stop_new_cmd && pipe_bl8_left!=0)) ||
        (pipe_state==pipe_WDATA && (pipe_enabled && wr_data_valid ? pipe_beat_cnt!=7 : !pipe_deadline_now));
`ifdef FORMAL
    // 组合许可条件包含当前状态与取消信号。
    always @* assert(be_ok_d==(pf_now ? pn_ok[1] : pn_ok[0]));
    always @* assert(wr_cmd_q==(pipe_enabled && pipe_state==pipe_CMD && pipe_is_write && be_ok_q));
    always @* assert(wr_data_q==(pipe_enabled && pipe_state==pipe_WDATA));
`endif
    // packed次态的col字段不复位，拆成正沿CE；其他字段继续由下降沿SR异步置位。
    localparam integer PIPE_COL_LSB=3+CLW+4+32+5;
    wire [PNW-1:0] pipe_next_vec=pf_now ? pn_vec[2*PNW-1:PNW] : pn_vec[PNW-1:0];
    always @(posedge clk)
        if(!pipe_rst_logic) pipe_col_next<=pipe_next_vec[PIPE_COL_LSB+:8];
    always @(posedge clk or posedge pipe_rst) begin
        if(pipe_rst) begin
            pipe_state<=pipe_IDLE;pipe_status<=pipe_OK;pipe_is_write<=0;pipe_bl8_left<=0;
            pipe_beat_cnt<=0;pipe_cl_cnt<=0;pipe_twr_cnt<=0;pipe_remaining<=0;pipe_cancel_seen<=0;pipe_next_read_issued<=0;pipe_write_issued<=0;pipe_deadline_seen<=0;
            pipe_rem1<=0;be_ok_q<=0;wr_cmd_q<=0;wr_data_q<=0;rdv_q<=0;
        end else begin
            {pipe_state,pipe_status,pipe_is_write,pipe_bl8_left,pipe_beat_cnt,pipe_cl_cnt,pipe_twr_cnt,pipe_remaining,
             pipe_cancel_seen,pipe_write_issued,pipe_deadline_seen,pipe_next_read_issued,pipe_rem1}<={pipe_next_vec[PNW-1:PIPE_COL_LSB+8],pipe_next_vec[PIPE_COL_LSB-1:0]};
            be_ok_q<=be_ok_d;
            wr_cmd_q<=wr_cmd_d;wr_data_q<=wr_data_d;
            rdv_q<=pf_now ? pn_rdv[1] : pn_rdv[0];
        end
    end
`ifdef FORMAL
    always @* assert(be_ok_q==be_ok_now && rdv_q==(pipe_enabled && pipe_state==pipe_RDATA));
`endif
    always @(posedge clk or posedge rst)
        if(rst) act_fired<=0;
        else if(act_cmd_fire) act_fired<=1;
        else if(be_started||state==S_DONE||state==S_IDLE) act_fired<=0;
    always @(posedge clk or posedge rst)
        if(rst) be_started<=0;
        else if(state==S_IDLE && seg_valid && seg_ready) be_started<=0;
        else if(act_fired && !be_started) be_started<=1;
        else if(be_done) be_started<=0;
    // 段接受/完成。
    assign seg_ready=enabled && state==S_IDLE && init_done && !ref_due;
    // seg_done 在控制模块 S_DONE 且所需读流水排空后发布。
    assign seg_done_was_write=be_done_w;
    // 物理引脚多路：init驱动或控制模块驱动。
    wire pre_cmd_fire=bt_req_valid && bt_kind==3'd4 && bt_grant;
    // seg_done 使用寄存输出，仅在控制模块完成且所需读数据已排空时有效。
    // 提前预充电模式还需等待读流水空闲，完成发布后才允许释放段所有权。
    wire seg_done_state_nx=
        (state==S_IDLE && seg_valid && seg_ready && (seg_budget_cycles==0||seg_bl8_count==0)) ||
        (state==S_ACT && !act_cmd_fire && seg_cancel) ||
        (state==S_SEG && !(be_done || (EARLY_RD_PRE && be_rd_tail)) && seg_cancel && !be_started && !act_fired) ||
        (state==S_PRE && pre_cmd_fire) ||
        (state==S_DONE && !seg_done);
    wire seg_done_pidle_nx=
        (pipe_state==pipe_IDLE && !(pipe_seg_valid && pipe_seg_ready)) ||
        (pipe_state==pipe_DONE && be_done) || pipe_state==3'd7;
    reg seg_done_q;
    always @(posedge clk or posedge rst)
        if(rst) seg_done_q<=0;
        else seg_done_q<=seg_done_state_nx && (!EARLY_RD_PRE || seg_done_pidle_nx);
    assign seg_done=seg_done_q;
`ifdef FORMAL
    always @* assert(seg_done_q==(enabled && state==S_DONE && (!EARLY_RD_PRE || pipe_state==pipe_IDLE)));
`endif
    // 命令引脚{RAS#,CAS#,WE#}：REF001、ACT011、READ101、WRITE100、PRE010。
    wire m_cs_n=!(ref_cmd_fire||act_cmd_fire||be_cmd_fire||pre_cmd_fire);
    wire m_ras_n=!(ref_cmd_fire||act_cmd_fire||pre_cmd_fire);
    wire m_cas_n=!(ref_cmd_fire||be_cmd_fire);
    // WE#只由PRE或真实WRITE拉低；按控制模块态直接选对应许可，
    // 请求类型与 grant 分开计算，减少重复译码。保持所有停止/数据就绪项。
    wire pre_we=state==S_PRE && enabled && bt_allow[3];
    wire write_we=state==S_SEG && enabled && bt_allow[2] && be_ok_q &&
        !seg_cancel && pipe_is_write && wr_data_valid;
    wire m_we_n=!(pre_we || write_we);
    // 初始化未完成时控制模块保持IDLE，正常命令均为高电平；初始化退出后自身
    // 命令也保持全高。因此低有效命令可以直接相与，不必再串初始化所有权mux。
    // 初始化结束后才允许业务命令，两者均保持空闲时输出 NOP。
    assign sd_cs_n=i_cs_n && m_cs_n;
    assign sd_ras_n=i_ras_n && m_ras_n;
    assign sd_cas_n=i_cas_n && m_cas_n;
    assign sd_we_n=i_we_n && m_we_n;
    assign sd_cke=i_oe ? i_cke : 1'b1;
    // 地址候选只由寄存状态选择：ACT 使用行地址，SEG 使用列地址，其余为零。
    // 末级 addr_go 根据本地许可决定是否发射；bank 直接来自已锁存的 seg_bank。
    // 初始化和业务地址按各自有效条件合成，保持地址与命令的器件采样沿一致。
    wire [10:0] addr_cand=(state==S_ACT) ? seg_row : (state==S_SEG) ? {3'b000,be_col} : 11'd0;
    // ag_act_q/ag_seg_q 是由控制模块状态和许可寄存器形成的组合别名。
    // 分开计算 ACT 与 SEG 的发射条件，避免把多路深次态逻辑汇入同一寄存器。
    wire ag_act_q=enabled && state==S_ACT && bt_allow[1];
    wire ag_seg_q=enabled && state==S_SEG && be_ok_q && bt_allow[2];
    (* keep = "true" *) wire addr_go =
        ag_act_q || (ag_seg_q && !seg_cancel && (!pipe_is_write || wr_data_valid));
    // 两阶段未使用的地址均为0；初始化期间seg_bank也为0，故地址相或/存储体编号直通。
    // 不增加流水拍，不改物理寄存器、引脚或命令/数据对应沿。
    assign sd_addr=i_addr | (addr_go ? addr_cand : 11'd0);
    assign sd_ba=seg_bank;
`ifdef FORMAL
    always @* assert(ag_act_q==(enabled && state==S_ACT && bt_allow[1]) &&
                     ag_seg_q==(enabled && state==S_SEG && be_ok_q && bt_allow[2]));
    // 初始化和业务两路仅在各自驱动地址时贡献非零值。
    always @* assert(sd_addr==(i_oe ? i_addr : act_cmd_fire ? seg_row : be_cmd_fire ? {2'b00,be_col} : 11'd0) &&
                     sd_ba==(i_oe ? i_ba : (act_cmd_fire ? seg_bank : be_bank)));
    // 初始化期间 i_ba 与 seg_bank 均为零。
    always @* if(i_oe) assert(i_ba==2'd0 && seg_bank==2'd0);
`endif
    // 控制模块状态机。
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=S_IDLE;seg_rw<=0;seg_addr_r<=0;seg_cnt<=0;seg_budget<=0;
            seg_elapsed<=0;seg_valid_hold<=0;
        end else begin
            case(state)
                S_IDLE:begin
                    seg_valid_hold<=0;seg_elapsed<=0;
                    if(ref_due) state<=S_REF;
                    else if(seg_valid && seg_ready) begin
                        seg_rw<=seg_is_write;seg_addr_r<=seg_addr;
                        seg_cnt<=seg_bl8_count;seg_budget<=seg_budget_cycles;
                        seg_valid_hold<=1;
                        if(seg_budget_cycles==0||seg_bl8_count==0) state<=S_DONE;
                        else state<=S_ACT;
                    end
                end
                S_ACT:if(act_cmd_fire) state<=S_SEG;
                      else if(seg_cancel) state<=S_DONE;
                S_SEG:begin
                    if(be_done || (EARLY_RD_PRE && be_rd_tail)) begin
                        // 段后关行：行悬挂会让后续REF(要求全关)永不服务。
                        state<=S_PRE;
                    end else if(seg_cancel && !be_started) begin
                        // 取消且行已打开：仍须PRE收尾，不允许悬挂行。
                        state<=act_fired ? S_PRE : S_DONE;
                    end
                end
                S_PRE:if(pre_cmd_fire) state<=S_DONE;
                S_REF:if(ref_cmd_fire) begin
                    state<=seg_valid_hold ? S_ACT : S_IDLE;
                end
                S_DONE:if(seg_done) state<=S_IDLE;
                default:state<=S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
