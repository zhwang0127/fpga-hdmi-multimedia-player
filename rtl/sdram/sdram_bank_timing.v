`timescale 1ns/1ps
`default_nettype none

// SDRAM命令许可引擎+四bank open-row表：只判定"本拍是否允许发射"，不驱动引脚。
// 每个请求(req_valid/req_kind/req_bank/req_row)在全部时序与开闭行判据满足时
// 得到单拍grant；发射沿更新计数器与行表。判据来自EM638325 -7档保守值(方案
// §9.2)：tRCD/tRP/tRRD/tRFC/tRAS最小、读写突发对PRE的排空界；打开行超过
// T_RAS_MAX(80µs前关闭)报告强制预充目标。REF要求全部bank已关闭。
// 本模块不做读写数据通路、地址多路、仲裁优先级或auto-precharge。
// *_logic管组合使能；rst接异步复位端，停钟时也能立即隔离输出。
module sdram_bank_timing #(
    parameter T_RCD=3,          // ACT到READ/WRITE
    parameter T_RP=3,           // PRE到ACT
    parameter T_RAS=6,          // ACT到PRE(最小)
    parameter T_RRD=2,          // 任意相邻ACT
    parameter T_RFC=8,          // REF到下一REF/ACT
    parameter READ_TO_PRE=11,   // READ命令到PRE(CAS3+BL8保守)
    parameter WRITE_TO_PRE=10,  // WRITE命令到PRE(BL8+tWR保守)
    parameter T_RAS_MAX=10000,  // 80µs@125MHz，超时强制关行
    // PREDECODE=1 时五类许可判据由“下一拍状态 + 下一拍请求 bank/row”预先算好并寄存，
    // 许可提前寄存，grant 仅按请求类型选择并与 valid 合并。
    // req_bank/req_row 必须对应上一拍提供的 req_bank_next/req_row_next。
    // PREDECODE=0 直接计算组合许可；=2 分别计算保持与装载候选，在寄存器输入末级选择。
    parameter PREDECODE=0
)(
    input wire clk,arst,
    input wire req_valid,
    input wire [2:0] req_kind,      // 0=REF 1=ACT 2=READ 3=WRITE 4=PRE 5=PRE_ALL
    input wire [1:0] req_bank,
    input wire [10:0] req_row,
    input wire [1:0] req_bank_next,
    input wire [10:0] req_row_next,
    input wire req_ld,                 // PREDECODE=2：下一拍请求取装载候选
    input wire [1:0] req_bank_ld,
    input wire [10:0] req_row_ld,
    output wire req_grant,
    output wire [4:0] allow_use,       // {PRE_ALL,PRE,RW,ACT,REF} 当拍许可（PREDECODE>0 时为寄存值），供引脚侧复制发射判据
    output wire [4:0] allow_next,      // PREDECODE>0 时 = 许可寄存器的 D 端（下一拍 allow_use），供控制器预寄存 addr_go
    output wire [3:0] bank_open,
    output wire row_close_needed,
    output wire [1:0] row_close_bank,
    output wire [1:0] open_bank_oldest
);
    wire rst_logic;
    localparam K_REF=0,K_ACT=1,K_READ=2,K_WRITE=3,K_PRE=4,K_PRE_ALL=5;
    localparam [3:0] RCD_RELOAD=T_RCD-1,RP_RELOAD=T_RP-1,RAS_RELOAD=T_RAS-1,
                     RDP_RELOAD=READ_TO_PRE-1,WDP_RELOAD=WRITE_TO_PRE-1,
                     RRD_RELOAD=T_RRD-1,RFC_RELOAD=T_RFC-1;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // rst 由 reset_sync 以 arst 异步置位，!arst 冗余；去掉以免 rst_mem 释放链进入命令组合路径。
    wire enabled=!rst_logic;
    // 每bank一行表+五类倒数计数器(0=约束解除)；全局tRRD/tRFC倒数。
    reg [3:0] open_valid;
    reg [10:0] open_row0,open_row1,open_row2,open_row3;
    reg [15:0] rcd_cnt,rp_cnt,rdp_cnt,wdp_cnt,ras_cnt;   // 每4位一个bank
    reg [13:0] ras_age0,ras_age1,ras_age2,ras_age3;
    reg [3:0] rrd_cnt,rfc_cnt;

    function [3:0] pick4;input [15:0] v;input [1:0] b;begin
        case(b) 2'd0:pick4=v[3:0]; 2'd1:pick4=v[7:4];
                2'd2:pick4=v[11:8]; default:pick4=v[15:12];
        endcase
    end
    endfunction
    function [15:0] drop4;input [15:0] v;input [1:0] b;input [3:0] reload;begin
        case(b) 2'd0:drop4={v[15:4],reload}; 2'd1:drop4={v[15:8],reload,v[3:0]};
                2'd2:drop4={v[15:12],reload,v[7:0]};
                default:drop4={reload,v[11:0]};
        endcase
    end
    endfunction
    reg [10:0] sel_row;
    always @(*) case(req_bank)
        2'd0:sel_row=open_row0; 2'd1:sel_row=open_row1;
        2'd2:sel_row=open_row2; default:sel_row=open_row3;
    endcase
    wire sel_open=open_valid[req_bank];
    wire [3:0] sel_rcd=pick4(rcd_cnt,req_bank);
    wire [3:0] sel_rp=pick4(rp_cnt,req_bank);
    wire [3:0] sel_rdp=pick4(rdp_cnt,req_bank);
    wire [3:0] sel_wdp=pick4(wdp_cnt,req_bank);
    wire [3:0] sel_ras=pick4(ras_cnt,req_bank);
    reg [13:0] sel_age;
    always @(*) case(req_bank)
        2'd0:sel_age=ras_age0; 2'd1:sel_age=ras_age1;
        2'd2:sel_age=ras_age2; default:sel_age=ras_age3;
    endcase
    // PRE_ALL：至少一行打开，且所有打开bank的tRAS/读/写排空界都已解除。
    wire all_drained =
        (!open_valid[0] || (pick4(ras_cnt,2'd0)==0 && pick4(rdp_cnt,2'd0)==0 && pick4(wdp_cnt,2'd0)==0)) &&
        (!open_valid[1] || (pick4(ras_cnt,2'd1)==0 && pick4(rdp_cnt,2'd1)==0 && pick4(wdp_cnt,2'd1)==0)) &&
        (!open_valid[2] || (pick4(ras_cnt,2'd2)==0 && pick4(rdp_cnt,2'd2)==0 && pick4(wdp_cnt,2'd2)==0)) &&
        (!open_valid[3] || (pick4(ras_cnt,2'd3)==0 && pick4(rdp_cnt,2'd3)==0 && pick4(wdp_cnt,2'd3)==0));
    wire allow_ref    = rfc_cnt==0 && open_valid==4'b0000;
    wire allow_act    = rrd_cnt==0 && rfc_cnt==0 && !sel_open && sel_rp==0;
    wire allow_rw     = sel_open && sel_row==req_row && sel_rcd==0;
    wire allow_pre    = sel_open && sel_ras==0 && sel_rdp==0 && sel_wdp==0;
    wire allow_preall = (|open_valid) && all_drained;
    // PREDECODE 寄存版本（下方 always 维护）；PREDECODE=0 时不使用。
    reg allow_ref_r,allow_act_r,allow_rw_r,allow_pre_r,allow_preall_r;
    wire use_ref   =PREDECODE ? allow_ref_r    : allow_ref;
    wire use_act   =PREDECODE ? allow_act_r    : allow_act;
    wire use_rw    =PREDECODE ? allow_rw_r     : allow_rw;
    wire use_pre   =PREDECODE ? allow_pre_r    : allow_pre;
    wire use_preall=PREDECODE ? allow_preall_r : allow_preall;
    assign allow_use={use_preall,use_pre,use_rw,use_act,use_ref};
    reg allow;
    always @(*) case(req_kind)
        K_REF:allow=use_ref;
        K_ACT:allow=use_act;
        K_READ,K_WRITE:allow=use_rw;
        K_PRE:allow=use_pre;
        K_PRE_ALL:allow=use_preall;
        default:allow=1'b0;
    endcase
    assign req_grant=enabled && req_valid && allow;
    assign bank_open=open_valid;
    // tRAS超时：报告任一超龄bank（编号优先级固定，仲裁层可遍历）。
    wire age0_old=(ras_age0>=T_RAS_MAX[13:0])&&open_valid[0];
    wire age1_old=(ras_age1>=T_RAS_MAX[13:0])&&open_valid[1];
    wire age2_old=(ras_age2>=T_RAS_MAX[13:0])&&open_valid[2];
    wire age3_old=(ras_age3>=T_RAS_MAX[13:0])&&open_valid[3];
    assign row_close_needed=age0_old|age1_old|age2_old|age3_old;
    reg [1:0] close_sel;
    always @(*) begin
        if(age0_old) close_sel=2'd0;
        else if(age1_old) close_sel=2'd1;
        else if(age2_old) close_sel=2'd2;
        else if(age3_old) close_sel=2'd3;
        else close_sel=2'd0;
    end
    assign row_close_bank=close_sel;
    reg [1:0] oldest;
    always @(*) begin
        if(open_valid[0]) oldest=2'd0;
        else if(open_valid[1]) oldest=2'd1;
        else if(open_valid[2]) oldest=2'd2;
        else if(open_valid[3]) oldest=2'd3;
        else oldest=2'd0;
    end
    assign open_bank_oldest=oldest;

    // 六种命令分别取自身许可，去掉“kind选许可后再译码同一个kind”的重复串联。
    // req_grant 按 kind 选择许可，各 fire 使用对应请求类型的专用许可。
    // ===对固定三位常量的硬件仍是普通相等；kind 含 X/Z 时返回零，按无效请求拒绝。
    // 发射沿与计数器维护。
    wire fire=req_grant;
        wire fire_ref=enabled && req_valid && (req_kind===K_REF) && use_ref;
        wire fire_act=enabled && req_valid && (req_kind===K_ACT) && use_act;
        wire fire_read=enabled && req_valid && (req_kind===K_READ) && use_rw;
        wire fire_write=enabled && req_valid && (req_kind===K_WRITE) && use_rw;
        wire fire_pre=enabled && req_valid && (req_kind===K_PRE) && use_pre;
        wire fire_pre_all=enabled && req_valid && (req_kind===K_PRE_ALL) && use_preall;
    // 下一值函数：每bank独立递减到0；发射沿覆盖目标bank或全部bank。
    function [15:0] tick4;input [15:0] v;input [1:0] rb;input single;input allbanks;
        input [3:0] reload;reg [3:0] n0,n1,n2,n3;begin
        n0=(single&&rb==2'd0)||allbanks ? reload : (v[3:0]!=0 ? v[3:0]-4'd1 : 4'd0);
        n1=(single&&rb==2'd1)||allbanks ? reload : (v[7:4]!=0 ? v[7:4]-4'd1 : 4'd0);
        n2=(single&&rb==2'd2)||allbanks ? reload : (v[11:8]!=0 ? v[11:8]-4'd1 : 4'd0);
        n3=(single&&rb==2'd3)||allbanks ? reload : (v[15:12]!=0 ? v[15:12]-4'd1 : 4'd0);
        tick4={n3,n2,n1,n0};
    end
endfunction
    // ---- PREDECODE：下一拍判据。发射(fire_*)只在末级做二选一，比较/选择与之并行。----
    // 4 位倒数计数下一值为 0 ⇔ 本拍被重装且重装值为 0，或本拍值 <=1。
    function nz0;input [3:0] v;input load;input [3:0] reload;
        begin nz0=load ? (reload==4'd0) : (v<=4'd1); end
    endfunction
    // 按本拍是否发射命令分别计算两组许可次态，fire 仅在寄存器输入末级选择。
    // 两组输入使用相同的请求、bank 行状态和计数，只将 fire_* 固定为对应的零或一。
    // 这样将晚到的发射信号与行比较、计数选择路径分开。
    wire [9:0] sh_d;   // [5*f+:5] = g_sh[f] 的 {preall,pre,rw,act,ref} D 端候选
    // PREDECODE=2 的两份候选锥各自使用一份 bank 寄存器。
    // D 根据下一拍 bank 选择关系计算（req_ld ? req_bank_ld : req_bank），不增加接口延迟。
    // keep 防止综合把同值副本重新合并；只在该模式生成，普通组合判据模式不增加寄存器。
    generate if(PREDECODE==2) begin: g_bank_replicas
        (* keep = "true" *) reg [1:0] rb0,rb1;
        always @(posedge clk or posedge rst)
            if(rst) begin rb0<=0;rb1<=0;end
            else begin
                rb0<=req_ld ? req_bank_ld : req_bank;
                rb1<=req_ld ? req_bank_ld : req_bank;
            end
`ifdef FORMAL
        // 该寄存器与 req_bank 的装载选择同步更新。
        always @* if(!rst) assert(rb0==req_bank && rb1==req_bank);
`endif
    end endgenerate
    genvar gf;
    generate for(gf=0;gf<2;gf=gf+1) begin: g_sh
        wire [1:0] rb;
        if(PREDECODE==2) begin: copied_bank
            assign rb=(gf==0) ? g_bank_replicas.rb0 : g_bank_replicas.rb1;
        end else begin: direct_bank
            assign rb=req_bank;
        end
        // 选择器也用本份副本，否则共享的 sel_* 会把高扇出重新接回原 bank。
        reg [10:0] sel_row;
        always @(*) case(rb)
            2'd0:sel_row=open_row0;2'd1:sel_row=open_row1;
            2'd2:sel_row=open_row2;default:sel_row=open_row3;
        endcase
        wire [3:0] sel_rcd=pick4(rcd_cnt,rb),sel_rp=pick4(rp_cnt,rb);
        wire [3:0] sel_ras=pick4(ras_cnt,rb),sel_rdp=pick4(rdp_cnt,rb),sel_wdp=pick4(wdp_cnt,rb);
        wire fire=(gf==1);
        wire fire_ref=fire&&req_kind==K_REF;
        wire fire_act=fire&&req_kind==K_ACT;
        wire fire_read=fire&&req_kind==K_READ;
        wire fire_write=fire&&req_kind==K_WRITE;
        wire fire_pre=fire&&req_kind==K_PRE;
        wire fire_pre_all=fire&&req_kind==K_PRE_ALL;
        wire [1:0] nb=req_bank_next;
        wire on_nb=rb==nb;
        reg [10:0] row_nb;
        always @(*) case(nb)
            2'd0:row_nb=open_row0; 2'd1:row_nb=open_row1;
            2'd2:row_nb=open_row2; default:row_nb=open_row3;
        endcase
        wire [3:0] open_nx;   // 每 bank 下一拍 open_valid
        genvar gb;
        for(gb=0;gb<4;gb=gb+1) begin: g_open_nx
            assign open_nx[gb]=(fire_act&&rb==gb) ? 1'b1 :
                ((fire_pre&&rb==gb)||fire_pre_all||fire_ref) ? 1'b0 : open_valid[gb];
        end
        wire [3:0] drained_nx;
        for(gb=0;gb<4;gb=gb+1) begin: g_drained_nx
            assign drained_nx[gb]=!open_nx[gb] ||
                (nz0(pick4(ras_cnt,gb),fire_act&&rb==gb,RAS_RELOAD) &&
                 nz0(pick4(rdp_cnt,gb),fire_read&&rb==gb,RDP_RELOAD) &&
                 nz0(pick4(wdp_cnt,gb),fire_write&&rb==gb,WDP_RELOAD));
        end
        wire nb_open_nx=open_nx[nb];
        wire nb_row_hit_nx=(fire_act&&on_nb) ? (req_row==req_row_next) : (row_nb==req_row_next);
        wire nb_rcd0_nx=nz0(pick4(rcd_cnt,nb),fire_act&&on_nb,RCD_RELOAD);
        wire nb_ras0_nx=nz0(pick4(ras_cnt,nb),fire_act&&on_nb,RAS_RELOAD);
        wire nb_rdp0_nx=nz0(pick4(rdp_cnt,nb),fire_read&&on_nb,RDP_RELOAD);
        wire nb_wdp0_nx=nz0(pick4(wdp_cnt,nb),fire_write&&on_nb,WDP_RELOAD);
        wire nb_rp0_nx =nz0(pick4(rp_cnt,nb),(fire_pre&&on_nb)||fire_pre_all,RP_RELOAD);
        wire rrd0_nx=nz0(rrd_cnt,fire_act,RRD_RELOAD);
        wire rfc0_nx=nz0(rfc_cnt,fire_ref,RFC_RELOAD);
        // ---- PREDECODE=2：两候选并行，req_ld 末级选择 ----
        // 保持候选 = 本拍 rb/req_row（on_nb 恒真）：行命中 = ACT 发射 ? 1 : sel_row==req_row。
        wire h_open_nx=open_nx[rb];
        reg h_table_hit;
        // 各 bank 的行地址并行比较，再根据 bank 选择命中位。
        // bank 选择的 case 默认分支读取 bank3。
        always @(*) case(rb)
            2'd0:h_table_hit=open_row0==req_row;
            2'd1:h_table_hit=open_row1==req_row;
            2'd2:h_table_hit=open_row2==req_row;
            default:h_table_hit=open_row3==req_row;
        endcase
        wire h_row_hit_nx=fire_act ? 1'b1 : h_table_hit;
        wire h_rcd0_nx=nz0(sel_rcd,fire_act,RCD_RELOAD);
        wire h_ras0_nx=nz0(sel_ras,fire_act,RAS_RELOAD);
        wire h_rdp0_nx=nz0(sel_rdp,fire_read,RDP_RELOAD);
        wire h_wdp0_nx=nz0(sel_wdp,fire_write,WDP_RELOAD);
        wire h_rp0_nx =nz0(sel_rp,fire_pre||fire_pre_all,RP_RELOAD);
        // 装载候选 = req_bank_ld/req_row_ld：行表比较两路（ACT 本拍写入的行 / 现有行表）与发射判据并行，最后按 fire_act 选。
        // 装载 bank 在末级选择；分别计算各 bank 的许可，避免选择信号先经过
        // 多组计数/行表选择器，再串入行比较和许可逻辑。所有候选保留真实fire语义。
        wire [3:0] load_act,load_rw,load_pre;
        genvar gl;
        for(gl=0;gl<4;gl=gl+1) begin: g_load_bank
        wire [1:0] lb=gl;
        wire on_lb=rb==lb;
        reg [10:0] row_lb;
        always @(*) case(lb)
            2'd0:row_lb=open_row0; 2'd1:row_lb=open_row1;
            2'd2:row_lb=open_row2; default:row_lb=open_row3;
        endcase
        wire l_hit_act=req_row==req_row_ld;
        wire l_hit_tab=row_lb==req_row_ld;
        wire l_open_nx=open_nx[lb];
        wire l_row_hit_nx=(fire_act&&on_lb) ? l_hit_act : l_hit_tab;
        wire l_rcd0_nx=nz0(pick4(rcd_cnt,lb),fire_act&&on_lb,RCD_RELOAD);
        wire l_ras0_nx=nz0(pick4(ras_cnt,lb),fire_act&&on_lb,RAS_RELOAD);
        wire l_rdp0_nx=nz0(pick4(rdp_cnt,lb),fire_read&&on_lb,RDP_RELOAD);
        wire l_wdp0_nx=nz0(pick4(wdp_cnt,lb),fire_write&&on_lb,WDP_RELOAD);
        wire l_rp0_nx =nz0(pick4(rp_cnt,lb),(fire_pre&&on_lb)||fire_pre_all,RP_RELOAD);
        assign load_act[gl]=!l_open_nx && l_rp0_nx;
        assign load_rw[gl]=l_open_nx && l_row_hit_nx && l_rcd0_nx;
        assign load_pre[gl]=l_open_nx && l_ras0_nx && l_rdp0_nx && l_wdp0_nx;
        end
        wire h_act_ok=!h_open_nx && h_rp0_nx, l_act_ok=load_act[req_bank_ld];
        wire h_rw_ok=h_open_nx && h_row_hit_nx && h_rcd0_nx, l_rw_ok=load_rw[req_bank_ld];
        wire h_pre_ok=h_open_nx && h_ras0_nx && h_rdp0_nx && h_wdp0_nx, l_pre_ok=load_pre[req_bank_ld];
        // 两种模式的寄存器 D 端（PREDECODE=1 直接请求；PREDECODE=2 末级选择）。
        wire act_d=(PREDECODE==2) ? (rrd0_nx && rfc0_nx && (req_ld ? l_act_ok : h_act_ok)) :
                                     (rrd0_nx && rfc0_nx && !nb_open_nx && nb_rp0_nx);
        wire rw_d =(PREDECODE==2) ? (req_ld ? l_rw_ok : h_rw_ok) : (nb_open_nx && nb_row_hit_nx && nb_rcd0_nx);
        wire pre_d=(PREDECODE==2) ? (req_ld ? l_pre_ok : h_pre_ok) : (nb_open_nx && nb_ras0_nx && nb_rdp0_nx && nb_wdp0_nx);
        assign sh_d[5*gf+:5]={(|open_nx) && (&drained_nx),pre_d,rw_d,act_d,rfc0_nx && open_nx==4'b0000};
    end endgenerate
    wire [4:0] allow_d=fire ? sh_d[9:5] : sh_d[4:0];
    assign allow_next=allow_d;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            // 复位态：全部关行、计数为 0 → REF/ACT 许可为 1，其余为 0（与组合判据的复位值相同）。
            allow_ref_r<=1'b1;allow_act_r<=1'b1;allow_rw_r<=1'b0;allow_pre_r<=1'b0;allow_preall_r<=1'b0;
        end else begin
            {allow_preall_r,allow_pre_r,allow_rw_r,allow_act_r,allow_ref_r}<=allow_d;
        end
    end
`ifdef FORMAL
    // 寄存许可应在每拍与当前状态直接计算的许可一致。
    always @* if(PREDECODE) assert(allow_ref_r==allow_ref && allow_act_r==allow_act &&
        allow_rw_r==allow_rw && allow_pre_r==allow_pre && allow_preall_r==allow_preall);
    // 装载请求拍不同时发出 ACT/PRE-ALL，保证装载候选中的时序计数与实际命令对应。
    always @* if(PREDECODE==2 && req_ld) assert(!fire_act && !fire_pre_all);
`endif
`ifndef SYNTHESIS
    // 外层 SYNTHESIS 宏同时兼容会定义该宏的工具；这些注释指令属于综合控制，须保留。
    /* synthesis translate_off */
    // 本拍请求的 bank/row 须与上一拍提供的 *_next 一致。
    reg [1:0] chk_bank;reg [10:0] chk_row;reg chk_armed;
    always @(posedge clk or posedge rst)
        if(rst) begin chk_armed<=0;chk_bank<=0;chk_row<=0;end
        else begin chk_armed<=1;
            chk_bank<=(PREDECODE==2) ? (req_ld ? req_bank_ld : req_bank) : req_bank_next;
            chk_row <=(PREDECODE==2) ? (req_ld ? req_row_ld  : req_row ) : req_row_next;
        end
    always @(negedge clk) if(PREDECODE && !rst && chk_armed && (req_bank!==chk_bank || req_row!==chk_row))
        $display("ERROR sdram_bank_timing PREDECODE contract: req_bank/req_row != previous *_next at %0t",$time);
    /* synthesis translate_on */
`endif
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            open_valid<=0;open_row0<=0;open_row1<=0;open_row2<=0;open_row3<=0;
            rcd_cnt<=0;rp_cnt<=0;rdp_cnt<=0;wdp_cnt<=0;ras_cnt<=0;
            ras_age0<=0;ras_age1<=0;ras_age2<=0;ras_age3<=0;
            rrd_cnt<=0;rfc_cnt<=0;
        end else begin
            rcd_cnt<=tick4(rcd_cnt,req_bank,fire_act,1'b0,RCD_RELOAD);
            rdp_cnt<=tick4(rdp_cnt,req_bank,fire_read,1'b0,RDP_RELOAD);
            wdp_cnt<=tick4(wdp_cnt,req_bank,fire_write,1'b0,WDP_RELOAD);
            ras_cnt<=tick4(ras_cnt,req_bank,fire_act,1'b0,RAS_RELOAD);
            rp_cnt<=tick4(rp_cnt,req_bank,fire_pre,fire_pre_all,RP_RELOAD);
            rrd_cnt<=fire_act ? RRD_RELOAD : (rrd_cnt!=0 ? rrd_cnt-4'd1 : 4'd0);
            rfc_cnt<=fire_ref ? RFC_RELOAD : (rfc_cnt!=0 ? rfc_cnt-4'd1 : 4'd0);
            // 打开行年龄向上计到饱和；行关闭/REF/PRE_ALL时清零。
            if(fire_ref||fire_pre_all) begin
                ras_age0<=0;ras_age1<=0;ras_age2<=0;ras_age3<=0;
            end else begin
                if((open_valid[0]&&!fire_pre)||fire_act&&req_bank==2'd0) begin
                    if(fire_act&&req_bank==2'd0) ras_age0<=0;
                    else if(ras_age0!=14'h3FFF) ras_age0<=ras_age0+14'd1;
                end
                if((open_valid[1]&&!fire_pre)||fire_act&&req_bank==2'd1) begin
                    if(fire_act&&req_bank==2'd1) ras_age1<=0;
                    else if(ras_age1!=14'h3FFF) ras_age1<=ras_age1+14'd1;
                end
                if((open_valid[2]&&!fire_pre)||fire_act&&req_bank==2'd2) begin
                    if(fire_act&&req_bank==2'd2) ras_age2<=0;
                    else if(ras_age2!=14'h3FFF) ras_age2<=ras_age2+14'd1;
                end
                if((open_valid[3]&&!fire_pre)||fire_act&&req_bank==2'd3) begin
                    if(fire_act&&req_bank==2'd3) ras_age3<=0;
                    else if(ras_age3!=14'h3FFF) ras_age3<=ras_age3+14'd1;
                end
            end
            if(fire_act) begin
                open_valid[req_bank]<=1'b1;
                case(req_bank)
                    2'd0:open_row0<=req_row; 2'd1:open_row1<=req_row;
                    2'd2:open_row2<=req_row; default:open_row3<=req_row;
                endcase
            end
            if(fire_pre) open_valid[req_bank]<=1'b0;
            if(fire_pre_all||fire_ref) open_valid<=4'b0000;
        end
    end
endmodule
`default_nettype wire
