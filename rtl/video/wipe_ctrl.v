`timescale 1ns/1ps
`default_nettype none
// 18 帧水平擦拭控制，全部运行在 pix 域；只产生旧图槽号和分界，不处理像素。
// 第 k 帧分界 b(k)=ceil(960*k/18)，k=1..18；分界左侧显示新图，右侧显示旧图。
// ref_switch 与新 ref_slot 同拍出现，旧槽从上一拍引用 last_slot 取得。
// 首次显示没有旧图，直接输出新图；擦拭中再次换图时，以当前引用为旧图重新计 18 帧。
// ref_switch 拍组合输出 b(1)，供 bundle_scanout 同拍锁存；其余帧在 vblank_start 更新。
// next_b_q 提前保存下一帧分界，避免帧边界路径串联加法和查表。
// 行包始终读取旧、新两半各 640 字，擦拭结束后仅显示新图，读带宽不随擦拭变化。
// 行标签的 old_gen 由顶层接当前引用 gen；它仅参与标签匹配，不用于寻址。
// 这一复用依赖同一 epoch 内槽内容不变，由 wipe_old_guard 和装载槽所有权保证。
// 本模块依赖 slot_owner 在垂直消隐换引用，以及 scanout 在同一时序点取得分界。
module wipe_ctrl(
    input  wire        clk,
    input  wire        rst,            // pix 域同步复位（rst_pix_logic）
    input  wire        ref_valid,      // slot_owner 引用有效（首次成功提交后恒 1）
    input  wire        ref_switch,     // 换引用单拍脉冲，与 ref_slot/ref_gen 同沿更新
    input  wire [1:0]  ref_slot,       // 当前（换引用拍起为新）显示槽
    input  wire        vblank_start,   // 帧边界（与 bundle_scanout 同源）
    output wire [1:0]  old_slot,       // 送 bundle_scanout.old_slot（ref_switch 拍有效）
    output wire [10:0] wipe_x,         // 送 bundle_scanout.wipe_x：下一帧的分界（960=全新图）
    output wire        wiping          // 诊断：本帧仍在擦拭（k_q<18）
);
    // b(k)=ceil(160*k/3)，k=1..18：54,107,160,214,267,320,374,427,480,534,587,640,694,747,800,854,907,960。
    // 18 项常数表避免在 pix 域执行乘除。
    // 查表直接由当前 k 给出 b(min(k+1,18))，下一帧分界寄存器前无需再串一级加法。
    function [10:0] next_boundary_of;
        input [4:0] k;
        begin
            case(k)
                5'd1: next_boundary_of=11'd107;  5'd2: next_boundary_of=11'd160;  5'd3: next_boundary_of=11'd214;
                5'd4: next_boundary_of=11'd267;  5'd5: next_boundary_of=11'd320;  5'd6: next_boundary_of=11'd374;
                5'd7: next_boundary_of=11'd427;  5'd8: next_boundary_of=11'd480;  5'd9: next_boundary_of=11'd534;
                5'd10:next_boundary_of=11'd587;  5'd11:next_boundary_of=11'd640;  5'd12:next_boundary_of=11'd694;
                5'd13:next_boundary_of=11'd747;  5'd14:next_boundary_of=11'd800;  5'd15:next_boundary_of=11'd854;
                5'd16:next_boundary_of=11'd907;
                default:next_boundary_of=11'd960; // k=17 的下一帧 b(18)=960；k=18（空闲）保持 960
            endcase
        end
    endfunction
    localparam [10:0] B1=11'd54;   // b(1)：换引用当拍直接给出

    // 寄存状态（全部先声明后使用）：
    reg        have_prev;   // 上一拍 ref_valid：本次换引用之前是否已有显示图
    reg [1:0]  last_slot;   // 上一拍 ref_slot：ref_switch 拍它就是旧图
    reg [1:0]  os_q;        // 本 epoch 锁存的旧图（ref_switch 之后保持）
    reg [4:0]  k_q;         // 当前擦拭帧序号 1..18；18=空闲/全新
    reg [10:0] next_b_q;    // 下一帧分界 = next_boundary_of(k_q)，比 k_q 晚一拍更新

    // 本次换引用是否真的要擦拭：ref_switch && have_prev（之前已有图；首图上电不擦拭）。
    // ref_switch 拍的组合输出；其余时间输出寄存器。
    assign old_slot = ref_switch ? (have_prev ? last_slot : ref_slot) : os_q;
    assign wipe_x   = ref_switch ? (have_prev ? B1 : 11'd960) : next_b_q;
    assign wiping   = k_q != 5'd18;

    // next_b_q 比 k_q 晚一拍：k_q 只在 vblank_start 拍（自增）和其后一拍的 ref_switch（置 1/18）改变，
    // 下一次有人读 next_b_q 是一整帧之后的 vblank_start（ref_switch 拍读的是常数 B1/960），所以晚一拍不影响任何快照。
    // 前提：vblank_start 每帧一次；ref_switch 只出现在 vblank_start 的下一拍（slot_owner 的 commit_go 只在 vblank_start）。
    always @(posedge clk) begin
        if(rst) begin
            have_prev<=1'b0;last_slot<=2'd0;
            os_q<=2'd0;k_q<=5'd18;next_b_q<=11'd960;
        end else begin
            have_prev<=ref_valid;last_slot<=ref_slot;
            if(ref_switch) os_q<=have_prev ? last_slot : ref_slot;
            // 优先级：ref_switch 高于 vblank_start（二者实际不会同拍）。
            if(ref_switch) k_q<=have_prev ? 5'd1 : 5'd18;
            else if(vblank_start && k_q!=5'd18) k_q<=k_q+5'd1;
            next_b_q<=next_boundary_of(k_q);
        end
    end
`ifdef FORMAL
    // 不变式：k_q 只在 1..18；k_q 上一拍未变时 next_b_q 等于查表值（“晚一拍”的精确含义）。
    reg [4:0] f_k_prev;reg f_init=1'b0;
    always @(posedge clk) begin f_k_prev<=k_q;f_init<=1'b1; end
    always @* if(f_init && !rst) begin
        assert(k_q>=5'd1 && k_q<=5'd18);
        if(f_k_prev==k_q) assert(next_b_q==next_boundary_of(k_q));
    end
`endif
endmodule
`default_nettype wire
