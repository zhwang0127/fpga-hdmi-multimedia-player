`timescale 1ns/1ps
`default_nettype none
// 频谱计算模块：在 pix 域采样，执行 FFT 并生成柱高。
// 柱高通过一拍同步读口 view_addr→view_h 提供给叠加器，读取该沿之前已发布的显示页。
// vblank_start 为单拍脉冲，本地寄存一拍后发布上一帧结果并启动计算。
// 所有输入均须在 pix 域；rst 同步清控制状态，bars_valid 前显示数据无效。
module spectrum_top #(
    parameter USE_EG_BRAM=1, USE_EG_DSP=1, FLOOR=82
)(
    input wire clk,rst,in_valid,vblank_start,
    input wire signed [15:0] in_l,in_r,
    input wire [3:0] view_addr,
    output wire [5:0] view_h,
    output wire bars_valid,busy,sat_flag,
    output wire bin_valid,
    output wire [7:0] bin_idx,
    output wire [8:0] bin_lg,
    output wire signed [17:0] debug_re,debug_im,
    output wire done
);
    wire wr_en,new_valid;
    wire [8:0] wr_addr,wptr,ring_rd_addr;
    wire [9:0] count;
    wire [15:0] wr_data,ring_wr_data,ring_rd_data;
    wire [95:0] new_h;
    wire column_valid;
    wire [3:0] column_index;
    wire [5:0] column_height;
    reg vb_q,start_pending;
    wire core_start;
    // 若本拍恰好写样本，把计算请求保留到写后的空拍再读wptr/count。
    // 48kHz的物理写必然隔开，所以最多晚一拍，窗口仍包含同拍提交的那一个样本。
    // 这样不用在采样指针上再串两条“提交后指针/计数”加法链。忙时vb仍忽略。
    assign core_start=(vb_q || start_pending) && !wr_en && !busy;
    always @(posedge clk) begin if(rst) vb_q<=0; else vb_q<=vblank_start; end
    always @(posedge clk) begin
        if(rst) start_pending<=0;
        else if(core_start || busy) start_pending<=0;
        else if(vb_q) start_pending<=1;
    end
    spectrum_tap u_tap(.clk(clk),.rst(rst),.in_valid(in_valid),.in_l(in_l),.in_r(in_r),
        .wr_en(wr_en),.wr_addr(wr_addr),.wr_data(wr_data),.wptr(wptr),.sample_count(count));
    // 原生 RAM 写数据的 bit0/11 插入带 keep 的恒等 BUF，为短路径提供保持时间延迟。
    // BUF 不增加时钟周期，也不改变写使能和数据值；其余位及可移植 RAM 分支直接连接。
    genvar hold_bit;
    generate for(hold_bit=0;hold_bit<16;hold_bit=hold_bit+1) begin : ring_hold
        if(USE_EG_BRAM && (hold_bit==0 || hold_bit==11)) begin : native_buffer
            (* keep="true" *) EG_LOGIC_BUF u_hold(.i(wr_data[hold_bit]),.o(ring_wr_data[hold_bit]));
        end else begin : direct_data
            assign ring_wr_data[hold_bit]=wr_data[hold_bit];
        end
    end endgenerate
    fifo_dual_clock_ram #(.DATA_WIDTH(16),.ADDR_WIDTH(9),.USE_EG_BRAM(USE_EG_BRAM),
        .RAM_IMPLEMENT("9K")) u_ring(.wr_clk(clk),.wr_enable(wr_en),.wr_addr(wr_addr),
        .wr_data(ring_wr_data),.rd_clk(clk),.rd_enable(1'b1),.rd_addr(ring_rd_addr),.rd_data(ring_rd_data));
    spectrum_fft_core #(.USE_EG_BRAM(USE_EG_BRAM),.USE_EG_DSP(USE_EG_DSP)) u_core(
        .clk(clk),.rst(rst),.start(core_start),.wptr(wptr),.sample_count(count),
        .ring_rd_addr(ring_rd_addr),.ring_rd_data(ring_rd_data),.busy(busy),
        .bin_valid(bin_valid),.bin_idx(bin_idx),.bin_lg(bin_lg),.debug_re(debug_re),.debug_im(debug_im),
        .done(done),.sat_flag(sat_flag));
    spectrum_bands #(.FLOOR(FLOOR)) u_bands(.clk(clk),.rst(rst),.start(core_start),
        .bin_valid(bin_valid),.bin_idx(bin_idx),.bin_lg(bin_lg),.done(done),.new_h(new_h),.new_valid(new_valid),
        .column_valid(column_valid),.column_index(column_index),.column_height(column_height));
    spectrum_bars_stream #(.USE_EG_BRAM(USE_EG_BRAM)) u_bars(.clk(clk),.rst(rst),.new_valid(new_valid),
        .column_valid(column_valid),.column_index(column_index),.column_height(column_height),
        .vblank_start(vb_q),.view_addr(view_addr),.view_h(view_h),.bars_valid(bars_valid));
endmodule
`default_nettype wire
