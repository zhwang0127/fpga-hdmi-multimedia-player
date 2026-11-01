`timescale 1ns/1ps
`default_nettype none
// pix 域逐列平滑，缓存 16 列×6 位柱高，接收 band 结果并准备下一显示页。
// 柱高使用双页缓存：一页累积本次计算结果，另一页供画面扫描读取。
// 以两份同写的 RAM 提供平滑计算和显示扫描所需的独立读口，每份仅使用 32 个地址及低 6 位。
// column_valid/index/height 必须同拍有效，并按 0..15 顺序每帧各到达一次。
// 六拍流水处理一列，允许连续列输入；new_valid 必须晚于最后一列至少六拍。
// new_valid 标记整帧计算完成，到下一次 vblank 才发布显示页，不在完成沿直接穿透。
// 整机每次 vblank 先发布再启动 FFT，前一组在下一组第零列到达前完成发布并腾出待写页。
// 仅复位有效位和等待计数器；bars_valid 之前不使用显示数据计算衰减。
module spectrum_bars_stream #(parameter USE_EG_BRAM=1)(
    input wire clk,rst,new_valid,vblank_start,column_valid,
    input wire [3:0] column_index,
    input wire [5:0] column_height,
    // 显示读口：view_addr 在某沿采样，下一拍 view_h 给出该沿“之前”显示页的柱高。
    // 叠加器提前一拍给列地址，读数来自已经发布的显示页。
    input wire [3:0] view_addr,
    output wire [5:0] view_h,
    output reg bars_valid
);
    // disp_page：当前显示页；pending 页 = !disp_page。复位清0只为给出确定地址，
    // 复位后 bars_valid=0，显示与衰减均不读取未初始化的页数据。
    reg disp_page;
    wire [17:0] view_q,decay_q;
    reg [5:0] previous_q,target_first,target_q,decayed_q,target_qq,target_pick,decay_pick,result_q;
    reg [3:0] tag_first,tag_q,tag_qq,tag_pick,tag_result;
    reg valid_first,valid_q,valid_qq,valid_pick,valid_result;
    reg first_first,first_q,first_qq,first_pick;
    reg high_gt,high_eq,low_ge;
    wire [5:0] decay_bits;
    reg pending_valid;
    genvar b;
    // 3位比较仅用布尔逻辑，不让工具把“小比较+数据选择”串成进位链。
    function gt3;
        input [2:0] a,b;
        begin gt3=(a[2]&&!b[2]) || ((a[2]~^b[2])&&a[1]&&!b[1]) ||
            ((a[2]~^b[2])&&(a[1]~^b[1])&&a[0]&&!b[0]);end
    endfunction
    // 第六拍结果写到待发布页；两份 RAM 同沿同址同数据，复位后整页由新结果覆盖。
    // 衰减读在列有效拍给出当前显示页地址，下一拍读出，再寄存为 previous_q。
    spectrum_work_ram #(.USE_EG_BRAM(USE_EG_BRAM)) u_decay_ram(
        .clk(clk),.wr_enable(valid_result),.wr_addr({4'd0,!disp_page,tag_result}),
        .wr_data({12'd0,result_q}),.rd_addr({4'd0,disp_page,column_index}),.rd_data(decay_q));
    spectrum_work_ram #(.USE_EG_BRAM(USE_EG_BRAM)) u_view_ram(
        .clk(clk),.wr_enable(valid_result),.wr_addr({4'd0,!disp_page,tag_result}),
        .wr_data({12'd0,result_q}),.rd_addr({4'd0,disp_page,view_addr}),.rd_data(view_q));
    assign view_h=view_q[5:0];
    generate for(b=0;b<6;b=b+1) begin : decay_logic
        if(b==0) assign decay_bits[b]=!previous_q[0] && (|previous_q[5:1]);
        else assign decay_bits[b]=(previous_q[b] ^ !(|previous_q[b-1:0])) && (|previous_q);
    end endgenerate
    always @(posedge clk) begin
        // 流数据直接提供target，省掉一整组96位16:1读选择。
        target_first<=column_height;tag_first<=column_index;first_first<=!bars_valid;
        previous_q<=decay_q[5:0];
        target_q<=target_first;tag_q<=tag_first;first_q<=first_first;
        decayed_q<=decay_bits;
        target_qq<=target_q; tag_qq<=tag_q; first_qq<=first_q;
        high_gt<=gt3(target_qq[5:3],decayed_q[5:3]);high_eq<=(target_qq[5:3]==decayed_q[5:3]);
        low_ge<=!gt3(decayed_q[2:0],target_qq[2:0]);
        target_pick<=target_qq;decay_pick<=decayed_q;tag_pick<=tag_qq;first_pick<=first_qq;
        result_q<=(first_pick || high_gt || (high_eq && low_ge))?target_pick:decay_pick;
        tag_result<=tag_pick;
        if(rst) begin
            valid_first<=0;valid_q<=0;valid_qq<=0;valid_pick<=0;valid_result<=0;
            pending_valid<=0; bars_valid<=0; disp_page<=0;
        end else begin
            valid_first<=column_valid;valid_q<=valid_first;valid_qq<=valid_q;valid_pick<=valid_qq;valid_result<=valid_pick;
            // 发布整帧结果时切换显示页，下一帧计算写入另一页。
            if(vblank_start && pending_valid) begin pending_valid<=0; bars_valid<=1; disp_page<=!disp_page; end
            if(new_valid) pending_valid<=1;
        end
    end
endmodule
`default_nettype wire
