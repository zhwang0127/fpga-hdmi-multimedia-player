`timescale 1ns/1ps
`default_nettype none
// 512 点 DIT FFT，运行在 pix 域，顺序执行蝶形运算，避免工作 RAM 的读写重叠。
// start 仅在空闲时接受；wptr/sample_count 为启动时的采样环快照。
// 采样环同步读延迟一拍，输入采样间隔至少 1546 拍，保证窗口拷贝前数据不被覆盖。
// 不足 512 个样本时对缺失部分补零；每级除以二，乘积加 2^16 后算术右移 17 位。
// 18 位运算保留保护位，饱和发生时锁存 sat_flag；系数量化格式为 Q1.17。
// 数据 RAM、乘法器和数据寄存器不复位，控制状态、地址、计数及有效位复位。
// bin_valid 同拍给出频格编号、对数功率及 debug_re/im 复数结果。
module spectrum_fft_core #(
    parameter USE_EG_BRAM=1, USE_EG_DSP=1
)(
    input wire clk,rst,start,
    input wire [8:0] wptr,
    input wire [9:0] sample_count,
    output reg [8:0] ring_rd_addr,
    input wire signed [15:0] ring_rd_data,
    output wire busy,
    output reg bin_valid,
    output reg [7:0] bin_idx,
    output reg [8:0] bin_lg,
    output reg signed [17:0] debug_re,debug_im,
    output reg done,sat_flag
);
    localparam IDLE=0,C_ADDR=1,C_WAIT=2,C_GET=3,C_WIN=4,C_MUL=5,C_MWAIT=6,
        C_MGET=7,C_ROUND=8,C_WRITE=9,C_NEXT=10,
        F_ADDR=11,F_READ_A=12,F_GET_A=13,F_READ_B=14,F_GET_B=15,
        F_M0=16,F_M1=17,F_M2=18,F_M3=19,F_M4=20,F_M5=21,F_M6=22,F_M7=23,
        F_PREP=24,F_ALU=25,F_SCALE=26,F_STORE=27,F_WRITE_A=28,F_WRITE_B=29,
        F_NEXT=30,P_ADDR=31,P_WAIT=32,P_GET=33,P_M0=34,P_M1=35,P_M2=36,P_M3=37,
        P_GROUP=38,P_BYTE=39,P_NIBBLE=40,P_LOG=41,P_OUT=42,P_NEXT=43,FINISH=44,
        F_CARRY=45,F_WAIT_A=46,C_PACK=47,C_HANN_ADJ=48;
    // 状态使用 49 位 one-hot 编码；复位进入 IDLE，正常运行时仅一位有效。
    reg [48:0] state;
    reg [8:0] base_ptr,n,addr_a,addr_b,ram_rd_addr;
    reg [8:0] base_next,n_next;
    reg copy_zero;
    reg [7:0] butterfly,scan_bin;
    reg [8:0] rom_addr;
    wire [8:0] next_a,next_b;
    reg [7:0] twiddle_phase;
    reg [7:0] butterfly_next,scan_next,phase_next;
    reg [8:0] twiddle_step;
    // 工作 RAM 和系数宽度为 18 位，部分中间数据使用 16 位。
    // 周期 Hann 窗与实数 16 位 PCM 输入下，首级半径不超过 16385，计入量化和舍入后小于 16400。
    // 该位宽依赖输入及窗函数的幅度范围，修改算法时须重新确定中间结果范围。
    reg signed [15:0] ar,ai,br,bi,tr,ti;
    reg signed [17:0] wr,wi,minus_wi;
    reg [17:0] not_wi;
    reg signed [18:0] hann_operand,hann_adjusted;
    reg hann_carry;
    reg signed [16:0] add_left,add_right,add_sum;
    reg signed [16:0] adjusted_right;
    reg add_carry;
    reg [1:0] operation;
    reg [1:0] operation_next;
    reg signed [16:0] scaled_q;
    reg signed [33:0] first_product;
    reg overflow_q,overflow_valid;
    reg dsp_bi,dsp_wi,dsp_minus_wi,dsp_square;
    reg signed [36:0] accumulator;
    reg signed [19:0] rounded;
    reg [18:0] log_group;
    reg [10:0] log_byte;
    reg [6:0] log_nibble;
    reg [4:0] log_base;
    wire signed [17:0] rom_q;
    // 同一个18位读口：C_GET/F_GET_A取cos，F_GET_B取sin；不能同时读两值。
    wire signed [17:0] cos_q=rom_q,sin_q=rom_q;
    wire signed [17:0] ram_re,ram_im;
    reg signed [17:0] mul_a,mul_b;
    wire signed [35:0] mul_p;
    wire ram_wr_re_en,ram_wr_im_en;
    wire [8:0] ram_wr_addr,bitrev_n;
    wire [17:0] ram_wr_re,ram_wr_im;
    wire signed [18:0] window_difference;
    wire rounded_overflow;
    wire signed [15:0] quantized,scaled_data;
    wire scaled_overflow;
    wire signed [33:0] mul_value;

    assign busy=((!state[IDLE]));
    assign bitrev_n={n[0],n[1],n[2],n[3],n[4],n[5],n[6],n[7],n[8]};
    assign window_difference=19'sd131072+hann_adjusted;
    assign rounded_overflow=(rounded[19:15]!={5{rounded[15]}});
    assign quantized=rounded_overflow?(rounded[19]?-16'sd32768:16'sd32767):rounded[15:0];
    assign scaled_overflow=(scaled_q[16]!=scaled_q[15]);
    assign scaled_data=scaled_overflow?(scaled_q[16]?-16'sd32768:16'sd32767):scaled_q[15:0];
    // 数据操作数由16位符号扩展，系数18位；完整乘积精确放进34位，不丢有效位。
    assign mul_value=mul_p[33:0];
    // 工作 RAM 在结果形成当拍写入：C_PACK 将 quantized 写入 bitrev_n，虚部写零。
    // F_STORE 按 operation 选择实/虚 RAM 和 A/B 地址，下一次读取发生在 F_NEXT 之后。
    // C_WRITE、F_WRITE_A、F_WRITE_B 保留为空拍，使计算调度与 busy/done/bin_valid 的输出时刻固定。
    assign ram_wr_re_en=(state[C_PACK] || (state[F_STORE] && !operation[1])) && !rst;
    assign ram_wr_im_en=(state[C_PACK] || (state[F_STORE] && operation[1])) && !rst;
    assign ram_wr_addr=state[C_PACK]?bitrev_n:(operation[0]?addr_b:addr_a);
    assign ram_wr_re=state[C_PACK]?{{2{quantized[15]}},quantized}:{{2{scaled_data[15]}},scaled_data};
    assign ram_wr_im=state[C_PACK]?18'd0:{{2{scaled_data[15]}},scaled_data};

    spectrum_twiddle_rom #(.USE_EG_BRAM(USE_EG_BRAM)) u_twiddle(
        .clk(clk),.addr(rom_addr),.q(rom_q));
    spectrum_mul18 #(.USE_EG_DSP(USE_EG_DSP)) u_mul(.clk(clk),.a(mul_a),.b(mul_b),.p(mul_p));
    // 工作 RAM 使用同钟 512×18 位接口，实部与虚部分别存储。
    spectrum_work_ram #(.USE_EG_BRAM(USE_EG_BRAM)) u_real(
        .clk(clk),.wr_enable(ram_wr_re_en),.wr_addr(ram_wr_addr),.wr_data(ram_wr_re),
        .rd_addr(ram_rd_addr),.rd_data(ram_re));
    spectrum_work_ram #(.USE_EG_BRAM(USE_EG_BRAM)) u_imag(
        .clk(clk),.wr_enable(ram_wr_im_en),.wr_addr(ram_wr_addr),.wr_data(ram_wr_im),
        .rd_addr(ram_rd_addr),.rd_data(ram_im));

    // ADDRESS_BLOCK_BEGIN：step是256右移形成的单热位，倒序就是half。
    // next_a的每一位只需选插入位左右的两个j位，省掉stage二进制译码和stage计数器。
    // 可用范围仅为step单热（FFT九级），power/idle时step=0而地址无效，不读它。
    genvar address_bit;
    generate for(address_bit=0;address_bit<9;address_bit=address_bit+1) begin : fft_address
        if(address_bit==0) begin : low_bit
            assign next_a[address_bit]=butterfly[0] && (|twiddle_step[7:0]);
        end else if(address_bit==8) begin : high_bit
            assign next_a[address_bit]=butterfly[7] && (|twiddle_step[8:1]);
        end else begin : middle_bit
            assign next_a[address_bit]=(butterfly[address_bit] && (|twiddle_step[7-address_bit:0])) |
                (butterfly[address_bit-1] && (|twiddle_step[8:9-address_bit]));
        end
        assign next_b[address_bit]=next_a[address_bit] | twiddle_step[8-address_bit];
    end endgenerate
    // ADDRESS_BLOCK_END
    // 只在调度的有效拍使用乘积。空拍保留确定的br/wr乘法，而不是把18位总线强制清零。
    // 加窗复用 br 和 wr 寄存器，copy 和蝶形阶段共用乘法器输入。
    always @* begin
        mul_a=br; mul_b=wr;
        if(dsp_bi) mul_a=bi;
        if(dsp_wi) mul_b=wi;
        else if(dsp_minus_wi) mul_b=minus_wi;
        else if(dsp_square) mul_b=mul_a;
    end

    // DSP_FLAGS_BEGIN：提前一拍生成乘法器输入选择，避免state译码串在数据选择前。
    always @(posedge clk) begin
        if(rst) begin dsp_bi<=0;dsp_wi<=0;dsp_minus_wi<=0;dsp_square<=0; end
        else begin
            dsp_bi<=state[F_M0] || state[F_M2] || state[P_M0];
            dsp_wi<=state[F_M0];dsp_minus_wi<=state[F_M1];
            dsp_square<=state[P_GET] || state[P_M0];
        end
    end
    // DSP_FLAGS_END
    // COUNTER_NEXT_BEGIN：循环计数每次更新至少相隔两拍，这些加法提前算好。
    // 真正的写入拍只在“初值/next寄存值”间选，不把进位链和该选择器串一起。
    // 不能把此技巧搬到允许每拍连续更新的tap指针，那里会读到过期next。
    always @(posedge clk) begin
        base_next<=base_ptr+1'b1;n_next<=n+1'b1;
        butterfly_next<=butterfly+1'b1;scan_next<=scan_bin+1'b1;
        phase_next<=twiddle_phase+twiddle_step[7:0];operation_next<=operation+1'b1;
    end
    // COUNTER_NEXT_END

    always @(posedge clk) begin
        // 固定的两个算术级不停钟、不带数据选择器：37 位求和，再做 20 位舍入。
        // 调度器只在正确拍取值；加窗前把 first_product 写零，同样走这条链。
        accumulator<=$signed({{3{mul_value[33]}},mul_value})+$signed({{3{first_product[33]}},first_product});
        rounded<=$signed(accumulator[36:17])+accumulator[16];
        // 一补先寄存，下一拍只加1，避免取负的输入反相再串进位。
        // 反相和加一分处两拍，缩短旋转因子取负路径上的组合进位链。
        not_wi<=~wi;
        overflow_q<=rounded_overflow;
        // 所有数据动作都由本地状态选择，复位不扇出到宽数据通路的 SR。
        (* full_case, parallel_case *) case(1'b1)
        state[C_GET]:begin
            br<=copy_zero?16'sd0:ring_rd_data;
            // 先选加/减的操作数，再独立进位，不把取负后的MUX串到下一条进位链。
            hann_operand<={cos_q[17],cos_q}^{19{!n[8]}};hann_carry<=!n[8];
        end
        state[C_HANN_ADJ]:hann_adjusted<=hann_operand+hann_carry;
        state[C_WIN]:begin wr<=window_difference[18:1]; first_product<=0; end
        state[F_GET_A]:begin ar<=ram_re[15:0]; ai<=ram_im[15:0]; wr<=cos_q; end
        state[F_GET_B]:begin br<=ram_re[15:0]; bi<=ram_im[15:0]; wi<=sin_q; end
        // wi在F_GET_B取得，下一F_M0备好一补，F_M1再加1，F_M2用负sin。
        state[F_M1]:minus_wi<=not_wi+1'b1;
        state[F_M2]:first_product<=mul_value;
        state[F_M4]:first_product<=mul_value;
        // 第三个乘积在 DSP 输入处变号，实/虚两个合成共用同一条 37 位加法链。
        state[F_M5]:tr<=quantized;
        state[F_M7]:ti<=quantized;
        // 四个 A±t 分四次做，操作数选择/变号先寄存，再走独立短进位链。
        state[F_PREP]:begin
            add_left<=operation[1]?{ai[15],ai}:{ar[15],ar};
            add_right<=(operation[1]?{ti[15],ti}:{tr[15],tr})^{17{operation[0]}};
            add_carry<=operation[0];
        end
        // carry-in先单独入寄存器，后面只剩两个17位操作数相加，不能串两条进位链。
        state[F_CARRY]:adjusted_right<=add_right+add_carry;
        state[F_ALU]:add_sum<=add_left+adjusted_right;
        state[F_SCALE]:scaled_q<=$signed({add_sum[16],add_sum[16:1]})+add_sum[0];
        // F_STORE 的结果直接由上面的 ram_wr_* 写入工作RAM，本状态仅推进调度。
        state[P_GET]:begin br<=ram_re[15:0]; bi<=ram_im[15:0]; end
        state[P_M2]:first_product<=mul_value;
        // 两个16位有符号数的平方和<=2^31，低32位完整，不会丢功率。
        // 前导位按16/8/4分组，每次带上后续三位；最高的 35..32 位组恒为零，不参与选择。
        state[P_GROUP]:begin
            if (|accumulator[31:16]) begin log_group<=accumulator[31:13]; log_base<=16; end
            else begin log_group<={accumulator[15:0],3'd0}; log_base<=0; end
        end
        state[P_BYTE]:begin
            if (|log_group[18:11]) begin log_byte<=log_group[18:8]; log_base[3]<=1; end
            else begin log_byte<=log_group[10:0]; log_base[3]<=0; end
        end
        state[P_NIBBLE]:begin
            if (|log_byte[10:7]) begin log_nibble<=log_byte[10:4]; log_base[2]<=1; end
            else begin log_nibble<=log_byte[6:0]; log_base[2]<=0; end
        end
        state[P_LOG]:begin
            if(log_nibble[6]) bin_lg<={1'b0,log_base[4:2],2'd3,log_nibble[5:3]};
            else if(log_nibble[5]) bin_lg<={1'b0,log_base[4:2],2'd2,log_nibble[4:2]};
            else if(log_nibble[4]) bin_lg<={1'b0,log_base[4:2],2'd1,log_nibble[3:1]};
            else if(log_nibble[3]) bin_lg<={1'b0,log_base[4:2],2'd0,log_nibble[2:0]};
            else bin_lg<=0;
            debug_re<={{2{br[15]}},br}; debug_im<={{2{bi[15]}},bi};
        end
        default:begin end
        endcase
        if(rst) begin
            state<=(49'd1<<IDLE); bin_valid<=0; done<=0; sat_flag<=0; overflow_valid<=0;
            base_ptr<=0; n<=0; copy_zero<=1; butterfly<=0;
            addr_a<=0; addr_b<=0; ram_rd_addr<=0; rom_addr<=0; ring_rd_addr<=0;
            scan_bin<=0; bin_idx<=0; operation<=0; twiddle_phase<=0; twiddle_step<=256;
        end else begin
            bin_valid<=0; done<=0;
            overflow_valid<=state[C_PACK] || state[F_M5] || state[F_M7];
            if(overflow_valid && overflow_q) sat_flag<=1;
            // 每个状态只出现一次，避免“默认ROM转移+特殊覆盖”串成深层状态选择。
            // 数据写入由上方独立 case 处理，此处更新状态、有效标志和地址指针。
            (* full_case, parallel_case *) case(1'b1)
            state[IDLE]:begin
                state<=(49'd1<<IDLE);
                // 未满时tap保证wptr==sample_count，回绕前补零，不能接不相关的两个快照。
                if(start) begin state<=(49'd1<<C_ADDR); base_ptr<=wptr; copy_zero<=!sample_count[9];
                    n<=0; sat_flag<=0; end
            end
            state[C_ADDR]:begin state<=(49'd1<<C_WAIT); ring_rd_addr<=base_ptr; rom_addr<={1'b0,n[7:0]}; end
            state[C_WAIT]:state<=(49'd1<<C_GET);
            state[C_GET]:state<=(49'd1<<C_HANN_ADJ);
            state[C_HANN_ADJ]:state<=(49'd1<<C_WIN);
            state[C_WIN]:state<=(49'd1<<C_MUL);
            state[C_MUL]:state<=(49'd1<<C_MWAIT);
            state[C_MWAIT]:state<=(49'd1<<C_MGET);
            state[C_MGET]:state<=(49'd1<<C_ROUND);
            state[C_ROUND]:state<=(49'd1<<C_PACK);
            state[C_PACK]:state<=(49'd1<<C_WRITE);
            state[C_WRITE]:state<=(49'd1<<C_NEXT);
            state[C_NEXT]:begin
                if(base_ptr==511) copy_zero<=0;
                if(n==511) begin state<=(49'd1<<F_ADDR); butterfly<=0; twiddle_phase<=0; twiddle_step<=256; end
                else begin n<=n_next; base_ptr<=base_next; state<=(49'd1<<C_ADDR); end
            end
            state[F_ADDR]:begin state<=(49'd1<<F_READ_A); addr_a<=next_a; addr_b<=next_b; rom_addr<={1'b0,twiddle_phase};operation<=0;end
            state[F_READ_A]:begin state<=(49'd1<<F_WAIT_A);ram_rd_addr<=addr_a;end
            state[F_WAIT_A]:state<=(49'd1<<F_GET_A);
            state[F_GET_A]:begin state<=(49'd1<<F_READ_B); ram_rd_addr<=addr_b; rom_addr<={1'b1,twiddle_phase}; end
            state[F_READ_B]:state<=(49'd1<<F_GET_B);
            state[F_GET_B]:state<=(49'd1<<F_M0);
            state[F_M0]:state<=(49'd1<<F_M1);
            state[F_M1]:state<=(49'd1<<F_M2);
            state[F_M2]:state<=(49'd1<<F_M3);
            state[F_M3]:state<=(49'd1<<F_M4);
            state[F_M4]:state<=(49'd1<<F_M5);
            state[F_M5]:state<=(49'd1<<F_M6);
            state[F_M6]:state<=(49'd1<<F_M7);
            state[F_M7]:state<=(49'd1<<F_PREP);
            state[F_PREP]:state<=(49'd1<<F_CARRY);
            state[F_CARRY]:state<=(49'd1<<F_ALU);
            state[F_ALU]:state<=(49'd1<<F_SCALE);
            state[F_SCALE]:state<=(49'd1<<F_STORE);
            state[F_STORE]:begin
                state<=(49'd1<<F_WRITE_A);
                if(scaled_overflow) sat_flag<=1;
                if(operation!=3) begin operation<=operation_next; state<=(49'd1<<F_PREP); end
            end
            state[F_WRITE_A]:state<=(49'd1<<F_WRITE_B);
            state[F_WRITE_B]:state<=(49'd1<<F_NEXT);
            state[F_NEXT]:begin
                if(butterfly==255) begin
                    butterfly<=0;twiddle_phase<=0;twiddle_step<={1'b0,twiddle_step[8:1]};
                    if(twiddle_step[0]) begin state<=(49'd1<<P_ADDR); scan_bin<=0; end
                    else state<=(49'd1<<F_ADDR);
                end else begin
                    butterfly<=butterfly_next;twiddle_phase<=phase_next;state<=(49'd1<<F_ADDR);
                end
            end
            state[P_ADDR]:begin state<=(49'd1<<P_WAIT); ram_rd_addr<={1'b0,scan_bin}; end
            state[P_WAIT]:state<=(49'd1<<P_GET);
            state[P_GET]:state<=(49'd1<<P_M0);
            state[P_M0]:state<=(49'd1<<P_M1);
            state[P_M1]:state<=(49'd1<<P_M2);
            state[P_M2]:state<=(49'd1<<P_M3);
            state[P_M3]:state<=(49'd1<<P_GROUP);
            state[P_GROUP]:state<=(49'd1<<P_BYTE);
            state[P_BYTE]:state<=(49'd1<<P_NIBBLE);
            state[P_NIBBLE]:state<=(49'd1<<P_LOG);
            state[P_LOG]:state<=(49'd1<<P_OUT);
            state[P_OUT]:begin state<=(49'd1<<P_NEXT); bin_valid<=1; bin_idx<=scan_bin; end
            state[P_NEXT]:begin
                if(scan_bin==255) state<=(49'd1<<FINISH);
                else begin scan_bin<=scan_next;state<=(49'd1<<P_ADDR);end
            end
            state[FINISH]:begin done<=1;state<=(49'd1<<IDLE);end
            default:begin end // one-hot不变量保证正常运行不走到这里
            endcase
        end
    end
endmodule
`default_nettype wire
