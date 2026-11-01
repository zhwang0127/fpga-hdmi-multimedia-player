`timescale 1ns/1ps
`default_nettype none
// HDMI 数据岛包内容生成（纯组合，规格 §10.5、§11.3）。
// 根据 sel 给出一个包的 24bit 包头与 4×56bit 子包，交给 hdmi_packet_serializer 加 ECC 并串行发送。
// 支持的包（规格 §10.5 包类型表）：
//   SEL_NULL  0x00 NULL：包头/子包全 0（规格“NULL payload设0”）；
//   SEL_ACR   0x01 音频时钟再生：N=6144、CTS=74250（规格 §11.3，74.25MHz×6144/(128×74250)=48000）；
//   SEL_ASP   0x02 音频样本包 Layout0：最多 4 个立体声样本，每子包 1 个，present 位逐个标记；
//   SEL_AVI   0x82 AVI InfoFrame v2：RGB、16:9、VIC=4（720p60），其余字段 0；
//   SEL_AIF   0x84 Audio InfoFrame v1：CC=1（两声道），CT/SF/SS=0（“参照流头”）。
// 本实现不发送 GCP；InfoFrame 的校验和与音频样本偶校验由字段值生成。
// 输出固定针对 720p60、48kHz 双声道 PCM，不进行 EDID 协商。
module hdmi_packet_builder(
    input  wire [2:0]  sel,
    input  wire [63:0] samp_l,        // 4 个左声道 signed16，样本 j 在 [16j+15:16j]
    input  wire [63:0] samp_r,        // 4 个右声道 signed16
    input  wire [2:0]  nsamp,         // 本包有效样本数 0..4（只对 SEL_ASP 有意义）
    input  wire [7:0]  iec_frame,     // 本包第 0 个样本的 IEC60958 帧号 0..191
    output reg  [23:0] header,
    output reg  [223:0] sub
);
    localparam [2:0] SEL_ACR=3'd1,SEL_ASP=3'd2,SEL_AVI=3'd3,SEL_AIF=3'd4;
    // ---- ACR 常量（20bit 字段）----
    localparam [19:0] ACR_N=20'd6144, ACR_CTS=20'd74250;
    // 子包字节 SB0..SB6：SB0=0，SB1..3=CTS 高到低，SB4..6=N 高到低；SB0 在 [7:0]。
    localparam [55:0] ACR_SUB={ACR_N[7:0],ACR_N[15:8],4'd0,ACR_N[19:16],
                               ACR_CTS[7:0],ACR_CTS[15:8],4'd0,ACR_CTS[19:16],8'h00};
    // ---- AVI InfoFrame（HB0=0x82 类型，HB1=0x02 版本，HB2=0x0D 长度 13）----
    // PB1=0x00：Y=00(RGB)、A0=0、B=00、S=00；PB2=0x28：C=00、M=10(16:9)、R=1000(同图像比例)；
    // PB3=0x00：Q=00(按 VIC 默认量化范围)；PB4=0x04：VIC=4。checksum 使包头 3 字节+PB0..PB13 之和模 256 为 0。
    localparam integer AVI_SUM=130+2+13+0+40+0+4;   // 0x82+0x02+0x0D+PB1 0x00+PB2 0x28+PB3 0x00+PB4 0x04
    localparam integer AVI_CHK_I=(256-(AVI_SUM%256))%256;
    localparam [7:0]   AVI_CHK=AVI_CHK_I[7:0];
    localparam [55:0]  AVI_SUB0={8'h00,8'h00,8'h04,8'h00,8'h28,8'h00,AVI_CHK};   // PB6..PB0
    // ---- Audio InfoFrame（HB0=0x84，HB1=0x01，HB2=0x0A 长度 10）----
    // PB1=0x01：CT=0(参照流头)、CC=1(两声道)；PB2..PB10=0（SF/SS 参照流头，CA=0 即 FL/FR）。
    localparam integer AIF_SUM=132+1+10+1;          // 0x84+0x01+0x0A+PB1 0x01
    localparam integer AIF_CHK_I=(256-(AIF_SUM%256))%256;
    localparam [7:0]   AIF_CHK=AIF_CHK_I[7:0];
    localparam [55:0]  AIF_SUB0={8'h00,8'h00,8'h00,8'h00,8'h00,8'h01,AIF_CHK};

    // IEC 60958 消费级 channel status（左右各 192bit，这里只有前 5 字节非 0）：
    // byte0=0x04（消费级、线性 PCM、未声明版权保护）；byte2 高 4 位=声道号（左 1、右 2）；
    // byte3=0x02（48kHz）；byte4=0x02（最大字长 20、实际 16bit）。其余位为 0。
    // 帧计数与样本偏移之和最多为 258，按 192 回绕时最多减一次。
    // 匹配 target 时并行检查 target 和 target+192，覆盖完整 8 位帧计数输入。
    function frame_eq(input [7:0] f,input [1:0] step,input [7:0] target);
        reg [8:0] first,wrapped;
        begin
            first={1'b0,target}-{7'd0,step};
            wrapped={1'b0,target}+9'd192-{7'd0,step};
            frame_eq=((target<8'd192 && {1'b0,target}>={7'd0,step}) && {1'b0,f}==first) || ({1'b0,f}==wrapped);
        end
    endfunction
    function cs_shifted(input [7:0] f,input [1:0] step,input right);
        begin cs_shifted=frame_eq(f,step,8'd2) || frame_eq(f,step,right?8'd21:8'd20) ||
            frame_eq(f,step,8'd25) || frame_eq(f,step,8'd33); end
    endfunction
    integer j;
    reg [23:0] l24,r24; reg cl,cr,pl,pr; reg [3:0] present,bflag;
    always @* begin
        // 先给所有中间变量默认值，避免综合出锁存器。
        header=24'd0; sub=224'd0; present=4'd0; bflag=4'd0;
        l24=24'd0; r24=24'd0; cl=1'b0; cr=1'b0; pl=1'b0; pr=1'b0;
        case(sel)
            SEL_ACR: begin
                header={8'h00,8'h00,8'h01};
                for(j=0;j<4;j=j+1) sub[56*j +: 56]=ACR_SUB;
            end
            SEL_ASP: begin
                for(j=0;j<4;j=j+1) begin
                    if(j<nsamp) begin
                        // signed16 左对齐进 24bit 容器：{pcm,8'h00}（规格 §11.3）。
                        l24={samp_l[16*j +: 16],8'h00}; r24={samp_r[16*j +: 16],8'h00};
                        cl=cs_shifted(iec_frame,j[1:0],1'b0); cr=cs_shifted(iec_frame,j[1:0],1'b1);
                        // 偶校验 P：使 24bit 样本 + V + U + C + P 中 1 的个数为偶数；V=U=0。
                        pl=^{l24,1'b0,1'b0,cl}; pr=^{r24,1'b0,1'b0,cr};
                        // SB6 = {PR,CR,UR,VR,PL,CL,UL,VL}，VL 在 bit48。
                        sub[56*j +: 56]={pr,cr,1'b0,1'b0,pl,cl,1'b0,1'b0,r24,l24};
                        present[j]=1'b1;
                        bflag[j]=frame_eq(iec_frame,j[1:0],8'd0);   // 192 帧块的第一帧标 B
                    end
                end
                // HB1={3'b000,layout=0,sample_present[3:0]}；HB2={B[3:0],sample_flat[3:0]=0}。
                header={bflag,4'b0000,3'b000,1'b0,present,8'h02};
            end
            SEL_AVI: begin header={8'h0D,8'h02,8'h82}; sub[55:0]=AVI_SUB0; end
            SEL_AIF: begin header={8'h0A,8'h01,8'h84}; sub[55:0]=AIF_SUB0; end
            default: ;   // SEL_NULL（0）及未定义值：全 0
        endcase
    end
endmodule
`default_nettype wire
