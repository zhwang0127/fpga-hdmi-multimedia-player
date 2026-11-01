`timescale 1ns/1ps
`default_nettype none

// FAT32 引导扇区解析器，消费完整的 512 字节扇区。
// 识别 superfloppy 或单个 0B/0C MBR 分区，提取 BPB 和分区字段，几何合法性由后级检查。
// IN_PLACE=0 输出字段原值；=1 以逐字节匹配标志保存固定值字段，并重建供后级比较的规范值。
// signature、扇区大小和要求为零的字段仅保留比较结果；ext_flags 保留 bit7 和低四位。
// 簇大小、FAT 数量、保留区、总扇区、FAT 大小、根簇及分区范围保留完整数值。
// 整机由 fat32_mount_storage 选择 IN_PLACE=1，以减少仅用于固定值比较的寄存器。
// abort 撤销本地解析，不代表 SD 源端已停止；父级须另行等待来源排空。
// rst 异步置位、同步释放，业务有效条件使用本域复位。
module fat32_boot_parser #(parameter IN_PLACE=0)(
    input wire clk,arst,abort,
    input wire start_valid,output wire start_ready,
    input wire byte_valid,output wire byte_ready,
    input wire [7:0] byte_data,input wire [8:0] byte_index,input wire byte_last,
    output wire result_valid,input wire result_ready,
    output wire [2:0] result_code,output wire result_is_mbr,
    output wire [31:0] partition_lba,output wire [32:0] partition_sectors,
    output wire [15:0] signature,bytes_per_sector,reserved_sectors,
    output wire [7:0] sectors_per_cluster,num_fats,
    output wire [15:0] root_entry_count,total_sectors16,fat_sectors16,fs_version,ext_flags,
    output wire [31:0] total_sectors,fat_sectors,root_cluster
);
    wire rst_logic;
    localparam IDLE=0,COLLECT=1,RESULT=2;
    localparam OK=0,MBR_SIGNATURE=1,PARTITION_LAYOUT=2,STREAM=3;
    wire rst;
    reset_sync u_reset(.clk(clk),.arst(arst),.rst(rst),.logic_rst(rst_logic));
    // 原始共同arst仅用于reset_sync；业务只看本域异步置位/同步释放的rst。
    wire enabled=!rst_logic && !abort;
    reg [1:0] state;
    reg [8:0] expected;
    reg stream_bad;
    reg [2:0] code;
    reg is_mbr;
    reg [15:0] b_signature,b_bps,b_reserved,b_root_entries,b_total16,b_fat16,b_version,b_flags;
    reg [7:0] b_spc,b_fats;
    reg [31:0] b_total,b_fat_size,b_root;
    reg [2:0] present_count;
    reg entry_nonzero;
    reg [7:0] entry_status,entry_type;
    reg [31:0] entry_start,entry_count;
    reg [7:0] selected_status,selected_type;
    reg [31:0] selected_start,selected_count;
    // 就地比对标志。*_ok = 该字节等于目标字节；*_z = 该字节为0。
    // 初值等于“原寄存器为0时”的比较结果：bps低字节目标00→1，高字节目标02→0；零字段→1；签名→0。
    reg sig_lo_ok,sig_hi_ok,bps_lo_ok,bps_hi_ok;
    reg re_lo_z,re_hi_z,t16_lo_z,t16_hi_z,f16_lo_z,f16_hi_z,ver_lo_z,ver_hi_z;
    reg flag7;reg [3:0] flag_low;
    // 分区项 status∈{00,80}、type∈{0B,0C} 只存1位；初值对应原寄存器0：status=00合法→1，type=00非法→0。
    reg entry_status_ok,entry_type_ok,selected_status_ok,selected_type_ok;
    wire fire=byte_valid && byte_ready;
    wire supported_spc=b_spc==1 || b_spc==2 || b_spc==4 || b_spc==8 ||
                       b_spc==16 || b_spc==32 || b_spc==64;
    wire bps_is_512=IN_PLACE ? (bps_lo_ok && bps_hi_ok) : b_bps==512;
    wire plausible_bpb=bps_is_512 && supported_spc && b_reserved!=0;
    // MBR 签名按低字节标志和当前高字节是否等于 AA 联合判断。
    wire mbr_sig_ok=IN_PLACE ? (byte_data==8'haa && sig_lo_ok) : {byte_data,b_signature[7:0]}==16'haa55;
    wire sel_status_ok=IN_PLACE ? selected_status_ok : (selected_status==0 || selected_status==8'h80);
    wire sel_type_ok=IN_PLACE ? selected_type_ok : (selected_type==8'h0b || selected_type==8'h0c);
    wire final_stream_bad=stream_bad || byte_index!=expected || byte_index!=9'd511 || !byte_last;
    assign start_ready=enabled && state==IDLE;
    assign byte_ready=enabled && state==COLLECT;
    assign result_valid=enabled && state==RESULT;
    assign result_code=code;assign result_is_mbr=is_mbr;
    assign partition_lba=is_mbr ? selected_start : 32'd0;
    assign partition_sectors=is_mbr ? {1'b0,selected_count} : 33'd0;
    // IN_PLACE=1 输出重建的规范值（只保证后级比较结果与原值相同，见文件头注释）。
    assign signature=IN_PLACE ? {sig_hi_ok ? 8'haa : 8'h00,sig_lo_ok ? 8'h55 : 8'h00} : b_signature;
    assign bytes_per_sector=IN_PLACE ? {bps_hi_ok ? 8'h02 : 8'h00,bps_lo_ok ? 8'h00 : 8'h01} : b_bps;
    assign reserved_sectors=b_reserved;assign sectors_per_cluster=b_spc;assign num_fats=b_fats;
    assign root_entry_count=IN_PLACE ? {7'd0,!re_hi_z,7'd0,!re_lo_z} : b_root_entries;
    assign total_sectors16=IN_PLACE ? {7'd0,!t16_hi_z,7'd0,!t16_lo_z} : b_total16;
    assign fat_sectors16=IN_PLACE ? {7'd0,!f16_hi_z,7'd0,!f16_lo_z} : b_fat16;
    assign fs_version=IN_PLACE ? {7'd0,!ver_hi_z,7'd0,!ver_lo_z} : b_version;
    assign ext_flags=IN_PLACE ? {8'd0,flag7,3'd0,flag_low} : b_flags;
    assign total_sectors=b_total;assign fat_sectors=b_fat_size;assign root_cluster=b_root;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;expected<=0;stream_bad<=0;code<=OK;is_mbr<=0;
            b_signature<=0;b_bps<=0;b_reserved<=0;b_root_entries<=0;b_total16<=0;
            b_fat16<=0;b_version<=0;b_flags<=0;b_spc<=0;b_fats<=0;
            b_total<=0;b_fat_size<=0;b_root<=0;present_count<=0;entry_nonzero<=0;
            entry_status<=0;entry_type<=0;entry_start<=0;entry_count<=0;
            selected_status<=0;selected_type<=0;selected_start<=0;selected_count<=0;
            sig_lo_ok<=0;sig_hi_ok<=0;bps_lo_ok<=1;bps_hi_ok<=0;
            re_lo_z<=1;re_hi_z<=1;t16_lo_z<=1;t16_hi_z<=1;f16_lo_z<=1;f16_hi_z<=1;ver_lo_z<=1;ver_hi_z<=1;
            flag7<=0;flag_low<=0;entry_status_ok<=1;entry_type_ok<=0;selected_status_ok<=1;selected_type_ok<=0;
        end else begin
            if(start_valid && start_ready) begin
                state<=COLLECT;expected<=0;stream_bad<=0;code<=OK;is_mbr<=0;
                b_signature<=0;b_bps<=0;b_reserved<=0;b_root_entries<=0;b_total16<=0;
                b_fat16<=0;b_version<=0;b_flags<=0;b_spc<=0;b_fats<=0;
                b_total<=0;b_fat_size<=0;b_root<=0;present_count<=0;entry_nonzero<=0;
                entry_status<=0;entry_type<=0;entry_start<=0;entry_count<=0;
                selected_status<=0;selected_type<=0;selected_start<=0;selected_count<=0;
                sig_lo_ok<=0;sig_hi_ok<=0;bps_lo_ok<=1;bps_hi_ok<=0;
                re_lo_z<=1;re_hi_z<=1;t16_lo_z<=1;t16_hi_z<=1;f16_lo_z<=1;f16_hi_z<=1;ver_lo_z<=1;ver_hi_z<=1;
                flag7<=0;flag_low<=0;entry_status_ok<=1;entry_type_ok<=0;selected_status_ok<=1;selected_type_ok<=0;
            end
            if(fire) begin
                expected<=expected+9'd1;
                if(byte_index!=expected || byte_last!=(expected==9'd511)) stream_bad<=1;
                // 就地比对标志，与下方原字段写入同索引、同拍、最后写入者生效。
                case(byte_index)
                    9'd11:bps_lo_ok<=byte_data==8'h00;9'd12:bps_hi_ok<=byte_data==8'h02;
                    9'd17:re_lo_z<=byte_data==0;9'd18:re_hi_z<=byte_data==0;
                    9'd19:t16_lo_z<=byte_data==0;9'd20:t16_hi_z<=byte_data==0;
                    9'd22:f16_lo_z<=byte_data==0;9'd23:f16_hi_z<=byte_data==0;
                    9'd40:begin flag7<=byte_data[7];flag_low<=byte_data[3:0];end
                    9'd42:ver_lo_z<=byte_data==0;9'd43:ver_hi_z<=byte_data==0;
                    9'd510:sig_lo_ok<=byte_data==8'h55;9'd511:sig_hi_ok<=byte_data==8'haa;
                    default:begin end
                endcase
                // IN_PLACE=1 时下列被就地比对取代的原字段不再写入（常量0，综合删除）。
                if(!IN_PLACE) case(byte_index)
                    9'd11:b_bps[7:0]<=byte_data;9'd12:b_bps[15:8]<=byte_data;
                    9'd17:b_root_entries[7:0]<=byte_data;9'd18:b_root_entries[15:8]<=byte_data;
                    9'd19:b_total16[7:0]<=byte_data;9'd20:b_total16[15:8]<=byte_data;
                    9'd22:b_fat16[7:0]<=byte_data;9'd23:b_fat16[15:8]<=byte_data;
                    9'd40:b_flags[7:0]<=byte_data;9'd41:b_flags[15:8]<=byte_data;
                    9'd42:b_version[7:0]<=byte_data;9'd43:b_version[15:8]<=byte_data;
                    9'd510:b_signature[7:0]<=byte_data;9'd511:b_signature[15:8]<=byte_data;
                    default:begin end
                endcase
                case(byte_index)
                    9'd13:b_spc<=byte_data;
                    9'd14:b_reserved[7:0]<=byte_data;9'd15:b_reserved[15:8]<=byte_data;
                    9'd16:b_fats<=byte_data;
                    9'd32:b_total[7:0]<=byte_data;9'd33:b_total[15:8]<=byte_data;
                    9'd34:b_total[23:16]<=byte_data;9'd35:b_total[31:24]<=byte_data;
                    9'd36:b_fat_size[7:0]<=byte_data;9'd37:b_fat_size[15:8]<=byte_data;
                    9'd38:b_fat_size[23:16]<=byte_data;9'd39:b_fat_size[31:24]<=byte_data;
                    9'd44:b_root[7:0]<=byte_data;9'd45:b_root[15:8]<=byte_data;
                    9'd46:b_root[23:16]<=byte_data;9'd47:b_root[31:24]<=byte_data;
                    default:begin end
                endcase
                // 四个16字节分区项只保留字段和“任一字节非零”信息；CHS仍计入present判断。
                if(byte_index>=446 && byte_index<=509) begin
                    if(byte_index==446 || byte_index==462 || byte_index==478 || byte_index==494) begin
                        entry_nonzero<=byte_data!=0;
                        if(!IN_PLACE) begin entry_status<=byte_data;entry_type<=0;end
                        entry_status_ok<=byte_data==8'h00 || byte_data==8'h80;entry_type_ok<=0;
                        entry_start<=0;entry_count<=0;
                    end else entry_nonzero<=entry_nonzero || byte_data!=0;
                    case((byte_index-446)&15)
                        4:begin
                            if(!IN_PLACE) entry_type<=byte_data;
                            entry_type_ok<=byte_data==8'h0b || byte_data==8'h0c;
                        end
                        8:entry_start[7:0]<=byte_data;9:entry_start[15:8]<=byte_data;
                        10:entry_start[23:16]<=byte_data;11:entry_start[31:24]<=byte_data;
                        12:entry_count[7:0]<=byte_data;13:entry_count[15:8]<=byte_data;
                        14:entry_count[23:16]<=byte_data;
                        15:begin
                            entry_count[31:24]<=byte_data;
                            if(entry_nonzero || byte_data!=0) begin
                                present_count<=present_count+3'd1;
                                if(!IN_PLACE) begin selected_status<=entry_status;selected_type<=entry_type;end
                                selected_status_ok<=entry_status_ok;selected_type_ok<=entry_type_ok;
                                selected_start<=entry_start;selected_count<={byte_data,entry_count[23:0]};
                            end
                        end
                        default:begin end
                    endcase
                end
                if(byte_last || expected==9'd511) begin
                    state<=RESULT;
                    if(final_stream_bad) begin code<=STREAM;is_mbr<=0;end
                    else if(plausible_bpb) begin code<=OK;is_mbr<=0;end
                    else begin
                        is_mbr<=1;
                        if(!mbr_sig_ok) code<=MBR_SIGNATURE;
                        else if(present_count!=1 || !sel_status_ok || !sel_type_ok ||
                                selected_start==0 || selected_count==0) code<=PARTITION_LAYOUT;
                        else code<=OK;
                    end
                end
            end
            if(state==RESULT && result_valid && result_ready) state<=IDLE;
            if(abort) begin state<=IDLE;stream_bad<=0;end
        end
    end
endmodule
`default_nettype wire
