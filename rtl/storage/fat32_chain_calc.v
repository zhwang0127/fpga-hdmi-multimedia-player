`timescale 1ns/1ps
`default_nettype none

// FAT32链计算共享单元：纯组合，无状态。
// 目录扫描与文件流是services里互斥的两个extent所有者，共用一个实例
// （输入由所有权多路选择），消除两份宽移位/加法与28位表项判定。
// 全部位宽先扩展后比较：39位数据LBA、33位FAT LBA、28位表项值，禁止截断。
module fat32_chain_calc(
    input wire [31:0] cur_cluster,base_fat_lba,base_data_lba,fat_value,
    input wire [27:0] cluster_count,
    input wire [2:0] cluster_shift,
    input wire [5:0] cluster_sector_off,
    output wire [38:0] cluster_lba,
    output wire [32:0] fat_entry_lba,
    output wire cluster_lba_overflow,fat_entry_lba_overflow,
    output wire [27:0] max_cluster,
    output wire entry_eoc,entry_bad_reserved,entry_in_range
);
    wire [37:0] cluster_shifted={6'd0,cur_cluster-32'd2}<<cluster_shift;
    assign cluster_lba={1'b0,cluster_shifted}+{7'd0,base_data_lba}+
                       {8'd0,{2'd0,cluster_sector_off[5:0]}};
    assign fat_entry_lba={1'b0,base_fat_lba}+{1'b0,cur_cluster[31:7]};
    assign cluster_lba_overflow=|cluster_lba[38:32];
    assign fat_entry_lba_overflow=|fat_entry_lba[32];
    assign max_cluster=cluster_count+28'd1;
    // FAT32表项只有低28位有效：EOC区间0FFFFFF8..0FFFFFFF、保留/坏簇0FFFFFF0..7。
    assign entry_eoc=fat_value[27:0]>=28'hffffff8;
    assign entry_bad_reserved=fat_value[27:0]>=28'hffffff0 &&
                              fat_value[27:0]<28'hffffff8;
    assign entry_in_range=fat_value[27:0]>=28'd2 && fat_value[27:0]<=max_cluster;
endmodule
`default_nettype wire
