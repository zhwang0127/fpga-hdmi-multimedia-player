// SDRAM 8 MiB 地址空间规划；所有宏均为 23bit 字节地址。
// 图像按 00RRGGBB 的 32bit 字存放，字地址接口使用这些值右移两位。
// 本文件只定义区域起点，不代替访问范围检查或缓冲区所有权管理。
`ifndef FPGA_PLAYER_MEM_MAP_VH
`define FPGA_PLAYER_MEM_MAP_VH
// Byte addresses; modules accepting word addresses must shift right by two.
// 四个图槽各预留 0x140000 字节；有效像素数据实际占 640*480*4 字节。
`define IMAGE0_BYTE_BASE 23'h000000
`define IMAGE1_BYTE_BASE 23'h140000
`define IMAGE2_BYTE_BASE 23'h280000
`define IMAGE3_BYTE_BASE 23'h3c0000
// 两个 PCM 缓冲区各 1 MiB：A[500000,600000)，B[600000,700000)。
`define PCM_A_BYTE_BASE 23'h500000
`define PCM_B_BYTE_BASE 23'h600000
// 缩略图区 [700000,740000)，工作区 [740000,800000)，区间右端不包含。
`define THUMB_BYTE_BASE 23'h700000
`define WORK_BYTE_BASE 23'h740000
`endif
