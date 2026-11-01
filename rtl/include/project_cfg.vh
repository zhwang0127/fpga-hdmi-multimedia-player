// 工程规格常量：使用方需显式 include，本文件不生成时钟或改变模块参数。
// include guard 防止同一编译单元重复定义；频率单位 Hz，尺寸单位像素。
`ifndef FPGA_PLAYER_CFG_VH
`define FPGA_PLAYER_CFG_VH
`define FPGA_PLAYER_PIX_HZ 74250000 // 720p60 的目标像素时钟。
`define FPGA_PLAYER_AUDIO_HZ 48000
`define FPGA_PLAYER_H_TOTAL 1650 // 包含有效区、前沿、同步脉冲与后沿。
`define FPGA_PLAYER_V_TOTAL 750
`define FPGA_PLAYER_IMAGE_WIDTH 640
`define FPGA_PLAYER_IMAGE_HEIGHT 480
`define FPGA_PLAYER_LUT_LIMIT 15680 // 建议 LUT 预算；器件容量和最终资源上限见 RELEASE.json。
`endif
