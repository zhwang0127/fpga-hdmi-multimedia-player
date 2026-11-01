# FPGA 智能多媒体展播终端

基于 HX4S20C 开发板和 EG4S20BG256 FPGA 的离线 HDMI 多媒体展播工程。系统从 TF 卡读取图片和短音频，使用 SDRAM 缓存，通过 HDMI 输出 720p 画面和音频，支持按键切换、自动轮播和 OSD 菜单。

## 获取源码

下载并解压 [fpga-hdmi-source.zip](fpga-hdmi-source.zip)。压缩包包含 RTL 源码、头文件、85 个活动 Verilog 文件清单、ADC/SDC 约束、构建脚本和工程说明，保留原目录结构。

## 构建与验证

使用 TD 6.2.1，器件 EG4S20BG256，顶层 multimedia_top。按照压缩包内 td/multimedia_top.f 导入源码，添加 multimedia.adc 和 multimedia.sdc。本机 GUI 使用 seed 20 完成综合、布局布线和位流生成，报告 SWNS +0.118 ns、HWNS +0.120 ns；该位流已由用户反馈上板测试通过。

BUILD.tcl 的 seed 设为 20，但命令行与 GUI MultiSeed 流程的结果尚未验证完全等效。SDC 未覆盖外部输入输出延时；原运行日志含 SDRAM 位置和局部时钟布线严重警告。不同环境请重新核查路径、时序与板上功能。

## 素材与操作

使用 FAT32 格式 TF 卡，在 MEDIA 目录准备 P0001.BMP 至 P0004.BMP 和同编号 WAV。图片为 640×480 BMP，音频为 48 kHz、16 位双声道 PCM 短音频。K1 切图，K2 启停轮播，K3 切换菜单，K4 调整菜单参数。

本仓库未指定开源许可证，使用和再分发须取得权利人许可。
