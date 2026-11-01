# FPGA 智能多媒体展播终端

基于 HX4S20C 开发板和安路 EG4S20BG256 FPGA 的离线 HDMI 多媒体展播工程。通过 TF 卡读取 BMP 图像与 PCM WAV 音频，使用 SDRAM 缓存，并输出 720p HDMI 画面与音频；支持按键切换、自动轮播、OSD 和音频频谱。

## 工程内容

- `rtl/`：产品 Verilog 源码和头文件。
- `td/multimedia_top.f`：85 个活动源码，路径相对于仓库根目录。
- `td/multimedia.adc` 和 `td/multimedia.sdc`：引脚和时序约束。
- `BUILD.tcl`：TD 命令行构建脚本。

## 构建及验证

使用 TD 6.2.1，目标器件 EG4S20BG256，顶层 `multimedia_top`。GUI 中按 `td/multimedia_top.f` 导入源码，并添加 ADC 与 SDC；本机 GUI 使用 MultiSeed seed 20 完成布局布线并生成位流，报告 SWNS +0.118 ns、HWNS +0.120 ns，用户反馈该位流上板测试通过。

`BUILD.tcl` 设置 seed 20，命令行流程与 GUI MultiSeed 是否完全等效尚未验证。请在自己的环境重新检查路径、综合、时序和板上运行。当前 SDC 未覆盖外部输入输出延时；原运行日志有 SDRAM 位置与局部时钟布线严重警告，需结合板卡进一步核查。时序余量较小，不能保证不同工具环境重建同样通过。

## 素材与操作

FAT32 TF 卡的 `MEDIA` 目录放置 `P0001.BMP` 至 `P0004.BMP` 及相同编号 WAV。图片为 640×480，音频为 48 kHz、16 位双声道 PCM 短音频。K1 切换图片，K2 启停轮播，K3 切换菜单，K4 调整菜单参数。

本仓库未指定开源许可证；使用和再分发须获得权利人许可。第三方权利以对应权利人条款为准。