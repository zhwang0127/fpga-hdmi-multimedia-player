# multimedia_top 时序约束：板载输入为 50MHz，PLL 输出由实际原语参数推导。
# pix、word、gear、serial 为同源相关时钟，保留它们之间的正常时序分析。
# CDC 例外按具体实例和网络设置，仅覆盖同步入口、mailbox 保持总线及 FIFO Gray 指针。
# 复位例外限于异步置位源网，同步释放级及本域复位路径仍参与时序分析。
# 本文件未设置外部 IO 延迟；器件内部 IO 路径仍由工具分析。
create_clock -name clk_50m -period 20.000 -waveform {0.000 10.000} [get_ports {clk_50m}]

# 推导存储及视频 PLL 的全部生成时钟。
derive_clocks -instance [get_cells {u_pll_mem/u_pll}]
derive_clocks -instance [get_cells {u_pll_video/u_pll_ref}]
derive_clocks -instance [get_cells {u_pll_video/u_pll_pix}]

# domain_arst 只驱动复位同步器的异步置位输入。
# 目标域的三级同步释放链不经过此源网，不属于该例外。
set_false_path -through [get_nets {domain_arst}]

# SDRAM 读 DQ 使用两周期捕获，只限定器件 dq 输出到 clk_mem 捕获端。
# mem/device 周期均为 10ns，device 延后 7.375ns，对应捕获间隔 2T-d=12.625ns。
# 控制器读延迟与寄存 PHY 配套；写方向及其他跨域路径不使用此多周期例外。
# 不添加 hold 1，保持相邻连续数据字之间的实际保持时间关系。
set_multicycle_path -setup 2 -from [get_pins {u_sdram_phy/anlogic$u_sdram.dq[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]

# CDC 控制网络：仅对已列明实例的首级同步入口设置例外。
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_audio/u_fifo_a/rd_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_audio/u_fifo_a/wr_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_audio/u_fifo_b/rd_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_audio/u_fifo_b/wr_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_evt/ack_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_evt/dst_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_evt/req_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_evt/src_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_rel/ack_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_rel/dst_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_rel/req_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_rel/src_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_req/ack_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_req/dst_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_req/req_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_req/src_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/px_fault}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_ack/ack_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_ack/dst_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_ack/req_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_ack/src_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_desc/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_desc/dst_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_desc/req_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_desc/src_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_table/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_table/dst_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_table/req_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_table/src_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_retired/ack_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_retired/dst_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_retired/req_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_retired/src_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_abort/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_abort/dst_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_abort/req_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_abort/src_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_command/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_command/dst_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_command/req_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_command/src_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_event/ack_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_event/dst_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_event/req_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_event/src_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_quantum_receipt/ack_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_quantum_receipt/dst_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_quantum_receipt/req_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_quantum_receipt/src_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_bytes/rd_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_bytes/wr_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_cancel/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_cancel/dst_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_cancel/req_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_cancel/src_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_descriptor/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_descriptor/dst_up}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_descriptor/req_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_descriptor/src_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_result/ack_toggle}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_result/dst_up}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_result/req_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_result/src_up}] -to [get_clocks {clk_50m}]

# mailbox 保持总线最大延迟限制为一个目标时钟周期，源端数据持续保持到确认返回。
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_disp_bundle/u_evt/held_bus[*]}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] 13.468
set_max_delay -datapath_only -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_rel/held_bus[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 10.000
set_max_delay -datapath_only -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_bundle/u_req/held_bus[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 10.000
set_max_delay -datapath_only -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_owner/u_ack/held_bus[*]}] -to [get_clocks {clk_50m}] 20.000
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_desc/held_bus[*]}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] 13.468
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_disp_owner/u_owner/u_table/held_bus[*]}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] 13.468
set_max_delay -datapath_only -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_disp_owner/u_retired/held_bus[*]}] -to [get_clocks {clk_50m}] 20.000
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_abort/held_bus[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 10.000
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_load/u_command/held_bus[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 10.000
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_load/u_event/held_bus[*]}] -to [get_clocks {clk_50m}] 20.000
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_quantum_receipt/held_bus[*]}] -to [get_clocks {clk_50m}] 20.000
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_cancel/held_bus[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 10.000
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_descriptor/held_bus[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 10.000
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_result/held_bus[*]}] -to [get_clocks {clk_50m}] 20.000

# FIFO Gray 总线最大延迟限制为一个源时钟周期，后续同步级保留正常时序检查。
set_max_delay -datapath_only -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_audio/u_fifo_a/rgray[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 13.468
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_audio/u_fifo_a/wgray[*]}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] 10.000
set_max_delay -datapath_only -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {u_audio/u_fifo_b/rgray[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 13.468
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_audio/u_fifo_b/wgray[*]}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] 10.000
set_max_delay -datapath_only -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_bytes/rgray[*]}] -to [get_clocks {clk_50m}] 10.000
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_bytes/wgray[*]}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}] 20.000

# FIFO flush_hold 只用于两端复位同步器的异步置位，不切断本域同步释放路径。
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_audio/u_fifo_a/flush_hold}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_mem/u_pll.clkc[1]}] -through [get_nets {u_audio/u_fifo_b/flush_hold}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {u_media/u_loader/u_storage/u_mount/u_extent/u_bridge/u_bytes/flush_hold}] -to [get_clocks {u_pll_mem/u_pll.clkc[1]}]

# OSD 状态 mailbox：控制翻转位进入同步级，保持总线最大延迟为一个 pix 周期。
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {g_osd$u_osd_status/req_toggle}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {clk_50m}] -through [get_nets {g_osd$u_osd_status/src_up}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {g_osd$u_osd_status/ack_toggle}] -to [get_clocks {clk_50m}]
set_false_path -from [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] -through [get_nets {g_osd$u_osd_status/dst_up}] -to [get_clocks {clk_50m}]
set_max_delay -datapath_only -from [get_clocks {clk_50m}] -through [get_nets {g_osd$u_osd_status/held_bus[*]}] -to [get_clocks {u_pll_video/u_pll_pix.clkc[0]}] 13.468
