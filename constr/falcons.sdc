// ============================================================================
// falcons.sdc —— FALCONS-embodied-robot 时序约束
//
// ★ 必须与 falcons.cst 成对给出：只给 .cst 不给 .sdc 时，工具按默认 100 MHz
//   估算时钟，会报出假的时序违规（曾出现 slack −7.32 ns 的假违规）。
//
// 本设计只有一个外部时钟：板载 50 MHz（Y18）。
//   * HDMI 的像素时钟（49 MHz，本设计里时钟名就叫 pixel_clk）与串行时钟
//     （u_hdmi/clk_serial_5x，245 MHz）都由 hdmi_1024x600 内部 PLL 派生，
//     工具会自动建立它们与 PLL 输入的关系，这里不重复 create_clock。
//   * 编码器 A/B 与遥控 PWM 是异步输入：quad_decoder / rc_pwm_capture 内部都有
//     两级同步器，故按保守做法不加 set_input_delay。
// ============================================================================

create_clock -name i_clk_50m -period 20.000 [get_ports {i_clk_50m}]

// ---- 摄像头 DVP 像素时钟（2026-10-17 接入摄像头后新增）----
//   ★ 周期 19.048 ns（≈52.5 MHz）是**实测值**，不是估计 —— 取自模块自带约束
//     vendor\verilog-reusable-modules\modules\ov5640_capture\constraints\ov5640_capture.sdc：
//     "Measured configuration: about 52.5 MHz DVP PCLK from the module's 24 MHz oscillator."
//   ★ 不加这条会怎样（实测）：Numbers of Setup Violated Endpoints 4 → 77、
//     Hold Violated 13 → 2429、SETUP 最差 +0.032 → −3.028 ns。
//     这些几乎全是"未约束时钟"导致的假违规 —— 工具会警告
//     `TA1132: 'camera_pclk' was determined to be a clock but was not created`，
//     然后用默认周期去算跨 PCLK 域路径，数字不可信。
//   ★ camera_pclk 与 50 MHz 系统时钟、49 MHz 像素时钟都是**异步**的；
//     跨域由 rtl\cam\cam_overlay.v 里的 DDR 读写 FIFO 完成，这里只声明时钟本身，
//     不做跨域对齐（与模块示例的做法一致）。
create_clock -name camera_pclk -period 19.048 [get_ports {camera_pclk}]

// ---- 跨时钟域（CDC）：sys 50 MHz ↔ pixel 49 MHz ----
// 本设计的跨域只走两种标准结构（见 rtl/top/falcons_top.v）：
//   ① 状态位/慢变参数（页面、模式、运行、急停、信号有效、菜单高亮）→ sys 域先打一拍，
//      再经 pixel 域两级同步器（源是纯 DFF 输出，组合深度归零）
//   ② 复位释放 → pixel 域内自产（等 PLL 锁 + 计数器释放，sys_rst_n 不进 pixel 域）
//
// ⚠ 以下是【历史实测记录，行号/实例名已随 RTL 演进变化，仅供追溯】：
//   实测过两条 CDC 负 slack：
//     SETUP −0.616 ns（复位路径，当时的实例名 pixel_rst_sync_*；该实例**已删除**，
//                      改为 falcons_top.v 里的域内计数器 pixel_pwr_cnt）
//     HOLD  −0.830 ns（u_rc/o_mode → mode_ps1；现已改为先经 mode_s 打一拍）
//   两处都通过在 RTL 上消除（域内自产复位 + 跨域源先寄存），而不是靠约束掩盖。
//   当前实测余量（**2026-10-16 20:51 综合**）见 doc\上板指南_v1.md 第六节：
//     SETUP   0 条（最差 +0.032 ns）/ REMOVAL 0 条（+0.269 ns）
//     HOLD   13 条（最差 −1.026 ns，sys→pixel 同步器第一级，**结构性伪违规**）
//     RECOVERY 8 条（最差 −5.983 ns，复位释放进 **245 MHz 串行域**）
//     ★ SETUP 另有 **4 个下降沿违规端点**（= 报告头部 Numbers of Setup Violated Endpoints
//       = Numbers of Falling Endpoints）—— 即 4 个由 245 MHz CLKOUT1 **下降沿**捕获的
//       u_hdmi/u_dvi_tx/rgb2dvi_inst/u_OSER10_{clk,b,r,g}/RESET，属**结构性伪违规**；
//       上升沿数据路径 0 条违规。（口径 2026-10-16 21:18 查清，见 doc\工作记录.md §21.9）
//   ★ RECOVERY 已按方案 A 改善：`u_hdmi.reset_n` 由 `sys_rst_n` 改接**域内自产**的 `pixel_rst_n`，
//     违规条数 **25 → 8**；剩下的 8 条源自"49 MHz 与 245 MHz 同源、相位固定"造成的**确定**偏移
//     （已超过一个 245 MHz 周期 → 物理上更接近"复位晚一拍释放"，见上板指南第六节的说明）。
//
//   【将来若要把跨域写成时序例外，用这两个真实时钟名】（从布线后报告里抄出来的，可直接引用）：
//     CLKOUT0（49 MHz 像素域）：u_hdmi/u_display_pll/PLL_inst/CLKOUT0.default_gen_clk
//     CLKOUT1（245 MHz 串行域）：u_hdmi/u_display_pll/PLL_inst/CLKOUT1.default_gen_clk
//   即：set_clock_groups -asynchronous -group [get_clocks {i_clk_50m}] \
//                                        -group [get_clocks {<上面两个名>}]
//   ⚠ 注意：这会消掉 HOLD 的 13 条噪音，**但不会消掉 RECOVERY 那 8 条**
//     —— 复位释放是真实功能路径，不能靠"声明为异步"了事。
// ⚠ 这里【不能】写 set_false_path -to [get_clocks {pixel_clk}]：
//   pixel_clk 是 hdmi_1024x600 内部 PLL 的派生时钟，在约束解析阶段还不是时钟对象，
//   写了会直接报 ERROR (TA2004): Cannot get clock with name 'pixel_clk'（综合即失败）。
//   若要做 CDC 约束，须改用布线后 report_clocks 得到的派生时钟实际名，
//   写 set_clock_groups -asynchronous（属待办项，见 doc\设计落实待办.md）。

// ============================================================================
// 跨时钟域分组（2026-10-20 加，兑现上面那两条待办）
//
// 本设计有四个时钟：
//   i_clk_50m      50.000 MHz  板载晶振（sys 域，控制链路全程在此域）
//   camera_pclk    52.499 MHz  摄像头 DVP PCLK
//   CLKOUT0        49.000 MHz  HDMI 像素域（u_hdmi 内部 PLL）
//   CLKOUT1       245.000 MHz  HDMI 串行域（同一 PLL 的 5 倍频）
//
// 前三者互不同源，是真正的异步关系。指向像素域的所有跨域数据都经
// falcons_top.v 里的两级同步器（page / menu_sel / tune_p / tune_active /
// pt_px / pt_py …），所以这里声明 asynchronous 是**如实描述**，不是掩盖问题。
//
// ★ CLKOUT0 与 CLKOUT1 必须放在**同一组**：它们同源、相位固定（5:1），
//   放不同组等于告诉工具"它们异步"，那是错的。
//   代价：`pixel_pwr_cnt[3]`（= pixel_rst_n）到 u_OSER10_*/RESET 的那 8 条
//   SETUP 仍会被分析并留在报告里 —— 它们是复位电平到串行器，不是数据路径，
//   属结构性伪违规（见上方 :47-50 的历史记录），本次不动它们。
//
// ★ 只写 set_clock_groups，不写 set_false_path：pixel_clk 在约束解析阶段
//   不是时钟对象，写了会 ERROR (TA2004)（见 :62-64）。
//
// ⚠⚠ 2026-10-20 实测失败 —— 这条命令**当前不能用**，保留在这里作为记录：
//   set_clock_groups -asynchronous -group [get_clocks {i_clk_50m}] \
//       -group [get_clocks {camera_pclk}] \
//       -group [get_clocks {u_hdmi/u_display_pll/PLL_inst/CLKOUT0.default_gen_clk
//                           u_hdmi/u_display_pll/PLL_inst/CLKOUT1.default_gen_clk}]
//   综合在 PnR 读约束时直接失败：
//     ERROR (TA2004) : "constr\falcons.sdc":92 |
//       Cannot get clock with name 'u_hdmi/u_display_pll/PLL_inst/CLKOUT0.default_gen_clk'
//     ERROR (TA2004) : ... | Cannot get clock with name '...CLKOUT1.default_gen_clk'
//     BUILD FAILED (run all): [exit code: 1]
//   原因：这两个名字是从**布线后**时序报告（Clock Summary / Timing Details）里抄来的，
//   那里 PLL 的 generated clock 已经建立；而 SDC 是在 read_netlist 之后、PnR 之前解析的，
//   此刻 generated clock 还不在时钟库里，`get_clocks` 取不到 → 直接终止。
//   与 :62-64 记录的 `pixel_clk` 是同一类问题的不同表现。
//   若要继续做这项，需要用 PnR 阶段确实可见的时钟对象名；待查的方法：
//     ① 在 SDC 里临时写 `report_clocks`，从 PnR 日志读出该阶段的实际时钟名；
//     ② 试 `get_clocks -include_generated_clocks {i_clk_50m}`（但那样会把 pixel/245M
//        一起归进 i_clk_50m 组，等于声明它们**同步**，方向反了，不可用）；
//     ③ 改用 `set_false_path -from/-to` 配合原始时钟，同样受"generated clock 不可见"限制。
//   在此之前，跨域违例继续按"人工按 Launch Clk == Latch Clk 过滤"的方式判读。
// ============================================================================

// ============================================================================
// 跨时钟域分组 v2（2026-10-21 重做）—— 用 create_generated_clock 自己命名
//
// 上一版（见上方 :90-109）失败的原因：直接引用 PnR 报告里的
// `...CLKOUT0.default_gen_clk`，而该名字是 PnR 阶段工具自动生成的，
// SDC 解析阶段（read_netlist 之后、PnR 之前）时钟库里没有它 → TA2004。
//
// 本版改为**自己用 create_generated_clock 命名**（SUG940 §5.2.2）：
//   * -source 选 PLL 的输入端口 i_clk_50m（两个 PLL 的 clkin 都接它）
//   * -multiply_by / -divide_by 按各 IP 的 defparam 折算：
//       display_pll: FCLKIN=50, IDIV=2, MDIV=49, ODIV0=25, ODIV1=5
//                    → CLKOUT0 = 50*49/2/25 = 49 MHz   (×49/÷50)
//                    → CLKOUT1 = 50*49/2/5  = 245 MHz  (×49/÷10)
//       gowin_pll  : FCLKIN=50, IDIV=1, MDIV=16, ODIV0=4,  ODIV1=16
//                    → CLKOUT0 = 50*16/1/4  = 200 MHz  (×4)
//                    → CLKOUT1 = 50*16/1/16 = 50 MHz   (×1)
//   * PLL 原语的引脚在综合网表里存在（`PLL PLL_inst`），get_pins 可解析
//
// 分四组：sys / 摄像头 PCLK / HDMI（像素+串行必须同组，5:1 同相）/
//         DDR（200M + 50M + PHY 内部 fclkdiv）
//
// ⚠ 若本版仍报 TA2004，按以下顺序逐个剥离，剥到能过为止：
//      ① 删 clk_ddrf 那行（DDR3 IP 内部路径最深，最可能解析不到）
//      ② 删 set_clock_groups 里的 `clk_ddrf` 与 `clk_cam200 clk_cam50` 两组
//      ③ 整段删掉，回到基线
// ============================================================================

// ---- HDMI PLL（u_hdmi 内部，clkin = i_clk_50m）----
create_generated_clock -name clk_pixel -source [get_ports {i_clk_50m}] -multiply_by 49 -divide_by 50 [get_pins {u_hdmi/u_display_pll/PLL_inst/CLKOUT0}]
create_generated_clock -name clk_serial -source [get_ports {i_clk_50m}] -multiply_by 49 -divide_by 10 [get_pins {u_hdmi/u_display_pll/PLL_inst/CLKOUT1}]

// ---- 摄像头/DDR PLL（u_cam 内部，clkin = i_clk_50m）----
create_generated_clock -name clk_cam200 -source [get_ports {i_clk_50m}] -multiply_by 4 [get_pins {u_cam/u_system_pll/PLL_inst/CLKOUT0}]
create_generated_clock -name clk_cam50 -source [get_ports {i_clk_50m}] -multiply_by 1 -divide_by 1 [get_pins {u_cam/u_system_pll/PLL_inst/CLKOUT1}]

// ---- DDR3 PHY 内部时钟分频器（200 MHz ÷ 4 = 50 MHz）----
create_generated_clock -name clk_ddrf -source [get_pins {u_cam/u_system_pll/PLL_inst/CLKOUT0}] -divide_by 4 [get_pins {u_cam/u_frame_buffer/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]

// ---- 四组异步 ----
// New AI FIFO Gray crossings must retain a physical propagation bound.
// Clock groups override max_delay in Gowin, so DDR is no longer grouped
// away from sys/pixel. Those CDCs use <=10ns propagation and no phase hold.
set_clock_groups -asynchronous -group [get_clocks {i_clk_50m}] -group [get_clocks {camera_pclk}] -group [get_clocks {clk_pixel clk_serial}]
set_clock_groups -asynchronous -group [get_clocks {camera_pclk}] -group [get_clocks {clk_cam200 clk_cam50 clk_ddrf}]
set_max_delay -from [get_clocks {i_clk_50m}] -to [get_clocks {clk_cam200 clk_cam50 clk_ddrf}] 10.000
set_max_delay -from [get_clocks {clk_cam200 clk_cam50 clk_ddrf}] -to [get_clocks {i_clk_50m}] 10.000
set_max_delay -from [get_clocks {clk_pixel clk_serial}] -to [get_clocks {clk_cam200 clk_cam50 clk_ddrf}] 10.000
set_max_delay -from [get_clocks {clk_cam200 clk_cam50 clk_ddrf}] -to [get_clocks {clk_pixel clk_serial}] 10.000
set_false_path -hold -from [get_clocks {i_clk_50m}] -to [get_clocks {clk_cam200 clk_cam50 clk_ddrf}]
set_false_path -hold -from [get_clocks {clk_cam200 clk_cam50 clk_ddrf}] -to [get_clocks {i_clk_50m}]
set_false_path -hold -from [get_clocks {clk_pixel clk_serial}] -to [get_clocks {clk_cam200 clk_cam50 clk_ddrf}]
set_false_path -hold -from [get_clocks {clk_cam200 clk_cam50 clk_ddrf}] -to [get_clocks {clk_pixel clk_serial}]

// ---- AI inspection FX2 interface (2026-10-26) ----
// Reviewed volatile vendor firmware sets IFCONFIG=A3: internal 30MHz IFCLK.
// USB high-speed 480Mbps is independent of this clock. Runtime diagnostics
// measure and require 30MHz before admitting any high-speed business data.
create_clock -name clk_fx2 -period 33.300 [get_ports {i_fx2_ifclk}]
// Infineon 38-08032 Rev.AD tables20/23/26/31; add 1.5ns board margin.
// Keep the conservative 15ns input budget; unknown minimum chip delay is
// modeled as zero with -1.5ns board skew. Falling-edge input registers avoid
// capturing data/flags during the FX2 rising-edge update window.
set_input_delay -clock [get_clocks {clk_fx2}] -max 15.000 [get_ports {io_fx2_fd[*] i_fx2_flagb i_fx2_flagc}]
set_input_delay -clock [get_clocks {clk_fx2}] -min -1.500 [get_ports {io_fx2_fd[*] i_fx2_flagb i_fx2_flagc}]
set_output_delay -clock [get_clocks {clk_fx2}] -max 20.200 [get_ports {o_fx2_slrd_n}]
set_output_delay -clock [get_clocks {clk_fx2}] -max 11.900 [get_ports {o_fx2_slwr_n}]
set_output_delay -clock [get_clocks {clk_fx2}] -max 10.700 [get_ports {io_fx2_fd[*]}]
set_output_delay -clock [get_clocks {clk_fx2}] -max 16.100 [get_ports {o_fx2_pktend_n}]
set_output_delay -clock [get_clocks {clk_fx2}] -max 12.000 [get_ports {o_fx2_sloe_n}]
set_output_delay -clock [get_clocks {clk_fx2}] -min -2.000 [get_ports {io_fx2_fd[*] o_fx2_slrd_n o_fx2_slwr_n o_fx2_sloe_n o_fx2_pktend_n}]
// Address/select are stable >=2 cycles before AND after a physical access,
// enforced by IDLE/SETUP/DO/TURN states and temporal simulation assertions.
// The hold exception refers to the previous qualified access, never a free
// running edge on which neither SLRD nor SLWR nor PKTEND is asserted.
set_output_delay -clock [get_clocks {clk_fx2}] -max 26.500 [get_ports {o_fx2_faddr[*] o_fx2_slcs_n}]
set_output_delay -clock [get_clocks {clk_fx2}] -min -11.500 [get_ports {o_fx2_faddr[*] o_fx2_slcs_n}]
set_multicycle_path -setup -to [get_ports {o_fx2_faddr[*] o_fx2_slcs_n}] 2
set_multicycle_path -hold -to [get_ports {o_fx2_faddr[*] o_fx2_slcs_n}] 2
// No clock groups over the new Gray crossings: SUG940 gives groups higher
// priority than max_delay. Bound propagation below either source period;
// unrelated-clock hold phase is excluded, setup remains constrained.
set_max_delay -from [get_clocks {clk_fx2}] -to [get_clocks {i_clk_50m}] 10.000
set_max_delay -from [get_clocks {i_clk_50m}] -to [get_clocks {clk_fx2}] 10.000
set_false_path -hold -from [get_clocks {clk_fx2}] -to [get_clocks {i_clk_50m}]
set_false_path -hold -from [get_clocks {i_clk_50m}] -to [get_clocks {clk_fx2}]
