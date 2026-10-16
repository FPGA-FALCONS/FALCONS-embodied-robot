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
//   当前实测余量（**2026-10-16 18:36 综合**）见 doc\上板指南_v1.md 第六节：
//     SETUP   0 条（最差 +0.092 ns）/ REMOVAL 0 条（+0.267 ns）
//     HOLD   13 条（最差 −1.026 ns，sys→pixel 同步器第一级，**结构性伪违规**）
//     RECOVERY 8 条（最差 −5.869 ns，复位释放进 **245 MHz 串行域**）
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