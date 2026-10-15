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
//   ① 状态位/慢变参数（页面、模式、运行、急停、信号有效）→ pixel 域两级同步器
//   ② 复位释放 → pixel 域内同步
// 这些路径在功能上不携带需要"按时钟节拍到达"的数据，因此声明为异步。
// 不声明时的实测现象：工具把同步器第一级按数据路径分析，报出
//   SETUP −0.616 ns（u_reset_sync/key_stable_s0 → pixel_rst_sync_0_s0）
//   HOLD  −0.830 ns（u_rc/o_mode_0_s0 → mode_ps1_0_s0）
// 这两条都出现在 CDC 上，属结构性伪违规。
// ⚠ 这里【不能】写 set_false_path -to [get_clocks {pixel_clk}]：
//   pixel_clk 是 hdmi_1024x600 内部 PLL 的派生时钟，在约束解析阶段还不是时钟对象，
//   写了会直接报 ERROR (TA2004): Cannot get clock with name 'pixel_clk'（综合即失败）。
// 因此跨域路径的时序问题改为【在 RTL 上消除】，而不是靠约束掩盖：
//   * 复位：像素域复位在域内自产（等 PLL 锁 + 计数释放），sys_rst_n 不进 pixel 域复位端；
//   * 状态位：sys→pixel 用两级同步器（这是标准 CDC，其第一级可能出现 HOLD 违规，
//     属同步器的固有结构性现象，不构成功能风险 —— 见 build 报告里的路径两端）。