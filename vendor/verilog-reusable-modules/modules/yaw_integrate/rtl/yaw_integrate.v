//////////////////////////////////////////////////////////////////////////////////
// 模块：yaw_integrate —— 用 Z 轴角速度积分出偏航角（纯定点，无浮点，无乘法器）
//
// 【工作原理】
//   角增量 = ω · dt。输入角速度单位是 0.01°/s，本模块自带积分时基
//   （默认 1024Hz），于是 dt = 1/1024 s = 2^-10 s 恰好是 2 的幂，
//   每个积分节拍的角增量 = gz × (0.01°/s) × 2^-10 s = gz / 1024 个 "0.01°"。
//
//   把累加器 acc 的单位定义为 u = 0.01°/1024，则：
//       每节拍增量 = (gz/1024 个 0.01°) × (1024 u / 0.01°) = gz 个 u
//   即【每节拍直接把 gz 累加进 acc】，零乘法器、零除法、零截断误差。
//   输出时取 acc >>> 10 即得单位 0.01° 的偏航角。
//
//   之所以能做到这一点，是因为 50MHz / 1024 = 48828.125，取分频系数
//   48828 得到实际节拍 1024.0026Hz（误差 2.6ppm，可忽略）。
//   若用 1000Hz 节拍，dt 不是 2 的幂，每节拍就得做 gz×65.536 这样的
//   定点乘法，既费 DSP 又引入截断误差——故本模块刻意选 1024Hz。
//
// 【与 imu_top 的配合：零阶保持（ZOH）】
//   本模块的积分时基与 imu_top 的采样节拍【完全解耦】：
//   i_sample_valid 只负责把最新样本锁存进 gz_hold，积分则按本模块自己的
//   等间隔节拍走。若 imu_top 以 1kHz 采样而传感器 ODR 只有 200Hz，则同一
//   样本会被读若干次——这在数值上等价于零阶保持积分（每个节拍用"最新可用
//   样本 × 节拍时长"），结果是正确的，只是多了半个 ODR 周期的相位滞后。
//   因此修改 imu_top 的 SAMPLE_DIV 不会破坏本模块的积分正确性。
//
// 【静止死区】
//   |gz| < DEADZONE_TH 时把角速度视为 0。零偏标定扣掉的是【常数偏置】，
//   但残余偏置与噪声仍在驱动积分做随机游走，静止死区能显著压住静止漂移。
//   代价：比阈值更慢的真实转动会被吞掉（阈值 0.3°/s 时，慢于 0.3°/s 的
//   匀速转动完全不产生角增量）。
//   注意本模块只做"归零"，不做"扣除阈值"，因为扣除会给慢速转动引入
//   恒定偏差，而归零的行为更容易预测和调试。
//
// 【输出回绕】
//   acc 每节拍折回 [-18000<<10, +18000<<10)，故 o_yaw 恒在 [-180, +180)
//   （单位 0.01°，即 [-18000, +18000)）。折回只需一次加减：
//   acc 折回后落在 ±18432000 内，单节拍最大增量是满量程 2000°/s 对应的
//   200000 u（≈195°），故 acc+增量 最坏 18632000，减一次 36000<<10 即回界。
//
// 【清零与冻结】
//   i_enable = 0 时 acc 被强制为 0 且不积分，解冻（回到 1）后从 0 开始积分。
//   典型用法是接"零偏标定已完成"，使 yaw 的零点正好对应"车静止、标定结束"
//   的那一刻。i_clear 是独立的单周期清零，优先级高于 i_enable。
//
// 【精度提示】
//   * 输出分辨率 0.01°（acc 内部精度为 0.01°/1024，累加过程不丢精度）
//   * 量程 ±180°，靠回绕实现，转满一圈后无法区分圈数
//   * 漂移主要来自残余零偏：若标定后残余 0.02°/s，则每分钟漂 1.2°
//
// 参数：
//   CLK_FREQ_HZ   系统时钟频率(Hz)     默认 50_000_000
//   INT_RATE_HZ   积分节拍频率(Hz)     默认 1024（务必取 2 的幂，见上）
//   DEADZONE_EN   1 = 启用静止死区     默认 1
//   DEADZONE_TH   死区阈值(0.01°/s)    默认 30 → 0.3°/s
//
// 端口：
//   i_clock         输入  系统时钟
//   i_reset_n       输入  异步低电平复位
//   i_gyro_z        输入  Z 轴角速度，0.01°/s（应为已扣零偏的值）
//   i_sample_valid  输入  样本更新脉冲（一周期宽），锁存 i_gyro_z
//   i_enable        输入  1 = 正常积分；0 = 冻结且 yaw 保持 0
//   i_clear         输入  单周期清零脉冲
//   o_yaw           输出  偏航角，0.01°，范围 [-18000, +18000)
//   o_yaw_valid     输出  跟随 i_enable（打一拍），指示 o_yaw 有效
//   o_deadzone_hit  输出  当前样本是否被死区归零（调阈值时观察用）
//   o_gz_eff        输出  死区处理后的角速度，0.01°/s（调试观察用）
//////////////////////////////////////////////////////////////////////////////////

module yaw_integrate #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer INT_RATE_HZ = 1024,
    parameter integer DEADZONE_EN = 1,
    parameter integer DEADZONE_TH = 30
)(
    input  wire               i_clock,
    input  wire               i_reset_n,

    input  wire signed [31:0] i_gyro_z,
    input  wire               i_sample_valid,

    input  wire               i_enable,
    input  wire               i_clear,

    output wire signed [31:0] o_yaw,
    output wire               o_yaw_valid,
    output wire               o_deadzone_hit,
    output wire signed [31:0] o_gz_eff
);

    //---------------------------------------------------------------
    // 积分时基：等间隔节拍，与采样节拍解耦
    //   INT_DIV 是常量折叠出的整数，综合无代价
    //---------------------------------------------------------------
    localparam integer INT_DIV = (CLK_FREQ_HZ / INT_RATE_HZ) - 1;

    reg  [31:0] tick_cnt;
    wire        int_tick = (tick_cnt == 32'd0);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)             tick_cnt <= 32'd0;
        else if (int_tick)          tick_cnt <= INT_DIV[31:0];
        else                        tick_cnt <= tick_cnt - 32'd1;
    end

    //---------------------------------------------------------------
    // 样本零阶保持（ZOH）
    //   gz_now 组合选择：采样与本拍积分同拍发生时直接用新样本，不延迟一拍
    //---------------------------------------------------------------
    reg  signed [31:0] gz_hold;
    wire signed [31:0] gz_now = i_sample_valid ? i_gyro_z : gz_hold;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)             gz_hold <= 32'sd0;
        else if (i_sample_valid)    gz_hold <= i_gyro_z;
    end

    //---------------------------------------------------------------
    // 静止死区
    //   gz_abs = |gz_now|；判据用严格小于，故 |gz| == 阈值 时不归零
    //---------------------------------------------------------------
    localparam signed [31:0] DZ_TH = DEADZONE_TH;

    wire signed [31:0] gz_abs = gz_now[31] ? (~gz_now + 32'sd1) : gz_now;
    wire               dz_hit = (DEADZONE_EN != 0) && (gz_abs < DZ_TH);
    wire signed [31:0] gz_eff = dz_hit ? 32'sd0 : gz_now;

    assign o_deadzone_hit = dz_hit;
    assign o_gz_eff       = gz_eff;

    //---------------------------------------------------------------
    // 积分 + 回绕
    //   acc 单位 u = 0.01°/1024；每节拍 acc += gz_eff（见文件头推导）
    //   加宽一位做中间和，避免工具报位宽截断，数学上界 18632000 « 2^31
    //---------------------------------------------------------------
    localparam signed [32:0] WRAP_LIM  = 33'sd18432000;   // 18000 << 10
    localparam signed [32:0] WRAP_SPAN = 33'sd36864000;   // 36000 << 10
    localparam signed [32:0] WRAP_NEG  = -33'sd18432000;

    reg  signed [31:0] acc;

    wire signed [32:0] acc_sum = {acc[31], acc} + {{1{gz_eff[31]}}, gz_eff};

    wire signed [32:0] acc_folded =
        (acc_sum >=  WRAP_LIM) ? (acc_sum - WRAP_SPAN) :
        (acc_sum <  WRAP_NEG)  ? (acc_sum + WRAP_SPAN) : acc_sum;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)          acc <= 32'sd0;
        else if (i_clear)        acc <= 32'sd0;      // 清零优先
        else if (!i_enable)      acc <= 32'sd0;      // 冻结期间保持零点
        else if (int_tick)       acc <= acc_folded[31:0];
    end

    //---------------------------------------------------------------
    // 输出
    //   acc ∈ [-18432000, +18432000) → o_yaw ∈ [-18000, +18000)
    //   o_yaw_valid 打一拍，避免组合信号在复位期间误报有效
    //---------------------------------------------------------------
    reg yaw_valid_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)          yaw_valid_r <= 1'b0;
        else                     yaw_valid_r <= i_enable;
    end

    assign o_yaw       = acc >>> 10;
    assign o_yaw_valid = yaw_valid_r;

endmodule
