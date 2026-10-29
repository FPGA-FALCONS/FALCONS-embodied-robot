//////////////////////////////////////////////////////////////////////////////////
// 模块：motor_unit —— 单路电机完整控制单元（驱动 + 解码 + 测速 + 堵转保护）
//
// 【为什么有这个封装层】
//   实际使用中这四件事几乎总是【一起出现】：
//       给功率(motor_pwm) → 读编码器(quad_decoder) → 算转速(speed_measure)
//       → 判断有没有卡住(stall_detect)
//   若每次都在顶层手动拼这 4 个模块，跨模块的信号（delta / delta_valid /
//   使能通路）都要自己接，而且接错不会报错——本工程踩过的
//   "o_data_valid 与 o_acc_valid 接错"就是这类问题。
//   本模块把它们封成一个"开箱即用"的单路单元：
//       * 使用者只需例化一次，4 路就是例化 4 次
//       * 内部连接写死，并由集成测试覆盖
//
//   ★ 这是"分层"而不是"合并"：4 个原子模块本身仍然独立保留，可以单独复用、
//     单独测试、单独替换：
//         换驱动芯片（DRV8833/A4950）  → 只改 motor_pwm
//         换编码器类型                 → 只改 quad_decoder
//     需要下探细节时随时可以，分层不牺牲灵活性。
//
// 【内部结构】
//   i_enc_a/b ─→ quad_decoder ─→ o_count / o_enc_dir
//                     │
//                     └─→ speed_measure ─→ o_delta / o_delta_valid
//                                │
//   i_duty ──────────────────────┴─→ stall_detect ─→ o_stall
//   i_enable ───────────────────────────┘   │
//                                    (保护后的使能)
//                                           ↓
//   i_dir / i_brake ─────────────────→ motor_pwm ─→ o_in1 / o_in2 / o_pwm
//
// 【接口契约】（使用者需要知道的全部）
//   * i_duty 满值 = 2^PWM_BITS，即占空比 = i_duty / 2^PWM_BITS
//   * o_count 单位 = quad_decoder 的 4 倍频计数
//   * o_delta 单位 = 每 WINDOW_MS 毫秒的计数增量
//       转速(rpm) = o_delta / 10   ← 500PPR(电机轴) + 减速比 30 时的标定值
//   * 堵转时 o_stall 置位【并锁存】，此期间电机输出已被自动切断；
//     必须给 i_stall_clear 一个脉冲才恢复（详见 stall_detect.v）
//   * i_enable = 0 时电机停止；停止方式由 i_brake 决定（0 滑行 / 1 刹车）
//
// 参数：全部直接透传给对应子模块，各参数含义见子模块文件头
//   CLK_FREQ_HZ / PWM_BITS / FILTER_BITS / WINDOW_MS / INV_DIR
//   STALL_STARTUP_WINDOWS / STALL_WINDOWS / STALL_DUTY_TH / STALL_MOVE_TH
//
// 端口：
//   i_clock      输入  系统时钟
//   i_reset_n    输入  异步低电平复位
//   i_enable     输入  1 = 允许驱动
//   i_dir        输入  0 = 正转，1 = 反转
//   i_brake      输入  停止方式：0 = 滑行，1 = 刹车
//   i_duty       输入  占空比，0 ~ 2^PWM_BITS-1
//   i_stall_clear 输入 清除堵转锁存（单周期脉冲）
//   o_in1/o_in2/o_pwm 输出 TB6612 三线
//   i_enc_a/i_enc_b 输入 正交编码器 A/B 相
//   i_count_clear 输入 位置计数清零（单周期脉冲）
//   o_count      输出  位置计数（有符号）
//   o_delta      输出  窗口转速增量（有符号）
//   o_delta_valid 输出 窗口结束脉冲
//   o_enc_dir    输出  最近运动方向：1 = 计数递增
//   o_stall      输出  堵转标志（锁存）
//////////////////////////////////////////////////////////////////////////////////

module motor_unit #(
    parameter integer CLK_FREQ_HZ   = 50_000_000,
    parameter integer PWM_BITS      = 11,
    parameter integer FILTER_BITS   = 3,
    parameter integer WINDOW_MS     = 10,
    // ---- 两个"极性修正"参数，作用不同、可以组合使用 ----
    //   INV_DIR     : 反【电机输出方向】
    //                    用于"该路电机线/IN 线接反"→ 给 dir=0 却反转
    //   ENC_INV_DIR : 反【编码器计数方向】
    //                    用于"电机转向对、但计数符号与其它路相反"
    //                    （典型场景：左右电机镜像安装、编码器接线约定相同，
    //                      于是同样"向前"却得到相反符号）
    //   ★ 两者不可互相替代：反电机只解决电机不转对方向的问题；反编码器只解决
    //     计数符号不统一的问题。同时反两者 = 回到原状。
    parameter integer INV_DIR     = 0,
    parameter integer ENC_INV_DIR = 0,
    parameter integer STALL_STARTUP_WINDOWS = 30,
    parameter integer STALL_WINDOWS         = 20,
    parameter integer STALL_DUTY_TH         = 200,
    parameter integer STALL_MOVE_TH         = 300
)(
    input  wire                    i_clock,
    input  wire                    i_reset_n,

    // ---- 控制 ----
    input  wire                    i_enable,
    input  wire                    i_dir,
    input  wire                    i_brake,
    input  wire [PWM_BITS-1:0]     i_duty,
    input  wire                    i_stall_clear,

    // ---- TB6612 接口 ----
    output wire                    o_in1,
    output wire                    o_in2,
    output wire                    o_pwm,

    // ---- 编码器接口 ----
    input  wire                    i_enc_a,
    input  wire                    i_enc_b,
    input  wire                    i_count_clear,

    // ---- 采集与状态 ----
    output wire signed [31:0]      o_count,
    output wire signed [31:0]      o_delta,
    output wire                    o_delta_valid,
    output wire                    o_enc_dir,
    output wire                    o_stall
);

    //---------------------------------------------------------------
    // 内部连线
    //---------------------------------------------------------------
    wire signed [31:0] count_w;
    wire signed [31:0] delta_w;
    wire               delta_valid_w;
    wire               enable_out_w;   // 经堵转保护后的实际使能

    // 占空比零扩展到 16 位供堵转检测做阈值比较（要求 PWM_BITS <= 16）
    wire [15:0]        duty16_w = {{(16-PWM_BITS){1'b0}}, i_duty};

    //---------------------------------------------------------------
    // 1. 正交解码 → 位置计数
    //---------------------------------------------------------------
    quad_decoder #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .COUNT_BITS  (32),
        .FILTER_BITS (FILTER_BITS),
        .INV_DIR  (ENC_INV_DIR)   // 编码器计数方向修正（见参数说明）
    ) u_enc (
        .i_clock   (i_clock),
        .i_reset_n (i_reset_n),
        .i_enc_a   (i_enc_a),
        .i_enc_b   (i_enc_b),
        .i_clear   (i_count_clear),
        .o_count   (count_w),
        .o_dir     (o_enc_dir),
        .o_step    ()
    );

    //---------------------------------------------------------------
    // 2. 定窗口测速 → 转速增量
    //---------------------------------------------------------------
    speed_measure #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .WINDOW_MS   (WINDOW_MS),
        .COUNT_BITS  (32)
    ) u_speed (
        .i_clock       (i_clock),
        .i_reset_n     (i_reset_n),
        .i_count       (count_w),
        .i_clear       (i_count_clear),
        .o_delta       (delta_w),
        .o_delta_valid (delta_valid_w)
    );

    //---------------------------------------------------------------
    // 3. 堵转检测（插在使能通路上）
    //    ★ 它的输入必须来自 speed_measure 的输出——这正是本封装要保证的
    //      内部契约：以前在顶层手拼时，这两根线接错不会报错，只会"保护不工作"。
    //---------------------------------------------------------------
    stall_detect #(
        .STARTUP_WINDOWS (STALL_STARTUP_WINDOWS),
        .STALL_WINDOWS   (STALL_WINDOWS),
        .DUTY_TH         (STALL_DUTY_TH),
        .MOVE_TH         (STALL_MOVE_TH),
        .PIPE_MONITOR_INPUT(1)
    ) u_stall (
        .i_clock       (i_clock),
        .i_reset_n     (i_reset_n),
        .i_enable      (i_enable),
        .i_duty        (duty16_w),
        .i_delta       (delta_w),
        .i_delta_valid (delta_valid_w),
        .i_stall_clear (i_stall_clear),
        .o_stall       (o_stall),
        .o_enable_out  (enable_out_w)
    );

    //---------------------------------------------------------------
    // 4. 单路 PWM 驱动
    //    使能用的是【堵转保护后的】enable_out_w，而不是直接 i_enable
    //---------------------------------------------------------------
    motor_pwm #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_BITS    (PWM_BITS),
        .INV_DIR  (INV_DIR)
    ) u_pwm (
        .i_clock   (i_clock),
        .i_reset_n (i_reset_n),
        .i_enable  (enable_out_w),
        .i_dir     (i_dir),
        .i_brake   (i_brake),
        .i_duty    (i_duty),
        .o_in1     (o_in1),
        .o_in2     (o_in2),
        .o_pwm     (o_pwm),
        .o_active  ()
    );

    //---------------------------------------------------------------
    // 输出
    //---------------------------------------------------------------
    assign o_count       = count_w;
    assign o_delta       = delta_w;
    assign o_delta_valid = delta_valid_w;

endmodule
