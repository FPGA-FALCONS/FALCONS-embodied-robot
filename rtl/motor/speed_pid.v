//////////////////////////////////////////////////////////////////////////////////
// 模块：speed_pid —— 单路【增量式】PID 速度环控制器（全定点、零浮点、零除法）
//
// 【为什么用增量式而不是位置式】
//   位置式 PID 需要保存误差累加器 Σe，输出限幅后累加器继续增长 →
//   典型的积分饱和：电机堵转/到限幅后，误差长期存在，一松手输出要"退饱和"
//   很久。增量式只对 Δu 积分（u 本身），输出被钳位时不会积累多余的积分量，
//   反号时立刻退出饱和，天然抗积分饱和 —— 对"上板手动调参"这种场景特别友好。
//
// 【算法】
//   e(k)   = setpoint − feedback          （误差，带限幅）
//   Δu(k)  = Kp·[e(k)−e(k−1)] + Ki·e(k) + Kd·[e(k)−2e(k−1)+e(k−2)]
//   u(k)   = clamp( u(k−1) + Δu(k), −U_LIMIT, +U_LIMIT )
//
//   注意增量式里 Ki 的作用与位置式不同：它作用在 Δu 上（每拍加 Ki·e），
//   所以 Ki 的合理取值范围比位置式更小，见下面参数表。
//
// 【定点格式】全部 Q8（256 = 1.0）
//   Kp/Ki/Kd 为 int16 的 Q8 值：0.05 → 13，0.1 → 26，1.0 → 256
//   Δu 在内部用 40 位累加后算术右移 8 位得到整数增量
//
// 【更新节拍】由上游给的 i_fb_valid 脉冲触发（本工程 = 测速窗口结束，100 Hz）。
//   不是自由跑的自激节拍 —— 必须与"新反馈数据到达"严格对齐，否则会重复使用
//   同一个反馈值算多次 Δu，等效于把 Ki 放大好几倍。
//
// 【位宽安全性】误差先限幅到 ±ERR_LIMIT，保证中间乘积不溢出：
//   |Kp·Δe| ≤ 32767×8190 ≈ 2.7e8，三项之和 < 2^31，40 位中间量留足余量。
//
// 参数：
//   PWM_BITS   输出限幅上限 2^PWM_BITS−1（与 motor_pwm 的占空比满值一致）
//   U_LIMIT    输出限幅（默认 = 2^PWM_BITS−1，可调小以限制最大占空比）
//   ERR_LIMIT  误差限幅（默认 4095；目标与反馈单位是"计数/窗口"）
//   KP_INIT/KI_INIT/KD_INIT  预设参数（Q8），仅作默认值，运行时由端口覆盖
//
// 端口：
//   i_clock / i_reset_n    系统时钟 / 异步低电平复位
//   i_enable               1 = 闭环工作；0 = 输出 0 且内部状态复位（安全态）
//   i_clear                单周期脉冲：u 与误差历史清零（重新起调）
//   i_setpoint             目标值（有符号，单位 = 计数/测速窗口）
//   i_feedback             反馈值（有符号，同单位）
//   i_fb_valid             单周期脉冲：本拍反馈是新值，执行一次 PID 更新
//   i_kp / i_ki / i_kd     Q8 定点参数（有符号，运行时可变 → 支持串口在线调参）
//   o_u                    输出（有符号）：正 = 正转方向，负 = 反转方向；
//                          绝对值为占空比，满值 2^PWM_BITS
//   o_err                  当前误差（等于 e(k)，便于上报观察）
//   o_sat                  饱和标志：本拍输出被限幅
//////////////////////////////////////////////////////////////////////////////////

module speed_pid #(
    parameter integer PWM_BITS = 11,
    parameter integer U_LIMIT  = (1 << PWM_BITS) - 1,
    parameter integer ERR_LIMIT = 4095,
    // ---- 预设参数（Q8 定点，256 = 1.0）----
    // 增量式 PID 的 Δu 单位是"占空比/窗口"，本车 1 rpm = 10 计数/窗口，
    // duty 满值 2048。以下预设是一组【保守】起点（先小后大，宁可慢不可震荡）：
    //   Kp = 26 (0.10)：误差 100 计数 → Δu ≈ 10（约 0.5% 占空比/窗口）
    //   Ki = 3  (0.012)：稳态误差靠它慢慢磨掉
    //   Kd = 0          ：微分先关掉（编码器量化噪声会被 Kd 放大）
    parameter integer KP_INIT = 26,
    parameter integer KI_INIT = 3,
    parameter integer KD_INIT = 0
)(
    input  wire        i_clock,
    input  wire        i_reset_n,

    input  wire        i_enable,
    input  wire        i_clear,

    input  wire signed [31:0] i_setpoint,
    input  wire signed [31:0] i_feedback,
    input  wire        i_fb_valid,

    input  wire signed [15:0] i_kp,
    input  wire signed [15:0] i_ki,
    input  wire signed [15:0] i_kd,

    output reg  signed [31:0] o_u,
    output reg  signed [31:0] o_err,
    output reg                o_sat
);

    localparam signed [15:0] ERR_MAX = ERR_LIMIT;
    localparam signed [31:0] U_MAX   = U_LIMIT;

    //---------------------------------------------------------------
    // 误差计算与限幅
    //---------------------------------------------------------------
    wire signed [32:0] err_raw = i_setpoint - i_feedback;

    wire signed [15:0] err_lim =
        (err_raw >  ERR_MAX) ?  ERR_MAX :
        (err_raw < -ERR_MAX) ? -ERR_MAX :
                                err_raw[15:0];

    //---------------------------------------------------------------
    // 误差历史
    //---------------------------------------------------------------
    reg signed [15:0] e1, e2;

    //---------------------------------------------------------------
    // 增量计算（组合逻辑，一个时钟沿打入）
    //   d1 = e(k) - e(k-1)
    //   d2 = e(k) - 2e(k-1) + e(k-2)
    //---------------------------------------------------------------
    wire signed [16:0] d1 = err_lim - e1;
    wire signed [17:0] d2 = err_lim - (e1 <<< 1) + e2;

    // 8 位小数（Q8）定点乘加：总和右移 8 位即得整数增量
    wire signed [39:0] du_sum = (i_kp * d1) + (i_ki * err_lim) + (i_kd * d2);
    wire signed [39:0] du     = du_sum >>> 8;

    //---------------------------------------------------------------
    // 输出更新（钳位即抗饱和：增量式没有独立积分累加器，u 被钳住后
    // 一旦误差反号，下一个节拍立刻往回走，不会"退饱和延迟"）
    //---------------------------------------------------------------
    wire signed [31:0] du32 = du[31:0];
    wire signed [32:0] u_next_raw = o_u + du32;

    wire signed [31:0] u_next =
        (u_next_raw >  U_MAX) ?  U_MAX :
        (u_next_raw < -U_MAX) ? -U_MAX :
                                 u_next_raw[31:0];

    wire sat_now = (u_next_raw > U_MAX) || (u_next_raw < -U_MAX);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            o_u   <= 32'sd0;
            o_err <= 32'sd0;
            o_sat <= 1'b0;
            e1    <= 16'sd0;
            e2    <= 16'sd0;
        end else if (!i_enable) begin
            // 安全态：输出 0 且状态复位（下次使能从零开始，不会残留旧 u）
            o_u   <= 32'sd0;
            o_err <= 32'sd0;
            o_sat <= 1'b0;
            e1    <= 16'sd0;
            e2    <= 16'sd0;
        end else if (i_clear) begin
            o_u   <= 32'sd0;
            o_err <= err_lim;
            o_sat <= 1'b0;
            e1    <= 16'sd0;
            e2    <= 16'sd0;
        end else if (i_fb_valid) begin
            o_u   <= u_next;
            o_err <= err_lim;
            o_sat <= sat_now;
            e2    <= e1;
            e1    <= err_lim;
        end
    end

endmodule
