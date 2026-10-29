//////////////////////////////////////////////////////////////////////////////////
// 模块：chassis_ctrl —— 底盘运动控制（差速分解 + 四路独立速度闭环）
//
// 【它做什么】
//   1. 把「遥控/自动导航」给出的指令翻译成左右侧目标轮速：
//        v_left  = v_center − Δ
//        v_right = v_center + Δ          （Δ = 差速量，由转向量算出）
//      左侧两轮同速、右侧两轮同速 → 差速转向（依据功能设计文档 §6.4）
//   2. 每路跑一个独立的增量式 PID（`speed_pid`），反馈 = 编码器窗口增量
//   3. 把 PID 的有符号输出翻译成 `motor_top` 需要的 (i_dir, i_duty)：
//        ★ motor_pwm 的语义是 i_dir = 0 → 正转，1 → 反转（与 o_u 的正负相反）
//        duty = |u|（PID 内部已把 u 限幅到 ±U_LIMIT = ±(2^PWM_BITS−1)）
//   4. 处理优先级与安全：
//        急停（ToF / 按键）> 堵转锁存 > 手动接管 > 自动导航 > 停止
//      急停 → 全部短路刹车；堵转 → 全部滑行停机（不继续给功率，避免持续发热）
//
// 【单位与定点】全部 Q10 常量，零除法、零浮点
//   内部速度单位 = 编码器**计数/测速窗口**（窗口 10 ms = 100 Hz，与
//   `speed_measure` 一致）。换算关系（依据 doc\参数总表与待定清单.md）：
//       1 rpm = 10.24 计数/10ms      → delta = rpm × 10.24
//       512 线 × 4 倍频 × 减速比 30 = 61440 计数/车轮转
//   遥控量 → 内部单位：
//       throttle(±1000) → 中心速度 delta：系数 3.0723  (Q10 = 3146)
//           依据：中心 300 rpm ↔ 满舵；300 × 10.24 / 1000 = 3.072
//       steer(±1000)    → 差速 delta：系数 0.6123    (Q10 = 627)
//           依据：差速系数 0.66415 rpm/(deg/s) × 最大角速度 90 deg/s
//                 × 10.24 / 1000 = 0.612  （原地转满舵 → 单轮 300×0.612/3.072 ≈ 59.8 rpm）
//
// 【自动导航预留】
//   顶层把 `i_auto_valid` 拉低即可让底盘忽略导航给定；将来避障/路径规划模块
//   只需驱动 `i_auto_center` / `i_auto_diff`（同一内部单位），本模块无需改动。
//   ★ 2026-10-16 修正：`i_auto_valid = 0` 现在**同时**把 `i_auto_center` 与
//     `i_auto_diff` 按 0 处理 —— 原先只门控了差速量（diff_raw），中心速度
//     （center_raw）却直接取 i_auto_center；自动导航未接入时该端口若残留非 0，
//     车就会被一个不该生效的中心速度推着走。两个量必须同门控。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module chassis_ctrl #(
    parameter integer PWM_BITS        = 11,
    // ---- 运动学（改车体参数只需改这三个 Q10 常量）----
    parameter integer CENTER_K_Q10    = 3146,   // throttle(±1000) → 中心速度 delta
    parameter integer DIFF_K_Q10      = 627,    // steer(±1000)    → 差速 delta
    parameter integer MAX_DELTA       = 4096,   // 单轮目标限幅（≈400 rpm）
    parameter integer CNT_PER_RPM_Q10 = 10486,  // 10.24 计数/rpm（显示换算用）
    // ---- PID 初值（Q8，可在运行时由串口命令在线改）----
    parameter integer KP_INIT         = 26,     // 0.1
    parameter integer KI_INIT         = 3,
    parameter integer KD_INIT         = 0
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- 使能与模式 ----
    input  wire                    i_enable,        // 系统启动（S1），0 = 停机
    input  wire                    i_estop,         // 急停（ToF 或按键），最高优先级
    input  wire                    i_manual,        // 1 = 遥控接管；0 = 自动导航
    input  wire signed [11:0]      i_steer,         // 遥控转向 ±1000
    input  wire signed [11:0]      i_throttle,      // 遥控油门 ±1000

    // ---- 自动导航预留（未接入时 i_auto_valid = 0）----
    input  wire                    i_auto_valid,
    input  wire signed [31:0]      i_auto_center,   // 中心速度（内部单位）
    input  wire signed [31:0]      i_auto_diff,     // 差速量（内部单位）

    // ---- PID 参数（运行时可变）----
    input  wire signed [15:0]      i_kp,
    input  wire signed [15:0]      i_ki,
    input  wire signed [15:0]      i_kd,

    // ---- 编码器反馈（来自 motor_top）----
    input  wire signed [31:0]      i_delta0,        // A 左前
    input  wire signed [31:0]      i_delta1,        // B 右前
    input  wire signed [31:0]      i_delta2,        // C 左后
    input  wire signed [31:0]      i_delta3,        // D 右后
    input  wire                    i_delta_valid,   // 测速窗口结束（共享）
    input  wire [3:0]              i_stall,         // 各路堵转标志（锁存）

    // ---- 到 motor_top ----
    output wire [4*PWM_BITS-1:0]   o_duty,
    output wire [3:0]              o_dir,
    output wire [3:0]              o_enable,
    output wire [3:0]              o_brake,

    // ---- 观测（显示/上报）----
    output wire signed [31:0]      o_tgt_left,      // 左侧目标（内部单位）
    output wire signed [31:0]      o_tgt_right,
    output wire signed [31:0]      o_rpm0,
    output wire signed [31:0]      o_rpm1,
    output wire signed [31:0]      o_rpm2,
    output wire signed [31:0]      o_rpm3,
    output wire [3:0]              o_sat,           // 各路 PID 饱和
    output wire                    o_running        // 1 = 正在输出（给 LED）
);

    // ------------------------------------------------------------------
    // 1. 指令 → 中心速度 + 差速量（内部单位：计数/10ms）
    // ------------------------------------------------------------------
    // ★ 插一级流水：遥控量→内部单位的乘法单独打一拍，避免"乘法+限幅+差速+4 个 PID 输入"
    //   串成一条长组合路径（首次综合时 setup slack = −7.02 ns 就出在这条链上）
    reg signed [31:0] man_center;
    reg signed [31:0] man_diff;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            man_center <= 32'sd0;
            man_diff   <= 32'sd0;
        end else begin
            man_center <= ($signed(i_throttle) * CENTER_K_Q10) >>> 10;
            man_diff   <= ($signed(i_steer)    * DIFF_K_Q10)   >>> 10;
        end
    end

    // ★ 2026-10-16 修正：`i_auto_valid` 原来**只门控了差速**（diff_raw），中心速度
    //   center_raw 直接取 i_auto_center → 自动导航未接入时残留值会把车推着走。
    //   现在中心速度与差速量同门控：i_auto_valid = 0 ⇒ 两者都为 0。
    wire signed [31:0] center_raw = i_manual ? man_center : (i_auto_valid ? i_auto_center : 32'sd0);
    wire signed [31:0] diff_raw   = i_manual ? man_diff   : (i_auto_valid ? i_auto_diff : 32'sd0);

    // 限幅（指令层）
    function signed [31:0] clamp_delta;
        input signed [31:0] v;
        begin
            if (v >  MAX_DELTA)      clamp_delta =  MAX_DELTA;
            else if (v < -MAX_DELTA) clamp_delta = -MAX_DELTA;
            else                     clamp_delta = v;
        end
    endfunction

    // ★ 指令流水：限幅 → 差速分解 各打一拍，把"中心速度 → 限幅 → 差速 → PID 输入"
    //   这条长组合链切断（首次综合时这条链的 setup slack = −2.59 ns）。
    //   代价只是目标值晚 1 拍生效，控制环在 100 Hz 更新，完全可以忽略。
    reg signed [31:0] center_r, diff_r, tgt_left_r, tgt_right_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            center_r     <= 32'sd0;
            diff_r       <= 32'sd0;
            tgt_left_r   <= 32'sd0;
            tgt_right_r  <= 32'sd0;
        end else begin
            // ★ 中心速度【不单独限幅】：限幅交给左右轮（下面的 ±MAX_DELTA）。
            //   若先把 center 截到 MAX_DELTA，再算差速，比例就被破坏：
            //   实测 center=5000/diff=2000 时左轮只有 2096（= 4096−2000）而不是 3000，
            //   表现为"自动模式下大给定 + 转向时车往一侧偏"。这是仿真抓出来的。
            center_r     <= center_raw;
            diff_r       <= clamp_delta(diff_raw);
            tgt_left_r   <= clamp_delta(center_r - diff_r);    // 左 = 中心 − 差速
            tgt_right_r  <= clamp_delta(center_r + diff_r);    // 右 = 中心 + 差速
        end
    end

    wire signed [31:0] tgt_left  = tgt_left_r;
    wire signed [31:0] tgt_right = tgt_right_r;

    assign o_tgt_left  = tgt_left;
    assign o_tgt_right = tgt_right;

    // ------------------------------------------------------------------
    // 2. 运行/安全逻辑
    // ------------------------------------------------------------------
    wire any_stall = |i_stall;
    // 使能：系统启动 且 未急停 且 无堵转
    wire run_en = i_enable & ~i_estop & ~any_stall;

    // 刹车：仅急停时短路刹车（快速停）；堵转/正常停机走滑行（不继续发热）
    wire brake_all = i_estop;

    assign o_enable = {4{run_en}};
    assign o_brake  = {4{brake_all}};
    assign o_running = run_en & ((tgt_left != 0) | (tgt_right != 0));

    // ------------------------------------------------------------------
    // 3. 四路独立速度环
    //    通道顺序按本车标定记录：0=A 左前、1=B 右前、2=C 左后、3=D 右后
    //
    //    ★ 2026-10-16：四处 `.i_clear(~run_en)` 连线已**删除**。
    //      它是永不成立的**死分支**：`speed_pid` 内部是
    //          if (!i_enable) … else if (i_clear) …
    //      —— i_enable 判断在前，而这里 run_en 正是喂给同一个 speed_pid 的 i_enable，
    //      所以只要 run_en = 0，i_enable = 0 就已经先把它清零了，i_clear 永远轮不到。
    //      ★ `speed_pid` 的 `i_clear` **端口本身保留**（库接口不动、其它使用者仍可用；
    //        其"重新起调"的语义由 i_enable = 0 的复位路径覆盖）。
    //      ★ 但端口**必须显式接常量**（下面四处均为 `.i_clear(1'b0)`）：完全不写这条连线
    //        会留下空连接，实测新增 8 条 Gowin 告警 —— 4 条 `WARN (EX3073) Port 'i_clear'
    //        remains unconnected` + 4 条 `WARN (EX2565) Input 'i_clear' … is undriven …
    //        simulation mismatch possible`。显式接 0 既消除这 8 条告警，又保持语义不变
    //        （该端口语义已被 i_enable = 0 的复位路径覆盖，见上一条）。
    // ------------------------------------------------------------------
    wire signed [31:0] u0, u1, u2, u3;
    wire signed [31:0] e0, e1, e2, e3;
    wire               s0, s1, s2, s3;

    // Four local reset-release branches keep the large DSP reset/data gates
    // close to each PID. Assertion is still asynchronous; only release waits
    // two clocks. Preserve branches so synthesis cannot merge the fanout back.
    reg [3:0] pid_reset_a /* synthesis syn_preserve=1 */;
    reg [3:0] pid_reset_b /* synthesis syn_preserve=1 */;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin pid_reset_a<=0;pid_reset_b<=0;end
        else begin pid_reset_a<=4'b1111;pid_reset_b<=pid_reset_a;end
    end

    // Coherent computation snapshot, separate from the live PWM inhibit.
    // One clock of latency applies equally to gains, setpoint, feedback and
    // enable. A disable/estop still zeros physical duty/enable immediately.
    reg pid_enable,pid_sample;
    reg signed [31:0] pid_left,pid_right,pid_delta0,pid_delta1,pid_delta2,pid_delta3;
    reg signed [15:0] pid_kp,pid_ki,pid_kd;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            pid_enable<=0;pid_sample<=0;pid_left<=0;pid_right<=0;
            pid_delta0<=0;pid_delta1<=0;pid_delta2<=0;pid_delta3<=0;pid_kp<=0;pid_ki<=0;pid_kd<=0;
        end else begin
            pid_enable<=run_en;pid_sample<=i_delta_valid;pid_left<=tgt_left;pid_right<=tgt_right;
            pid_delta0<=i_delta0;pid_delta1<=i_delta1;pid_delta2<=i_delta2;pid_delta3<=i_delta3;
            pid_kp<=i_kp;pid_ki<=i_ki;pid_kd<=i_kd;
        end
    end

    speed_pid #(
        .PWM_BITS(PWM_BITS), .KP_INIT(KP_INIT), .KI_INIT(KI_INIT), .KD_INIT(KD_INIT)
    ) u_pid_lf (
        .i_clock(clk), .i_reset_n(pid_reset_b[0]), .i_enable(pid_enable),
        .i_setpoint(pid_left),  .i_feedback(pid_delta0), .i_fb_valid(pid_sample),
        .i_kp(pid_kp), .i_ki(pid_ki), .i_kd(pid_kd),
        .i_clear(1'b0),
        .o_u(u0), .o_err(e0), .o_sat(s0)
    );

    speed_pid #(
        .PWM_BITS(PWM_BITS), .KP_INIT(KP_INIT), .KI_INIT(KI_INIT), .KD_INIT(KD_INIT)
    ) u_pid_rf (
        .i_clock(clk), .i_reset_n(pid_reset_b[1]), .i_enable(pid_enable),
        .i_setpoint(pid_right), .i_feedback(pid_delta1), .i_fb_valid(pid_sample),
        .i_kp(pid_kp), .i_ki(pid_ki), .i_kd(pid_kd),
        .i_clear(1'b0),
        .o_u(u1), .o_err(e1), .o_sat(s1)
    );

    speed_pid #(
        .PWM_BITS(PWM_BITS), .KP_INIT(KP_INIT), .KI_INIT(KI_INIT), .KD_INIT(KD_INIT)
    ) u_pid_lr (
        .i_clock(clk), .i_reset_n(pid_reset_b[2]), .i_enable(pid_enable),
        .i_setpoint(pid_left),  .i_feedback(pid_delta2), .i_fb_valid(pid_sample),
        .i_kp(pid_kp), .i_ki(pid_ki), .i_kd(pid_kd),
        .i_clear(1'b0),
        .o_u(u2), .o_err(e2), .o_sat(s2)
    );

    speed_pid #(
        .PWM_BITS(PWM_BITS), .KP_INIT(KP_INIT), .KI_INIT(KI_INIT), .KD_INIT(KD_INIT)
    ) u_pid_rr (
        .i_clock(clk), .i_reset_n(pid_reset_b[3]), .i_enable(pid_enable),
        .i_setpoint(pid_right), .i_feedback(pid_delta3), .i_fb_valid(pid_sample),
        .i_kp(pid_kp), .i_ki(pid_ki), .i_kd(pid_kd),
        .i_clear(1'b0),
        .o_u(u3), .o_err(e3), .o_sat(s3)
    );

    assign o_sat = {s3, s2, s1, s0};

    // ------------------------------------------------------------------
    // 4. u → (dir, duty)
    //    ★ i_dir 语义：0 = 正转，1 = 反转（见 motor_pwm.v 注释）
    //    × 停机时强制 duty = 0（motor_pwm 里 duty=0 等价于停，避免"PWM 恒 0
    //      却在刹车"造成电机嗡嗡响）
    // ------------------------------------------------------------------
    function [PWM_BITS-1:0] duty_of;
        input signed [31:0] u;
        input               en;
        reg [PWM_BITS-1:0] mag;
        begin
            if (!en) begin
                duty_of = {PWM_BITS{1'b0}};
            end else begin
                mag = u[31] ? (~u[PWM_BITS-1:0] + 1'b1) : u[PWM_BITS-1:0];
                duty_of = mag;
            end
        end
    endfunction

    assign o_duty[0*PWM_BITS +: PWM_BITS] = duty_of(u0, run_en);
    assign o_duty[1*PWM_BITS +: PWM_BITS] = duty_of(u1, run_en);
    assign o_duty[2*PWM_BITS +: PWM_BITS] = duty_of(u2, run_en);
    assign o_duty[3*PWM_BITS +: PWM_BITS] = duty_of(u3, run_en);

    assign o_dir[0] = run_en & u0[31];
    assign o_dir[1] = run_en & u1[31];
    assign o_dir[2] = run_en & u2[31];
    assign o_dir[3] = run_en & u3[31];

    // ------------------------------------------------------------------
    // 5. 反馈 → rpm（显示/上报）：rpm = delta / 10.24 ≈ (delta × 100) >>> 10
    // ------------------------------------------------------------------
    function signed [31:0] delta_to_rpm;
        input signed [31:0] d;
        begin
            delta_to_rpm = (d * 100) >>> 10;
        end
    endfunction

    assign o_rpm0 = delta_to_rpm(i_delta0);
    assign o_rpm1 = delta_to_rpm(i_delta1);
    assign o_rpm2 = delta_to_rpm(i_delta2);
    assign o_rpm3 = delta_to_rpm(i_delta3);

endmodule
