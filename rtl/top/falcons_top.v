//////////////////////////////////////////////////////////////////////////////////
// 模块：falcons_top —— FPGA 差速小车整车顶层
//
// 【它做什么】只做例化与连线（依据功能设计文档 §4.1：「car_top.v —— 只做例化与连线」）
//
//   系统时钟/复位    50 MHz 板载时钟（Y18）+ 复位同步
//   输入            4 个板载按键、6 路遥控 PWM、4 路编码器、串口 RX、雷达 RX(C14)
//   输出            4 路电机（IN1/IN2/PWM）+ STBY、串口 TX、HDMI TMDS、8 个 LED
//   显示            1024x600 HDMI，中文四页界面（ui_render）
//   预留            ToF（J7 的 I2C 四线，模块未做出）、雷达（C14）；RK3588 通路待选型（J1 GPIO 已用尽）
//
// 【时钟域】两个（依据功能设计文档 §4.2 的策略）
//   * sys 域   50 MHz：IMU、控制、估计、遥控、串口、按键、页面状态
//   * pixel 域 49 MHz：hdmi_1024x600 内部 PLL 产生的像素时钟，UI 渲染、显示时序
//   跨域只走两种标准方式：
//     ① 状态位（页面/模式/急停/信号有效）→ pixel 域两级同步器
//     ② 慢变参数（页面号）→ 同上（变化间隔远大于同步器延迟）
//   ★ 控制链路全程 sys 域，绝不进像素域。
//
// 【安全链】急停优先级最高，其次堵转锁存，再是遥控/自动
//   急停来源：按键 S4（锁存切换）、ToF（预留，模块未做出时恒 0）
//   上电/复位期间：chassis_ctrl 的 i_enable = 0 → 电机 duty = 0、STBY 由 motor_top 门控
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module falcons_top (
    // ---- 时钟与复位 ----
    input  wire        i_clk_50m,          // Y18，50 MHz
    input  wire        i_key_reset_n,      // F15（S0），低有效复位

    // ---- 板载按键（S1~S4，低有效）----
    input  wire        i_key1_n,           // A20：启动 / 停止（切换）
    input  wire        i_key2_n,           // B20：编码器计数清零 + 堵转清除
    input  wire        i_key3_n,           // A21：切换界面页面
    input  wire        i_key4_n,           // B21：急停（锁存切换）

    // ---- 串口（板载 USB 转串口）----
    output wire        o_uart_tx,          // M15
    input  wire        i_uart_rx,          // J21

    // ---- 遥控接收机（6 路 PWM，通道映射见 rc_input 参数）----
    input  wire [5:0]  i_rc_pwm,

    // ---- 电机驱动（WHEELTEC D24A，J4/J5 排线直连）----
    output wire        o_stby,
    output wire [3:0]  o_motor_in1,
    output wire [3:0]  o_motor_in2,
    output wire [3:0]  o_motor_pwm,

    // ---- 编码器（A/B ×4）----
    input  wire [3:0]  i_enc_a,
    input  wire [3:0]  i_enc_b,

    // ---- HDMI 输出（板载 HDMI 口，1024x600@60）----
    output wire        o_tmds_clk_p,
    output wire        o_tmds_clk_n,
    output wire [2:0]  o_tmds_data_p,
    output wire [2:0]  o_tmds_data_n,

    // ---- IMU660RA（J2 接口，SPI Mode0，四线）----
    output wire        o_imu_sck,          // A14（J2 SCL）
    output wire        o_imu_mosi,         // A13（J2 SDA）
    input  wire        i_imu_miso,         // A16（J2 SDO/SA0）
    output wire        o_imu_cs_n,         // A15（J2 CS）

    // ---- 预留外设 ----
    // 雷达：J3.3 = C14（★ 不要接 A15 —— 那是 J2.6 IMU CS，且 J1 与 J2 同网络，会对打）
    input  wire        i_radar_rx,         // C14（J3.3 雷达 TX → 主板 RX，仅接收）
    // ToF：J7 排针（VIN / GND / SDA / SCL / INT / SHUT），是 **I2C** 接口
    //   ★ 2026-10-15 引脚复验修正（依据手册 J7 表）：
    //     原先把 J7.3/J7.4 误当成"ToF 回波 + RK3588 串口 TX"，语义是错的。
    //     现改为正确的 I2C 四线；ToF 模块尚未做出，四线暂置安全空闲态。
    output wire        o_tof_scl,          // F16（J7.4 SCL）
    inout  wire        io_tof_sda,         // E17（J7.3 SDA，I2C 双向）
    input  wire        i_tof_int,          // C20（J7.5 INT）
    output wire        o_tof_shut,         // D20（J7.6 SHUT）

    // ---- 状态指示 ----
    output wire [7:0]  o_led
);

    // ==================================================================
    // 0. 参数（本车实测标定值集中在这里，改车只改这一段）
    // ==================================================================
    localparam integer CLK_HZ        = 50_000_000;
    localparam integer PWM_BITS      = 11;              // PWM 周期 2^11 = 2048 拍 → 24.4 kHz
    localparam integer BAUD_DEBUG    = 115200;

    // 本车极性标定（见 class\car_fpga\doc\电机硬件标定记录.md）
    localparam [3:0]   INV_DIR_VEC     = 4'b0000;       // 电机方向：全不反相
    localparam [3:0]   ENC_INV_DIR_VEC = 4'b1010;       // 编码器计数方向：右前/右后反相

    // 速度环 PID 初值（Q8；运行时可经串口 P/I/D 命令在线改）
    //   ★ 2026-10-16 用户指定：P=0 / I=64 / D=0（Q8 口径，256 = 1.0）
    //     → Kp = 0.0、Ki = 0.25、Kd = 0.0，即【纯积分控制】
    //   量级提醒（原 Ki = 3，即 0.012，这是它的 21 倍）：
    //     每个 10 ms 窗口的增量 Δu = Ki × err / 256 = err / 4；
    //     误差 100 rpm 时每窗口 +25、每秒 +2500，而占空比满量程只有 2047
    //     → 约 0.8 s 就会积分饱和（CSV 的 flags bit3 会置位）。
    //     上板若表现为"起步猛冲 / 到速后来回振荡"，先把 Ki 降回来再试。
    //   注意 speed_pid.v 里也有一套同名默认值（26/3/0），那是**库的默认值**；
    //   本工程由顶层这三个 localparam 显式覆盖，实际生效值以这里为准。
    localparam integer KP_INIT = 0;                     // 0.0  —— 用户指定
    localparam integer KI_INIT = 64;                    // 0.25 —— 用户指定
    localparam integer KD_INIT = 0;                     // 0.0  —— 用户指定（先关微分，编码器量化噪声会被放大）

    // ==================================================================
    // 1. 复位与按键
    // ==================================================================
    wire sys_rst_n;

    reset_sync #(
        .CLK_FREQ_HZ (CLK_HZ),
        .DEBOUNCE_MS (20)
    ) u_reset_sync (
        .i_clock  (i_clk_50m),
        .i_key_n  (i_key_reset_n),
        .o_reset_n(sys_rst_n)
    );

    wire key1_press, key2_press, key3_press, key4_press;
    wire key1_hold,  key2_hold,  key3_hold,  key4_hold;

    app_key #(.CLK_FREQ_HZ(CLK_HZ), .DEBOUNCE_MS(10)) u_key1 (
        .i_clock(i_clk_50m), .i_reset_n(sys_rst_n), .i_key_n(i_key1_n),
        .o_pressed(key1_hold), .o_press_pulse(key1_press), .o_release_pulse()
    );
    app_key #(.CLK_FREQ_HZ(CLK_HZ), .DEBOUNCE_MS(10)) u_key2 (
        .i_clock(i_clk_50m), .i_reset_n(sys_rst_n), .i_key_n(i_key2_n),
        .o_pressed(key2_hold), .o_press_pulse(key2_press), .o_release_pulse()
    );
    app_key #(.CLK_FREQ_HZ(CLK_HZ), .DEBOUNCE_MS(10)) u_key3 (
        .i_clock(i_clk_50m), .i_reset_n(sys_rst_n), .i_key_n(i_key3_n),
        .o_pressed(key3_hold), .o_press_pulse(key3_press), .o_release_pulse()
    );
    app_key #(.CLK_FREQ_HZ(CLK_HZ), .DEBOUNCE_MS(10)) u_key4 (
        .i_clock(i_clk_50m), .i_reset_n(sys_rst_n), .i_key_n(i_key4_n),
        .o_pressed(key4_hold), .o_press_pulse(key4_press), .o_release_pulse()
    );

    // ==================================================================
    // 2. 遥控输入（6 路单路捕获实例化）
    // ==================================================================
    wire signed [11:0] rc_steer, rc_throttle;
    wire [1:0]         rc_mode;
    wire               rc_select, rc_confirm, rc_back;
    wire               rc_menu_up, rc_menu_down;      // 摇杆上下拨动（需求原文 §17）
    wire               rc_signal_ok;
    wire [6*16-1:0]    rc_chan_us;
    wire [5:0]         rc_timeout, rc_valid;

    rc_input #(
        .CLK_FREQ_HZ (CLK_HZ),
        .NUM_CH      (6),
        .CH_STEER    (0),
        .CH_THROTTLE (1),
        .CH_CONFIRM  (2),
        .CH_BACK     (3),
        .CH_MODE     (4),
        .CH_SELECT   (5)
    ) u_rc (
        .clk            (i_clk_50m),
        .rst_n          (sys_rst_n),
        .rc_pwm         (i_rc_pwm),
        .o_steer        (rc_steer),
        .o_throttle     (rc_throttle),
        .o_mode         (rc_mode),
        .o_btn_select   (rc_select),
        .o_btn_confirm  (rc_confirm),
        .o_btn_back     (rc_back),
        .o_menu_up      (rc_menu_up),
        .o_menu_down    (rc_menu_down),
        .o_signal_valid (rc_signal_ok),
        .o_chan_us      (rc_chan_us),
        .o_chan_timeout (rc_timeout),
        .o_chan_valid   (rc_valid)
    );

    // ==================================================================
    // 3. 运行状态：启动开关、急停锁存、页面选择
    // ==================================================================
    reg run_en;          // 底盘使能（S1 切换）
    reg estop_latch;     // 急停锁存（S4 切换；ToF 触发也会置位）
    reg [1:0] page;      // 界面页面 0=摄像头 1=雷达 2=车身数据 3=路径点导航

    // ToF 急停（模块未做出：保持 0；将来接上后由 distance < D_STOP 触发）
    wire tof_estop = 1'b0;
    // 雷达 RX：本轮未接模块，引脚已在约束里分配，先接住避免悬空告警
    wire radar_rx_unused = i_radar_rx;

    // ToF 尚未实现（模块没做出来）：四线置安全空闲态，等 tof_i2c 模块接入
    //   SCL 空闲高、SHUT 高（不关断）、SDA 高阻（不驱动总线）
    assign o_tof_scl  = 1'b1;
    assign o_tof_shut = 1'b1;
    assign io_tof_sda = 1'bz;

    // 遥控模式：0=手动接管 1=屏幕光标 2=自动
    //   光标模式下摇杆用于操作界面，不能开车 → 把油门/转向强制为 0
    //   ★ 本组 wire 必须在下面的 always 之前声明：always 里按 B2 要用 cursor_mode
    wire        manual_drive = (rc_mode == 2'd0);
    wire        cursor_mode  = (rc_mode == 2'd1);
    wire signed [11:0] steer_in    = cursor_mode ? 12'sd0 : rc_steer;
    wire signed [11:0] throttle_in = cursor_mode ? 12'sd0 : rc_throttle;

    // 手动接管时才允许底盘接收摇杆；自动模式下由导航给定（未接入 → 0）
    wire chassis_manual = manual_drive;

    // ---- 启动门禁（A1，2026-10-15 加）----
    //   ★ 用户要求"油门中位要有死区"：归一化油门是连续量，摇杆回弹误差与接收机
    //     中位漂移都会让"精确等于 0"永远不成立，所以按死区判定而不是判 ==0。
    //   量程：rc_input 输出的油门是 −1000…+1000，死区 100 = 满行程的 10%，
    //   与 rc_input 自身的 DEADZONE_US(25 µs) 叠加后，允许的中位误差约 ±15% 行程。
    //   上板实测若"杆已回中却启动不了"→ 调大此值；若"轻轻一碰就启动"→ 调小。
    localparam integer ARM_THROTTLE_DEADZONE = 100;
    wire        throttle_centered = (rc_throttle <= ARM_THROTTLE_DEADZONE) &&
                                    (rc_throttle >= -ARM_THROTTLE_DEADZONE);
    // 允许启动：油门在中位死区内 且 遥控信号有效（急停由下面的强制停机兜住）
    wire        arm_ready = throttle_centered & rc_signal_ok;
    // 提示"请回中位"：没在运行、信号正常、油门却没回中位、也没急停
    wire        arm_hint  = ~run_en & rc_signal_ok & ~throttle_centered & ~estop_latch;

    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            run_en      <= 1'b0;
            estop_latch <= 1'b0;
            page        <= 2'd0;
        end else begin
            // S1：启动 / 停止切换（A1 启动门禁）
            //   停止：任何时候按 S1 都能停（安全优先，不加任何附加条件）
            //   启动：必须"油门在中位死区内 + 遥控信号有效"，防止上电/失控时一按就窜车
            if (key1_press) begin
                if (run_en)         run_en <= 1'b0;       // 停止
                else if (arm_ready) run_en <= 1'b1;       // 启动（门禁通过）
            end
            // 急停锁存：S4 或 ToF 触发置位；再按 S4 清除
            if (tof_estop)        estop_latch <= 1'b1;
            else if (key4_press)  estop_latch <= ~estop_latch;
            // ---- 页面（菜单选项）切换 ----
            //   ★ 依据需求原文《使用FPGA实现的差速小车功能设计.md》§17：
            //     "左侧…为菜单列表…**使用摇杆上下拨动来切换选项**"，
            //     且 §13 规定摇杆操作屏幕只在"模式开关选择控制屏幕光标"档。
            //   ★ 更正（2026-10-16）：上一版把切页做成"遥控 select/confirm 或板载 S3"，
            //     **与需求原文不符**（review 时被指出）。现改回摇杆：
            //       - 光标档（mode=1）+ 摇杆向上/向下拨 → 切上一项/下一项
            //       - 板载 S3 保留为"无遥控时的备用翻页"，不占用遥控按键
            //       - select/confirm 不再参与切页：§33 里它们是"切辅助菜单项 / 确认路径点"，
            //         属于路径点导航页的交互（该页尚未实现），不该在有菜单的地方顺手翻页
            //   ★ 四个页面都是"摇杆仍可上下切换菜单选项"（§21/§25/§29），
            //     所以本设计里"选中项"与"显示的页面"是同一个变量 page：拨动即切换并显示。
            if (key3_press) begin
                page <= (page == 2'd3) ? 2'd0 : (page + 2'd1);
            end else if (cursor_mode && rc_menu_up) begin
                page <= (page == 2'd0) ? 2'd3 : (page - 2'd1);
            end else if (cursor_mode && rc_menu_down) begin
                page <= (page == 2'd3) ? 2'd0 : (page + 2'd1);
            end
            // 急停时强制停机
            if (estop_latch) run_en <= 1'b0;
        end
    end

    // ==================================================================
    // 4. 电机与编码器（vendor: motor_tb6612）
    // ==================================================================
    wire [4*PWM_BITS-1:0] duty_flat;
    wire [3:0]            dir_vec, en_vec, brake_vec;
    wire [127:0]          count_flat, delta_flat;
    wire                  delta_valid;
    wire [3:0]            enc_dir_vec, stall_vec;

    wire signed [31:0] dlt0 = delta_flat[ 0*32 +: 32];
    wire signed [31:0] dlt1 = delta_flat[ 1*32 +: 32];
    wire signed [31:0] dlt2 = delta_flat[ 2*32 +: 32];
    wire signed [31:0] dlt3 = delta_flat[ 3*32 +: 32];

    // 编码器计数清零（S2 按键，或串口命令）
    reg count_clear_r;
    // 堵转清除（S2 按键，或串口命令）
    wire stall_clear_cmd;   // 来自串口
    reg  stall_clear_r;
    // 航向角清零（S2 或串口 X 命令）：★ 必须声明在用它的 always 块之前
    reg  yaw_clear_r;

    // ==================================================================
    // 5. 底盘控制（差速分解 + 4×增量式 PID）
    // ==================================================================
    wire signed [31:0] tgt_left, tgt_right;
    wire signed [31:0] rpm0, rpm1, rpm2, rpm3;
    wire [3:0]         pid_sat;
    wire               chassis_running;

    // PID 参数（初值来自参数，可经串口 P/I/D 在线改）
    reg signed [15:0] kp_r, ki_r, kd_r;

    chassis_ctrl #(
        .PWM_BITS     (PWM_BITS),
        .KP_INIT      (KP_INIT),
        .KI_INIT      (KI_INIT),
        .KD_INIT      (KD_INIT)
    ) u_chassis (
        .clk          (i_clk_50m),
        .rst_n        (sys_rst_n),
        .i_enable     (run_en),
        .i_estop      (estop_latch | tof_estop),
        .i_manual     (chassis_manual),
        .i_steer      (steer_in),
        .i_throttle   (throttle_in),
        // 自动导航预留：未接入
        .i_auto_valid (1'b0),
        .i_auto_center(32'sd0),
        .i_auto_diff  (32'sd0),
        .i_kp         (kp_r),
        .i_ki         (ki_r),
        .i_kd         (kd_r),
        .i_delta0     (dlt0),
        .i_delta1     (dlt1),
        .i_delta2     (dlt2),
        .i_delta3     (dlt3),
        .i_delta_valid(delta_valid),
        .i_stall      (stall_vec),
        .o_duty       (duty_flat),
        .o_dir        (dir_vec),
        .o_enable     (en_vec),
        .o_brake      (brake_vec),
        .o_tgt_left   (tgt_left),
        .o_tgt_right  (tgt_right),
        .o_rpm0       (rpm0),
        .o_rpm1       (rpm1),
        .o_rpm2       (rpm2),
        .o_rpm3       (rpm3),
        .o_sat        (pid_sat),
        .o_running    (chassis_running)
    );

    motor_top #(
        .CLK_FREQ_HZ   (CLK_HZ),
        .PWM_BITS      (PWM_BITS),
        .WINDOW_MS     (10),                      // 100 Hz 测速窗口（与文档 §4.3 一致）
        .INV_DIR_VEC   (INV_DIR_VEC),
        .ENC_INV_DIR_VEC(ENC_INV_DIR_VEC),
        // A3：堵转启动宽限 30 → 50 窗口（300 ms → 500 ms）
        //   库默认 30 窗口是给小惯量电机调的：本车 30:1 减速 + 整车质量，
        //   从静止给到目标转速需要更久，300 ms 内位移不足会被误判堵转并锁存。
        .STALL_STARTUP_WINDOWS (50)
    ) u_motor (
        .i_clock       (i_clk_50m),
        .i_reset_n     (sys_rst_n),
        .i_enable      (en_vec),
        .i_dir         (dir_vec),
        .i_brake       (brake_vec),
        .i_duty        (duty_flat),
        .i_stall_clear (stall_clear_r),
        .o_stby        (o_stby),
        .o_in1         (o_motor_in1),
        .o_in2         (o_motor_in2),
        .o_pwm         (o_motor_pwm),
        .o_stall       (stall_vec),
        .i_enc_a       (i_enc_a),
        .i_enc_b       (i_enc_b),
        .i_count_clear (count_clear_r),
        .o_count       (count_flat),
        .o_delta       (delta_flat),
        .o_delta_valid (delta_valid),
        .o_enc_dir     (enc_dir_vec)
    );

    // ==================================================================
    // 6. 串口：命令接收（调参）+ 上报预留
    // ==================================================================
    wire [7:0] rx_data;
    wire       cmd_error;      // 串口命令解析错误标志（bit2 上报用）
    wire       rx_valid;

    uart_byte_rx #(
        .CLK_FREQ_HZ (CLK_HZ),
        .BAUD_RATE   (BAUD_DEBUG),
        .RECOVERY_EN (1)
    ) u_uart_rx (
        .i_clock   (i_clk_50m),
        .i_reset_n (sys_rst_n),
        .i_uart_rx (i_uart_rx),
        .o_rx_data (rx_data),
        .o_rx_valid(rx_valid),
        .i_rx_ready(1'b1),
        .o_rx_error()
    );

    wire        cmd_kp_valid, cmd_ki_valid, cmd_kd_valid;
    wire signed [15:0] cmd_kp, cmd_ki, cmd_kd;
    wire        cmd_count_clear, cmd_stall_clear;

    uart_cmd_rx #(
        .PWM_BITS (PWM_BITS)
    ) u_cmd (
        .i_clock      (i_clk_50m),
        .i_reset_n    (sys_rst_n),
        .i_rx_data    (rx_data),
        .i_rx_valid   (rx_valid),
        .o_tgt_valid  (),
        .o_tgt_sel    (),
        .o_tgt_value  (),
        .o_kp_valid   (cmd_kp_valid),
        .o_kp_value   (cmd_kp),
        .o_ki_valid   (cmd_ki_valid),
        .o_ki_value   (cmd_ki),
        .o_kd_valid   (cmd_kd_valid),
        .o_kd_value   (cmd_kd),
        .o_mode_valid (),
        .o_closed_loop(),
        .o_duty_valid (),
        .o_duty_value (),
        .o_count_clear(cmd_count_clear),
        .o_stall_clear(cmd_stall_clear),
        .o_report_req (),
        .o_cmd_error  (cmd_error)
    );

    // 串口 TX 由下面的 speed_report 驱动（见 6c 节）

    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            kp_r           <= KP_INIT[15:0];
            ki_r           <= KI_INIT[15:0];
            kd_r           <= KD_INIT[15:0];
            count_clear_r  <= 1'b0;
            stall_clear_r  <= 1'b0;
            yaw_clear_r    <= 1'b0;
        end else begin
            if (cmd_kp_valid) kp_r <= cmd_kp;
            if (cmd_ki_valid) ki_r <= cmd_ki;
            if (cmd_kd_valid) kd_r <= cmd_kd;

            count_clear_r <= key2_press | cmd_count_clear;
            stall_clear_r <= key2_press | cmd_stall_clear;
            yaw_clear_r   <= key2_press | cmd_count_clear;   // S2 同时清零航向角
        end
    end

    // ==================================================================
    // 6b. IMU 链路：imu660ra（SPI 驱动）→ 倾斜补偿 → 偏航角积分
    //
    // ★ 三条接线契约（模块库文档 + car_fpga 已上板验证过的接法）：
    //   ① gyro_tilt_comp.i_sample_valid 必须接 imu_top.o_acc_valid，
    //      **不是** o_data_valid —— o_data_valid 只在 RUN 态产生，
    //      标定窗口期内一个样本都收不到，会让倾斜补偿永远不生效且不报错；
    //   ② gyro_tilt_comp.i_calib_start 接 imu_top.o_calib_busy；
    //   ③ yaw_integrate.i_sample_valid 接 imu_top.o_data_valid。
    //
    // ★ imu_top 的 SCK_HALF_DIV 必须**显式写 25**（SCK = 1 MHz）：
    //   源码默认值是 3（8.33 MHz），实测会导致 8 KB 配置加载失败，
    //   表现为 o_err_code = 2（配置校验失败），极易误判成"模块坏了"。
    // ==================================================================
    wire signed [31:0] imu_gyro_x, imu_gyro_y, imu_gyro_z;
    wire signed [15:0] imu_acc_x, imu_acc_y, imu_acc_z;
    wire               imu_data_valid, imu_acc_valid, imu_calib_busy;
    wire               imu_init_done, imu_err_flag, imu_calib_done, imu_calib_fail;
    wire [7:0]         imu_err_code, imu_state_dbg;

    imu_top #(
        .CLK_FREQ_HZ   (CLK_HZ),
        .SCK_HALF_DIV  (25),            // ★ 1 MHz，显式覆盖源码默认 3
        .SAMPLE_DIV    (50_000),        // 1 kHz 读取
        .PWR_WAIT_MS   (20),
        .CHK_RETRY_MAX (10),
        .CALIB_DEV_TH  (100)            // 抖动判据 1°/s；标定失败就调大它
    ) u_imu (
        .i_clock      (i_clk_50m),
        .i_reset_n    (sys_rst_n),
        .o_imu_sck    (o_imu_sck),
        .o_imu_mosi   (o_imu_mosi),
        .i_imu_miso   (i_imu_miso),
        .o_imu_cs_n   (o_imu_cs_n),
        .o_init_busy  (),
        .o_init_done  (imu_init_done),
        .o_err_flag   (imu_err_flag),
        .o_err_code   (imu_err_code),
        .o_state_dbg  (imu_state_dbg),
        .o_calib_busy (imu_calib_busy),
        .o_calib_done (imu_calib_done),
        .o_calib_fail (imu_calib_fail),
        .o_bias_x     (),
        .o_bias_y     (),
        .o_bias_z     (),
        .o_gyro_x     (imu_gyro_x),
        .o_gyro_y     (imu_gyro_y),
        .o_gyro_z     (imu_gyro_z),
        .o_acc_x      (imu_acc_x),
        .o_acc_y      (imu_acc_y),
        .o_acc_z      (imu_acc_z),
        .o_data_valid (imu_data_valid),
        .o_acc_valid  (imu_acc_valid)
    );

    wire signed [31:0] gyro_z_comp;
    wire               tilt_fail;
    wire [31:0]        tilt_gain;

    gyro_tilt_comp #(
        .CALIB_SAMPLES_LG (8),          // 256 个样本
        .GAIN_MIN_Q16     (32768),      // 0.5
        .GAIN_MAX_Q16     (131072)      // 2.0 → 对应倾斜 60°
    ) u_tilt (
        .i_clock        (i_clk_50m),
        .i_reset_n      (sys_rst_n),
        .i_acc_x        (imu_acc_x),
        .i_acc_y        (imu_acc_y),
        .i_acc_z        (imu_acc_z),
        .i_gyro_z       (imu_gyro_z),
        .i_sample_valid (imu_acc_valid),    // ★ 必须 acc_valid（契约①）
        .i_calib_start  (imu_calib_busy),   // ★ 契约②
        .o_gyro_z       (gyro_z_comp),
        .o_ready        (),
        .o_tilt_fail    (tilt_fail),
        .o_gain_q16     (tilt_gain),
        .o_calib_busy   ()
    );

    wire signed [31:0] yaw_raw;
    wire               yaw_valid;
    // 注：yaw_clear_r 的声明放在文件前面的"清零信号"处（串口 always 块要用它）

    yaw_integrate #(
        .CLK_FREQ_HZ (CLK_HZ),
        .INT_RATE_HZ (1024),            // 2 的幂 → dt = 2⁻¹⁰ s，每拍零乘法
        .DEADZONE_EN (1),
        .DEADZONE_TH (30)               // 静止死区 0.3°/s；上板后按漂移现象调
    ) u_yaw (
        .i_clock        (i_clk_50m),
        .i_reset_n      (sys_rst_n),
        .i_gyro_z       (gyro_z_comp),      // 已做安装倾斜补偿
        .i_sample_valid (imu_data_valid),   // ★ 契约③
        .i_enable       (1'b1),             // 常使能（航向保持是后续功能）
        .i_clear        (yaw_clear_r),      // S2 / 串口 X 命令清零
        .o_yaw          (yaw_raw),
        .o_yaw_valid    (yaw_valid),
        .o_deadzone_hit (),
        .o_gz_eff       ()
    );

    // 航向角显示用整数度：yaw 单位 0.01°，÷100 ≈ (×10486) >>> 20
    //   （1/100 = 0.01；0.01 × 2²⁰ = 10485.76 → 取 10486，误差 < 0.003%）
    wire signed [31:0] yaw_deg = (yaw_raw * 10486) >>> 20;

    // ==================================================================
    // ==================================================================
    // 6c. 串口 CSV 上报：速度环关键量 @50 Hz
    //
    // 协议（8 列，115200 8N1）：tgt,act,duty,kp,ki,kd,flags,err
    //   tgt  = 左侧目标轮速 (rpm)      act = 左侧实际轮速 (rpm)
    //   duty = 通道 0 的有符号占空比（dir 为 1 时取负）
    //   kp/ki/kd = 当前 PID 参数（Q8，256 = 1.0），可用串口 P/I/D 命令在线改
    //   flags 位：bit0 闭环使能 / bit1 堵转 / bit2 串口命令错误 / bit3 PID 饱和
    //   err  = 目标 − 实际 (rpm)
    //   steer/throttle/mode/sig_ok = 遥控输入（第 9~12 列，2026-10-15 加）
    //     ★ 调遥控时必须能看到"杆位 → 归一化值"这一环，否则轮速不对时无法判断
    //       是遥控进来的值就不对，还是 PID/底盘的问题。
    //
    // 注：speed_report 自带 FIFO 与 UART 发送器，所以顶层**不需要**再例化
    //     uart_byte_tx；直接用它的 o_uart_tx 驱动串口输出脚（M15）。
    // ==================================================================
    wire [PWM_BITS-1:0] duty0_raw   = duty_flat[0*PWM_BITS +: PWM_BITS];
    wire signed [31:0]  report_tgt   = (tgt_left * 100) >>> 10;   // delta → rpm（÷10.24）
    wire signed [31:0]  report_duty  = dir_vec[0] ? -$signed({1'b0, duty0_raw})
                                                  :  $signed({1'b0, duty0_raw});
    wire signed [31:0]  report_err   = report_tgt - rpm0;
    wire signed [31:0]  report_flags = {28'd0, (|pid_sat), cmd_error, (|stall_vec), run_en};

    speed_report #(
        .CLK_FREQ_HZ    (CLK_HZ),
        .BAUD_RATE      (BAUD_DEBUG),
        .REPORT_DIV     (1_000_000),    // 50 MHz / 1_000_000 = 50 Hz
        // A2：12 列最长一行约 96 字符，32 字节 FIFO 会让状态机频繁等空位，
        //     加大到 128 字节（一行可整行缓冲，BSRAM 开销可忽略）
        .FIFO_DEPTH_LOG2(7)
    ) u_report (
        .i_clock   (i_clk_50m),
        .i_reset_n (sys_rst_n),
        .i_tgt     (report_tgt),
        .i_act     (rpm0),
        .i_duty    (report_duty),
        .i_kp      (kp_r),
        .i_ki      (ki_r),
        .i_kd      (kd_r),
        .i_flags   (report_flags),
        .i_err     (report_err),
        // 遥控输入原样上报（12 位有符号 → 32 位，符号扩展后便于示波/绘图对照）
        .i_steer    ({{20{rc_steer[11]}},    rc_steer}),
        .i_throttle ({{20{rc_throttle[11]}}, rc_throttle}),
        .i_mode     ({30'd0, rc_mode}),
        .i_sig_ok   ({31'd0, rc_signal_ok}),
        .i_force   (1'b0),
        .o_uart_tx (o_uart_tx)
    );

    // 7. 显示：HDMI 时序 + 中文界面
    // ==================================================================
    wire        pixel_clk, pixel_req, frame_start, frame_sync, video_active, pll_lock;
    wire [11:0] pixel_x, pixel_y;
    wire [23:0] ui_rgb;

    hdmi_1024x600 u_hdmi (
        .clk_50m       (i_clk_50m),
        .reset_n       (sys_rst_n),
        .pixel_rgb888  (ui_rgb),
        .pixel_clk     (pixel_clk),
        .pixel_req     (pixel_req),
        .pixel_x       (pixel_x),
        .pixel_y       (pixel_y),
        .frame_start   (frame_start),
        .frame_sync    (frame_sync),
        .video_active  (video_active),
        .pll_lock      (pll_lock),
        .tmds_clk_p    (o_tmds_clk_p),
        .tmds_clk_n    (o_tmds_clk_n),
        .tmds_data_p   (o_tmds_data_p),
        .tmds_data_n   (o_tmds_data_n)
    );

    // 像素域复位：等 PLL 锁定后在像素域同步释放
    //   （hdmi_1024x600 顶层内部直接用 reset_n & pll_lock，按文档 §3.1-2
    //     要求由使用方在顶层自建像素域同步链）
    // ★ 像素域复位【完全在域内自产】：只要 PLL 锁定就计数，数满后释放复位。
    //   不把 sys_rst_n 引到 pixel 域的复位端 —— 那会形成一条跨时钟域路径
    //   （实测 SETUP slack = −0.616 ns：u_reset_sync/key_stable_s0 → pixel_rst_sync_0_s0），
    //   而 SDC 里又无法用 get_clocks 引用 PLL 派生时钟名（约束解析阶段它还不存在，
    //   会报 TA2004 "Cannot get clock with name 'pixel_clk'"）。
    //   域内自产后该路径不复存在，时序自然满足。
    reg [3:0] pixel_pwr_cnt;
    always @(posedge pixel_clk) begin
        if (!pll_lock)                  pixel_pwr_cnt <= 4'd0;
        else if (pixel_pwr_cnt != 4'hF) pixel_pwr_cnt <= pixel_pwr_cnt + 4'd1;
    end
    wire pixel_rst_n = pixel_pwr_cnt[3];

    // ---- sys 域先打一拍，再跨到 pixel 域（2026-10-15 加，为消掉 CDC 的深组合路径）----
    //   为什么要多这一级：
    //     STA 会把 sys→pixel 当成同步关系分析（PLL 派生时钟名无法用 get_clocks 引用，
    //     写 set_false_path 会报 TA2004，见上面注释）。若跨域源是**组合逻辑**，
    //     组合深度会直接算进 slack：
    //       * arm_hint  = ~run_en & rc_signal_ok & ~throttle_centered & ~estop_latch
    //                     （含一个 12 位比较器）→ 实测 slack = −2.473 ns
    //       * rc_signal_ok = 6 路 timed_out 相或（在 rc_input 内部）→ −2.199 ns
    //     这里先在 sys 域寄存一拍，源就变成 DFF 输出、组合深度归零，
    //     同步器第一级的 slack 回落到与纯 DFF→DFF（estop_latch，−0.404 ns）同一量级。
    //   代价：延迟 1 个 50 MHz 周期（20 ns）。这些都是"人操作级别"的慢变量，无影响。
    reg [1:0] page_s, mode_s;
    reg       run_s, estop_s, sig_s, arm_s;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            page_s  <= 2'd0; mode_s  <= 2'd0;
            run_s   <= 1'b0; estop_s <= 1'b0; sig_s <= 1'b0; arm_s <= 1'b0;
        end else begin
            page_s  <= page;
            mode_s  <= rc_mode;
            run_s   <= chassis_running;
            estop_s <= estop_latch;
            sig_s   <= rc_signal_ok;
            arm_s   <= arm_hint;
        end
    end

    // sys → pixel 跨域：状态位两级同步（慢变量，间隔远大于同步器延迟）
    //   源一律用上面打过一拍的 *_s（纯 DFF 输出）
    reg [1:0] page_ps1, page_ps2, mode_ps1, mode_ps2;
    reg       run_ps1, run_ps2, estop_ps1, estop_ps2, sig_ps1, sig_ps2;
    reg       arm_ps1, arm_ps2;                          // A1："请回中位"提示
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            page_ps1  <= 2'd0; page_ps2  <= 2'd0;
            mode_ps1  <= 2'd0; mode_ps2  <= 2'd0;
            run_ps1   <= 1'b0; run_ps2   <= 1'b0;
            estop_ps1 <= 1'b0; estop_ps2 <= 1'b0;
            sig_ps1   <= 1'b0; sig_ps2   <= 1'b0;
            arm_ps1   <= 1'b0; arm_ps2   <= 1'b0;
        end else begin
            page_ps1  <= page_s;          page_ps2  <= page_ps1;
            mode_ps1  <= mode_s;          mode_ps2  <= mode_ps1;
            run_ps1   <= run_s;           run_ps2   <= run_ps1;
            estop_ps1 <= estop_s;         estop_ps2 <= estop_ps1;
            sig_ps1   <= sig_s;           sig_ps2   <= sig_ps1;
            arm_ps1   <= arm_s;           arm_ps2   <= arm_ps1;
        end
    end

    // ---- 实时数值链路：sys 域格式化 → 双口 RAM → pixel 域渲染 ----
    // 目标轮速用与 chassis_ctrl 相同的换算：rpm = delta × 100 >> 10（= ÷10.24）
    wire signed [31:0] tgt_left_rpm  = (tgt_left  * 100) >>> 10;
    wire signed [31:0] tgt_right_rpm = (tgt_right * 100) >>> 10;

    wire       dyn_we;
    wire [5:0] dyn_waddr;
    wire [7:0] dyn_wdata;
    wire [5:0] dyn_raddr;
    wire [7:0] dyn_rdata;

    // ★ 2026-10-16 字段按需求原文 §29 的数据页布局重排（9 项）：
    //     0 航向角 / 1~4 四轮转速 A,B,C,D / 5 角速度 / 6 中心速度 / 7,8 航位 x,y
    //   车轮旁显示的是【车轮转速】（rpm，±400），不是编码器累计计数 ——
    //   所以字段宽度仍是"符号 + 3 位数字"，字符 RAM 也仍是 64 深（9×4 = 36 字符）。
    wire signed [31:0] wheel_sum_rpm = rpm0 + rpm1 + rpm2 + rpm3;
    wire signed [31:0] center_rpm    = wheel_sum_rpm >>> 2;      // 中心速度 = 四轮平均
    // 角速度：gyro_tilt_comp 输出按 Q16 理解（若上板后量纲不对，改这个右移位数）
    wire signed [31:0] gyro_deg_s    = gyro_z_comp >>> 16;

    num_text #(
        .CLK_HZ     (CLK_HZ),
        .REFRESH_MS (100),
        .NUM_FIELDS (9)
    ) u_num_text (
        .clk    (i_clk_50m),
        .rst_n  (sys_rst_n),
        .i_v0   (yaw_deg),           // 字段 0：航向角（整数度，画在小车前端）
        .i_v1   (rpm0),              // 字段 1：A 左前轮转速
        .i_v2   (rpm1),              // 字段 2：B 右前轮转速
        .i_v3   (rpm2),              // 字段 3：C 左后轮转速
        .i_v4   (rpm3),              // 字段 4：D 右后轮转速
        .i_v5   (gyro_deg_s),        // 字段 5：角速度（整数 °/s，量纲待上板校准）
        .i_v6   (center_rpm),        // 字段 6：中心速度（四轮平均 rpm）
        .i_v7   (32'sd0),            // 字段 7：航位 x（odom 未实现 → 暂 0）
        .i_v8   (32'sd0),            // 字段 8：航位 y（odom 未实现 → 暂 0）
        .o_we   (dyn_we),
        .o_addr (dyn_waddr),
        .o_data (dyn_wdata)
    );

    dyn_text_ram #(
        .DEPTH     (64),
        .ADDR_BITS (6)
    ) u_dyn_text (
        .i_clk_w (i_clk_50m),        // 写：sys 域
        .i_we    (dyn_we),
        .i_waddr (dyn_waddr),
        .i_wdata (dyn_wdata),
        .i_clk_r (pixel_clk),        // 读：pixel 域
        .i_raddr (dyn_raddr),
        .o_rdata (dyn_rdata)
    );

    ui_render u_ui (
        .i_pixel_clk    (pixel_clk),
        .i_reset_n      (pixel_rst_n),
        .i_pixel_x      (pixel_x),
        .i_pixel_y      (pixel_y),
        .i_pixel_req    (pixel_req),
        .i_video_active (video_active),
        .o_pixel_rgb888 (ui_rgb),
        .i_page         (page_ps2),
        .i_ui_mode      (mode_ps2),
        .i_run          (run_ps2),
        .i_estop        (estop_ps2),
        .i_signal_ok    (sig_ps2),
        .i_arm_hint     (arm_ps2),
        .o_dyn_raddr    (dyn_raddr),
        .i_dyn_rdata    (dyn_rdata)
    );

    // ==================================================================
    // 8. 状态指示
    // ==================================================================
    reg [23:0] hb_cnt;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) hb_cnt <= 24'd0;
        else            hb_cnt <= hb_cnt + 24'd1;
    end

    assign o_led[0] = chassis_running;                 // 运行中
    assign o_led[1] = estop_latch;                     // 急停锁存
    assign o_led[2] = |stall_vec;                      // 任一路堵转
    assign o_led[3] = rc_signal_ok;                    // 遥控信号有效
    assign o_led[5:4] = rc_mode;                       // 当前模式
    assign o_led[6] = run_en;                          // 使能
    assign o_led[7] = hb_cnt[23];                      // 心跳（约 0.34 s 翻转）

    // 未使用的信号显式接住，避免综合器报"悬空网络"
    wire unused_ok = (|rc_chan_us) | (|rc_timeout) | (|rc_valid) | (|count_flat) |
                     (|enc_dir_vec) | (|pid_sat) | (|tgt_left) | (|tgt_right) |
                     (|rpm0) | (|rpm1) | (|rpm2) | (|rpm3) |
                     radar_rx_unused | i_tof_int |
                     imu_err_flag | (|imu_err_code) | (|imu_state_dbg) |
                     imu_init_done | imu_calib_done | imu_calib_fail |
                     (|imu_gyro_x) | (|imu_gyro_y) | (|imu_acc_x) | (|imu_acc_y) | (|imu_acc_z) |
                     (|tilt_gain) | tilt_fail | yaw_valid |
                     frame_start | frame_sync | key1_hold | key2_hold |
                     key3_hold | key4_hold | rc_back;

endmodule
