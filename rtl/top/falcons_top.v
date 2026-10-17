//////////////////////////////////////////////////////////////////////////////////
// 模块：falcons_top —— FPGA 差速小车整车顶层
//
// 【它做什么】只做例化与连线（依据功能设计文档 §4.1：「car_top.v —— 只做例化与连线」）
//
//   系统时钟/复位    50 MHz 板载时钟（Y18）+ 复位同步
//   输入            4 个板载按键、6 路遥控 PWM 接口（实际用 5 路）、4 路编码器、串口 RX、雷达 RX(C14)
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
    input  wire        i_key1_n,           // A20：接管档内的"停机锁存"（按一次停、再按解除）
    input  wire        i_key2_n,           // B20：编码器计数清零 + 堵转清除 + 航向角清零
    input  wire        i_key3_n,           // A21：备用翻页（无遥控时用；同时移高亮与页面）
    input  wire        i_key4_n,           // B21：急停（锁存切换）★ 实物是否为板上复位键待核

    // ---- 串口（板载 USB 转串口）----
    output wire        o_uart_tx,          // M15
    input  wire        i_uart_rx,          // J21

    // ---- 遥控接收机（6 路 PWM 接口，实际用 5 路；通道映射见 rc_input 参数）----
    //   实际控件（需求方 2026-10-16 明确，全部是 PWM 通道）：
    //     2 摇杆（前后 / 左右） + 2 按钮（确认 / select） + 1 拨杆开关（模式）
    //   ★ 没有独立"返回"按钮（§33 的"返回"是导航页辅助菜单项）
    //   ★ 保留 6 路接口，第 6 路（bit5 / J6.6 / CH8）预留未接 —— 因此 `SIG_VALID_MASK` 要把
    //     它排除在"信号有效"判定之外，否则未接通道的 timed_out 恒 1 → 整车永不能动
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
    input  wire        i_tof_int,          // C20（J7.5 INT）；★ 预留、暂未使用（ToF 模块未做出）
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
    // 2. 遥控输入（板级配置层 ht10a_input → 通用适配层 rc_input）
    // ==================================================================
    //   ★ 2026-10-16 合并：通道映射与全部标定参数集中在 `ht10a_input`（板级配置层），
    //     顶层只传"本车取值"。J6 槽位 ↔ 接收机通道的对应关系见该文件头注释与
    //     doc\HT10A接线与验收.md：
    //       bit0 转向(CH4) / bit1 油门(CH2) / bit2 确认(CH5) /
    //       bit3 select(CH7，原"返回"键位) / bit4 模式(CH6) / bit5 闲置(CH8)
    wire signed [11:0] rc_steer, rc_throttle;
    wire [1:0]         rc_mode;
    wire               rc_select, rc_confirm;
    wire               rc_menu_up, rc_menu_down;      // 摇杆上下拨动（需求原文 §17）
    wire               rc_signal_ok;
    wire [6*16-1:0]    rc_chan_us;
    wire [5:0]         rc_timeout, rc_valid;

    // ★ 下面全部是【开发默认值 / 未实测】，标定方法见 doc\HT10A接线与验收.md §3
    ht10a_input #(
        .CLK_FREQ_HZ(CLK_HZ),
        .STEER_CENTER_US(1500), .THROTTLE_CENTER_US(1500),
        .STEER_HALF_RANGE_US(500), .THROTTLE_HALF_RANGE_US(500),
        .STEER_DEADZONE_US(25), .THROTTLE_DEADZONE_US(25),
        // ★ 2026-10-17 上板实测标定：**推杆向前（油门为正）时车反而后退** → 油门通道取反。
        //   为什么只改这一处（别改错地方）：
        //     · 不反 `STEER_REVERSE` —— 左右方向实测正确（右拨 = 右转）；
        //     · 不动 `u_motor` 的 `INV_DIR_VEC` —— 四路全反会把**差速转向一起带反**
        //       （左右轮差速关系随前进方向整体翻转），修好前进、坏掉转向；
        //     · 闭环自洽性已核：油门反号后 `tgt` 反号，编码器符号定义未变，
        //       轮子物理转向也随之反 → `act` 与 `tgt` 仍**同号**，PID 不会变成正反馈。
        .STEER_REVERSE(0), .THROTTLE_REVERSE(1), .MENU_REVERSE(0),
        .MODE_REVERSE(0), .CONFIRM_ACTIVE_LOW(0), .SELECT_ACTIVE_LOW(0),
        .MIN_PULSE_US(900), .MAX_PULSE_US(2100), .TIMEOUT_US(100_000),
        .SW_LO_US(1300), .SW_HI_US(1700), .SW_HYST_US(30), .SW_CONFIRM_FRAMES(2),
        .MENU_STEP_TH(400), .MENU_IDLE_TH(150),
        // 6'b011111：bit5（J6.6，闲置未接）不参与"信号有效"判定 ——
        //   否则该路 timed_out 恒 1 → 整车永不能动（详见 rc_input.v 头注释②）
        //   ★ 位序：MSB=bit5 → 0 表示排除；其余 bit4..bit0 = 1 表示参与。
        .SIG_VALID_MASK(6'b011111)
    ) u_rc (
        .clk            (i_clk_50m),
        .rst_n          (sys_rst_n),
        .rc_pwm         (i_rc_pwm),
        .o_steer        (rc_steer),
        .o_throttle     (rc_throttle),
        .o_mode         (rc_mode),
        .o_btn_select   (rc_select),
        .o_btn_confirm  (rc_confirm),
        .o_menu_up      (rc_menu_up),
        .o_menu_down    (rc_menu_down),
        .o_signal_valid (rc_signal_ok),
        .o_chan_us      (rc_chan_us),
        .o_chan_timeout (rc_timeout),
        .o_chan_valid   (rc_valid)
    );

    // ==================================================================
    // 3. 运行状态：解锁守卫、急停锁存、页面选择
    // ==================================================================
    wire run_en;         // 底盘使能（由 rc_drive_guard 产生：回中解锁后保持，见下）
    reg estop_latch;     // 急停锁存（S4 切换；ToF 触发也会置位）
    reg [1:0] page;      // 界面【已确认进入】的页面 0=摄像头 1=雷达 2=车身数据 3=路径点导航
    reg [1:0] menu_sel;  // 菜单【高亮选中项】（摇杆拨动改它，按确认才赋给 page）

    // ToF 急停（模块未做出：保持 0；将来接上后由 distance < D_STOP 触发）
    wire tof_estop = 1'b0;
    // ★ 预留、暂未使用：雷达 RX（C14），本轮未接模块；引脚已在约束里分配。
    //   它不再被任何逻辑消费 —— 未用信号交给综合优化即可（理由见文件末尾第 9 节）。
    wire radar_rx_unused = i_radar_rx;

    // ToF 尚未实现（模块没做出来）：四线置安全空闲态，等 tof_i2c 模块接入
    //   SCL 空闲高、SHUT 高（不关断）、SDA 高阻（不驱动总线）
    assign o_tof_scl  = 1'b1;
    assign o_tof_shut = 1'b1;
    assign io_tof_sda = 1'bz;

    // 遥控模式：**两档**（"自动"档 2026-10-16 已按需求方要求删除）
    //   §13 只描述两档：接管车辆控制 / 控制屏幕光标。
    //   ★ 需求方 2026-10-16 明确的模式语义（本文件的全部交互门控都以此为准）：
    //      · 接管档：手柄**只**遥控车辆，不做任何屏幕交互（不切页、不切菜单）
    //      · 光标档：手柄**不**控制车辆，只操控屏幕的选项/设置/选择/开始任务
    //   ★ 本组 wire 必须在下面的 always 之前声明：always 里按 B2 要用 cursor_mode
    wire        manual_drive = (rc_mode == 2'd0);
    wire        cursor_mode  = (rc_mode == 2'd1);
    wire signed [11:0] steer_in    = cursor_mode ? 12'sd0 : rc_steer;
    wire signed [11:0] throttle_in = cursor_mode ? 12'sd0 : rc_throttle;

    // 手动接管时才允许底盘接收摇杆；自动模式下由导航给定（未接入 → 0）
    wire chassis_manual = manual_drive;

    // ---- 手动解锁守卫（`rc_drive_guard`，2026-10-16 合并自队友分支）----
    //   ★ 修的是一个**功能错误**：原实现把"油门在中位"当成**持续条件**
    //       run_en <= manual_drive & throttle_centered & rc_signal_ok & ...
    //     结果一推油门 `throttle_centered` 立刻变 0 → `run_en` 变 0 → **车根本不会动**。
    //     需求方的原话是"切到接管档即可开**但需要油门在中位**"，语义是**切档那一刻**
    //     的准入条件，不是持续约束。
    //   ★ 正确做法：回中**解锁一次**并锁存，之后推杆保持使能；
    //     切出接管档 / 丢信号 / 急停 / 按 S1 停机 → 撤销解锁，恢复后须**重新回中**。
    //   ARM_DEADZONE=100 = 归一化 ±1000 的 10%（需求方要求的"油门中位死区"）；
    //   "请回中位"提示只在**未解锁**时给（已解锁就不再提示）。
    wire arm_hint;
    rc_drive_guard #(.ARM_DEADZONE(100)) u_rc_guard (
        .clk(i_clk_50m), .rst_n(sys_rst_n), .i_manual(manual_drive),
        .i_signal_valid(rc_signal_ok), .i_estop(estop_latch | tof_estop | key4_press),
        .i_stop_pulse(key1_press), .i_throttle(rc_throttle),
        .o_enable(run_en), .o_arm_hint(arm_hint)
    );

    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            estop_latch <= 1'b0;
            page        <= 2'd0;
            menu_sel    <= 2'd0;      // 需求原文 §17：初始化默认选中"摄像头画面"
        end else begin
            // 底盘使能与 S1 停机锁存都搬进了 `rc_drive_guard`（见第 3 节开头说明）
            // 急停锁存：S4 或 ToF 触发置位；再按 S4 清除
            if (tof_estop)        estop_latch <= 1'b1;
            else if (key4_press)  estop_latch <= ~estop_latch;
            // ---- 菜单：摇杆只移动【高亮】，按确认才切页 ----
            //   §17"使用摇杆上下拨动来切换选项" + §21"选中该项**并确认时**…显示"
            //   两段式：menu_sel 由摇杆改；rc_confirm 把 menu_sel 赋给 page。
            //   ★★ 门控**全部在下层**：`rc_input` 的 `cursor_ready` 已要求
            //      「信号有效 + 模式通道档位连续 2 帧已确认 + 处于光标档」，
            //      不满足时按钮/菜单事件**根本不会产生**（见 rtl\rc\rc_input.v 头注释③）。
            //      所以这里**不再重复**写 `cursor_mode` / `rc_signal_ok` —— 单一真源；
            //      要改门控请改 rc_input，不要在这里加第二道条件。
            //   ★ 需求方 2026-10-16："模式为接管时，手柄不做遥控车辆之外的任何交互"
            //      —— 该语义由 `rc_input` 的 `cursor_ready` 保证（定义在 `rtl\rc\rc_input.v`）：
            //          cursor_ready = o_signal_valid & switch_ready[CH_MODE] & !mode_manual & (o_mode==1)
            //      ★ 本层**没有** `mode_manual` / `o_mode` 这两个信号（顶层只把 `rc_mode`
            //        即 o_mode 接出来，见第 2 节的 ht10a_input 例化）—— 原注释引用了本层
            //        不存在的名字，已按实际重写。要看去处的看 rc_input.v，不要在这里找。
            //   select 按钮属于路径点导航页的逻辑（§33 辅助菜单换项），本层不接
            //   （预留给路径点导航页 §33）。
            //   板载 S3 是板载按键、不经遥控链路，保留为"无遥控时的备用翻页"。
            if (key3_press) begin
                menu_sel <= (menu_sel == 2'd3) ? 2'd0 : (menu_sel + 2'd1);
                page     <= (page     == 2'd3) ? 2'd0 : (page     + 2'd1);
            end else if (rc_menu_up) begin
                menu_sel <= (menu_sel == 2'd0) ? 2'd3 : (menu_sel - 2'd1);
            end else if (rc_menu_down) begin
                menu_sel <= (menu_sel == 2'd3) ? 2'd0 : (menu_sel + 2'd1);
            end else if (rc_confirm) begin
                page <= menu_sel;                      // ★ 确认才进入该项页面（门控在下层）
            end
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
    // 6c. 串口 CSV 上报：速度环关键量 @50 Hz
    //
    // 协议（12 列，115200 8N1）：tgt,act,duty,kp,ki,kd,flags,err,steer,throttle,mode,sig_ok
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

    // ---- 像素域复位：等 PLL 锁定后在**像素域内**同步释放 ----
    //   ★ 必须声明在 u_hdmi 例化【之前】：u_hdmi 的 `reset_n` 就用它。
    //
    //   【为什么 u_hdmi 的 reset_n 必须接这个、不能接 sys_rst_n】(2026-10-16 修正)
    //     vendor 模块内部把 reset_n 同时送给两个**像素/串行域**的模块：
    //         disp_driver.Rst_n  = reset_n & pll_lock   （clk_pixel 域）
    //         DVI_TX_Top.I_rst_n = reset_n & pll_lock   （clk_serial_5x 域）
    //     把 sys 域的 sys_rst_n 直接接过去，等于让一个**异步释放**的复位去驱动
    //     pixel 域的触发器复位端 → 复位释放相对像素时钟沿的位置不确定，
    //     实测就是那 25 条 RECOVERY 违规（最差 −4.282 ns：
    //     u_reset_sync/key_stable → u_hdmi/…/u_OSER10_*/RESET）。
    //     这是**真实违规**（不是 CDC 伪违规）：复位释放不确定可能表现为
    //     **上电后 HDMI 偶发不点亮 / 花屏**。
    //     设计文档侧的要求见 `doc\设计落实待办.md` 的 §3.1-2（= 3.1 节第 2 条，
    //     条款编号回溯自 `doc\使用FPGA实现的差速小车功能设计_ai优化版.md`）：
    //     像素域复位链由使用方在顶层自建。
    //     ★ 2026-10-16 核对：vendor 包内两份 README 里**没有**这一节 —— 原注释写
    //       "vendor 文档 §3.1-2 也明确要求…"是引错了出处，已改为引本工程的文档。
    //
    //   【像素域复位为什么能完全在域内自产】
    //     PLL 的 lock 只取决于参考时钟与配置 —— `display_pll` 例化里**没有 reset 端口**
    //     （只有 .lock/.clkout0/.clkout1/.clkin），所以"复位依赖 pll_lock、
    //     pll_lock 又不依赖复位"不构成环路，不会死锁。
    //     做法：只要 PLL 锁定就计数，数到 bit3 置位时释放复位 ——
    //     释放沿由 pixel_clk 域 DFF 输出决定，对像素域而言是**同步释放**，
    //     recovery/removal 检查自然满足。
    //   ★ 原来只把它给了 ui_render 与跨域同步器，漏掉了 u_hdmi —— 本次补上。
    //     另：不能靠 SDC 写 `set_false_path -to [get_clocks {pixel_clk}]` 绕过：
    //     pixel_clk 是内部 PLL 的派生时钟，约束解析阶段还不是时钟对象，
    //     写了会报 ERROR (TA2004) 让综合直接失败。
    //   ★★ 2026-10-16 修正：`pll_lock` 是 **50 MHz 侧**产生的信号（display_pll 的参考
    //      时钟就是 clk_50m），原来直接拿它当 pixel 域计数条件 = **跨域未同步直接用**。
    //      它虽然只影响"释放早/晚一拍"，但它的下游 `pixel_rst_n` 又驱动 u_hdmi 的复位端
    //      —— 未同步的异步输入进复位路径，正是上面刚说过的那类 recovery/removal 风险。
    //      所以先用**两级同步器**把 pll_lock 同步到 pixel_clk，再用同步后的版本
    //      作为 pixel_pwr_cnt 的计数条件。
    //   ★ 注意：同步器与 pixel_pwr_cnt 都**不设异步复位端**（保留本 always 原有的性质）：
    //      它们等的就是"PLL 锁定"这件事本身；若给它们挂异步复位，就又要回答
    //      "这个复位从哪来" —— 最终会绕回 sys 域的 sys_rst_n，形成新的跨域复位路径，
    //      等于把刚修掉的问题换个地方再引入一次。
    //      上电时 DFF 初值为 0 → pll_lock 同步为 1 之后才开始计数释放，安全。
    reg pll_lock_px1, pll_lock_px2;
    always @(posedge pixel_clk) begin
        pll_lock_px1 <= pll_lock;
        pll_lock_px2 <= pll_lock_px1;
    end
    wire pll_lock_px = pll_lock_px2;

    reg [3:0] pixel_pwr_cnt;
    always @(posedge pixel_clk) begin
        if (!pll_lock_px)               pixel_pwr_cnt <= 4'd0;   // ★ 用同步后的 lock
        else if (pixel_pwr_cnt != 4'hF) pixel_pwr_cnt <= pixel_pwr_cnt + 4'd1;
    end
    wire pixel_rst_n = pixel_pwr_cnt[3];

    hdmi_1024x600 u_hdmi (
        .clk_50m       (i_clk_50m),
        .reset_n       (pixel_rst_n),     // ★ 像素域同步释放（原为 sys_rst_n，见上方说明）
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

    // ---- sys 域先打一拍，再跨到 pixel 域（2026-10-15 加，为消掉 CDC 的深组合路径）----
    //   为什么要多这一级：
    //     STA 会把 sys→pixel 当成同步关系分析（PLL 派生时钟名无法用 get_clocks 引用，
    //     写 set_false_path 会报 TA2004，见上面注释）。若跨域源是**组合逻辑**，
    //     组合深度会直接算进 slack：
    //       * arm_hint —— 由 `rtl\rc\rc_drive_guard.v` **组合**产生：
    //                     i_manual & i_signal_valid & !i_estop & !stopped & !armed & !centered
    //                     （其中 centered 是一个 12 位比较器）→ 实测 slack = −2.473 ns
    //       * rc_signal_ok —— = `rc_input` 的 o_signal_valid
    //                     = all_good & command_ready[3]，而 all_good 是"参与判定通道的
    //                       valid & ~expired 全体相与"（见 rtl\rc\rc_input.v）→ −2.199 ns
    //     ★ 2026-10-16 核对：原注释把 arm_hint 写成
    //       `~run_en & rc_signal_ok & ~throttle_centered & ~estop_latch`、把 rc_signal_ok
    //       写成"参与判定通道的 timed_out 相或" —— 两者都与现状不符（回中解锁逻辑早已
    //       搬进 rc_drive_guard，「信号有效」也早已改成"掩码内通道健康"的相与）。已按实际重写。
    //     这里先在 sys 域寄存一拍，源就变成 DFF 输出、组合深度归零，
    //     同步器第一级的 slack 回落到与纯 DFF→DFF（estop_latch，−0.404 ns）同一量级。
    //   代价：延迟 1 个 50 MHz 周期（20 ns）。这些都是"人操作级别"的慢变量，无影响。
    reg [1:0] page_s, mode_s, menu_s;
    //   ★ 2026-10-16：`run_s`（= chassis_running）已删 —— 它唯一的下游是 ui_render 的
    //     i_run，而该端口已删（模块内从不使用）。sys 域这一拍现在只剩 estop/sig/arm。
    reg       estop_s, sig_s, arm_s;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            page_s  <= 2'd0; mode_s  <= 2'd0; menu_s <= 2'd0;
            estop_s <= 1'b0; sig_s <= 1'b0; arm_s <= 1'b0;
        end else begin
            page_s  <= page;
            mode_s  <= rc_mode;
            menu_s  <= menu_sel;
            estop_s <= estop_latch;
            sig_s   <= rc_signal_ok;
            arm_s   <= arm_hint;
        end
    end

    // sys → pixel 跨域：状态位两级同步（慢变量，间隔远大于同步器延迟）
    //   源一律用上面打过一拍的 *_s（纯 DFF 输出）
    reg [1:0] page_ps1, page_ps2, mode_ps1, mode_ps2, menu_ps1, menu_ps2;
    reg       estop_ps1, estop_ps2, sig_ps1, sig_ps2;
    reg       arm_ps1, arm_ps2;                          // A1："请回中位"提示
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            page_ps1  <= 2'd0; page_ps2  <= 2'd0;
            mode_ps1  <= 2'd0; mode_ps2  <= 2'd0;
            menu_ps1  <= 2'd0; menu_ps2  <= 2'd0;
            estop_ps1 <= 1'b0; estop_ps2 <= 1'b0;
            sig_ps1   <= 1'b0; sig_ps2   <= 1'b0;
            arm_ps1   <= 1'b0; arm_ps2   <= 1'b0;
        end else begin
            page_ps1  <= page_s;          page_ps2  <= page_ps1;
            mode_ps1  <= mode_s;          mode_ps2  <= mode_ps1;
            menu_ps1  <= menu_s;          menu_ps2  <= menu_ps1;
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
        .i_menu_sel     (menu_ps2),
        .i_ui_mode      (mode_ps2),
        // ★ 2026-10-16：ui_render 的 i_run 端口已**删除**（模块内从不使用它，
        //   状态栏也没有"运行中"这一档）。其上游 sys 域 run_s 与 pixel 域
        //   run_ps1/run_ps2 整条跨域链随之删除；底盘运行指示仍由第 8 节的
        //   `assign o_led[0] = chassis_running;` 提供（chassis_running 未失去消费者）。
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

    // ==================================================================
    // 9. 未使用信号的处理（2026-10-16 修正）
    // ==================================================================
    //   ★ 这里原先有一条 `wire unused_ok = (|rc_chan_us) | ... ;`，把 30 多个未用
    //     信号"接住"，注释写的是"避免综合器报悬空网络"。**它是错的，已删除**：
    //       · 它本身是一个**没有任何消费者的死网**（全工程只出现 1 次）→ 依赖它的
    //         那棵"接住"逻辑树被综合整体优化掉，一条告警也没消掉，反而制造误导；
    //       · 实测综合告警：WARN (NL0002) … g_cap[5].u_cap is swept，
    //         以及 2732 条 PA1001（悬空网络）。
    //   ★ 正确认知：**未被使用的信号交给综合器优化即可**（综合按扇出自动裁掉，
    //     不消费就不会进网表）。
    //   ★ 副作用说明：**PA1001 悬空网络告警是 PnR 阶段的固有现象** —— 删 `unused_ok`
    //     前后**同为 2732 条**，不会因为"不消费"而消失（别指望靠删死代码消掉它）。
    //   ★ 确实需要"留在网表里"的信号（例如要上板用 Gowin Analyzer 抓波形、
    //     或要保住某个触发器做时序分析），必须让它被**可观测端口**消费
    //     （输出到 LED / 引脚，或写进可读 RAM），或给它加综合属性
    //     （`/* synthesis syn_keep = 1 */`、`preserve` 之类）—— 光靠一条没人用的
    //     wire 是接不住的，IO 上的悬空也照样报。
    //   ★ 本文件里下列信号都属于**预留、暂未使用**，只保留定义与注释、不做伪消费：
    //       radar_rx_unused（雷达 RX C14）/ i_tof_int（ToF 中断 C20）/
    //       rc_select（路径点导航页辅助菜单换项 §33）/ rc_chan_us / rc_timeout /
    //       rc_valid / count_flat / enc_dir_vec / pid_sat / yaw_valid /
    //       frame_start / frame_sync / key1~key4_hold / IMU 状态与诊断位等。
    //     将来要用它们时，直接把消费者接上即可（端口与 wire 都还在）。

endmodule
