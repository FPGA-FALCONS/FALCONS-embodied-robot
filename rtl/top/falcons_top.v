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
    output wire [7:0]  o_led,

    // ==================================================================
    // ★ 2026-10-17 新增：摄像头 OV5640(DVP) + DDR3 帧缓存
    //   引脚真源 = constr\falcons.cst（**按模块示例**的接法，BY_OV5640_V1.1 2x9 直插）
    //   数据通路 = rtl\cam\cam_overlay.v（架构照 examples\acg720_ddr3_hdmi，已实板验证）
    // ==================================================================
    output wire        camera_pwdn,        // L1 （PWDN，ov5640_capture 驱动为低=工作）
    output wire        camera_rst_n,       // B1 （RST）
    output wire        camera_sclk,        // L18（SCLK，SCCB/I2C）
    inout  wire        camera_sdat,        // A1 （SDAT，SCCB/I2C 双向）
    input  wire [7:0]  camera_data,        // J1/G1/H2/F1/G2/D1/E1/M17（D7..D0）
    input  wire        camera_pclk,        // J2 （PCLK，异步于系统时钟）
    input  wire        camera_href,        // B2 （HREF 行有效）
    input  wire        camera_vsync,       // M20（VSYNC 场同步）

    // ACG720 J5 high-speed USB FIFO; RK camera images only, no DVP upload.
    input wire i_fx2_ifclk,i_fx2_flagb,i_fx2_flagc,
    inout wire [15:0] io_fx2_fd,
    output wire [1:0] o_fx2_faddr,
    output wire o_fx2_slrd_n,o_fx2_slwr_n,o_fx2_sloe_n,o_fx2_pktend_n,o_fx2_slcs_n,
    output wire o_fx2_pa0,

    // ---- DDR3 帧缓存（SSTL15 / BANK_VCCIO=1.5；与官方管脚表逐针一致）----
    output wire [13:0] O_ddr_addr,
    output wire [2:0]  O_ddr_ba,
    output wire        O_ddr_cs_n,
    output wire        O_ddr_ras_n,
    output wire        O_ddr_cas_n,
    output wire        O_ddr_we_n,
    output wire        O_ddr_clk,
    output wire        O_ddr_clk_n,
    output wire        O_ddr_cke,
    output wire        O_ddr_odt,
    output wire        O_ddr_reset_n,
    output wire [1:0]  O_ddr_dqm,
    inout  wire [15:0] IO_ddr_dq,
    inout  wire [1:0]  IO_ddr_dqs,
    inout  wire [1:0]  IO_ddr_dqs_n
);

    // ==================================================================
    // 0. 参数（本车实测标定值集中在这里，改车只改这一段）
    // ==================================================================
    localparam integer CLK_HZ        = 50_000_000;
    localparam integer PWM_BITS      = 11;              // PWM 周期 2^11 = 2048 拍 → 24.4 kHz
    localparam integer BAUD_DEBUG    = 115200;

    // ★ 2026-10-19（需求方指示）：一级菜单 4 项 → 5 项，顺序改为需求原文的章节顺序。
    //   原第 4 项「路径点导航」在需求原文里拆成了「避障导航」+「非避障导航」两页。
    //   ★ 编码整体变了：数据页由 **2 → 0**（新顺序的第 1 项），摄像头页由 **0 → 1**。
    //     所以 `cam_overlay.show_cam` 不再是 `page == 0`，改为 `page == PAGE_CAMERA`。
    //   ★ `rtl\ui\ui_render.v` 里有等价的一份 `PG_*` 常量，**改序必须两处同改**。
    localparam [2:0] PAGE_DATA     = 3'd0;   // 运行数据显示
    localparam [2:0] PAGE_CAMERA   = 3'd1;   // 摄像头画面
    localparam [2:0] PAGE_RADAR    = 3'd2;   // 雷达建图展示
    localparam [2:0] PAGE_NAV_OBS  = 3'd3;   // 避障导航
    localparam [2:0] PAGE_NAV_FREE = 3'd4;   // 非避障导航
    localparam [2:0] PAGE_TUNE     = 3'd5;   // 参数整定（2026-10-20 新增，一期：存 RAM）
    localparam [2:0] PAGE_AI       = 3'd6;
    localparam [2:0] PAGE_LAST     = PAGE_AI;

    // ★ 2026-10-18 需求变更：上电/复位后**默认进入的页面**
    //   默认值 = **PAGE_DATA（运行数据显示）** —— 上电即可直接看到航位/转速/航向角，
    //   不必先按键翻页。原默认是摄像头页（§17 的旧要求，2026-10-18 已改）。
    //   ★★ 必须**同时**把 `menu_sel` 的复位值也设成它 ★★
    //      否则菜单高亮停在别的项上，用户在光标档按一次「确认」就会被拉过去，
    //      表现为"默认页没生效 / 自己跳走了"。
    localparam [2:0]   PAGE_DEFAULT  = PAGE_DATA;      // 0 = 运行数据显示页

    // ==================================================================
    // 本车极性标定（2026-10-17 深夜整体重构，口径：**内部"正" = 物理前进**）
    //
    //   【为什么要重构】此前为了修"推杆向前车却后退"，是在**遥控输入层**
    //   （`.THROTTLE_REVERSE(1)`）翻转的。那让车能正确前进，但整条内部数值链
    //   （center → tgt → u → dir → delta → rpm）都变成"负 = 物理前进"，
    //   于是 UI 得再翻一次、串口上报的 tgt 与 act 符号还会打架（§33 修显示时
    //   就撞上过：`report_err = report_tgt - rpm0_disp` 两端符号不一致）。
    //   本轮把极性下沉到**电机/编码器层**，让"正 = 前进"在内部天然成立。
    //
    //   【物理对应（需求方 2026-10-17 确认，真源）】
    //     通道 A = 右后 / B = 左后 / C = 右前 / D = 左前
    //   【参数】四路电机方向全反 + 编码器计数方向**逐位取反**
    //       INV_DIR_VEC     4'b0000 -> 4'b1111   （电机方向全反）
    //       ENC_INV_DIR_VEC 4'b1010 -> 4'b0101   （取反另一侧）
    //     ★ ENC_INV 的两位按真映射是 **左后(bit1=B) + 左前(bit3=D) = 左侧反相**；
    //       旧注释写的"右前/右后反相"是按错误映射推的，已纠正。
    //       镜像安装下取反哪一侧都能统一，这里取"与原来相反的一侧"以配合硬件全反。
    //
    //   【等价性】（改前 → 改后，物理运动完全相同，只有内部符号翻转）
    //     直行前进：改前 tgt 负 → dir=1(反转) → 电机实际反转 → 车前进
    //               改后 tgt 正 → dir=0(正转) → INV_DIR 全反 → 电机实际反转 → 车前进 ✅ 同
    //     左转：    改前 center 负、diff 负 → 左轮慢右轮快 → 左转
    //               改后 center 正、diff 正（STEER_REVERSE 同步改 0）→ 左轮慢右轮快 → 左转 ✅ 同
    //     编码器：  物理仍是"反转"，故计数符号不变；ENC_INV 取反后 rpm 由负变正
    //               → 与 tgt 同号 → 闭环仍收敛（关键：**必须**一起改，只改一半就是正反馈）
    //   ★ §26.4 当年记过"反 INV_DIR_VEC 会让差速一起反"——那是因为当时**只**反了
    //     INV_DIR，没同时把 THROTTLE/STEER 的取反撤掉。三者同步改，结论不适用。
    //
    // ==================================================================
    // ★★ 2026-10-18 车体整体掉头 180°（设计变更，**不是修 bug**）★★
    //
    //   【需求】需求方要把车的"前后朝向"调转：原本会产生前进的命令，今后让车往
    //   现在的**反方向**（= 新车头方向）走。改前改后，四轮/极性/映射都是自洽且
    //   经过实测正确的，本轮**只做刚体旋转的等价变换**，不修正任何既有行为。
    //
    //   【做法】180° 绕垂直轴旋转 = "前↔后"与"左↔右"同时交换。于是：
    //     ① 电机方向整体取反  INV_DIR     1111 → 0000
    //     ② 编码器计数整体取反 ENC_INV     0101 → 1010
    //       —— 两者**必须同时**取反：tgt 与 feedback 同号翻转 ⇒ 闭环仍收敛；
    //          只反一个就是正反馈（§26.5 记过这个坑）。
    //     ③ 通道 ↔ 轮位映射随之改变（旋转 180° 后每个通道落到对角象限）：
    //          旧：A=右后 B=左后 C=右前 D=左前
    //          新：**A=左前 B=右前 C=左后 D=右后**
    //     ④ 显示层：前轮改显示 A/B、后轮改显示 C/D（见 ui_render.v）
    //
    //   【为什么 chassis_ctrl 的接线一行都不用改】
    //     `chassis_ctrl` 把 {A,C} 接 tgt_left、{B,D} 接 tgt_right（见其 i_delta0..3 与
    //     四个 speed_pid 的对应）。180° 旋转把"A 是否在左侧"这个属性翻了一面，但
    //     {A,C} 与 {B,D} 的**分组结构不变** —— 旧 {A,C}=物理右侧，新 {A,C}=新左前+新左后
    //     = 新左侧。两组各自仍是"两个对角轮的 2+2 划分"，所以 tgt_left/right 的接线
    //     无需改动，差速语义自然跟随。
    //   【转向手感：掉头后**必须**把 STEER_REVERSE 从 0 改成 1】★★ 2026-10-18 实板实测修正 ★★
    //
    //   ★ 我原先的判断是错的："旋转同时交换物理左右与车体系左右，两次交换抵消，转向不变"。
    //     实板实测（需求方 2026-10-18）是**左右指令反了**：发左转指令、车朝右转。
    //
    //   真正的因果链（`chassis_ctrl.v` 第 148-149 行）：
    //       tgt_left = center − diff        tgt_right = center + diff
    //       tgt_left → {A,C}                tgt_right → {B,D}
    //   掉头前：{A,C} = 右后+右前 = **物理右侧**
    //           摇杆左 → steer 负 → diff 负 → tgt_left = center+|diff|（大）→ 物理右侧快 → 车左转 ✅
    //   掉头后：{A,C} = 左前+左后 = **新左侧**
    //           同一套符号下 → 新左侧快 → **车右转** ✗ ← 实测现象
    //
    //   为什么没有"两次抵消"：tgt_left 的符号语义没变（小 = 该组往负方向），
    //   但它驱动的实体从"物理右"换成了"物理左" —— **只翻转了一次**，所以必须再翻一次。
    //   修法：STEER_REVERSE 0 → **1**（在遥控层取反，tgt 链一行不动）。
    //   ★ 前后方向不受影响（THROTTLE_REVERSE 保持 0），与实测"车轮前后方向正确"一致。
    //   ★ 原地转向的对称性、串口 tgt/act 同号都保持（center 与 diff 的符号关系没变）。
    // ==================================================================
    localparam [3:0]   INV_DIR_VEC     = 4'b0000;       // 电机方向：掉头后**不再取反**（原 1111）
    localparam [3:0]   ENC_INV_DIR_VEC = 4'b1010;       // 编码器计数方向：原 0101 整体取反

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
    localparam integer KP_INIT = 24;                    // 24/256 = 0.09375 —— 2026-10-20 用户指定（原 0）
    localparam integer KI_INIT = 36;                    // 36/256 = 0.140625 —— 2026-10-20 用户指定（原 64）
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
    wire               rc_back, rc_aux_up, rc_aux_down;   // ★ 2026-10-19 新增三路事件
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
        // ★ 2026-10-17 深夜：两个取反**都撤掉**（原为 1/1），极性下沉到电机/编码器层。
        //   历史（§26.3 / §26.5，保留备查）：
        //     第一轮"推杆向前车反而后退" → 置 THROTTLE_REVERSE=1，前进修好；
        //     第二轮"左右也跟着反了"     → 再置 STEER_REVERSE=1，转向修好。
        //   但那样做的副作用是**整条内部链变成"负 = 物理前进"**（见文件上方
        //   INV_DIR_VEC 处的大段说明），UI 与串口上报都得跟着翻，容易漏。
        //   现在改成：THROTTLE/STEER 不取反（内部"正 = 前进"），方向交给
        //   `INV_DIR_VEC=1111` + `ENC_INV_DIR_VEC=0101` 一次性定死。
        //   ★ §26.5 那条"改油门却让转向翻转"的疑案也随映射真相（§34）一并结案：
        //     根因是四个通道与轮位整体错位，不是取反参数的锅。
        .STEER_REVERSE(1), .THROTTLE_REVERSE(0), .MENU_REVERSE(0),
        // ★ 2026-10-19：CH7 现在是模式开关（上=手动 下=光标），CH5 是三档（下=确认 上=返回）。
        //   这两个"哪个方向算哪档"由开关极性决定，**上板标定时改这两个参数**：
        //     MODE_REVERSE  = 1  → 让高档(宽脉冲)成为手动档
        //     CONFIRM_ACTIVE_LOW / AUX_SWAP → 决定确认与返回、辅助菜单上下的方向
        // ★ 2026-10-19 上板反馈：两个三档开关的上下方向都反了，这里按实测标定。
        //   原先 CONFIRM_ACTIVE_LOW=1 / AUX_SWAP=0 → 实测下拨成了"返回"、上拨成了"确认"，
        //   CH8 上拨成了"向下切"、下拨成了"向上切"。两个参数一起取反即可，不动译码逻辑。
        .MODE_REVERSE(0), .CONFIRM_ACTIVE_LOW(0), .SELECT_ACTIVE_LOW(0), .AUX_SWAP(1),
        .MIN_PULSE_US(900), .MAX_PULSE_US(2100), .TIMEOUT_US(100_000),
        .SW_LO_US(1300), .SW_HI_US(1700), .SW_HYST_US(30), .SW_CONFIRM_FRAMES(2),
        .MENU_STEP_TH(400), .MENU_IDLE_TH(150),
        // 6'b011111：bit5（J6.6，闲置未接）不参与"信号有效"判定 ——
        //   否则该路 timed_out 恒 1 → 整车永不能动（详见 rc_input.v 头注释②）
        //   ★ 位序：MSB=bit5 → 0 表示排除；其余 bit4..bit0 = 1 表示参与。
        // ★ 2026-10-19：CH8（bit5）现在有角色（辅助菜单上下切），6 路全部参与判定
        .SIG_VALID_MASK(6'b111111)
    ) u_rc (
        .clk            (i_clk_50m),
        .rst_n          (sys_rst_n),
        .rc_pwm         (i_rc_pwm),
        .o_steer        (rc_steer),
        .o_throttle     (rc_throttle),
        .o_mode         (rc_mode),
        .o_btn_select   (rc_select),
        .o_btn_confirm  (rc_confirm),
        .o_back         (rc_back),
        .o_aux_up       (rc_aux_up),
        .o_aux_down     (rc_aux_down),
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
    // ★ 2026-10-19：导航控制层（u_nav_ctrl）的输出线。u_nav_ctrl 在文件后部例化，
    //   但这些线要在 u_chassis（第 500 多行）就用，所以必须在**这里**声明。
    //   放后面声明 → Verilog 先按隐式 1 位 net 处理，位宽变 1 位，综合后极难查。
    wire signed [31:0] free_nav_cur_x_mm, free_nav_cur_y_mm;
    wire [2:0]   free_nav_aux_sel;
    wire [4:0]   free_nav_pt_count;      // 预留（未消费）：将来自检/上报用
    wire         free_nav_run, free_nav_mode; // free_nav_mode 预留（未消费）：0 间断 / 1 非间断
    wire [1:0]   free_nav_state;         // 0=SETUP 1=RUNNING 2=PAUSED（预留：给导航控制层）
    wire         free_nav_ui_active;     // 导航页交互态（菜单锁、状态栏）
    wire         free_nav_paused;        // 导航暂停
    wire [159:0] free_nav_pt_px, free_nav_pt_py;
    wire [255:0] free_nav_pt_x_mm, free_nav_pt_y_mm;   // 点表 mm 版（给导航控制层）
    wire [15:0]  free_nav_pt_v;
    wire [9:0]   free_nav_car_px, free_nav_car_py, free_nav_cur_px, free_nav_cur_py;

    // 导航控制层的回告（先声明后用：u_nav_free 在上、u_nav_ctrl 在下）
    wire         nav_ctrl_done, nav_ctrl_fault;

    reg [2:0] page;      // 界面【已确认进入】的页面 0=数据 1=摄像头 2=雷达 3=避障导航 4=非避障导航

    // 避障页为新增功能；原非避障 UI 和控制链保持独立用于调试。
    wire signed [31:0]      obs_cur_x_mm;
    wire signed [31:0]      obs_cur_y_mm;
    wire [2:0]              obs_aux_sel;
    wire [2:0]              nav_aux_sel = (page == PAGE_NAV_OBS) ? obs_aux_sel : free_nav_aux_sel;
    wire [4:0]              obs_pt_count;
    wire [4:0]              nav_pt_count = (page == PAGE_NAV_OBS) ? obs_pt_count : free_nav_pt_count;
    wire                    obs_run;
    wire                    nav_run = (page == PAGE_NAV_OBS) ? obs_run : free_nav_run;
    wire [2:0]              obs_ui_state;
    wire                    obs_ui_active;
    wire                    nav_ui_active = (page == PAGE_NAV_OBS) ? obs_ui_active : free_nav_ui_active;
    wire                    obs_paused;
    wire                    nav_paused = (page == PAGE_NAV_OBS) ? obs_paused : free_nav_paused;
    wire [159:0]            obs_pt_px;
    wire [159:0]            nav_pt_px = (page == PAGE_NAV_OBS) ? obs_pt_px : free_nav_pt_px;
    wire [159:0]            obs_pt_py;
    wire [159:0]            nav_pt_py = (page == PAGE_NAV_OBS) ? obs_pt_py : free_nav_pt_py;
    wire [15:0]             obs_pt_v;
    wire [15:0]             nav_pt_v = (page == PAGE_NAV_OBS) ? obs_pt_v : free_nav_pt_v;
    wire [9:0]              obs_car_px;
    wire [9:0]              nav_car_px = (page == PAGE_NAV_OBS) ? obs_car_px : free_nav_car_px;
    wire [9:0]              obs_car_py;
    wire [9:0]              nav_car_py = (page == PAGE_NAV_OBS) ? obs_car_py : free_nav_car_py;
    wire [9:0]              obs_cur_px;
    wire [9:0]              nav_cur_px = (page == PAGE_NAV_OBS) ? obs_cur_px : free_nav_cur_px;
    wire [9:0]              obs_cur_py;
    wire [9:0]              nav_cur_py = (page == PAGE_NAV_OBS) ? obs_cur_py : free_nav_cur_py;
    wire obs_goal_valid;
    wire [7:0] obs_goal_x, obs_goal_y;
    wire               nav_auto_valid;
    wire signed [31:0] nav_auto_center, nav_auto_diff;
    wire [3:0] planner_nav_state;
    wire planner_cmd_valid, planner_emergency_stop;
    wire [1:0] planner_target_line_dir;
    wire signed [31:0] planner_target_x, planner_target_y;
    wire obs_ctrl_done, obs_linear_active, obs_ctrl_fault;
    wire obs_auto_valid;
    wire signed [31:0] obs_auto_center, obs_auto_diff;
    wire inspection_owned,inspection_goal_valid,inspection_motion_enable,inspection_force_stop,inspection_pending_motion;
    wire [7:0] inspection_goal_x,inspection_goal_y,inspection_status;
    wire inspection_begin,inspection_commit,inspection_word_valid,inspection_stopped;
    wire inspection_word_ready,inspection_layer_valid,inspection_committed,inspection_error,inspection_layer_busy;
    wire [31:0] inspection_layer_epoch,inspection_layer_crc,inspection_layer_word;
    wire [15:0] inspection_begin_version,inspection_layer_version;
    wire [12:0] inspection_word_index;
    wire [3:0] inspection_error_code;
    wire [7:0] inspection_tx_data;
    wire inspection_tx_valid,inspection_tx_last,inspection_tx_ready;
    wire [31:0] inspection_session,inspection_task_id,inspection_context;
    reg inspection_auto_valid;
    reg signed [31:0] inspection_auto_center,inspection_auto_diff;
    wire obs_owns_control = planner_cmd_valid || planner_emergency_stop;
    wire nav_selected_valid = inspection_owned ? inspection_auto_valid : obs_owns_control ? obs_auto_valid : nav_auto_valid;
    wire signed [31:0] nav_selected_center = inspection_owned ? inspection_auto_center : obs_owns_control ? obs_auto_center : nav_auto_center;
    wire signed [31:0] nav_selected_diff = inspection_owned ? inspection_auto_diff : obs_owns_control ? obs_auto_diff : nav_auto_diff;
    // Same drive-code mapping as the reviewed MOT1 bridge: 30 << 2 = 120.
    // Clamp/register separately from navigation arithmetic and final muxes.
    always @(posedge i_clk_50m or negedge sys_rst_n)begin
        if(!sys_rst_n)begin inspection_auto_valid<=0;inspection_auto_center<=0;inspection_auto_diff<=0;end
        else if(inspection_owned && !inspection_force_stop)begin
            inspection_auto_valid<=obs_auto_valid;
            inspection_auto_center<=obs_auto_center>120?32'sd120:obs_auto_center< -120?-32'sd120:obs_auto_center;
            inspection_auto_diff<=obs_auto_diff;
        end else begin inspection_auto_valid<=0;inspection_auto_center<=0;inspection_auto_diff<=0;end
    end
    wire voice_owned, voice_auto_valid;
    wire signed [31:0] voice_auto_center, voice_auto_diff;
    wire auto_selected_valid;
    wire signed [31:0] auto_selected_center, auto_selected_diff;

    // Ownership persists after STOP/completion/fault. Never fall back to a
    // still-running navigation command when a voice action finishes.
    voice_nav_arbiter u_voice_nav_arbiter (
        .i_voice_owned(voice_owned && !inspection_owned), .i_voice_valid(voice_auto_valid),
        .i_voice_center(voice_auto_center), .i_voice_diff(voice_auto_diff),
        .i_nav_valid(nav_selected_valid), .i_nav_center(nav_selected_center),
        .i_nav_diff(nav_selected_diff), .o_valid(auto_selected_valid),
        .o_center(auto_selected_center), .o_diff(auto_selected_diff)
    );

    wire               nav_ctrl_active;      // 导航页 + 交互态
    wire signed [31:0] nav_dbg_ahead, nav_dbg_left;
    wire signed [15:0] nav_dbg_psi;
    wire [4:0]         nav_dbg_idx;
    wire [2:0]         nav_fsm_state;
    reg estop_latch;     // 急停锁存（S4 切换；ToF 触发也会置位）
    reg [2:0] menu_sel;  // 菜单【高亮选中项】（摇杆拨动改它，按确认才赋给 page）
                        // ★ 复位值统一取 PAGE_DEFAULT（= 数据页），见第 0 节

    // ★ 2026-10-19：当前是否停在「非避障导航」页。
    //   这一页摇杆被准心占用、确认键被"记录路径点"占用，所以第 3 节的菜单/切页逻辑
    //   要用它把 rc_menu_up / rc_menu_down / rc_confirm 全部屏蔽掉。
    //   ★ 必须在这里声明（组合读 page）：放到文件后面的例化区再声明，就是"先用后声明"，
    //     Verilog 会先隐式建一个 1 位 net、再与显式声明冲突而报错。
    wire ai_master,ai_chat,ai_record,ai_vision;
    wire [15:0] ai_flags;
    wire [7:0] ai_controls;
    wire [1:0] ai_mode,ai_selected;
    wire ai_inspection_open,ai_inspection_close;
    wire [1:0] ai_inspection_view;
    wire [7:0] ai_display_ack;
    wire ai_display_bank,ai_host_alive;
    wire nav_page_now = (page == PAGE_NAV_FREE);
    wire nav_page_any = (page == PAGE_NAV_FREE) || (page == PAGE_NAV_OBS);

    // ★ 2026-10-19：导航页"菜单锁"= 在导航页 **且** 处于交互态（ui_active）。
    //   按下返回键退出交互态后，菜单恢复可操作（摇杆能切、确认能进），
    //   这正是需求方说的"返回菜单选择（此时菜单不冻结）"。
    //   ★ `nav_lock` 必须**先声明、后 assign**：page 的 always 块在文件靠前就要用它，
    //     而 `free_nav_ui_active` 由后面的 nav_free_ctrl 例化产生。
    wire nav_lock;
    // ★ 2026-10-20：参数整定页信号（先声明后用 —— 页面状态机在文件靠前就要用）
    //   tune_active = 参数页处于「编辑态」：CH2 选行、CH4 调值、CH5 下复位、CH5 上退出编辑。
    //   tune_lock   = 停在参数页 **且** 处于编辑态 ⇒ 菜单键与确认键全部屏蔽。
    //   ★ 2026-10-20 上板二次修：退出编辑态**不换页**，page 仍停在 PAGE_TUNE ——
    //     与导航页退出 nav_ui 完全同构（只退交互态、不换页）。需求方原话：
    //     「退出参数页，右侧页面要仍留在参数页，跟退出导航的是相似的」。
    //     此时菜单键恢复可用，再按确认即重新进编辑态。
    wire tune_exit;
    wire tune_lock;
    reg  tune_active;
    assign tune_lock = (page == PAGE_TUNE) && tune_active;

    // ToF 急停（模块未做出：保持 0；将来接上后由 distance < D_STOP 触发）
    wire tof_estop = 1'b0;

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
    wire manual_ready,manual_takeover_brake;
    wire guard_arm_hint;
    wire takeover_off_center=(rc_throttle>12'sd100 || rc_throttle< -12'sd100 ||
                              rc_steer>12'sd100 || rc_steer< -12'sd100);
    wire arm_hint=guard_arm_hint || (manual_drive && rc_signal_ok && !manual_ready &&
        takeover_off_center && !(estop_latch | tof_estop | key4_press));
    rc_drive_guard #(.ARM_DEADZONE(100)) u_rc_guard (
        .clk(i_clk_50m), .rst_n(sys_rst_n), .i_manual(manual_drive),
        .i_signal_valid(rc_signal_ok && manual_ready), .i_estop(estop_latch | tof_estop | key4_press),
        .i_stop_pulse(key1_press), .i_throttle(rc_throttle),
        .o_enable(run_en), .o_arm_hint(guard_arm_hint)
    );

    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            estop_latch <= 1'b0;
            page        <= PAGE_DEFAULT;
            menu_sel    <= PAGE_DEFAULT;   // ★ 高亮与页面必须同源，否则确认一次就跳走
            tune_active <= 1'b0;
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
                menu_sel <= (menu_sel == PAGE_LAST) ? 3'd0 : (menu_sel + 3'd1);
                page     <= (page     == PAGE_LAST) ? 3'd0 : (page     + 3'd1);
            // ★ 2026-10-19：停在「非避障导航」页时，摇杆被准心占用、确认键被"记录路径点"
            //   占用，所以这里**必须**把两者屏蔽掉 —— 否则按确认会同时"记一个点"和"切页"。
            //   退出该页仍可用板载 S3（key3_press，不受此门控影响），等独立返回键接入后
            //   再由 nav_free_ctrl 的 i_back 走 SETUP→IDLE。
            // ★ 2026-10-20：参数整定页（PAGE_TUNE）的交互与导航页同构 ——
            //   CH2 上/下选行、CH4 左右调值、CH5 下复位该行、CH5 上退出。
            //   所以这里把菜单键与确认键**全部屏蔽**（tune_lock），只留返回键退编辑态；
            //   否则按确认会既"复位参数"又"切到菜单高亮项"，两个动作打架。
            //   ★ 退出编辑态**不换页**：page 与 menu_sel 此刻都已是 PAGE_TUNE，不赋值即保持。
            end else if (tune_exit) begin
                tune_active <= 1'b0;
            end else if (rc_menu_up && !nav_lock && !tune_lock && !ai_master) begin
                menu_sel <= (menu_sel == 3'd0) ? PAGE_LAST : (menu_sel - 3'd1);
            end else if (rc_menu_down && !nav_lock && !tune_lock && !ai_master) begin
                menu_sel <= (menu_sel == PAGE_LAST) ? 3'd0 : (menu_sel + 3'd1);
            end else if (rc_confirm && !nav_lock && !tune_lock && !ai_master) begin
                page <= menu_sel;                      // ★ 确认才进入该项页面（门控在下层）
                // 进参数页即进编辑态；切到别的页则退出编辑态
                tune_active <= (menu_sel == PAGE_TUNE);
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

    manual_takeover #(.CLK_HZ(CLK_HZ)) u_manual_takeover(
        .clk(i_clk_50m),.rst_n(sys_rst_n),.i_manual(manual_drive),
        .i_signal_valid(rc_signal_ok),.i_estop(estop_latch | tof_estop | key4_press | (|stall_vec)),
        .i_throttle(rc_throttle),.i_steer(rc_steer),.i_sample_valid(delta_valid),
        .i_delta0(dlt0),.i_delta1(dlt1),.i_delta2(dlt2),.i_delta3(dlt3),
        .o_ready(manual_ready),.o_brake(manual_takeover_brake));

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

    // ==================================================================
    // ★ 显示层转速极性（2026-10-17，与闭环反馈符号【解耦】）
    //
    //   为什么单独一层：轮速符号由 `ENC_INV_DIR_VEC`（编码器计数方向取反）决定，
    //   而那个参数**同时是 PID 的反馈符号** —— 直接改它会让该侧闭环变**正反馈**
    //   （误差与反馈同向 → 越推越猛），风险不可接受（见 `工作记录.md` §26.5）。
    //   所以这里把"屏幕上显示的正负"和"闭环用的正负"分开：
    //       闭环：完全不动，仍用 rpm0..rpm3
    //       显示：rpm*_disp = RPM_DISP_INV[i] ? -rpm* : rpm*
    //   屏幕上的轮速、中心速度、以及串口 CSV 的 tgt/act/err 一律用 `_disp` 版本，
    //   保证"屏幕看到的"与"串口上报的"永远一致。
    //
    //   【标定方法 · 零风险】接管档 + 油门中位（电机不转），用手把车整体往前推：
    //   看四个轮子旁的数字符号，哪个轮子"前滚时显示负"，就把对应位设 1。
    //     bit0 = A 右后 / bit1 = B 左后 / bit2 = C 右前 / bit3 = D 左前
    //   这一步只影响显示，不动闭环，可以反复试。
    //
    //   【为什么当前是 4'b0000】（2026-10-17 深夜极性重构后）
    //     上一轮曾用 4'b1111 把显示翻正，那是因为当时内部链是"负 = 物理前进"
    //     （`THROTTLE_REVERSE=1` 在遥控层翻转所致）。本轮把极性下沉到电机/编码器层
    //     （`INV_DIR_VEC=1111` + `ENC_INV_DIR_VEC=0101` + `THROTTLE/STEER_REVERSE=0`），
    //     内部已经天然"正 = 前进"，**显示与上报都不再需要翻转**，故回到 0000。
    //   ★ 保留这个参数不删：它是"逐轮独立"的保险丝。若换硬件后发现某一侧单独反
    //     （四个里三个正一个负），把那个轮的 bit 置 1 就能单独纠正，不必动闭环。
    //     同时它是 `report_err` 一致性的一部分：tgt 与 act 必须同符号体系。
    // ==================================================================
    localparam [3:0] RPM_DISP_INV = 4'b0000;
    wire signed [31:0] rpm0_disp = RPM_DISP_INV[0] ? -rpm0 : rpm0;
    wire signed [31:0] rpm1_disp = RPM_DISP_INV[1] ? -rpm1 : rpm1;
    wire signed [31:0] rpm2_disp = RPM_DISP_INV[2] ? -rpm2 : rpm2;
    wire signed [31:0] rpm3_disp = RPM_DISP_INV[3] ? -rpm3 : rpm3;

    // PID 参数（初值来自参数，可经串口 P/I/D 在线改）
    reg signed [15:0] kp_r, ki_r, kd_r;

    // ------------------------------------------------------------------
    // 参数整定页的 14 项参数（2026-10-20，一期：存 RAM，掉电丢失）
    //   ★ 位序：第 k 项在 tune_p[k*32 +: 32]，k = 0..13，与 rtl\ui\tune_ctrl.v
    //     的 o_sel、ui_render 的字段号、num_text 的字段号**四处一致**。
    //   ★ 这里只声明 wire，实例 u_tune_ctrl 在文件末尾（第 12c 节）—— Verilog
    //     允许先声明后实例化，但不能先使用后声明。
    // ------------------------------------------------------------------
    wire [447:0] tune_p;        // 参数值本身（给 nav_ctrl / speed_pid）
    wire [447:0] tune_disp;     // 显示值（已按单位换算，给 num_text）
    wire [3:0]   tune_sel;      // 当前选中行 0..13
    wire signed [31:0] tune_k_slow;   // = V_CRUISE×1024/D_SLOW，由 tune_ctrl 的除法器算

    //   速度环 PID 的两路写方（串口 / 页面）：靠"页面值变化沿"触发，避免每拍覆盖
    //   串口刚写入的值。复位初值与 tune_ctrl 的默认值保持一致（24 / 36 / 0）。
    // ★ 2026-10-20：速度环三项从参数索引 8/9/10 后移到 **9/10/11**
    //   （积分限幅插到索引 8，见 rtl\ui\tune_ctrl.v 的 p_def 注释）。
    //   变量名按语义改 —— 原 tune_p8_d 等已与新索引错位，留着必误读。
    reg [31:0] tune_kp_d, tune_ki_d, tune_kd_d;
    wire tune_kp_ev = (tune_p[319:288] != tune_kp_d);
    wire tune_ki_ev = (tune_p[351:320] != tune_ki_d);
    wire tune_kd_ev = (tune_p[383:352] != tune_kd_d);
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tune_kp_d <= 32'd24;
            tune_ki_d <= 32'sd36;
            tune_kd_d <= 32'd0;
        end else begin
            tune_kp_d <= tune_p[319:288];
            tune_ki_d <= tune_p[351:320];
            tune_kd_d <= tune_p[383:352];
        end
    end
    chassis_ctrl #(
        .PWM_BITS     (PWM_BITS),
        .KP_INIT      (KP_INIT),
        .KI_INIT      (KI_INIT),
        .KD_INIT      (KD_INIT)
    ) u_chassis (
        .clk          (i_clk_50m),
        .rst_n        (sys_rst_n),
        .i_enable     (manual_drive ? run_en : (auto_selected_valid && rc_signal_ok && cursor_mode)),
        .i_estop      (estop_latch | key4_press | tof_estop | !rc_signal_ok | manual_takeover_brake | (!manual_drive && (planner_emergency_stop | inspection_force_stop))),
        .i_manual     (chassis_manual),
        .i_steer      (steer_in),
        .i_throttle   (throttle_in),
        // 自动导航：来自 u_nav_ctrl。
        //   ★ `chassis_ctrl` 内部 `i_manual ? man_* : (i_auto_valid ? auto_* : 0)`
        //     已经实现了"手动压制导航"（决策 A3），这里不需要再加门控。
        //   ★ 暂停时 u_nav_ctrl 给 valid=1、center=diff=0 → 四路 speed_pid 主动把
        //     轮速拉到 0（主动制动），比"断 PWM 靠滑行"更能保证真的停住。
        .i_auto_valid (auto_selected_valid),
        .i_auto_center(auto_selected_center),
        .i_auto_diff  (auto_selected_diff),
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
        // ===== 堵转保护参数（2026-10-17 上板实测后大幅放宽）=====
        // 触发场景（需求方实测）：**车停下来时会误触发堵转**。
        //   机理：停车后 tgt = 0，但纯积分 PID（Ki = 0.25）的**积分残留**使 duty 仍 ≥ DUTY_TH，
        //   而车此时净位移为 0 → 宽限期一过就被判堵转并**锁存**（需 S2 清除）。
        //   本轮按需求方"大幅放宽"的要求，四个参数一起放大（有效门槛见下方换算）。
        //   ⚠ 根治办法不是调阈值，而是让 tgt = 0 时该侧 duty 归零（见 doc\工作记录.md §27.4）。
        //     ★ 2026-10-17：该根治方案**曾实现并通过仿真**，但按需求方要求**已完整还原**
        //       （`git checkout d405f82 -- rtl/motor/chassis_ctrl.v`），当前**只有阈值放宽生效**。
        //       若停车误判复发，直接照 §27.4 的 diff 重新启用即可。
        //
        // 换算（61440 计数/车轮转、10 ms/窗口）：
        //   有效判定门槛 = MOVE_TH / (WINDOWS × 10 ms) = 4000 / 400 ms ≈ 9.8 rpm
        //   正常行驶 300 rpm、纯转向 60 rpm 都远高于它 → 不会误报；
        //   真堵转时净位移≈0（振动互相抵消）→ 仍能抓到。
        .STALL_STARTUP_WINDOWS (100),    // 起步宽限 50 → 100 窗口（500 ms → 1000 ms）
        .STALL_WINDOWS         (40),     // 判定窗口 20 → 40 个（200 ms → 400 ms）
        .STALL_DUTY_TH         (400),    // 驱动意图门槛 200 → 400（≈19.5 % of 2047）
        .STALL_MOVE_TH         (4000)    // 净位移门槛 300 → 4000 计数（≈23.4° 车轮角）
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
    wire [7:0] ascii_rx_data, motion_rx_data;
    wire ascii_rx_valid, ascii_rx_abort, motion_rx_valid;

    // The existing UART is shared with PID tuning; binary payload/CRC must
    // never reach the ASCII parser (e.g. a CRC byte 0x78 is ASCII clear-stall).
    uart_command_router #(.CLK_FREQ_HZ(CLK_HZ)) u_uart_command_router (
        .i_clock(i_clk_50m), .i_reset_n(sys_rst_n),
        .i_rx_data(rx_data), .i_rx_valid(rx_valid),
        .o_ascii_data(ascii_rx_data), .o_ascii_valid(ascii_rx_valid),
        .o_ascii_abort(ascii_rx_abort),
        .o_motion_data(motion_rx_data), .o_motion_valid(motion_rx_valid)
    );

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
        .i_abort      (ascii_rx_abort),
        .i_rx_data    (ascii_rx_data),
        .i_rx_valid   (ascii_rx_valid),
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
            // ★ 2026-10-20：速度环 PID 现在有两个写方 —— 串口调参（P/I/D 命令）
            //   与参数整定页的第 8/9/10 项。**串口优先**（它是一次性命令，改完即止），
            //   页面在"值变化的下一拍"跟随（tune_pN_ev），不会每拍覆盖串口刚写进去的值。
            //   ★ 已知偏差：页面显示不会自动跟随串口改的值（tune_ctrl 在 sys 域独立
            //     维护自己的副本），串口改完再看页面会看到旧值。两个入口本就分属调试
            //     与正式两条路，不建议混用；要统一得把 kp_r 反灌给 tune_ctrl，本期不做。
            if (cmd_kp_valid)      kp_r <= cmd_kp;
            else if (tune_kp_ev)   kp_r <= tune_p[303:288];
            if (cmd_ki_valid)      ki_r <= cmd_ki;
            else if (tune_ki_ev)   ki_r <= tune_p[335:320];
            if (cmd_kd_valid)      kd_r <= cmd_kd;
            else if (tune_kd_ev)  kd_r <= tune_p[367:352];

            count_clear_r <= (key2_press | cmd_count_clear) & ~obs_run;
            stall_clear_r <= key2_press | cmd_stall_clear;
            yaw_clear_r   <= (key2_press | cmd_count_clear) & ~obs_run;   // S2 同时清零航向角
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

    // ---- ★ 2026-10-21：零偏标定失败也按「IMU 不可用」上报 ----
    // 原实现只认 imu_err_flag（停在 S_ERROR），但零偏标定超时失败时状态机已回到
    //   S_RUN，屏幕完全无提示：数据照常输出，只是 CALIB_APPLY 分支不生效（bias=0），
    //   yaw 带着零偏持续漂移，现象上只能靠事后怀疑「是不是没标定成」。
    // 现在与初始化失败走同一条显示路径（红字「imu初始化错误」+ 中心速度字段显示诊断码）。
    wire               imu_fail_any = imu_err_flag | imu_calib_fail;

    // 诊断码低位区分来源：
    //   初始化失败 → err_code（≥1，01=CHIP_ID / 02=配置校验，屏幕读作「S.01」「S.02」）；
    //   标定失败   → 固定填 90（此时 state=S_RUN=14、err_code 无意义），屏幕读作「14.90」。
    // 注意两者语义不同：14.90 表示 **IMU 通信是好的，只是零偏没标定出来**。
    wire [7:0]         imu_err_dbg  = imu_calib_fail ? 8'd90 : imu_err_code;

    // ---- IMU 初始化失败自恢复看门狗（2026-10-19 新增）----
    // 背景：vendor 的 imu_top 进 S_ERROR 后**没有任何出口**（该状态只置 err_flag_r，
    //   不做状态转移），一旦上电初始化偶发失败（CHIP_ID 读不到 0x24，重试
    //   CHK_RETRY_MAX 次后报 err_code=1），就会永久停住，必须断电才能恢复。
    //   上板现象：航向角恒 0、角速度恒 0，重新上电后正常。
    // 做法：**不修改 vendor 快照**，在顶层给 u_imu 一个局部复位：
    //   检测到 err_flag 持续 → 等 IMU_WD_MS 毫秒（给芯片上电/自身恢复留时间）
    //   → 拉低 imu_local_rst_n IMU_WD_RST_US 微秒 → 放开 → imu_top 从
    //   S_PWR_WAIT 重新走完整初始化。成功后 err_flag 自动清 0，看门狗回到监视态。
    // 注意：只复位 IMU，不复位 u_tilt / u_yaw —— 航向角保持不清零，
    //   避免 IMU 重连瞬间航向突跳到 0。
    localparam integer IMU_WD_MS     = 500;   // 失败后等待多少毫秒再硬复位
    localparam integer IMU_WD_RST_US = 20;    // 复位保持多少微秒
    localparam integer IMU_WD_MS_DIV = CLK_HZ / 1000;            // 1 ms 的时钟拍数
    localparam integer IMU_WD_US_DIV = CLK_HZ / 1000 / 1000;     // 1 µs 的时钟拍数

    reg        imu_local_rst_n;
    reg [15:0] imu_wd_ms_div;
    reg [15:0] imu_wd_ms_cnt;
    reg [9:0]  imu_wd_us_cnt;

    imu_top #(
        .CLK_FREQ_HZ   (CLK_HZ),
        .SCK_HALF_DIV  (25),            // ★ 1 MHz，显式覆盖源码默认 3
        .SAMPLE_DIV    (50_000),        // 1 kHz 读取
        .PWR_WAIT_MS   (20),
        .CHK_RETRY_MAX (10),
        .CALIB_DEV_TH  (100)            // 抖动判据 1°/s；标定失败就调大它
    ) u_imu (
        .i_clock      (i_clk_50m),
        .i_reset_n    (imu_local_rst_n),   // ★ 局部复位：失败可被看门狗重置（见上）
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

    // ---- 看门狗本体（说明见上面的参数区）----
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            // ★ 2026-10-21：系统复位同样拉低 IMU 局部复位，让 imu_top 从 S_IDLE
            //   重走初始化 + 零偏标定。原实现这里置 1（= 复位期间把 IMU 放开），
            //   导致按 S0 复位键后 IMU 保持旧状态、标定失败无法重试。
            //   代价：复位后要等约 3.2 秒（20 ms 上电等待 + 配置写入 + 3 秒标定）
            //   才有数据，期间 o_data_valid=0、界面数值保持 0。
            imu_local_rst_n <= 1'b0;
            imu_wd_ms_div   <= 16'd0;
            imu_wd_ms_cnt   <= 16'd0;
            imu_wd_us_cnt   <= 10'd0;
        end else if (!imu_local_rst_n) begin
            // 复位保持阶段：计满 IMU_WD_RST_US 微秒后放开，imu_top 重走初始化
            if (imu_wd_us_cnt >= (IMU_WD_US_DIV - 1)) begin
                imu_local_rst_n <= 1'b1;
                imu_wd_us_cnt   <= 10'd0;
                imu_wd_ms_div   <= 16'd0;
                imu_wd_ms_cnt   <= 16'd0;
            end else begin
                imu_wd_us_cnt <= imu_wd_us_cnt + 10'd1;
            end
        end else if (imu_err_flag) begin
            // 错误态：累计 IMU_WD_MS 毫秒后拉低局部复位
            if (imu_wd_ms_div >= (IMU_WD_MS_DIV - 1)) begin
                imu_wd_ms_div <= 16'd0;
                if (imu_wd_ms_cnt >= (IMU_WD_MS - 1)) begin
                    imu_wd_ms_cnt   <= 16'd0;
                    imu_local_rst_n <= 1'b0;
                    imu_wd_us_cnt   <= 10'd0;
                end else begin
                    imu_wd_ms_cnt <= imu_wd_ms_cnt + 16'd1;
                end
            end else begin
                imu_wd_ms_div <= imu_wd_ms_div + 16'd1;
            end
        end else begin
            // 正常态：计时归零，保持监视
            imu_wd_ms_div <= 16'd0;
            imu_wd_ms_cnt <= 16'd0;
            imu_wd_us_cnt <= 10'd0;
        end
    end

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

    // ★ 2026-10-19 时序修复：gyro 补偿值流水一级
    //   原路径 gain_r -> 32x32 乘法器 -> o_gyro_z -> u_yaw 内部组合 -> acc 累加器，
    //   2026-10-19 综合里 slack = -0.992（i_clk_50m 同域，数据延迟 20.17 ns）。
    //   把补偿值打一拍，拆成两段：
    //     ① gain_r -> 乘法器 -> gyro_z_comp_r/D   （约 8~10 ns）
    //     ② gyro_z_comp_r/Q -> u_yaw -> acc/D     （约 6~8 ns）
    //   ★ 数据配对：i_gyro_z 与 i_sample_valid 必须**同步**延后一拍，
    //     否则 u_yaw 会在 data_valid 当拍采到上一帧的补偿值。所以 valid 也打一拍。
    reg signed [31:0] gyro_z_comp_r;
    reg               imu_data_valid_d1;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            gyro_z_comp_r     <= 32'sd0;
            imu_data_valid_d1 <= 1'b0;
        end else begin
            gyro_z_comp_r     <= gyro_z_comp;
            imu_data_valid_d1 <= imu_data_valid;
        end
    end
    // 注：yaw_clear_r 的声明放在文件前面的"清零信号"处（串口 always 块要用它）

    yaw_integrate #(
        .CLK_FREQ_HZ (CLK_HZ),
        .INT_RATE_HZ (1024),            // 2 的幂 → dt = 2⁻¹⁰ s，每拍零乘法
        .DEADZONE_EN (1),
        .DEADZONE_TH (30),              // 静止死区 0.3°/s；上板后按漂移现象调
        .SUM_PIPELINE(1)               // 分开积分加法与回绕，结果仅延迟20ns
    ) u_yaw (
        .i_clock        (i_clk_50m),
        .i_reset_n      (sys_rst_n),
        .i_gyro_z       (gyro_z_comp_r),        // 已做安装倾斜补偿（流水一级，见上）
        .i_sample_valid (imu_data_valid_d1),    // ★ 契约③：与 gyro 同步延后 1 拍
        .i_enable       (1'b1),             // 常使能（航向保持是后续功能）
        .i_clear        (yaw_clear_r),      // S2 / 串口 X 命令清零
        .o_yaw          (yaw_raw),
        .o_yaw_valid    (yaw_valid),
        .o_deadzone_hit (),
        .o_gz_eff       ()
    );

    // ---- 航向角符号（★★ 2026-10-19 需求方重新约定：改用标准数学坐标系 ★★）----
    //   ★ 这是**需求变更，不是修 bug**：2026-10-18 那套（朝 +y、右为 +x、右转为正）
    //     已上板实测确认正确；本次是把坐标系整体换成标准数学约定，旧实现在旧约定下没有错。
    //
    //   本车坐标约定（**现行，真源**）：
    //     · 初始化时车头朝 **+x**
    //     · 车辆**左侧**为 **+y**
    //     · **向左转（逆时针）⇒ 航向角增大**
    //   ⇒ 车头单位向量 = ( cos(yaw), sin(yaw) )
    //      yaw = 0      → (+1,  0) 朝 +x
    //      yaw = +90°   → ( 0, +1) 朝 +y（左转 90°）
    //      yaw = ±180°  → (−1,  0) 朝 −x
    //      yaw = −90°   → ( 0, −1) 朝 −y（右转 90°）
    //   ★ 与旧约定相比，这是**镜像 + 旋转**的整体换基。
    //     `rtl\odom\odom.v` 的投影公式必须**同时**改（sin/cos 互换）：
    //       · 只改这里不改投影 ⇒ 车头向量成 (sin, cos)，前进落到 +y 而非 +x，且左右反；
    //       · 只改投影不改这里 ⇒ 车头向量成 (cos, −sin)，绕 x 轴镜像，左变右。
    //
    //   IMU660RA 的 Z 轴朝上（右手系）时**逆时针旋转的角速度为正**，与"左转增大"
    //   方向一致，所以这里**不取反**（旧约定要取反，是因为旧约定用的是顺时针为正）。
    //
    //   ⚠ 若实测"左转时航向角变小"，说明 IMU 装反/芯片定义不同，把 YAW_SIGN 改成 −1 ——
    //     只改这一处，`yaw_deg` 与 `u_odom.i_yaw` 两个消费者都跟着变。
    localparam integer YAW_SIGN = +1;            // +1 = 左转（逆时针）增大（当前）；−1 = 左转减小
    wire signed [31:0] yaw_raw_s = (YAW_SIGN < 0) ? -yaw_raw : yaw_raw;

    // 航向角显示用整数度：yaw 单位 0.01°，÷100 ≈ (×10486) >>> 20
    //   （1/100 = 0.01；0.01 × 2²⁰ = 10485.76 → 取 10486，误差 < 0.003%）
    reg signed [31:0] yaw_deg;
    reg imu_degree_sample;
    always @(posedge i_clk_50m or negedge sys_rst_n)begin
        if(!sys_rst_n)begin yaw_deg<=0;imu_degree_sample<=0;end
        else begin
            yaw_deg<=(yaw_raw_s * 10486) >>> 20;
            imu_degree_sample<=imu_data_valid_d1;
        end
    end

    // Voice motion only uses the existing sys-domain yaw and fresh IMU
    // samples. yaw_valid alone is an enable flag, not a sample heartbeat.
    reg voice_imu_calibrated;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) voice_imu_calibrated <= 1'b0;
        else if (!imu_local_rst_n || !imu_init_done || imu_calib_busy || imu_fail_any)
            voice_imu_calibrated <= 1'b0;
        else if (imu_calib_done) voice_imu_calibrated <= 1'b1;
    end
    wire voice_imu_healthy = imu_local_rst_n && imu_init_done &&
                             voice_imu_calibrated && !imu_calib_busy && !imu_fail_any;
    // No automatic release to an old UI route: voice patrol start is not yet
    // wired to a fresh, explicitly confirmed waypoint task in this release.
    wire voice_nav_release_ready = 1'b0;
    // SWB on the existing CH6/J6.5 starts/stops recording. No extra PWM pins.
    ai_rc_control #(.CLK_HZ(CLK_HZ),.INSPECTION_ENABLE(1)) u_ai_control(
        .clk(i_clk_50m),.rst_n(sys_rst_n),
        .i_enter_confirm(rc_confirm && menu_sel==PAGE_AI && !nav_lock && !tune_lock && !ai_master),
        .i_ai_page(page==PAGE_AI),.i_back(rc_back | key1_press | count_clear_r | yaw_clear_r),
        .i_select_prev(rc_aux_up && page==PAGE_AI),.i_select_next(rc_aux_down && page==PAGE_AI),
        .i_feature_confirm(rc_confirm && page==PAGE_AI && ai_master),
        .i_rc_valid(rc_signal_ok),.i_manual(manual_drive),
        .i_host_alive(ai_host_alive),
        .i_estop(estop_latch | key1_press | key4_press | count_clear_r | yaw_clear_r | tof_estop | (planner_emergency_stop && !inspection_owned) | (|stall_vec)),
        .i_swb_pwm(i_rc_pwm[4]),
        .i_inspection_close(ai_inspection_close || (rc_back && !inspection_owned)),
        .o_inspection_open(ai_inspection_open),.o_inspection_view(ai_inspection_view),
        .o_master(ai_master),.o_chat(ai_chat),.o_record(ai_record),.o_vision(ai_vision),
        .o_mode(ai_mode),.o_selected(ai_selected),
        .o_flags(ai_flags),.o_controls(ai_controls));

    reg legacy_nav_run_d;
    always @(posedge i_clk_50m or negedge sys_rst_n)
        if(!sys_rst_n)legacy_nav_run_d<=0;else legacy_nav_run_d<=nav_run;
    wire ai_local_release=!ai_master && nav_run && !legacy_nav_run_d &&
        (page==PAGE_NAV_FREE || page==PAGE_NAV_OBS) && rc_signal_ok && !manual_drive &&
        !(estop_latch | key4_press | tof_estop | planner_emergency_stop | (|stall_vec));
    // Separate the multiplier's registered DSP output from voice turn-state
    // comparisons. Keep sample and angle coherent; live health still revokes
    // immediately, independently of this one-clock measurement pipeline.
    reg signed [31:0] voice_yaw_r /* synthesis syn_preserve=1 */;
    reg voice_yaw_valid_r,voice_yaw_sample_r;
    always @(posedge i_clk_50m or negedge sys_rst_n)begin
        if(!sys_rst_n)begin voice_yaw_r<=0;voice_yaw_valid_r<=0;voice_yaw_sample_r<=0;end
        else begin voice_yaw_r<=yaw_deg;voice_yaw_valid_r<=yaw_valid;voice_yaw_sample_r<=imu_degree_sample;end
    end
    voice_motion_bridge #(.CLK_HZ(CLK_HZ), .YAW_SIGN(1),.LOCAL_RELEASE_EN(1)) u_voice_motion (
        .clk(i_clk_50m), .rst_n(sys_rst_n),
        .i_rx_data(motion_rx_data), .i_rx_valid(motion_rx_valid),
        .i_enable(cursor_mode && rc_signal_ok && ai_master && ai_mode==0 && !ai_record),
        .i_estop(estop_latch | key1_press | key4_press | count_clear_r | yaw_clear_r | tof_estop | planner_emergency_stop | (|stall_vec)),
        .i_manual(manual_drive), .i_yaw_deg(voice_yaw_r), .i_yaw_valid(yaw_valid && voice_yaw_valid_r),
        .i_imu_sample_valid(voice_yaw_sample_r), .i_imu_healthy(voice_imu_healthy),
        .i_nav_release_ready(voice_nav_release_ready),
        .i_local_release(ai_local_release),
        .o_owned(voice_owned), .o_auto_valid(voice_auto_valid),
        .o_auto_center(voice_auto_center), .o_auto_diff(voice_auto_diff),
        .o_active(), .o_last_seq(), .o_last_cmd(), .o_last_flags(),
        .o_cmd_error(), .o_patrol_start(), .o_patrol_stop()
    );

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
    reg [PWM_BITS-1:0] duty0_report;
    reg dir0_report;
    always @(posedge i_clk_50m or negedge sys_rst_n)
        if(!sys_rst_n)begin duty0_report<=0;dir0_report<=0;end
        else begin duty0_report<=duty0_raw;dir0_report<=dir_vec[0];end
    wire signed [31:0]  report_tgt   = (tgt_left * 100) >>> 10;   // delta → rpm（÷10.24）
    wire signed [31:0]  report_duty  = dir0_report ? -$signed({1'b0, duty0_report})
                                                  :  $signed({1'b0, duty0_report});
    wire signed [31:0]  report_err   = report_tgt - rpm0_disp;
    wire signed [31:0]  report_flags = {28'd0, (|pid_sat), cmd_error, (|stall_vec), run_en};

    wire [7:0] csv_stream_data;
    wire csv_stream_valid,csv_stream_ready;
    ai_uart_tx #(.CLK_HZ(CLK_HZ),.BAUD_RATE(BAUD_DEBUG),.INSPECTION_ENABLE(1)) u_ai_uart_tx(
        .clk(i_clk_50m),.rst_n(sys_rst_n),.i_csv_data(csv_stream_data),
        .i_csv_valid(csv_stream_valid),.o_csv_ready(csv_stream_ready),
        .i_flags({ai_display_ack,2'd0,ai_display_bank,ai_flags[4:0]}),.i_controls(ai_controls),
        .i_ai_link_active(ai_host_alive),.i_inspection_data(inspection_tx_data),.i_inspection_valid(inspection_tx_valid),
        .i_inspection_last(inspection_tx_last),.o_inspection_ready(inspection_tx_ready),.o_uart_tx(o_uart_tx));
    speed_report #(
        .CLK_FREQ_HZ    (CLK_HZ),
        .BAUD_RATE      (BAUD_DEBUG),
        .REPORT_DIV     (1_000_000),    // 50 MHz / 1_000_000 = 50 Hz
        // A2：12 列最长一行约 96 字符，32 字节 FIFO 会让状态机频繁等空位，
        //     加大到 128 字节（一行可整行缓冲，BSRAM 开销可忽略）
        .FIFO_DEPTH_LOG2(7),.EXTERNAL_UART(1)
    ) u_report (
        .i_clock   (i_clk_50m),
        .i_reset_n (sys_rst_n),
        .i_tgt     (report_tgt),
        .i_act     (rpm0_disp),
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
        .o_uart_tx (),.o_stream_data(csv_stream_data),.o_stream_valid(csv_stream_valid),
        .i_stream_ready(csv_stream_ready)
    );

    // 7. 显示：HDMI 时序 + 中文界面
    // ==================================================================
    wire        pixel_clk, pixel_req, frame_start, frame_sync, video_active, pll_lock;
    wire [11:0] pixel_x, pixel_y;
    wire [23:0] ui_rgb_raw;     // ui_render 生成的 UI 像素
    wire [23:0] ui_rgb;         // 经 cam_overlay 二选一后的最终像素（送 u_hdmi）

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
    reg [2:0] page_s, menu_s;
    reg [1:0] mode_s;
    //   ★ 2026-10-16：`run_s`（= chassis_running）已删 —— 它唯一的下游是 ui_render 的
    //     i_run，而该端口已删（模块内从不使用）。sys 域这一拍现在只剩 estop/sig/arm。
    reg       estop_s, sig_s, arm_s;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            page_s  <= PAGE_DEFAULT; mode_s  <= 2'd0; menu_s <= PAGE_DEFAULT;
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
    reg [2:0] page_ps1, page_ps2, menu_ps1, menu_ps2;
    reg [1:0] mode_ps1, mode_ps2;
    reg       estop_ps1, estop_ps2, sig_ps1, sig_ps2;
    reg       arm_ps1, arm_ps2;                          // A1："请回中位"提示
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            page_ps1  <= PAGE_DEFAULT; page_ps2  <= PAGE_DEFAULT;
            mode_ps1  <= 2'd0; mode_ps2  <= 2'd0;
            menu_ps1  <= PAGE_DEFAULT; menu_ps2  <= PAGE_DEFAULT;
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

    // ---- 非避障导航页状态 → pixel 域（两级同步）----
    //   ★ 点表只在按确认时整体变化一次，是典型的"慢变量"，两级同步足够。
    //   ★ 380 位同步器 = 760 个触发器（占 GW5AT-138 的 FF 不到 1%），
    //     换来 pixel 域在 1 拍内完成 16 个点的比较，比"每帧读 RAM"简单得多。
    reg [9:0]   navc_ps1_x, navc_ps2_x, navc_ps1_y, navc_ps2_y;
    reg [9:0]   navcar_ps1_x, navcar_ps2_x, navcar_ps1_y, navcar_ps2_y;
    reg [159:0] navpt_ps1_x, navpt_ps2_x, navpt_ps1_y, navpt_ps2_y;
    reg [15:0]  navpv_ps1, navpv_ps2;
    reg [2:0]   navaux_ps1, navaux_ps2;
    reg         navrun_ps1, navrun_ps2;
    reg         menu_dim_ps1, menu_dim_ps2; // 导航/AI交互态，共用菜单冻结样式
    reg         navpz_ps1, navpz_ps2;      // 导航暂停
    // ★ 2026-10-19：IMU 初始化失败 → 数据页「航向角」上方红字提示。
    //   源 `imu_err_flag` 本身就是 `u_imu` 里的纯 DFF 输出（`i_clk_50m` 域），
    //   直接进两级同步器即可，不需要额外打一拍。
    reg         imufail_ps1, imufail_ps2;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            navc_ps1_x <= 10'd0; navc_ps2_x <= 10'd0;
            navc_ps1_y <= 10'd0; navc_ps2_y <= 10'd0;
            navcar_ps1_x <= 10'd0; navcar_ps2_x <= 10'd0;
            navcar_ps1_y <= 10'd0; navcar_ps2_y <= 10'd0;
            navpt_ps1_x <= 160'd0; navpt_ps2_x <= 160'd0;
            navpt_ps1_y <= 160'd0; navpt_ps2_y <= 160'd0;
            navpv_ps1 <= 16'd0; navpv_ps2 <= 16'd0;
            navaux_ps1 <= 3'd0; navaux_ps2 <= 3'd0;
            navrun_ps1 <= 1'b0; navrun_ps2 <= 1'b0;
            menu_dim_ps1 <= 1'b0; menu_dim_ps2 <= 1'b0;
            navpz_ps1  <= 1'b0; navpz_ps2  <= 1'b0;
            imufail_ps1 <= 1'b0; imufail_ps2 <= 1'b0;
        end else begin
            navc_ps1_x <= nav_cur_px;      navc_ps2_x <= navc_ps1_x;
            navc_ps1_y <= nav_cur_py;      navc_ps2_y <= navc_ps1_y;
            navcar_ps1_x <= nav_car_px;    navcar_ps2_x <= navcar_ps1_x;
            navcar_ps1_y <= nav_car_py;    navcar_ps2_y <= navcar_ps1_y;
            navpt_ps1_x <= nav_pt_px;      navpt_ps2_x <= navpt_ps1_x;
            navpt_ps1_y <= nav_pt_py;      navpt_ps2_y <= navpt_ps1_y;
            navpv_ps1 <= nav_pt_v;         navpv_ps2 <= navpv_ps1;
            navaux_ps1 <= nav_aux_sel;     navaux_ps2 <= navaux_ps1;
            navrun_ps1 <= nav_run;         navrun_ps2 <= navrun_ps1;
            menu_dim_ps1 <= nav_ui_active || ai_master; menu_dim_ps2 <= menu_dim_ps1;
            navpz_ps1  <= nav_paused;      navpz_ps2  <= navpz_ps1;
            imufail_ps1 <= imu_fail_any;   imufail_ps2 <= imufail_ps1;
        end
    end

    // ---- 实时数值链路：sys 域格式化 → 双口 RAM → pixel 域渲染 ----
    // 目标轮速用与 chassis_ctrl 相同的换算：rpm = delta × 100 >> 10（= ÷10.24）
    wire signed [31:0] tgt_left_rpm  = (tgt_left  * 100) >>> 10;
    wire signed [31:0] tgt_right_rpm = (tgt_right * 100) >>> 10;

    wire       dyn_we;
    wire [6:0] dyn_waddr;
    wire [7:0] dyn_wdata;
    wire [6:0] dyn_raddr;
    wire [7:0] dyn_rdata;

    // ★ 2026-10-16 字段按需求原文 §29 的数据页布局重排（9 项）：
    //     0 航向角 / 1~4 四轮转速 A,B,C,D / 5 角速度 / 6 中心速度 / 7,8 航位 x,y
    //   车轮旁显示的是【车轮转速】（rpm，±400），不是编码器累计计数。
    //   ★ 2026-10-18：num_text 字段宽 4 → 5（"符号 + 4 位数字"，9×5 = 45 字符 < 64）。
    //     轮速/航向角仍按 4 字符渲染（ui_render 从字段 base+1 读），列表数值按 5 字符。
    //
    // ---- 中心速度：直接对原始 counts 求和，最后一次性换算（2026-10-18 改）----
    //   为什么不再用 (rpm0+rpm1+rpm2+rpm3)>>>2：
    //     rpm 已经过一次 >>10 截断，四轮的截断偏差**同向**（都向下），平均压不掉。
    //     量化基准：1 rpm = 10.24 counts / 10ms。
    //     例：四轮真实 100 rpm、窗口边界抖动成 1023/1024/1024/1025 counts
    //         → rpm = 99/100/100/100 → 平均 99 → 偏低 1 rpm ≈ 0.34 cm/s，
    //         正好盖住 0.01 m/s 的显示台阶（合计 0~0.6 cm/s 的系统偏差）。
    //   改为 sum_delta（4 路原始 counts，无截断）→ 乘系数 → 只右移一次：
    //   ★★ 2026-10-18（需求方实测参数更新）：**轮半径 32 mm ⇒ 轮径 D = 64 mm**。
    //      初版按 D = 65 mm 推的 8717 与正确值只差 1.6%（可忽略），此处按 64 mm 精确化。
    //     系数 8580/2^20 = 0.0081825 cm/s 每个 sum-count
    //       1 count/10ms = 100 count/s ÷ 61440 count/转 = 0.00162760 rev/s
    //       × πD（D = 64 mm → 0.201062 m）  = 3.27259e-4 m/s = 0.0327259 cm/s
    //       ÷ 4（四轮平均）                  = 0.00818148 cm/s  ← 与 8580/2^20 差 0.011%
    //     量化台阶 0.0082 cm/s，比 1 cm/s 的显示台阶小 122 倍 → 0.01 位平稳。
    //   ★ 轮径若再变，系数 = (πD × 100) / (61440 × 4) × 2^20
    //     **R=32mm ⇒ D=64mm → 8580**；D=39.8mm → 5334；D=65mm → 8717
    //   溢出：sum 最大 4×4096 = 16384，×8580 = 1.41e8 < 2^32 ✓
    //   ★ 轮速显示（i_v1..i_v4）用 rpm，与轮径无关，不受影响；
    //     显示极性只补到这里（dlt* 只含 ENC_INV_DIR_VEC，不含 RPM_DISP_INV）。
    wire signed [31:0] dlt0_disp = RPM_DISP_INV[0] ? -dlt0 : dlt0;
    wire signed [31:0] dlt1_disp = RPM_DISP_INV[1] ? -dlt1 : dlt1;
    wire signed [31:0] dlt2_disp = RPM_DISP_INV[2] ? -dlt2 : dlt2;
    wire signed [31:0] dlt3_disp = RPM_DISP_INV[3] ? -dlt3 : dlt3;
    wire signed [31:0] sum_delta  = dlt0_disp + dlt1_disp + dlt2_disp + dlt3_disp;
    wire signed [31:0] center_cms = (sum_delta * 32'sd8580) >>> 20;   // 单位 0.01 m/s（D=64mm）

    // ------------------------------------------------------------------
    // ★★ 临时诊断口（2026-10-19）：定位上板现象「偏航角一直为 0」★★
    //   静态排查已确认 yaw 数据链（u_imu → u_tilt → 流水级 → u_yaw → yaw_deg）
    //   在 23:03 版与 18:12 版之间**一行未改**，无法从源码定位，只能上板读内部状态。
    //   做法：借「中心速度」那一行的显示位输出 IMU 内部状态，读数规则：
    //     屏幕「中心速度」= S.EE  →  imu_top 状态机 state = S，错误码 err = EE
    //     例：4.00 → state=4、err=0；  0.00 → state=0（停在上电态，说明 IMU 没跑起来）
    //   判读口径：
    //     state 停在 S_RUN 且 err=0  → IMU 通信正常，问题在流水级/积分器
    //     state 不是 S_RUN           → IMU 初始化或配置失败（SPI/上电/时序）
    //   定位完成后删除本段，并把 `.i_v6` 恢复成 `center_cms`。
    // ------------------------------------------------------------------
    wire signed [31:0] imu_diag = ($signed({1'b0, imu_state_dbg}) * 32'sd100)
                                + $signed({1'b0, imu_err_dbg});

    // ★ 2026-10-19：诊断值只在失败时占用该字段。
    //   正常时显示真实中心速度（否则这一行永远是 14.00，等于把功能弄坏）。
    //   失败时同时出现红字「imu初始化错误」，两者一起看：红字说明原因，
    //   数值说明卡在哪个状态（S=state、EE=err_code），便于区分 CHIP_ID 失败(01)
    //   与 INTERNAL_STATUS 失败(02)。
    // ★ 2026-10-21：标定失败也走这条路径（见上面的 imu_fail_any / imu_err_dbg）。
    //   此时数值是 **14.90**，与初始化失败的 S.01/S.02 不同 —— 14.90 说明通信正常、
    //   只是零偏没标定出来，应当去查 CALIB_DEV_TH / 上电静置 / PWR_CTRL 后的等待时间。
    wire signed [31:0] center_cms_show = imu_fail_any ? imu_diag : center_cms;

    // ---- 角速度：gyro_tilt_comp 输出的单位是 **0.01 °/s** ----
    //   ★ 2026-10-18 修正：原来写 `gyro_z_comp >>> 16`，把 0.01 °/s 当成了 Q16
    //     → 100 °/s（内部值 10000）>>16 = 0，要 655 °/s 才显示 1，
    //     这就是"屏幕角速度只显示 0 或 1"的根因。
    //   ★ 2026-10-18 需求方追加口径：**向零取整**（正值向下、负值向上），
    //     即"让绝对值的绝对值向下取整"。
    //     不能直接 `(gyro_z_comp * 10486) >>> 20` —— `>>>` 是算术右移 = floor，
    //     负数会多减 1：-100.002 °/s → 104,860,000 的补码右移 = **-101**，而应为 **-100**。
    //     做法：取绝对值 → 右移（此时是正数，floor 就是向零）→ 再补符号。
    //     溢出检查：|gyro_z_comp| ≤ 200000，×10486 = 2.10e9 < 2^31 ✓
    //     （-200000 取反成 +200000 也在 int32 范围内，不存在 -2^31 的边界问题）
    // 仅显示支路增加流水，补偿数据供控制使用的时序保持不变。
    reg signed [31:0] gyro_show;
    reg        [31:0] gyro_abs;
    reg               gyro_negative;
    reg signed [31:0] gyro_deg_s;
    wire [31:0] gyro_abs_deg = (gyro_abs * 32'd10486) >> 20;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            gyro_show     <= 0;
            gyro_abs      <= 0;
            gyro_negative <= 0;
            gyro_deg_s    <= 0;
        end else begin
            gyro_show     <= gyro_z_comp;
            gyro_abs      <= gyro_show < 0 ? -gyro_show : gyro_show;
            gyro_negative <= gyro_show < 0;
            gyro_deg_s    <= gyro_negative ? -gyro_abs_deg : gyro_abs_deg;
        end
    end

    // ==================================================================
    // 航位推算 odom（2026-10-18 新增，依据 设计文档 §5.3）
    //   输入：四轮 10 ms 计数增量（已含 ENC_INV_DIR_VEC 极性）+ 陀螺航向
    //   输出：x/y（mm）、连续航向（0.01°）、线速度（mm/s）、编码器角速度（0.01°/s）
    //   ★ 航向直接用 `yaw_raw`（陀螺），编码器只提供位移 —— §5.1 的"编码器互补"
    //     是精度优化，不是通路前提，留到 k 标定之后再做。
    //   ★ 清零与航向角同源（S2 / 串口 X 命令），保证"位置基准 = 航向基准"。
    // ==================================================================
    wire signed [31:0] odom_x_mm, odom_y_mm, odom_yaw_cont, odom_v_mms, odom_omega_enc;

    odom #(
        .CLK_HZ      (CLK_HZ),
        .WINDOW_MS   (10),
        .CNT_PER_REV (61440),           // 512 PPR × 4 倍频 × 30 减速比
        .WHEEL_R_MM  (32),              // ★ 轮半径 32 mm ⇒ 直径 64 mm（需求方 2026-10-18 实测）
        .TRACK_MM10  (1615),            // ★ 轮距 161.5 mm，单位 0.1mm
        // ★ 里程标定：理论 3431（D=64mm）；实测推 3.00m 推算 2.92m（偏小 2.67%），
        //   按比例校正为 3525。改这一个数即可整体缩放航位与线速度。
        .DS_K_Q10    (3525)
    ) u_odom (
        .clk           (i_clk_50m),
        .rst_n         (sys_rst_n),
        .i_delta0      (dlt0),
        .i_delta1      (dlt1),
        .i_delta2      (dlt2),
        .i_delta3      (dlt3),
        .i_delta_valid (delta_valid),
        .i_yaw         (yaw_raw_s),      // ★ 已按 YAW_SIGN 归一为"右转增大"
        .i_yaw_valid   (yaw_valid),
        .i_clear       (count_clear_r),  // 与编码器计数/航向角同源清零
        .o_x_mm        (odom_x_mm),
        .o_y_mm        (odom_y_mm),
        .o_yaw_cont    (odom_yaw_cont),
        .o_v_mms       (odom_v_mms),
        .o_omega_enc   (odom_omega_enc)
    );

    // 雷达与规划共用定位原点；暂不授权自动动作，后续接入页面及控制握手。
    reg nav_calibrated;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) nav_calibrated <= 0;
        else if (imu_calib_busy || imu_err_flag) nav_calibrated <= 0;
        else if (imu_calib_done) nav_calibrated <= 1;
    end
    wire nav_reset_n = sys_rst_n && !count_clear_r;
    wire navigation_pose_valid, navigation_pose_tick;
    wire signed [31:0] navigation_x_mm, navigation_y_mm, navigation_yaw_cdeg;
    nav_pose u_nav_pose (
        .clk(i_clk_50m), .rst_n(sys_rst_n), .clear(count_clear_r),
        .ready(nav_calibrated && imu_init_done && !imu_err_flag),
        .position_tick(delta_valid), .x_mm(odom_x_mm), .y_mm(odom_y_mm),
        .yaw_cdeg(yaw_raw_s), .pose_valid(navigation_pose_valid), .pose_tick(navigation_pose_tick),
        .pose_x_mm(navigation_x_mm), .pose_y_mm(navigation_y_mm),
        .pose_yaw_cdeg(navigation_yaw_cdeg)
    );
    wire map_disp_valid, map_disp_occupied, map_disp_start, map_disp_done;
    wire [15:0] navigation_map_version;
    wire inspection_usb_valid,inspection_usb_release,inspection_usb_read;
    wire [8:0] inspection_usb_words;
    wire [7:0] inspection_usb_address;
    wire [15:0] inspection_usb_data;
    wire inspection_frame_begin,inspection_frame_commit,inspection_frame_abort,inspection_frame_ready,inspection_pixel_valid,inspection_pixel_ready;
    wire [15:0] inspection_pixel;
    wire [31:0] inspection_frame_sequence;
    wire inspection_map_valid,inspection_map_release,inspection_map_read;
    wire [11:0] inspection_map_address;
    wire [15:0] inspection_map_data;
    wire [31:0] inspection_map_session,inspection_map_task,inspection_map_epoch,inspection_map_context,inspection_map_sequence,inspection_map_crc;
    wire [1:0] inspection_map_view;
    reg [31:0] inspection_epoch;
    reg [15:0] inspection_user_token;
    wire inspection_permission=ai_master && ai_mode==3 && ai_inspection_open && page==PAGE_AI;
    wire navigation_sensors_valid;
    always @(posedge i_clk_50m or negedge sys_rst_n)begin
        if(!sys_rst_n)begin inspection_epoch<=1;inspection_user_token<=0;end
        else begin
            if(count_clear_r || yaw_clear_r || imu_calib_done)inspection_epoch<=inspection_epoch+1'b1;
            if(inspection_permission && !manual_drive && rc_confirm)inspection_user_token<=inspection_user_token+1'b1;
        end
    end
    inspection_application #(.CLK_HZ(CLK_HZ),.UI_ENABLE(1),.HIGHSPEED_ENABLE(1),.CLOCK_ENABLE(1)) u_inspection(
        .clk(i_clk_50m),.rst_n(sys_rst_n),.i_rx_data(motion_rx_data),.i_rx_valid(motion_rx_valid),
        .i_usb_valid(inspection_usb_valid),.i_usb_words(inspection_usb_words),.o_usb_release(inspection_usb_release),
        .o_usb_read(inspection_usb_read),.o_usb_address(inspection_usb_address),.i_usb_data(inspection_usb_data),
        .i_permission(inspection_permission),.i_manual(manual_drive),
        .i_safety(rc_signal_ok && voice_imu_healthy && navigation_pose_valid && navigation_sensors_valid &&
                  !estop_latch && !tof_estop && !(|stall_vec)),
        .i_link(ai_host_alive),.i_local_pause((rc_confirm || rc_back || ai_record) && (inspection_motion_enable || inspection_pending_motion)),
        .i_user_token(inspection_user_token),.i_epoch(inspection_epoch),
        .i_ui_open(ai_inspection_open),.i_ui_master(ai_master),.i_ui_record(ai_record),.i_ui_rc_valid(rc_signal_ok),
        .i_ui_confirm(rc_confirm),.i_ui_back(rc_back),.i_ui_view(ai_inspection_view),
        .i_ui_steer(rc_steer),.i_ui_throttle(rc_throttle),.i_vehicle_width_mm(tune_p[431:416]),
        .i_forward_check_cells(tune_p[391:384]),.o_ui_close(ai_inspection_close),
        .o_session(inspection_session),.o_task(inspection_task_id),.o_context(inspection_context),
        .i_pose_tick(navigation_pose_tick),.i_pose_valid(navigation_pose_valid),
        .i_x_mm(navigation_x_mm),.i_y_mm(navigation_y_mm),.i_yaw_cdeg(navigation_yaw_cdeg),
        .i_ctrl_done(obs_ctrl_done),.i_command_idle(!planner_cmd_valid && !planner_emergency_stop && !obs_ctrl_done),
        .i_ctrl_fault(obs_ctrl_fault),.i_nav_state(planner_nav_state),
        .o_owned(inspection_owned),.o_goal_valid(inspection_goal_valid),.o_motion_enable(inspection_motion_enable),
        .o_force_stop(inspection_force_stop),.o_goal_x(inspection_goal_x),.o_goal_y(inspection_goal_y),
        .o_pending_motion(inspection_pending_motion),
        .o_layer_begin(inspection_begin),.o_layer_commit(inspection_commit),.o_layer_word_valid(inspection_word_valid),
        .o_layer_epoch(inspection_layer_epoch),.o_layer_crc(inspection_layer_crc),.o_layer_word(inspection_layer_word),
        .o_layer_version(inspection_begin_version),.o_layer_word_index(inspection_word_index),
        .i_layer_ready(inspection_word_ready),.i_layer_valid(inspection_layer_valid),.i_layer_committed(inspection_committed),
        .i_layer_error(inspection_error),.i_layer_busy(inspection_layer_busy),.i_layer_version(inspection_layer_version),
        .i_layer_error_code(inspection_error_code),.o_stopped(inspection_stopped),.o_status(inspection_status),
        .o_tx_data(inspection_tx_data),.o_tx_valid(inspection_tx_valid),.o_tx_last(inspection_tx_last),.i_tx_ready(inspection_tx_ready));
    // ACG720 L13 is FX2 PA0, not USB_RESET. Vendor firmware polls PA0 and
    // clears EP6 while it is high; match the vendor Slave FIFO example.
    assign o_fx2_pa0=1'b0;
    wire route_disp_valid, route_disp_start, route_disp_done;
    inspection_highspeed #(.CLK_HZ(CLK_HZ),.CLOCK_PROBE_ENABLE(1)) u_inspection_usb(
        .clk(i_clk_50m),.rst_n(sys_rst_n),.fx2_clk(i_fx2_ifclk),.fx2_fd(io_fx2_fd),
        .fx2_flagb(i_fx2_flagb),.fx2_flagc(i_fx2_flagc),.fx2_address(o_fx2_faddr),
        .fx2_slrd_n(o_fx2_slrd_n),.fx2_slwr_n(o_fx2_slwr_n),.fx2_sloe_n(o_fx2_sloe_n),
        .fx2_pktend_n(o_fx2_pktend_n),.fx2_slcs_n(o_fx2_slcs_n),
        .i_display_enabled(inspection_permission),.i_session(inspection_session),.i_task(inspection_task_id),
        .i_epoch(inspection_epoch),.i_context(inspection_context),.i_view(ai_inspection_view),
        .o_frame_begin(inspection_frame_begin),.o_frame_commit(inspection_frame_commit),.o_frame_abort(inspection_frame_abort),.o_frame_sequence(inspection_frame_sequence),
        .i_frame_ready(inspection_frame_ready),.o_frame_data(inspection_pixel),.o_frame_valid(inspection_pixel_valid),.i_frame_word_ready(inspection_pixel_ready),
        .i_map_valid(inspection_map_valid),.o_map_release(inspection_map_release),
        .i_map_session(inspection_map_session),.i_map_task(inspection_map_task),.i_map_epoch(inspection_map_epoch),.i_map_context(inspection_map_context),
        .i_map_sequence(inspection_map_sequence),.i_map_crc(inspection_map_crc),.i_map_view(inspection_map_view),
        .o_map_read(inspection_map_read),.o_map_address(inspection_map_address),.i_map_data(inspection_map_data),
        .o_control_valid(inspection_usb_valid),.o_control_words(inspection_usb_words),.i_control_ready(inspection_usb_release),.i_control_read(inspection_usb_read),
        .i_control_address(inspection_usb_address),.o_control_data(inspection_usb_data),.o_diagnostics());
    inspection_map_export u_inspection_map(
        .clk(i_clk_50m),.rst_n(sys_rst_n),.i_enabled(inspection_permission && ai_host_alive),
        .i_valid(map_disp_valid),.i_occupied(map_disp_occupied),.i_start(map_disp_start),.i_done(map_disp_done),.i_version(navigation_map_version),
        .i_session(inspection_session),.i_task(inspection_task_id),.i_epoch(inspection_epoch),.i_context(inspection_context),.i_view(ai_inspection_view),
        .o_valid(inspection_map_valid),.i_release(inspection_map_release),.o_session(inspection_map_session),.o_task(inspection_map_task),
        .o_epoch(inspection_map_epoch),.o_context(inspection_map_context),.o_sequence(inspection_map_sequence),.o_crc(inspection_map_crc),.o_view(inspection_map_view),
        .i_read(inspection_map_read),.i_address(inspection_map_address),.o_data(inspection_map_data),.o_error());
    wire [7:0] route_disp_x, route_disp_y;
    reg navigation_motion_enable;
    always @(posedge i_clk_50m or negedge sys_rst_n) begin
        if (!sys_rst_n) navigation_motion_enable <= 0;
        else navigation_motion_enable <= (inspection_owned?inspection_motion_enable:obs_run) && !manual_drive && rc_signal_ok &&
                                         !estop_latch && !tof_estop && !obs_ctrl_fault;
    end
    radar_navigation #(.AI_LAYER_ENABLE(1)) u_radar_navigation (
        .forward_check_cells(tune_p[391:384]), .vehicle_width_mm(tune_p[431:416]),
        .clk(i_clk_50m), .reset_n(nav_reset_n),
        .map_enable(navigation_pose_valid), .plan_enable(inspection_owned?inspection_goal_valid:obs_goal_valid),
        .motion_enable(navigation_motion_enable),
        .radar_rx(i_radar_rx), .pose_valid(navigation_pose_valid),
        .pose_x_mm(navigation_x_mm), .pose_y_mm(navigation_y_mm),
        .pose_yaw_cdeg(navigation_yaw_cdeg),
        .goal_valid(inspection_owned?inspection_goal_valid:obs_goal_valid),
        .goal_x_cell(inspection_owned?inspection_goal_x:obs_goal_x),.goal_y_cell(inspection_owned?inspection_goal_y:obs_goal_y),
        .ai_enable(inspection_owned),.ai_begin(inspection_begin),.ai_abort(1'b0),.ai_commit(inspection_commit),
        .ai_word_valid(inspection_word_valid),.ai_stopped(inspection_stopped),.ai_epoch(inspection_epoch),
        .ai_begin_epoch(inspection_layer_epoch),.ai_expected_crc(inspection_layer_crc),.ai_word(inspection_layer_word),
        .ai_begin_version(inspection_begin_version),.ai_word_index(inspection_word_index),
        .ai_word_ready(inspection_word_ready),.ai_layer_valid(inspection_layer_valid),.ai_layer_version(inspection_layer_version),
        .ai_committed(inspection_committed),.ai_error(inspection_error),.ai_error_code(inspection_error_code),.ai_layer_busy(inspection_layer_busy),
        .ctrl_done(obs_ctrl_done), .ctrl_linear_active(obs_linear_active),
        .cmd_valid(planner_cmd_valid), .emergency_stop(planner_emergency_stop),
        .target_x_mm(planner_target_x), .target_y_mm(planner_target_y),
        .target_line_dir(planner_target_line_dir),
        .nav_state(planner_nav_state),.sensors_valid(navigation_sensors_valid),
        .map_version(navigation_map_version),
        .display_enable((page == PAGE_RADAR) || (page == PAGE_NAV_OBS) || inspection_permission),
        .map_disp_valid(map_disp_valid), .map_disp_occupied(map_disp_occupied),
        .map_disp_frame_start(map_disp_start), .map_disp_frame_done(map_disp_done),
        .route_disp_valid(route_disp_valid), .route_disp_x_cell(route_disp_x),
        .route_disp_y_cell(route_disp_y), .route_disp_frame_start(route_disp_start),
        .route_disp_frame_done(route_disp_done),
        .cloud_valid(), .cloud_angle_cdeg(), .cloud_distance_mm()
    );

    // ==================================================================
    // 6b. 非避障导航页控制器（2026-10-19 新增）
    // ==================================================================
    //   它只维护这一页的**可交互状态**：准心位置、已记录点表、辅助菜单高亮、运行态。
    //   ★ 按「间断寻点 / 非间断寻点」+ 确认后**只把 free_nav_run 拉高，车一步不动** ——
    //     需求方本轮明确："任务导航只需留下接口或状态机等，并不真正开始导航。"
    //     真接入时把 {free_nav_run, free_nav_mode, nav_pt_*} 接到导航控制层
    //     （方案见 doc\导航控制方案.md）。
    //   ★ 摇杆用 `rc_steer / rc_throttle` 的**原始归一化值**，不用 `steer_in` ——
    //     后者在光标档被强制 0（那是给底盘的），准心要的是摇杆本身。
    nav_free_ctrl #(
        .CLK_HZ       (CLK_HZ),
        .CURSOR_SPEED (500),        // 满杆 500 mm/s（需求方 2026-10-19 选定）
        .DEADZONE     (100),        // 归一化 ±1000 的 10%，盖住中位抖动 ±50
        .MAX_POINTS   (16)
    ) u_nav_free (
        .clk        (i_clk_50m),
        .rst_n      (sys_rst_n),
        .i_active   (nav_page_now),         // ★ 两页共用同一套状态机
        .i_is_obs   (1'b0), // 避障页：辅助菜单 4 项、第 3 项=开始导航
        .i_clear    (count_clear_r),        // 与编码器计数/航向角同源清零
        .i_x_mm     (odom_x_mm),
        .i_y_mm     (odom_y_mm),
        .i_manual   (manual_drive),         // 手动接管档：摇杆归底盘，准心不动
        .i_pause    (key1_press || estop_latch || key4_press || tof_estop || !rc_signal_ok),
        .i_steer    (rc_steer),
        .i_throttle (rc_throttle),
        .i_confirm  (rc_confirm),           // CH5 下拨
        .i_back     (rc_back),              // CH5 上拨
        .i_aux_up   (rc_aux_up),            // CH8 上拨
        .i_aux_down (rc_aux_down),          // CH8 下拨
        // ---- 与导航/控制层对接 ----
        .i_ctrl_done  (nav_ctrl_done),   // 来自 u_nav_ctrl.o_done（仅最后一点）
        .i_ctrl_fault (nav_ctrl_fault),  // 来自 u_nav_ctrl.o_fault
        .o_cur_x_mm (free_nav_cur_x_mm),
        .o_cur_y_mm (free_nav_cur_y_mm),
        .o_aux_sel  (free_nav_aux_sel),
        .o_pt_count (free_nav_pt_count),
        .o_nav_state     (free_nav_state),
        .o_nav_ui_active (free_nav_ui_active),
        .o_nav_run       (free_nav_run),
        .o_nav_paused    (free_nav_paused),
        .o_nav_mode      (free_nav_mode),
        .o_pt_px    (free_nav_pt_px),
        .o_pt_py    (free_nav_pt_py),
        .o_pt_v     (free_nav_pt_v),
        .o_pt_x_mm  (free_nav_pt_x_mm),
        .o_pt_y_mm  (free_nav_pt_y_mm),
        .o_car_px   (free_nav_car_px),
        .o_car_py   (free_nav_car_py),
        .o_cur_px   (free_nav_cur_px),
        .o_cur_py   (free_nav_cur_py)
    );

    nav_point_ctrl u_nav_point_ctrl (
        .clk(i_clk_50m), .rst_n(sys_rst_n),
        .i_cmd_valid(planner_cmd_valid), .i_emergency_stop(planner_emergency_stop | (inspection_force_stop && !manual_drive)),
        .i_target_x_mm(planner_target_x), .i_target_y_mm(planner_target_y),
        .i_tick(navigation_pose_tick), .i_x_mm(navigation_x_mm), .i_y_mm(navigation_y_mm),
        .i_yaw_centi(navigation_yaw_cdeg), .i_v_mms(odom_v_mms), .i_stall(stall_vec),
        .i_v_cruise(tune_p[31:0]), .i_d_slow(tune_p[63:32]),
        .i_align_in(tune_p[95:64]), .i_align_out(tune_p[127:96]),
        .i_max_diff_cruise(tune_p[159:128]), .i_kp_yaw(tune_p[191:160]),
        .i_ki_yaw(tune_p[223:192]), .i_kd_yaw(tune_p[255:224]),
        .i_i_limit(tune_p[287:256]), .i_k_slow_q10(tune_k_slow),
        .o_auto_valid(obs_auto_valid), .o_center(obs_auto_center), .o_diff(obs_auto_diff),
        .o_done(obs_ctrl_done), .o_linear_active(obs_linear_active), .o_fault(obs_ctrl_fault)
    );

    nav_obstacle_ui #(.CLK_HZ(CLK_HZ)) u_nav_obstacle_ui (
        .clk(i_clk_50m), .rst_n(sys_rst_n), .i_active(page == PAGE_NAV_OBS),
        .i_clear(count_clear_r), .i_x_mm(odom_x_mm), .i_y_mm(odom_y_mm),
        .i_manual(manual_drive), .i_steer(rc_steer), .i_throttle(rc_throttle),
        .i_confirm(rc_confirm), .i_back(rc_back), .i_aux_up(rc_aux_up), .i_aux_down(rc_aux_down),
        .i_nav_state(planner_nav_state),
        .i_command_idle(!planner_cmd_valid && !planner_emergency_stop && !obs_ctrl_done),
        .i_pause(key1_press || estop_latch || key4_press || tof_estop || !rc_signal_ok), .i_ctrl_fault(obs_ctrl_fault),
        .o_cur_x_mm(obs_cur_x_mm), .o_cur_y_mm(obs_cur_y_mm), .o_aux_sel(obs_aux_sel),
        .o_pt_count(obs_pt_count), .o_nav_state(obs_ui_state),
        .o_nav_ui_active(obs_ui_active), .o_nav_run(obs_run), .o_nav_paused(obs_paused),
        .o_goal_valid(obs_goal_valid), .o_goal_x_cell(obs_goal_x), .o_goal_y_cell(obs_goal_y),
        .o_pt_px(obs_pt_px), .o_pt_py(obs_pt_py), .o_pt_v(obs_pt_v),
        .o_car_px(obs_car_px), .o_car_py(obs_car_py),
        .o_cur_px(obs_cur_px), .o_cur_py(obs_cur_py)
    );

    // ★ 菜单锁：导航页 **且** 处于交互态。返回键退出交互态后菜单即恢复可操作。
    assign nav_lock = nav_page_any & nav_ui_active;

    // ==================================================================
    // 12b. 导航控制层 `u_nav_ctrl`（rtl\nav\nav_ctrl.v）
    // ==================================================================
    // 【坐标系】只有两套，**没有世界系**：
    //   ① odom 相对系：原点 = 上次 S2 清零那一刻的车心，+x = 那一刻的车头，
    //      **永不重基准、不做分段重基准**。就是 `odom.o_x_mm/o_y_mm` 天然那套。
    //      点表存在这里（点位置不随车转；存车体系的话车一转旧点就全废）。
    //   ② 车体系：每 10 ms 由 ① 旋转得到（ahead 前正 / left 左正），不存储、只在拍内用。
    //   全程没有第二套绝对坐标，也不引入"地图原点""重基准"概念。
    //
    // 【使能】`nav_ctrl_active = 当前在导航页 & 处于交互态`。
    //   返回键退出交互态后立即拉 0 → u_nav_ctrl 输出 valid=0 → 底盘不再吃自动指令，
    //   与"菜单不冻结"的语义一致（人已经退出导航任务了）。
    //
    // 【暂停时一定停住】`i_paused=1` 时 u_nav_ctrl 内部走专门分支：
    //   `o_auto_valid=1, o_center=0, o_diff=0` —— 故意**不**把它设成 invalid，
    //   因为 invalid 会让 chassis_ctrl 输出 0 占空比（= 断驱动、靠惯性滑行），
    //   而 valid + 目标 0 会让四路 speed_pid 主动把轮速拉向 0（= 主动制动）。
    //   同时 `i_enable = run_en | nav_auto_valid` 保证导航期间底盘使能不依赖遥控解锁。
    assign nav_ctrl_active = nav_page_now & free_nav_ui_active;

    // ★ 2026-10-20：8 个可调值（巡航速度 / 减速距离 / 两个对准门限 / 行进差速限幅 /
    //   航向 PID 三项）已从 module parameter 改为**运行时端口**，由 u_tune_ctrl 在线
    //   给值（参数整定页第 0..7 项）。K_SLOW 是派生量，也由 tune_ctrl 的除法器算好送进来。
    //   仍留在 parameter 里的都是结构性/安全性常量（到点半径、L2 切点几何、超时、
    //   MAX_CENTER / MAX_DIFF 上限、停稳判据等），不适合在界面上随手改。
    nav_ctrl #(
        .ARRIVE_R_MM     (25),     // L1 到点半径
        .NP_CUT_DIST_MM  (90),     // L2 提前切点 = v²/2a
        .NP_MIN_SPACE_MM (260),    // L2 最小点距（D_SLOW=100 后仍取 260 更保守）
        .MAX_CENTER      (3072),   // = 1033 mm/s
        .MAX_DIFF        (612)     // 原地转向 ≈146 °/s
    ) u_nav_ctrl (
        .clk   (i_clk_50m),
        .rst_n (sys_rst_n),
        .i_v_cruise       (tune_p[31:0]),      // 300 mm/s
        .i_d_slow         (tune_p[63:32]),     // 100 mm
        .i_align_in       (tune_p[95:64]),     // 1000 = 10°
        .i_align_out      (tune_p[127:96]),    // 3500 = 35°
        .i_max_diff_cruise(tune_p[159:128]),   // 180 ≈ 43 °/s
        .i_kp_yaw         (tune_p[191:160]),   // 21
        .i_ki_yaw         (tune_p[223:192]),   // 4
        .i_kd_yaw         (tune_p[255:224]),   // 6
        .i_i_limit        (tune_p[287:256]),   // 默认 90（积分限幅，原 I_LIMIT=400）
        .i_k_slow_q10     (tune_k_slow),       // 3072 = 300×1024/100
        .i_active  (nav_ctrl_active),
        .i_run     (free_nav_run),
        .i_paused  (free_nav_paused),
        .i_mode    (free_nav_mode),                 // 0 间断 / 1 非间断
        .i_is_obs  (page == PAGE_NAV_OBS),     // 避障页：目标取准心
        .i_cur_x_mm(free_nav_cur_x_mm),
        .i_cur_y_mm(free_nav_cur_y_mm),
        .i_pt_x_mm (free_nav_pt_x_mm),
        .i_pt_y_mm (free_nav_pt_y_mm),
        .i_pt_count(free_nav_pt_count),
        .i_tick    (delta_valid),              // 10 ms，与 odom 测速同拍
        .i_x_mm      (odom_x_mm),
        .i_y_mm      (odom_y_mm),
        .i_yaw_centi (odom_yaw_cont),          // 0.01°，连续值，模块内部自己折返
        .i_v_mms     (odom_v_mms),
        .i_stall     (stall_vec),
        .o_auto_valid(nav_auto_valid),
        .o_center    (nav_auto_center),
        .o_diff      (nav_auto_diff),
        .o_done      (nav_ctrl_done),
        .o_fault     (nav_ctrl_fault),
        .o_state     (nav_fsm_state),
        .o_dbg_ahead (nav_dbg_ahead),
        .o_dbg_left  (nav_dbg_left),
        .o_dbg_psi   (nav_dbg_psi),
        .o_dbg_idx   (nav_dbg_idx)
    );

    // ==================================================================
    // 12c. 参数整定页控制器 `u_tune_ctrl`（rtl\ui\tune_ctrl.v）
    //
    //   一期（用户 2026-10-20 指示）：参数运行时化 + 屏上整定页，参数**存 RAM**，
    //   掉电丢失。二期（Flash 持久化：A/B 双份 + CRC16 + 高位分区）暂缓。
    //
    //   交互（与导航页同构，不占用底盘摇杆的控制语义）：
    //     CH8 上/下    选行（0..10 循环）
    //     CH4 左/右    调值（越过 ±200 死区变一次，必须回中才能再触发；
    //                       按住超 400 ms 后每 120 ms 连发一步）
    //     CH5 下(确认) 当前行恢复默认值
    //     CH5 上(返回) 退出**编辑态**（不换页：page 仍停在参数页，菜单高亮也留在本项；
    //                  退出后菜单键恢复可用，再按确认即重新进编辑态）
    //   手动档（manual_drive=1）时 can_adj=0：只显示不改，摇杆归底盘。
    // ==================================================================
    tune_ctrl u_tune_ctrl (
        .clk        (i_clk_50m),
        .rst_n      (sys_rst_n),
        .i_active   (tune_active),        // 只在参数页的编辑态工作
        .i_manual   (manual_drive),       // 手动档：不允许调
        .i_signal   (rc_signal_ok),
        .i_steer    (rc_steer),
        .i_confirm  (rc_confirm),         // CH5 下拨
        .i_back     (rc_back),            // CH5 上拨
        .i_throttle (rc_throttle),        // CH2 上下（选行）
        .o_param    (tune_p),
        .o_disp     (tune_disp),
        .o_sel      (tune_sel),
        .o_k_slow_q10(tune_k_slow),
        .o_exit     (tune_exit),
        .o_div_busy ()
    );

    // 参数页选中行 → pixel 域（4 位，两级同步；人操作级别慢变量）
    reg [3:0] tune_ps1, tune_ps2;
    // 参数页编辑态 → pixel 域（1 位，两级同步）。用于两件事：
    //   ① 编辑态时左侧菜单栏变暗；② 未进编辑态时不画行高亮条。
    reg tune_act_ps1, tune_act_ps2;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            tune_ps1 <= 4'd0; tune_ps2 <= 4'd0;
            tune_act_ps1 <= 1'b0; tune_act_ps2 <= 1'b0;
        end else begin
            tune_ps1 <= tune_sel; tune_ps2 <= tune_ps1;
            tune_act_ps1 <= tune_active; tune_act_ps2 <= tune_act_ps1;
        end
    end

    // 调试观测口（nav_dbg_* / nav_fsm_state）本轮不消费：综合会把它们优化掉，
    //   留着是为了将来接 SignalTap 一类在线观测，不必改端口。

    // 各来源独立换算、寄存，页面选择不再进入除法路径。
    wire signed [31:0] odom_x_cm_r, odom_y_cm_r;
    wire signed [31:0] free_cur_x_cm, free_cur_y_cm, obs_cur_x_cm, obs_cur_y_cm;
    wire signed [31:0] nav_cur_x_cm_r = page == PAGE_NAV_OBS ? obs_cur_x_cm : free_cur_x_cm;
    wire signed [31:0] nav_cur_y_cm_r = page == PAGE_NAV_OBS ? obs_cur_y_cm : free_cur_y_cm;
    display_mm_to_cm u_show_odom_x (
        .clk(i_clk_50m), .reset_n(sys_rst_n), .mm(odom_x_mm), .cm(odom_x_cm_r)
    );
    display_mm_to_cm u_show_odom_y (
        .clk(i_clk_50m), .reset_n(sys_rst_n), .mm(odom_y_mm), .cm(odom_y_cm_r)
    );
    display_mm_to_cm u_show_free_x (
        .clk(i_clk_50m), .reset_n(sys_rst_n), .mm(free_nav_cur_x_mm), .cm(free_cur_x_cm)
    );
    display_mm_to_cm u_show_free_y (
        .clk(i_clk_50m), .reset_n(sys_rst_n), .mm(free_nav_cur_y_mm), .cm(free_cur_y_cm)
    );
    display_mm_to_cm u_show_obs_x (
        .clk(i_clk_50m), .reset_n(sys_rst_n), .mm(obs_cur_x_mm), .cm(obs_cur_x_cm)
    );
    display_mm_to_cm u_show_obs_y (
        .clk(i_clk_50m), .reset_n(sys_rst_n), .mm(obs_cur_y_mm), .cm(obs_cur_y_cm)
    );

    // ---- num_text 的 11 路输入：数据页用真实量，参数整定页用 tune_ctrl 的显示值 ----
    wire tune_page = (page == PAGE_TUNE);
    wire signed [31:0] nt_v0  = tune_page ? tune_disp[31:0]    : yaw_deg;
    wire signed [31:0] nt_v1  = tune_page ? tune_disp[63:32]   : rpm0_disp;
    wire signed [31:0] nt_v2  = tune_page ? tune_disp[95:64]   : rpm1_disp;
    wire signed [31:0] nt_v3  = tune_page ? tune_disp[127:96]  : rpm2_disp;
    wire signed [31:0] nt_v4  = tune_page ? tune_disp[159:128] : rpm3_disp;
    wire signed [31:0] nt_v5  = tune_page ? tune_disp[191:160] : gyro_deg_s;
    wire signed [31:0] nt_v6  = tune_page ? tune_disp[223:192] : center_cms_show;
    wire signed [31:0] nt_v7  = tune_page ? tune_disp[255:224] : odom_x_cm_r;
    wire signed [31:0] nt_v8  = tune_page ? tune_disp[287:256] : odom_y_cm_r; // 参数 8 积分限幅
    wire signed [31:0] nt_v9  = tune_page ? tune_disp[319:288] : nav_cur_x_cm_r; // 参数 9 速度环 Kp
    wire signed [31:0] nt_v10 = tune_page ? tune_disp[351:320] : nav_cur_y_cm_r; // 参数 10 速度环 Ki
    wire signed [31:0] nt_v11 = tune_page ? tune_disp[383:352] : 32'sd0;
    wire signed [31:0] nt_v12 = tune_page ? tune_disp[415:384] : 32'sd0;
    wire signed [31:0] nt_v13 = tune_page ? tune_disp[447:416] : 32'sd0;
    // ★ 2026-10-20（综合报错后补）：哪些字段"带小数"（= num_text 里"整数部分至少几位"）。
    //   数据页：字段 6 中心速度 / 7 航位 x / 8 航位 y / 9 准心 x / 10 准心 y，均 2 位小数
    //           → bit 6..10 置 1 → 12'b0111_1100_0000（= 0x7C0）
    //   参数页：14 项全是整数 → 全 0。否则航向 Kd=6 会显示成 "  006"，
    //           且 ui_render 的 field_dec 还会在百位后画点，变成 "  0.06"。
    wire [13:0] nt_frac_mask = tune_page ? 14'b00_0000_0000_0000 : 14'b00_0111_1100_0000;

    num_text #(
        .CLK_HZ     (CLK_HZ),
        .REFRESH_MS (100),
        .NUM_FIELDS (14)
    ) u_num_text (
        .clk    (i_clk_50m),
        .rst_n  (sys_rst_n),
        // ★ 2026-10-20：参数整定页复用这 11 个字段（字段号 = 参数行号）。
        //   两页互斥（page 不同），按页选择来源；14 个字段使用 128 字节字符 RAM。
        //   切换页时字段可能短暂混写（num_text 100 ms 刷新一轮），视觉可接受。
        .i_v0   (nt_v0),             // 字段 0：航向角 / 参数 0 车速
        .i_v1   (nt_v1),             // 字段 1：A 右后轮 / 参数 1 减速距离
        .i_v2   (nt_v2),             // 字段 2：B 左后轮 / 参数 2 对准小角
        .i_v3   (nt_v3),             // 字段 3：C 右前轮 / 参数 3 对准大角
        .i_v4   (nt_v4),             // 字段 4：D 左前轮 / 参数 4 转向角速度
        .i_v5   (nt_v5),             // 字段 5：角速度 / 参数 5 航向 Kp
        .i_v6   (nt_v6),             // 字段 6：中心速度 / 参数 6 航向 Ki
        .i_v7   (nt_v7),             // 字段 7：航位 x / 参数 7 航向 Kd
        .i_v8   (nt_v8),             // 字段 8：航位 y / 参数 8 积分限幅
        .i_v9   (nt_v9),             // 字段 9：准心 x / 参数 9 速度环 Kp
        .i_v10  (nt_v10),            // 字段 10：准心 y / 参数 10 速度环 Ki
        .i_v11  (nt_v11),            // 字段 11：参数 11 速度环 Kd（数据页恒 0）
        .i_v12  (nt_v12),            // 前探格数
        .i_v13  (nt_v13),            // 车宽 mm
        .i_frac_mask(nt_frac_mask),   // 哪些字段带小数（按页面切换，见声明处）
        .o_we   (dyn_we),
        .o_addr (dyn_waddr),
        .o_data (dyn_wdata)
    );

    dyn_text_ram #(
        .DEPTH     (128),
        .ADDR_BITS (7)
    ) u_dyn_text (
        .i_clk_w (i_clk_50m),        // 写：sys 域
        .i_we    (dyn_we),
        .i_waddr (dyn_waddr),
        .i_wdata (dyn_wdata),
        .i_clk_r (pixel_clk),        // 读：pixel 域
        .i_raddr (dyn_raddr),
        .o_rdata (dyn_rdata)
    );

    wire map_pixel, route_pixel;
    reg [2:0] obstacle_state_ps1, obstacle_state_ps2;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            obstacle_state_ps1 <= 0;
            obstacle_state_ps2 <= 0;
        end else begin
            obstacle_state_ps1 <= obs_ui_state;
            obstacle_state_ps2 <= obstacle_state_ps1;
        end
    end
    nav_map_view u_nav_map_view (
        .clk(i_clk_50m), .rst_n(sys_rst_n),
        .map_valid(map_disp_valid), .map_occupied(map_disp_occupied),
        .map_start(map_disp_start), .map_done(map_disp_done),
        .route_valid(route_disp_valid), .route_x(route_disp_x), .route_y(route_disp_y),
        .route_start(route_disp_start), .route_done(route_disp_done),
        .cmd_valid(planner_cmd_valid), .target_line_dir(planner_target_line_dir),
        .pose_x_mm(navigation_x_mm), .pose_y_mm(navigation_y_mm),
        .target_x_mm(planner_target_x), .target_y_mm(planner_target_y),
        .pixel_clk(pixel_clk), .pixel_rst_n(pixel_rst_n), .frame_start(frame_start),
        .pixel_x(pixel_x), .pixel_y(pixel_y), .map_pixel(map_pixel), .route_pixel(route_pixel)
    );

    wire [23:0] ui_rgb_base;
    wire ai_glyph,ai_connected,ai_local_region,ai_highlight;
    ai_display_link #(.CLK_HZ(CLK_HZ)) u_ai_display(
        .clk(i_clk_50m),.rst_n(sys_rst_n),.rx_data(motion_rx_data),.rx_valid(motion_rx_valid),
        .pixel_clk(pixel_clk),.pixel_rst_n(pixel_rst_n),.pixel_x(pixel_x),.pixel_y(pixel_y),
        .selected(ai_inspection_open?ai_inspection_view:ai_selected),.i_inspection(ai_inspection_open),
        .o_glyph(ai_glyph),.o_connected(ai_connected),.o_local_region(ai_local_region),.o_highlight(ai_highlight),
        .o_display_ack(ai_display_ack),.o_display_bank(ai_display_bank),.o_host_alive(ai_host_alive));
    assign ui_rgb_raw=(page_ps2==PAGE_AI && pixel_x>=208 &&
        (ai_local_region || (ai_connected && pixel_y>=120 && pixel_y<540)))
        ? (ai_glyph ? (ai_highlight?24'hffe066:24'he8f0f8) : (ai_highlight?24'h1d4ed8:24'h101820)) : ui_rgb_base;
    ui_render u_ui (
        .i_map_pixel(map_pixel), .i_route_pixel(route_pixel), .i_obstacle_state(obstacle_state_ps2),
        .i_pixel_clk    (pixel_clk),
        .i_reset_n      (pixel_rst_n),
        .i_pixel_x      (pixel_x),
        .i_pixel_y      (pixel_y),
        .i_pixel_req    (pixel_req),
        .i_video_active (video_active),
        .o_pixel_rgb888 (ui_rgb_base),
        .i_page         (page_ps2),
        .i_menu_sel     (menu_ps2),
        .i_nav_run      (navrun_ps2),
        .i_nav_paused   (navpz_ps2),
        .i_menu_dim     (menu_dim_ps2),
        .i_aux_sel      (navaux_ps2),
        .i_car_px       (navcar_ps2_x),
        .i_car_py       (navcar_ps2_y),
        .i_cur_px       (navc_ps2_x),
        .i_cur_py       (navc_ps2_y),
        .i_pt_px        (navpt_ps2_x),
        .i_pt_py        (navpt_ps2_y),
        .i_pt_v         (navpv_ps2),
        .i_ui_mode      (mode_ps2),
        // ★ 2026-10-16：ui_render 的 i_run 端口已**删除**（模块内从不使用它，
        //   状态栏也没有"运行中"这一档）。其上游 sys 域 run_s 与 pixel 域
        //   run_ps1/run_ps2 整条跨域链随之删除；底盘运行指示仍由第 8 节的
        //   `assign o_led[0] = chassis_running;` 提供（chassis_running 未失去消费者）。
        .i_estop        (estop_ps2),
        .i_signal_ok    (sig_ps2),
        .i_arm_hint     (arm_ps2),
        .o_dyn_raddr    (dyn_raddr),
        .i_dyn_rdata    (dyn_rdata),
        // ★ 2026-10-19：IMU 初始化失败 → 数据页「航向角」上方红字「imu初始化错误」
        .i_imu_fail     (imufail_ps2),
        // ★ 2026-10-20：参数整定页的选中行（画高亮条用）
        .i_tune_sel     (tune_ps2),
        .i_tune_active  (tune_act_ps2)
    );

    // ==================================================================
    // 7.5 摄像头画面叠加（★ 2026-10-17 新增）
    // ==================================================================
    // 链路（架构照模块示例 examples\acg720_ddr3_hdmi，该示例已实板验证）：
    //   ov5640_capture →（PCLK 域写）→ ddr3_ctrl_2port 帧缓存 →（pixel 域读）→ 显示二选一
    //
    // 需求原文 §17/§21：左侧约 1/5~1/6 宽度是菜单，**选中摄像头页时菜单右侧全部画面**
    // 实时显示摄像头采集的画面，且该页不占摇杆。所以这里只把 `page == PAGE_CAMERA` 且
    // `pixel_x >= 菜单宽` 的区域换成摄像头像素，其余仍由 ui_render 负责。
    //
    // ★ 两个关键点（改动前先读）：
    //   1) ddr3_ctrl_2port 的 `rfifo_rden` 必须**绑 pixel_req 全程连续读**
    //      （见 rtl\cam\cam_overlay.v 注释）—— 只读显示区会让 DDR 读写地址累积错位。
    //   2) 摄像头 PCLK 与 pixel_clk 是**两个异步时钟**，跨域由 DDR 读写 FIFO 完成；
    //      不要试图用单级寄存器直接跨。
    //
    // 三个状态位当前只做占位（LED 已排满 8 个，接 wire 供以后扩展/示波器观察）。
    // ==================================================================
    wire cam_init_done_unused;
    wire ddr_init_done_unused;
    wire cam_pll_lock_unused;

    cam_overlay #(
        .AI_DISPLAY_ENABLE(1),        .IMAGE_WIDTH  (1024),
        .IMAGE_HEIGHT (600),
        .MENU_W       (208),            // 与 ui_render 的菜单宽度（208 px）保持一致
        .SYS_CLOCK    (CLK_HZ)
    ) u_cam (
        .sys_clk      (i_clk_50m),
        .sys_rst_n    (sys_rst_n),
        .ai_begin(inspection_frame_begin),.ai_commit(inspection_frame_commit),.ai_abort(inspection_frame_abort),
        .ai_session(inspection_session),.ai_task(inspection_task_id),.ai_epoch(inspection_epoch),.ai_context(inspection_context),
        .ai_sequence(inspection_frame_sequence),.ai_view(ai_inspection_view),.ai_frame_ready(inspection_frame_ready),
        .ai_pixel(inspection_pixel),.ai_pixel_valid(inspection_pixel_valid),.ai_pixel_ready(inspection_pixel_ready),
        .ai_show(inspection_permission && ai_host_alive && !manual_drive),.ai_fault(),

        // 摄像头物理引脚（直通到顶层）
        .camera_pwdn  (camera_pwdn),
        .camera_rst_n (camera_rst_n),
        .camera_sclk  (camera_sclk),
        .camera_sdat  (camera_sdat),
        .camera_data  (camera_data),
        .camera_pclk  (camera_pclk),
        .camera_href  (camera_href),
        .camera_vsync (camera_vsync),

        // DDR3 物理引脚（直通到顶层）
        .O_ddr_addr    (O_ddr_addr),
        .O_ddr_ba      (O_ddr_ba),
        .O_ddr_cs_n    (O_ddr_cs_n),
        .O_ddr_ras_n   (O_ddr_ras_n),
        .O_ddr_cas_n   (O_ddr_cas_n),
        .O_ddr_we_n    (O_ddr_we_n),
        .O_ddr_clk     (O_ddr_clk),
        .O_ddr_clk_n   (O_ddr_clk_n),
        .O_ddr_cke     (O_ddr_cke),
        .O_ddr_odt     (O_ddr_odt),
        .O_ddr_reset_n (O_ddr_reset_n),
        .O_ddr_dqm     (O_ddr_dqm),
        .IO_ddr_dq     (IO_ddr_dq),
        .IO_ddr_dqs    (IO_ddr_dqs),
        .IO_ddr_dqs_n  (IO_ddr_dqs_n),

        // 显示侧（pixel 域）
        .pixel_clk    (pixel_clk),
        .pixel_req    (pixel_req),
        .pixel_x      (pixel_x),
        .pixel_y      (pixel_y),
        .frame_sync   (frame_sync),
        // 摄像头页 = PAGE_CAMERA（2026-10-19 起编码为 1；此前是 0，换序后跟着变）
        .show_cam     (page_ps2 == PAGE_CAMERA),
        .ui_rgb       (ui_rgb_raw),
        .o_rgb        (ui_rgb),

        .cam_init_done (cam_init_done_unused),
        .ddr_init_done (ddr_init_done_unused),
        .pll_lock      (cam_pll_lock_unused)
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
    //       i_tof_int（ToF 中断 C20）/
    //       rc_select（路径点导航页辅助菜单换项 §33）/ rc_chan_us / rc_timeout /
    //       rc_valid / count_flat / enc_dir_vec / pid_sat / yaw_valid /
    //       frame_start / frame_sync / key1~key4_hold / IMU 状态与诊断位等。
    //     将来要用它们时，直接把消费者接上即可（端口与 wire 都还在）。

endmodule
