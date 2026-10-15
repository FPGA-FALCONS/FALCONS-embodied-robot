//////////////////////////////////////////////////////////////////////////////////
// 模块：motor_top —— 四路电机驱动 + 转速采集（WHEELTEC D24A / TB6612FNG ×2）
//
// 【结构】
//   本模块不含算法，只是把 4 组「单路驱动 + 正交解码 + 测速」拼起来，
//   并统一管理 TB6612 的 STBY：
//
//       i_enable/dir/brake/duty ─→ motor_pwm ×4 ─→ o_in1/in2/pwm ×4（接 J4/J5）
//       i_enc_a/b              ─→ quad_decoder ×4 ─→ o_count / o_enc_dir
//                                      └→ speed_measure ×4 ─→ o_delta
//
//   4 路的位序（A/B 在 J4 上，对应编码器 E1/E2；C/D 在 J5 上，对应 E3/E4）：
//       bit0 = Motor A    bit1 = Motor B    bit2 = Motor C    bit3 = Motor D
//   哪个位对应车上哪个轮位由接线与装车决定，本模块不作假设。
//
//   【接线参考 —— 已上板验证】按 car_fpga 工程的 J4/J5 排线接法：
//       A = 左前(E1)    B = 右前(E2)    C = 左后(E3)    D = 右后(E4)
//   该接线下实测的编码器计数符号与极性标定相符，四路计数同号：
//       INV_DIR_VEC     = 4'b0000   四轮转向都对
//       ENC_INV_DIR_VEC = 4'b1010   左侧不动、右侧反相（左右电机镜像安装）
//   将来接线可照此复刻；具体引脚号见 constraints/acg720_motor_tb6612.cst。
//
// 【STBY 管理 ★ 安全要点】
//   驱动模块出厂时用跳线帽把 STBY 短接到 3V3（永久使能）。**拔掉跳线帽后
//   STBY 悬空**，如果直接接 FPGA 而不加下拉，上电配置期间（0.1~1s，此时
//   FPGA 的 IO 是高阻）电机可能被噪声误触发而乱转。
//   本模块的做法：o_stby = 「任一路 i_enable 为 1」，复位期间强制 0。
//   上层还必须配合：
//      1) .cst 里给 o_stby 引脚加 PULL_MODE=DOWN
//      2) 转接板上给 STBY 到 GND 加 10 kΩ 下拉电阻
//   三层防护齐了，才能保证"上电/复位时小车不会自己窜出去"。
//
//   注意 STBY 与刹车的区别：
//       清零 i_enable 且 i_brake=1  → 短路刹车（电机被短接，急停，仍有保持力）
//       清零 i_enable（STBY 自动变 0）→ 整片待机（输出全关，电机自由滑行）
//   想彻底断电就用后者，想"停住不动"就用前者。
//
// 【编码器计数位宽】
//   内部固定 32 位有符号。500PPR 的 GMR 编码器 4 倍频后每转 2000 计数，
//   32 位可以数到 ±100 万转，绝无溢出之虞。
//
// 参数：
//   CLK_FREQ_HZ   系统时钟频率(Hz)     默认 50_000_000
//   PWM_BITS      PWM 计数器位宽       默认 11（周期 2048 拍 → 24.41kHz @50MHz）
//   FILTER_BITS   编码器去毛刺指数      默认 3（需连续 8 拍一致）
//   WINDOW_MS     测速窗口(ms)         默认 10（100Hz 速度更新率）
//   INV_DIR_VEC       4 位，每位对应一路的【电机方向】是否取反（默认全 0）
//                 用于"该路电机线/IN 线接反"→ 给 dir=0 却反转
//   ENC_INV_DIR_VEC   4 位，每位对应一路的【编码器计数方向】是否取反（默认全 0）
//                 用于"电机转向对、但计数符号与其它路相反"。
//                 典型场景：左右电机镜像安装、编码器接线约定相同，
//                 于是同样"向前"却得到相反符号——此时反编码器即可统一。
//                 ★ 不能靠 INV_DIR_VEC 替代：反电机会把转向也弄反。
//
// 端口：
//   i_clock        输入  系统时钟
//   i_reset_n      输入  异步低电平复位
//   i_enable       输入  [3:0] 每路使能
//   i_dir          输入  [3:0] 每路方向，0 = 正转，1 = 反转
//   i_brake        输入  [3:0] 每路停止方式，0 = 滑行，1 = 刹车
//   i_duty         输入  占空比，4 路拼接：bit[PWM_BITS-1:0] 为 Motor A，依次类推
//   o_stby         输出  TB6612 STBY（任一路使能时拉高）
//   o_in1/o_in2    输出  [3:0] 每路 TB6612 方向线
//   o_pwm          输出  [3:0] 每路 TB6612 PWM
//   i_enc_a/i_enc_b 输入 [3:0] 四路编码器 A/B 相
//   i_count_clear  输入  位置计数清零（单周期脉冲，须已同步去抖）
//   o_count        输出  4 路位置计数拼接（每路 32 位有符号）
//   o_delta        输出  4 路测速增量拼接（每路 32 位有符号）
//   o_delta_valid  输出  测速窗口结束脉冲（4 路同步，合并为一个）
//   o_enc_dir      输出  [3:0] 每路最近运动方向：1 = 计数递增
//////////////////////////////////////////////////////////////////////////////////

module motor_top #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer PWM_BITS    = 11,
    parameter integer FILTER_BITS = 3,
    parameter integer WINDOW_MS   = 10,
    parameter [3:0]   INV_DIR_VEC     = 4'b0000,   // 每路【电机方向】取反
    parameter [3:0]   ENC_INV_DIR_VEC = 4'b0000,   // 每路【编码器计数方向】取反
    // ---- 堵转保护（详细原理见 stall_detect.v）----
    parameter integer STALL_STARTUP_WINDOWS = 30,   // 启动宽限 30 窗口 = 300ms @10ms
    parameter integer STALL_WINDOWS         = 20,   // 判定窗口 20 个 = 200ms
    parameter integer STALL_DUTY_TH         = 200,  // 占空比阈值，≈10% of 2048
    parameter integer STALL_MOVE_TH         = 300   // 净位移阈值(计数)，≈1.8° @60000/转
)(
    input  wire                      i_clock,
    input  wire                      i_reset_n,

    input  wire [3:0]                i_enable,
    input  wire [3:0]                i_dir,
    input  wire [3:0]                i_brake,
    input  wire [4*PWM_BITS-1:0]     i_duty,
    input  wire                      i_stall_clear,   // 清除堵转锁存（单周期脉冲）

    output wire                      o_stby,
    output wire [3:0]                o_in1,
    output wire [3:0]                o_in2,
    output wire [3:0]                o_pwm,
    output wire [3:0]                o_stall,         // 每路堵转标志（锁存）

    input  wire [3:0]                i_enc_a,
    input  wire [3:0]                i_enc_b,
    input  wire                      i_count_clear,

    output wire [4*32-1:0]           o_count,
    output wire [4*32-1:0]           o_delta,
    output wire                      o_delta_valid,
    output wire [3:0]                o_enc_dir
);

    localparam integer CNT_BITS = 32;

    //---------------------------------------------------------------
    // 内部连线（Verilog-2001 端口不能是数组，故内部用数组、出口再拼接）
    //---------------------------------------------------------------
    wire signed [CNT_BITS-1:0] count_w [0:3];
    wire signed [CNT_BITS-1:0] delta_w [0:3];
    wire                       valid_w [0:3];

    //---------------------------------------------------------------
    // STBY：任一路使能才拉起；复位期间恒为 0（待机）
    //   打一拍输出，避免组合信号直接出引脚
    //---------------------------------------------------------------
    reg stby_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) stby_r <= 1'b0;
        else            stby_r <= |i_enable;
    end

    assign o_stby = stby_r;

    //---------------------------------------------------------------
    // 4 路例化（每路一个 motor_unit：驱动 + 解码 + 测速 + 堵转保护）
    //   相比手工拼 4 个原子模块，这里每路只例化一次；delta / delta_valid /
    //   使能通路这些【跨模块契约】全部封在 motor_unit 内部，不会在顶层接错。
    //   原子模块本身仍然独立，需要下探细节时可直接替换其中某一个。
    //   详见 motor_unit.v。
    //---------------------------------------------------------------
    genvar g;

    generate
        for (g = 0; g < 4; g = g + 1) begin : gen_motor

            wire [PWM_BITS-1:0] duty_g = i_duty[g*PWM_BITS +: PWM_BITS];

            motor_unit #(
                .CLK_FREQ_HZ           (CLK_FREQ_HZ),
                .PWM_BITS              (PWM_BITS),
                .FILTER_BITS           (FILTER_BITS),
                .WINDOW_MS             (WINDOW_MS),
                .INV_DIR            (INV_DIR_VEC[g]),
                .ENC_INV_DIR        (ENC_INV_DIR_VEC[g]),
                .STALL_STARTUP_WINDOWS (STALL_STARTUP_WINDOWS),
                .STALL_WINDOWS         (STALL_WINDOWS),
                .STALL_DUTY_TH         (STALL_DUTY_TH),
                .STALL_MOVE_TH         (STALL_MOVE_TH)
            ) u_unit (
                .i_clock       (i_clock),
                .i_reset_n     (i_reset_n),
                .i_enable      (i_enable[g]),
                .i_dir         (i_dir[g]),
                .i_brake       (i_brake[g]),
                .i_duty        (duty_g),
                .i_stall_clear (i_stall_clear),
                .o_in1         (o_in1[g]),
                .o_in2         (o_in2[g]),
                .o_pwm         (o_pwm[g]),
                .i_enc_a       (i_enc_a[g]),
                .i_enc_b       (i_enc_b[g]),
                .i_count_clear (i_count_clear),
                .o_count       (count_w[g]),
                .o_delta       (delta_w[g]),
                .o_delta_valid (valid_w[g]),
                .o_enc_dir     (o_enc_dir[g]),
                .o_stall       (o_stall[g])
            );

        end
    endgenerate

    //---------------------------------------------------------------
    // 输出拼接（低位对应 Motor A）
    //   4 个 speed_measure 用同一个复位启动、同参数，窗口天然同步，
    //   所以 valid 直接相或即可。
    //---------------------------------------------------------------
    assign o_count       = {count_w[3], count_w[2], count_w[1], count_w[0]};
    assign o_delta       = {delta_w[3], delta_w[2], delta_w[1], delta_w[0]};
    assign o_delta_valid = valid_w[0] | valid_w[1] | valid_w[2] | valid_w[3];

endmodule
