//////////////////////////////////////////////////////////////////////////////////
// 模块：motor_pwm —— 单路直流电机 PWM 驱动（TB6612FNG / WHEELTEC D24A）
//
// 【它做什么】
//   把「占空比 + 方向 + 刹车方式 + 使能」翻译成 TB6612 需要的三根线
//       xIN1 / xIN2  : 方向控制（普通 IO 电平即可）
//       PWMx         : 调速（占空比调制）
//   一个模块只管一个电机；四路就例化 4 次（见 motor_top.v）。
//
// 【TB6612 真值表】（依据 Toshiba 官方手册；注意厂家的简化表把 IN1=IN2=0
//   也写成"制动停止"，实际那是输出高阻的【滑行】，本项目以官方行为为准）
//
//   IN1  IN2   PWM   电机状态
//   ---------------------------------------------------------------
//    H    L    调制   正转
//    H    L     L     短路刹车
//    L    H    调制   反转
//    L    H     L     短路刹车
//    H    H   任意    短路刹车（电机两端被短接，急停）
//    L    L   任意    停止 / 滑行（输出高阻，电机自由转）
//    STBY = 0         整片待机（所有输出关闭）
//
//   ★ 重要：只给 IN1/IN2 不产生输出，必须有 PWM 输入电机才会转。
//   ★ 重要：PWM=L 的每个周期内 TB6612 处于"短路刹车"，这是 drive/brake
//           模式的特点——响应快但会发热。若发现电机偏热，可改用
//           "PWM 固定在 H、用 IN1 打 PWM"的 drive/coast 接法。
//
// 【本模块的对外语义】
//   停止有两种，由 i_brake 选择：
//       i_brake = 0  → 滑行（coast），电机自由转，慢慢停
//       i_brake = 1  → 刹车（short brake），急停
//   触发停止的条件有两个（都会被 i_brake 生效）：
//       i_enable = 0        （上层主动停）
//       i_duty   = 0        （占空比为 0 等价于停，避免"PWM 恒 0 却在刹车"
//                            造成电机嗡嗡响/发热）
//
// 【PWM 周期为什么取 2 的幂】
//   周期 = 2^PWM_BITS 拍，于是占空比 = i_duty / 2^PWM_BITS，不需要任何乘法
//   或除法：i_duty = 2^(PWM_BITS-1) 就是精确的 50%。这与 yaw_integrate 选
//   1024Hz 节拍是同一个设计哲学（取 2 的幂，消掉乘除法与截断误差）。
//
//   默认 PWM_BITS = 11 → 周期 2048 拍 → 50MHz/2048 = 24.41 kHz
//   （TB6612 的 PWM 允许 0~100kHz，手册推荐 10kHz；取 24.41kHz 是为了
//    落在人耳听觉上限以上，避免电机啸叫，同时远低于 100kHz。）
//
//   频率对照（CLK = 50MHz）：
//       PWM_BITS = 9  → 97.7 kHz   （太接近上限，不推荐）
//       PWM_BITS = 10 → 48.8 kHz
//       PWM_BITS = 11 → 24.41 kHz  ★默认
//       PWM_BITS = 12 → 12.21 kHz  （可能有轻微可闻噪声）
//
// 【上电安全】★ 这一条很重要
//   TB6612 的 IN1/IN2 若悬空，电机可能在上电瞬间乱转。本模块在复位期间
//   强制输出 IN1=IN2=0（滑行，输出高阻）、PWM=0；上层还应配合：
//     1) .cst 里给 IN1/IN2/PWM 引脚加 PULL_MODE=DOWN
//     2) 转接板上给 STBY 加 10kΩ 下拉
//   三层防护才能保证"上电/复位时小车不会自己窜出去"。
//
// 参数：
//   CLK_FREQ_HZ   系统时钟频率(Hz)     默认 50_000_000
//   PWM_BITS      PWM 计数器位宽       默认 11（周期 2^11 = 2048 拍）
//   INV_DIR    1 = 反转方向语义     默认 0（用于不想改接线就反过来）
//
// 端口：
//   i_clock      输入  系统时钟
//   i_reset_n    输入  异步低电平复位
//   i_enable     输入  1 = 允许驱动；0 = 停止（按 i_brake 决定滑行/刹车）
//   i_dir        输入  0 = 正转，1 = 反转
//   i_brake      输入  停止方式：0 = 滑行，1 = 刹车
//   i_duty       输入  占空比，0 ~ 2^PWM_BITS-1（0 = 停，满值 ≈ 100%）
//   o_in1        输出  TB6612 xIN1
//   o_in2        输出  TB6612 xIN2
//   o_pwm        输出  TB6612 PWMx
//   o_active     输出  当前是否在驱动（= i_enable 且 i_duty != 0）
//////////////////////////////////////////////////////////////////////////////////

module motor_pwm #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer PWM_BITS    = 11,
    parameter integer INV_DIR  = 0
)(
    input  wire                    i_clock,
    input  wire                    i_reset_n,

    input  wire                    i_enable,
    input  wire                    i_dir,
    input  wire                    i_brake,
    input  wire [PWM_BITS-1:0]     i_duty,

    output wire                    o_in1,
    output wire                    o_in2,
    output wire                    o_pwm,
    output wire                    o_active
);

    //---------------------------------------------------------------
    // 1. PWM 计数器：自由运行，周期恒为 2^PWM_BITS
    //    把 i_duty 定义为「以 2^PWM_BITS 为满刻度」，即占空比 = duty / 2^PWM_BITS
    //---------------------------------------------------------------
    reg [PWM_BITS-1:0] pwm_cnt;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) pwm_cnt <= {PWM_BITS{1'b0}};
        else            pwm_cnt <= pwm_cnt + 1'b1;      // 自然回绕，无需比较
    end

    //---------------------------------------------------------------
    // 2. 有效驱动判定
    //    duty = 0 时同样视为停止：否则 TB6612 会一直处于"IN 有效但 PWM 恒 0"
    //    的短路刹车状态（电机嗡嗡响、发热，却不转）。
    //    enable=0 时立即停止（不等 PWM 周期结束）。
    //---------------------------------------------------------------
    wire duty_nz  = |i_duty;
    wire active   = i_enable & duty_nz;

    // PWM 原始波形（占空比 = duty / 2^PWM_BITS）
    wire pwm_raw  = (pwm_cnt < i_duty);

    //---------------------------------------------------------------
    // 3. 输出组合（纯组合，逻辑很浅：11 位比较 + 几个选择）
    //    方向位按 INV_DIR 取反，方便不拆线就纠正"A/B 相序/电机极性"相反
    //---------------------------------------------------------------
    wire dir_fwd = INV_DIR ? i_dir : ~i_dir;        // 1 = 正转

    reg in1_c, in2_c, pwm_c;

    always @(*) begin
        if (!active) begin
            if (i_brake) begin
                // 短路刹车：IN1=IN2=1（PWM 电平在此模式下无关，给 0 更保守）
                in1_c = 1'b1;
                in2_c = 1'b1;
                pwm_c = 1'b0;
            end else begin
                // 滑行停止：IN1=IN2=0，输出高阻
                in1_c = 1'b0;
                in2_c = 1'b0;
                pwm_c = 1'b0;
            end
        end else if (dir_fwd) begin
            in1_c = 1'b1;      // 正转
            in2_c = 1'b0;
            pwm_c = pwm_raw;
        end else begin
            in1_c = 1'b0;      // 反转
            in2_c = 1'b1;
            pwm_c = pwm_raw;
        end
    end

    assign o_in1    = in1_c;
    assign o_in2    = in2_c;
    assign o_pwm    = pwm_c;
    assign o_active = active;

endmodule
