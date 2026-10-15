//////////////////////////////////////////////////////////////////////////////////
// 模块：gyro_tilt_comp —— 陀螺 Z 轴「固定安装倾斜」补偿
//
// 【为什么需要】
//   陀螺测的是【本体轴】上的角速度分量。当 IMU 相对竖直轴倾斜 α 时，绕竖直轴
//   以角速度 Ω 旋转，在本体 z 轴上只产生
//         ω_z = Ω · cos α
//   直接积分 ω_z 得到的是 ψ·cos α，属于纯标度误差：
//        倾斜 10° → 误差 1.5%     30° → 13.4%     45° → 29.3%     90° → 100%
//
// 【补偿原理】
//   静止时加速度计测到的就是重力方向，也就是「竖直轴在本体坐标系中的方向」：
//         v_B = a_B / |a_B|                       （单位矢量）
//   于是 cos α = v_B,z = a_z / |a_B|，因此
//         真实偏航角速度 = ω_z / cos α = ω_z · |a_B| / a_z
//   本模块在标定阶段测出增益 k = |a_B| / a_z（Q16 定点），运行阶段做
//         o_gyro_z = i_gyro_z × k >> 16
//
//   k 自动带符号：IMU 倒置安装时 a_z < 0 → k < 0 → 输出符号自动翻回来，
//   不需要额外的方向配置位。
//
// 【标定时机】
//   i_calib_start 为高期间（顶层接 imu_top 的 o_calib_busy）按 i_sample_valid
//   累加加速度计样本，取满 CALIB_SAMPLES 个后停止累加；i_calib_start 的下降沿
//   启动计算。这样直接复用零偏标定的静止窗口，不额外要求用户保持静止。
//   若样本不足 CALIB_SAMPLES（例如零偏标定超时），则放弃补偿（k 保持 1.0）
//   并置 o_tilt_fail。
//
// 【定点流程】全部只在标定结束后执行一次，约 55 拍
//   1. 求均值      m = Σa >> CALIB_SAMPLES_LG           （有符号）
//   2. 平方和      s = m_x² + m_y² + m_z²               （16×16，复用 1 个乘法器）
//   3. 开方        |a| = isqrt(s)                       （二分逼近，16 拍）
//   4. 除法        |k| = (|a| << 16) / |m_z|            （恢复余数法，32 拍）
//   5. 限幅定符号  |k| ∈ [GAIN_MIN_Q16, GAIN_MAX_Q16]，符号取自 m_z
//
// 【局限】
//   1. 只补偿【固定】倾斜。车体动态侧倾/俯仰时真实 v_B 在变，而本模块用的是
//      标定值，仍有残差（残差与姿态变化量和角速度的乘积同阶）。
//   2. 前提是「绕竖直轴旋转」。若旋转轴明显偏离竖直，本补偿不适用。
//   3. 重力对偏航无约束，本模块【不】解决陀螺 yaw 的长期漂移——那是原理性的，
//      只能靠磁力计等外部观测。
//
// 参数：
//   CALIB_SAMPLES_LG  标定样本数取 2 的幂的指数，默认 8 → 256 个样本
//   GAIN_MIN_Q16      增益下限（Q16），默认 32768（0.5）
//   GAIN_MAX_Q16      增益上限（Q16），默认 131072（2.0，对应倾斜 60°）
//
// 端口：
//   i_clock        输入  系统时钟
//   i_reset_n      输入  异步低电平复位
//   i_acc_x/y/z    输入  加速度，mg（int16），静止时用于测倾角
//   i_gyro_z       输入  待补偿的角速度，0.01°/s（int32）
//   i_sample_valid 输入  样本更新脉冲
//   i_calib_start  输入  标定窗口，高有效；下降沿触发计算
//   o_gyro_z       输出  补偿后的角速度，0.01°/s（int32）
//   o_ready        输出  增益已确定（标定流程结束）
//   o_tilt_fail    输出  倾斜过大或样本不足，已退化为限幅/不补偿
//   o_gain_q16     输出  当前增益（Q16，1.0 = 65536），可用来观察倾斜程度
//   o_calib_busy   输出  正在计算增益
//////////////////////////////////////////////////////////////////////////////////

module gyro_tilt_comp #(
    parameter integer CALIB_SAMPLES_LG = 8,
    parameter integer GAIN_MIN_Q16     = 32768,
    parameter integer GAIN_MAX_Q16     = 131072
)(
    input  wire               i_clock,
    input  wire               i_reset_n,

    input  wire signed [15:0] i_acc_x,
    input  wire signed [15:0] i_acc_y,
    input  wire signed [15:0] i_acc_z,

    input  wire signed [31:0] i_gyro_z,
    input  wire               i_sample_valid,
    input  wire               i_calib_start,

    output wire signed [31:0] o_gyro_z,
    output wire               o_ready,
    output wire               o_tilt_fail,
    output wire [31:0]        o_gain_q16,
    output wire               o_calib_busy
);

    localparam integer CALIB_SAMPLES = (1 << CALIB_SAMPLES_LG);

    localparam [2:0] S_IDLE = 3'd0,
                     S_SQ   = 3'd1,
                     S_SQRT = 3'd2,
                     S_DIV  = 3'd3,
                     S_FIN  = 3'd4;

    //---------------------------------------------------------------
    // 1. 标定窗口内累加加速度计（独立于状态机，只受 i_calib_start 控制）
    //---------------------------------------------------------------
    reg signed [31:0] sum_x, sum_y, sum_z;
    reg        [15:0] acc_cnt;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            sum_x   <= 32'sd0;
            sum_y   <= 32'sd0;
            sum_z   <= 32'sd0;
            acc_cnt <= 16'd0;
        end else if (i_calib_start && i_sample_valid && (acc_cnt < CALIB_SAMPLES)) begin
            sum_x   <= sum_x + {{16{i_acc_x[15]}}, i_acc_x};
            sum_y   <= sum_y + {{16{i_acc_y[15]}}, i_acc_y};
            sum_z   <= sum_z + {{16{i_acc_z[15]}}, i_acc_z};
            acc_cnt <= acc_cnt + 16'd1;
        end
    end

    // i_calib_start 下降沿 = 标定窗口结束，启动计算
    reg  calib_start_d;
    wire calib_fall = calib_start_d & ~i_calib_start;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) calib_start_d <= 1'b0;
        else            calib_start_d <= i_calib_start;
    end

    //---------------------------------------------------------------
    // 2. 均值（有符号）。取满 CALIB_SAMPLES 个才有效
    //    用 indexed part-select 取「>> CALIB_SAMPLES_LG 后的低 16 位」，
    //    等价于算术右移但不会触发工具的位宽截断警告（EX3791）。
    //    累加范围 ±256×32767 ≈ ±8.4e6，右移 8 位后必落在 int16 内。
    //---------------------------------------------------------------
    wire signed [15:0] m_x = sum_x[CALIB_SAMPLES_LG +: 16];
    wire signed [15:0] m_y = sum_y[CALIB_SAMPLES_LG +: 16];
    wire signed [15:0] m_z = sum_z[CALIB_SAMPLES_LG +: 16];

    //---------------------------------------------------------------
    // 3. 平方和：复用 1 个 16×16 乘法器，4 拍
    //    乘法结果比 mul_in 晚一拍有效，故 step1 才把 m_x² 存入累加器
    //---------------------------------------------------------------
    reg signed [15:0] mul_in;
    wire signed [31:0] mul_sq = mul_in * mul_in;

    reg        [1:0]  sq_step;
    reg        [31:0] sq_sum;

    //---------------------------------------------------------------
    // 4. 开方：二分逼近，从 bit15 试到 bit0，共 16 拍
    //    （sqrt_try 无符号，最大 65535，平方不超过 32 位无符号上限）
    //---------------------------------------------------------------
    reg        [15:0] sqrt_root;
    reg        [3:0]  sqrt_bit;
    wire       [15:0] sqrt_try    = sqrt_root | (16'd1 << sqrt_bit);
    wire       [31:0] sqrt_try_sq = sqrt_try * sqrt_try;
    wire              sqrt_take   = (sqrt_try_sq <= sq_sum);
    // 处理完 bit0 后的最终根（组合取出，供除法装载）
    wire       [15:0] sqrt_final  = sqrt_take ? sqrt_try : sqrt_root;

    //---------------------------------------------------------------
    // 5. 除法：恢复余数法，32 拍。被除数 = |a| << 16，除数 = |m_z|
    //---------------------------------------------------------------
    reg        [31:0] div_shift;      // 被除数，每拍左移
    reg        [16:0] div_rem;
    reg        [31:0] div_quot;
    reg        [5:0]  div_cnt;

    wire       [15:0] az_abs  = m_z[15] ? (~m_z + 16'd1) : m_z;
    wire       [16:0] rem_sh  = {div_rem[15:0], div_shift[31]};
    wire              div_ge  = (rem_sh >= {1'b0, az_abs});

    //---------------------------------------------------------------
    // 6. 增益与状态
    //---------------------------------------------------------------
    reg  signed [31:0] gain_r;        // Q16，复位值 65536 = 1.0（不补偿）
    reg                ready_r;
    reg                tilt_fail_r;
    reg        [2:0]   state;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state       <= S_IDLE;
            sq_step     <= 2'd0;
            sq_sum      <= 32'd0;
            mul_in      <= 16'sd0;
            sqrt_root   <= 16'd0;
            sqrt_bit    <= 4'd15;
            div_shift   <= 32'd0;
            div_rem     <= 17'd0;
            div_quot    <= 32'd0;
            div_cnt     <= 6'd0;
            gain_r      <= 32'sd65536;      // 默认 1.0
            ready_r     <= 1'b0;
            tilt_fail_r <= 1'b0;
        end else begin
            case (state)
                //-----------------------------------------------
                S_IDLE: begin
                    if (calib_fall) begin
                        if (acc_cnt >= CALIB_SAMPLES) begin
                            // 样本足够，启动计算
                            sq_step  <= 2'd0;
                            sq_sum   <= 32'd0;
                            sqrt_root<= 16'd0;
                            sqrt_bit <= 4'd15;
                            state    <= S_SQ;
                        end else begin
                            // 样本不足（例如零偏标定超时）：不补偿
                            ready_r     <= 1'b1;
                            tilt_fail_r <= 1'b1;
                        end
                    end
                end
                //-----------------------------------------------
                // 平方和：4 拍
                S_SQ: begin
                    case (sq_step)
                        2'd0: begin
                            mul_in  <= m_x;
                            sq_step <= 2'd1;
                        end
                        2'd1: begin
                            sq_sum  <= mul_sq;          // m_x²
                            mul_in  <= m_y;
                            sq_step <= 2'd2;
                        end
                        2'd2: begin
                            sq_sum  <= sq_sum + mul_sq; // + m_y²
                            mul_in  <= m_z;
                            sq_step <= 2'd3;
                        end
                        default: begin
                            sq_sum  <= sq_sum + mul_sq; // + m_z²
                            sq_step <= 2'd0;
                            state   <= S_SQRT;
                        end
                    endcase
                end
                //-----------------------------------------------
                // 开方：16 拍
                S_SQRT: begin
                    if (sqrt_bit == 4'd0) begin
                        if (sqrt_take) sqrt_root <= sqrt_try;
                        // 装载除法：被除数 = |a| << 16
                        div_shift <= {sqrt_final, 16'd0};
                        div_rem   <= 17'd0;
                        div_quot  <= 32'd0;
                        div_cnt   <= 6'd32;
                        state     <= S_DIV;
                    end else begin
                        if (sqrt_take) sqrt_root <= sqrt_try;
                        sqrt_bit <= sqrt_bit - 4'd1;
                    end
                end
                //-----------------------------------------------
                // 除法：32 拍
                S_DIV: begin
                    if (div_cnt == 6'd0) begin
                        state <= S_FIN;
                    end else begin
                        div_rem   <= div_ge ? (rem_sh - {1'b0, az_abs}) : rem_sh;
                        div_quot  <= {div_quot[30:0], div_ge};
                        div_shift <= {div_shift[30:0], 1'b0};
                        div_cnt   <= div_cnt - 6'd1;
                    end
                end
                //-----------------------------------------------
                // 限幅 + 定符号
                S_FIN: begin
                    if (div_quot > GAIN_MAX_Q16[31:0]) begin
                        gain_r      <= m_z[15] ? -$signed(GAIN_MAX_Q16[31:0])
                                               :  $signed(GAIN_MAX_Q16[31:0]);
                        tilt_fail_r <= 1'b1;        // 倾斜 > 60°
                    end else if (div_quot < GAIN_MIN_Q16[31:0]) begin
                        gain_r      <= m_z[15] ? -$signed(GAIN_MIN_Q16[31:0])
                                               :  $signed(GAIN_MIN_Q16[31:0]);
                        tilt_fail_r <= 1'b1;        // 异常值
                    end else begin
                        gain_r      <= m_z[15] ? -$signed(div_quot)
                                               :  $signed(div_quot);
                        tilt_fail_r <= 1'b0;
                    end
                    ready_r <= 1'b1;
                    state   <= S_IDLE;
                end
                //-----------------------------------------------
                default: state <= S_IDLE;
            endcase
        end
    end

    //---------------------------------------------------------------
    // 7. 输出：一次 32×32 乘法，取 >>16
    //    i_gyro_z 实际只用 19 位、gain 只用 18 位，乘积不会溢出 48 位
    //---------------------------------------------------------------
    wire signed [63:0] comp_prod = i_gyro_z * gain_r;

    assign o_gyro_z     = comp_prod[47:16];
    assign o_ready      = ready_r;
    assign o_tilt_fail  = tilt_fail_r;
    assign o_gain_q16   = gain_r;
    assign o_calib_busy = (state != S_IDLE);

endmodule
