//////////////////////////////////////////////////////////////////////////////////
// 模块：odom —— 航位推算（里程计）
//
// 【依据】doc\使用FPGA实现的差速小车功能设计_ai优化版.md §5.3 + §5.1
//
//   原文：3. 航位计算。使用编码器采集的电机转速数据，用 IMU 解算的航向角进行矫正
//         （差速车可能有侧向打滑），计算当前航位并显示在屏幕上。
//
//   推算公式（四驱 skid-steer）：
//       v     = (v_l + v_r) / 2
//       ω     = k · (v_r − v_l) / L
//       yaw  += ω · dt
//       x    += v · cos(yaw) · dt
//       y    += v · sin(yaw) · dt
//
// 【本实现的取舍（第一版）】
//   1. **航向直接取陀螺 `i_yaw`**（`yaw_integrate` 的输出），不走 ω_enc 积分。
//      理由：IMU 已经接好且 `yaw_integrate` 在跑，陀螺的短时精度远好于轮速推算；
//      §5.1 的"编码器互补"是**精度优化**（压长期漂移），不是通路的前提。
//      所以本版先打通"陀螺航向 + 编码器位移"，`o_omega_enc` 先输出给打滑检测用。
//   2. **侧滑系数 k 暂不参与**：k 标定（原地转 360°，`k = θ_gyro/θ_enc`）还没做。
//      `o_omega_enc` 就是给标定用的，标完可以再引入互补。
//   3. **位移用 `delta`（10 ms 增量）而非 `count`**：`delta_valid` 自带时基，
//      且已经被 `ENC_INV_DIR_VEC` 校过极性（与轮速显示同一套符号）。
//
// 【★ 两个必须处理的坑（§5.3 原文点名）】
//   ① **角度解缠**：`yaw_integrate.o_yaw` 是 ±18000（±180°）**回绕**且**不含圈数**。
//      直接喂 cos/sin 会在车头绕过 ±180° 时把 x/y 算飞。
//      本模块自己维护 `o_yaw_cont`（连续航向，0.01°）：每拍取增量并折返到 ±18000 内再累加。
//   ② **`yaw_integrate.i_enable = 0` 的语义是"冻结并强制 yaw = 0"** ——
//      所以本模块**不能**用 i_enable 来"暂停积分"，只能靠 `i_clear` 重新定基准。
//      停车时 `yaw_cont` 自然保持不变（因为 i_yaw 不再变化、增量为 0），符合"冻结但保持"。
//
// 【定点约定】
//   位移：内部 **Q10 mm**（1 LSB = 1/1024 mm），输出取整到 mm。
//     1 个编码器计数 = πD / CNT_PER_REV = 201.062 / 61440 = 0.0032726 mm
//     ⇒ Δs_q10 = center_counts × 0.0032726 × 1024 = center_counts × 3.3515
//     用 `(center_counts × 3352) >> 10` 实现（误差 +0.024%）。
//   三角：256 点 Q15 表（sin_lut.vh），cos 用 sin 表偏移 90° 取。
//   乘法位宽：Δs_q10 最大 ≈ 13410（14 位）× cos_q15(15 位) ≈ 4.4e8 < 2^32 ✓
//
// 【精度】1 mm 轮径误差 ≈ 1.6% 里程误差（D=64mm 分母），所以 WHEEL_R_MM 必须实测。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module odom #(
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer WINDOW_MS   = 10,        // 测速窗口（与 speed_measure 同拍）
    parameter integer CNT_PER_REV = 61440,     // 编码器计数 / 车轮转（512PPR × 4 × 30）
    parameter integer WHEEL_R_MM  = 32,        // 轮**半径** mm ⇒ 直径 64 mm
    parameter integer TRACK_MM10  = 1615,      // 轮距 L，单位 0.1 mm（161.5 mm）
    // ---- 里程标定系数（★ 唯一需要实测校准的量）----
    //   含义：每个"平均计数/窗口"对应的位移，单位 Q10 mm × 1024
    //   理论值（D=64mm, 61440 计数/转）：
    //     Δs_q10 = count × πD / 61440 × 1024 = count × 3.3511 → 用 (count×3431)>>10
    //   2026-10-18 实测：推 3.00 m，推算 2.92 m（偏小 2.67%）
    //     反推两种可能：① 轮径实际 65.75mm（半径 32.88）② CPR 实际 63123（减速比 30.82）
    //     文档里"手转一圈≈61657 计数"只能解释 0.35%，故主因判为轮径（滚动半径含轮胎压缩）
    //     按实测校正：3431 × 3.00/2.92 = **3525**
    //   ★ 若后续确认路径够直、且用卷尺量过滚动周长，按同一比例再改这里即可。
    parameter integer DS_K_Q10    = 3525
)(
    input  wire                clk,
    input  wire                rst_n,

    // ---- 编码器：四轮 10 ms 增量（已含 ENC_INV_DIR_VEC 极性，正 = 车前进）----
    input  wire signed [31:0]  i_delta0,
    input  wire signed [31:0]  i_delta1,
    input  wire signed [31:0]  i_delta2,
    input  wire signed [31:0]  i_delta3,
    input  wire                i_delta_valid,

    // ---- IMU 航向（0.01°，±18000 回绕）----
    input  wire signed [31:0]  i_yaw,
    input  wire                i_yaw_valid,

    // ---- 清零（S2 / 串口 X 命令；与 yaw_clear 同源）----
    input  wire                i_clear,

    output reg  signed [31:0]  o_x_mm,        // 航位 x（mm）
    output reg  signed [31:0]  o_y_mm,        // 航位 y（mm）
    output reg  signed [31:0]  o_yaw_cont,    // 连续航向（0.01°，不折返）
    output reg  signed [31:0]  o_v_mms,       // 中心线速度（mm/s）
    output reg  signed [31:0]  o_omega_enc    // 编码器推算角速度（0.01°/s，标定/打滑检测用）

);

    // ------------------------------------------------------------------
    // 1. sin / cos 查找表：例化独立 ROM 模块（sin_lut_rom.v，256 点 Q15）
    //     ★ 为什么不是 `define 宏展开：256 行宏体超出 ModelSim 宏长度上限，
    //       vlog 会报 'Expecting numeric digits'（实测 127 个错误）。
    // ------------------------------------------------------------------
    wire signed [15:0] sin_v;
    wire signed [15:0] cos_v;

    // ------------------------------------------------------------------
    // 2. 四轮平均计数（每 10 ms 一次）
    // ------------------------------------------------------------------
    wire signed [31:0] d_sum  = i_delta0 + i_delta1 + i_delta2 + i_delta3;
    wire signed [31:0] d_cen  = d_sum >>> 2;              // 平均计数 / 10ms
    // 左右差：左侧 = {A, C}（与 chassis_ctrl 的 tgt_left 分组一致）、右侧 = {B, D}
    wire signed [31:0] d_left  = (i_delta0 + i_delta2) >>> 1;
    wire signed [31:0] d_right = (i_delta1 + i_delta3) >>> 1;

    // ------------------------------------------------------------------
    // 3. 位移增量（Q10 mm）与线速度（mm/s）
    //      1 count = πD/CNT_PER_REV = 0.201062/61440 = 3.2726e-3 mm
    //      Q10 表示：1 count = 3.2726e-3 × 1024 = 3.3511（Q10）
    //      ⇒ Δs_q10 = (d_cen × 3431) >> 10    （3431/1024 = 3.3506，误差 −0.015%）
    //      ★ 首版误写成 3352（把 3.2726e-3×1024×1000 当成了系数），导致里程偏小 2.4%
    //      v_mms  = Δs_q10 × 100 / 1024 = Δs_q10 × 25 >> 8
    // ------------------------------------------------------------------
    // ★ 组合计算：首版把它寄存在 always 里，导致 ds_q10 比 d_cen 慢一拍，
    //   再叠加 o_x_mm 又取"更新前"的 x_q10 —— 两个非阻塞延迟各丢一个窗口，
    //   实测 100 窗口只累到 328mm（应 335mm）。改成组合后一次寄存即可。
    wire signed [31:0] ds_q10 = (d_cen * DS_K_Q10) >>> 10;   // 标定系数见参数区

    // v_mms：用乘法而不是两段移位，避免精度损失
    //   v = ds_q10 × 100 / 1024 = ds_q10 × 25 / 256
    //   v = ds_q10 × 100 / 1024 = ds_q10 × 25 >> 8（组合）
    wire signed [31:0] v_mms_w = (ds_q10 * 32'sd25) >>> 8;

    // ------------------------------------------------------------------
    // 4. 编码器角速度（0.01 °/s）—— 给 §5.1 的 k 标定与打滑检测用
    //     ω_enc(rad/s) = (v_r − v_l) / L
    //     以 0.01mm/s 为单位：dv_01mms = (d_right − d_left) × 32.726
    //       1 count/10ms ⇒ 0.32726 mm/s ⇒ ×100 = 32.726 (0.01mm/s)
    //       ⇒ dv_01mms = (d_right − d_left) × 32726 >> 10
    //     ω(0.01°/s) = dv_01mms/100 mm/s ÷ L_mm × (180/π) × 100
    //                = dv_01mms / L_mm × 5729.58
    //     用 Q10 常数：5729.58 × 1024 / 1000 ≈ 5867（每 0.1mm 轮距）
    //     ⚠ 这一项只用于标定，精度要求不高（1% 足够），先用一次乘+移位。
    // ------------------------------------------------------------------
    wire signed [31:0] d_diff   = d_right - d_left;                       // counts / 10ms
    wire signed [31:0] dv_01mms = (d_diff * 32'sd32726) >>> 10;            // 0.01 mm/s

    // ω_01deg_s = dv_01mms × 5729.58 / L_mm，用 Q8 常数 146677（=5729.58×256/10）再 >>8
    wire signed [31:0] omega_enc_w = (dv_01mms * 32'sd146677) / TRACK_MM10 / 256;

    // ------------------------------------------------------------------
    // 5. 航向解缠（★ 坑①）
    //      d_raw = i_yaw − yaw_prev
    //      若 d_raw >  18000 → d_raw − 36000
    //      若 d_raw < −18000 → d_raw + 36000
    //      o_yaw_cont += d_raw
    //    ★ 用 `i_yaw_valid`（1 kHz）而不是 `i_delta_valid`（100 Hz）来采样，
    //      否则 10 ms 内转过的角度在解缠时会丢失分辨率（快速旋转时 >180°/10ms 不可能，
    //      但用 1 kHz 更稳且不额外花钱）。
    // ------------------------------------------------------------------
    reg signed [31:0] yaw_prev;
    reg               yaw_prev_vld;
    // ★ 必须是 **signed**：`yaw_wrap + dyaw` 里 dyaw 是 signed，若 yaw_wrap 是 unsigned，
    //   Verilog 会把整个表达式按 unsigned 算 —— `0 + (-9000)` 变成 4 294 958 552，
    //   折返逻辑（`< 0` 判据）直接失效，实测"左转 90° 后前进"会偏约 12°。
    reg signed [31:0] yaw_wrap;          // 圈内角度 0..35999（相对清零时刻，用于查表）

    wire signed [31:0] dyaw_raw = i_yaw - yaw_prev;
    wire signed [31:0] dyaw     = (yaw_prev_vld == 1'b0) ? 32'sd0 :
                                  (dyaw_raw >  32'sd18000) ? (dyaw_raw - 32'sd36000) :
                                  (dyaw_raw < -32'sd18000) ? (dyaw_raw + 32'sd36000) :
                                                             dyaw_raw;

    // ------------------------------------------------------------------
    // 6. 位置积分：x/y += Δs × cos/sin(yaw)
    //    三角索引：yaw_cont 是 0.01°，256 点表覆盖 36000 个 0.01°
    //      idx_sin = (yaw_cont × 256 / 36000) & 255
    //              = (yaw_cont × 8 / 1125) & 255
    //      idx_cos = idx_sin + 64（90°）
    //    乘法：Δs_q10(14b) × sin_q15(15b) = 29b ✓；再 >> 15 回到 Q10 mm
    // ------------------------------------------------------------------
    //   ★★ 2026-10-18 修正（首次综合卡死 17 分钟无输出的根因）：
    //     原写法用 `yaw_cont % 36000` 和 `(yaw_pos*8)/1125` 两个 **32 位组合除法器**，
    //     综合后是巨大的组合树，直接压垮布局布线。
    //     改成：① 另维护一个"圈内角度" `yaw_wrap`（0..35999），每次加 dyaw 后
    //              用比较+加减完成折返（dyaw 很小，一次折返足够）；
    //           ② 索引用"乘常数 + 右移"：256/36000 = 7457/2^20。
    //     验证：yaw_wrap=0 → 0；18000(180°) → 128；35999 → 255 ✓
    wire signed [31:0] idx_sin_w = (yaw_wrap * 32'sd7457) >>> 20;   // signed × signed，算术右移
    wire        [7:0]  idx_sin   = idx_sin_w[7:0];
    wire        [7:0]  idx_cos   = idx_sin + 8'd64;      // +90° = +64 点（256 点表的 1/4）

    sin_lut_rom u_sin (
        .i_idx (idx_sin),
        .o_val (sin_v)
    );
    sin_lut_rom u_cos (
        .i_idx (idx_cos),
        .o_val (cos_v)
    );

    // ------------------------------------------------------------------
    // Δs 投影到 x / y（Q10 mm × Q15 → >>15 回到 Q10 mm）
    //
    // ★★ 2026-10-18 修正（需求方实测报错）★★
    //   本车的坐标约定是：
    //     · 初始车头朝 **+y**
    //     · 车辆**右侧**为 **+x**
    //     · **向右转 ⇒ 航向角增大**
    //   在这个约定下，车头单位向量是：
    //       (x, y) = ( sin(yaw),  cos(yaw) )
    //   验证：yaw=0   → (0, +1) 朝 +y ✅
    //         yaw=90° → (+1, 0) 朝 +x ✅（右转 90° 后车头朝右）
    //         yaw=180°→ (0, −1) 朝 −y ✅
    //         yaw=270°→ (−1, 0) 朝 −x ✅
    //
    //   首版写成了 x += Δs·cos、y += Δs·sin —— 那是"yaw 自 +x 逆时针"的数学约定，
    //   本车不是那套约定。表现为：初始化 yaw=0 时车其实朝 +x，与"朝 +y"不符，
    //   移动时分量也对不上（需求方观察到"x 与 y 同时增大"这种 45° 现象）。
    // ------------------------------------------------------------------
    wire signed [31:0] dx_q10 = (ds_q10 * sin_v) >>> 15;   // x += Δs · sin(yaw)
    wire signed [31:0] dy_q10 = (ds_q10 * cos_v) >>> 15;   // y += Δs · cos(yaw)

    // 累加器（Q10 mm）与"下一拍值"（用 next 同时驱动寄存器与输出，避免慢一拍）
    reg signed [31:0] x_q10, y_q10;
    wire signed [31:0] x_next = x_q10 + dx_q10;
    wire signed [31:0] y_next = y_q10 + dy_q10;

    // ------------------------------------------------------------------
    // 7. 时序：清零优先；否则按各自的 valid 更新
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_q10        <= 32'sd0;
            y_q10        <= 32'sd0;
            o_x_mm       <= 32'sd0;
            o_y_mm       <= 32'sd0;
            o_yaw_cont   <= 32'sd0;
            o_v_mms      <= 32'sd0;
            o_omega_enc  <= 32'sd0;
            yaw_prev     <= 32'sd0;
            yaw_prev_vld <= 1'b0;
            yaw_wrap     <= 32'd0;
        end else if (i_clear) begin
            // ★ 清零：位置与连续航向一起归零，并重新对齐 yaw_prev
            x_q10        <= 32'sd0;
            y_q10        <= 32'sd0;
            o_x_mm       <= 32'sd0;
            o_y_mm       <= 32'sd0;
            o_yaw_cont   <= 32'sd0;
            yaw_prev     <= i_yaw;
            yaw_prev_vld <= 1'b1;
            // ★★ 2026-10-18 修正（上板实测报错后改回）★★
            //   航位用【相对航向】：**清零时刻的车头方向恒为 +y**。
            //   所以清零时 yaw_wrap 归 0，而不是跟随 i_yaw。
            //
            //   为什么不能用绝对航向：`yaw_integrate` 上电从 0 积分，但 IMU 有零偏/漂移，
            //   上电后几秒 yaw 就可能不是 0。若按绝对角投影，"沿车头直线前进"会被投到
            //   任意方向 —— 需求方实测到"y 减小、x 也减小"，说明实际投到了 ≈225°。
            //   用相对角后：清零时朝哪，+y 就朝哪，与 yaw 的绝对值无关 ✅
            yaw_wrap     <= 32'd0;
        end else begin
            // ---- 航向（1 kHz 采样，解缠并累加）----
            if (i_yaw_valid) begin
                yaw_prev     <= i_yaw;
                yaw_prev_vld <= 1'b1;
                o_yaw_cont   <= o_yaw_cont + dyaw;
                // 圈内角度折返：dyaw 每拍最多几度，一次比较足够
                if (yaw_wrap + dyaw >= 32'sd36000)
                    yaw_wrap <= yaw_wrap + dyaw - 32'sd36000;
                else if (yaw_wrap + dyaw < 32'sd0)
                    yaw_wrap <= yaw_wrap + dyaw + 32'sd36000;
                else
                    yaw_wrap <= yaw_wrap + dyaw;
            end

            // ---- 位移与速度（10 ms 一次）----
            if (i_delta_valid) begin
                x_q10       <= x_next;
                y_q10       <= y_next;
                // ★ 必须用算术右移 `>>>`，不能写 `x_next[31:10]` ——
                //   位选是**逻辑右移**，负航位会变成巨大的正数（实测倒车 y 显示 4193968）。
                o_x_mm      <= x_next >>> 10;
                o_y_mm      <= y_next >>> 10;
                o_v_mms     <= v_mms_w;
                o_omega_enc <= omega_enc_w;
            end
        end
    end

    // synopsys translate_off
    // 参数自检：轮径/轮距必须落在合理范围，否则换算系数一定是错的
    initial begin
        if (WHEEL_R_MM < 10 || WHEEL_R_MM > 200)
            $display("ERROR: odom WHEEL_R_MM=%0d 不合理（应为 10~200 mm 的**半径**）", WHEEL_R_MM);
        if (TRACK_MM10 < 500 || TRACK_MM10 > 5000)
            $display("ERROR: odom TRACK_MM10=%0d 不合理（应为 500~5000，单位 0.1mm）", TRACK_MM10);
        if (CNT_PER_REV < 1000)
            $display("ERROR: odom CNT_PER_REV=%0d 不合理", CNT_PER_REV);
        if (WINDOW_MS != 10)
            $display("WARNING: odom 的位移/速度换算按 WINDOW_MS=10 写死（×25>>8 即 ×100/1024），改成 %0d 会算错",
                     WINDOW_MS);
    end
    // synopsys translate_on

endmodule
