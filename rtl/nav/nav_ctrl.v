//////////////////////////////////////////////////////////////////////////////////
// 模块：nav_ctrl —— 导航控制层（点位追踪 + 航向闭环 + L1/L2 寻点）
//
// 【依据】doc\导航控制方案.md（2026-10-19 定稿）+ 需求方 2026-10-19 追加：
//   ① "大转角要降级为间断" ② "路径规划模块会输出距离够长的路径点"
//   ③ 需求方 2026-10-19：「我想做 L1 和 L2」
//
// ==============================================================================
// 一、本模块在系统里的位置
// ==============================================================================
//   odom（航位） ──┐
//   nav_free_ctrl（点表 + 三态） ──┤──► 本模块 ──► chassis_ctrl.i_auto_{valid,center,diff}
//   rc（确认/返回/模式） ──┘                      （chassis_ctrl 内已含四路轮速 PID）
//
//   ★ **本模块不写速度环**。底盘 `chassis_ctrl` 里已有 4 路 `speed_pid`（100 Hz 增量式）。
//     再套一层速度环会让两个环共用同一个编码器和同一个 10 ms 滞后：外环带宽必须压到
//     内环 1/5 以下（约 2 Hz），对 300 mm/s 的巡航等于没有调节能力，反而引入低频振荡。
//     所以 `v_ref` 只做一次"mm/s → 计数/10ms"的乘法，直接送 `i_auto_center`。
//
// ==============================================================================
// 二、坐标系：只有两套，**没有世界系**
// ==============================================================================
//   ① **odom 相对系**（本模块点表所在系）
//        原点 = 上电后 odom 清零那一刻的车心位置
//        +x   = 那一刻的车头朝向
//        **永不重基准、不分段重基准**（决策 A2 + 需求方 2026-10-19 明确"不做重基准"）
//        它就是 `odom.o_x_mm / o_y_mm` 天然输出的那套坐标，没有任何额外的"全局/地图"层。
//   ② **车体系**（每 10 ms 由 odom 相对系旋转得到，只在拍内用，不存储）
//        ahead = 纵向（前正）、left = 横向（左正）
//
//   · 点表存 ①，因为点位置不随车转；若存 ②，车一转旧点全废。
//   · 每拍把目标从 ① 旋到 ② 再算 psi/距离，只用一次 cos/sin。
//   · 全程**没有**第二套绝对坐标，也不引入任何"地图原点""重基准"概念。
//
//   决策 A2：不用世界坐标，只用**相对初始化位置**的坐标 + 航向。目标点每拍重新投影：
//       dx = tgt_x − x_now,  dy = tgt_y − y_now          （① 里算）
//       ahead =  dx·cos(yaw) + dy·sin(yaw)     纵向投影（"还要往前开多远"）（转到 ②）
//       left  = −dx·sin(yaw) + dy·cos(yaw)     横向投影（左正，"目标在我左边多少"）
//   cos/sin 复用 odom 的 256 点 Q15 表（sin_lut_rom.v）：idx = yaw×7457>>20，
//   cos 用 idx+64（+90°），不新增 ROM。
//
//   ★ yaw 必须先**折返到 ±18000** 再查表。`odom.o_yaw_cont` 是连续航向，跑几圈会到
//     几十万；直接乘 7457 会溢出（1e6×7457 = 7.4e9 > 2^31）。本模块自维护 `yaw_wrap`：
//     每 10 ms 取 o_yaw_cont 的增量、折到 ±18000 后累加（与 odom 内部互为补充）。
//
// ==============================================================================
// 三、减速与到点判据用两个不同的量（最易做错处）
// ==============================================================================
//   · 减速看 ahead：目标在正侧方时 ahead≈0 → v_ref→0，车自然停下先转。
//     若用欧氏距离减速，目标在侧方时距离还很大，车会边前进边打舵，画大弧线。
//   · 判到点看 dist² = ahead² + left²：欧氏距离做判据有静态偏差缺陷 —— 几何上恒有
//     dist ≥ |left|，若横向稳态偏差 30 mm，则 dist 最小只能到 30 mm，判据 dist<25
//     **永不满足**，车在点附近反复找点、完成信号永不置位。
//   · v_ref = clamp(K_slow·ahead, 0, v_cruise)，K_slow = v_cruise/d_slow。
//     **硬约束** d_slow ≥ v_cruise²/(2·a_max) = 300²/1000 = 90 mm，现取 100 是 1.11 倍余量。
//     取小了减速不足会冲过头，且失效只在末段暴露，调参时极易误判成"死区太大"。
//
// ==============================================================================
// 四、L1 间断 / L2 非间断（需求方选定：两层都做）
// ==============================================================================
//   L1（i_mode=0，间断寻点）：
//       到点（dist² ≤ 25²）→ 停车 → 停稳 100 ms → 切下一点 → 原地对准 → 再走。
//       精度由 dist² ≤ R² 保证，**对任意转角都成立**（不需要转角门限）。
//
//   L2（i_mode=1，非间断寻点）：
//       到点**不停车**，但"不停车"必须**提前切点**，提前切点必然切内角。
//       设提前量 e、段间转角 θ、点距 L，实际轨迹到旧点的垂距：
//         h = e·L·sinθ / sqrt(e² + L² − 2eL·cosθ)  →（L≫e）e·sinθ
//       要求 h ≤ R = 25 mm；"不停车"又要求 e ≥ v²/(2a) = 90 mm。两式联立：
//         **θ_max = asin(25/90) = 16.1°**
//       所以本模块取 e = 90 mm、θ 门限 16°，两者是同一个约束的两面，不能单独改。
//       L2 启用条件（缺一降级为 L1）：
//         ① 段间转角 |θ| ≤ 16°
//         ② 下一段点距 ≥ 260 mm（d_slow=130 时即 2·d_slow；现 d_slow=100，仍取 260 更保守）
//         ③ 还有下一点
//
//   ★ 判"转角 ≤ 16°"不跑第二次 atan2。两段路径在**同一个 odom 相对系**里都有向量：
//         段1 = 车→当前点 = (dx_r, dy_r)
//         段2 = 当前点→下一点 = (sp_dx, sp_dy)
//         dot   = dx_r·sp_dx + dy_r·sp_dy
//         cross = dx_r·sp_dy − dy_r·sp_dx
//         tanθ = cross/dot，|θ| ≤ 16° ⟺ dot>0 且 |cross| ≤ dot·tan16°（tan16°=0.28674）
//     ★ 两段都在 odom 相对系 ⇒ 可直接点乘；**不要**拿车体系的 (ahead,left) 去和 odom 系的段2 点乘，
//       那是两个坐标系，θ 会算错。
//     ★ 两个量先 >>>10 缩到 19 位以内，避免 dot×294 溢出 32 位。
//
//   ★ `ctrl_done` 语义：只在**最后一点**完成时置位。中间点不报 ——
//     `rtl\ui\nav_free_ctrl.v:349-350` 收到它就把状态机打回 SETUP，
//     中间点报会把整趟任务提前终止。
//
// ==============================================================================
// 五、符号链（错一个符号即正反馈）
// ==============================================================================
//   `chassis_ctrl`：`tgt_left = center − diff`，`tgt_right = center + diff`
//     ⇒ diff > 0 → 左轮慢、右轮快 → ω = 2·diff/L > 0 → 车**左转**（CCW）
//       → yaw **增大**（本项目 yaw 以 CCW 为正，`YAW_SIGN=+1`）
//   `psi = atan2(left, ahead)`：正 = 目标在左 ⇒ 需要**左转** ⇒ diff 必须为**正**
//     ⇒ diff_cmd = +(kp·psi + ki·∫psi − kd·ω)  **不带负号**
//   ★ 2026-10-19 上板实测：最初误判成 "diff>0 是右转"、写了负号，结果航向环变成
//     **正反馈**（对准后越摆越大）。判据来自 `rtl\motor\chassis_ctrl.v:148-149`
//     的接线：tgt_left 减 diff、tgt_right 加 diff，右轮快即左转。
//   ω 取 (yaw 增量)×100（0.01°/s），**不要**用屏幕上的整数 °/s ——
//   那是显示用取整值，分辨率 1 °/s，在 3 °/s 的停稳阈值附近等于噪声。
//   ω 进 D 项前限幅 ±3000（30°/s）：原地全力转时 ω 可达 146°/s，不限幅 D 项会盖过 P 项。
//
// ==============================================================================
// 六、节拍与延迟
// ==============================================================================
//   `i_tick` = 10 ms（直接用 `delta_valid`，与 odom 测速窗口同拍）。
//   每次 tick 后跑约 20 拍：锁存→乘→投影→CORDIC(15 拍)→决策，约 400 ns，
//   占 10 ms 的 0.004%。其余时间输出保持不变（chassis_ctrl 每 10 ms 采一次）。
//
// 【定点约定】
//   位置 mm（32 位有符号）；航向 0.01°；角速度 0.01°/s；
//   速度指令 = 计数/10 ms（与 chassis_ctrl 的 center/diff 同单位，纯计数无小数）；
//   mm/s → 计数/10ms：×3046 >>10（DS_K_Q10=3525 的倒数，误差 0.0025%）。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module nav_ctrl #(
    // ---- 结构性参数（改这些仍要重综合；页面不开放）----
    parameter integer ARRIVE_R_MM      = 25,      // L1 到点半径 mm
    parameter integer NP_CUT_DIST_MM   = 90,      // L2 提前切点距离 e（= v²/2a）
    parameter integer NP_CUT_DEG       = 1600,    // L2 段间转角门限 16°（仅文档口径）
    parameter integer NP_MIN_SPACE_MM  = 260,     // L2 最小点距 mm（D_SLOW=130 时为 2·D_SLOW；现 D_SLOW=100，仍取 260 更保守）
    parameter integer STOP_V_EPS       = 50,      // 停稳判据 mm/s
    parameter integer STOP_W_EPS       = 300,     // 停稳判据 0.01°/s（=3°/s）
    parameter integer STOP_SETTLE_TICKS= 10,      // 停稳保持节拍数（×10 ms = 100 ms）
    parameter integer TIMEOUT_TICKS    = 3000,    // 单点超时节拍数（×10 ms = 30 s）
    parameter integer MAX_CENTER       = 3072,    // center 限幅（计数/10ms，=1033 mm/s）
    parameter integer MAX_DIFF         = 612,     // 原地转向差速限幅（≈146 °/s）
    parameter integer OM_CLIP          = 3000     // ω 进 D 项的限幅（0.01°/s）
)(
    input  wire                clk,
    input  wire                rst_n,

    // ==================================================================
    // 运行时参数（2026-10-20 从 module parameter 改为端口）
    //   由 `rtl\ui\tune_ctrl.v` 的参数整定页在线修改，改完下一个 10 ms 节拍
    //   就生效（不需要重综合/烧录）。
    //   ★ 默认值与取值范围（min/max/step）的**唯一真源**是
    //     `rtl\ui\tune_ctrl.v` 的 p_def() / p_min() / p_max() / p_step()，
    //     本模块不再保留副本，避免两处漂移。
    //   ★ 注意时序：KP/KI/KD 原来是常量（综合成移位加），现在变成真乘法器，
    //     见下方 PID 段的说明。
    // ==================================================================
    input  wire signed [31:0]  i_v_cruise,       // 300    mm/s  巡航线速度
    input  wire signed [31:0]  i_d_slow,         // 100    mm    开始减速的纵向距离（硬约束 ≥ v²/2a）
    input  wire signed [31:0]  i_align_in,       // 800    0.01° ALIGN → TRACK 门限（8°）
    input  wire signed [31:0]  i_align_out,      // 3500   0.01° TRACK → ALIGN 门限（35°）
    input  wire signed [31:0]  i_max_diff_cruise,// 126    cnt/10ms 行进中差速限幅（≈30 °/s）
    input  wire signed [31:0]  i_kp_yaw,         // 40     Q8    航向 PID 比例
    input  wire signed [31:0]  i_ki_yaw,         // 3      Q8    航向 PID 积分
    input  wire signed [31:0]  i_kd_yaw,         // 6      Q8    航向 PID 微分
    input  wire signed [31:0]  i_i_limit,        // 120    cnt/10ms 积分项限幅
                                                 //   ★ 2026-10-20 上板后新增：原为 module
                                                 //     parameter `I_LIMIT = 400`，而行进中
                                                 //     差速上限 `MAX_DIFF_CRUISE = 180` ——
                                                 //     积分单独就能涨到 400（2.2 倍），
                                                 //     输出被 clamp 成恒定 ±180，PID 退化成
                                                 //     bang-bang，表现为"走直线左右摆头"。
                                                 //     需求方实测「航向积分清零后能走直」，
                                                 //     确认根因即此，故把限幅开放为运行时参数。
    input  wire signed [31:0]  i_k_slow_q10,     // 3072   派生量 (V_CRUISE×1024)/D_SLOW，
                                                 //        由 tune_ctrl 的迭代除法器算好后送进来
                                                 //        （放到本模块会让 nav_ctrl 里出现真除法）

    // ---- 来自 UI 层（nav_free_ctrl）----
    input  wire                i_active,      // 导航页交互态（0 = 已退出，立即停）
    input  wire                i_run,         // o_nav_run（1 = 导航中）
    input  wire                i_paused,      // o_nav_paused（1 = 暂停，冻结输出）
    input  wire                i_mode,        // 0 = 间断寻点(L1) / 1 = 非间断寻点(L2)
    input  wire                i_is_obs,      // 1 = 避障页（目标取准心，无点表）
    input  wire signed [31:0]  i_cur_x_mm,    // 准心（避障页当目标）
    input  wire signed [31:0]  i_cur_y_mm,

    // ---- 点表（来自 nav_free_ctrl，毫米坐标，16 点）----
    input  wire [255:0]        i_pt_x_mm,     // 每点 16 位有符号
    input  wire [255:0]        i_pt_y_mm,
    input  wire [4:0]          i_pt_count,

    // ---- 来自 odom ----
    input  wire                i_tick,        // 10 ms（= delta_valid）
    input  wire signed [31:0]  i_x_mm,
    input  wire signed [31:0]  i_y_mm,
    input  wire signed [31:0]  i_yaw_centi,   // 0.01°，连续（不折返）
    input  wire signed [31:0]  i_v_mms,
    input  wire [3:0]          i_stall,       // 堵转

    // ---- 到底盘 ----
    output reg                 o_auto_valid,
    output reg  signed [31:0]  o_center,
    output reg  signed [31:0]  o_diff,

    // ---- 回告 UI ----
    output reg                 o_done,        // 整趟完成（单拍）
    output reg                 o_fault,       // 未到达而终止（单拍）

    // ---- 观测 / 调试 ----
    output reg  [2:0]          o_state,
    output reg  signed [31:0]  o_dbg_ahead,   // mm
    output reg  signed [31:0]  o_dbg_left,    // mm
    output reg  signed [15:0]  o_dbg_psi,     // 0.01°
    output reg  [4:0]          o_dbg_idx
);

    // ==================================================================
    // 0. 常量
    // ==================================================================
    // mm/s → 计数/10ms：1 count/10ms = 0.3361701 mm/s（DS_K_Q10=3525）
    //   倒数 2.974684 → Q10 = 3046（误差 0.0025%）
    localparam signed [31:0] MMPS_TO_CNT_Q10 = 32'sd3046;
    // ★ 2026-10-20：这两项原来是 localparam，参数运行时可调后必须改派生方式
    //   K_SLOW 由 tune_ctrl 的迭代除法器算好送进来（nav_ctrl 内不出现除法器）；
    //   v_cruise_q10mm 是寄存器左移 10 位，综合成连线、不占乘法器。
    wire signed [31:0] k_slow_q10     = i_k_slow_q10;
    wire signed [31:0] v_cruise_q10mm = i_v_cruise <<< 10;
    localparam signed [31:0] R2_IN           = ARRIVE_R_MM    * ARRIVE_R_MM;
    localparam signed [31:0] E2_CUT          = NP_CUT_DIST_MM * NP_CUT_DIST_MM;
    localparam signed [31:0] SPACE2_MIN      = NP_MIN_SPACE_MM * NP_MIN_SPACE_MM;
    // tan(16°) = 0.286745 → Q10 = 294（与 NP_CUT_DEG 是同一约束的两面，见头部第四节）
    localparam signed [31:0] TAN_CUT_Q10     = 32'sd294;

    // 状态
    localparam [2:0] S_IDLE  = 3'd0,
                     S_ALIGN = 3'd1,
                     S_TRACK = 3'd2,
                     S_HOLD  = 3'd3,
                     S_DONE  = 3'd4,
                     S_FAULT = 3'd5;

    // 测量序列（tick 触发）
    //   ★ 2026-10-19 时序：把「折返航向 → ×7457 → 查表 → 投影乘法」这条长链拆成 4 拍。
    //     原写法 22:43 综合报 `u_odom/o_yaw_cont_3_s3/Q → u_nav_ctrl/mult_166_s1/B[21]`
    //     slack **−1.067 ns**（delay 17.903），是「减法 + 折返比较 + 加 + 再折返 + 乘 7457」
    //     全串在一条路径上。现拆为 M_Y1(加) / M_Y2(折返) / M_Y3(乘) / M_A1(投影)。
    //     ★ i_tick 是 1 kHz（10 ms = 500000 拍），整个序列拉长到 8 拍毫无代价。
    localparam [3:0] M_IDLE = 4'd0,
                     M_Y1   = 4'd1,   // 折返航向：yaw_wrap + dyaw_lat
                     M_Y2   = 4'd2,   // 折回 ±18000
                     M_Y3   = 4'd3,   // ×7457 得到查表索引 + ω
                     M_A1   = 4'd4,   // 4 个投影乘法
                     M_A2   = 4'd5,   // ahead/left → 启动 CORDIC
                     M_V1   = 4'd6,   // 速度规划 1/3：ahead_mm + ×K_SLOW
                     M_V2   = 4'd7,   // 速度规划 2/3：v_ref 选择
                     M_V3   = 4'd8,   // 速度规划 3/3：×3046 → counts/10ms
                     M_A3   = 4'd9,   // 等 CORDIC（15 拍）→ 锁 psi_r / i_lim_r
                     M_A4   = 4'd10,  // PID 三项求和 → diff_pid_r / i_acc
                     M_A5   = 4'd11;  // 用寄存器值决策 + 出指令

    // ==================================================================
    // 1. sin / cos 查找表（复用 odom 的 ROM，不新增资源）
    // ==================================================================
    reg  signed [31:0] yaw_wrap;       // ±18000 折返航向
    reg  signed [31:0] yaw_prev;       // 上一拍 i_yaw_centi
    reg  signed [31:0] dyaw_lat;       // 角增量锁存（om_raw 拆拍用，见 M_IDLE）
    wire signed [31:0] dyaw_raw = i_yaw_centi - yaw_prev;
    wire signed [31:0] dyaw_win = (dyaw_raw > 32'sd18000)  ? (dyaw_raw - 32'sd36000) :
                                  (dyaw_raw < -32'sd18000) ? (dyaw_raw + 32'sd36000) :
                                                             dyaw_raw;

    // 表索引：i = yaw_0.01° × 256/36000 = yaw × 7457 >> 20（同 odom.v:216）
    //   已在 M_Y3 拍寄存（见上面测量序列的时序说明），消费点 M_A1 隔了至少一拍。
    reg  signed [31:0] yaw_sum_r;      // yaw_wrap + dyaw_lat（M_Y1）
    reg  signed [31:0] idx_w_r;        // 查表索引（M_Y3）
    wire        [7:0]  idx_sin = idx_w_r[7:0];
    wire        [7:0]  idx_cos = idx_sin + 8'd64;   // +90° = +64 点

    wire signed [15:0] sin_v, cos_v;
    sin_lut_rom u_sin (.i_idx(idx_sin), .o_val(sin_v));
    sin_lut_rom u_cos (.i_idx(idx_cos), .o_val(cos_v));

    // ==================================================================
    // 2. 目标选择：避障页取准心，非避障页取点表第 idx 点
    // ==================================================================
    reg  [4:0] pt_idx;
    reg  [4:0] pt_cnt;

    wire signed [15:0] pt_x_sel = i_pt_x_mm[{pt_idx, 4'b0000} +: 16];
    wire signed [15:0] pt_y_sel = i_pt_y_mm[{pt_idx, 4'b0000} +: 16];

    wire signed [31:0] tgt_x = i_is_obs ? i_cur_x_mm : {{16{pt_x_sel[15]}}, pt_x_sel};
    wire signed [31:0] tgt_y = i_is_obs ? i_cur_y_mm : {{16{pt_y_sel[15]}}, pt_y_sel};

    // 下一点（L2 判据用）
    wire [4:0]         nxt_idx  = (pt_idx + 5'd1 >= pt_cnt) ? pt_idx : (pt_idx + 5'd1);
    wire signed [15:0] pt_x_nxt = i_pt_x_mm[{nxt_idx, 4'b0000} +: 16];
    wire signed [15:0] pt_y_nxt = i_pt_y_mm[{nxt_idx, 4'b0000} +: 16];
    wire signed [31:0] tgt_x_n  = {{16{pt_x_nxt[15]}}, pt_x_nxt};
    wire signed [31:0] tgt_y_n  = {{16{pt_y_nxt[15]}}, pt_y_nxt};

    wire has_next = (pt_idx + 5'd1 < pt_cnt);

    // ==================================================================
    // 3. 测量流水线寄存器
    // ==================================================================
    reg  [3:0]         mst;
    reg  signed [31:0] dx_r, dy_r;
    reg  signed [31:0] sp_dx, sp_dy;                 // 段2 odom 相对系：当前点→下一点
    reg  signed [31:0] p_dcs, p_dss, p_dsc, p_dcc;   // dx·cos, dy·sin, dx·sin, dy·cos
    reg  signed [31:0] ahead_r, left_r;              // Q8
    reg  signed [31:0] om_raw;                       // ω 原始（0.01°/s）
    reg  signed [31:0] dot_r, cross_r;               // 段间夹角判据

    // CORDIC
    reg                cd_start;
    wire signed [15:0] cd_angle;
    wire               cd_valid, cd_zero;

    cordic_atan2 u_cordic (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_start(cd_start),
        .i_x    (ahead_r),          // 已是 Q8
        .i_y    (left_r),
        .o_angle(cd_angle),
        .o_valid(cd_valid),
        .o_zero (cd_zero)
    );

    // PID 状态
    reg signed [31:0] i_acc;
    // 停稳计时 / 超时
    reg [15:0] settle_cnt;
    reg [15:0] elapsed_cnt;

    // ------------------------------------------------------------------
    // 3a. 组合计算
    // ------------------------------------------------------------------
    wire signed [31:0] m_dcs = dx_r * cos_v;
    wire signed [31:0] m_dss = dy_r * sin_v;
    wire signed [31:0] m_dsc = dx_r * sin_v;
    wire signed [31:0] m_dcc = dy_r * cos_v;

    // Q8 版本（供 CORDIC）：
    //   ahead_mm = (p_dcs + p_dss) >>> 15;  ahead_q8 = ahead_mm << 8 = (···) >>> 7
    wire signed [31:0] ahead_q8 = (p_dcs + p_dss) >>> 7;
    wire signed [31:0] left_q8  = (p_dcc - p_dsc) >>> 7;
    wire signed [31:0] ahead_mm = ahead_q8 >>> 8;
    wire signed [31:0] left_mm  = left_q8  >>> 8;

    // 段间转角判据（两段都在 odom 相对系，见头部第四节）
    wire signed [31:0] s_dot   = (dx_r * sp_dx) + (dy_r * sp_dy);
    wire signed [31:0] s_cross = (dx_r * sp_dy) - (dy_r * sp_dx);
    wire signed [31:0] dot_s   = dot_r >>> 10;
    wire signed [31:0] cross_s = (cross_r < 0) ? ((-cross_r) >>> 10) : (cross_r >>> 10);
    wire signed [31:0] cut_limit = (dot_s * TAN_CUT_Q10) >>> 10;
    wire               space_ok = ((sp_dx * sp_dx) + (sp_dy * sp_dy)) >= SPACE2_MIN;
    // M_A2 锁存阈值，M_V1 并行比较四个字节，M_V2 合并结果。
    // 最高字节按有符号比较，其余按无符号比较，保持完整 32 位判据。
    reg signed [31:0] cut_limit_r;
    reg               cut_forward_r;
    reg [2:0]         cut_lt_r, cut_eq_r;
    reg               cut_le_low_r;
    reg               cut_ok_r;
    reg                space_ok_r;

    // 距离判据
    wire signed [31:0] dist2 = (ahead_mm * ahead_mm) + (left_mm * left_mm);

    // ★ 2026-10-19 晚 时序：`psi` / `i_lim` 都在 M_A3（CORDIC 刚给 valid）锁存。
    //   原因：`i_acc <= i_lim` 是**自反馈**（i_acc → i_sum → i_lim → i_acc），
    //   一条 32 位加法 + 32 位 clamp 必须在一拍内走完（约 21 ns，slack −2.598）。
    //   拆法：M_A3 只锁 `psi_r`/`i_lim_r`，M_A4 只做 PID 求和 + 回存积分，
    //   M_A5 才决策出指令 —— 每拍只走一级运算。
    reg  signed [15:0] psi_r;          // M_A3：CORDIC 角度
    reg  signed [31:0] i_lim_r;        // M_A3：限幅后的积分新值

    // 方位角绝对值（ALIGN 判据）—— 用寄存后的 psi_r（决策在 M_A5）
    wire signed [31:0] psi_abs = (psi_r < 16'sd0) ? -psi_r : psi_r;

    // 速度规划：v_ref（Q10 mm/s）→ 计数/10ms
    //   ★ 实际的 v_ref/v_cnt 计算已按拍拆到 M_V1/M_V2/M_V3（见下方寄存器声明）。

    // ★ 2026-10-19 时序：`o_center` 原来直接吃整条组合链
    //   `p_dcs(Q) → 加 → 移位 → ×K_SLOW → 多级 mux → ×3046 → 移位 → clamp → mux → o_center.D`，
    //   22:52 综合报 **10 条 −0.789 ~ −0.840**（delay 20.7 ns，`n965_s/DOUT[7] → o_center_*_s0/D`）。
    //   拆成 M_V1/M_V2/M_V3 三拍，每拍只走一个乘法；M_A4 决策时直接用 `v_cnt_r`。
    reg  signed [31:0] ahead_mm_r;     // M_V1
    reg  signed [31:0] v_slow_r;       // M_V1：ahead_mm_r × k_slow_q10
    reg  signed [31:0] v_ref_r;        // M_V2：限到 v_cruise_q10mm
    reg  signed [31:0] v_cnt_r;        // M_V3：counts/10ms
    wire signed [31:0] v_ref_final = (v_ref_r > v_cruise_q10mm) ? v_cruise_q10mm : v_ref_r;

    // ★ 2026-10-20：k_slow_q10 从常量变寄存器后，乘法器显式收到 16 位
    //   （ahead ≤ ±1000 mm、K_SLOW ≤ 12288，两边都在 16 位内）。
    //   `ahead_r[23:8]` 与 `ahead_r >>> 8` 的低 16 位完全等价（算术右移不动低位）。
    wire signed [15:0] ahead_mm16 = ahead_r[23:8];
    wire signed [15:0] ks16       = k_slow_q10[15:0];

    // 航向 PID
    //   `psi_ext`（当前拍 cd_angle）只给 M_A3 算 `i_inc` 用；
    //   `psi_ext_r`（寄存的 psi_r）给 M_A4 算 `p_term` 用。两者不能混：
    //   混用会让 p_term 与 i_lim 落在不同拍，积分与比例项差一个 10 ms 窗口。
    wire signed [31:0] psi_ext   = {{16{cd_angle[15]}}, cd_angle};
    wire signed [31:0] psi_ext_r = {{16{psi_r[15]}},    psi_r};
    wire signed [31:0] om_lim  = (om_raw >  OM_CLIP) ?  OM_CLIP :
                                 (om_raw < -OM_CLIP) ? -OM_CLIP : om_raw;
    //   ★ 2026-10-20：KP/KI/KD 从 module parameter 变成寄存器端口后，乘法器
    //     必须**显式收到 16 位**，否则综合器会按 32×32 生成（4 个 DSP、延迟更长）。
    //     psi ≤ ±18000（0.01°）、ω ≤ ±3000、PID 系数 ≤ 200，两边都在 16 位内，截断无损。
    //     —— 这也是把这项改动限制在「换数据源、不换结构」的关键：乘法器从 0 个
    //     （常量时综合成移位加）变成 3 个 DSP，但位宽没有膨胀。
    wire signed [15:0] psi_r16 = psi_ext_r[15:0];
    wire signed [15:0] psi_c16 = psi_ext[15:0];
    wire signed [15:0] om16    = om_lim[15:0];
    wire signed [15:0] kp16    = i_kp_yaw[15:0];
    wire signed [15:0] ki16    = i_ki_yaw[15:0];
    wire signed [15:0] kd16    = i_kd_yaw[15:0];
    wire signed [31:0] p_term  = (psi_r16 * kp16) >>> 8;
    wire signed [31:0] i_inc   = (psi_c16 * ki16) >>> 8;
    wire signed [31:0] d_term  = (om16    * kd16) >>> 8;
    wire signed [31:0] i_sum   = i_acc + i_inc;
    //   ★ 2026-10-20 时序：`i_i_limit` 从 `tune_ctrl` 的参数数组经 `o_param` 打包直连进来，
    //     扇出 6 个 32 位比较器（`i_sum` 的 6 个高位）。20:05 那版报 6 条同域 setup 违例
    //     （−1.271 ~ −1.424，路径 `u_tune_ctrl/p[8]_s0 → u_nav_ctrl/i_lim_r_*`）。
    //     这里把参数先打一拍，切断 "参数寄存器 → 比较器 → i_lim_r" 的长组合链。
    //     代价：调参后积分上限晚 10 ms 生效（人工手调感知不到）。
    (* syn_keep = 1 *) reg  signed [31:0] i_limit_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) i_limit_r <= 32'sd120;
        else        i_limit_r <= i_i_limit;
    end
    //   第二级：`i_sum`（= i_acc + i_inc，i_inc 是乘法器输出）也先落一拍，
    //   让比较器前只剩"寄存器 → 比较 → mux → 寄存器"，不再叠加法器。
    //   多出的 1 拍延迟不影响功能：i_acc 每 10 ms 才回存一次。
    reg  signed [31:0] i_sum_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) i_sum_r <= 32'sd0;
        else        i_sum_r <= i_acc + i_inc;
    end
    wire signed [31:0] i_lim   = (i_sum_r >  i_limit_r) ?  i_limit_r :
                                 (i_sum_r < -i_limit_r) ? -i_limit_r : i_sum_r;
    wire signed [31:0] u_pid   = p_term + i_lim_r - d_term;
    // ★ 不带负号：psi>0（目标在左）⇒ 需左转 ⇒ diff 必须为**正**
    //   （`chassis_ctrl.v:148-149`：tgt_left=center−diff、tgt_right=center+diff，
    //     diff>0 ⇒ 右轮快 ⇒ 左转 ⇒ yaw 增大）
    //   见文件头「五、符号链」—— 2026-10-19 上板实测原来写反，导致正反馈。

    // ★ 2026-10-19 晚 时序：`u_pid` 在 M_A4 锁存、M_A5 才消费。
    //   M_A4 那一拍只做"p_term + i_lim_r − d_term"（两个加法器，输入全是寄存器），
    //   `o_diff` 前面只剩 `reg → clamp → mux`。
    //   历史：22:30 报 −0.831/−0.673（当时链上还串着乘法器，且 i_lim 未打拍）；
    //         23:42 报 −2.598（`i_acc` 自反馈链把 i_lim 拖进同一拍）。
    reg  signed [31:0] diff_pid_r;

    function signed [31:0] clamp32;
        input signed [31:0] v;
        input signed [31:0] lim;
        begin
            clamp32 = (v > lim) ? lim : ((v < -lim) ? -lim : v);
        end
    endfunction

    // ==================================================================
    // 4. 主状态机
    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mst          <= M_IDLE;
            pt_idx       <= 5'd0;
            pt_cnt       <= 5'd0;
            yaw_wrap     <= 32'sd0;
            yaw_prev     <= 32'sd0;
            dx_r         <= 32'sd0;
            dy_r         <= 32'sd0;
            sp_dx        <= 32'sd0;
            sp_dy        <= 32'sd0;
            p_dcs        <= 32'sd0;
            p_dss        <= 32'sd0;
            p_dsc        <= 32'sd0;
            p_dcc        <= 32'sd0;
            ahead_r      <= 32'sd0;
            left_r       <= 32'sd0;
            om_raw       <= 32'sd0;
            dyaw_lat     <= 32'sd0;
            yaw_sum_r    <= 32'sd0;
            idx_w_r      <= 32'sd0;
            ahead_mm_r   <= 32'sd0;
            v_slow_r     <= 32'sd0;
            v_ref_r      <= 32'sd0;
            v_cnt_r      <= 32'sd0;
            diff_pid_r   <= 32'sd0;
            dot_r        <= 32'sd0;
            cross_r      <= 32'sd0;
            cut_limit_r   <= 32'sd0;
            cut_forward_r <= 1'b0;
            cut_lt_r      <= 3'd0;
            cut_eq_r      <= 3'd0;
            cut_le_low_r  <= 1'b0;
            cut_ok_r      <= 1'b0;
            space_ok_r   <= 1'b0;
            psi_r        <= 16'sd0;
            i_lim_r      <= 32'sd0;
            cd_start     <= 1'b0;
            i_acc        <= 32'sd0;
            settle_cnt   <= 16'd0;
            elapsed_cnt  <= 16'd0;
            o_auto_valid <= 1'b0;
            o_center     <= 32'sd0;
            o_diff       <= 32'sd0;
            o_done       <= 1'b0;
            o_fault      <= 1'b0;
            o_state      <= S_IDLE;
            o_dbg_ahead  <= 32'sd0;
            o_dbg_left   <= 32'sd0;
            o_dbg_psi    <= 16'sd0;
            o_dbg_idx    <= 5'd0;
        end else begin
            cd_start <= 1'b0;
            o_done   <= 1'b0;
            o_fault  <= 1'b0;

            // ---------------- 使能 / 暂停 / 退出：最高优先级 ----------------
            // ★ 顺序要点：UI 侧 `o_nav_run = (state==RUNNING)`、`o_nav_paused = (state==PAUSED)`，
            //   所以**暂停时 `i_run` 也是 0**。若先判 `!i_run` 就永远进不到暂停分支，
            //   车会走 "valid=0 → 断驱动 → 靠惯性滑行"。所以退出条件必须写成
            //   `(!i_run && !i_paused)`，把"暂停"从"退出"里摘出来。
            if (!i_active || (!i_run && !i_paused)) begin
                o_auto_valid <= 1'b0;
                o_center     <= 32'sd0;
                o_diff       <= 32'sd0;
                o_state      <= S_IDLE;
                mst          <= M_IDLE;
                pt_idx       <= 5'd0;
                i_acc        <= 32'sd0;
                settle_cnt   <= 16'd0;
                elapsed_cnt  <= 16'd0;

            end else if (i_paused) begin
                // ★ 暂停：**主动停住**，不是"断驱动滑行"。
                //   `o_auto_valid` 故意保持 1、目标给 0：若置 invalid，chassis_ctrl 会
                //   直接输出 0 占空比（= 断驱动，靠惯性滑行）；保持 valid + 目标 0，
                //   四路 speed_pid 会主动把轮速拉向 0（= 主动制动）。
                //   需求方 2026-10-19：「确保任务暂停时车能停下来」。
                //   ★ 不销毁状态机：`o_state` 保持不变，恢复后从原状态继续；
                //     `mst` 回 M_IDLE 让下一个 tick 重新测一遍（暂停期间不计时、不累积分）。
                o_auto_valid <= 1'b1;
                o_center     <= 32'sd0;
                o_diff       <= 32'sd0;
                mst          <= M_IDLE;

            end else begin

                // ---------- 运行起动：锁存点数、从第一点开始对准 ----------
                if (o_state == S_IDLE) begin
                    pt_cnt  <= i_is_obs ? 5'd1 : ((i_pt_count == 5'd0) ? 5'd1 : i_pt_count);
                    pt_idx  <= 5'd0;
                    i_acc   <= 32'sd0;
                    o_state <= S_ALIGN;
                end

                // ---------- 测量序列 ----------
                case (mst)
                    M_IDLE: begin
                        if (i_tick) begin
                            dx_r     <= tgt_x - i_x_mm;
                            dy_r     <= tgt_y - i_y_mm;
                            sp_dx    <= tgt_x_n - tgt_x;
                            sp_dy    <= tgt_y_n - tgt_y;
                            // ★ 时序：dyaw_win 是 i_yaw_centi−yaw_prev 的组合折返。
                            //   原写法把「减法 + 折返比较 + 加 + 再折返 + 乘 7457」全串在
                            //   一条 17.9 ns 路径上（22:43 综合 −1.067）。这里只锁存差值，
                            //   折返/乘索引挪到 M_Y1~M_Y3 三拍做。
                            dyaw_lat <= dyaw_win;
                            yaw_prev <= i_yaw_centi;
                            mst      <= M_Y1;
                        end
                    end
                    // ---- 折返航向流水：加 → 折返 → 乘索引（见测量序列的时序说明）----
                    M_Y1: begin
                        yaw_sum_r <= yaw_wrap + dyaw_lat;
                        mst       <= M_Y2;
                    end
                    M_Y2: begin
                        yaw_wrap <= (yaw_sum_r > 32'sd18000)  ? (yaw_sum_r - 32'sd36000) :
                                    (yaw_sum_r < -32'sd18000) ? (yaw_sum_r + 32'sd36000) :
                                                                yaw_sum_r;
                        mst      <= M_Y3;
                    end
                    M_Y3: begin
                        idx_w_r <= (yaw_wrap * 32'sd7457) >>> 20;
                        om_raw  <= dyaw_lat * 32'sd100;   // 0.01°/s
                        mst     <= M_A1;
                    end
                    M_A1: begin
                        p_dcs  <= m_dcs;
                        p_dss  <= m_dss;
                        p_dsc  <= m_dsc;
                        p_dcc  <= m_dcc;
                        dot_r  <= s_dot;
                        cross_r<= s_cross;
                        mst    <= M_A2;
                    end
                    M_A2: begin
                        ahead_r       <= ahead_q8;
                        left_r        <= left_q8;
                        cut_limit_r   <= cut_limit;
                        cut_forward_r <= (dot_s > 32'sd0);
                        mst           <= M_V1;
                    end
                    // ---- 速度规划三级流水（见 v_cnt 附近的时序说明）----
                    M_V1: begin
                        cut_lt_r[2]  <= $signed(cross_s[31:24]) < $signed(cut_limit_r[31:24]);
                        cut_lt_r[1]  <= cross_s[23:16] < cut_limit_r[23:16];
                        cut_lt_r[0]  <= cross_s[15:8]  < cut_limit_r[15:8];
                        cut_eq_r[2]  <= cross_s[31:24] == cut_limit_r[31:24];
                        cut_eq_r[1]  <= cross_s[23:16] == cut_limit_r[23:16];
                        cut_eq_r[0]  <= cross_s[15:8]  == cut_limit_r[15:8];
                        cut_le_low_r <= cross_s[7:0] <= cut_limit_r[7:0];
                        ahead_mm_r <= ahead_r >>> 8;
                        v_slow_r   <= ahead_mm16 * ks16;
                        space_ok_r <= space_ok;  // ★ 同上，与上面两个乘法并行
                        mst        <= M_V2;
                    end
                    M_V2: begin
                        cut_ok_r <= cut_forward_r &&
                                    (cut_lt_r[2] ||
                                     (cut_eq_r[2] && cut_lt_r[1]) ||
                                     (cut_eq_r[2] && cut_eq_r[1] && cut_lt_r[0]) ||
                                     ((&cut_eq_r) && cut_le_low_r));
                        v_ref_r <= (ahead_mm_r >= i_d_slow) ? v_cruise_q10mm :
                                   (ahead_mm_r > 32'sd0)     ? v_slow_r : 32'sd0;
                        mst     <= M_V3;
                    end
                    M_V3: begin
                        v_cnt_r  <= (v_ref_final * MMPS_TO_CNT_Q10) >>> 20;
                        cd_start <= 1'b1;        // 下一拍（M_A3）CORDIC 采样
                        mst      <= M_A3;
                    end
                    M_A3: begin
                        // cd_start 在 M_A3 当拍为 1 → CORDIC 正在采样；等它给 valid
                        // ★ 这一拍只锁两个值：psi 与限幅后的积分新值（见声明处说明）
                        if (cd_valid) begin
                            psi_r   <= cd_angle;
                            i_lim_r <= i_lim;
                            mst     <= M_A4;
                        end
                    end
                    M_A4: begin
                        // 一拍只做 PID 三项求和 + 积分回存，全部落到寄存器
                        if ((o_state == S_ALIGN) || (o_state == S_TRACK)) i_acc <= i_lim_r;
                        diff_pid_r <= u_pid;      // u_pid = p_term + i_lim_r − d_term
                        mst        <= M_A5;
                    end
                    M_A5: begin
                        mst <= M_IDLE;
                    end
                    default: mst <= M_IDLE;
                endcase

                // ---------- 非决策拍：保持使能 ----------
                if (mst != M_A5) begin
                    o_auto_valid <= (o_state == S_ALIGN) || (o_state == S_TRACK) ||
                                    (o_state == S_HOLD);
                end

                // ---------- 决策（只在 M_A5 拍）----------
                //   ★ 用寄存后的 psi_r / diff_pid_r / cut_ok_r / space_ok_r，
                //     这一拍只剩"比较 + 选择"，不再串乘法器与加法器。
                if (mst == M_A5) begin
                    o_dbg_ahead <= ahead_mm;
                    o_dbg_left  <= left_mm;
                    o_dbg_psi   <= psi_r;
                    o_dbg_idx   <= pt_idx;

                    // ---- 航向积分累加（每 tick 一次）----
                    //   ★ 原来 `i_acc` 只有 5 处清零、**没有回存**，`i_sum = i_acc + i_inc`
                    //     实际恒等于 `i_inc`，等于 KI 项退化成第二个比例项、
                    //     积分永远不累积 ⇒ 横向稳态误差 `e_ss = 扰动/Kp` 消不掉。
                    //   ★ 自 2026-10-19 晚起，回存移到 M_A4 拍（`i_acc <= i_lim_r`），
                    //     因为 `i_acc → i_sum → i_lim → i_acc` 的自反馈链一拍走不完（21 ns）。
                    //     下面的 `case` 分支里仍有 5 处 `i_acc <= 0`（模式切换清积分），
                    //     它们晚一拍生效，会覆盖 M_A4 的回存 —— 语义不变。

                    case (o_state)
                        // ==================================================
                        S_ALIGN: begin
                            // 原地转向目标方位：center=0、diff 全力
                            o_center     <= 32'sd0;
                            o_diff       <= clamp32(diff_pid_r, MAX_DIFF);
                            o_auto_valid <= 1'b1;
                            // 对准（30° 以内）→ TRACK
                            if (psi_abs <= i_align_in) begin
                                o_state <= S_TRACK;
                                i_acc   <= 32'sd0;      // 模式切换必须清积分
                            end
                        end
                        // ==================================================
                        S_TRACK: begin
                            // ---- L2：提前切点（e = 90 mm），不减速 ----
                            if (i_mode && has_next && cut_ok_r && space_ok_r &&
                                (dist2 <= E2_CUT)) begin
                                pt_idx     <= pt_idx + 5'd1;
                                elapsed_cnt<= 16'd0;
                                i_acc      <= 32'sd0;   // 目标跳变 → 清积分
                                o_center   <= v_cnt_r;   // 已由 M_V3 限到 V_CRUISE（892 counts < MAX_CENTER），无需再 clamp
                                o_diff     <= clamp32(diff_pid_r, i_max_diff_cruise);
                                o_auto_valid <= 1'b1;
                            end
                            // ---- L1：到点（25 mm）→ 停车 ----
                            else if (dist2 <= R2_IN) begin
                                o_state      <= S_HOLD;
                                settle_cnt   <= 16'd0;
                                o_center     <= 32'sd0;
                                o_diff       <= 32'sd0;
                                o_auto_valid <= 1'b1;
                            end
                            // ---- 正常行进 ----
                            else begin
                                if (psi_abs > i_align_out) begin
                                    // 偏太多：回 ALIGN 重新对准。
                                    // ★ 这一拍就**不许再给前进速度**，否则"该转却在冲"
                                    //   会多走 10 ms 的冤枉路（300 mm/s 下 3 mm）。
                                    o_state      <= S_ALIGN;
                                    i_acc        <= 32'sd0;
                                    o_center     <= 32'sd0;
                                    o_diff       <= clamp32(diff_pid_r, MAX_DIFF);
                                end else begin
                                    o_center     <= v_cnt_r;   // 已限到 V_CRUISE
                                    o_diff       <= clamp32(diff_pid_r, i_max_diff_cruise);
                                end
                                o_auto_valid <= 1'b1;
                            end
                        end
                        // ==================================================
                        S_HOLD: begin
                            // 停车 + 保持（diff = 0：差速车不打舵即保持航向）
                            o_center     <= 32'sd0;
                            o_diff       <= 32'sd0;
                            o_auto_valid <= 1'b1;
                            // 停稳：|v| ≤ 50 mm/s 且 |ω| ≤ 3°/s，保持 STOP_SETTLE_TICKS
                            if ((i_v_mms <= STOP_V_EPS) && (i_v_mms >= -STOP_V_EPS) &&
                                (om_raw <= STOP_W_EPS) && (om_raw >= -STOP_W_EPS)) begin
                                if (settle_cnt >= STOP_SETTLE_TICKS) begin
                                    settle_cnt <= 16'd0;
                                    if (has_next) begin
                                        pt_idx  <= pt_idx + 5'd1;
                                        i_acc   <= 32'sd0;
                                        o_state <= S_ALIGN;   // L1：切下一点后重新对准
                                    end else begin
                                        o_done       <= 1'b1;  // ★ 只在最后一点报
                                        o_state      <= S_DONE;
                                        o_center     <= 32'sd0;
                                        o_diff       <= 32'sd0;
                                        o_auto_valid <= 1'b0;
                                    end
                                end else begin
                                    settle_cnt <= settle_cnt + 16'd1;
                                end
                            end else begin
                                settle_cnt <= 16'd0;
                            end
                        end
                        // ==================================================
                        S_DONE: begin
                            o_auto_valid <= 1'b0;
                            o_center     <= 32'sd0;
                            o_diff       <= 32'sd0;
                        end
                        S_FAULT: begin
                            o_auto_valid <= 1'b0;
                            o_center     <= 32'sd0;
                            o_diff       <= 32'sd0;
                        end
                        default: o_state <= S_IDLE;
                    endcase

                    // ---------- 超时 / 堵转保护 ----------
                    if ((o_state == S_ALIGN) || (o_state == S_TRACK) || (o_state == S_HOLD)) begin
                        if (i_stall != 4'd0) begin
                            o_fault      <= 1'b1;
                            o_state      <= S_FAULT;
                            o_auto_valid <= 1'b0;
                        end else if (elapsed_cnt >= TIMEOUT_TICKS) begin
                            o_fault      <= 1'b1;
                            o_state      <= S_FAULT;
                            o_auto_valid <= 1'b0;
                        end else begin
                            elapsed_cnt  <= elapsed_cnt + 16'd1;
                        end
                    end
                end
            end
        end
    end

endmodule
