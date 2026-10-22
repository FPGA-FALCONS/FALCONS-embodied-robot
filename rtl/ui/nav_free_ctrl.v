//////////////////////////////////////////////////////////////////////////////////
// 模块：nav_free_ctrl —— 「非避障导航」页的状态控制（sys 域，50 MHz）
//
// 【依据】需求原文（doc\使用FPGA实现的差速小车功能设计.md）「非避障导航」一节：
//   "选中并确认该项时，菜单右侧画面显示一个坐标系（纵向正负5m，水平方向与纵向比例
//    相同，只画坐标轴不画网格，坐标轴延伸到画面边缘），x轴正方向朝右，y轴正方向朝上，
//    原点在该页面（不含菜单栏）竖直方向1/2、水平方向1/4处。车的位置以一个直径6px的
//    绿点表示，同时绿点上出现一个白色十字准心。右下角靠边显示一个辅助菜单…
//    此时摇杆的移动不控制菜单项，而是控制该准心上下左右移动。屏幕右上角以24px密集
//    数字实时显示准心的实时坐标…当按下确认键时（若选中"确认路径点"）记录准心位置…
//    当辅助菜单选中"间断寻点"/"非间断寻点"并按下确认键，开始对应寻点模式。"
//
// 【本模块只做什么】
//   维护这一页的**全部可交互状态**：准心位置、已记录点表、辅助菜单高亮项、运行状态。
//   **不驱动任何电机**：按「间断寻点 / 非间断寻点」+ 确认后只把 `o_nav_run` 拉高、
//   `o_nav_mode` 置成对应模式，车一步都不动 —— 真正的寻点执行留给后续的导航控制层
//   （方案见 doc\导航控制方案.md）。这是需求方本轮明确要求的边界。
//
// 【屏幕几何（与 rtl\ui\ui_render.v 必须一致）】
//   内容区 = x 208..1023（宽 816）× y 0..599（高 600）；
//   原点 = (208 + 816/4, 600/2) = **(412, 300)**；
//   纵向 ±5 m 铺满上下各 300 px ⇒ **60 px/m = 0.06 px/mm**；
//   横向同比例（需求方："x 轴范围没有要求，确保与 y 轴比例一致"）：
//     x = 208 → (208−412)/0.06 = **−3400 mm**
//     x = 1023 → (1023−412)/0.06 = **+10183 mm**
//     y = 0   → (300−0)/0.06   = **+5000 mm**
//     y = 599 → (300−599)/0.06 = **−4983 mm**
//   以上四个值就是准心的钳位边界（准心不许移出可见坐标系）。
//
// 【像素换算（0.06 px/mm 的定点实现）】
//   px = 412 + (x_mm × 3932) >>> 16     —— 3932/65536 = 0.059998，5 m 处误差 0.15 mm
//   py = 300 − (y_mm × 3932) >>> 16     —— 屏幕 y 向下，数学 y 向上，故取负
//   ★ 不要用 (x_mm × 61) >> 10（= 0.059570）：5 m 处偏 35 mm ≈ 2 px，肉眼可见。
//
// 【准心移动】
//   每 1 ms 更新一次：cursor_q10 += steer/throttle × K_Q10，K_Q10 = 500 × 1024/1000 = 512
//   满杆 500 mm/s（需求方 2026-10-19 选定）。1 ms 走 0.5 mm，远细于 0.01 m 的显示台阶。
//   死区 ±100（归一化 ±1000 的 10%）：遥控中位抖动约 ±50，覆盖得住。
//
// 【符号】
//   steer 正 = 摇杆右推 ⇒ 准心向 +x（右）移动；
//   throttle 正 = 摇杆前推 ⇒ 准心向 +y（上）移动。
//   若上板发现左右/上下反了，只改 `STEER_SIGN` / `THROTTLE_SIGN` 两个 localparam。
//
// 【点表】
//   上限 16（见 MAX_POINTS）。本页演示场景 3~5 点足够；16 点的像素坐标跨域后
//   展开成 2×160 位扁平总线，pixel 域每像素做 16 次比较，组合深度约 5 级，时序可过。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module nav_free_ctrl #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer CURSOR_SPEED = 500,     // mm/s @ 满杆
    parameter integer DEADZONE     = 100,     // 摇杆死区（±1000 归一化）
    parameter integer MAX_POINTS   = 16,
    // ---- 准心钳位（mm），由屏幕几何推出，见头部注释 ----
    parameter integer X_MIN_MM     = -3400,
    parameter integer X_MAX_MM     = 10183,
    parameter integer Y_MIN_MM     = -4983,
    parameter integer Y_MAX_MM     = 5000,
    // ---- 屏幕几何 ----
    parameter integer PX_ORIGIN    = 412,     // 原点像素 x
    parameter integer PY_ORIGIN    = 300,     // 原点像素 y
    parameter integer SCALE_Q16    = 3932     // 0.06 px/mm 的 Q16（= 0.059998）
)(
    input  wire                clk,
    input  wire                rst_n,

    input  wire                i_active,      // 1 = 当前停在某个导航页（避障 / 非避障）
    input  wire                i_is_obs,      // 1 = 避障导航页；0 = 非避障导航页
    input  wire                i_clear,       // S2 清零（与 odom 同源）
    input  wire signed [31:0]  i_x_mm,        // 车辆当前航位（mm，相对清零点）
    input  wire signed [31:0]  i_y_mm,

    input  wire signed [11:0]  i_steer,       // 摇杆（归一化 ±1000）
    input  wire signed [11:0]  i_throttle,
    input  wire                i_manual,      // 1 = 手动接管档（摇杆此时归底盘，不动准心）
    input  wire                i_confirm,     // 确认键（脉冲，CH5 下拨）
    input  wire                i_back,        // 返回键（脉冲，CH5 上拨）
    input  wire                i_aux_up,      // 辅助菜单向上（脉冲，CH8 上拨）
    input  wire                i_aux_down,    // 辅助菜单向下（脉冲，CH8 下拨）

    // ---- 与导航 / 控制层对接（2026-10-19 预留；顶层暂接常量）----
    //   ★ 真接入时：导航侧把 `i_ctrl_done` / `i_ctrl_fault` 打一拍送进来；
    //     本模块的 `o_nav_state / o_nav_mode / o_pt_* / o_cur_*` 送出去即可。
    input  wire                i_ctrl_done,   // 导航层回告：当前点已到达
    input  wire                i_ctrl_fault,  // 导航层回告：未到达而终止（堵转/超时/不可达）

    // ---- 观测量（给 num_text / 渲染用）----
    output reg  signed [31:0]  o_cur_x_mm,
    output reg  signed [31:0]  o_cur_y_mm,
    output reg  [2:0]          o_aux_sel,     // 辅助菜单高亮 0..4
    output wire [4:0]          o_pt_count,
    // ★ 2026-10-19：运行态扩成三态，并对外给出"导航页交互态"（菜单锁与状态栏都用它）
    output reg  [1:0]          o_nav_state,   // 0=SETUP 1=RUNNING 2=PAUSED
    output wire                o_nav_ui_active, // 1 = 导航页处于交互态（菜单锁、状态栏显示导航状态）
    output wire                o_nav_run,     // 1 = 导航中
    output wire                o_nav_paused,  // 1 = 导航暂停
    output reg                 o_nav_mode,    // 0 = 间断寻点 / 1 = 非间断寻点

    // ---- 像素坐标（扁平总线，跨域给 pixel 域）----
    output wire [MAX_POINTS*10-1:0] o_pt_px,  // 每个点 10 位
    output wire [MAX_POINTS*10-1:0] o_pt_py,
    output wire [MAX_POINTS-1:0]    o_pt_v,   // 该点有效
    // ---- 点表（mm，扁平总线，同域直连导航控制层）----
    //   ★ 点表内部本来就是 mm（`pt_x/pt_y` 是 `signed [15:0]` 毫米），不需要任何
    //     "像素→毫米"反变换；这里只是把它摊成一条扁平总线给 nav_ctrl 逐点取值。
    //     原点 = 上一次 S2 清零的位置，+x = 那一刻的车头 —— 与 odom 同一套相对系。
    output wire [MAX_POINTS*16-1:0] o_pt_x_mm,  // 每点 16 位有符号 mm
    output wire [MAX_POINTS*16-1:0] o_pt_y_mm,
    output wire [9:0]               o_car_px, // 车绿点（来自 odom 航位）
    output wire [9:0]               o_car_py,
    output wire [9:0]               o_cur_px, // 准心
    output wire [9:0]               o_cur_py
);

    // ------------------------------------------------------------------
    // 0. 常量
    // ------------------------------------------------------------------
    // 每 1 ms 的满杆增量（Q10 mm）：
    //   满杆位移 = CURSOR_SPEED [mm/s] × 0.001 s = 0.5 mm = 512 Q10
    //   实现：dx_step = (st_eff × K_SHIFT) >>> 10，K_SHIFT = 500×1024×1024/1e6 = 524
    //         满杆 1000 → 1000×524>>10 = 511 Q10 = 0.499 mm/ms（499 mm/s，误差 0.2%）
    //   ★ 不能直接写 st_eff × 512：那等于"每 1 ms 走 500 mm"，100 ms 就撞到边界了。
    localparam integer K_SHIFT = (CURSOR_SPEED * 1024 * 1024) / (1000 * 1000);
    localparam integer TICK_END = CLK_HZ / 1000 - 1;      // 50000−1

    localparam [2:0] AUX_SET    = 3'd0,   // 确认路径点
                     AUX_UNDO   = 3'd1,   // 撤销路径点
                     AUX_GAP    = 3'd2,   // 间断寻点（非避障页）
                     AUX_CONT   = 3'd3,   // 非间断寻点（非避障页）
                     AUX_CLEAR  = 3'd4;   // 清除所有路径点（非避障页，第 5 项）
    // 避障页：0 确认路径点 / 1 撤销路径点 / 2 开始导航 / 3 清除所有路径点
    localparam [2:0] AUX_START_OBS = 3'd2;
    localparam      AUX_CLEAR_OBS  = 3'd3;

    // 运行状态（2026-10-19 由"两级"扩为"三态"）
    localparam [1:0] ST_SETUP   = 2'd0,   // 未开跑：可移准心、可编辑点表
                     ST_RUNNING = 2'd1,   // 导航中
                     ST_PAUSED  = 2'd2;   // 导航暂停（返回键暂停，确认键继续）

    // ------------------------------------------------------------------
    // 0.1 手动挡按键屏蔽（2026-10-19 需求）
    //   ★ 原话："手动挡时确认键和 select 键都屏蔽！只有挡位切换键是有效的"
    //   手动挡下车归人开，摇杆是给底盘的，此时任何菜单操作都是误触：
    //     · 确认键（CH5 下拨）       → 记点 / 开跑 / 暂停恢复，全部屏蔽
    //     · 返回键（CH5 上拨）       → 暂停 / 退出交互态，屏蔽（挡位键切回光标挡后才能退）
    //     · 辅助菜单上/下切（CH8）   → "select 键"，屏蔽
    //   唯一有效的 CH7 挡位切换在 rc_input 里译码，本模块只被动接收 i_manual。
    //   ★ 门控放在**消费端**而不是信号源：`o_nav_state` 的手动分支要看得见 i_manual 的
    //     真实值（靠它进 PAUSED），所以不能把 i_manual 本身干掉，只能干掉四个按键。
    wire key_confirm  = i_confirm  & ~i_manual;
    wire key_back     = i_back     & ~i_manual;
    wire key_aux_up   = i_aux_up   & ~i_manual;
    wire key_aux_down = i_aux_down & ~i_manual;

    localparam signed [31:0] XLO = X_MIN_MM, XHI = X_MAX_MM,
                             YLO = Y_MIN_MM, YHI = Y_MAX_MM;

    // 符号（上板若发现反向，只改这两个）
    localparam integer STEER_SIGN    = -1;   // ★ 2026-10-19 上板实测：原 +1 时准心左右反向，取反
    localparam integer THROTTLE_SIGN =  1;

    // ------------------------------------------------------------------
    // 1. 1 ms 节拍
    // ------------------------------------------------------------------
    reg [15:0] tick_cnt;
    wire tick = (tick_cnt == TICK_END);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      tick_cnt <= 16'd0;
        else if (tick)   tick_cnt <= 16'd0;
        else             tick_cnt <= tick_cnt + 16'd1;
    end

    // ------------------------------------------------------------------
    // 2. 摇杆 → 每 ms 增量（带死区 + 符号）
    // ------------------------------------------------------------------
    wire signed [11:0] st_eff = (i_steer    >  DEADZONE) ? i_steer    :
                                (i_steer    < -DEADZONE) ? i_steer    : 12'sd0;
    wire signed [11:0] th_eff = (i_throttle >  DEADZONE) ? i_throttle :
                                (i_throttle < -DEADZONE) ? i_throttle : 12'sd0;

    // 位移先寄存，再做准心累加与限幅，乘法不进入坐标反馈链。
    reg signed [31:0] dx_step, dy_step;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dx_step <= 0;
            dy_step <= 0;
        end else begin
            dx_step <= (STEER_SIGN < 0) ? -((st_eff * K_SHIFT) >>> 10)
                                       :  ((st_eff * K_SHIFT) >>> 10);
            dy_step <= (THROTTLE_SIGN < 0) ? -((th_eff * K_SHIFT) >>> 10)
                                          :  ((th_eff * K_SHIFT) >>> 10);
        end
    end

    // ------------------------------------------------------------------
    // 3. 进入本页的上升沿：准心落到车的位置（并钳位）
    // ------------------------------------------------------------------
    reg active_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) active_d <= 1'b0;
        else        active_d <= i_active;
    end
    wire enter = i_active & ~active_d;

    function signed [31:0] clamp_mm;
        input signed [31:0] v;
        input signed [31:0] lo;
        input signed [31:0] hi;
        begin
            if (v > hi)      clamp_mm = hi;
            else if (v < lo) clamp_mm = lo;
            else             clamp_mm = v;
        end
    endfunction

    // ------------------------------------------------------------------
    // 4. 准心位置（Q10 mm 内部，mm 精度输出）
    // ------------------------------------------------------------------
    // 只在 SETUP（未开跑）时接受摇杆；开跑后锁定，避免边跑边改目标。
    // ★ 2026-10-19 需求方："当车处于手动挡，摇杆不控制准心移动。"
    //   手动接管档下摇杆是**给底盘开车的**，同一根杆不能既开车又移准心。
    //   ★ 注意：确认键 / select 键**不需要**在这里门控 —— `rc_input` 的 `cursor_ready`
    //     已要求"处于光标档 + 档位连续 2 帧确认"，手动档下这两个事件根本不产生。
    //     只有 `rc_steer` / `rc_throttle` 是从未被门控的原始归一化值，必须在这里挡。
    wire cursor_en = o_nav_ui_active & (o_nav_state == ST_SETUP) & ~i_manual;

    reg signed [31:0] cur_x_q10, cur_y_q10;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_x_q10 <= 32'sd0;
            cur_y_q10 <= 32'sd0;
        end else if (i_clear) begin
            cur_x_q10 <= 32'sd0;
            cur_y_q10 <= 32'sd0;
        end else if (enter) begin
            // 进入本页：准心 = 车当前位置（钳位到可见坐标范围）
            cur_x_q10 <= clamp_mm(i_x_mm, XLO, XHI) <<< 10;
            cur_y_q10 <= clamp_mm(i_y_mm, YLO, YHI) <<< 10;
        end else if (tick && cursor_en) begin
            cur_x_q10 <= clamp_mm(cur_x_q10 + dx_step, XLO <<< 10, XHI <<< 10);
            cur_y_q10 <= clamp_mm(cur_y_q10 + dy_step, YLO <<< 10, YHI <<< 10);
        end
    end

    // 输出 mm（四舍五入到 1 mm；显示只要 10 mm 分辨率，1 mm 足够）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_cur_x_mm <= 32'sd0;
            o_cur_y_mm <= 32'sd0;
        end else begin
            o_cur_x_mm <= cur_x_q10 >>> 10;
            o_cur_y_mm <= cur_y_q10 >>> 10;
        end
    end

    // ------------------------------------------------------------------
    // 5. 已记录点表（mm）与像素坐标
    // ------------------------------------------------------------------
    reg signed [15:0] pt_x [0:MAX_POINTS-1];
    reg signed [15:0] pt_y [0:MAX_POINTS-1];
    reg        [MAX_POINTS-1:0] pt_v;

    // 点数 = 有效位的个数（MAX_POINTS=16 时就是一个 popcount）
    //   ★ 必须声明在点表 always **之前**：点表写入要用它当索引，
    //     放到后面就是"先用后声明"，vlog 会报 (vlog-2730) undefined variable。
    //   ★ 这是组合函数、输入只有寄存器 pt_v，不构成组合环。
    integer j;
    reg [4:0] cnt_sum;
    always @(*) begin
        cnt_sum = 5'd0;
        for (j = 0; j < MAX_POINTS; j = j + 1)
            cnt_sum = cnt_sum + {4'd0, pt_v[j]};
    end

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pt_v <= {MAX_POINTS{1'b0}};
            for (k = 0; k < MAX_POINTS; k = k + 1) begin
                pt_x[k] <= 16'sd0;
                pt_y[k] <= 16'sd0;
            end
        end else if (i_clear || enter) begin
            // ★ 进入本页也清空：坐标原点是"上一次 S2 清零点"，换原点后旧点已无意义
            pt_v <= {MAX_POINTS{1'b0}};
        end else if (o_nav_ui_active && (o_nav_state == ST_SETUP)) begin
            if (key_confirm) begin
                case (o_aux_sel)
                    AUX_SET: begin
                        // 追加一个点（满了就不再收，避免覆盖旧点）
                        if (cnt_sum < MAX_POINTS) begin
                            pt_x[cnt_sum[3:0]] <= o_cur_x_mm[15:0];
                            pt_y[cnt_sum[3:0]] <= o_cur_y_mm[15:0];
                            pt_v[cnt_sum[3:0]] <= 1'b1;
                        end
                    end
                    AUX_UNDO: begin
                        if (cnt_sum != 5'd0) pt_v[cnt_sum - 5'd1] <= 1'b0;
                    end
                    AUX_CLEAR: pt_v <= {MAX_POINTS{1'b0}};
                    default: ;   // GAP / CONT 不改点表
                endcase
            end
        end
    end


    // ★ 点数直接组合输出：寄存器版本比 pt_v 晚一拍，测试台"按下确认后立刻读"会读到旧值
    //   （实测三连加出现 +1 的滞后）。cnt_sum 本身就是 pt_v 的纯函数，组合输出更真实。
    assign o_pt_count = cnt_sum;

    // 点表扁平化（mm）：给 nav_ctrl 用 `i_pt_x_mm[{idx,4'b0} +: 16]` 逐点取值。
    //   ★ 纯组合，不打拍：点表在 SETUP 态才写，导航 RUNNING 态只读，中间有按键节拍隔开。
    //   ★ genvar 必须用**另一个名字**：本模块下方（o_pt_px/o_pt_py）已声明过 `gi`，
    //     重复声明会报 `vlog-2388 'gi' already declared in this scope`。
    genvar gmm;
    generate
        for (gmm = 0; gmm < MAX_POINTS; gmm = gmm + 1) begin : g_pt_mm
            assign o_pt_x_mm[gmm*16 +: 16] = pt_x[gmm];
            assign o_pt_y_mm[gmm*16 +: 16] = pt_y[gmm];
        end
    endgenerate

    // ------------------------------------------------------------------
    // 6. 辅助菜单高亮（select 循环 0→1→2→3→4→0）
    // ------------------------------------------------------------------
    // 菜单项数按页不同：非避障 5 项（0..4）；避障 4 项（0..3，第 5 项"清除"在避障页是 3）
    //   ★ 避障页辅助菜单（确认路径点 / 撤销路径点 / 开始导航 / 清除所有路径点）——
    //     本轮**先不画**（依赖雷达地图），但状态机与项数先定义好，接雷达时不用改这里。
    wire [2:0] aux_last = i_is_obs ? AUX_CLEAR_OBS : AUX_CLEAR;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_aux_sel <= AUX_SET;          // 需求原文："初始默认选中确认路径点"
        end else if (i_clear || enter) begin
            o_aux_sel <= AUX_SET;
        end else if (o_nav_ui_active && (o_nav_state == ST_SETUP)) begin
            if (key_aux_down) o_aux_sel <= (o_aux_sel >= aux_last) ? AUX_SET : (o_aux_sel + 3'd1);
            else if (key_aux_up) o_aux_sel <= (o_aux_sel == AUX_SET) ? aux_last : (o_aux_sel - 3'd1);
        end
    end

    // ------------------------------------------------------------------
    // 7. 运行状态机
    //    SETUP(0) --确认  间断/非间断 + 至少 1 个点--> RUNNING(1)
    //    RUNNING  --返回键（预留）--> SETUP
    //    RUNNING 下确认/select 一律无效：原文没有"暂停寻点"，退出靠独立返回键
    // ------------------------------------------------------------------
    // ---- 导航页交互态（菜单锁 / 状态栏都靠它）----
    //   ★ 置 1：① `enter`（从菜单确认进入导航页的那一刻）
    //           ② `reenter`（已在本页、已退出交互态，再按一次确认）
    //   ★ 清 0：page 切走 / S2 清零 / 按返回且在 SETUP 或 PAUSED
    //   ★ 在 RUNNING 时按返回**不清**，因为那是"暂停"，不是退出。
    //   ★★ 2026-10-19 修 bug：原来只有 `enter` 能置 1，而 `enter` 是
    //      "page 进入本页的上升沿"（i_active & ~active_d）。退出交互态后 page 没变，
    //      active_d 一直为 1 → enter 再也不会产生 → **必须先切到别的页面再回来**
    //      才能重新进入。上板实测就是这个现象。加 `reenter` 修掉。
    reg ui_active;
    wire reenter = i_active & ~ui_active & key_confirm;   // 手动挡下不重进
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  ui_active <= 1'b0;
        else if (i_clear)            ui_active <= 1'b0;
        else if (!i_active)          ui_active <= 1'b0;
        else if (enter)              ui_active <= 1'b1;
        else if (key_back && (o_nav_state != ST_RUNNING)) ui_active <= 1'b0;
        else if (reenter)            ui_active <= 1'b1;
    end
    assign o_nav_ui_active = ui_active;
    assign o_nav_run       = (o_nav_state == ST_RUNNING);
    assign o_nav_paused    = (o_nav_state == ST_PAUSED);

    // ---- 三态状态机（2026-10-19）----
    //   SETUP --确认(寻点/开始导航)--> RUNNING
    //   RUNNING --返回--> PAUSED      PAUSED --确认--> RUNNING
    //   RUNNING --切手动挡--> PAUSED（★ 2026-10-19 补）
    //   SETUP/PAUSED --返回--> 退出交互态（ui_active=0，回到菜单选择）
    //
    //   ★ 为什么"切手动挡"要落成 PAUSED 而不是什么都不做：
    //     ① 底盘侧 `chassis_ctrl.v:117-118` 已是手动优先（`i_manual ? man_* : auto_*`），
    //        所以"车被手动接管"本来就会发生；但如果 UI 还停在 RUNNING，
    //        nav_ctrl 仍会每 10 ms 更新目标，切回光标挡那一瞬间车会**突然扑向旧目标**。
    //     ② 落 PAUSED 后 nav_ctrl 输出 center=diff=0（主动制动），且**内部状态不清**
    //        （pt_idx / 积分 / 完成计时都留着），所以按确认能"从刚才那一点继续"。
    //     ③ 手动挡本身不是"任务的终点"，所以不用 SETUP —— 那会把点表和进度一起清掉。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_nav_state <= ST_SETUP;
            o_nav_mode  <= 1'b0;
        end else if (i_clear || !i_active || enter) begin
            o_nav_state <= ST_SETUP;
            o_nav_mode  <= 1'b0;
        end else if (key_back) begin
            if (o_nav_state == ST_RUNNING) o_nav_state <= ST_PAUSED;
            else                           o_nav_state <= ST_SETUP;
        end else if (i_manual && (o_nav_state == ST_RUNNING)) begin
            // 运行中切手动挡 → 暂停（不是退出）。o_nav_mode 保持不变，恢复时沿用原寻点模式。
            o_nav_state <= ST_PAUSED;
        end else if (i_ctrl_fault) begin
            // 导航层报故障 → 退回 SETUP，不留在暂停态让人误以为还能继续
            o_nav_state <= ST_SETUP;
        end else if (i_ctrl_done) begin
            // 导航层报"当前点已到达" → 回 SETUP。本轮 i_ctrl_done 恒 0（未接）。
            o_nav_state <= ST_SETUP;
        end else if (key_confirm && o_nav_ui_active) begin
            // ★ 2026-10-19：确认必须**已在交互态**才算"执行辅助菜单项"。
            //   否则 ui_active=0 时按确认去"重新进入交互态"，会在同一拍被这里当成
            //   "开始导航"（点到「间断寻点」且点表非空时直接进 RUNNING）——那就是误触发。
            case (o_nav_state)
                ST_SETUP: begin
                    if (i_is_obs) begin
                        // 避障页：辅助菜单第 3 项「开始导航」
                        if (o_aux_sel == AUX_START_OBS) o_nav_state <= ST_RUNNING;
                    end else if ((o_aux_sel == AUX_GAP) || (o_aux_sel == AUX_CONT)) begin
                        // 非避障页：没有点就开跑是无意义的（会立刻"到达"），要求至少 1 个点
                        if (cnt_sum != 5'd0) begin
                            o_nav_state <= ST_RUNNING;
                            o_nav_mode  <= (o_aux_sel == AUX_CONT);
                        end
                    end
                end
                ST_PAUSED: begin
                    // 暂停中按确认 = 继续。
                    // ★ 但**手动挡下不恢复**：此刻底盘还在人手上，恢复了下一拍又会被
                    //   i_manual 顶回 PAUSED，只会让状态栏闪一下。要求先回光标挡。
                    if (!i_manual) o_nav_state <= ST_RUNNING;
                end
                default:   ;                              // RUNNING 下确认无效
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 8. mm → 像素（组合，宽 32 位中间量防溢出）
    // ------------------------------------------------------------------
    function [9:0] px_of;
        input signed [15:0] xmm;
        begin
            px_of = PX_ORIGIN + (((xmm * SCALE_Q16) >>> 16));
        end
    endfunction

    function [9:0] py_of;
        input signed [15:0] ymm;
        begin
            py_of = PY_ORIGIN - (((ymm * SCALE_Q16) >>> 16));
        end
    endfunction

    // ---- 车绿点：先算像素，再**钳到可见画面** ----
    //   ★ 2026-10-19 上板反馈"绿点没有正确显示小车的坐标"：准心是钳位过的（mm 级），
    //     车绿点原来**只做了换算、没做钳位**。车一旦跑出坐标系（例如 S2 之后又推了几米，
    //     或反向退到 x < −6.8 m），`car_px_w` 会变负或 >1023，而下面取的是 `[9:0]`——
    //     负值按补码截断成一个完全无关的正数，绿点就出现在画面里一个毫无意义的位置。
    //     钳到 [208,1023]×[0,599] 后，车出界时绿点停在对应边缘，位置始终可解释。
    // ---- 车辆位置输入先打一拍（2026-10-20 修时序）----
    //   实测 `u_odom/dyaw_acc_3_s1 → u_nav_free/mult_5763_s1` 共 7 条同域 SETUP
    //   违约（−0.958 ~ −0.792），逻辑级数只有 14。14 级逻辑只值约 7 ns 却超时 1 ns，
    //   说明是**纯布线**：`odom.o_x_mm` 那个寄存器的边界被综合器吸收进了消费者，
    //   于是 `odom` 内部积分树 → `nav_free_ctrl` 的 32 位乘法器连成一条长链，
    //   而两个模块在布局上被摆得很远。
    //   修法：在本模块输入端再插一级寄存器，强制多一个 DFF 边界（跨两级不会被吸收）。
    //   代价 1 拍（20 ns）—— 车辆位置是 10 ms 更新一次的量，完全无感。
    reg signed [31:0] ix_d1, iy_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_d1 <= 32'sd0;
            iy_d1 <= 32'sd0;
        end else begin
            ix_d1 <= i_x_mm;
            iy_d1 <= i_y_mm;
        end
    end

    wire signed [31:0] car_px_raw = PX_ORIGIN + (((ix_d1 * SCALE_Q16) >>> 16));
    wire signed [31:0] car_py_raw = PY_ORIGIN - (((iy_d1 * SCALE_Q16) >>> 16));
    wire signed [31:0] car_px_w   = (car_px_raw < 32'sd208)  ? 32'sd208 :
                                    (car_px_raw > 32'sd1023) ? 32'sd1023 : car_px_raw;
    wire signed [31:0] car_py_w   = (car_py_raw < 32'sd0)    ? 32'sd0 :
                                    (car_py_raw > 32'sd599)  ? 32'sd599 : car_py_raw;
    wire signed [31:0] cur_px_w = PX_ORIGIN + (((o_cur_x_mm * SCALE_Q16) >>> 16));
    wire signed [31:0] cur_py_w = PY_ORIGIN - (((o_cur_y_mm * SCALE_Q16) >>> 16));

    // ------------------------------------------------------------------
    // 8b. 像素坐标**打一拍再输出**（2026-10-19 首次综合后修时序）
    //
    //   首次综合实测：
    //     SETUP −3.086 ns  u_nav_free/nav_pt_py_10x_s1 → navpt_ps1_y_108_s0
    //   根因：`o_pt_py` 原来是**组合**输出（16 组 × (16 位常数乘 3932 + 移位) 一起
    //   接进顶层跨域同步器的第一级），这条链一拍做不完。`o_pt_px` 与车/准心同理
    //   （后者的乘法还是 32 位）。
    //   修法：在模块内部先打一拍，端口变成纯 DFF 输出 → 同步器第一级回到
    //   DFF→DFF 量级。代价 1 拍（20 ns），对 60 Hz 显示完全无感。
    // ------------------------------------------------------------------
    reg [9:0] car_px_r, car_py_r, cur_px_r, cur_py_r;
    reg [9:0] pt_px_r [0:MAX_POINTS-1];
    reg [9:0] pt_py_r [0:MAX_POINTS-1];

    integer g2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            car_px_r <= 10'd0; car_py_r <= 10'd0;
            cur_px_r <= 10'd0; cur_py_r <= 10'd0;
            for (g2 = 0; g2 < MAX_POINTS; g2 = g2 + 1) begin
                pt_px_r[g2] <= 10'd0;
                pt_py_r[g2] <= 10'd0;
            end
        end else begin
            car_px_r <= car_px_w[9:0];
            car_py_r <= car_py_w[9:0];
            cur_px_r <= cur_px_w[9:0];
            cur_py_r <= cur_py_w[9:0];
            for (g2 = 0; g2 < MAX_POINTS; g2 = g2 + 1) begin
                pt_px_r[g2] <= px_of(pt_x[g2]);
                pt_py_r[g2] <= py_of(pt_y[g2]);
            end
        end
    end

    assign o_car_px = car_px_r;
    assign o_car_py = car_py_r;
    assign o_cur_px = cur_px_r;
    assign o_cur_py = cur_py_r;

    genvar gi;
    generate
        for (gi = 0; gi < MAX_POINTS; gi = gi + 1) begin : g_pt
            assign o_pt_px[gi*10 +: 10] = pt_px_r[gi];
            assign o_pt_py[gi*10 +: 10] = pt_py_r[gi];
        end
    endgenerate
    assign o_pt_v = pt_v;

endmodule
