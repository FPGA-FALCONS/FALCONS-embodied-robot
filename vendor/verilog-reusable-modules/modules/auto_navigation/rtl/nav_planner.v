// ============================================================================
// 模块：nav_planner
// ----------------------------------------------------------------------------
// 职责：
//   - 按 cost 图做四邻接反向 BFS，从目标搜索到起点，再沿父方向输出轴向航点。
//   - 对足够长的中间段探测两侧余量并居中，逐格复核平移后的连接及末端桥接。
//   - 仅起点确被 cost 覆盖且 raw 空时允许局部接出；不负责活动 bank 或控制器握手。
// 接口约定：
//   - 单时钟、异步低有效复位；req_valid && req_ready 锁存起终点、wp_limit 和局部接出许可。
//   - mrd_valid && mrd_ready 接受一次读请求，以 mrd_rsp_valid 返回该地址 raw/cost。
//   - 航点格坐标 wp_valid 等待 wp_ready；wp_last 为本轮输出末点，可能只是容量截断前缀。
//   - success/no_path 在本轮结束给出并保持到下一请求；busy 只覆盖规划器自身计算。
// 关键不变量：
//   - QMAX 为 2 的幂，队列索引带环掩码；W_MAX>=2；位宽面向不超过 65536 格。
//   - raw 障碍始终不可穿越；局部放宽仅适用于锁存起点附近 ±BFS_RELAX 格。
//   - 发射背压不改变航点；居中探测、几何钳位、预算和最终线段复核分状态进行。
// 禁止事项：
//   - 不要把前缀 wp_last 直接解释为全局到点，也不要在居中后跳过完整连接段复核。
//   - 不要合并居中判决寄存链，或复用未测量段的陈旧探针结果。
// ============================================================================
module nav_planner #(
    parameter integer GRID_W        = 16,
    parameter integer GRID_H        = 16,
    // 队列环（ring），深度为 2 的幂。搜索以目标格为根、扩展的同时弹出，所以环里
    // 装的是边界（frontier），而不是整棵树。
    // 队列满时本轮 no_path；该结果也可能是容量限制，不能据此断言任意复杂地图无解。
    parameter integer QMAX          = 8192,
    // 规划器输出中继点的距离上限，实际间距 GAP=W_MAX-1 格。
    // 集成后的 nav_segments 合并连续同向中继，控制器只在最终拐点/前缀末点停稳。
    parameter integer W_MAX         = 39,
    // ---- 长直线横向居中（streaming centering）----------------------------
    // 0 = 不平移中间段；1 = 按 cost 余量居中，首段和末段不平移。
    parameter integer CENTER_EN     = 1,
    // 参与居中的最短段（格）。太短的段没有可平移的余量，硬平移只会破坏几何。
    parameter integer L_MIN         = 12,
    // 法向探针深度（格，每侧）。能测到 PROBE_D 格；超过则记为 PROBE_D+1（"无界"）。
    // 候选偏移为两侧最小间隙差的一半；PROBE_D=10 时探测候选最大为 5 格。
    parameter integer PROBE_D       = 10,
    // 沿段探针的等密度间距（格）：在段内每 PROBE_STEP 格取一个探测位置
    // （p = PROBE_STEP, 2*PROBE_STEP, ... < 段长），每个位置两侧各探 PROBE_D 格。
    // 长段增加探测位置，以覆盖段中和靠近端点的局部凸起。
    // 每个位置的成本 = 2*PROBE_D 次读；太小会显著增加轮次时长，建议 2~8。
    parameter integer PROBE_STEP    = 8,
    // 只有当前整段最小间隙 < CLEAR_TRIGGER 时才尝试居中（"只修贴边"，空旷处不折腾）。
    parameter integer CLEAR_TRIGGER = 8,
    // 起点"逃逸窗口"半径（格）：车辆可能已经处在膨胀带里（墙是后来观测到的），
    // 此时起点四邻在 cost 图上全是阻挡 → BFS 到不了起点 → no_path。允许 BFS 在
    // 起点周围该半径内穿过"raw 自由、只是被膨胀覆盖"的格子，把路线接出去；
    // 真正的障碍（raw=1）不受影响。取值应与 nav_map 的 INFLATE 一致。
    parameter integer BFS_RELAX     = 4,
    // 全路线长度最大增加量（格）。**关键**：平移一段时，路线总长的变化只取决于两侧
    // 相邻段的方向：
    //   * 两侧相邻段同向（Z 形，如 ↑→↑）：一段 +o、一段 −o ⇒ 总长不变 ⇒ 不设限；
    //   * 两侧相邻段反向（C/⊓ 形，如 ↑→↓）：两段都 +|o| ⇒ 总长 +2|o| ⇒ 用本预算约束。
    // 所以本参数只约束 C 形段（累计记账、可被"缩短方向"的平移退款）。
    parameter integer PATH_INC_MAX  = 20
) (
    input  wire               clk,
    input  wire               reset_n,

    input  wire               req_valid,
    input  wire               allow_start_escape,
    output wire               req_ready,
    input  wire        [15:0] start_x,
                              start_y,
                              goal_x,
                              goal_y,

    output reg                mrd_valid,
    input  wire               mrd_ready,
    output reg         [15:0] mrd_x,
                              mrd_y,
    input  wire               mrd_rsp_valid,
    // 未膨胀的"真障碍"位（nav_map 的 raw）。只用于起点逃逸窗口：区分
    input  wire               mrd_cost,
    // "真墙"与"只是被膨胀覆盖的自由格"。
    input  wire               mrd_raw,
    input  wire        [15:0] wp_limit,

    output reg                wp_valid,
    input  wire               wp_ready,
    output reg         [15:0] wp_x,
                              wp_y,
    output reg                wp_last,

    output wire               busy,
    output reg                success,
    output reg                no_path,
    output reg         [15:0] path_len
);
    localparam integer N = GRID_W * GRID_H;
    localparam integer WKLIM = 4096;  // 行走步数上限，仅作安全网（safety net）
    // 中继点间距：默认 W_MAX=39，对应 GAP=38 格；同向点在下游合并。
    // **约束：W_MAX 必须 >= 2**。W_MAX=1 会让 GAP=0，发射状态里 `em_rem > GAP`
    // 恒真、每拍产出同一个中继点，形成死循环（没有运行时保护，靠参数选择保证）。
    localparam integer GAP = W_MAX - 1;
    // 探测/偏移的位宽边界：每侧最多测 PROBE_D 格，"测不到"记 PROBE_D+1；
    // 单段偏移的硬上限（几何钳位用）取 OFF_MAX 格，保证位宽固定。
    localparam integer D_UNBOUND = PROBE_D + 1;
    localparam integer PROBE_END = 2 * PROBE_D;
    localparam integer OFF_MAX = 16;

    localparam [5:0] S_IDLE = 6'd0;
    localparam [5:0] S_CLR = 6'd1;
    localparam [5:0] S_SEED = 6'd2;
    localparam [5:0] BFS_POP = 6'd3;
    localparam [5:0] BFS_EXP = 6'd4;
    localparam [5:0] BFS_REQ = 6'd5;
    localparam [5:0] BFS_RSP = 6'd6;
    localparam [5:0] BFS_NEXT = 6'd7;
    localparam [5:0] WK_PRES = 6'd8;
    localparam [5:0] WK_CAP = 6'd9;
    localparam [5:0] WK_DEC = 6'd10;
    localparam [5:0] WK_STEP = 6'd11;
    localparam [5:0] CR_SETUP = 6'd12;
    localparam [5:0] PR_ISSUE = 6'd13;
    localparam [5:0] PR_TAKE = 6'd14;
    localparam [5:0] CR_CAND = 6'd15;
    localparam [5:0] CR_CLAMP1 = 6'd16;
    localparam [5:0] CR_CLAMP2 = 6'd17;
    localparam [5:0] CR_CLAMP3 = 6'd18;
    localparam [5:0] CR_APPLY = 6'd21;
    localparam [5:0] EM_SETUP = 6'd22;
    localparam [5:0] EM_REM = 6'd23;
    localparam [5:0] EM_SEND = 6'd24;
    localparam [5:0] GL_SETUP = 6'd25;
    localparam [5:0] RD_REQ = 6'd26;
    localparam [5:0] RD_RSP = 6'd27;
    localparam [5:0] CR_BUDGET = 6'd28;
    localparam [5:0] PR_SETUP = 6'd29;

    localparam [5:0] START_SETUP = 30,
                     START_TAKE  = 31,
                     GOAL_SETUP  = 32,
                     GOAL_TAKE   = 33,
                     CHECK_SETUP = 34,
                     CHECK_ISSUE = 35,
                     CHECK_TAKE  = 36,
                     CR_FILTER   = 37;
    reg [ 5:0] state;
    reg        start_cost_blocked;
    reg        start_escape_q;
    reg [15:0] wp_limit_q;
    reg [ 1:0] check_phase;
    reg signed [16:0] check_x, check_y, check_ex, check_ey;
    reg signed [16:0] line_sx, line_sy, line_ex, line_ey;

    reg        seen [   0:N-1];
    reg [ 1:0] pdir [   0:N-1];
    // 搜索方向随格号入队；每格先扩展直行，再左、右、反向，FIFO 层次保持不变。
    reg [17:0] queue[0:QMAX-1];

    localparam [15:0] QMASK = QMAX - 1;

    reg [15:0] q_head, q_tail;
    reg [15:0] clr_i;
    reg [15:0] cur;
    reg [ 1:0] search_dir;
    reg [ 2:0] k;
    reg [16:0] nb;
    reg [15:0] nb_q;
    reg [ 1:0] dir_q;
    reg        seen_q;
    reg [15:0] start_idx, goal_idx;

    // ---- 行走（forward walk）----
    // 输出拐点和有限间距中继；wp_limit 限制本轮数量，达到容量就结束为可执行前缀。
    // 不要求车辆同时消费候选，候选完整后才允许外部验证和发布。
    //
    // wk_first 强制起点格被跳过（它没有可以先作比较的入方向）；目标格则由 WK_DEC
    // 里它自己的条件输出。
    reg [15:0] wk_cell, wk_steps, wk_rd;
    reg [ 1:0] wk_dir;
    reg [ 1:0] wk_prev_dir;
    reg        wk_first;
    // 本轮到目前为止已输出的航点数：用它把路径约束在缓冲区容量内，它和
    // wk_steps（已走过的单元数）不是同一个量。
    reg [15:0] wp_count;
    // 当前段已走过的格数（从上一个拐点起算），也是待测量段的长度。
    reg [15:0] run_len;

    // ---- 段级居中上下文 ----
    // run_sx/run_sy：当前段起点 = 上一个拐点 c_{j-1} 的**真实**坐标，也是等待
    //   发射的拐点（待发拐点）的真实位置。
    // last_x/last_y：已发出的上一个拐点 c_{j-2} 的**最终**坐标（已含平移）。
    // seg_dir：到达 c_{j-1} 的那一段 r_{j-2} 的方向（钳位与发射时判断轴）。
    // off_prev/off_cur：o_{j-2} 与 o_{j-1}。
    //   探测候选默认最多 2 格；几何钳位允许抬高至 OFF_MAX，以防相邻段塌缩。
    reg signed [15:0] run_sx, run_sy;
    reg signed [15:0] last_x, last_y;
    reg [1:0] seg_dir;
    reg       corner_goal;  // 当前处理的拐点就是目标格（最后一段的终点）
    reg       emit_pending;  // 已经有待发拐点（j >= 2 时为真）
    reg signed [5:0] off_prev, off_cur;
    // 本轮累计的"路线长度增加量"（格）：C 形段计 2|o|、Z 形段 0；缩短方向为负（退款）。
    reg signed [15:0] cum_inc;
    reg [15:0] corner_x, corner_y;  // 当前拐点的真实坐标

    // ---- 探针 / 复核 ----
    // 等密度布点：p = PROBE_STEP, 2*PROBE_STEP, ... < run_len；每个位置两侧
    // 各探 PROBE_D 格。mid_x/mid_y 是该位置的"基点"（沿段方向走 p 格后的格子）。
    reg [15:0] mid_x, mid_y;  // 当前探针位置（基点）
    reg [15:0] probe_p;  // 沿段方向的偏移（格，0 < probe_p < run_len）
    reg [ 4:0] probe_k;  // 1..2*PROBE_D
    reg [4:0]
        d_plus,
        d_minus;  // 全段所有探针位置的 + / - 侧最小间隙（PROBE_D+1 = 无界）
    // 本拐点是否真的做了测量。d_plus/d_minus 是跨拐点保留的寄存器，一旦某一段
    // 没测（首段/末段/太短/居中关闭），它们仍是上一段的值；不门控的话组合判决会拿
    // 陈旧探针结果去改本段偏移，导致末段目标格被平移、相邻航点变斜线。
    reg               meas_on;

    // ---- 居中判决的寄存链（见“禁止事项”最后一条）----
    reg signed [ 5:0] cand_v;  // 通过 CLEAR_TRIGGER 的候选偏移
    reg signed [ 5:0] off_c1;  // 长度钳位后
    reg signed [ 5:0] off_grid;  // 网格钳位后（组合）
    reg signed [15:0] cl_d;  // last_ax - pend_ax（长度参考）
    // cl_hi/cl_lo 必须是有符号：它们要和 6 bit 有符号的偏移比较，如果这里是无符号，
    // 负偏移会被当成大正数，"-1 越界"就变成一个假的正向钳位（实测把 -1 变成 +7）。
    reg signed [15:0] cl_hi;  // GRID-1-pend_ax
    reg signed [15:0] cl_lo;  // -pend_ax
    // C/Z 分类：u = 该段的"+法向"方向，s1 = u·(前一段方向)，s2 = u·(后一段方向)。
    //   s1 == s2 ⇒ Z 形（平移不改变总长）；s1 != s2 ⇒ C 形（总长 +2|o|）。
    reg cls_s1, cls_s2;
    reg is_c;
    reg cl_raised;  // 长度钳位把偏移抬大了（几何优先，不再让预算压回去）
    reg signed [15:0]
        charge_cur;  // 本段实际计入 cum_inc 的长度增量（复核失败要退款）

    // 由 current 段的朝向派生两个朝向判别（见“禁止事项”最后一条）：
    //   seg_horiz  ：r_{j-1}（测量中的段）水平
    //   pseg_horiz ：r_{j-2}（待发射的段）水平 —— 与 seg_horiz 相反
    // 必须声明在下面的组合块之前：ModelSim 会把提前引用的名字当成隐式线网。
    wire seg_horiz = (wk_prev_dir == 2'd0) || (wk_prev_dir == 2'd1);
    wire pseg_horiz = (seg_dir == 2'd0) || (seg_dir == 2'd1);

    // C/Z 分类（组合，进入 CR_CAND 时寄存）：
    //   u = 测量段 r_{j-1} 的"+法向"（水平段 = +y=dir 3，垂直段 = +x=dir 1）
    //   s1 = u·d_{j-2}（前一段方向），s2 = u·d_j（后一段方向）
    //   s1 == s2 ⇒ Z 形：平移一段使一侧变长 |o|、另一侧变短 |o| ⇒ 总长不变；
    //   s1 != s2 ⇒ C 形：两侧都变长 |o| ⇒ 总长 +2|o|，要用 PATH_INC_MAX 约束。
    wire cls1_w = seg_horiz ? (seg_dir == 2'd3) : (seg_dir == 2'd1);
    wire cls2_w = seg_horiz ? (wk_dir == 2'd3) : (wk_dir == 2'd1);

    // 组合：候选偏移（只依赖探针结果与常量，全部 ≤5 bit）。
    //   候选 = (d_plus - d_minus) / 2，即"向较远的一侧移动到两侧间隙相等"，
    //   也就是尽可能居中。只有 min(d+, d-) < CLEAR_TRIGGER（确实贴边）才动手。
    //   **未知区按可通行**（与原始地图语义一致）：某一侧在 PROBE_D 格内没探到墙，
    //   就按"这一侧的开阔宽度 = PROBE_D+1"参与计算（= 最大宽度），不再额外限幅。
    //   平移量因此完全由探针结果决定；能否真正采用还要过整段复核、几何钳位和 C 形预算。
    reg signed [4:0] cand_c;
    reg [4:0] cand_old;
    reg cand_use;
    always @(*) begin
        cand_old = (d_plus < d_minus) ? d_plus : d_minus;
        if (d_plus > d_minus) cand_c = ({1'b0, d_plus} - {1'b0, d_minus}) >> 1;
        else if (d_plus < d_minus) cand_c = -(({1'b0, d_minus} -{1'b0, d_plus}) >> 1);
        else cand_c = 5'sd0;
        cand_use = (cand_old < CLEAR_TRIGGER[4:0]) && (cand_c != 5'sd0);
    end

    // 组合：网格钳位（只依赖 off_c1 与两个 16 bit 边界）
    always @(*) begin
        off_grid = off_c1;
        if (off_grid > cl_hi) off_grid = cl_hi[5:0];
        if (off_grid < cl_lo) off_grid = cl_lo[5:0];
    end

    // 组合：C 形段的长度预算。Z 形段（is_c=0）完全不受预算约束；
    // C 形段若"变长"，就要 cumulative + 2|o| ≤ PATH_INC_MAX；"变短"方向免费（还退款）。
    // 几何钳位把偏移抬大的情况（cl_raised）例外：此时上一段会塌陷，几何优先。
    reg signed [ 5:0] off_bud;
    reg signed [15:0] charge_bud;
    reg signed [15:0] bud_lim;
    always @(*) begin
        off_bud = off_grid;
        charge_bud = 16'sd0;
        bud_lim = 16'sd0;  // 必须初始化：否则综合会在部分路径上推断出锁存器
        if (is_c) begin
            // 正偏移沿 +u：s1=+1 时两段都变长（+2o），s1=-1 时两段都变短（-2o）。
            charge_bud = cls_s1 ? ({{10{off_grid[5]}}, off_grid} <<< 1) :
                -({{10{off_grid[5]}}, off_grid} <<< 1);
            if (!cl_raised && (charge_bud > 16'sd0)) begin
                if (cum_inc >= PATH_INC_MAX) begin
                    off_bud    = 6'sd0;
                    charge_bud = 16'sd0;
                end else if ((cum_inc + charge_bud) > PATH_INC_MAX) begin
                    bud_lim = (PATH_INC_MAX - cum_inc) >> 1;  // 剩余预算支持的 |o|
                    if (off_grid > 0) begin
                        if (off_grid > bud_lim) begin
                            off_bud    = bud_lim[5:0];
                            charge_bud = bud_lim <<< 1;
                        end
                    end else begin
                        if (off_grid < -bud_lim) begin
                            off_bud    = -bud_lim[5:0];
                            charge_bud = bud_lim <<< 1;
                        end
                    end
                end
            end
        end
    end

    // ---- 通用读握手 ----
    reg [5:0] rd_ret;  // 响应后返回的状态
    reg rd_cost_q, rd_raw_q;

    // ---- 航点发射 ----
    // 发射是“按段”的：每个发射元素从一个已发点走到一个终点，途中每 GAP 格发一个
    // 中继点，最后发终点。em_* 只是两个坐标加一个剩余距离，没有拐点表。
    reg [15:0] em_cx, em_cy;  // 当前已发到哪
    reg [15:0] em_ex, em_ey;  // 本元素终点
    reg [15:0] em_rem;  // 剩余（轴对齐，所以就是该轴距离）
    reg [ 1:0] em_dir;
    reg        em_final;  // 本元素终点就是目标格
    reg        em_point_end;  // 刚产出的点是本段终点
    reg        term_goal;  // 刚产出的点是目标格（胜利用 goal 分支）

    // 组合：待发拐点 c_{j-1} 的最终坐标（r_{j-2} 的终点）。
    //   r_{j-2} 水平：x 是段方向轴，加 o_{j-1}；y 是法向轴，加 o_{j-2}
    //   r_{j-2} 垂直：y 加 o_{j-1}；x 加 o_{j-2}
    reg signed [15:0] em_px, em_py;
    always @(*) begin
        if (pseg_horiz) begin
            em_px = run_sx + off_cur;
            em_py = run_sy + off_prev;
        end else begin
            em_px = run_sx + off_prev;
            em_py = run_sy + off_cur;
        end
    end

    always @* begin
        line_sx = last_x;
        line_sy = last_y;
        line_ex = emit_pending ? em_px : corner_x;
        line_ey = emit_pending ? em_py : corner_y;
        if (check_phase == 1) begin
            line_sx = emit_pending ? em_px : last_x;
            line_sy = emit_pending ? em_py : last_y;
            line_ex = $signed({1'b0, corner_x}) + (seg_horiz ? 0 : off_cur);
            line_ey = $signed({1'b0, corner_y}) + (seg_horiz ? off_cur : 0);
        end else if (check_phase == 2) begin
            line_sx = $signed({1'b0, corner_x});
            line_sy = $signed({1'b0, corner_y});
            line_ex = line_sx + (seg_horiz ? 0 : off_cur);
            line_ey = line_sy + (seg_horiz ? off_cur : 0);
        end
    end
    task reject_shift;
        begin
            if (off_cur != 0) begin
                off_cur    <= 0;
                cum_inc    <= cum_inc - charge_cur;
                charge_cur <= 0;
                state      <= CR_FILTER;
            end else begin
                no_path  <= 1;
                success  <= 0;
                wp_valid <= 0;
                state    <= S_IDLE;
            end
        end
    endtask
    assign req_ready = reset_n && state == S_IDLE;
    assign busy      = reset_n && (state != S_IDLE);

    function [15:0] parent_of;
        input [15:0] idx;
        input [1:0] dir;
        begin
            case (dir)
                2'd0:    parent_of = idx - 16'd1;
                2'd1:    parent_of = idx + 16'd1;
                2'd2:    parent_of = idx - GRID_W[15:0];
                default: parent_of = idx + GRID_W[15:0];
            endcase
        end
    endfunction

    function [16:0] neighbour;
        input [15:0] idx;
        input [2:0] dir;
        reg [15:0] x, y;
        begin
            x = idx % GRID_W;
            y = idx / GRID_W;
            case (dir)
                3'd0:    x = x + 16'd1;
                3'd1:    x = x - 16'd1;
                3'd2:    y = y + 16'd1;
                default: y = y - 16'd1;
            endcase
            if (x < GRID_W && y < GRID_H) neighbour = y * GRID_W + x;
            else neighbour = 17'h10000;
        end
    endfunction

    function [1:0] expansion_dir;
        input [1:0] heading;
        input [2:0] slot;
        begin
            // 编码为 +x、-x、+y、-y；这是反向搜索的方向，不是车辆朝向。
            case (slot)
                3'd0:    expansion_dir = heading;
                3'd1:    expansion_dir = {~heading[1], heading[1] ^ heading[0]};
                3'd2:    expansion_dir = {~heading[1], ~(heading[1] ^ heading[0])};
                default: expansion_dir = {heading[1], ~heading[0]};
            endcase
        end
    endfunction
    wire [1:0] expand_dir = expansion_dir(search_dir, k);

    // 有符号差：钳位需要知道“已发点 - 待发拐点”的正负。16 位无符号相减会回绕，
    // 所以先把两边零扩展成 17 位有符号再相减。
    function integer sdiff;
        input [15:0] a, b;
        begin
            sdiff = $signed({1'b0, a}) - $signed({1'b0, b});
        end
    endfunction

    wire clr_en = reset_n && (state == S_CLR);
    wire seed_en = reset_n && (state == S_SEED);
    function cell_allowed;
        input [15:0] x, y;
        input raw, cost;
        reg signed [16:0] dx, dy;
        begin
            dx = $signed({1'b0, x}) - $signed(start_idx % GRID_W);
            dy = $signed({1'b0, y}) - $signed(start_idx / GRID_W);
            cell_allowed = x < GRID_W && y < GRID_H && !raw &&
                (!cost || (start_cost_blocked && dx >= -BFS_RELAX && dx <= BFS_RELAX &&
                           dy >= -BFS_RELAX && dy <= BFS_RELAX));
        end
    endfunction
    wire nb_allowed = cell_allowed(nb_q % GRID_W, nb_q / GRID_W, mrd_raw, mrd_cost);
    wire arrive_start = reset_n && state == BFS_RSP && mrd_rsp_valid && nb_q == start_idx &&
        nb_allowed;
    wire new_cell = reset_n && state == BFS_RSP && mrd_rsp_valid && !seen_q && nb_allowed;
    wire mark_en = arrive_start | new_cell;
    wire push_en = new_cell && (nb_q != start_idx) && ((q_tail - q_head) < QMAX);

    wire seen_we = clr_en | seed_en | mark_en;
    wire [15:0] seen_wa = clr_en ? clr_i : (seed_en ? goal_idx : nb_q);
    wire seen_wd = clr_en ? 1'b0 : 1'b1;

    wire pdir_we = clr_en | mark_en;
    wire [15:0] pdir_wa = clr_en ? clr_i : nb_q;
    wire [1:0] pdir_wd = clr_en ? 2'd0 : dir_q;

    wire queue_we = seed_en | push_en;
    // q_head/q_tail 是本轮累计出入队计数，不是 RAM 索引；访问数组必须取环掩码。
    // 队列占用由尾减头判断，避免长搜索时索引越界或覆盖尚未消费的边界项。
    wire [15:0] queue_wa = seed_en ? 16'd0 : (q_tail & QMASK);
    // 目标根节点没有入方向，固定以 +x 为首选；父方向与实际扩展方向使用同一编码。
    wire [17:0] queue_wd = seed_en ? {2'd0, goal_idx} : {dir_q, nb_q};

    always @(posedge clk) begin
        if (seen_we) seen[seen_wa] <= seen_wd;
        if (pdir_we) pdir[pdir_wa] <= pdir_wd;
        if (queue_we) queue[queue_wa] <= queue_wd;
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state              <= S_IDLE;
            mrd_valid          <= 1'b0;
            wp_valid           <= 1'b0;
            wp_last            <= 1'b0;
            wp_x               <= 16'd0;
            wp_y               <= 16'd0;
            success            <= 1'b0;
            no_path            <= 1'b0;
            path_len           <= 16'd0;
            clr_i              <= 16'd0;
            q_head             <= 16'd0;
            q_tail             <= 16'd0;
            k                  <= 3'd0;
            search_dir         <= 2'd0;
            nb_q               <= 16'd0;
            dir_q              <= 2'd0;
            seen_q             <= 1'b0;
            wk_cell            <= 16'd0;
            wk_steps           <= 16'd0;
            wk_rd              <= 16'd0;
            wk_dir             <= 2'd0;
            wk_prev_dir        <= 2'd0;
            wk_first           <= 1'b0;
            wp_count           <= 16'd0;
            run_len            <= 16'd0;
            run_sx             <= 16'sd0;
            run_sy             <= 16'sd0;
            last_x             <= 16'sd0;
            last_y             <= 16'sd0;
            seg_dir            <= 2'd0;
            corner_goal        <= 1'b0;
            emit_pending       <= 1'b0;
            off_prev           <= 6'sd0;
            off_cur            <= 6'sd0;
            cum_inc            <= 16'sd0;
            corner_x           <= 16'd0;
            corner_y           <= 16'd0;
            mid_x              <= 16'd0;
            mid_y              <= 16'd0;
            probe_p            <= 16'd0;
            probe_k            <= 6'd0;
            d_plus             <= 6'd0;
            d_minus            <= 6'd0;
            meas_on            <= 1'b0;
            cand_v             <= 6'sd0;
            off_c1             <= 6'sd0;
            cls_s1             <= 1'b0;
            cls_s2             <= 1'b0;
            is_c               <= 1'b0;
            cl_raised          <= 1'b0;
            charge_cur         <= 16'sd0;
            cl_d               <= 16'sd0;
            cl_hi              <= 16'd0;
            cl_lo              <= 16'sd0;
            rd_ret             <= S_IDLE;
            rd_cost_q          <= 1'b0;
            rd_raw_q           <= 0;
            start_cost_blocked <= 0;
            start_escape_q    <= 0;
            wp_limit_q         <= 0;
            check_phase        <= 0;
            check_x            <= 0;
            check_y            <= 0;
            check_ex           <= 0;
            check_ey           <= 0;
            em_cx              <= 16'd0;
            em_cy              <= 16'd0;
            em_ex              <= 16'd0;
            em_ey              <= 16'd0;
            em_rem             <= 16'd0;
            em_dir             <= 2'd0;
            em_final           <= 1'b0;
            em_point_end       <= 1'b0;
            term_goal          <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    wp_valid <= 1'b0;
                    wp_last  <= 1'b0;
                    if (req_valid && start_x < GRID_W && start_y < GRID_H && goal_x < GRID_W &&
                        goal_y < GRID_H && wp_limit != 0) begin
                        wp_limit_q   <= wp_limit;
                        start_escape_q <= allow_start_escape;
                        start_idx    <= start_y * GRID_W + start_x;
                        goal_idx     <= goal_y * GRID_W + goal_x;
                        clr_i        <= 16'd0;
                        success      <= 1'b0;
                        no_path      <= 1'b0;
                        path_len     <= 16'd0;
                        cum_inc      <= 16'sd0;
                        emit_pending <= 1'b0;
                        off_prev     <= 6'sd0;
                        off_cur      <= 6'sd0;
                        state        <= START_SETUP;
                    end
                end

                START_SETUP: begin
                    mrd_x     <= start_idx % GRID_W;
                    mrd_y     <= start_idx / GRID_W;
                    mrd_valid <= 1;
                    rd_ret    <= START_TAKE;
                    state     <= RD_REQ;
                end
                START_TAKE: begin
                    start_cost_blocked <= rd_cost_q && start_escape_q;
                    if (rd_raw_q || (rd_cost_q && !start_escape_q)) begin
                        no_path <= 1;
                        state   <= S_IDLE;
                    end else state <= GOAL_SETUP;
                end
                GOAL_SETUP: begin
                    mrd_x     <= goal_idx % GRID_W;
                    mrd_y     <= goal_idx / GRID_W;
                    mrd_valid <= 1;
                    rd_ret    <= GOAL_TAKE;
                    state     <= RD_REQ;
                end
                GOAL_TAKE: begin
                    if (!cell_allowed(
                            goal_idx % GRID_W, goal_idx / GRID_W, rd_raw_q, rd_cost_q
                        )) begin
                        no_path <= 1;
                        state   <= S_IDLE;
                    end else state <= S_CLR;
                end
                S_CLR: begin
                    if (clr_i == N - 1) state <= S_SEED;
                    else clr_i <= clr_i + 16'd1;
                end

                S_SEED: begin
                    q_head <= 16'd0;
                    q_tail <= 16'd1;
                    state  <= BFS_POP;
                end

                BFS_POP: begin
                    if (start_idx == goal_idx) begin
                        wk_cell  <= start_idx;
                        wk_steps <= 16'd0;
                        wk_first <= 1'b1;
                        wp_count <= 16'd0;
                        run_len  <= 16'd0;
                        // 起终点重合：唯一一个航点就是车辆所在格。先把它记成“已发点”，
                        // 最后一段（长度为 0）会把它作为目标格发出去。
                        last_x   <= $signed(start_idx % GRID_W);
                        last_y   <= $signed(start_idx / GRID_W);
                        run_sx   <= $signed(start_idx % GRID_W);
                        run_sy   <= $signed(start_idx / GRID_W);
                        state    <= WK_PRES;
                    end else if (q_head == q_tail) begin
                        no_path <= 1'b1;
                        state   <= S_IDLE;
                    end else begin
                        {search_dir, cur} <= queue[q_head&QMASK];
                        q_head <= q_head + 16'd1;
                        k      <= 3'd0;
                        state  <= BFS_EXP;
                    end
                end

                BFS_EXP: begin
                    nb = neighbour(cur, {1'b0, expand_dir});
                    if (nb[16]) begin
                        state <= BFS_NEXT;
                    end else begin
                        nb_q   <= nb[15:0];
                        seen_q <= seen[nb[15:0]];
                        dir_q  <= expand_dir;
                        mrd_x  <= nb % GRID_W;
                        mrd_y  <= nb / GRID_W;
                        state  <= BFS_REQ;
                    end
                end

                BFS_REQ: begin
                    if (!mrd_valid) begin
                        mrd_valid <= 1'b1;
                    end else if (mrd_ready) begin
                        mrd_valid <= 1'b0;
                        state     <= BFS_RSP;
                    end
                end

                BFS_RSP: begin
                    if (mrd_rsp_valid) begin
                        if (nb_q == start_idx && nb_allowed) begin
                            wk_cell  <= start_idx;
                            wk_steps <= 16'd0;
                            wk_first <= 1'b1;
                            wp_count <= 16'd0;
                            run_len  <= 16'd0;
                            // 起点格是已发点（不输出），也是当前段的起点。
                            last_x   <= $signed(start_idx % GRID_W);
                            last_y   <= $signed(start_idx / GRID_W);
                            run_sx   <= $signed(start_idx % GRID_W);
                            run_sy   <= $signed(start_idx / GRID_W);
                            // 经由 WK_PRES 进入，而不是直接跳进发射：方向由 WK_CAP 读
                            // 出的 pdir 决定，直接发射意味着 wk_dir 还停留在上一轮的
                            // 方向，第一步就走错方向。
                            state    <= WK_PRES;
                        end else if (seen_q || !nb_allowed) begin
                            state <= BFS_NEXT;
                        end else if ((q_tail - q_head) >= QMAX) begin
                            no_path <= 1'b1;
                            state   <= S_IDLE;
                        end else begin
                            q_tail <= q_tail + 16'd1;
                            state  <= BFS_NEXT;
                        end
                    end
                end

                BFS_NEXT: begin
                    if (k == 3'd3) state <= BFS_POP;
                    else begin
                        k     <= k + 3'd1;
                        state <= BFS_EXP;
                    end
                end

                // 每格：读出 pdir，判定“方向变化处 / 目标格”两个出口。
                WK_PRES: begin
                    wk_rd <= wk_cell;
                    state <= WK_CAP;
                end

                WK_CAP: begin
                    wk_dir <= pdir[wk_rd];
                    // 在单独的状态里做判决。在这里测 wk_dir 会读到上一格的值，因为非阻塞
                    // 赋值还没落地：比较的对象会变成倒数第二格的方向，拐点检测就会
                    // 悄无声息地误判。
                    state  <= WK_DEC;
                end

                WK_DEC: begin
                    // 契约：发布的路线只包含车辆前方需要转向的拐点，第一项就是“下一个
                    // 去的拐点”。起点格由 nav_drive 自己所在的格隐含掉，不占路线槽位。
                    // 这样发布端每轮重规划后把进度重置为 0 就天然正确，也不会出现“目标
                    // 落在车辆后方（起点格）而把车拉回去”的情形。
                    //
                    // 例外：起点同时也是目标（start == goal）时必须输出它，否则这条路线
                    // 会是 0 个拐点，nav_route 无法表达“已经到达”。
                    if ((wk_cell == goal_idx) || (!wk_first && (wk_dir != wk_prev_dir)))
                        state <= CR_SETUP;
                    else state <= WK_STEP;
                end

                WK_STEP: begin
                    if (wk_steps >= WKLIM) begin
                        // 行走安全网：只有行走侧异常才会触发。
                        no_path <= 1'b1;
                        state   <= S_IDLE;
                    end else begin
                        wk_cell     <= parent_of(wk_cell, wk_dir);
                        wk_steps    <= wk_steps + 16'd1;
                        run_len     <= run_len + 16'd1;
                        wk_prev_dir <= wk_dir;
                        wk_first    <= 1'b0;
                        state       <= WK_PRES;
                    end
                end

                // ---------------- 拐点处理：测量刚结束的段 r_{j-1} ----------------
                // 进入条件：在当前格 wk_cell 发现拐点（方向变化或目标格）。此时
                //   run_sx/run_sy = c_{j-1} 真实坐标（待发拐点）
                //   run_len       = r_{j-1} 的段长（格）
                //   wk_prev_dir   = r_{j-1} 的方向
                //   emit_pending  = 1 表示 j >= 2（存在待发拐点，也就存在 r_{j-2}）
                // 输出：off_cur = o_{j-1}，随后发射 r_{j-2}。
                CR_SETUP: begin
                    corner_goal <= (wk_cell == goal_idx);
                    corner_x    <= wk_cell % GRID_W;
                    corner_y    <= wk_cell / GRID_W;
                    // 不测量本段的四种情况：居中关闭 / 目标格（末段不平移）/ 首段
                    // （r_0，起点格不参与平移）/ 段太短。
                    if ((CENTER_EN == 0) || (wk_cell == goal_idx) || !emit_pending ||
                        (run_len < L_MIN[15:0])) begin
                        meas_on    <= 1'b0;
                        off_cur    <= 6'sd0;
                        charge_cur <= 0;
                        state      <= CR_FILTER;
                    end else begin
                        meas_on <= 1'b1;
                        probe_k <= 6'd1;
                        d_plus <= D_UNBOUND[4:0];
                        d_minus <= D_UNBOUND[4:0];
                        // 等密度布点的第一个位置：p = PROBE_STEP（段太短时退化为中点）。
                        probe_p <= (PROBE_STEP[15:0] < run_len) ? PROBE_STEP[15:0] : (run_len >> 1);
                        state <= PR_SETUP;
                    end
                end

                // 把"沿段方向走 probe_p 格"的基点寄存下来（一个 16 bit 加减），
                // 后面的法向探针只需在基点上加 ±probe_k，地址路径短。
                PR_SETUP: begin
                    case (wk_prev_dir)
                        2'd0: begin
                            mid_x <= run_sx - probe_p;
                            mid_y <= run_sy;
                        end
                        2'd1: begin
                            mid_x <= run_sx + probe_p;
                            mid_y <= run_sy;
                        end
                        2'd2: begin
                            mid_x <= run_sx;
                            mid_y <= run_sy - probe_p;
                        end
                        default: begin
                            mid_x <= run_sx;
                            mid_y <= run_sy + probe_p;
                        end
                    endcase
                    state <= PR_ISSUE;
                end

                // 法向探针：k = 1..PROBE_D 走 + 侧，PROBE_D+1..2*PROBE_D 走 - 侧。
                // 探测深度决定"能看多远"：候选偏移最多 PROBE_D/2 格，所以 Z 形段可以
                // 一直挪到（可测到的）居中位置，而不是被固定 ±3 截断。
                // 地址越界由 nav_map 回答 blocked（rd_x/y 越界 ⇒ cost=1），正好把地图边界
                // 当作"无界"的终点。
                PR_ISSUE: begin
                    if (seg_horiz) begin
                        mrd_x <= mid_x;
                        if (probe_k <= PROBE_D[4:0]) mrd_y <= mid_y + probe_k;
                        else mrd_y <= mid_y - (probe_k - PROBE_D[4:0]);
                    end else begin
                        if (probe_k <= PROBE_D[4:0]) mrd_x <= mid_x + probe_k;
                        else mrd_x <= mid_x - (probe_k - PROBE_D[4:0]);
                        mrd_y <= mid_y;
                    end
                    mrd_valid <= 1'b1;
                    rd_ret    <= PR_TAKE;
                    state     <= RD_REQ;
                end

                // 通用读握手：置起请求后等授予；响应拍只锁存 cost（响应是按 owner 路由
                // 的单拍脉冲，同一拍再分支容易漏掉）。
                RD_REQ: begin
                    if (mrd_ready) begin
                        mrd_valid <= 1'b0;
                        state     <= RD_RSP;
                    end
                end

                RD_RSP: begin
                    if (mrd_rsp_valid) begin
                        rd_cost_q <= mrd_cost;
                        rd_raw_q  <= mrd_raw;
                        state     <= rd_ret;
                    end
                end

                PR_TAKE: begin
                    // 记录两侧第一处阻挡的距离；没有阻挡则保持 PROBE_D+1（该侧"无界"，
                    // 按最大宽度参与候选计算：未知区本来就按可通行处理）。
                    if (rd_cost_q) begin
                        if (probe_k <= PROBE_D[4:0]) begin
                            if (d_plus == D_UNBOUND[4:0]) d_plus <= probe_k;
                        end else begin
                            if (d_minus == D_UNBOUND[4:0]) d_minus <= probe_k - PROBE_D[4:0];
                        end
                    end
                    if (probe_k == PROBE_END[4:0]) begin
                        // 这个位置探完了：沿段方向等密度推进；越过后一半就结束。
                        // 位置 p = STEP, 2*STEP, ... < run_len（短段退化为中点，见 CR_SETUP）。
                        if ((probe_p + PROBE_STEP[15:0]) >= run_len) begin
                            state <= CR_CAND;
                        end else begin
                            probe_p <= probe_p + PROBE_STEP[15:0];
                            probe_k <= 6'd1;
                            state   <= PR_SETUP;
                        end
                    end else begin
                        probe_k <= probe_k + 5'd1;
                        state   <= PR_ISSUE;
                    end
                end

                // 候选偏移 + C/Z 分类：全部在 5~6 bit 上算，再寄存。
                CR_CAND: begin
                    cand_v <= cand_use ? {cand_c[4], cand_c[4:0]} : 6'sd0;
                    cls_s1 <= cls1_w;
                    cls_s2 <= cls2_w;
                    is_c   <= (cls1_w != cls2_w);
                    state  <= CR_CLAMP1;
                end

                // 长度参考与网格上下界：各一个 16 bit 减法，一拍寄存。
                CR_CLAMP1: begin
                    if (pseg_horiz) begin
                        cl_d  <= sdiff(last_x, run_sx);
                        cl_hi <= GRID_W - 1 - run_sx;
                        cl_lo <= -run_sx;
                    end else begin
                        cl_d  <= sdiff(last_y, run_sy);
                        cl_hi <= GRID_H - 1 - run_sy;
                        cl_lo <= -run_sy;
                    end
                    state <= CR_CLAMP2;
                end

                // 长度钳位：待发拐点的纵向坐标由 o_{j-1} 决定，钳位只保证上一段不反向、
                // 不塌缩（正向段至少前进 1 格，反向段至少后退 1 格），
                // 上限改由 OFF_MAX（位宽硬边界）和 C 形段的长度预算负责。
                CR_CLAMP2: begin
                    if (seg_dir == 2'd1 || seg_dir == 2'd3) begin
                        if (cand_v < (cl_d + 16'sd1)) begin
                            cl_raised <= 1'b1;
                            off_c1 <= ((cl_d + 16'sd1) > OFF_MAX) ? OFF_MAX[5:0] : (cl_d + 16'sd1);
                        end else begin
                            cl_raised <= 1'b0;
                            off_c1    <= cand_v;
                        end
                    end else begin
                        if (cand_v > (cl_d - 16'sd1)) begin
                            cl_raised <= 1'b1;
                            off_c1 <= ((cl_d - 16'sd1) < -OFF_MAX) ?
                                -OFF_MAX[5:0] : (cl_d - 16'sd1);
                        end else begin
                            cl_raised <= 1'b0;
                            off_c1    <= cand_v;
                        end
                    end
                    state <= CR_CLAMP3;
                end

                // 网格钳位：平移后的终点必须仍在网格内。
                CR_CLAMP3: begin
                    state <= CR_BUDGET;
                end

                // C 形段的长度预算（组合结果 off_bud 在这里寄存），Z 形段不受限。
                // 记账：C 形段 charge = ±2|o|，Z 形段 0；缩短方向为负，等于"退款"。
                CR_BUDGET: begin
                    off_cur    <= off_bud;
                    charge_cur <= charge_bud;
                    cum_inc    <= cum_inc + charge_bud;
                    state      <= CR_FILTER;
                end
                CR_FILTER: begin
                    check_phase <= 0;
                    state       <= CHECK_SETUP;
                end
                CHECK_SETUP: begin
                    if (line_sx < 0 || line_sy < 0 || line_ex < 0 || line_ey < 0 ||
                        line_sx >= GRID_W || line_ex >= GRID_W || line_sy >= GRID_H ||
                        line_ey >= GRID_H || (line_sx != line_ex && line_sy != line_ey))
                        reject_shift;
                    else begin
                        check_x  <= line_sx;
                        check_y  <= line_sy;
                        check_ex <= line_ex;
                        check_ey <= line_ey;
                        state    <= CHECK_ISSUE;
                    end
                end
                CHECK_ISSUE: begin
                    mrd_x     <= check_x[15:0];
                    mrd_y     <= check_y[15:0];
                    mrd_valid <= 1;
                    rd_ret    <= CHECK_TAKE;
                    state     <= RD_REQ;
                end
                CHECK_TAKE: begin
                    if (!cell_allowed(check_x[15:0], check_y[15:0], rd_raw_q, rd_cost_q))
                        reject_shift;
                    else if (check_x == check_ex && check_y == check_ey) begin
                        if (check_phase == 2) state <= CR_APPLY;
                        else begin
                            check_phase <= check_phase + 1'b1;
                            state       <= CHECK_SETUP;
                        end
                    end else begin
                        if (check_x != check_ex)
                            check_x <= check_x < check_ex ? check_x + 1'b1 : check_x - 1'b1;
                        else check_y <= check_y < check_ey ? check_y + 1'b1 : check_y - 1'b1;
                        state <= CHECK_ISSUE;
                    end
                end
                CR_APPLY: begin
                    if (emit_pending) begin
                        // 发射 r_{j-2}（终点 = 待发拐点 c_{j-1}）
                        state <= EM_SETUP;
                    end else if (corner_goal) begin
                        // 没有任何段可发（N = 1）：直接从起点发最后一段到目标格。
                        em_cx <= last_x;
                        em_cy <= last_y;
                        state <= GL_SETUP;
                    end else begin
                        // 首个拐点（j = 1）：没有 r_{-1} 可发，把 c_1 记为待发拐点。
                        run_sx       <= $signed(corner_x);
                        run_sy       <= $signed(corner_y);
                        run_len      <= 16'd0;
                        seg_dir      <= wk_prev_dir;
                        off_prev     <= off_cur;
                        emit_pending <= 1'b1;
                        wk_first     <= 1'b0;
                        state        <= WK_STEP;
                    end
                end

                // 发射 r_{j-2}：起点 = 已发点，终点 = 待发拐点 c_{j-1} 的最终坐标。
                EM_SETUP: begin
                    em_ex    <= em_px;
                    em_ey    <= em_py;
                    em_cx    <= last_x;
                    em_cy    <= last_y;
                    em_final <= 1'b0;
                    state    <= EM_REM;
                end

                // 由两个端点算出方向与剩余格数：单轴比较 + 单轴减法，一拍寄存。
                // 方向由实际坐标决定（平移可能让终点落在已发点后方，此时方向自然反过来，
                // 仍然保持轴对齐契约）。
                EM_REM: begin
                    if (em_ex != em_cx) begin
                        em_dir <= (em_ex > em_cx) ? 2'd1 : 2'd0;
                        em_rem <= (em_ex > em_cx) ? (em_ex - em_cx) : (em_cx - em_ex);
                    end else begin
                        em_dir <= (em_ey > em_cy) ? 2'd3 : 2'd2;
                        em_rem <= (em_ey > em_cy) ? (em_ey - em_cy) : (em_cy - em_ey);
                    end
                    state <= EM_SEND;
                end

                // 最后一段：终点必须是目标格本身（o_N = 0），所以不平移。
                GL_SETUP: begin
                    em_ex    <= goal_idx % GRID_W;
                    em_ey    <= goal_idx / GRID_W;
                    em_final <= 1'b1;
                    state    <= EM_REM;
                end

                EM_SEND: begin
                    if (!wp_valid) begin
                        if (em_rem > GAP[15:0]) begin
                            // 中继点：每 GAP 格一个，方向不变。
                            wp_valid <= 1'b1;
                            // 每次同时写两轴；不动的轴也必须赋值，保证首次发射没有未定义坐标。
                            case (em_dir)
                                2'd0: begin
                                    wp_x  <= em_cx - GAP[15:0];
                                    wp_y  <= em_cy;
                                    em_cx <= em_cx - GAP[15:0];
                                end
                                2'd1: begin
                                    wp_x  <= em_cx + GAP[15:0];
                                    wp_y  <= em_cy;
                                    em_cx <= em_cx + GAP[15:0];
                                end
                                2'd2: begin
                                    wp_y  <= em_cy - GAP[15:0];
                                    wp_x  <= em_cx;
                                    em_cy <= em_cy - GAP[15:0];
                                end
                                default: begin
                                    wp_y  <= em_cy + GAP[15:0];
                                    wp_x  <= em_cx;
                                    em_cy <= em_cy + GAP[15:0];
                                end
                            endcase
                            em_rem       <= em_rem - GAP[15:0];
                            em_point_end <= 1'b0;
                            term_goal    <= 1'b0;
                            wp_last      <= (wp_count >= wp_limit_q - 16'd1);
                        end else begin
                            // 本段终点。若它就是目标格则标记 wp_last；缓冲区写满时也
                            // 标记 wp_last 并发布一条可用前缀（否则 load_addr 会钉在
                            // MAX_WP 上，publish 永不触发）。
                            wp_valid     <= 1'b1;
                            wp_x         <= em_ex;
                            wp_y         <= em_ey;
                            em_cx        <= em_ex;
                            em_cy        <= em_ey;
                            em_rem       <= 16'd0;
                            em_point_end <= 1'b1;
                            term_goal    <= em_final;
                            wp_last      <= em_final || (wp_count >= wp_limit_q - 16'd1);
                        end
                        wp_count <= wp_count + 16'd1;
                    end else if (wp_ready) begin
                        wp_valid <= 1'b0;
                        wp_last  <= 1'b0;
                        if (wp_last) begin
                            if (term_goal) begin
                                success  <= 1'b1;
                                path_len <= wk_steps + 16'd1;
                            end else begin
                                // 这条路径是被缓冲区截断的，不是被地图截断的。已发布的
                            // 内容仍是已复核路线的可执行前缀（prefix），本轮算成功。
                                success  <= 1'b1;
                                path_len <= wk_steps;
                            end
                            state <= S_IDLE;
                        end else if (em_point_end) begin
                            last_x <= em_ex;
                            last_y <= em_ey;
                            if (corner_goal) begin
                                // 还要发最后一段（到目标格）。
                                em_cx <= em_ex;
                                em_cy <= em_ey;
                                state <= GL_SETUP;
                            end else begin
                                // r_{j-2} 发完：把 c_j（当前格）记为新的待发拐点。
                                run_sx       <= $signed(corner_x);
                                run_sy       <= $signed(corner_y);
                                run_len      <= 16'd0;
                                seg_dir      <= wk_prev_dir;
                                off_prev     <= off_cur;
                                emit_pending <= 1'b1;
                                wk_first     <= 1'b0;
                                state        <= WK_STEP;
                            end
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
