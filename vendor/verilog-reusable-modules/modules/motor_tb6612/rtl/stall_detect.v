//////////////////////////////////////////////////////////////////////////////////
// 模块：stall_detect —— 电机堵转检测与保护（基于【净位移】，锁存停机）
//
// 【为什么必须有这个模块 —— 本项目是硬需求】
//   MG513P30_12V 的【堵转电流 3.2A】正好等于 TB6612FNG 的【峰值电流 3.2A】。
//   任何一个轮子被卡住（顶墙、卡台阶、被手按住），该通道立刻冲到芯片的峰值
//   极限：TB6612 会在 65℃ 过温保护下自己关断（表现为"转一会儿就停"），
//   长时间堵转则会损坏。而四个轮子同时堵转更远超其能力。
//   本模块用【编码器反馈】在几百毫秒内识别堵转并自动切断该路使能。
//
//============================================================================
// 【★ 判据为什么用"净位移"而不是"转速为 0"或"单窗口阈值"】
//
//   本系统是 60000 计数/转，于是 1° 机械角 = 167 个计数。
//
//   关键事实：**堵转时电机并不是完全静止的**。通电却转不动时会有轻微抖动、
//   弹跳，哪怕只有 ±1° 的振动，单窗口 delta 就会达到 ±167。
//   而正常低速转动（例如 10% 占空比、约 15rpm）单窗口 delta 也只有约 150。
//   两者【在同一量级】，所以：
//
//     * 阈值取小 → 堵转振动被判成"在动" → 保护永不触发（漏报，最危险）
//     * 阈值取大 → 正常低速转动被误判为堵转 → 好好的车被停机（误报）
//
//   单窗口判据无解。但"来回"和"单向"有本质区别：
//
//       堵转振动： +167 -167 +167 -167 ...  累加净位移 ≈ 0     （原地抖）
//       正常转动： +150 +150 +150 +150 ...  累加净位移 = 600   （单向走）
//
//   所以本模块【累加 delta 得到净位移】，用净位移判据：
//
//       在 STALL_WINDOWS 个窗口内，|净位移| 始终达不到 MOVE_TH → 判定堵转
//
//   振动自动抵消、转动持续累积，漏报与误报同时避开。
//============================================================================
//
// 【另外一个必要条件：驱动意图】
//   堵转 = "我给了功率，但轮子不走"。所以还要 i_enable = 1 且 i_duty >= DUTY_TH。
//   占空比很低时轮子本来就可能因静摩擦不转，那不算堵转。
//
// 【启动宽限期（STARTUP_WINDOWS）】
//   使能瞬间电机还没转起来，头几个窗口净位移必然很小。若不设宽限期，
//   一启动就会误判。默认 30 个窗口（10ms/窗口 → 300ms），足够覆盖带载启动；
//   期间一旦净位移达到 MOVE_TH（转起来了），立刻提前进入正常监测。
//
// 【为什么锁存停机，而不是自动重试】
//   堵转通常意味着"车被卡住了"，继续给功率只会持续加热芯片；自动重试会在
//   卡住状态下反复冲击峰值电流，风险更大。所以堵转【锁存】，直到上层明确给
//   i_stall_clear 才恢复。典型用法：把车挪开、按一下"停止"键即清除，再启动。
//
// 参数：
//   STARTUP_WINDOWS  使能后忽略的窗口数（默认 30 → 300ms @10ms 窗口）
//   STALL_WINDOWS    判定窗口数（默认 20 → 200ms）
//   DUTY_TH          占空比阈值，低于此值不判定（默认 200 ≈ 10% of 2048）
//   MOVE_TH          ★ 净位移阈值（计数）。默认 300 ≈ 1.8° 机械角。
//                    应满足：明显大于堵转时的振动幅值，明显小于正常最低转速
//                    在 STALL_WINDOWS 内的位移。
//                    本系统标定值：60000 计数/转
//                        1° = 167 计数；正常 10% 占空比 200ms 内约 3000 计数
//                        所以 300 这个阈值两侧都有 10 倍以上余量
//
//   ★ 这里以"测速窗口"为单位，一个窗口 = speed_measure 的 WINDOW_MS 毫秒。
//     若把 WINDOW_MS 改成 20，则 20 个窗口变成 400ms，需同步调整本参数。
//
// 端口：
//   i_clock        输入  系统时钟
//   i_reset_n      输入  异步低电平复位
//   i_enable       输入  上层请求的使能
//   i_duty         输入  当前占空比（16 位，0 ~ 满值）
//   i_delta        输入  本窗口的编码器增量（来自 speed_measure，有符号）
//   i_delta_valid  输入  测速窗口结束脉冲（本模块的状态机节拍）
//   i_stall_clear  输入  清除堵转锁存（单周期脉冲）
//   o_stall        输出  堵转标志（锁存，直到 i_stall_clear）
//   o_enable_out   输出  实际使能输出 = i_enable & ~o_stall
//////////////////////////////////////////////////////////////////////////////////

module stall_detect #(
    parameter integer STARTUP_WINDOWS = 30,
    parameter integer STALL_WINDOWS   = 20,
    parameter integer DUTY_TH         = 200,
    parameter integer MOVE_TH         = 300
)(
    input  wire               i_clock,
    input  wire               i_reset_n,

    input  wire               i_enable,
    input  wire [15:0]        i_duty,
    input  wire signed [31:0] i_delta,
    input  wire               i_delta_valid,
    input  wire               i_stall_clear,

    output wire               o_stall,
    output wire               o_enable_out
);

    //---------------------------------------------------------------
    // 状态编码
    //---------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0;   // 未使能 / 无驱动意图
    localparam [1:0] S_START = 2'd1;   // 启动宽限期
    localparam [1:0] S_MON   = 2'd2;   // 正常监测
    localparam [1:0] S_STALL = 2'd3;   // 已判堵转（锁存）

    localparam [15:0] START_LOAD = STARTUP_WINDOWS - 1;
    localparam [15:0] STALL_LAST = STALL_WINDOWS   - 1;   // 判定的最后一个窗口

    localparam signed [31:0] MTH = MOVE_TH;

    //---------------------------------------------------------------
    // 状态与累加器
    //   ★ 寄存器必须写在前面：下面的 accum_next 会引用 pos_accum。
    //     若把 reg 声明放在使用之后，Verilog 会把它当作 1 位隐式 wire，
    //     综合出完全错误的逻辑（且仿真行为诡异难查）。
    //---------------------------------------------------------------
    reg  [1:0]         state;
    reg  [15:0]        win_cnt;      // 启动宽限计数（递减）
    reg  [15:0]        still_win;    // 本判定周期内"未移动"的窗口数
    reg  signed [31:0] pos_accum;    // 净位移累加器

    //---------------------------------------------------------------
    // 组合判据
    //---------------------------------------------------------------
    wire driving = (i_duty >= DUTY_TH[15:0]);        // 有驱动意图

    // ★ 净位移累加器（跨窗口累加，振动会互相抵消）
    wire signed [31:0] accum_next = pos_accum + i_delta;
    wire               moved      = (accum_next >= MTH) || (accum_next <= -MTH);

    //---------------------------------------------------------------
    // 状态机：节拍 = 测速窗口结束脉冲（默认 100Hz）
    //---------------------------------------------------------------
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state     <= S_IDLE;
            win_cnt   <= 16'd0;
            still_win <= 16'd0;
            pos_accum <= 32'sd0;
        end else if (i_stall_clear) begin
            // 清除优先：回到未使能态，重新开始
            state     <= S_IDLE;
            win_cnt   <= 16'd0;
            still_win <= 16'd0;
            pos_accum <= 32'sd0;
        end else if (i_delta_valid) begin
            case (state)
                //-----------------------------------------------
                S_IDLE: begin
                    win_cnt   <= START_LOAD;
                    still_win <= 16'd0;
                    pos_accum <= 32'sd0;
                    if (i_enable && driving) state <= S_START;
                end
                //-----------------------------------------------
                // 启动宽限期：不计入判定，但一旦净位移够大就提前转正
                S_START: begin
                    if (!i_enable || !driving) begin
                        state     <= S_IDLE;
                        win_cnt   <= 16'd0;
                        pos_accum <= 32'sd0;
                    end else if (moved) begin
                        state     <= S_MON;        // 已经转起来了
                        still_win <= 16'd0;
                        pos_accum <= 32'sd0;
                    end else if (win_cnt == 16'd0) begin
                        state     <= S_MON;        // 宽限期结束，开始判定
                        still_win <= 16'd0;
                        pos_accum <= 32'sd0;
                    end else begin
                        pos_accum <= accum_next;
                        win_cnt   <= win_cnt - 16'd1;
                    end
                end
                //-----------------------------------------------
                // 正常监测：净位移够大 → 判定周期重置；连续 N 窗口不够 → 堵转
                S_MON: begin
                    if (!i_enable || !driving) begin
                        state     <= S_IDLE;
                        still_win <= 16'd0;
                        pos_accum <= 32'sd0;
                    end else if (moved) begin
                        // 净位移达标 → 确实在走，重新开始一个新的判定周期
                        still_win <= 16'd0;
                        pos_accum <= 32'sd0;
                    end else if (still_win == STALL_LAST) begin
                        state <= S_STALL;          // 连续 N 窗口净位移不足 → 堵转
                    end else begin
                        still_win <= still_win + 16'd1;
                        pos_accum <= accum_next;
                    end
                end
                //-----------------------------------------------
                // 已判堵转：锁存，只等 i_stall_clear
                S_STALL: begin
                    state <= S_STALL;
                end
                //-----------------------------------------------
                default: state <= S_IDLE;
            endcase
        end
    end

    //---------------------------------------------------------------
    // 输出
    //   堵转时切断使能（motor_pwm 收到 enable=0 会立即停止输出）
    //---------------------------------------------------------------
    assign o_stall      = (state == S_STALL);
    assign o_enable_out = i_enable & ~o_stall;

endmodule
