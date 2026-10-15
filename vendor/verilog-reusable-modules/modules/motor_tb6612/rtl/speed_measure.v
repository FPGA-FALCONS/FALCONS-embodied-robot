//////////////////////////////////////////////////////////////////////////////////
// 模块：speed_measure —— 定窗口测速（M 法）
//
// 【它做什么】
//   每 WINDOW_MS 毫秒采样一次 quad_decoder 的位置计数，
//   输出「本窗口内的计数增量」。正数表示递增方向，负数表示反向。
//
// 【为什么先输出"计数增量"而不是直接输出 rpm】
//   增量 → 物理转速需要三个只有你知道的参数（编码器线数、倍频、减速比）。
//   把换算放在上层，模块才能通用于任何轮子。换算公式：
//
//       转速 n(rpm) = Δcount × 60000 / ( COUNT_PER_REV × WINDOW_MS )
//
//       Δcount         : 本模块输出的 o_delta
//       COUNT_PER_REV  : 电机转一圈的计数 = PPR × 4 × 减速比
//                        （PPR = 编码器标称线数；×4 是 quad_decoder 的 4 倍频）
//       WINDOW_MS      : 本模块的测速窗口（默认 10ms）
//
//   举例：13 线编码器 + 减速比 30 → COUNT_PER_REV = 13×4×30 = 1560
//         若 20ms 窗口内读到 Δcount = 520，则
//         n = 520 × 60000 / (1560 × 20) = 1000 rpm
//
// 【M 法的分辨率与局限】
//   一个窗口内至少要有一个计数才动得起来：
//       Δcount = 1（20ms 窗口，1560 计数/转）
//       → 0.032 转/s → 1.9 rpm（这是可分辨的下限）
//   低速时量化误差占比变大（±1 个计数就是 ±1.9rpm）。
//   如果需要更平滑的低速测速，可改用 M/T 法（同时测脉冲数和时间），
//   或把 WINDOW_MS 调大（代价是速度更新变慢）。
//
// 【清零处理】★ 容易踩的坑
//   上层若清零编码器位置（i_clear），而本模块还留着旧的基准值，
//   下一个窗口就会算出一个巨大的假增量。所以本模块同样接收 i_clear，
//   清零时把基准复位并重新开始计时（该窗口不产生有效数据）。
//
// 参数：
//   CLK_FREQ_HZ   系统时钟频率(Hz)   默认 50_000_000
//   WINDOW_MS     测速窗口(ms)       默认 10（即 100Hz 速度更新率）
//   COUNT_BITS    位置计数位宽       默认 32（须与 quad_decoder 一致）
//
// 端口：
//   i_clock        输入  系统时钟
//   i_reset_n      输入  异步低电平复位
//   i_count        输入  来自 quad_decoder 的位置计数（有符号）
//   i_clear        输入  与 quad_decoder 同一个清零信号（复位基准）
//   o_delta        输出  本窗口计数增量（有符号）
//   o_delta_valid  输出  单周期脉冲：o_delta 已更新，可以采样
//////////////////////////////////////////////////////////////////////////////////

module speed_measure #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer WINDOW_MS   = 10,
    parameter integer COUNT_BITS  = 32
)(
    input  wire                         i_clock,
    input  wire                         i_reset_n,

    input  wire signed [COUNT_BITS-1:0] i_count,
    input  wire                         i_clear,

    output wire signed [COUNT_BITS-1:0] o_delta,
    output wire                         o_delta_valid
);

    // 窗口长度（时钟数）。先除后乘，避免 CLK × WINDOW_MS 在 32 位里溢出
    localparam integer WINDOW_DIV_RAW = (CLK_FREQ_HZ / 1000) * WINDOW_MS;
    localparam integer WINDOW_TICKS   = (WINDOW_DIV_RAW < 1) ? 1 : WINDOW_DIV_RAW;

    // ★ 加载值必须减 1（差一错误的经典陷阱）
    //   计数器是"减到 0 的那一拍产生 tick 并重新加载"：若直接加载
    //   WINDOW_TICKS，则从加载到下一次归零要经过 WINDOW_TICKS + 1 拍，
    //   实际窗口会比设定值多一个时钟。
    //   yaw_integrate 的 INT_DIV 也是同样的处理（那里写的是 (CLK/RATE)-1）。
    localparam integer WINDOW_LOAD    = WINDOW_TICKS - 1;

    reg  [31:0] win_cnt;
    wire        win_tick = (win_cnt == 32'd0);

    reg  signed [COUNT_BITS-1:0] count_prev;
    reg  signed [COUNT_BITS-1:0] delta_r;
    reg                          valid_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            win_cnt    <= WINDOW_LOAD[31:0];
            count_prev <= {COUNT_BITS{1'b0}};
            delta_r    <= {COUNT_BITS{1'b0}};
            valid_r    <= 1'b0;
        end else begin
            valid_r <= 1'b0;                   // 默认单周期脉冲

            if (i_clear) begin
                // 位置基准复位并重新计时；本窗口不产生有效数据
                win_cnt    <= WINDOW_LOAD[31:0];
                count_prev <= {COUNT_BITS{1'b0}};
                delta_r    <= {COUNT_BITS{1'b0}};
            end else if (win_tick) begin
                // 窗口结束：算出增量，并把基准更新到当前位置
                win_cnt    <= WINDOW_LOAD[31:0];
                delta_r    <= i_count - count_prev;
                count_prev <= i_count;
                valid_r    <= 1'b1;
            end else begin
                win_cnt <= win_cnt - 32'd1;
            end
        end
    end

    assign o_delta       = delta_r;
    assign o_delta_valid = valid_r;

endmodule
