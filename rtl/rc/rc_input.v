//////////////////////////////////////////////////////////////////////////////////
// 模块：rc_input —— 遥控接收机多路 PWM 输入（单路捕获模块多路实例化）
//
// 【它做什么】
//   把 N 路遥控接收机的 PWM 脉宽（1000~2000 µs）翻译成工程单位：
//     * 两路模拟量：转向 / 油门（signed 12 位，−1000…+1000，中位 0，带死区）
//     * 一路三档开关：模式（手动接管 / 屏幕光标 / 自动）
//     * 三路按钮：选择(select) / 确认 / 返回 —— 按下产生单周期脉冲
//   并给出"信号是否有效"（任一路超时即视为丢信号）。
//
// 【为什么用库里的单路捕获 + 多路实例化】
//   库里 `rc_pwm_capture` 是**单路**模块（CLK_HZ / MIN_PULSE_US / MAX_PULSE_US /
//   TIMEOUT_US 四个参数），可以独立实例化任意路数；`rc_receiver` 只是把两路
//   包成 steering/throttle。本车需要 6 路（2 摇杆 + 模式 + select + 确认 + 返回），
//   所以直接例化 6 个 `rc_pwm_capture`，并把映射做成参数（换接线只改参数）。
//
// 【单位与定点】
//   归一化：|us − 1500| ≤ DEADZONE_US → 0；否则线性映射到 ±1000
//     标度 = 1000 / (HALF_RANGE_US − DEADZONE_US)，以 Q10 常量在综合期算好
//     （零除法、零浮点）
//
// 【三档判定】
//   us < SW_LO_US          → 位置 0（低位）
//   SW_LO_US…SW_HI_US      → 位置 1（中位）
//   us > SW_HI_US          → 位置 2（高位）
//   "按下"定义为"位置 2"的上升沿 → 单周期脉冲；开关拨到高位同样算一次触发。
//
// 【丢信号】
//   任一路 timed_out 拉高 → o_signal_valid = 0。上层的安全策略是
//   200 ms 内停车（依据功能设计文档 §9 L4 判据），所以 TIMEOUT_US 取 100 ms，
//   留出一半余量。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module rc_input #(
    parameter integer CLK_FREQ_HZ   = 50_000_000,
    parameter integer NUM_CH        = 6,          // 实际使用的通道数
    parameter integer CENTER_US     = 1500,       // 摇杆中位脉宽
    parameter integer HALF_RANGE_US = 500,        // 中位到端点（1000/2000）
    parameter integer DEADZONE_US   = 25,         // 中位死区（±25 µs）
    parameter integer MIN_PULSE_US  = 900,        // 合法脉宽下限（库默认）
    parameter integer MAX_PULSE_US  = 2100,       // 合法脉宽上限（库默认）
    parameter integer TIMEOUT_US    = 100_000,    // 单路超时（100 ms）
    parameter integer SW_LO_US      = 1300,       // 三档：低位阈值
    parameter integer SW_HI_US      = 1700,       // 三档：高位阈值
    // ---- 通道映射（按实际接线改这里，不要改逻辑）----
    parameter integer CH_STEER      = 0,
    parameter integer CH_THROTTLE   = 1,
    parameter integer CH_CONFIRM    = 2,
    parameter integer CH_BACK       = 3,
    parameter integer CH_MODE       = 4,
    parameter integer CH_SELECT     = 5
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [NUM_CH-1:0]       rc_pwm,          // 各通道 PWM 输入（异步）

    output reg  signed [11:0]      o_steer,         // −1000…+1000（右为正）
    output reg  signed [11:0]      o_throttle,      // −1000…+1000（前为正）
    output reg  [1:0]              o_mode,          // 0=手动接管 1=屏幕光标 2=自动
    output reg                     o_btn_select,    // 单周期脉冲
    output reg                     o_btn_confirm,
    output reg                     o_btn_back,

    output wire                    o_signal_valid,  // 1 = 全部通道都在刷新
    output wire [NUM_CH*16-1:0]    o_chan_us,       // 各路脉宽（调试/显示）
    output wire [NUM_CH-1:0]       o_chan_timeout,
    output wire [NUM_CH-1:0]       o_chan_valid
);

    // 标度：1000 / (HALF_RANGE_US − DEADZONE_US)，Q10
    localparam integer NORM_SPAN = (HALF_RANGE_US > DEADZONE_US) ? (HALF_RANGE_US - DEADZONE_US) : 1;
    localparam integer SCALE_Q10 = (1000 * 1024) / NORM_SPAN;

    // ------------------------------------------------------------------
    // 单路捕获 × NUM_CH（库里 rc_pwm_capture 无 i_/o_ 前缀，按名连接）
    // ------------------------------------------------------------------
    wire [15:0] chan_us   [0:NUM_CH-1];
    wire        chan_upd  [0:NUM_CH-1];
    wire        chan_vld  [0:NUM_CH-1];
    wire        chan_to   [0:NUM_CH-1];
    wire [NUM_CH*16-1:0] flat_us;   // ★ 必须 NUM_CH×16 位；写成 [15:0] 时 ModelSim 容忍、Gowin 报 EX3784 越界
    wire [NUM_CH-1:0] flat_to;
    wire [NUM_CH-1:0] flat_vld;

    genvar gi;
    generate
        for (gi = 0; gi < NUM_CH; gi = gi + 1) begin : g_cap
            rc_pwm_capture #(
                .CLK_HZ       (CLK_FREQ_HZ),
                .MIN_PULSE_US (MIN_PULSE_US),
                .MAX_PULSE_US (MAX_PULSE_US),
                .TIMEOUT_US   (TIMEOUT_US)
            ) u_cap (
                .clk             (clk),
                .rst_n           (rst_n),
                .rc_pwm          (rc_pwm[gi]),
                .pulse_width_us  (chan_us[gi]),
                .pulse_valid     (chan_vld[gi]),
                .timed_out       (chan_to[gi]),
                .pulse_update    (chan_upd[gi])
            );
            assign flat_us[gi*16 +: 16]   = chan_us[gi];
            assign flat_vld[gi]           = chan_vld[gi];
            assign flat_to[gi]            = chan_to[gi];
        end
    endgenerate

    assign o_chan_us      = flat_us;
    assign o_chan_valid   = flat_vld;
    assign o_chan_timeout = flat_to;
    // 全部通道都没超时才算"信号有效"
    assign o_signal_valid = ~(|flat_to);

    // ------------------------------------------------------------------
    // 取某通道脉宽（参数索引，综合期常量）
    // ------------------------------------------------------------------
    wire [15:0] us_steer    = flat_us[CH_STEER    * 16 +: 16];
    wire [15:0] us_throttle = flat_us[CH_THROTTLE * 16 +: 16];
    wire [15:0] us_confirm  = flat_us[CH_CONFIRM  * 16 +: 16];
    wire [15:0] us_back     = flat_us[CH_BACK     * 16 +: 16];
    wire [15:0] us_mode     = flat_us[CH_MODE     * 16 +: 16];
    wire [15:0] us_select   = flat_us[CH_SELECT   * 16 +: 16];

    // ------------------------------------------------------------------
    // 归一化：脉宽 → ±1000（带死区，零除法）
    // ------------------------------------------------------------------
    // 归一化：脉宽 → ±1000（带死区）
    // ★ 用【移位加法】而不是通用乘法：1000/(HALF_RANGE_US−DEADZONE_US) ≈ 2.105，
    //   取 2 + 1/8 = 2.125（误差 0.95%）→ 只需两级加法，既省 DSP 又把组合路径缩短。
    //   教训：这里原来写 (delta*SCALE_Q10)>>>10，综合后 setup slack = −7.02 ns
    //   （关键路径 u_rc/o_steer → chassis 乘法链），换成移位加法后满足时序。
    function signed [11:0] norm_cmd;
        input [15:0] us;
        input        valid;
        integer d;
        integer m;
        integer scaled;
        begin
            if (!valid) begin
                norm_cmd = 12'sd0;
            end else begin
                d = us - CENTER_US;
                if (d > DEADZONE_US) begin
                    m = (d - DEADZONE_US);
                end else if (d < -DEADZONE_US) begin
                    m = -((-d) - DEADZONE_US);
                end else begin
                    m = 0;
                end
                scaled = (m << 1) + (m >>> 3);      // ×2.125
                if (scaled >  1000) scaled =  1000;
                if (scaled < -1000) scaled = -1000;
                norm_cmd = scaled[11:0];
            end
        end
    endfunction

    wire signed [11:0] steer_now    = norm_cmd(us_steer,    flat_vld[CH_STEER]    & ~flat_to[CH_STEER]);
    wire signed [11:0] throttle_now = norm_cmd(us_throttle, flat_vld[CH_THROTTLE] & ~flat_to[CH_THROTTLE]);

    // ------------------------------------------------------------------
    // 三档判定 + 按钮上升沿
    // ------------------------------------------------------------------
    function [1:0] sw_pos;
        input [15:0] us;
        input        valid;
        begin
            if (!valid)               sw_pos = 2'd1;      // 无效时按中位处理
            else if (us > SW_HI_US)   sw_pos = 2'd2;
            else if (us < SW_LO_US)   sw_pos = 2'd0;
            else                      sw_pos = 2'd1;
        end
    endfunction

    wire [1:0] pos_mode    = sw_pos(us_mode,    flat_vld[CH_MODE]    & ~flat_to[CH_MODE]);
    wire [1:0] pos_select  = sw_pos(us_select,  flat_vld[CH_SELECT]  & ~flat_to[CH_SELECT]);
    wire [1:0] pos_confirm = sw_pos(us_confirm, flat_vld[CH_CONFIRM] & ~flat_to[CH_CONFIRM]);
    wire [1:0] pos_back    = sw_pos(us_back,    flat_vld[CH_BACK]    & ~flat_to[CH_BACK]);

    // 位置 2 的上升沿 = 一次"按下"
    function btn_edge;
        input [1:0] now;
        input [1:0] prev;
        begin
            btn_edge = (now == 2'd2) && (prev != 2'd2);
        end
    endfunction

    reg [1:0] prev_select, prev_confirm, prev_back;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_steer       <= 12'sd0;
            o_throttle    <= 12'sd0;
            o_mode        <= 2'd0;
            o_btn_select  <= 1'b0;
            o_btn_confirm <= 1'b0;
            o_btn_back    <= 1'b0;
            prev_select   <= 2'd1;
            prev_confirm  <= 2'd1;
            prev_back     <= 2'd1;
        end else begin
            o_steer    <= steer_now;
            o_throttle <= throttle_now;

            // 模式开关三档 → 0 手动 / 1 屏幕光标 / 2 自动
            if (pos_mode == 2'd2)      o_mode <= 2'd2;
            else if (pos_mode == 2'd0) o_mode <= 2'd0;
            else                       o_mode <= 2'd1;

            o_btn_select  <= btn_edge(pos_select,  prev_select);
            o_btn_confirm <= btn_edge(pos_confirm, prev_confirm);
            o_btn_back    <= btn_edge(pos_back,    prev_back);

            prev_select   <= pos_select;
            prev_confirm  <= pos_confirm;
            prev_back     <= pos_back;
        end
    end

endmodule
