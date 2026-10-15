//////////////////////////////////////////////////////////////////////////////////
// 模块：quad_decoder —— 增量式正交编码器解码（4 倍频）
//
// 【它做什么】
//   把编码器的 A/B 两相方波翻译成「位置计数 + 方向」。四路电机就例化 4 次。
//
// 【为什么是 4 倍频】
//   正交编码的 A/B 相位差 90°，一个机械周期内 {A,B} 会经历 4 个状态：
//       正转：00 → 10 → 11 → 01 → 00
//       反转：00 → 01 → 11 → 10 → 00
//   本模块【每个状态跳变都计一次数】，所以分辨率是编码器标称线数的 4 倍。
//   例如 13 线（PPR）的霍尔编码器，电机转一圈产生 13 × 4 = 52 个计数。
//
// 【方向为什么能判出来】
//   看相邻两个状态是"顺时针走"还是"逆时针走"：
//       00→10、10→11、11→01、01→00  → 递增（A 领先 B）
//       00→01、01→11、11→10、10→00  → 递减（B 领先 A）
//   若出现【两位同时变化】（如 00→11、11→00），说明丢了一步或受了干扰，
//   本模块【不计数】——宁可漏一个计数，也不要把方向判反。
//
// 【★ 抗干扰：三级处理】
//   电机就在编码器旁边，换向毛刺是常态。本模块做三级防护：
//     1) 两级同步器     —— 异步输入先过 2 级触发器，防亚稳态
//     2) 数字去毛刺     —— 电平必须连续保持 2^FILTER_BITS 拍才被认可
//                          （毛刺通常只有几个时钟宽，会被完全滤掉）
//     3) 非法跳变忽略   —— 见上，双位跳变不计数
//   去毛刺会限制最高可测频率：2^FILTER_BITS 拍 = 2^3 = 8 拍（@50MHz 即 160ns），
//   要求信号半周期 > 160ns → 最高约 3.1MHz，远高于任何编码器的输出频率
//   （500 线、四倍频、3000rpm 也只有 100kHz），所以完全够用。
//
//   注意：不要用 RC 硬件滤波代替它——RC 会把边沿拖慢，反而更容易误判；
//   数字滤波在 FPGA 里是零成本的。
//
// 参数：
//   CLK_FREQ_HZ   系统时钟频率(Hz)          默认 50_000_000
//   COUNT_BITS    位置计数位宽              默认 32（有符号，可正可负）
//   FILTER_BITS   去毛刺长度取 2 的幂的指数  默认 3 → 需连续 8 拍一致
//                 （最小 1；设 0 会被当成 1 处理，即 2 拍确认）
//   INV_DIR    1 = 交换计数方向          默认 0
//                 （实际接线里"A/B 谁领先"取决于编码器与电机的相对极性，
//                   如果发现转向与计数符号相反，改这个参数或对调 A/B 接线）
//
// 端口：
//   i_clock      输入  系统时钟
//   i_reset_n    输入  异步低电平复位
//   i_enc_a      输入  编码器 A 相（异步信号）
//   i_enc_b      输入  编码器 B 相（异步信号）
//   i_clear      输入  位置清零（单周期脉冲，须由上层同步去抖后给出）
//   o_count      输出  位置计数，有符号；转一圈的计数 = PPR × 4 × 减速比
//   o_dir        输出  最近一次的运动方向：1 = 计数递增，0 = 计数递减
//   o_step       输出  本拍发生了计数（调试/诊断用，可用它抓丢步）
//////////////////////////////////////////////////////////////////////////////////

module quad_decoder #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer COUNT_BITS  = 32,
    parameter integer FILTER_BITS = 3,
    parameter integer INV_DIR  = 0
)(
    input  wire                  i_clock,
    input  wire                  i_reset_n,

    input  wire                  i_enc_a,
    input  wire                  i_enc_b,
    input  wire                  i_clear,

    output wire signed [COUNT_BITS-1:0] o_count,
    output wire                  o_dir,
    output wire                  o_step
);

    // FILTER_BITS 下限保护：位宽不能为 0
    localparam integer FB = (FILTER_BITS < 1) ? 1 : FILTER_BITS;
    localparam [FB-1:0] FILTER_MAX = {FB{1'b1}};

    //---------------------------------------------------------------
    // 1. 两级同步器：异步输入 → 时钟域
    //    （编码器信号与本时钟无关，直接使用会引入亚稳态）
    //---------------------------------------------------------------
    reg [1:0] a_sync, b_sync;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            a_sync <= 2'b00;
            b_sync <= 2'b00;
        end else begin
            a_sync <= {a_sync[0], i_enc_a};
            b_sync <= {b_sync[0], i_enc_b};
        end
    end

    //---------------------------------------------------------------
    // 2. 数字去毛刺：电平连续保持 2^FILTER_BITS 拍才被认可
    //    计数器在"输入与当前认可值相同"时清零，于是短于阈值的抖动
    //    永远攒不满计数，被彻底滤掉。
    //---------------------------------------------------------------
    reg [FB-1:0] a_fcnt, b_fcnt;
    reg          a_filt, b_filt;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            a_fcnt <= {FB{1'b0}};
            a_filt <= 1'b0;
        end else if (a_sync[1] == a_filt) begin
            a_fcnt <= {FB{1'b0}};
        end else if (a_fcnt == FILTER_MAX) begin
            a_filt <= a_sync[1];
            a_fcnt <= {FB{1'b0}};
        end else begin
            a_fcnt <= a_fcnt + 1'b1;
        end
    end

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            b_fcnt <= {FB{1'b0}};
            b_filt <= 1'b0;
        end else if (b_sync[1] == b_filt) begin
            b_fcnt <= {FB{1'b0}};
        end else if (b_fcnt == FILTER_MAX) begin
            b_filt <= b_sync[1];
            b_fcnt <= {FB{1'b0}};
        end else begin
            b_fcnt <= b_fcnt + 1'b1;
        end
    end

    //---------------------------------------------------------------
    // 3. 正交解码：看 {前一状态, 当前状态} 判断走了一步还是退了一步
    //---------------------------------------------------------------
    wire [1:0] curr_state = {a_filt, b_filt};

    reg  [1:0] prev_state;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) prev_state <= 2'b00;
        else            prev_state <= curr_state;
    end

    wire [3:0] trans = {prev_state, curr_state};

    // 单步转移（按 A 领先 B 定义递增）
    wire step_up_raw = (trans == 4'b00_10) | (trans == 4'b10_11) |
                       (trans == 4'b11_01) | (trans == 4'b01_00);
    wire step_dn_raw = (trans == 4'b00_01) | (trans == 4'b01_11) |
                       (trans == 4'b11_10) | (trans == 4'b10_00);
    // 双位跳变（00→11 / 11→00）落入上述之外 → 不计数，方向不会被判反

    wire step_up = INV_DIR ? step_dn_raw : step_up_raw;
    wire step_dn = INV_DIR ? step_up_raw : step_dn_raw;

    //---------------------------------------------------------------
    // 4. 位置计数（有符号，可正可负）
    //---------------------------------------------------------------
    reg signed [COUNT_BITS-1:0] count_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)          count_r <= {COUNT_BITS{1'b0}};
        else if (i_clear)        count_r <= {COUNT_BITS{1'b0}};   // 清零优先
        else if (step_up)        count_r <= count_r + 1'b1;
        else if (step_dn)        count_r <= count_r - 1'b1;
    end

    //---------------------------------------------------------------
    // 5. 方向锁存（只在真正走步时更新，静止时保持最后一次的方向）
    //---------------------------------------------------------------
    reg dir_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)   dir_r <= 1'b0;
        else if (step_up) dir_r <= 1'b1;      // 递增
        else if (step_dn) dir_r <= 1'b0;      // 递减
    end

    assign o_count = count_r;
    assign o_dir   = dir_r;
    assign o_step  = step_up | step_dn;

endmodule
