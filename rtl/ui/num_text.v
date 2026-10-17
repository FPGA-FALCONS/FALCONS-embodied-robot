//////////////////////////////////////////////////////////////////////////////////
// 模块：num_text —— 把 9 个有符号整数格式化成定宽字符串，写进 dyn_text_ram
//
// 【格式】每个数值固定 4 个字符：`符号 + 百位 + 十位 + 个位`，例如
//      0    → "   0"（索引序列：空格,空格,空格,'0'）
//      123  → " 123"
//      -45  → "- 45"   —— 注：百位为 0 时写空格，所以是 "-  45"（等宽右对齐）
//   9 个数值依次放在地址 0..35（每个占 4 字节；共 36 字符，字符 RAM 深 64）。
//   显示范围【饱和到 ±999】：四轮转速 ±400、航向角因 yaw_integrate 在 ±180° 回绕
//   也不会超；角速度（字段 5）量纲待上板校准，若量级超 999 会看到数值被钉在 999。
//
// 【为什么用慢速状态机而不是组合除法】
//   50 MHz 下一拍一次比较/减法即可，刷新周期是 100 ms（远大于 24 个字符的生成时间），
//   组合除法会拉出很长的组合路径（本工程已因组合乘法吃过 setup −7 ns 的亏）。
//
// 【数值来源（顶层接线，2026-10-16 按需求原文 §29 的数据页重排）】
//   ★ 口径裁定（需求方 2026-10-16 明确）：车轮旁显示的是【车轮转速】，
//     不是编码器累计读数。所以字段宽度（符号 + 3 位数字）保持不变。
//   v0    = 航向角（整数度）
//   v1..v4 = 四路**车轮转速**（rpm）A 左前 / B 右前 / C 左后 / D 右后
//   v5    = 角速度（整数 °/s，量纲待上板校准）
//   v6    = 中心速度（四轮平均 rpm）
//   v7/v8 = 航位 x / y（odom 未实现 → 暂恒 0）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module num_text #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer REFRESH_MS = 100,      // 刷新周期
    parameter integer FIELD_W    = 4,        // 每字段字符数（符号 + 3 位数字，±999）
    // ★ 2026-10-16：7 → 9，按需求原文 §29 的数据页布局排字段
    //   0 航向角 / 1~4 四轮转速 A B C D / 5 角速度 / 6 中心速度 / 7~8 航位 x,y
    //   （车轮旁显示的是【车轮转速】而不是累计计数，量级 ±400，所以字段宽度不用动；
    //     9 字段 × 4 = 36 字符，原 64 深的字符 RAM 够用）
    parameter integer NUM_FIELDS = 9
)(
    input  wire                clk,
    input  wire                rst_n,

    input  wire signed [31:0]  i_v0,
    input  wire signed [31:0]  i_v1,
    input  wire signed [31:0]  i_v2,
    input  wire signed [31:0]  i_v3,
    input  wire signed [31:0]  i_v4,
    input  wire signed [31:0]  i_v5,
    input  wire signed [31:0]  i_v6,      // 字段 6：中心速度（rpm）
    input  wire signed [31:0]  i_v7,      // 字段 7：航位 x（odom 未实现，暂 0）
    input  wire signed [31:0]  i_v8,      // 字段 8：航位 y（odom 未实现，暂 0）

    output reg                 o_we,
    output reg  [5:0]          o_addr,
    output reg  [7:0]          o_data
);

    // 字库索引常量（索引 = ASCII − 32）
    localparam [7:0] CH_SPACE = 8'd0;        // ' '
    localparam [7:0] CH_MINUS = 8'd13;       // '-'
    localparam [7:0] CH_ZERO  = 8'd16;       // '0'

    // ★ REFRESH_DIV 的 27 位上限（2026-10-16 补）：
    //   div 是 `reg [26:0]`、S_WAIT 里比较的也是 `REFRESH_DIV[26:0]`，
    //   所以能表示的最大值是 2^27 − 1 = 134_217_727。
    //   REFRESH_DIV = (CLK_HZ/1000) × REFRESH_MS，本工程 50 MHz 下：
    //       REFRESH_MS = 2684 → 134_200_000（还在范围内）
    //       REFRESH_MS = 2685 → 134_250_000（**越界**）
    //   越界时**不会报错**：比较常量被位选截断、div 也数不到那么大 → 刷新周期静默变短，
    //   属于"参数写错却毫无迹象"的一类问题（本工程的 tb 用 10 MHz × 1 ms，
    //   REFRESH_DIV 只有 1 万，离该边界很远，测不出来）。
    //   本车实际取 REFRESH_MS = 100 → REFRESH_DIV = 5_000_000（23 位），余量充足。
    //   ★ 结论：要么保证 REFRESH_MS < 2685（50 MHz 时），要么同时加大 div 与
    //     比较常量的位宽；下面的仿真期断言会在越界时直接报错退出。
    localparam integer REFRESH_DIV     = (CLK_HZ / 1000) * REFRESH_MS;
    localparam integer REFRESH_DIV_MAX = 134_217_727;   // 27 位能表示的最大值
    // synopsys translate_off
    initial begin
        if (REFRESH_DIV > REFRESH_DIV_MAX) begin
            $display("ERROR: num_text REFRESH_DIV=%0d 超过 27 位上限 %0d（REFRESH_MS=%0d 会被静默截断）",
                     REFRESH_DIV, REFRESH_DIV_MAX, REFRESH_MS);
            $finish;
        end
    end
    // synopsys translate_on

    localparam [3:0] S_WAIT = 4'd0,
                     S_LOAD = 4'd1,
                     S_HUND = 4'd2,
                     S_TENS = 4'd3,
                     S_ONES = 4'd4,
                     S_W0   = 4'd5,
                     S_W1   = 4'd6,
                     S_W2   = 4'd7,
                     S_W3   = 4'd8;

    reg [3:0]  state;
    reg [26:0] div;
    reg [3:0]  fidx;          // 字段号 0..NUM_FIELDS-1（4 位：最多 16 个字段）
    reg        sign;          // 1 = 负数
    reg [15:0] mag;           // 剩余量（用于逐位减法）
    reg [3:0]  dig_h, dig_t, dig_o;
    reg signed [31:0] cur;    // 当前字段的原始值
    reg        [31:0] absv;   // |cur|：单独成信号，因为 Verilog 不允许对表达式做位选择（(-cur)[15:0] 是语法错误）

    // 取字段值
    always @(*) begin
        case (fidx)
            4'd0:    cur = i_v0;
            4'd1:    cur = i_v1;
            4'd2:    cur = i_v2;
            4'd3:    cur = i_v3;
            4'd4:    cur = i_v4;
            4'd5:    cur = i_v5;
            4'd6:    cur = i_v6;
            4'd7:    cur = i_v7;
            default: cur = i_v8;
        endcase
        absv = (cur < 0) ? (32'd0 - cur) : cur;
    end

    // 字段基地址 = fidx × FIELD_W
    wire [5:0] base_addr = fidx * FIELD_W[5:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_WAIT;
            div    <= 27'd0;
            fidx   <= 4'd0;
            sign   <= 1'b0;
            mag    <= 16'd0;
            dig_h  <= 4'd0;
            dig_t  <= 4'd0;
            dig_o  <= 4'd0;
            o_we   <= 1'b0;
            o_addr <= 6'd0;
            o_data <= CH_SPACE;
        end else begin
            o_we <= 1'b0;      // 默认不写

            case (state)
                // ---------------- 等刷新周期 ----------------
                S_WAIT: begin
                    // ★ 取 [26:0] 是与 div 的位宽对齐（超出 27 位的参数会在编译期
                    //   被截断成"更短的刷新周期"且不报错 —— 上限与断言见参数区说明）
                    if (div >= REFRESH_DIV[26:0]) begin
                        div   <= 27'd0;
                        fidx  <= 4'd0;
                        state <= S_LOAD;
                    end else begin
                        div <= div + 27'd1;
                    end
                end

                // ---------------- 载入字段值 + 取绝对值 ----------------
                S_LOAD: begin
                    sign  <= (cur < 0);
                    // 显示范围饱和到 999（本车轮速实际不超 ±400）
                    mag   <= (absv > 32'd999) ? 16'd999 : absv[15:0];
                    dig_h <= 4'd0;
                    dig_t <= 4'd0;
                    dig_o <= 4'd0;
                    state <= S_HUND;
                end

                // ---------------- 逐位减法：百位 → 十位 → 个位 ----------------
                S_HUND: begin
                    if (mag >= 16'd100) begin
                        mag   <= mag - 16'd100;
                        dig_h <= dig_h + 4'd1;
                    end else begin
                        state <= S_TENS;
                    end
                end

                S_TENS: begin
                    if (mag >= 16'd10) begin
                        mag   <= mag - 16'd10;
                        dig_t <= dig_t + 4'd1;
                    end else begin
                        state <= S_ONES;
                    end
                end

                S_ONES: begin
                    dig_o <= mag[3:0];
                    state <= S_W0;
                end

                // ---------------- 依次写 4 个字符 ----------------
                // ★ 2026-10-18：负号改为"跟随数字最高位的左侧一格"，数字仍右对齐。
                //   位数 ndigits = dig_h≠0 ? 3 : (dig_t≠0 ? 2 : 1)；负号位置 = 3 − ndigits。
                //      -5  → "  -5"（负号在位置 2）
                //      -45 → " -45"（位置 1）
                //      -400→ "-400"（位置 0）
                //   非负数不变：前 3 格空格 + 个位，数值右对齐。
                //   字段宽度仍是 4 字符，所以 UI 布局不用改。
                S_W0: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd0;
                    // 3 位数（dig_h≠0）时负号才在位置 0
                    o_data <= (sign && (dig_h != 4'd0)) ? CH_MINUS : CH_SPACE;
                    state  <= S_W1;
                end

                S_W1: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd1;
                    // 2 位数（百位为 0、十位非 0）时负号在位置 1；否则该位是百位或空格
                    o_data <= (sign && (dig_h == 4'd0) && (dig_t != 4'd0))
                              ? CH_MINUS
                              : ((dig_h == 4'd0) ? CH_SPACE : (CH_ZERO + {4'd0, dig_h}));
                    state  <= S_W2;
                end

                S_W2: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd2;
                    // 1 位数（百、十位都为 0）时负号在位置 2；否则该位是十位或空格
                    o_data <= (sign && (dig_h == 4'd0) && (dig_t == 4'd0))
                              ? CH_MINUS
                              : (((dig_h == 4'd0) && (dig_t == 4'd0))
                                 ? CH_SPACE : (CH_ZERO + {4'd0, dig_t}));
                    state  <= S_W3;
                end

                S_W3: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd3;
                    o_data <= CH_ZERO + {4'd0, dig_o};
                    // 下一个字段
                    if (fidx >= (NUM_FIELDS - 1)) begin
                        state <= S_WAIT;
                    end else begin
                        fidx  <= fidx + 4'd1;
                        state <= S_LOAD;
                    end
                end

                default: state <= S_WAIT;
            endcase
        end
    end

endmodule
