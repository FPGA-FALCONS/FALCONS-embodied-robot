//////////////////////////////////////////////////////////////////////////////////
// 模块：num_text —— 把 9 个有符号整数格式化成定宽字符串，写进 dyn_text_ram
//
// 【格式】★ 2026-10-18：字段宽度 4 → 5（为「航位两位数(m) + 0.01 精度」腾出第 4 位数字）
//   每个数值固定占 FIELD_W = 5 个字符位置：
//      位置 0..4 = `符号 + 千位 + 百位 + 十位 + 个位`
//   但**符号不是固定在位置 0**，而是"跟随最高数字左移一格"：
//      0     → "    0"（4 空格 + '0'）
//      45    → "   45"
//      -45   → "  -45"   （负号在位置 2）
//      123   → "  123"
//      -123  → " -123"   （负号在位置 1）
//      1234  → " 1234"
//      -1234 → "-1234"   （负号在位置 0）
//
// 【同一份 5 格结果，服务两种显示宽度】
//   渲染端（ui_render.v）的两类消费者从同一片 RAM 里按不同起点取字符：
//     4 字符字段（航向角 / 四轮转速）  → 从 base+1 读 4 格 = "符号 + 3 位数字"
//     5 字符字段（角速度/中心速度/航位）→ 从 base+0 读 5 格 = "符号 + 4 位数字"
//   为什么可行：5 格里的 1..4 号位置正好构成一个"符号 + 3 位数字"的右对齐 4 格窗口。
//     轮速 400  → "   400" → 读 1..4 = " 400"  ✓
//     轮速 -45  → "  -45" → 读 1..4 = " -45"  ✓
//     轮速 -400 → " -400" → 读 1..4 = "-400"  ✓
//   ★ 好处：状态机**不需要**再按字段宽度分支，只有一条格式化路径。
//
//   9 个数值依次放在地址 0, 5, 10, ..., 40（每个占 5 字节；共 45 字符，字符 RAM 深 64）。
//   显示范围【饱和到 ±9999】：
//     ★ 4 字符的消费者（轮速 ±400、航向角 ±180）实际永远不到 1000，
//       但输入若异常变大也不会溢出到相邻字段（饱和后再截 4 格只会显示 9999，不会串位）。
//
// 【为什么用慢速状态机而不是组合除法】
//   50 MHz 下一拍一次比较/减法即可，刷新周期是 100 ms（远大于 45 个字符的生成时间），
//   组合除法会拉出很长的组合路径（本工程已因组合乘法吃过 setup −7 ns 的亏）。
//   格式化耗时：每字段最多 9(千) + 9(百) + 9(十) + 9(个) + 5(写) ≈ 41 拍，
//   9 字段 ≈ 370 拍 ≈ 7.4 µs @50 MHz，对 100 ms 刷新周期仍是 0.0074%。
//
// 【数值来源（顶层接线，2026-10-16 按需求原文 §29 的数据页重排）】
//   ★ 口径裁定（需求方 2026-10-16 明确）：车轮旁显示的是【车轮转速】，
//     不是编码器累计读数。
//   v0    = 航向角（整数度）
//   v1..v4 = 四路**车轮转速**（rpm）A 右后 / B 左后 / C 右前 / D 左前
//   v5    = 角速度（整数 °/s）
//   v6    = 中心速度（★ 2026-10-18：单位改为 0.01 m/s，显示时小数点悬挂）
//   v7/v8 = 航位 x / y（★ 单位 0.01 m；odom 未实现 → 暂恒 0，但要能显示两位数）
//   小数点**不在本模块产生**：它由 ui_render 的小数点叠加层按「字段号 → 小数位数」
//   在字符格交界处画一个 3x3 像素点，完全不占字符位。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module num_text #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer REFRESH_MS = 100,      // 刷新周期
    // ★ 2026-10-18：4 → 5（"符号 + 4 位数字"，±9999）
    parameter integer FIELD_W    = 5,        // 每字段字符数（地址步长）
    // ★ 2026-10-16：7 → 9，按需求原文 §29 的数据页布局排字段
    //   0 航向角 / 1~4 四轮转速 A B C D / 5 角速度 / 6 中心速度 / 7~8 航位 x,y
    //   9 字段 × 5 = 45 字符，字符 RAM 深 64 够用（余量 19）。
    //   ★ 2026-10-19：9 → 11。新增字段 9/10 = 「非避障导航」页右上角的准心坐标 x / y
    //     （单位 0.01 m，与航位同口径）。11 字段 × 5 字符 = 55 < 字符 RAM 深度 64，仍够。
    //   ★ 2026-10-20：11 → 12。新增字段 11 = 「参数整定」页第 12 行（速度环 Kd）。
    //     14 字段 × 5 字符 = 70；字符 RAM 使用 7 位地址、深度 128。
    parameter integer NUM_FIELDS = 16
)(
    input  wire                clk,
    input  wire                rst_n,

    input  wire signed [31:0]  i_v0,
    input  wire signed [31:0]  i_v1,
    input  wire signed [31:0]  i_v2,
    input  wire signed [31:0]  i_v3,
    input  wire signed [31:0]  i_v4,
    input  wire signed [31:0]  i_v5,
    input  wire signed [31:0]  i_v6,      // 字段 6：中心速度（单位 0.01 m/s）
    input  wire signed [31:0]  i_v7,      // 字段 7：航位 x（单位 0.01 m，odom 未实现暂 0）
    input  wire signed [31:0]  i_v8,      // 字段 8：航位 y（单位 0.01 m）
    input  wire signed [31:0]  i_v9,      // 字段 9：非避障导航页 准心 x（0.01 m）
    input  wire signed [31:0]  i_v10,     // 字段 10：非避障导航页 准心 y（0.01 m）
    input  wire signed [31:0]  i_v11,     // 字段 11：参数整定页第 12 行（速度环 Kd）

    input  wire signed [31:0]  i_v12,     // 前探格数
    input  wire signed [31:0]  i_v15,     // 巡检转向限幅显示值
    input  wire signed [31:0]  i_v14,     // 规划窗口格数
    input  wire signed [31:0]  i_v13,     // 车宽 mm

    // ★ 2026-10-20（综合报错后补）：哪些字段带小数（= 整数部分至少几位）。
    //   原实现写死 `fidx >= 6 → min_dig = 3`，把"带小数的字段号"钉在了模块里。
    //   参数整定页复用了数据页的字段 0..11，但**参数全是整数**，若沿用 3 位最小宽度，
    //   航向 Kd = 6 会显示成 "  006" 且 ui_render 还会在百位后画小数点（"  0.06"）。
    //   故改为由顶层按页面给掩码：数据页 12'b0111_1100_0000（字段 6..10），参数页 12'b0。
    input  wire [15:0]         i_frac_mask,

    output reg                 o_we,
    output reg  [6:0]          o_addr,
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

    // ★ 2026-10-18：新增 S_THOU（千位）与 S_W4（第 5 个字符）
    localparam [3:0] S_WAIT = 4'd0,
                     S_LOAD = 4'd1,
                     S_THOU = 4'd2,
                     S_HUND = 4'd3,
                     S_TENS = 4'd4,
                     S_ONES = 4'd5,
                     S_W0   = 4'd6,
                     S_W1   = 4'd7,
                     S_W2   = 4'd8,
                     S_W3   = 4'd9,
                     S_W4   = 4'd10;

    reg [3:0]  state;
    reg [26:0] div;
    reg [3:0]  fidx;          // 字段号 0..NUM_FIELDS-1（4 位：最多 16 个字段）
    reg        sign;          // 1 = 负数
    reg [15:0] mag;           // 剩余量（用于逐位减法）
    reg [3:0]  dig_th, dig_h, dig_t, dig_o;   // 千 / 百 / 十 / 个
    reg signed [31:0] cur;    // 当前字段的原始值
    reg        [31:0] absv;   // |cur|：单独成信号，因为 Verilog 不允许对表达式做位选择（(-cur)[15:0] 是语法错误）

    // 取字段值
    // ★ 2026-10-18（时序修复）：输入先打一拍再参与取绝对值/饱和。
    //   为什么必须打拍：`i_v5`/`i_v6` 在上层是**组合乘法链**（中心速度 `sum_delta × 8580 >> 20`（D=64mm）、
    //   角速度 `|gyro_z_comp| × 10486 >> 20`），若直接接到 `absv → mag` 的组合逻辑上，
    //   整条路径 = gyro_tilt_comp 补偿(9.1 ns) + 取绝对值(3.4) + 乘法(2.2) + 本模块 mux/abs/饱和(11.2)
    //   ≈ 25.9 ns，超过 50 MHz 周期，实测 **12 条 `mag_*` 路径 slack −1.59 ~ −1.97 ns**。
    //   打一拍后拆成两段：① 计算链 → v*_r（≈16.7 ns，过）；② v*_r → cur → absv → mag（≈11.2 ns，过）。
    //   代价：9 × 32 = 288 个触发器 + 1 拍延迟（20 ns，对 100 ms 刷新周期完全无感）。
    reg signed [31:0] v0_r, v1_r, v2_r, v3_r, v4_r,
                      v5_r, v6_r, v7_r, v8_r, v9_r, v10_r, v11_r, v12_r, v13_r, v14_r, v15_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v0_r <= 32'sd0; v1_r <= 32'sd0; v2_r <= 32'sd0;
            v3_r <= 32'sd0; v4_r <= 32'sd0; v5_r <= 32'sd0;
            v6_r <= 32'sd0; v7_r <= 32'sd0; v8_r <= 32'sd0;
            v9_r <= 32'sd0; v10_r <= 32'sd0; v11_r <= 32'sd0; v12_r <= 32'sd0; v13_r <= 32'sd0; v14_r <= 32'sd0; v15_r <= 32'sd0;
        end else begin
            v0_r <= i_v0; v1_r <= i_v1; v2_r <= i_v2;
            v3_r <= i_v3; v4_r <= i_v4; v5_r <= i_v5;
            v6_r <= i_v6; v7_r <= i_v7; v8_r <= i_v8;
            v9_r <= i_v9; v10_r <= i_v10; v11_r <= i_v11; v12_r <= i_v12; v13_r <= i_v13; v14_r <= i_v14; v15_r <= i_v15;
        end
    end

    always @(*) begin
        case (fidx)
            4'd0:    cur = v0_r;
            4'd1:    cur = v1_r;
            4'd2:    cur = v2_r;
            4'd3:    cur = v3_r;
            4'd4:    cur = v4_r;
            4'd5:    cur = v5_r;
            4'd6:    cur = v6_r;
            4'd7:    cur = v7_r;
            4'd8:    cur = v8_r;
            4'd9:    cur = v9_r;
            4'd10:   cur = v10_r;
            4'd11:   cur = v11_r;
            4'd12:   cur = v12_r;
            4'd13: cur = v13_r;
            4'd14: cur = v14_r;
            default: cur = v15_r;
        endcase
        absv = (cur < 0) ? (32'd0 - cur) : cur;
    end

    // 字段基地址 = fidx × FIELD_W
    //   ★ FIELD_W = 5 不是 2 的幂，综合器会生成一个 3 位×常量的移位相加
    //     （5 = 4 + 1 → fidx<<2 + fidx），几十个 LUT，无时序风险。
    wire [6:0] base_addr = fidx * FIELD_W[6:0];

    // 数字位选择辅助：某一位为 0 时该位置写空格或负号
    //   统一给出「该位置上的数字字符」
    wire [7:0] dch_th = CH_ZERO + {4'd0, dig_th};
    wire [7:0] dch_h  = CH_ZERO + {4'd0, dig_h};
    wire [7:0] dch_t  = CH_ZERO + {4'd0, dig_t};
    wire [7:0] dch_o  = CH_ZERO + {4'd0, dig_o};

    // ★ 2026-10-18（需求方指出）：带小数的字段必须保证「整数部分至少 1 位」，
    //   否则 0.00 / 0.05 会显示成 "   .0" / "   .5"（小数点左边是空格）。
    //   最小数字位数 = dec + 1：
    //     字段 6/7/8（中心速度 / 航位）在 ui_render.v 里 dec = 2 → **最少 3 位数字**
    //     其余（航向角 / 四轮转速 / 角速度）→ 最少 1 位（维持原行为，轮速显示不变）
    //   ⚠ 这个表必须与 `ui_render.v` 的 `field_dec()` 保持一致：dec=2 ⇔ min_dig=3。
    //     改小数位数时**两处一起改**（num_text_tb 有 "v6=0 → 000" 的断言锁住它）。
    //   ★ 2026-10-20：由字段号判断改为由 `i_frac_mask` 决定（见端口注释）。
    //      带小数（掩码位 = 1）→ 最少 3 位数字（0.00 的整数部分是 1 位 '0'）
    //      不带小数（掩码位 = 0）→ 最少 1 位数字（6 就显示 "    6"，不出前导零）
    //      fidx 是 4 位、掩码 14 位，fidx ≥ 14 时越界，用 guard 兜成 0。
    wire       frac_bit = (fidx < 5'd16) ? i_frac_mask[fidx] : 1'b0;
    wire [3:0] min_dig  = frac_bit ? 4'd3 : 4'd1;

    // ★ 2026-10-18（预览图抓出的 bug）：前导零抑制必须**按"高位是否已开始"判断**，
    //   不能"该位是否为 0"。原写法 `(dig_t != 0) ? 数字 : 空格` 会让 400 显示成
    //   " 4 0"、-200 显示成 "-2 0" —— 十位是 0 但百位已经非零，此时十位必须显示 '0'。
    //   started_X = "X 位及更高位中至少有一个非零" **或** "该位在最小位数之内"
    wire started_h = (dig_th != 4'd0) | (min_dig >= 4'd4);   // 千位已开始
    wire started_t = started_h | (dig_h != 4'd0) | (min_dig >= 4'd3);   // 百位已开始
    wire started_o = started_t | (dig_t != 4'd0) | (min_dig >= 4'd2);   // 十位已开始
    // 负号位置 = 最高非零位的左边一格：4 位→0，3 位→1，2 位→2，1 位→3
    wire [1:0] npos = started_h ? 2'd0 : (started_t ? 2'd1 : (started_o ? 2'd2 : 2'd3));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_WAIT;
            div    <= 27'd0;
            fidx   <= 4'd0;
            sign   <= 1'b0;
            mag    <= 16'd0;
            dig_th <= 4'd0;
            dig_h  <= 4'd0;
            dig_t  <= 4'd0;
            dig_o  <= 4'd0;
            o_we   <= 1'b0;
            o_addr <= 7'd0;
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
                    // 显示范围饱和到 9999（5 格 = 符号 + 4 位数字）
                    mag   <= (absv > 32'd9999) ? 16'd9999 : absv[15:0];
                    dig_th <= 4'd0;
                    dig_h <= 4'd0;
                    dig_t <= 4'd0;
                    dig_o <= 4'd0;
                    state <= S_THOU;
                end

                // ---------------- 逐位减法：千位 → 百位 → 十位 → 个位 ----------------
                //   每位最多减 9 次（最坏 9 拍），四层合计最多 36 拍 + 5 拍写。
                S_THOU: begin
                    if (mag >= 16'd1000) begin
                        mag    <= mag - 16'd1000;
                        dig_th <= dig_th + 4'd1;
                    end else begin
                        state <= S_HUND;
                    end
                end

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

                // ---------------- 依次写 5 个字符 ----------------
                // ★ 2026-10-18：负号改为"跟随数字最高位左侧一格"，数字仍右对齐。
                //   位数 ndigits = dig_th≠0 ? 4 : (dig_h≠0 ? 3 : (dig_t≠0 ? 2 : 1))
                //   负号位置    = 4 − ndigits  →  4位:0  3位:1  2位:2  1位:3  （= npos）
                //   ★ 低位补 0 的规则见 started_* / npos 的声明处（预览图抓过的 bug）。
                //
                //   位置 0：只有 4 位数才放负号；否则一定是空格
                S_W0: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 7'd0;
                    o_data <= (sign && (npos == 2'd0)) ? CH_MINUS : CH_SPACE;
                    state  <= S_W1;
                end

                //   位置 1：是千位；若千位为 0（3 位数及以下），这里放负号或空格
                S_W1: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 7'd1;
                    o_data <= started_h ? dch_th
                            : ((sign && (npos == 2'd1)) ? CH_MINUS : CH_SPACE);
                    state  <= S_W2;
                end

                //   位置 2：是百位；若百位为 0（2 位数及以下），这里放负号或空格
                S_W2: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 7'd2;
                    o_data <= started_t ? dch_h
                            : ((sign && (npos == 2'd2)) ? CH_MINUS : CH_SPACE);
                    state  <= S_W3;
                end

                //   位置 3：是十位；若十位为 0（1 位数），这里放负号或空格
                S_W3: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 7'd3;
                    o_data <= started_o ? dch_t
                            : ((sign && (npos == 2'd3)) ? CH_MINUS : CH_SPACE);
                    state  <= S_W4;
                end

                //   位置 4：个位，永远显示（0 也要显示 '0'）
                S_W4: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 7'd4;
                    o_data <= dch_o;
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
