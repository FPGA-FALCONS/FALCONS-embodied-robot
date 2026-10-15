//////////////////////////////////////////////////////////////////////////////////
// 模块：num_text —— 把 6 个有符号整数格式化成定宽字符串，写进 dyn_text_ram
//
// 【格式】每个数值固定 4 个字符：`符号 + 百位 + 十位 + 个位`，例如
//      0    → "   0"（索引序列：空格,空格,空格,'0'）
//      123  → " 123"
//      -45  → "- 45"   —— 注：百位为 0 时写空格，所以是 "-  45"（等宽右对齐）
//   6 个数值依次放在地址 0..23（每个占 4 字节）。
//
// 【为什么用慢速状态机而不是组合除法】
//   50 MHz 下一拍一次比较/减法即可，刷新周期是 100 ms（远大于 24 个字符的生成时间），
//   组合除法会拉出很长的组合路径（本工程已因组合乘法吃过 setup −7 ns 的亏）。
//
// 【数值来源（顶层接线）】
//   v0/v1 = 左右侧**目标**轮速（rpm，由 chassis_ctrl 的 delta 目标换算）
//   v2..v5 = 四路**实际**轮速（rpm）A 左前 / B 右前 / C 左后 / D 右后
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module num_text #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer REFRESH_MS = 100,      // 刷新周期
    parameter integer FIELD_W    = 4,        // 每字段字符数（符号+3 位数字）
    parameter integer NUM_FIELDS = 7
)(
    input  wire                clk,
    input  wire                rst_n,

    input  wire signed [31:0]  i_v0,
    input  wire signed [31:0]  i_v1,
    input  wire signed [31:0]  i_v2,
    input  wire signed [31:0]  i_v3,
    input  wire signed [31:0]  i_v4,
    input  wire signed [31:0]  i_v5,
    input  wire signed [31:0]  i_v6,      // 字段 6：航向角（整数度）

    output reg                 o_we,
    output reg  [5:0]          o_addr,
    output reg  [7:0]          o_data
);

    // 字库索引常量（索引 = ASCII − 32）
    localparam [7:0] CH_SPACE = 8'd0;        // ' '
    localparam [7:0] CH_MINUS = 8'd13;       // '-'
    localparam [7:0] CH_ZERO  = 8'd16;       // '0'

    localparam integer REFRESH_DIV = (CLK_HZ / 1000) * REFRESH_MS;

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
    reg [2:0]  fidx;          // 字段号 0..NUM_FIELDS-1
    reg        sign;          // 1 = 负数
    reg [15:0] mag;           // 剩余量（用于逐位减法）
    reg [3:0]  dig_h, dig_t, dig_o;
    reg signed [31:0] cur;    // 当前字段的原始值
    reg        [31:0] absv;   // |cur|：单独成信号，因为 Verilog 不允许对表达式做位选择（(-cur)[15:0] 是语法错误）

    // 取字段值
    always @(*) begin
        case (fidx)
            3'd0:    cur = i_v0;
            3'd1:    cur = i_v1;
            3'd2:    cur = i_v2;
            3'd3:    cur = i_v3;
            3'd4:    cur = i_v4;
            3'd5:    cur = i_v5;
            default: cur = i_v6;
        endcase
        absv = (cur < 0) ? (32'd0 - cur) : cur;
    end

    // 字段基地址 = fidx × FIELD_W
    wire [5:0] base_addr = fidx * FIELD_W[5:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_WAIT;
            div    <= 27'd0;
            fidx   <= 3'd0;
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
                    if (div >= REFRESH_DIV[26:0]) begin
                        div   <= 27'd0;
                        fidx  <= 3'd0;
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
                S_W0: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd0;
                    o_data <= sign ? CH_MINUS : CH_SPACE;
                    state  <= S_W1;
                end

                S_W1: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd1;
                    o_data <= (dig_h == 4'd0) ? CH_SPACE : (CH_ZERO + {4'd0, dig_h});
                    state  <= S_W2;
                end

                S_W2: begin
                    o_we   <= 1'b1;
                    o_addr <= base_addr + 6'd2;
                    o_data <= ((dig_h == 4'd0) && (dig_t == 4'd0)) ? CH_SPACE : (CH_ZERO + {4'd0, dig_t});
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
                        fidx  <= fidx + 3'd1;
                        state <= S_LOAD;
                    end
                end

                default: state <= S_WAIT;
            endcase
        end
    end

endmodule
