//////////////////////////////////////////////////////////////////////////////////
// 模块：reset_sync —— 按键复位同步与去抖
//
// 功能：
//   * 对异步按键输入做两级同步（避免亚稳态）
//   * 去抖：输入需连续稳定 DEBOUNCE_MS 毫秒才被采纳
//   * 输出可作为全局低有效复位使用（异步复位、同步释放）
//
// 说明：
//   上电时内部稳定值为 0，因此天然产生一段上电复位（约 DEBOUNCE_MS），
//   待按键电平确认后才释放复位。
//
// 参数：
//   CLK_FREQ_HZ   系统时钟频率(Hz)   默认 50_000_000
//   DEBOUNCE_MS   去抖确认时间(ms)   默认 20
//
// 端口：
//   i_clock        输入  系统时钟
//   i_key_n        输入  按键输入，低有效（按下为低）
//   o_reset_n      输出  同步去抖后的复位输出，低有效
//////////////////////////////////////////////////////////////////////////////////

module reset_sync #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire i_clock,
    input  wire i_key_n,
    output wire o_reset_n
);

    localparam integer MS_TICKS   = (CLK_FREQ_HZ >= 1000) ? (CLK_FREQ_HZ / 1000) : 1;
    localparam integer DBNC_TICKS = MS_TICKS * DEBOUNCE_MS;

    // 两级同步
    // Explicit startup values match key_stable=0; do not depend on simulator/tool FF defaults.
    reg [1:0] key_sync = 2'b00;
    always @(posedge i_clock) begin
        key_sync <= {key_sync[0], i_key_n};
    end

    // 去抖计数
    reg [31:0] dbnc_cnt = 32'd0;
    reg        key_stable = 1'b0;    // 上电默认处于复位态，去抖确认后释放

    always @(posedge i_clock) begin
        if (key_sync[1] == key_stable) begin
            dbnc_cnt <= 32'd0;
        end else if (dbnc_cnt >= DBNC_TICKS) begin
            key_stable <= key_sync[1];
            dbnc_cnt   <= 32'd0;
        end else begin
            dbnc_cnt <= dbnc_cnt + 32'd1;
        end
    end

    assign o_reset_n = key_stable;

endmodule
