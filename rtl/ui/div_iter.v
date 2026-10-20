//////////////////////////////////////////////////////////////////////////////////
// 模块：div_iter —— 无符号迭代除法器（恢复余数法，WIDTH 拍出结果）
//
// 【为什么需要它】
//   `rtl\nav\nav_ctrl.v` 里减速斜率是**派生量**：
//       K_SLOW_Q10 = (V_CRUISE_MMS × 1024) / D_SLOW_MM
//   参数写死时综合器把这条除法折叠成常量；一旦 V_CRUISE / D_SLOW 变成运行时
//   可调，它就变成真除法，而**组合除法**在本工程是明确禁止的（本工程已因
//   组合乘法吃过大亏，见 av_ctrl/nav_ctrl 里 M_V1/M_V2/M_V3 的拆拍记录）。
//
//   所以这里做一个"只在参数变更时跑一次"的多周期除法器：
//     · 被除数 ≤ 600×1024 = 614400（20 位）
//     · 除数   ≤ 300（9 位）
//     · 商     ≤ 12288（14 位）
//   用 WIDTH=24 位通用实现，24 拍 @50 MHz = 0.48 µs，相对 10 ms 控制周期是 0.005%。
//
// 【接口约定】
//   `i_start` 单拍脉冲启动一次；启动当拍 `i_a` / `i_b` 被采样。
//   运算中 `o_busy=1`；完成后 `o_valid` 拉高一拍，`o_q` / `o_r` 保持到下次启动。
//   ★ `i_start` 可以在 busy 期间再次拉高 —— 会**立即重新开始**（覆盖上次运算）。
//     这是刻意的：摇杆连发调参时参数会连续变，不必等上一次算完。
//
// 【除零保护】`i_b == 0` 时直接出 `o_q = WIDTH 位全 1`、`o_r = i_a`、`o_valid=1`。
//   本模块的调用方（tune_ctrl）保证除数有下限，这里只是不让它锁死。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module div_iter #(
    parameter integer WIDTH = 24
)(
    input  wire             clk,
    input  wire             rst_n,

    input  wire             i_start,
    input  wire [WIDTH-1:0] i_a,        // 被除数
    input  wire [WIDTH-1:0] i_b,        // 除数

    output reg  [WIDTH-1:0] o_q,        // 商
    output reg  [WIDTH-1:0] o_r,        // 余数
    output reg              o_valid,    // 完成脉冲（1 拍）
    output reg              o_busy
);

    localparam integer CW = WIDTH + 1;  // 余数需多 1 位（比较时可能临时达到 2×b）

    reg [WIDTH-1:0] a_sh;   // 被除数（逐拍左移）
    reg [WIDTH-1:0] b_r;    // 除数
    reg [WIDTH-1:0] q_sh;   // 商（逐拍左移补位）
    reg [CW-1:0]    rem;    // 余数
    reg [4:0]       bitcnt; // 剩余位数（WIDTH ≤ 32，5 位够）

    wire [CW-1:0] rem_shift = {rem[CW-2:0], a_sh[WIDTH-1]};
    wire          ge        = (rem_shift >= {1'b0, b_r});
    wire [CW-1:0] rem_next  = ge ? (rem_shift - {1'b0, b_r}) : rem_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sh    <= {WIDTH{1'b0}};
            b_r     <= {WIDTH{1'b0}};
            q_sh    <= {WIDTH{1'b0}};
            rem     <= {CW{1'b0}};
            bitcnt  <= 5'd0;
            o_q     <= {WIDTH{1'b0}};
            o_r     <= {WIDTH{1'b0}};
            o_valid <= 1'b0;
            o_busy  <= 1'b0;
        end else if (i_start) begin
            o_valid <= 1'b0;
            if (i_b == {WIDTH{1'b0}}) begin
                // 除零：商全 1、余数为被除数，立即完成
                o_q    <= {WIDTH{1'b1}};
                o_r    <= i_a;
                o_valid <= 1'b1;
                o_busy <= 1'b0;
                a_sh   <= {WIDTH{1'b0}};
                b_r    <= {WIDTH{1'b0}};
                q_sh   <= {WIDTH{1'b0}};
                rem    <= {CW{1'b0}};
                bitcnt <= 5'd0;
            end else begin
                a_sh   <= i_a;
                b_r    <= i_b;
                q_sh   <= {WIDTH{1'b0}};
                rem    <= {CW{1'b0}};
                bitcnt <= WIDTH[4:0];
                o_busy <= 1'b1;
            end
        end else if (o_busy) begin
            o_valid <= 1'b0;
            if (bitcnt == 5'd1) begin
                // 最后一位：本次的 q 位直接拼进结果
                o_q     <= {q_sh[WIDTH-2:0], ge};
                o_r     <= rem_next[CW-2:0];
                o_valid <= 1'b1;
                o_busy  <= 1'b0;
                bitcnt  <= 5'd0;
            end else begin
                a_sh   <= {a_sh[WIDTH-2:0], 1'b0};
                q_sh   <= {q_sh[WIDTH-2:0], ge};
                rem    <= rem_next;
                bitcnt <= bitcnt - 5'd1;
            end
        end else begin
            o_valid <= 1'b0;
        end
    end

endmodule
