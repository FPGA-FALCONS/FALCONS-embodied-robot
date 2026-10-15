//////////////////////////////////////////////////////////////////////////////////
// 模块：app_key —— 板载按键去抖 + 边沿检测
//
// 板载按键低有效（按下 = 0），需要：
//   * 去抖：机械抖动通常 < 10 ms，计时稳定 DEBOUNCE_MS 后才认账
//   * 边沿：按下瞬间给一个单周期脉冲（用于"切换页面 / 清零 / 急停锁存切换"）
//   * 电平：去抖后的按下电平（用于"按住 = 启动"这类语义）
//
// 与 `reset_sync` 的区别：reset_sync 专门产生复位（上电先拉低、按键也拉低），
// 本模块是通用按键输入，不参与复位。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module app_key #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer DEBOUNCE_MS = 10
)(
    input  wire i_clock,
    input  wire i_reset_n,
    input  wire i_key_n,          // 低有效按键输入（异步）

    output reg  o_pressed,        // 去抖后的按下电平（1 = 按下）
    output reg  o_press_pulse,    // 按下瞬间：单周期脉冲
    output reg  o_release_pulse   // 松开瞬间：单周期脉冲
);

    localparam integer CNT_MAX = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

    // 两级同步（异步输入）
    reg s1, s2;
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            s1 <= 1'b1;
            s2 <= 1'b1;
        end else begin
            s1 <= i_key_n;
            s2 <= s1;
        end
    end

    // 计时稳定后认账
    reg [31:0] cnt;
    reg        stable;            // 去抖后的原始电平（1 = 未按下）

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            cnt    <= 32'd0;
            stable <= 1'b1;
        end else begin
            if (s2 != stable) begin
                if (cnt >= CNT_MAX) begin
                    stable <= s2;
                    cnt    <= 32'd0;
                end else begin
                    cnt <= cnt + 32'd1;
                end
            end else begin
                cnt <= 32'd0;
            end
        end
    end

    // 电平与边沿
    reg pressed_d;
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            o_pressed       <= 1'b0;
            pressed_d       <= 1'b0;
            o_press_pulse   <= 1'b0;
            o_release_pulse <= 1'b0;
        end else begin
            o_pressed       <= ~stable;            // 按下 = 低有效
            pressed_d       <= ~stable;
            o_press_pulse   <= (~stable) & ~pressed_d;
            o_release_pulse <=   stable  &  pressed_d;
        end
    end

endmodule
