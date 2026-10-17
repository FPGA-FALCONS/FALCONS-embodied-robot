`timescale 1ns / 1ps

// Observation only. CH5/CH6 indicate PWM switch positions, not an emergency
// stop or proof of the transmitter's physical button-to-channel assignment.
module rc_6ch_led_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer TIMEOUT_US = 100000,
    parameter integer CHANNEL_US = 10000000,
    parameter integer DIGIT_US = 500,
    parameter integer SNAPSHOT_US = 1000,
    parameter integer SCLK_DIVIDER = 25,
    parameter integer HEARTBEAT_US = 500000,
    parameter integer CH5_ACTIVE_LOW = 0,
    parameter integer CH6_ACTIVE_LOW = 0
) (
    input wire clk,
    input wire rst_n,
    input wire [5:0] rc_pwm,
    input wire sw0,
    output wire ds,
    output wire sh_cp,
    output wire st_cp,
    output wire [7:0] led
);
    wire [7:0] axis_led;
    rc_4ch_top #(
        .CLK_HZ(CLK_HZ), .TIMEOUT_US(TIMEOUT_US),
        .CHANNEL_US(CHANNEL_US), .DIGIT_US(DIGIT_US),
        .SNAPSHOT_US(SNAPSHOT_US), .SCLK_DIVIDER(SCLK_DIVIDER),
        .HEARTBEAT_US(HEARTBEAT_US)
    ) axes (
        .clk(clk), .rst_n(rst_n), .rc_pwm(rc_pwm[3:0]), .sw0(sw0),
        .ds(ds), .sh_cp(sh_cp), .st_cp(st_cp), .led(axis_led)
    );

    wire [15:0] ch5_width;
    wire [15:0] ch6_width;
    wire ch5_valid;
    wire ch6_valid;
    wire unused_ch5_timeout;
    wire unused_ch6_timeout;
    wire unused_ch5_update;
    wire unused_ch6_update;
    // D4/D5 replace the old top's binary channel-number indicators.
    wire [1:0] unused_axis_status = axis_led[5:4];
    rc_pwm_capture #(
        .CLK_HZ(CLK_HZ), .MIN_PULSE_US(900), .MAX_PULSE_US(2100),
        .TIMEOUT_US(TIMEOUT_US)
    ) ch5_capture (
        .clk(clk), .rst_n(rst_n), .rc_pwm(rc_pwm[4]),
        .pulse_width_us(ch5_width), .pulse_valid(ch5_valid),
        .timed_out(unused_ch5_timeout), .pulse_update(unused_ch5_update)
    );
    rc_pwm_capture #(
        .CLK_HZ(CLK_HZ), .MIN_PULSE_US(900), .MAX_PULSE_US(2100),
        .TIMEOUT_US(TIMEOUT_US)
    ) ch6_capture (
        .clk(clk), .rst_n(rst_n), .rc_pwm(rc_pwm[5]),
        .pulse_width_us(ch6_width), .pulse_valid(ch6_valid),
        .timed_out(unused_ch6_timeout), .pulse_update(unused_ch6_update)
    );

    assign led[3:0] = axis_led[3:0];
    assign led[4] = rst_n && ch5_valid &&
        ((CH5_ACTIVE_LOW != 0) ? (ch5_width < 16'd1250) :
                                (ch5_width > 16'd1750));
    assign led[5] = rst_n && ch6_valid &&
        ((CH6_ACTIVE_LOW != 0) ? (ch6_width < 16'd1250) :
                                (ch6_width > 16'd1750));
    assign led[7:6] = axis_led[7:6];
endmodule
