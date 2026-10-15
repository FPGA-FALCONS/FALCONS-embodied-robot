`timescale 1ns / 1ps

// Two-channel receiver: CH1 = steering; CH2 = throttle.
// Width calibration is independent for each channel. Initial calibration
// values are assumptions for software testing, not measured HT-10A values.
// In addition to rc_pwm_capture's parameter contract, each channel requires:
//   MIN_US < CENTER_US < MAX_US, all in 1..65535;
//   0 <= DEADZONE_US < min(CENTER_US-MIN_US, MAX_US-CENTER_US).
// Commands are signed 12-bit integers in -1000..1000, not physical v/omega.
// Either channel invalid => BOTH commands are zero immediately.
// This detects missing/illegal PWM; it cannot detect a radio link lost while
// the receiver continues to emit legal held/failsafe PWM. Test that behavior
// separately on the actual receiver before using commands to drive a robot.
module rc_receiver #(
    parameter integer CLK_HZ       = 50000000,
    parameter integer MIN_PULSE_US = 900,
    parameter integer MAX_PULSE_US = 2100,
    parameter integer TIMEOUT_US   = 100000,
    parameter integer CH1_MIN_US      = 1000,
    parameter integer CH1_CENTER_US   = 1500,
    parameter integer CH1_MAX_US      = 2000,
    parameter integer CH1_DEADZONE_US = 25,
    parameter integer CH2_MIN_US      = 1000,
    parameter integer CH2_CENTER_US   = 1500,
    parameter integer CH2_MAX_US      = 2000,
    parameter integer CH2_DEADZONE_US = 25,
    parameter integer CH1_REVERSE     = 0,
    parameter integer CH2_REVERSE     = 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rc_ch1,
    input  wire               rc_ch2,
    output wire signed [11:0] steering,
    output wire signed [11:0] throttle,
    output wire               signal_valid,
    output wire [15:0]        ch1_width_us,
    output wire [15:0]        ch2_width_us,
    output wire               ch1_valid,
    output wire               ch2_valid,
    output wire               ch1_timeout,
    output wire               ch2_timeout
);

    // Captures expose update strobes for standalone use; the receiver
    // consumes their persistent validity levels instead.
    wire unused_ch1_update;
    wire unused_ch2_update;

    rc_pwm_capture #(
        .CLK_HZ(CLK_HZ),
        .MIN_PULSE_US(MIN_PULSE_US),
        .MAX_PULSE_US(MAX_PULSE_US),
        .TIMEOUT_US(TIMEOUT_US)
    ) capture_ch1 (
        .clk(clk),
        .rst_n(rst_n),
        .rc_pwm(rc_ch1),
        .pulse_width_us(ch1_width_us),
        .pulse_valid(ch1_valid),
        .timed_out(ch1_timeout),
        .pulse_update(unused_ch1_update)
    );

    rc_pwm_capture #(
        .CLK_HZ(CLK_HZ),
        .MIN_PULSE_US(MIN_PULSE_US),
        .MAX_PULSE_US(MAX_PULSE_US),
        .TIMEOUT_US(TIMEOUT_US)
    ) capture_ch2 (
        .clk(clk),
        .rst_n(rst_n),
        .rc_pwm(rc_ch2),
        .pulse_width_us(ch2_width_us),
        .pulse_valid(ch2_valid),
        .timed_out(ch2_timeout),
        .pulse_update(unused_ch2_update)
    );

    function signed [11:0] normalized_command;
        input [15:0] width_us;
        input integer minimum_us;
        input integer center_us;
        input integer maximum_us;
        input integer deadzone_us;
        input integer reverse_axis;
        integer width_integer;
        integer displacement;
        integer scaled;
        begin
            width_integer = {16'd0, width_us};
            displacement = width_integer - center_us;

            if (width_integer <= minimum_us)
                scaled = -1000;
            else if (width_integer >= maximum_us)
                scaled = 1000;
            else if (displacement > deadzone_us)
                scaled = (displacement - deadzone_us) * 1000 /
                         (maximum_us - center_us - deadzone_us);
            else if (displacement < -deadzone_us)
                scaled = -((-displacement - deadzone_us) * 1000 /
                          (center_us - minimum_us - deadzone_us));
            else
                scaled = 0;

            if (reverse_axis != 0)
                scaled = -scaled;
            normalized_command = scaled[11:0];
        end
    endfunction

    assign signal_valid = ch1_valid && ch2_valid &&
                          !ch1_timeout && !ch2_timeout;

    assign steering = signal_valid ? normalized_command(
        ch1_width_us, CH1_MIN_US, CH1_CENTER_US, CH1_MAX_US,
        CH1_DEADZONE_US, CH1_REVERSE) : 12'sd0;

    assign throttle = signal_valid ? normalized_command(
        ch2_width_us, CH2_MIN_US, CH2_CENTER_US, CH2_MAX_US,
        CH2_DEADZONE_US, CH2_REVERSE) : 12'sd0;

endmodule
