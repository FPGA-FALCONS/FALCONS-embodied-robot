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

    // Capture update strobes restart normalization of the latest pair.
    wire ch1_update;
    wire ch2_update;

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
        .pulse_update(ch1_update)
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
        .pulse_update(ch2_update)
    );

    // Reset assertion is asynchronous; state leaves reset on a clock edge.
    reg reset_meta;
    reg reset_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_meta <= 1'b0;
            reset_sync <= 1'b0;
        end else begin
            reset_meta <= 1'b1;
            reset_sync <= reset_meta;
        end
    end

    wire captures_valid = ch1_valid && ch2_valid &&
                          !ch1_timeout && !ch2_timeout;
    reg have_command;
    reg calculating;
    reg signed [11:0] steering_committed;
    reg signed [11:0] throttle_committed;
    wire start_pair = captures_valid &&
        (ch1_update || ch2_update || (!calculating && !have_command));
    wire cancel_pair = !captures_valid;
    wire signed [11:0] steering_next;
    wire signed [11:0] throttle_next;
    wire steering_done;
    wire throttle_done;
    wire unused_steering_busy;
    wire unused_throttle_busy;

    rc_axis_normalize #(
        .MIN_US(CH1_MIN_US), .CENTER_US(CH1_CENTER_US),
        .MAX_US(CH1_MAX_US), .DEADZONE_US(CH1_DEADZONE_US),
        .REVERSE(CH1_REVERSE)
    ) normalize_ch1 (
        .clk(clk), .rst_n(reset_sync), .start(start_pair),
        .cancel(cancel_pair), .width_us(ch1_width_us),
        .command(steering_next), .busy(unused_steering_busy),
        .done(steering_done)
    );
    rc_axis_normalize #(
        .MIN_US(CH2_MIN_US), .CENTER_US(CH2_CENTER_US),
        .MAX_US(CH2_MAX_US), .DEADZONE_US(CH2_DEADZONE_US),
        .REVERSE(CH2_REVERSE)
    ) normalize_ch2 (
        .clk(clk), .rst_n(reset_sync), .start(start_pair),
        .cancel(cancel_pair), .width_us(ch2_width_us),
        .command(throttle_next), .busy(unused_throttle_busy),
        .done(throttle_done)
    );

    // Normalizer latency is 27 clocks after start; pair commit is one clock
    // later. A new capture replaces an in-flight snapshot. The last committed
    // pair remains valid while calculating ordinary fresh updates, preventing
    // a spurious loss indication every RC frame. A real invalidation cancels
    // that session; recovery MUST finish a new calculation before valid rises.
    always @(posedge clk or negedge reset_sync) begin
        if (!reset_sync) begin
            have_command <= 1'b0;
            calculating <= 1'b0;
            steering_committed <= 12'sd0;
            throttle_committed <= 12'sd0;
        end else if (!captures_valid) begin
            have_command <= 1'b0;
            calculating <= 1'b0;
            steering_committed <= 12'sd0;
            throttle_committed <= 12'sd0;
        end else if (start_pair) begin
            calculating <= 1'b1;
        end else if (calculating && steering_done && throttle_done) begin
            steering_committed <= steering_next;
            throttle_committed <= throttle_next;
            have_command <= 1'b1;
            calculating <= 1'b0;
        end
    end

    assign signal_valid = rst_n && reset_sync && captures_valid && have_command;
    assign steering = signal_valid ? steering_committed : 12'sd0;
    assign throttle = signal_valid ? throttle_committed : 12'sd0;
endmodule
