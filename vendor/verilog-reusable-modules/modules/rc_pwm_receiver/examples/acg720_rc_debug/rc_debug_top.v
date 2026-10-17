`timescale 1ns/1ps
// Real receiver commands are observable on LEDs and on the physical HC595 bus.
module rc_debug_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer TIMEOUT_US = 100000,
    parameter integer SNAPSHOT_US = 1000,
    parameter integer DIGIT_US = 500,
    parameter integer SCLK_DIVIDER = 25,
    parameter integer HEARTBEAT_US = 500000
) (
    input wire clk,
    input wire rst_n,
    input wire rc_steering_pwm,
    input wire rc_throttle_pwm,
    input wire sw0,
    output wire ds,
    output wire sh_cp,
    output wire st_cp,
    output wire [7:0] led
);
    reg reset_meta, reset_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_meta <= 1'b0;
            reset_sync <= 1'b0;
        end else begin
            reset_meta <= 1'b1;
            reset_sync <= reset_meta;
        end
    end
    reg mode_meta, mode_sync;
    always @(posedge clk or negedge reset_sync) begin
        if (!reset_sync) begin
            mode_meta <= 1'b0;
            mode_sync <= 1'b0;
        end else begin
            mode_meta <= sw0;
            mode_sync <= mode_meta;
        end
    end
    wire signed [11:0] steering, throttle;
    wire signal_valid;
    wire [15:0] steering_width, throttle_width;
    wire steering_valid, throttle_valid, steering_timeout, throttle_timeout;
    rc_receiver #(.CLK_HZ(CLK_HZ), .TIMEOUT_US(TIMEOUT_US)) receiver (
        .clk(clk), .rst_n(reset_sync),
        .rc_ch1(rc_steering_pwm), .rc_ch2(rc_throttle_pwm),
        .steering(steering), .throttle(throttle), .signal_valid(signal_valid),
        .ch1_width_us(steering_width), .ch2_width_us(throttle_width),
        .ch1_valid(steering_valid), .ch2_valid(throttle_valid),
        .ch1_timeout(steering_timeout), .ch2_timeout(throttle_timeout)
    );
    wire [11:0] steering_abs = steering[11] ? -steering : steering;
    wire [11:0] throttle_abs = throttle[11] ? -throttle : throttle;
    wire [15:0] left_value = mode_sync ? {4'd0, steering_abs} : steering_width;
    wire [15:0] right_value = mode_sync ? {4'd0, throttle_abs} : throttle_width;
    wire left_valid = mode_sync ? signal_valid : steering_valid;
    wire right_valid = mode_sync ? signal_valid : throttle_valid;
    rc_hc595_display #(.CLK_HZ(CLK_HZ), .SNAPSHOT_US(SNAPSHOT_US),
        .DIGIT_US(DIGIT_US), .SCLK_DIVIDER(SCLK_DIVIDER)) display (
        .clk(clk), .rst_n(reset_sync), .ch1_width_us(left_value),
        .ch2_width_us(right_value), .ch1_valid(left_valid), .ch2_valid(right_valid),
        .ds(ds), .sh_cp(sh_cp), .st_cp(st_cp)
    );
    function integer counter_width;
        input integer maximum;
        integer remainder;
        begin
            remainder = maximum;
            counter_width = 0;
            while (remainder > 0) begin
                counter_width = counter_width + 1;
                remainder = remainder / 2;
            end
            if (counter_width < 1) counter_width = 1;
        end
    endfunction
    localparam integer HEARTBEAT_CYCLES = (CLK_HZ / 1000000) * HEARTBEAT_US;
    localparam integer HEARTBEAT_BITS = counter_width(HEARTBEAT_CYCLES - 1);
    localparam [HEARTBEAT_BITS-1:0] HEARTBEAT_LAST =
        HEARTBEAT_CYCLES[HEARTBEAT_BITS-1:0] - 1'b1;
    reg [HEARTBEAT_BITS-1:0] heartbeat_counter;
    reg heartbeat;
    always @(posedge clk or negedge reset_sync) begin
        if (!reset_sync) begin
            heartbeat_counter <= {HEARTBEAT_BITS{1'b0}};
            heartbeat <= 1'b0;
        end else if (heartbeat_counter == HEARTBEAT_LAST) begin
            heartbeat_counter <= {HEARTBEAT_BITS{1'b0}};
            heartbeat <= ~heartbeat;
        end else heartbeat_counter <= heartbeat_counter + 1'b1;
    end
    assign led = {heartbeat, signal_valid, steering_timeout || throttle_timeout,
        mode_sync, signal_valid && (throttle > 12'sd0),
        signal_valid && (throttle < 12'sd0),
        signal_valid && (steering > 12'sd0), signal_valid && (steering < 12'sd0)};
endmodule
