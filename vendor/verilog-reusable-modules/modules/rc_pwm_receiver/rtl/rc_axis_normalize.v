`timescale 1ns / 1ps

// One calibrated RC axis. Pure Verilog-2001; no combinational division.
// start snapshots width_us, and can replace an in-flight request. cancel
// discards the request. done pulses exactly 27 clocks after accepted start.
// Parameters require 1 <= MIN_US < CENTER_US < MAX_US <= 65535 and
// 0 <= DEADZONE_US < min(CENTER_US-MIN_US, MAX_US-CENTER_US).
// The magnitude is truncated towards zero, matching the original mapping.
module rc_axis_normalize #(
    parameter integer MIN_US = 1000,
    parameter integer CENTER_US = 1500,
    parameter integer MAX_US = 2000,
    parameter integer DEADZONE_US = 25,
    parameter integer REVERSE = 0
) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire cancel,
    input wire [15:0] width_us,
    output reg signed [11:0] command,
    output reg busy,
    output reg done
);
    localparam [15:0] LOW_LIMIT = MIN_US[15:0];
    localparam [15:0] HIGH_LIMIT = MAX_US[15:0];
    localparam integer LOW_EDGE_I = CENTER_US - DEADZONE_US;
    localparam integer HIGH_EDGE_I = CENTER_US + DEADZONE_US;
    localparam integer LOW_SPAN_I = CENTER_US - MIN_US - DEADZONE_US;
    localparam integer HIGH_SPAN_I = MAX_US - CENTER_US - DEADZONE_US;
    localparam [15:0] LOW_EDGE = LOW_EDGE_I[15:0];
    localparam [15:0] HIGH_EDGE = HIGH_EDGE_I[15:0];
    localparam [15:0] LOW_SPAN = LOW_SPAN_I[15:0];
    localparam [15:0] HIGH_SPAN = HIGH_SPAN_I[15:0];

    reg prep;
    reg negative;
    reg [15:0] magnitude;
    reg [15:0] divisor;
    reg [25:0] dividend;
    reg [15:0] remainder;
    reg [8:0] quotient;
    reg [4:0] bits_left;
    // At each step remainder < divisor. This 17-bit trial cannot overflow.
    wire [16:0] trial = {remainder, dividend[25]};
    wire subtract_divisor = trial >= {1'b0, divisor};
    wire [16:0] next_remainder = subtract_divisor ?
        trial - {1'b0, divisor} : trial;
    wire [9:0] next_quotient = {quotient, subtract_divisor};
    wire signed [11:0] unsigned_command = {2'b00, next_quotient};
    // next_remainder[16] is zero by the restoring-division invariant.
    wire unused_remainder_msb = next_remainder[16];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            command <= 12'sd0;
            busy <= 1'b0;
            done <= 1'b0;
            prep <= 1'b0;
            negative <= 1'b0;
            magnitude <= 16'd0;
            divisor <= 16'd1;
            dividend <= 26'd0;
            remainder <= 16'd0;
            quotient <= 9'd0;
            bits_left <= 5'd0;
        end else begin
            done <= 1'b0;
            if (cancel) begin
                busy <= 1'b0;
                prep <= 1'b0;
                command <= 12'sd0;
            end else if (start) begin
                busy <= 1'b1;
                prep <= 1'b1;
                negative <= (width_us < LOW_EDGE) ^ (REVERSE != 0);
                // Endpoints use the same divider and latency as every input.
                if (width_us <= LOW_LIMIT || width_us >= HIGH_LIMIT) begin
                    magnitude <= 16'd1000;
                    divisor <= 16'd1000;
                end else if (width_us < LOW_EDGE) begin
                    magnitude <= LOW_EDGE - width_us;
                    divisor <= LOW_SPAN;
                end else if (width_us > HIGH_EDGE) begin
                    magnitude <= width_us - HIGH_EDGE;
                    divisor <= HIGH_SPAN;
                end else begin
                    magnitude <= 16'd0;
                    divisor <= 16'd1;
                end
            end else if (busy && prep) begin
                // Largest legal product is below 2^26; constant multiplication
                // is pipelined separately from the iterative subtractor.
                dividend <= {10'd0, magnitude} * 26'd1000;
                remainder <= 16'd0;
                quotient <= 9'd0;
                bits_left <= 5'd25;
                prep <= 1'b0;
            end else if (busy) begin
                dividend <= {dividend[24:0], 1'b0};
                remainder <= next_remainder[15:0];
                quotient <= next_quotient[8:0];
                if (bits_left == 5'd0) begin
                    command <= negative ? -unsigned_command : unsigned_command;
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    bits_left <= bits_left - 1'b1;
                end
            end
        end
    end
endmodule
