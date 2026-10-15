`timescale 1ns / 1ps

// Single-channel RC PWM pulse capture. All logic uses clk; no derived clock.
// Parameter contract:
//   CLK_HZ is a positive, exact multiple of 1,000,000 Hz.
//   1 <= MIN_PULSE_US <= MAX_PULSE_US <= 65535.
//   TIMEOUT_US > MAX_PULSE_US.
//   MAX_PULSE_US * (CLK_HZ / 1000000) + 1 and
//   TIMEOUT_US * (CLK_HZ / 1000000) fit a positive signed 32-bit integer.
//
// pulse_valid is a level: a legal completed pulse makes it 1; an illegal
// pulse, an overlong high level, reset, or freshness timeout makes it 0.
// pulse_update is a one-clock strobe for each legal completed pulse.
// timed_out becomes 1 after TIMEOUT_US without a legal completed pulse,
// including when no pulse has been received since reset. Illegal pulses do
// not restart this timer. A new legal pulse clears timed_out.
// pulse_width_us retains the last legal width even after invalidation; use
// pulse_valid, never pulse_width_us alone, to authorize a command.
module rc_pwm_capture #(
    parameter integer CLK_HZ       = 50000000,
    parameter integer MIN_PULSE_US = 900,
    parameter integer MAX_PULSE_US = 2100,
    parameter integer TIMEOUT_US   = 100000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rc_pwm,
    output reg  [15:0] pulse_width_us,
    output reg         pulse_valid,
    output reg         timed_out,
    output reg         pulse_update
);

    function integer counter_width;
        input integer maximum;
        integer remaining;
        begin
            remaining = maximum;
            counter_width = 0;
            while (remaining > 0) begin
                counter_width = counter_width + 1;
                remaining = remaining / 2;
            end
            if (counter_width < 1)
                counter_width = 1;
        end
    endfunction

    localparam integer CYCLES_PER_US = CLK_HZ / 1000000;
    localparam integer MIN_CYCLES_I = MIN_PULSE_US * CYCLES_PER_US;
    localparam integer MAX_CYCLES_I = MAX_PULSE_US * CYCLES_PER_US;
    localparam integer PULSE_LIMIT_I = MAX_CYCLES_I + 1;
    localparam integer TIMEOUT_CYCLES_I = TIMEOUT_US * CYCLES_PER_US;
    localparam integer PULSE_BITS = counter_width(PULSE_LIMIT_I);
    localparam integer TIMEOUT_BITS = counter_width(TIMEOUT_CYCLES_I);

    localparam [PULSE_BITS-1:0] MIN_CYCLES = MIN_CYCLES_I[PULSE_BITS-1:0];
    localparam [PULSE_BITS-1:0] MAX_CYCLES = MAX_CYCLES_I[PULSE_BITS-1:0];
    localparam [PULSE_BITS-1:0] PULSE_LIMIT = PULSE_LIMIT_I[PULSE_BITS-1:0];
    localparam [TIMEOUT_BITS-1:0] TIMEOUT_CYCLES =
        TIMEOUT_CYCLES_I[TIMEOUT_BITS-1:0];

    // Asynchronous assertion, two-clock synchronous reset release.
    // Each capture instance handles reset release in its own clock domain.
    reg reset_meta;
    reg reset_sync;
    wire local_rst_n = reset_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_meta <= 1'b0;
            reset_sync <= 1'b0;
        end else begin
            reset_meta <= 1'b1;
            reset_sync <= reset_meta;
        end
    end

    // The external PWM is asynchronous. Only the second flip-flop's
    // output is used by edge detection and pulse measurement.
    reg pwm_meta;
    reg pwm_sync;
    reg pwm_previous;
    wire pwm_rising  =  pwm_sync & ~pwm_previous;
    wire pwm_falling = ~pwm_sync &  pwm_previous;

    always @(posedge clk or negedge local_rst_n) begin
        if (!local_rst_n) begin
            pwm_meta <= 1'b0;
            pwm_sync <= 1'b0;
        end else begin
            pwm_meta <= rc_pwm;
            pwm_sync <= pwm_meta;
        end
    end

    reg [1:0] sync_warmup;
    reg       armed;
    reg       measuring;
    reg [PULSE_BITS-1:0] pulse_cycles;
    reg [TIMEOUT_BITS-1:0] freshness_cycles;

    wire [31:0] pulse_cycles_extended =
        {{(32-PULSE_BITS){1'b0}}, pulse_cycles};
    // The result's high 16 bits are intentionally unused: the parameter
    // contract and raw-cycle legality check limit accepted widths to 65535.
    wire [31:0] unused_completed_width_us = pulse_cycles_extended / CYCLES_PER_US;

    always @(posedge clk or negedge local_rst_n) begin
        if (!local_rst_n) begin
            pwm_previous    <= 1'b0;
            sync_warmup     <= 2'd0;
            armed           <= 1'b0;
            measuring       <= 1'b0;
            pulse_cycles    <= {PULSE_BITS{1'b0}};
            freshness_cycles <= {TIMEOUT_BITS{1'b0}};
            pulse_width_us  <= 16'd0;
            pulse_valid     <= 1'b0;
            timed_out       <= 1'b0;
            pulse_update    <= 1'b0;
        end else begin
            pwm_previous <= pwm_sync;
            pulse_update <= 1'b0;

            // Saturating last-good-pulse age. A legal falling edge below
            // has priority over timeout on the same clock.
            if (freshness_cycles >= TIMEOUT_CYCLES - 1'b1) begin
                freshness_cycles <= TIMEOUT_CYCLES;
                timed_out <= 1'b1;
                pulse_valid <= 1'b0;
            end else begin
                freshness_cycles <= freshness_cycles + 1'b1;
            end

            // Wait at least three active clocks for the synchronizer,
            // then observe a real low level before accepting any rise.
            // This prevents accepting a partial high pulse after reset.
            if (sync_warmup != 2'd3) begin
                sync_warmup <= sync_warmup + 1'b1;
            end else if (!armed) begin
                if (!pwm_sync)
                    armed <= 1'b1;
            end else if (pwm_rising) begin
                measuring <= 1'b1;
                // Include the first sampled high clock in the width.
                pulse_cycles <= {{(PULSE_BITS-1){1'b0}}, 1'b1};
            end else if (pwm_falling && measuring) begin
                measuring <= 1'b0;
                // Compare raw clock counts, not truncated microseconds.
                // For example, MAX_PULSE_US + 20 ns is rejected at 50 MHz.
                if (pulse_cycles >= MIN_CYCLES && pulse_cycles <= MAX_CYCLES) begin
                    pulse_width_us <= unused_completed_width_us[15:0];
                    pulse_valid <= 1'b1;
                    pulse_update <= 1'b1;
                    timed_out <= 1'b0;
                    freshness_cycles <= {TIMEOUT_BITS{1'b0}};
                end else begin
                    pulse_valid <= 1'b0;
                end
            end else if (measuring && pwm_sync) begin
                // Saturate at one clock beyond the largest legal width.
                // A stuck-high input therefore cannot wrap into validity.
                if (pulse_cycles >= MAX_CYCLES) begin
                    pulse_cycles <= PULSE_LIMIT;
                    pulse_valid <= 1'b0;
                end else begin
                    pulse_cycles <= pulse_cycles + 1'b1;
                end
            end
        end
    end

endmodule
