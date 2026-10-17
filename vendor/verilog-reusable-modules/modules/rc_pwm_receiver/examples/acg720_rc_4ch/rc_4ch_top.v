`timescale 1ns / 1ps

// Four independent receiver PWM inputs, observation only; no motor commands.
// Left display: channel 0001..0004. Right: its valid pulse width in us.
// sw0=0 cycles channels; sw0=1 freezes the current channel.
module rc_4ch_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer TIMEOUT_US = 100000,
    parameter integer CHANNEL_US = 10000000,
    parameter integer DIGIT_US = 500,
    parameter integer SNAPSHOT_US = 1000,
    parameter integer SCLK_DIVIDER = 25,
    parameter integer HEARTBEAT_US = 500000
) (
    input wire clk,
    input wire rst_n,
    input wire [3:0] rc_pwm,
    input wire sw0,
    output wire ds,
    output wire sh_cp,
    output wire st_cp,
    output wire [7:0] led
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

    // Positive exact-MHz clock and intervals; products must fit signed integer.
    localparam integer CHANNEL_CYCLES_I = (CLK_HZ / 1000000) * CHANNEL_US;
    localparam integer HEARTBEAT_CYCLES_I = (CLK_HZ / 1000000) * HEARTBEAT_US;
    localparam integer CHANNEL_BITS = counter_width(CHANNEL_CYCLES_I - 1);
    localparam integer HEARTBEAT_BITS = counter_width(HEARTBEAT_CYCLES_I - 1);
    localparam [CHANNEL_BITS-1:0] CHANNEL_LAST =
        CHANNEL_CYCLES_I[CHANNEL_BITS-1:0] - 1'b1;
    localparam [HEARTBEAT_BITS-1:0] HEARTBEAT_LAST =
        HEARTBEAT_CYCLES_I[HEARTBEAT_BITS-1:0] - 1'b1;

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

    reg sw0_meta;
    reg sw0_sync;
    always @(posedge clk or negedge reset_sync) begin
        if (!reset_sync) begin
            sw0_meta <= 1'b0;
            sw0_sync <= 1'b0;
        end else begin
            sw0_meta <= sw0;
            sw0_sync <= sw0_meta;
        end
    end

    wire [15:0] widths [0:3];
    wire [3:0] channel_valid;
    wire [3:0] unused_timed_out;
    wire [3:0] unused_pulse_update;
    genvar ch;
    generate
        for (ch = 0; ch < 4; ch = ch + 1) begin : capture_channels
            rc_pwm_capture #(
                .CLK_HZ(CLK_HZ), .TIMEOUT_US(TIMEOUT_US)
            ) capture (
                .clk(clk), .rst_n(rst_n), .rc_pwm(rc_pwm[ch]),
                .pulse_width_us(widths[ch]), .pulse_valid(channel_valid[ch]),
                .timed_out(unused_timed_out[ch]),
                .pulse_update(unused_pulse_update[ch])
            );
        end
    endgenerate

    reg [1:0] current_channel;
    reg [CHANNEL_BITS-1:0] channel_counter;
    reg [HEARTBEAT_BITS-1:0] heartbeat_counter;
    reg heartbeat;
    always @(posedge clk or negedge reset_sync) begin
        if (!reset_sync) begin
            current_channel <= 2'd0;
            channel_counter <= {CHANNEL_BITS{1'b0}};
            heartbeat_counter <= {HEARTBEAT_BITS{1'b0}};
            heartbeat <= 1'b0;
        end else begin
            if (sw0_sync) begin
                channel_counter <= {CHANNEL_BITS{1'b0}};
            end else if (channel_counter == CHANNEL_LAST) begin
                channel_counter <= {CHANNEL_BITS{1'b0}};
                current_channel <= current_channel + 1'b1;
            end else begin
                channel_counter <= channel_counter + 1'b1;
            end
            if (heartbeat_counter == HEARTBEAT_LAST) begin
                heartbeat_counter <= {HEARTBEAT_BITS{1'b0}};
                heartbeat <= ~heartbeat;
            end else begin
                heartbeat_counter <= heartbeat_counter + 1'b1;
            end
        end
    end

    wire [15:0] selected_width_us = widths[current_channel];
    wire selected_valid = channel_valid[current_channel];
    wire [15:0] channel_number = {14'd0, current_channel} + 16'd1;
    rc_hc595_display #(
        .CLK_HZ(CLK_HZ), .SNAPSHOT_US(SNAPSHOT_US),
        .DIGIT_US(DIGIT_US), .SCLK_DIVIDER(SCLK_DIVIDER)
    ) display (
        .clk(clk), .rst_n(reset_sync),
        .ch1_width_us(channel_number), .ch2_width_us(selected_width_us),
        .ch1_valid(1'b1), .ch2_valid(selected_valid),
        .ds(ds), .sh_cp(sh_cp), .st_cp(st_cp)
    );

    assign led[3:0] = channel_valid;
    assign led[5:4] = current_channel;
    assign led[6] = selected_valid &&
        ((selected_width_us < 16'd1475) || (selected_width_us > 16'd1525));
    assign led[7] = heartbeat;
endmodule
