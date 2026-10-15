`timescale 1ns/1ps
// Public-reader integration test uses the default 50 MHz clock and actual
// 460800 baud UART waveforms, not a direct injection into the packet parser.
module m10_radar_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg reset_n = 0, radar_rx = 1, point_ready = 0;
    wire point_valid, packet_valid, packet_error, packet_overflow;
    wire [15:0] point_angle_cdeg, point_distance_mm, packet_speed;
    wire [5:0] point_index;
    wire [2:0] packet_error_reason, last_error_reason;
    wire [7:0] rx_data, last_rx_byte;
    wire rx_valid, rx_error, data_active, error_sticky, overflow_sticky;
    wire [31:0] packet_count;
    wire [15:0] rx_byte_count, uart_error_count, bad_packet_count;
    m10_radar dut (
        .clk(clk), .reset_n(reset_n), .radar_rx(radar_rx),
        .point_valid(point_valid), .point_ready(point_ready),
        .point_angle_cdeg(point_angle_cdeg), .point_distance_mm(point_distance_mm),
        .point_index(point_index), .packet_valid(packet_valid),
        .packet_error(packet_error), .packet_overflow(packet_overflow),
        .packet_error_reason(packet_error_reason), .packet_speed(packet_speed),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_error(rx_error),
        .data_active(data_active), .error_sticky(error_sticky),
        .overflow_sticky(overflow_sticky), .packet_count(packet_count),
        .rx_byte_count(rx_byte_count), .uart_error_count(uart_error_count),
        .bad_packet_count(bad_packet_count), .last_rx_byte(last_rx_byte),
        .last_error_reason(last_error_reason)
    );

    reg [7:0] frame [0:101];
    reg [7:0] expected_raw [0:203];
    integer raw_expected = 0, raw_seen = 0, points_seen = 0;
    integer packets_seen = 0, errors_seen = 0, uart_errors_seen = 0;
    integer stalls_checked = 0, checks = 0, cycle_number = 0;
    integer expected_angle, expected_distance, k;
    reg automatic_stalls = 0, was_stalled = 0;
    reg [15:0] held_angle, held_distance;
    reg [5:0] held_index;

    always @(negedge clk) begin
        cycle_number = cycle_number + 1;
        if (automatic_stalls) point_ready = (cycle_number % 7) >= 3;
    end
    always @(posedge clk) begin
        if (!reset_n) begin
            was_stalled = 0;
            raw_seen = 0;
            points_seen = 0;
            packets_seen = 0;
            errors_seen = 0;
            uart_errors_seen = 0;
        end else begin
            if (was_stalled) begin
                if (!point_valid || point_angle_cdeg !== held_angle ||
                    point_distance_mm !== held_distance || point_index !== held_index)
                    $fatal(1,"point data/valid changed under backpressure");
                stalls_checked = stalls_checked + 1;
            end
            was_stalled = point_valid && !point_ready;
            held_angle = point_angle_cdeg;
            held_distance = point_distance_mm;
            held_index = point_index;
            if (rx_valid) begin
                if (raw_seen >= raw_expected || rx_data !== expected_raw[raw_seen])
                    $fatal(1,"raw UART byte mismatch at index %0d: got=%h",raw_seen,rx_data);
                raw_seen = raw_seen + 1;
            end
            if (packet_valid) begin
                if (packets_seen != 0 || packet_speed !== 16'h1068)
                    $fatal(1,"unexpected packet or wrong speed endian");
                packets_seen = packets_seen + 1;
            end
            if (packet_error) begin
                if ((errors_seen == 0 && packet_error_reason !== 3'd2) ||
                    (errors_seen == 1 && packet_error_reason !== 3'd1) || errors_seen >= 2)
                    $fatal(1,"unexpected packet error/reason: index=%0d reason=%0d",errors_seen,packet_error_reason);
                errors_seen = errors_seen + 1;
            end
            if (rx_error) uart_errors_seen = uart_errors_seen + 1;
            if (packet_overflow || overflow_sticky)
                $fatal(1,"reader overflowed with only one queued validated packet");
            if (point_valid && point_ready) begin
                if (points_seen >= 42) $fatal(1,"rejected packet leaked points");
                // Independent sector oracle: 41 valid distances plus FFFF
                // padding. The first angle crosses 360 degrees near slot 3.
                expected_angle = (35900 + points_seen * 1500 / 41) % 36000;
                expected_distance = points_seen == 41 ? 65535 : 1200 + points_seen * 10;
                if (point_index !== points_seen[5:0] ||
                    point_angle_cdeg !== expected_angle[15:0] ||
                    point_distance_mm !== expected_distance[15:0])
                    $fatal(1,"point %0d got index/angle/distance=%0d/%0d/%0d expected=%0d/%0d/%0d",
                           points_seen,point_index,point_angle_cdeg,point_distance_mm,
                           points_seen,expected_angle,expected_distance);
                points_seen = points_seen + 1;
                checks = checks + 1;
            end
        end
    end

    task make_frame;
        integer p, distance;
        begin
            frame[0] = 8'hA5; frame[1] = 8'h5A;
            frame[2] = 16'd35900 >> 8; frame[3] = 16'd35900;
            frame[4] = 8'h10; frame[5] = 8'h68;
            for (p = 0; p < 42; p = p + 1) begin
                if (p == 41) distance = 65535;
                else distance = (1200 + p*10) | ((p % 2) ? 32768 : 0);
                frame[6 + 2*p] = distance >> 8;
                frame[7 + 2*p] = distance;
            end
            // Timestamp bytes include apparent headers/tails without changing
            // the fixed packet length or publishing the packet prematurely.
            frame[90] = 8'hA5; frame[91] = 8'h5A;
            frame[92] = 8'hFA; frame[93] = 8'hFB;
            frame[94] = 0; frame[95] = 8'hA5; frame[96] = 8'h5A;
            frame[97] = 8'hFA; frame[98] = 8'hFB; frame[99] = 0;
            frame[100] = 8'hFA; frame[101] = 8'hFB;
        end
    endtask
    task send_byte;
        input [7:0] data;
        integer bit_index;
        begin
            radar_rx = 0; #2170.138889;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                radar_rx = data[bit_index]; #2170.138889;
            end
            radar_rx = 1; #2170.138889;
        end
    endtask
    task send_packet;
        input [7:0] tail;
        integer byte_index;
        begin
            frame[101] = tail;
            for (byte_index = 0; byte_index < 102; byte_index = byte_index + 1) begin
                expected_raw[raw_expected] = frame[byte_index];
                raw_expected = raw_expected + 1;
            end
            for (byte_index = 0; byte_index < 102; byte_index = byte_index + 1)
                send_byte(frame[byte_index]);
        end
    endtask
    task expect_diagnostics;
        input integer bytes, good, bad, uart;
        input [7:0] last_byte;
        input [2:0] reason;
        begin
            if (rx_byte_count !== bytes[15:0] || packet_count !== good[31:0] ||
                bad_packet_count !== bad[15:0] || uart_error_count !== uart[15:0] ||
                last_rx_byte !== last_byte || last_error_reason !== reason)
                $fatal(1,"diagnostic counters or last-byte/reason mismatch");
            checks = checks + 1;
        end
    endtask

    initial begin
        make_frame();
        repeat(4) @(negedge clk); reset_n = 1;
        repeat(300) @(negedge clk); // Idle long enough for recovery-enabled RX.
        send_packet(8'hFB);
        repeat(30) @(negedge clk);
        expect_diagnostics(102,1,0,0,8'hFB,0);
        if (!data_active || error_sticky || !point_valid || points_seen != 0)
            $fatal(1,"validated packet was not retained while stalled");
        send_packet(8'hFC); // Still stalled: receive/parser must keep running.
        repeat(30) @(negedge clk);
        expect_diagnostics(204,1,1,0,8'hFC,2);
        if (raw_seen != 204 || packets_seen != 1 || errors_seen != 1 ||
            !error_sticky || !data_active || points_seen != 0)
            $fatal(1,"bad-tail validation or independent UART reception failed");
        automatic_stalls = 1;
        wait(points_seen == 42);
        repeat(10) @(negedge clk);
        automatic_stalls = 0; point_ready = 0;
        if (point_valid || stalls_checked < 100)
            $fatal(1,"point queue did not drain or stalls were not exercised");

        // One sustained break is one UART fault, without fabricated raw bytes.
        radar_rx = 0; #30000;
        radar_rx = 1; #6000;
        repeat(30) @(negedge clk);
        expect_diagnostics(204,1,2,1,8'hFC,1);
        if (uart_errors_seen != 1 || errors_seen != 2 || !data_active)
            $fatal(1,"sustained-break error accounting failed");

        @(negedge clk); reset_n = 0;
        repeat(4) @(negedge clk);
        expect_diagnostics(0,0,0,0,0,0);
        if (point_valid || packet_valid || packet_error || packet_overflow ||
            rx_valid || rx_error || data_active || error_sticky || overflow_sticky)
            $fatal(1,"reset did not clear reader state and diagnostic flags");
        $display("PASS m10_radar_tb: %0d checks, real 460800 baud GPS102, 204 raw bytes, 42 wrapped/intensity/padded points, %0d stable stalls, bad tail, break and reset",checks,stalls_checked);
        $finish;
    end
    initial begin #20000000; $fatal(1,"public reader test timeout"); end
endmodule
