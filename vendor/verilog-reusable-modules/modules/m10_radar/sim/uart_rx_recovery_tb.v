`timescale 1ns/1ps
// Real 460800 8N1 traffic, including reset release inside a continuous stream.
// The old level-triggered receiver loses framing indefinitely for 0A AA / 15 55.
module uart_rx_recovery_tb;
    localparam integer PHASES = 40;
    localparam integer PACKET_BITS = 102 * 10;
    reg clk = 0;
    always #10.204081633 clk = ~clk; // 49 MHz
    reg radar_rx = 1;
    reg [PHASES-1:0] reset_n = 0;
    reg scan_active = 0;
    reg check_bytes = 0;
    reg check_points = 0;
    real bit_ns = 2170.138889;
    real case_start;
    real release_time [0:PHASES-1];
    integer uart_errors [0:PHASES-1];
    integer late_uart_errors [0:PHASES-1];
    integer valid_packets [0:PHASES-1];
    integer late_packets [0:PHASES-1];
    integer good_bytes [0:PHASES-1];
    integer received_points [0:PHASES-1];
    reg [15:0] expected_distance;
    integer expected_byte;
    integer sample, rate, cases_run = 0;

    always @(posedge clk) begin
        if (scan_active) begin
            for (sample = 0; sample < PHASES; sample = sample + 1)
                if (!reset_n[sample] && $realtime >= release_time[sample])
                    reset_n[sample] <= 1;
        end
    end

    genvar g;
    generate for (g = 0; g < PHASES; g = g + 1) begin: receiver
        wire [7:0] byte_data;
        wire byte_valid, byte_error, packet_valid, point_valid;
        wire [15:0] distance, angle;
        wire [5:0] point_index;
        uart_byte_rx #(.CLK_FREQ_HZ(49_000_000), .BAUD_RATE(460800), .RECOVERY_EN(1)) uart (
            .i_clock(clk), .i_reset_n(reset_n[g]), .i_uart_rx(radar_rx),
            .o_rx_data(byte_data), .o_rx_valid(byte_valid), .i_rx_ready(1'b1),
            .o_rx_error(byte_error)
        );
        m10_packet_parser #(.CLK_FREQ_HZ(49_000_000)) parser (
            .clk(clk), .rst_n(reset_n[g]), .rx_data(byte_data),
            .rx_valid(byte_valid), .rx_error(byte_error),
            .point_valid(point_valid), .point_ready(1'b1),
            .point_angle_cdeg(angle), .point_distance_mm(distance),
            .point_index(point_index), .packet_valid(packet_valid),
            .packet_error(), .packet_overflow(), .packet_speed(), .packet_error_reason()
        );
        always @(posedge clk) if (reset_n[g]) begin
            if (byte_error) begin
                uart_errors[g] = uart_errors[g] + 1;
                if (scan_active && $realtime >= case_start + 2.0 * PACKET_BITS * bit_ns)
                    late_uart_errors[g] = late_uart_errors[g] + 1;
            end
            if (byte_valid) begin
                good_bytes[g] = good_bytes[g] + 1;
                if (check_bytes && g == 0) begin
                    if (byte_data !== (expected_byte & 255))
                        $fatal(1, "UART byte mismatch: got %h expected %h", byte_data, expected_byte & 255);
                    expected_byte = expected_byte + 1;
                end
            end
            if (packet_valid) begin
                valid_packets[g] = valid_packets[g] + 1;
                if (scan_active && $realtime >= case_start + 2.0 * PACKET_BITS * bit_ns)
                    late_packets[g] = late_packets[g] + 1;
            end
            if (point_valid && check_points) begin
                received_points[g] = received_points[g] + 1;
                if (distance !== expected_distance || angle !== (point_index * 1500) / 42)
                    $fatal(1, "Recovered packet point corrupted at phase %0d: %0d mm/%0d cdeg index %0d", g, distance, angle, point_index);
            end
        end
    end endgenerate

    task clear_counters;
        integer k;
        begin
            for (k = 0; k < PHASES; k = k + 1) begin
                uart_errors[k] = 0; late_uart_errors[k] = 0;
                valid_packets[k] = 0; late_packets[k] = 0;
                good_bytes[k] = 0; received_points[k] = 0;
            end
        end
    endtask
    task send_byte;
        input [7:0] data;
        integer b;
        begin
            radar_rx = 0; #(bit_ns);
            for (b = 0; b < 8; b = b + 1) begin
                radar_rx = data[b]; #(bit_ns);
            end
            radar_rx = 1; #(bit_ns); // Exactly one stop bit; no extra gap.
        end
    endtask
    task send_packet;
        input [15:0] distance;
        integer p;
        begin
            send_byte(8'hA5); send_byte(8'h5A);
            send_byte(8'h00); send_byte(8'h00); // Start angle 0 cdeg.
            send_byte(8'h10); send_byte(8'h68);
            for (p = 0; p < 42; p = p + 1) begin
                send_byte(distance[15:8]); send_byte(distance[7:0]);
            end
            // Ten time bytes taken from the real captured M10_GPS stream.
            send_byte(0); send_byte(0); send_byte(0); send_byte(0);
            send_byte(0); send_byte(0); send_byte(8'h02); send_byte(8'hC8);
            send_byte(8'h02); send_byte(8'hA6);
            send_byte(8'hFA); send_byte(8'hFB);
        end
    endtask
    task startup_case;
        input [15:0] distance;
        input integer rate_permille;
        integer k, p;
        begin
            scan_active = 0; check_bytes = 0; check_points = 1;
            reset_n = 0; radar_rx = 1;
            bit_ns = 2170.138889 * 1000.0 / rate_permille;
            expected_distance = distance;
            clear_counters();
            #10000;
            case_start = $realtime;
            // Spread resets over every part of the first 102-byte packet.
            for (k = 0; k < PHASES; k = k + 1)
                release_time[k] = case_start + (k + 0.5) * PACKET_BITS * bit_ns / PHASES;
            scan_active = 1;
            for (p = 0; p < 6; p = p + 1) send_packet(distance);
            #(3.0 * bit_ns);
            scan_active = 0;
            for (k = 0; k < PHASES; k = k + 1) begin
                if (late_packets[k] != 4 || late_uart_errors[k] != 0)
                    $fatal(1, "Persistent receive failure: rate=%0d distance=%h phase=%0d packets=%0d late=%0d UART errors=%0d", rate_permille, distance, k, valid_packets[k], late_packets[k], late_uart_errors[k]);
                if (received_points[k] != valid_packets[k] * 42)
                    $fatal(1, "Recovered packet dropped points at phase %0d", k);
            end
            cases_run = cases_run + PHASES;
        end
    endtask
    task steady_and_break_case;
        input integer rate_permille;
        integer b;
        begin
            scan_active = 0; check_points = 0; check_bytes = 0;
            reset_n = 0; radar_rx = 1;
            bit_ns = 2170.138889 * 1000.0 / rate_permille;
            clear_counters();
            #10000;
            reset_n[0] = 1;
            #(3.0 * bit_ns);
            check_bytes = 1; expected_byte = 0;
            for (b = 0; b < 256; b = b + 1) send_byte(b);
            #(3.0 * bit_ns);
            check_bytes = 0;
            if (expected_byte != 256 || uart_errors[0] != 0)
                $fatal(1, "Continuous 256-byte test failed at rate=%0d: bytes=%0d errors=%0d", rate_permille, expected_byte, uart_errors[0]);
            clear_counters();
            radar_rx = 0;
            #(100.0 * bit_ns); // Continuous low break must produce one error.
            if (uart_errors[0] != 1 || good_bytes[0] != 0)
                $fatal(1, "Break caused an error storm: errors=%0d bytes=%0d", uart_errors[0], good_bytes[0]);
            radar_rx = 1;
            #(3.0 * bit_ns);
            expected_distance = 2730;
            check_points = 1;
            send_packet(expected_distance);
            #(3.0 * bit_ns);
            if (uart_errors[0] != 1 || valid_packets[0] != 1 || received_points[0] != 42)
                $fatal(1, "Receiver failed after break at rate=%0d", rate_permille);
        end
    endtask

    initial begin
        for (rate = 990; rate <= 1010; rate = rate + 10) begin
            startup_case(16'h0AAA, rate);
            startup_case(16'h1555, rate);
            startup_case(16'h0FA0, rate);
            startup_case(16'h0000, rate);
            steady_and_break_case(rate);
        end
        $display("PASS uart_rx_recovery_tb: %0d startup phases, 460800bps +/-1%%, continuous 8N1 bytes, validated points, single-error break recovery", cases_run);
        $finish;
    end
    initial begin
        #500000000;
        $fatal(1, "UART recovery simulation timeout");
    end
endmodule
