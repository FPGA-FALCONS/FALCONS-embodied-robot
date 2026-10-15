`timescale 1ns/1ps
module m10_gps_packet_parser_tb;
    localparam integer TIMEOUT_CLKS = 24;
    localparam integer CAPTURE_PACKETS = 719;
    localparam integer CAPTURE_BYTES = CAPTURE_PACKETS * 102;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg [7:0] rx_data = 0;
    reg rx_valid = 0, rx_error = 0, point_ready = 1;
    wire point_valid, packet_valid, packet_error, packet_overflow;
    wire [15:0] point_angle_cdeg, point_distance_mm, packet_speed;
    wire [5:0] point_index;
    wire [2:0] packet_error_reason;

    m10_packet_parser #(.CLK_FREQ_HZ(49000000),
        .BYTE_TIMEOUT_CLKS(TIMEOUT_CLKS)) dut (
        .clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_valid(rx_valid),
        .rx_error(rx_error), .point_valid(point_valid), .point_ready(point_ready),
        .point_angle_cdeg(point_angle_cdeg), .point_distance_mm(point_distance_mm),
        .point_index(point_index), .packet_valid(packet_valid),
        .packet_error(packet_error), .packet_error_reason(packet_error_reason),
        .packet_overflow(packet_overflow), .packet_speed(packet_speed));

    reg [7:0] frame [0:101];
    reg [7:0] capture [0:CAPTURE_BYTES-1];
    reg [15:0] expected_angle [0:32767];
    reg [15:0] expected_distance [0:32767];
    reg [5:0] expected_index [0:32767];
    reg [15:0] expected_speed [0:1023];
    reg [2:0] expected_reason [0:31];
    integer expected_tail = 0, actual_head = 0;
    integer expected_packets = 0, actual_packets = 0;
    integer expected_errors = 0, actual_errors = 0, actual_overflows = 0;
    integer failures = 0;
    integer checkpoint, point_checkpoint, packet_checkpoint, error_checkpoint;
    integer packet_number, byte_number, valid_slots, slot_number, cycle_number = 0;
    reg automatic_stalls = 0, was_stalled = 0;
    reg [15:0] stalled_angle, stalled_distance;
    reg [5:0] stalled_index;
    reg [2:0] previous_reason = 0;

    task fail;
        input [511:0] message;
        begin
            failures = failures + 1;
            $display("FAIL: %0s at %0t", message, $time);
        end
    endtask

    always @(negedge clk) begin
        cycle_number = cycle_number + 1;
        if (automatic_stalls) point_ready = ((cycle_number % 7) >= 3);
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (was_stalled && (!point_valid || point_angle_cdeg !== stalled_angle ||
                point_distance_mm !== stalled_distance || point_index !== stalled_index))
                fail("GPS point changed under backpressure");
            was_stalled = point_valid && !point_ready;
            stalled_angle = point_angle_cdeg;
            stalled_distance = point_distance_mm;
            stalled_index = point_index;
            if (point_valid && point_ready) begin
                if (actual_head >= expected_tail) fail("unvalidated GPS point escaped");
                else if (point_angle_cdeg !== expected_angle[actual_head] ||
                         point_distance_mm !== expected_distance[actual_head] ||
                         point_index !== expected_index[actual_head]) begin
                    $display("point %0d: got angle/distance/index %0d/%0d/%0d; expected %0d/%0d/%0d",
                        actual_head, point_angle_cdeg, point_distance_mm, point_index,
                        expected_angle[actual_head], expected_distance[actual_head], expected_index[actual_head]);
                    fail("GPS point or FIFO ordering mismatch");
                end
                actual_head = actual_head + 1;
            end
            if (packet_valid) begin
                if (actual_packets >= expected_packets) fail("unexpected GPS packet_valid");
                else if (packet_speed !== expected_speed[actual_packets]) fail("GPS speed endian mismatch");
                actual_packets = actual_packets + 1;
            end
            if (packet_error_reason !== previous_reason && !packet_error)
                fail("GPS error reason changed without error pulse");
            previous_reason = packet_error_reason;
            if (packet_error) begin
                if (actual_errors >= expected_errors) fail("unexpected GPS packet_error");
                else if (packet_error_reason !== expected_reason[actual_errors]) fail("wrong GPS error reason");
                actual_errors = actual_errors + 1;
            end
            if (packet_overflow) begin
                actual_overflows = actual_overflows + 1;
                if (!packet_error || packet_error_reason !== 3'd5) fail("GPS overflow reason missing");
            end
        end else begin
            was_stalled = 0;
            previous_reason = 0;
        end
    end

    task idle_cycles;
        input integer count;
        integer k;
        begin
            for (k = 0; k < count; k = k + 1) @(negedge clk);
        end
    endtask

    task send_byte;
        input [7:0] value;
        begin
            @(negedge clk);
            rx_data = value;
            rx_valid = 1;
            @(negedge clk);
            rx_valid = 0;
        end
    endtask

    task expect_error;
        input [2:0] reason;
        begin
            expected_reason[expected_errors] = reason;
            expected_errors = expected_errors + 1;
        end
    endtask

    task make_frame;
        input integer angle, speed, distance_base;
        integer k, d;
        begin
            frame[0] = 8'hA5; frame[1] = 8'h5A;
            frame[2] = angle >> 8; frame[3] = angle;
            frame[4] = speed >> 8; frame[5] = speed;
            for (k = 0; k < 42; k = k + 1) begin
                case (k)
                    0: d = 16'hFFFF;
                    1: d = 0;
                    2: d = 16'h8001;
                    3: d = 16'hA55A;
                    4: d = 16'hFAFB;
                    default: d = (distance_base + k) | ((k % 2) ? 16'h8000 : 0);
                endcase
                frame[6 + 2*k] = d >> 8;
                frame[7 + 2*k] = d;
            end
            // False headers and tails inside the ten timestamp bytes.
            frame[90] = 8'hA5; frame[91] = 8'h5A;
            frame[92] = 8'hFA; frame[93] = 8'hFB;
            frame[94] = 0; frame[95] = 8'hA5; frame[96] = 8'hA5;
            frame[97] = 8'h5A; frame[98] = 8'hFA; frame[99] = 8'hFB;
            frame[100] = 8'hFA; frame[101] = 8'hFB;
        end
    endtask

    task expect_frame;
        integer k, a, d, first_angle, noninvalid_count;
        reg [15:0] decoded_distance [0:41];
        begin
            first_angle = {frame[2], frame[3]};
            if (first_angle == 36000) first_angle = 0;
            noninvalid_count = 0;
            for (k = 0; k < 42; k = k + 1) begin
                d = {frame[6 + 2*k], frame[7 + 2*k]};
                if ((k == 40) && (({frame[84], frame[85]} == 16'hFFFF) ||
                                    (frame[86] >= 8'hF5))) d = 65535;
                if (d != 65535) begin
                    noninvalid_count = noninvalid_count + 1;
                    d = d & 32767;
                end
                decoded_distance[k] = d;
            end
            for (k = 0; k < 42; k = k + 1) begin
                // Independent integer oracle; the DUT uses a fractional accumulator.
                if (noninvalid_count == 0) a = first_angle;
                else a = (first_angle + (k * 1500) / noninvalid_count) % 36000;
                expected_angle[expected_tail] = a;
                expected_distance[expected_tail] = decoded_distance[k];
                expected_index[expected_tail] = k;
                expected_tail = expected_tail + 1;
            end
            expected_speed[expected_packets] = {frame[4], frame[5]};
            expected_packets = expected_packets + 1;
        end
    endtask

    task send_prefix;
        input integer count;
        integer k;
        begin
            for (k = 0; k < count; k = k + 1) send_byte(frame[k]);
        end
    endtask

    task send_frame;
        input integer accepted;
        begin
            if (accepted) expect_frame();
            send_prefix(102);
        end
    endtask

    task drain;
        input integer use_stalls;
        integer k;
        begin
            k = 0;
            while ((actual_head < expected_tail || point_valid) && k < 2000) begin
                @(negedge clk);
                point_ready = !use_stalls || ((k % 7) >= 3);
                k = k + 1;
            end
            @(negedge clk);
            point_ready = 1;
            idle_cycles(3);
            if (actual_head != expected_tail) fail("GPS drain timed out");
            if (actual_packets != expected_packets) fail("GPS packet count mismatch");
        end
    endtask

    initial begin
        idle_cycles(4);
        rst_n = 1;
        idle_cycles(2);
        if (packet_error_reason !== 0) fail("GPS error reason reset mismatch");

        send_byte(8'h11);
        send_byte(8'hA5);
        make_frame(0, 16'h1068, 1000);
        expect_frame();
        send_prefix(100);
        idle_cycles(3);
        if (point_valid || actual_packets || actual_head)
            fail("timestamp markers prematurely validated a GPS packet");
        send_byte(frame[100]); send_byte(frame[101]);
        drain(1);
        make_frame(35999, 16'hA55A, 2000); send_frame(1); drain(1);
        make_frame(36000, 16'hFAFB, 3000); send_frame(1); drain(0);

        expect_error(3'd2);
        make_frame(1500, 4200, 4000); frame[100] = 0; send_frame(0);
        make_frame(1500, 4200, 5000); send_frame(1); drain(0);

        // A bad tail takes priority over an invalid angle, and A5 overlaps.
        expect_error(3'd2);
        make_frame(36001, 3000, 6000); frame[101] = 8'hA5; send_frame(0);
        make_frame(3000, 3000, 7000); expect_frame();
        for (byte_number = 1; byte_number < 102; byte_number = byte_number + 1)
            send_byte(frame[byte_number]);
        drain(1);

        expect_error(3'd4);
        make_frame(4500, 4200, 8000); send_prefix(95);
        idle_cycles(TIMEOUT_CLKS + 4);
        make_frame(4500, 4200, 9000); send_frame(1); drain(0);

        expect_error(3'd3);
        make_frame(36001, 4200, 10000); send_frame(0); idle_cycles(4);

        expect_error(3'd1);
        make_frame(6000, 4200, 11000); send_prefix(93);
        @(negedge clk); rx_error = 1; rx_valid = 1; rx_data = 8'hA5;
        @(negedge clk); rx_error = 0; rx_valid = 0;
        make_frame(6000, 4200, 12000); send_frame(1); drain(1);

        @(negedge clk); point_ready = 0;
        make_frame(7200, 4200, 13000); send_frame(1);
        make_frame(9000, 4100, 14000); send_frame(1);
        expect_error(3'd5);
        make_frame(18000, 4000, 15000); send_frame(0);
        idle_cycles(4); drain(1);
        if (actual_overflows != 1) fail("GPS two-bank overflow count mismatch");
        make_frame(27000, 16'h125A, 16000); send_frame(1); drain(1);
        if (actual_head != 420 || actual_packets != 10 || actual_errors != 6)
            fail("GPS synthetic totals mismatch");

        // Exercise every constant quotient/remainder entry, including the
        // observed 40/41-slot sectors, all 42 valid slots and an all-FFFF packet.
        for (valid_slots = 0; valid_slots <= 42; valid_slots = valid_slots + 1) begin
            make_frame(35990, 16'hF123, 2000);
            for (slot_number = 0; slot_number < 42; slot_number = slot_number + 1) begin
                if (slot_number < valid_slots) begin
                    frame[6 + 2*slot_number] = 8'h20;
                    frame[7 + 2*slot_number] = slot_number;
                end else begin
                    frame[6 + 2*slot_number] = 8'hFF;
                    frame[7 + 2*slot_number] = 8'hFF;
                end
            end
            send_frame(1); drain(1);
        end

        // SDK special slot-40 invalidation: test both its >=F5 high byte and
        // a preceding FFFF followed by a non-FFFF distance.
        make_frame(12000, 4200, 3000);
        frame[86] = 8'hF5; frame[87] = 0;
        send_frame(1); drain(0);
        make_frame(24000, 4200, 3000);
        frame[84] = 8'hFF; frame[85] = 8'hFF;
        send_frame(1); drain(0);

        // Replay all complete packets extracted from the user's actual capture.
        $readmemh("m10_capture.hex", capture);
        if (^capture[0] === 1'bx || ^capture[CAPTURE_BYTES-1] === 1'bx)
            fail("GPS capture fixture missing or incomplete");
        point_checkpoint = actual_head;
        packet_checkpoint = actual_packets;
        error_checkpoint = actual_errors;
        automatic_stalls = 1;
        for (packet_number = 0; packet_number < CAPTURE_PACKETS; packet_number = packet_number + 1) begin
            for (byte_number = 0; byte_number < 102; byte_number = byte_number + 1)
                frame[byte_number] = capture[packet_number * 102 + byte_number];
            if (frame[0] !== 8'hA5 || frame[1] !== 8'h5A ||
                frame[100] !== 8'hFA || frame[101] !== 8'hFB)
                fail("GPS capture frame boundary mismatch");
            send_frame(1);
        end
        automatic_stalls = 0;
        drain(1);
        if (actual_head - point_checkpoint != 30198 ||
            actual_packets - packet_checkpoint != 719 || actual_errors != error_checkpoint)
            fail("GPS capture replay must validate 719 packets and 30198 points without errors");
        if (actual_errors != expected_errors) fail("GPS expected error count mismatch");
        if (failures == 0)
            $display("PASS m10_gps_packet_parser_tb: 719 captured packets / 30198 captured points, %0d total points, %0d expected synthetic errors",
                actual_head, actual_errors);
        else $display("FAIL: m10_gps_packet_parser_tb, %0d failures", failures);
        $finish;
    end

    initial begin
        #5000000;
        $display("FAIL: GPS parser simulation watchdog expired");
        $finish;
    end
endmodule
