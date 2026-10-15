`timescale 1ns/1ps
// UART pins -> validated packets -> Cartesian conversion -> bitmap -> RGB.
// Vendor PLL and encrypted TMDS PHY are verified separately by Gowin build.
module m10_point_cloud_core_tb;
    reg clk = 0;
    always #10.204081633 clk = ~clk; // 49 MHz
    reg reset_n = 0;
    reg radar_rx = 1;
    reg zoom_in_n = 1, zoom_out_n = 1;
    wire [11:0] x, y;
    wire de, hs, vs, frame_start;
    wire [23:0] rgb;
    wire out_de, out_hs, out_vs, data_active, error_sticky, overflow_sticky;
    wire [15:0] front_points;
    wire bitmap_ready;
    wire debug_uart_tx, debug_rx_valid, debug_rx_error;
    wire [7:0] debug_rx_data;
    reg [7:0] mirrored_bytes [0:4095];
    integer mirror_written = 0, mirror_read = 0;
    integer packets_seen = 0;
    integer points_seen = 0;
    integer frames_seen = 0;
    integer pixels_seen = 0;
    integer cardinal_seen = 0;
    integer alignment_checks = 0, zoom_changes = 0;
    integer image_file;
    reg capture = 0;
    reg capture_done = 0;
    reg [11:0] x_d1, x_d2, x_d3, y_d1, y_d2, y_d3;
    reg de_d1, de_d2, de_d3, hs_d1, hs_d2, hs_d3, vs_d1, vs_d2, vs_d3;
    reg [1:0] previous_zoom = 2;
    reg previous_plot_clear = 0, sampled_frame_start;

    disp_driver timing (
        .ClkDisp(clk), .Rst_n(reset_n), .Data(24'd0), .DataReq(),
        .H_Addr(x), .V_Addr(y), .Disp_HS(hs), .Disp_VS(vs),
        .Disp_Red(), .Disp_Green(), .Disp_Blue(), .Frame_Begin(frame_start),
        .Disp_DE(de), .Disp_PCLK()
    );
    m10_point_cloud_core #(.REFRESH_FRAMES(2), .KEY_DEBOUNCE_CLKS(4)) dut (
        .clk(clk), .reset_n(reset_n), .radar_rx(radar_rx),
        .zoom_in_n(zoom_in_n), .zoom_out_n(zoom_out_n),
        .video_x(x), .video_y(y), .video_de(de), .video_hs(hs),
        .video_vs(vs), .frame_start(frame_start), .rgb888(rgb),
        .out_de(out_de), .out_hs(out_hs), .out_vs(out_vs),
        .debug_uart_tx(debug_uart_tx),
        .data_active(data_active), .error_sticky(error_sticky),
        .overflow_sticky(overflow_sticky), .front_points(front_points),
        .bitmap_ready(bitmap_ready)
    );
    uart_byte_rx #(.CLK_FREQ_HZ(49_000_000), .BAUD_RATE(921600)) debug_receiver (
        .i_clock(clk), .i_reset_n(reset_n), .i_uart_rx(debug_uart_tx),
        .o_rx_data(debug_rx_data), .o_rx_valid(debug_rx_valid),
        .i_rx_ready(1'b1), .o_rx_error(debug_rx_error)
    );
    always @(posedge clk) begin
        if (!reset_n) begin
            mirror_written = 0;
            mirror_read = 0;
        end else begin
            if (dut.rx_valid) begin
                if (!dut.debug_tx_ready)
                    $fatal(1,"raw byte mirror transmitter was busy");
                if (mirror_written >= 4096) $fatal(1,"raw byte mirror test buffer overflow");
                mirrored_bytes[mirror_written] = dut.rx_data;
                mirror_written = mirror_written + 1;
            end
            if (debug_rx_error) $fatal(1,"raw byte mirror UART error");
            if (debug_rx_valid) begin
                if (mirror_read >= mirror_written ||
                    debug_rx_data !== mirrored_bytes[mirror_read])
                    $fatal(1,"raw byte mirror lost/reordered/changed data");
                mirror_read = mirror_read + 1;
            end
        end
    end
    task send_byte;
        input [7:0] data;
        integer b;
        begin
            // Exact 460800 bit/s, intentionally different from rounded UART divider.
            radar_rx = 0; #2170.138889;
            for (b=0; b<8; b=b+1) begin
                radar_rx = data[b]; #2170.138889;
            end
            radar_rx = 1; #2170.138889;
        end
    endtask
    task send_packet;
        input [15:0] angle;
        input [15:0] distance;
        input [7:0] tail_byte;
        integer p;
        begin
            send_byte(8'hA5); send_byte(8'h5A);
            send_byte(angle[15:8]); send_byte(angle[7:0]);
            send_byte(8'h10); send_byte(8'h68);
            for (p=0; p<42; p=p+1) begin
                if (p == 41) begin
                    send_byte(8'hFF); send_byte(8'hFF); // GPS padding.
                end else begin
                    send_byte(distance[15:8]); send_byte(distance[7:0]);
                end
            end
            // GPS time bytes can contain header/tail markers without ending
            // this packet early. This transport ignores timestamp content.
            send_byte(8'hA5); send_byte(8'h5A); send_byte(8'hFA); send_byte(8'hFB);
            send_byte(0); send_byte(0); send_byte(8'h02); send_byte(8'hC8);
            send_byte(8'h02); send_byte(8'hA6);
            send_byte(8'hFA); send_byte(tail_byte);
        end
    endtask
    task check_diagnostics;
        input [15:0] bytes_expected, good_expected, bad_expected, uart_expected;
        input [7:0] byte_expected;
        input [2:0] reason_expected;
        begin
            if (mirror_read != bytes_expected)
                $fatal(1,"raw byte mirror count mismatch: read %0d expected %0d",mirror_read,bytes_expected);
            // Check the actual UI input ports, including the core wiring.
            if (dut.u_video.diagnostic_rx_bytes !== bytes_expected ||
                dut.u_video.diagnostic_good_packets !== good_expected ||
                dut.u_video.diagnostic_bad_packets !== bad_expected ||
                dut.u_video.diagnostic_uart_errors !== uart_expected ||
                dut.u_video.diagnostic_last_byte !== byte_expected ||
                dut.u_video.diagnostic_last_error !== reason_expected)
                $fatal(1,"diagnostic counters/reason or UI wiring incorrect");
        end
    endtask
    function bitmap_bit;
        input integer pixel_x,pixel_y;
        integer address,bit_index;
        begin
            address = (pixel_y-44)*16 + ((pixel_x-256) >> 5);
            bit_index = (pixel_x-256) & 31;
            bitmap_bit = dut.u_video.u_bitmap.front_bank
                ? dut.u_video.u_bitmap.bank1[address][bit_index]
                : dut.u_video.u_bitmap.bank0[address][bit_index];
        end
    endfunction
    task expect_cloud_bit;
        input integer pixel_x,pixel_y;
        input expected;
        begin
            if (bitmap_bit(pixel_x,pixel_y) !== expected)
                $fatal(1,"zoom cloud (%0d,%0d) expected=%b got=%b",
                       pixel_x,pixel_y,expected,bitmap_bit(pixel_x,pixel_y));
        end
    endtask
    task expect_empty_bitmap;
        integer address;
        begin
            if (front_points !== 0) $fatal(1,"zoom retained the old front point count");
            for (address = 0; address < 8192; address = address + 1)
                if (dut.u_video.u_bitmap.bank0[address] !== 32'b0 ||
                    dut.u_video.u_bitmap.bank1[address] !== 32'b0)
                    $fatal(1,"zoom retained stale bitmap word %0d",address);
        end
    endtask
    task zoom_with_live_packet;
        input zoom_in;
        input [1:0] old_zoom,new_zoom;
        input [15:0] bytes_before,good_before,angle;
        begin
            // Press well before the boundary and confirm debouncing cannot
            // change the applied scale while the current frame is scanning.
            wait(timing.vcount_r >= 12'd20 && timing.vcount_r < 12'd500);
            @(negedge clk);
            if (zoom_in) zoom_in_n = 0;
            else zoom_out_n = 0;
            repeat(12) @(negedge clk);
            zoom_in_n = 1; zoom_out_n = 1;
            repeat(12) @(negedge clk);
            if (dut.zoom_level !== old_zoom || dut.plot_clear)
                $fatal(1,"zoom changed before the frame boundary");
            check_diagnostics(bytes_before,good_before,2,1,8'hFB,1);

            fork
                begin
                    // Starting near frame end puts the boundary inside a real
                    // GPS102 packet, rather than between complete packets.
                    wait(timing.vcount_r == 12'd580 && timing.hcount_r == 0);
                    @(negedge clk);
                    send_packet(angle,16'd2000,8'hFB);
                end
                begin
                    while (!frame_start) begin
                        @(negedge clk);
                        if (dut.zoom_level !== old_zoom)
                            $fatal(1,"applied zoom changed during a video frame");
                    end
                    @(posedge clk); #1;
                    if (dut.zoom_level !== new_zoom || !dut.plot_clear || bitmap_ready)
                        $fatal(1,"zoom/plot clear did not apply at the next frame boundary");
                    if (dut.rx_byte_count <= bytes_before ||
                        dut.rx_byte_count >= bytes_before+102 ||
                        dut.u_radar.u_parser.state == 0 || dut.packet_count != good_before)
                        $fatal(1,"test packet was not being received across the zoom boundary");
                    if (dut.uart_error_count != 1 || dut.bad_packet_count != 2 ||
                        dut.last_error_reason != 1 || !error_sticky || overflow_sticky)
                        $fatal(1,"zoom reset the receiver diagnostics/status");
                    @(posedge clk); #1;
                    if (dut.plot_clear) $fatal(1,"plot clear lasted longer than one clock");
                    wait(bitmap_ready); #1;
                    // Clearing completes before the packet tail; both bitmap
                    // banks must be empty while the parser still receives.
                    expect_empty_bitmap;
                    if (dut.packet_count != good_before)
                        $fatal(1,"cross-boundary packet completed before clear check");
                end
            join
            repeat(2000) @(negedge clk);
            check_diagnostics(bytes_before+102,good_before+1,2,1,8'hFB,1);
            if (packets_seen != good_before+1 || points_seen != (good_before+1)*42 ||
                !data_active || !error_sticky || overflow_sticky)
                $fatal(1,"zoom interrupted GPS packets, points, mirror or receiver status");
            wait(front_points == 41); #1;
            if (dut.zoom_level !== new_zoom)
                $fatal(1,"one key press caused repeated zoom changes");
        end
    endtask
    always @(posedge clk) begin
        sampled_frame_start = frame_start;
        x_d1 <= x; x_d2 <= x_d1;
        x_d3 <= x_d2;
        y_d1 <= y; y_d2 <= y_d1;
        y_d3 <= y_d2;
        de_d1 <= de; de_d2 <= de_d1;
        de_d3 <= de_d2;
        hs_d1 <= hs; hs_d2 <= hs_d1;
        hs_d3 <= hs_d2;
        vs_d1 <= vs; vs_d2 <= vs_d1;
        vs_d3 <= vs_d2;
        if (reset_n) begin
            if (dut.packet_valid) packets_seen = packets_seen + 1;
            if (dut.point_valid && dut.point_ready) points_seen = points_seen + 1;
            if (frame_start) begin
                frames_seen = frames_seen + 1;
                if (frames_seen == 3) begin
                    image_file = $fopen("point_cloud_sim.ppm", "w");
                    if (!image_file) $fatal(1, "cannot create simulation image");
                    $fwrite(image_file, "P3\n1024 600\n255\n");
                    capture = 1;
                end
            end
        end
        #1;
        if (!reset_n) begin
            previous_zoom = 2;
            previous_plot_clear = 0;
        end else begin
            if (dut.zoom_level !== previous_zoom) begin
                if (!sampled_frame_start || !dut.plot_clear)
                    $fatal(1,"zoom changed outside a clearing frame boundary");
                zoom_changes = zoom_changes + 1;
            end
            if (dut.plot_clear && (!sampled_frame_start || previous_plot_clear))
                $fatal(1,"plot clear was not a single frame-boundary pulse");
            previous_zoom = dut.zoom_level;
            previous_plot_clear = dut.plot_clear;
        end
        if (reset_n && frames_seen > 1) begin
            if (out_de !== de_d3 || out_hs !== hs_d3 || out_vs !== vs_d3)
                $fatal(1, "RGB/control pipeline alignment failure");
            alignment_checks = alignment_checks + 1;
        end
        if (capture && out_de) begin
            $fwrite(image_file, "%0d %0d %0d\n", rgb[23:16],rgb[15:8],rgb[7:0]);
            pixels_seen = pixels_seen + 1;
            // Four exact cardinal input points are on a 4 m circle (r=100 px).
            if ((x_d3 == 512 && y_d3 == 200) ||
                (x_d3 == 612 && y_d3 == 300) ||
                (x_d3 == 512 && y_d3 == 400) ||
                (x_d3 == 412 && y_d3 == 300)) begin
                if (rgb[15:8] <= rgb[23:16] || rgb[15:8] <= rgb[7:0])
                    $fatal(1, "expected green cloud at %0d,%0d, RGB=%h",x_d3,y_d3,rgb);
                cardinal_seen = cardinal_seen + 1;
            end
            if (pixels_seen == 614400) begin
                $fclose(image_file);
                if (packets_seen != 4 || points_seen != 168 || cardinal_seen != 4)
                    $fatal(1,"wrong packet/point/pixel counts: %0d/%0d/%0d",packets_seen,points_seen,cardinal_seen);
                if (!data_active || error_sticky || overflow_sticky)
                    $fatal(1,"unexpected receiver status");
                check_diagnostics(408, 4, 0, 0, 8'hFB, 0);
                capture = 0;
                capture_done = 1;
            end
        end
    end
    initial begin
        repeat (10) @(negedge clk);
        reset_n = 1;
        wait(bitmap_ready);
        @(negedge clk);
        send_packet(16'd0,16'd4000,8'hFB);
        send_packet(16'd9000,16'd4000,8'hFB);
        send_packet(16'd18000,16'd4000,8'hFB);
        send_packet(16'd27000,16'd4000,8'hFB);
        wait(capture_done);

        // A rejected tail must increment only the parser counter; no points
        // may leak out. Then a sustained break must yield just one UART error.
        send_packet(16'd0,16'd2730,8'hFC);
        // Allow the final 921600bps mirror byte to finish after input stop.
        repeat(1000) @(negedge clk);
        check_diagnostics(510,4,1,0,8'hFC,2);
        if (!error_sticky || points_seen != 168)
            $fatal(1,"bad packet was published or error was not latched");
        radar_rx = 0; #30000;
        radar_rx = 1; #6000;
        repeat(200) @(negedge clk);
        check_diagnostics(510,4,2,1,8'hFC,1);

        // Valid continuous data resumes after the break without a reset.
        send_packet(16'd0,16'd2730,8'hFB);
        repeat(2000) @(negedge clk);
        check_diagnostics(612,5,2,1,8'hFB,1);
        if (!data_active || !error_sticky || overflow_sticky || points_seen != 210)
            $fatal(1,"receiver did not recover after break");

        // S1 doubles the radius of a 2 m point to 100 pixels. S2 returns to
        // 40 mm/pixel and 50 pixels, without clearing UART/parser history.
        zoom_with_live_packet(1'b1,2'd2,2'd1,16'd612,16'd5,16'd0);
        expect_cloud_bit(512,200,1'b1); // 2 m / 20 mm per pixel = 100.
        expect_cloud_bit(512,250,1'b0); // Old 40 mm-per-pixel position.
        expect_cloud_bit(612,300,1'b0);
        expect_cloud_bit(512,400,1'b0);
        expect_cloud_bit(412,300,1'b0);
        zoom_with_live_packet(1'b0,2'd1,2'd2,16'd714,16'd6,16'd9000);
        expect_cloud_bit(562,300,1'b1); // 2 m / 40 mm per pixel = 50.
        expect_cloud_bit(612,300,1'b0); // Old 20 mm-per-pixel position.
        expect_cloud_bit(512,200,1'b0); // Previous cloud was cleared.
        check_diagnostics(816,7,2,1,8'hFB,1);
        if (mirror_written != 816 || zoom_changes != 2 || alignment_checks == 0)
            $fatal(1,"zoom/mirror/continuous alignment checks incomplete");
        reset_n = 0;
        repeat(5) @(negedge clk);
        check_diagnostics(0,0,0,0,0,0);
        if (error_sticky || overflow_sticky || data_active)
            $fatal(1,"reset did not clear status");
        $display("PASS m10_point_cloud_core_tb: 460800bps GPS102 UART, RGB frame, diagnostics, tail/break recovery, live S1/S2 zoom, 816-byte 921600bps mirror, continuous video alignment and reset");
        $finish;
    end
    initial begin
        #200000000;
        $fatal(1,"integration simulation timeout");
    end
endmodule
