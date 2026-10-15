`timescale 1ns/1ps
module point_cloud_video_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg reset_n = 0, point_valid = 0, frame_start = 0;
    reg [1:0] zoom_level = 2;
    reg plot_clear = 0;
    reg [9:0] point_x = 0, point_y = 0;
    reg [11:0] video_x = 0, video_y = 0;
    reg video_de = 0, video_hs = 1, video_vs = 1;
    reg status_data_active = 1, status_error = 0, status_overflow = 0;
    reg [15:0] diagnostic_rx_bytes = 0, diagnostic_uart_errors = 0;
    reg [15:0] diagnostic_bad_packets = 0, diagnostic_good_packets = 0;
    reg [7:0] diagnostic_last_byte = 0;
    reg [2:0] diagnostic_last_error = 0;
    wire point_ready, out_de, out_hs, out_vs, bitmap_ready;
    wire [23:0] rgb888;
    wire [15:0] front_points;
    integer checks = 0, cycle;
    reg [2:0] previous_control, previous_control_2, expected_control;
    point_cloud_video #(.REFRESH_FRAMES(1)) dut (
        .clk(clk), .reset_n(reset_n), .point_valid(point_valid),
        .point_ready(point_ready), .point_x(point_x), .point_y(point_y),
        .video_x(video_x), .video_y(video_y), .video_de(video_de),
        .video_hs(video_hs), .video_vs(video_vs), .frame_start(frame_start),
        .zoom_level(zoom_level), .plot_clear(plot_clear),
        .status_data_active(status_data_active), .status_error(status_error),
        .status_overflow(status_overflow),
        .diagnostic_rx_bytes(diagnostic_rx_bytes),
        .diagnostic_uart_errors(diagnostic_uart_errors),
        .diagnostic_bad_packets(diagnostic_bad_packets),
        .diagnostic_good_packets(diagnostic_good_packets),
        .diagnostic_last_byte(diagnostic_last_byte),
        .diagnostic_last_error(diagnostic_last_error),
        .rgb888(rgb888), .out_de(out_de),
        .out_hs(out_hs), .out_vs(out_vs), .front_points(front_points),
        .bitmap_ready(bitmap_ready)
    );
    task write_point;
        input integer x,y;
        begin
            @(negedge clk); point_x = x; point_y = y;
            while (!point_ready) @(negedge clk);
            point_valid = 1;
            @(posedge clk); #1;
            @(negedge clk); point_valid = 0;
        end
    endtask
    task expect_text;
        input integer x,y;
        input [95:0] expected;
        begin
            @(negedge clk); video_x = x; video_y = y;
            #1;
            if (!dut.text_enabled || dut.text !== expected)
                $fatal(1,"text (%0d,%0d) expected=%s got=%s",x,y,expected,dut.text);
            checks = checks + 1;
        end
    endtask
    task expect_glyph;
        input integer x,y;
        input [34:0] expected;
        input [23:0] color;
        integer row,column;
        begin
            for (row = 0; row < 7; row = row + 1)
                for (column = 0; column < 5; column = column + 1)
                    expect_pixel(x + 2*column, y + 2*row,
                        expected[(6-row)*5+4-column] ? color : 24'h080f18);
        end
    endtask
    task expect_pixel;
        input integer x,y;
        input [23:0] expected;
        begin
            @(negedge clk); video_x = x; video_y = y;
            video_de = 1; video_hs = 1; video_vs = 0;
            repeat (3) @(posedge clk); #1;
            if (rgb888 !== expected || !out_de || !out_hs || out_vs)
                $fatal(1,"RGB (%0d,%0d) expected=%h got=%h control=%b%b%b",
                       x,y,expected,rgb888,out_de,out_hs,out_vs);
            checks = checks + 1;
        end
    endtask
    initial begin
        repeat (4) @(negedge clk); reset_n = 1;
        wait(bitmap_ready);
        write_point(512,200); write_point(612,300);
        write_point(256,44); write_point(767,555); write_point(511,555);
        write_point(255,44); write_point(768,555); // Consume off-canvas safely.
        @(negedge clk); frame_start = 1;
        @(posedge clk); #1;
        @(negedge clk); frame_start = 0;
        if (front_points != 5) $fatal(1,"off-canvas points reached RAM");
        repeat (30) @(posedge clk);
        if (dut.decimal_digits !== 20'h00005) $fatal(1,"point-count BCD conversion failed");
        expect_pixel(512,200,24'h56f0ba);
        expect_pixel(612,300,24'h56f0ba);
        expect_pixel(256,44,24'h56f0ba);
        expect_pixel(767,555,24'h56f0ba);
        expect_pixel(511,555,24'h56f0ba);
        expect_pixel(513,201,24'h0b1720);
        expect_pixel(512,240,24'h245660);
        expect_pixel(562,301,24'h193d45);
        expect_pixel(256,45,24'h34515d);
        expect_pixel(512,300,24'hc1dedc);
        expect_pixel(255,44,24'h080f18);
        expect_pixel(768,555,24'h080f18);
        expect_pixel(24,132,24'hb6cad4); // P top-left font pixel.
        expect_pixel(42,348,24'hb6cad4); // 040 MM/PX, second digit is 4.
        status_error = 1;
        expect_pixel(820,232,24'hff675d);
        status_overflow = 1;
        expect_pixel(820,328,24'hffb851);
        // Each diagnosis identifies its source while retaining error color.
        diagnostic_last_error = 1;
        expect_text(820,232,"UART        ");
        expect_pixel(820,232,24'hff675d);
        diagnostic_last_error = 2;
        expect_text(820,232,"TAIL        ");
        expect_pixel(820,232,24'hff675d);
        diagnostic_last_error = 3;
        expect_text(820,232,"ANGLE       ");
        expect_pixel(822,232,24'hff675d);
        diagnostic_last_error = 4;
        expect_text(820,232,"TIMEOUT     ");
        expect_pixel(820,232,24'hff675d);
        diagnostic_last_error = 5;
        expect_text(820,232,"OVERFLOW    ");
        expect_pixel(822,232,24'hff675d);
        status_error = 0;
        expect_text(820,232,"OK          ");
        expect_pixel(822,232,24'h56f0ba);

        // Exercise every hexadecimal digit in visible counters, including B.
        diagnostic_rx_bytes = 16'h0123;
        diagnostic_good_packets = 16'h4567;
        diagnostic_uart_errors = 16'h89ab;
        diagnostic_bad_packets = 16'hcdef;
        diagnostic_last_byte = 8'hab;
        expect_text(24,72,"RX BYTES    ");
        expect_text(24,96,"0123 HEX    ");
        expect_text(24,384,"GOOD PACKETS");
        expect_text(24,408,"4567 HEX    ");
        expect_text(820,480,"UART ERR    ");
        expect_text(820,504,"89AB HEX    ");
        expect_text(820,544,"BAD PACKETS ");
        expect_text(820,568,"CDEF HEX    ");
        expect_text(24,520,"LAST BYTE   ");
        expect_text(24,544,"AB HEX      ");
        expect_glyph(36,544,
            {5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110},
            24'hb6cad4);
        expect_glyph(820,568,
            {5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110},
            24'hb6cad4);
        expect_glyph(856,568,
            {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000},
            24'hb6cad4);

        // Zoom changes SCALE and the 250-pixel range together, including 2.5 m.
        zoom_level = 0;
        expect_text(24,348,"010 MM/PX   ");
        expect_text(24,256,"2.5 M       ");
        expect_glyph(36,256,
            {5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00110,5'b00110},
            24'hb6cad4);
        zoom_level = 1;
        expect_text(24,348,"020 MM/PX   ");
        expect_text(24,256,"5 M         ");
        zoom_level = 2;
        expect_text(24,348,"040 MM/PX   ");
        expect_text(24,256,"10 M        ");
        zoom_level = 3;
        expect_text(24,348,"080 MM/PX   ");
        expect_text(24,256,"20 M        ");
        expect_text(24,452,"S1 ZOOM IN  ");
        expect_text(24,484,"S2 ZOOM OUT ");
        expect_glyph(60,452,
            {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111},
            24'hb6cad4);

        // Check varying control signals against an independent 3-stage queue.
        // A zoom/plot clear during this stream must not reset video controls.
        previous_control = {video_de,video_hs,video_vs};
        previous_control_2 = previous_control;
        for (cycle = 0; cycle < 100; cycle = cycle + 1) begin
            @(negedge clk);
            video_x = cycle; video_y = 0;
            video_de = cycle % 3 != 0;
            video_hs = cycle % 5 != 0;
            video_vs = cycle % 7 != 0;
            plot_clear = cycle == 40;
            if (cycle == 40) zoom_level = 0;
            expected_control = previous_control_2;
            previous_control_2 = previous_control;
            previous_control = {video_de,video_hs,video_vs};
            @(posedge clk); #1;
            if ({out_de,out_hs,out_vs} !== expected_control)
                $fatal(1,"video control latency failed cycle %0d",cycle);
            if (!out_de && rgb888 != 0) $fatal(1,"RGB is nonzero during blanking");
            if (cycle >= 2 && out_de && rgb888 !== 24'h080f18)
                $fatal(1,"plot clear disturbed RGB pipeline cycle %0d",cycle);
            if (cycle == 40 && (bitmap_ready || front_points != 0))
                $fatal(1,"plot clear did not reset bitmap and point count");
        end
        // Both former front-bank points and its count must disappear on clear.
        wait(bitmap_ready);
        repeat (30) @(posedge clk);
        if (front_points != 0 || dut.decimal_digits !== 20'h00000)
            $fatal(1,"point count was not cleared after zoom");
        expect_pixel(512,200,24'h245660);
        expect_pixel(612,300,24'h245660);
        expect_pixel(256,44,24'h34515d);
        expect_text(24,256,"2.5 M       ");
        expect_text(24,348,"010 MM/PX   ");
        expect_text(24,96,"0123 HEX    ");
        expect_text(820,568,"CDEF HEX    ");
        $display("PASS point_cloud_video_tb: %0d RGB/UI checks and 100 control-latency checks",checks);
        $finish;
    end
    initial begin #3000000; $fatal(1,"video test timeout"); end
endmodule
