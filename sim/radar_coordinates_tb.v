`timescale 1ns/1ps
// 从真实 UART 位流检验解析、采样位姿、极坐标、地图与显示。
module radar_coordinates_tb;
    reg clk = 0, pclk = 0, rst_n = 0, rx = 1;
    always #10 clk = ~clk;
    always #11 pclk = ~pclk;
    reg signed [31:0] px = 0, py = 0, yaw = 0;
    wire mv, mo, ms, md, mp, rp, cv;
    wire [15:0] ca, cd;
    reg fs = 0;
    reg [11:0] sx = 600, sy = 300;
    integer cycles = 0, frames = 0, checks = 0, errors = 0;
    integer j, k, expect_a, expect_d, parsed = 0;
    integer got_x, got_y, ex, ey, flip_x, flip_y, base;
    integer tests = 0, parsed_errors = 0;
    integer accepted = 0, geometry_count = 0, geometry_errors = 0, stream_errors = 0;
    integer n;
    reg control_phase = 1;
    reg [7:0] capture [0:73337];
    real reference_dx, reference_dy, actual_dx, actual_dy, delta_dx, delta_dy, range_mm;
    real angle_rad, dx_mm, dy_mm;
    radar_navigation dut (
        .clk                    (clk),
        .reset_n                (rst_n),
        .map_enable             (1'b1),
        .plan_enable            (1'b0),
        .motion_enable          (1'b0),
        .radar_rx               (rx),
        .pose_valid             (1'b1),
        .pose_x_mm              (px),
        .pose_y_mm              (py),
        .pose_yaw_cdeg          (yaw),
        .goal_valid             (1'b0),
        .goal_x_cell            (8'd0),
        .goal_y_cell            (8'd0),
        .ctrl_done              (1'b0),
        .ctrl_linear_active     (1'b0),
        .display_enable         (1'b1),
        .map_disp_valid         (mv),
        .map_disp_occupied      (mo),
        .map_disp_frame_start   (ms),
        .map_disp_frame_done    (md),
        .route_disp_valid       (),
        .route_disp_x_cell      (),
        .route_disp_y_cell      (),
        .route_disp_frame_start (),
        .route_disp_frame_done  (),
        .cloud_valid            (cv),
        .cloud_angle_cdeg       (ca),
        .cloud_distance_mm      (cd)
    );
    defparam dut.u_adapter.u_navigation.u_nav.REPLAN_PERIOD = 23'd500000;
    nav_map_view view (
        .clk            (clk),
        .rst_n          (rst_n),
        .map_valid      (mv),
        .map_occupied   (mo),
        .map_start      (ms),
        .map_done       (md),
        .route_valid    (1'b0),
        .route_x        (8'd0),
        .route_y        (8'd0),
        .route_start    (1'b0),
        .route_done     (1'b0),
        .cmd_valid      (1'b0),
        .target_line_dir  (2'd0),
        .pose_x_mm      (px),
        .pose_y_mm      (py),
        .target_x_mm    (32'sd0),
        .target_y_mm    (32'sd0),
        .pixel_clk      (pclk),
        .pixel_rst_n    (rst_n),
        .frame_start    (fs),
        .pixel_x        (sx),
        .pixel_y        (sy),
        .map_pixel      (mp),
        .route_pixel    (rp)
    );
    always @(posedge clk) begin
        cycles <= cycles + 1;
        if (md) frames <= frames + 1;
        if (rst_n && (dut.u_adapter.radar_error || dut.u_adapter.radar_overflow))
            stream_errors = stream_errors + 1;
        if (cv && ca < 36000 && cd <= 10000) begin
            accepted = accepted + 1;
            range_mm = (cd == 0 || cd == 10000) ? 10000.0 : $itor(cd);
            angle_rad = ($itor($signed(dut.u_adapter.point_context[31:0])) - $itor(ca)) *
                        3.141592653589793 / 18000.0;
            reference_dx = range_mm * $cos(angle_rad);
            reference_dy = range_mm * $sin(angle_rad);
        end
        if (control_phase && cv && cd != 16'hffff) begin
            parsed = parsed + 1;
            $display("PARSED angle=%0d distance=%0d pose=(%0d,%0d,%0d)",ca,cd,
                $signed(dut.u_adapter.point_context[95:64]),$signed(dut.u_adapter.point_context[63:32]),
                $signed(dut.u_adapter.point_context[31:0]));
            if (ca != expect_a || cd != expect_d) parsed_errors = parsed_errors + 1;
        end
        if (dut.u_adapter.u_navigation.grid_valid && dut.u_adapter.u_navigation.grid_ready) begin
            got_x = $signed(dut.u_adapter.u_navigation.grid_x);
            got_y = $signed(dut.u_adapter.u_navigation.grid_y);
            geometry_count = geometry_count + 1;
            actual_dx = $itor($signed(dut.u_adapter.u_navigation.observation_ray_dx_q16)) / 65536.0;
            actual_dy = $itor($signed(dut.u_adapter.u_navigation.observation_ray_dy_q16)) / 65536.0;
            delta_dx = actual_dx - reference_dx;
            delta_dy = actual_dy - reference_dy;
            if ((^dut.u_adapter.u_navigation.observation_ray_dx_q16 === 1'bx) ||
                (^dut.u_adapter.u_navigation.observation_ray_dy_q16 === 1'bx) ||
                delta_dx * delta_dx + delta_dy * delta_dy >= 1.0) begin
                geometry_errors = geometry_errors + 1;
                if (geometry_errors <= 8)
                    $display("GEOMETRY_MISMATCH expected=(%0f,%0f) actual=(%0f,%0f)",
                             reference_dx, reference_dy, actual_dx, actual_dy);
            end
        end
    end
    integer pc = 0;
    always @(negedge pclk) begin
        pc = pc + 1;
        fs = pc % 20000 == 0;
    end
    task check;
        input condition;
        input [1023:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s", message);
            end else $display("pass %0s", message);
        end
    endtask

    task byte_out;
        input [7:0] data;
        begin
            rx = 0;
            repeat (109) @(negedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                rx = data[k];
                repeat (109) @(negedge clk);
            end
            rx = 1;
            repeat (109) @(negedge clk);
        end
    endtask

    task packet;
        input integer a, d, x, y, heading;
        begin
            px = x;
            py = y;
            yaw = heading;
            expect_a = a;
            expect_d = d;
            got_x = -999;
            got_y = -999;
            angle_rad = (heading - a) * 3.141592653589793 / 18000.0;
            dx_mm = d * $cos(angle_rad);
            dy_mm = d * $sin(angle_rad);
            ex = $rtoi($floor((x + dx_mm + 6400.0) / 50.0));
            ey = $rtoi($floor((y + dy_mm + 6400.0) / 50.0));
            flip_x = $rtoi($floor((x - dx_mm + 6400.0) / 50.0));
            flip_y = $rtoi($floor((y - dy_mm + 6400.0) / 50.0));
            byte_out(8'ha5);
            byte_out(8'h5a);
            byte_out(a >> 8);
            byte_out(a);
            byte_out(0);
            byte_out(60);
            byte_out(d >> 8);
            byte_out(d);
            for (j = 1; j < 42; j = j + 1) begin
                byte_out(8'hff);
                byte_out(8'hff);
            end
            repeat (10) byte_out(0);
            byte_out(8'hfa);
            byte_out(8'hfb);
            repeat (10000) @(negedge clk);
            $display("CASE angle=%0d distance=%0d pose=(%0d,%0d,%0d) expected=(%0d,%0d) actual=(%0d,%0d)",
                     a, d, x, y, heading, ex, ey, got_x, got_y);
            check(got_x == ex && got_y == ey, "UART angle/distance maps to correct world cell");
            check(dut.u_adapter.u_navigation.u_nav.u_map.raw[got_y * 256 + got_x],
                  "scanner stores geometry endpoint without changing coordinates");
            tests = tests + 1;
        end
    endtask

    task sample_cell;
        input integer x, y;
        begin
            @(negedge pclk);
            sx = x * 6 - 356 + 3;
            sy = 1068 - y * 6 - 3;
            repeat (5) @(negedge pclk);
        end
    endtask
    initial begin
        $readmemh("../vendor/verilog-reusable-modules/modules/m10_radar/sim/m10_capture.hex", capture);
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (66000) @(negedge clk);
        packet(4500,1500,0,0,0);
        packet(13500,1500,0,0,0);
        packet(22500,1500,0,0,0);
        packet(31500,1500,0,0,0);
        packet(1,1234,0,0,0);
        packet(8999,1234,0,0,0);
        packet(27001,1234,0,0,0);
        packet(35999,1234,0,0,0);
        packet(0,1250,0,0,0);
        packet(9000,1250,0,0,0);
        packet(18000,1250,0,0,0);
        packet(27000,1250,0,0,0);
        packet(7500,1500,3000,-500,3000);
        check(parsed == tests && parsed_errors == 0,"UART parser retains every input angle and distance");
        check(stream_errors == 0,"no UART parse or queue overflow errors");
        base = frames;
        wait (frames >= base + 2);
        repeat (30000) @(negedge pclk);
        sample_cell(ex, ey);
        check(mp, "correct positive-x endpoint reaches the double-buffer display");
        sample_cell(flip_x, flip_y);
        check(!mp, "display has no inverted endpoint for the translated pose");
        check(accepted == 13 && geometry_count == 13 && geometry_errors == 0,
              "all control rays agree with independent trigonometry within 1 mm");
        $display("METRICS: radar_coordinate_control cycles=%0d packets=%0d errors=%0d",
                 cycles, tests, geometry_errors);

        // 独立复位后按真实约 4.166 ms 包间隔回放原始捕获的前三圈。
        @(negedge clk);
        rst_n = 0;
        control_phase = 0;
        px = 0;
        py = 0;
        yaw = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        accepted = 0;
        geometry_count = 0;
        geometry_errors = 0;
        stream_errors = 0;
        repeat (66000) @(negedge clk);
        base = cycles;
        for (n = 0; n < 7344; n = n + 1) begin
            byte_out(capture[n]);
            if (n % 102 == 101) repeat (97120) @(negedge clk);
        end
        repeat (100000) @(negedge clk);
        check(accepted == 2934 && geometry_count == accepted,
              "72 captured packets deliver all 2934 legal rays");
        check(geometry_errors == 0,
              "captured UART rays agree with independent trigonometry within 1 mm");
        check(stream_errors == 0 && dut.u_adapter.u_radar.packet_count == 72 &&
              dut.u_adapter.u_radar.bad_packet_count == 0 && dut.u_adapter.u_radar.uart_error_count == 0,
              "capture replay has no UART, parser or queue overflow errors");
        $display("METRICS: radar_coordinate_capture cycles=%0d packets=72 rays=%0d errors=%0d",
                 cycles - base, geometry_count, geometry_errors);
        $display("CHECKS: %0d", checks);
        $display("METRICS: radar_coordinates cycles=%0d control_packets=%0d checks=%0d errors=%0d",
                 cycles, tests, checks, errors);
        if (!errors) $display("PASS: radar_coordinates_tb");
        $finish;
    end
    initial begin
        #600000000;
        $display("FAIL: radar_coordinates_tb timeout");
        $finish;
    end
endmodule
