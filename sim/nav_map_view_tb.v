`timescale 1ns/1ps
module nav_map_view_tb;
    reg clk = 0, pixel_clk = 0;
    always #10 clk = ~clk;
    always #11 pixel_clk = ~pixel_clk;
    reg rst_n = 0;
    reg map_valid = 0, map_occupied = 0, map_start = 0, map_done = 0;
    reg route_valid = 0, route_start = 0, route_done = 0;
    reg [7:0] route_x = 0, route_y = 0;
    reg cmd_valid = 0, target_axis_y = 0;
    reg signed [31:0] pose_x_mm = 0, pose_y_mm = 0;
    reg signed [31:0] target_x_mm = 0, target_y_mm = 0;
    reg frame_start = 0;
    reg [11:0] pixel_x = 475, pixel_y = 297;
    wire map_pixel, route_pixel;
    integer cycles = 0, checks = 0, errors = 0, n;
    always @(posedge clk) cycles <= cycles + 1;

    nav_map_view dut (
        .clk(clk), .rst_n(rst_n),
        .map_valid(map_valid), .map_occupied(map_occupied),
        .map_start(map_start), .map_done(map_done),
        .route_valid(route_valid), .route_x(route_x), .route_y(route_y),
        .route_start(route_start), .route_done(route_done),
        .cmd_valid(cmd_valid), .target_axis_y(target_axis_y),
        .pose_x_mm(pose_x_mm), .pose_y_mm(pose_y_mm),
        .target_x_mm(target_x_mm), .target_y_mm(target_y_mm),
        .pixel_clk(pixel_clk), .pixel_rst_n(rst_n),
        .frame_start(frame_start), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .map_pixel(map_pixel), .route_pixel(route_pixel)
    );

    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s", message);
            end else $display("pass %0s", message);
        end
    endtask

    // 每次只发一个终点，当前段必须由命令补画；收完后立即改变输入验证取样。
    task command_frame;
        input integer px, py, tx, ty;
        input axis_y;
        reg old_request;
        integer begin_cycle;
        begin
            wait (dut.route_state == 0 && dut.route_available);
            @(negedge clk);
            old_request = dut.route_request;
            cmd_valid = 1;
            target_axis_y = axis_y;
            pose_x_mm = px;
            pose_y_mm = py;
            target_x_mm = tx;
            target_y_mm = ty;
            point((tx + 6400) / 50, (ty + 6400) / 50, 1, 1);
            @(negedge clk);
            route_start = 0;
            route_valid = 0;
            route_done = 0;
            begin_cycle = cycles;
            cmd_valid = 0;
            target_axis_y = !axis_y;
            pose_x_mm = 3000;
            pose_y_mm = 3000;
            target_x_mm = 3000;
            target_y_mm = 3000;
            wait (dut.route_request != old_request);
            $display("METRICS: command axis=%0d pose=(%0d,%0d) target=(%0d,%0d) draw_cycles=%0d",
                     axis_y, px, py, tx, ty, cycles - begin_cycle);
            next_frame();
        end
    endtask

    task next_frame;
        begin
            repeat (5) @(negedge pixel_clk);
            frame_start = 1;
            @(negedge pixel_clk);
            frame_start = 0;
            repeat (5) @(negedge pixel_clk);
        end
    endtask

    task sample_cell;
        input integer x, y;
        begin
            @(negedge pixel_clk);
            pixel_x = x * 6 - 356 + 3;
            pixel_y = 1068 - y * 6 - 3;
            repeat (3) @(negedge pixel_clk);
        end
    endtask

    task map_frame;
        input integer occupied_cell;
        begin
            for (n = 0; n < 65536; n = n + 1) begin
                @(negedge clk);
                map_start = n == 0;
                map_done = n == 65535;
                map_valid = 1;
                map_occupied = n == occupied_cell;
            end
            @(negedge clk);
            map_start = 0;
            map_done = 0;
            map_valid = 0;
        end
    endtask

    task point;
        input [7:0] x, y;
        input first, last;
        begin
            @(negedge clk);
            route_x = x;
            route_y = y;
            route_start = first;
            route_done = last;
            route_valid = 1;
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge pixel_clk);
        check(!map_pixel && !route_pixel, "uninitialized RAM is hidden");
        map_frame(128 * 256 + 138);
        sample_cell(138, 128);
        check(!map_pixel, "completed map waits for frame boundary");
        next_frame();
        check(map_pixel, "first full map appears at frame boundary");
        map_frame(128 * 256 + 140);
        check(map_pixel, "writing next map preserves old front image");
        next_frame();
        check(!map_pixel, "next frame removes old obstacle");
        sample_cell(140, 128);
        check(map_pixel, "next frame displays new obstacle");

        @(negedge clk);
        map_start = 1;
        map_valid = 1;
        map_occupied = 1;
        @(negedge clk);
        map_start = 0;
        map_valid = 0;
        next_frame();
        check(map_pixel, "interrupted map cannot publish partial back image");
        map_frame(128 * 256 + 140);
        next_frame();
        check(map_pixel, "new map start replaces interrupted frame");

        point(128, 140, 1, 0);
        point(128, 128, 1, 0);
        point(138, 128, 0, 0);
        point(138, 138, 0, 1);
        @(negedge clk);
        route_start = 0;
        route_valid = 0;
        route_done = 0;
        wait (dut.route_request == 1);
        sample_cell(133, 128);
        check(!route_pixel, "completed route waits for frame boundary");
        next_frame();
        check(route_pixel, "horizontal segment includes cells between waypoints");
        sample_cell(138, 133);
        check(route_pixel, "vertical segment includes cells between waypoints");
        sample_cell(133, 133);
        check(!route_pixel, "route does not fill unrelated cells");
        sample_cell(128, 140);
        check(!route_pixel, "new route start discards incomplete prior points");
        sample_cell(133, 128);
        repeat (6) @(negedge clk);
        route_start = 1;
        route_done = 1;
        @(negedge clk);
        route_start = 0;
        route_done = 0;
        wait (dut.route_request == 0);
        repeat (5) @(negedge pixel_clk);
        check(route_pixel, "empty replacement leaves old route until frame boundary");
        next_frame();
        check(!route_pixel && dut.map_front == 1, "empty route clears independently of map");
        sample_cell(140, 128);
        check(map_pixel, "route replacement preserves obstacle image");
        @(negedge pixel_clk);
        pixel_x = 100;
        repeat (3) @(negedge pixel_clk);
        check(!map_pixel && !route_pixel, "display layers stay inside map viewport");

        command_frame(-400, 175, 500, 25, 0);
        sample_cell(125, 128);
        check(route_pixel, "east current segment includes negative-world start region");
        sample_cell(133, 128);
        check(route_pixel, "single waypoint draws missing east segment");
        sample_cell(125, 131);
        check(!route_pixel, "horizontal line ignores lateral pose error");

        command_frame(100, -275, 500, 25, 0);
        sample_cell(125, 128);
        check(!route_pixel, "moving forward clears old current-segment tail");
        sample_cell(135, 128);
        check(route_pixel, "moving forward keeps remaining current segment");

        command_frame(100, 125, 800, 25, 0);
        sample_cell(142, 128);
        check(route_pixel, "same-direction target extension appears");
        command_frame(100, 125, 300, 25, 0);
        sample_cell(142, 128);
        check(!route_pixel, "target shortening clears old extension");
        sample_cell(132, 128);
        check(route_pixel, "target shortening keeps new prefix");

        command_frame(500, 175, -400, 25, 0);
        sample_cell(130, 128);
        check(route_pixel, "west current segment is horizontal");
        sample_cell(130, 131);
        check(!route_pixel, "west segment does not follow lateral offset");

        command_frame(275, -400, 25, 500, 1);
        sample_cell(128, 125);
        check(route_pixel, "north current segment is vertical");
        sample_cell(133, 125);
        check(!route_pixel, "vertical line ignores lateral pose error");
        sample_cell(130, 128);
        check(!route_pixel, "axis change clears old horizontal segment");

        command_frame(-175, 500, 25, -400, 1);
        sample_cell(128, 130);
        check(route_pixel, "south current segment is vertical");
        sample_cell(124, 130);
        check(!route_pixel, "south segment does not follow lateral offset");

        command_frame(25, -400, 25, -400, 1);
        sample_cell(128, 120);
        check(route_pixel, "zero-length current segment draws endpoint");
        sample_cell(128, 125);
        check(!route_pixel, "zero-length replacement clears prior segment");

        wait (dut.route_state == 0 && dut.route_available);
        point(128, 140, 1, 1);
        @(negedge clk);
        route_start = 0;
        route_valid = 0;
        route_done = 0;
        wait (dut.route_state == 0 && !dut.route_available);
        next_frame();
        sample_cell(128, 120);
        check(!route_pixel, "invalid command removes supplemental segment on refresh");
        sample_cell(128, 140);
        check(route_pixel, "invalid command preserves normal route waypoint");

        wait (dut.route_available);
        @(negedge clk);
        route_start = 1;
        route_done = 1;
        @(negedge clk);
        route_start = 0;
        route_done = 0;
        wait (dut.route_state == 0 && !dut.route_available);
        next_frame();
        check(!route_pixel, "cancelled empty route clears all lines");

        wait (dut.route_available);
        @(negedge clk);
        cmd_valid = 1;
        target_axis_y = 0;
        pose_x_mm = -200;
        pose_y_mm = 175;
        target_x_mm = 525;
        target_y_mm = 25;
        point(132, 128, 1, 0);
        point(138, 128, 0, 0);
        point(138, 138, 0, 1);
        @(negedge clk);
        route_start = 0;
        route_valid = 0;
        route_done = 0;
        wait (dut.route_state == 0 && !dut.route_available);
        next_frame();
        sample_cell(125, 128);
        check(route_pixel, "current prefix joins stored route with locked intermediate point");
        sample_cell(138, 133);
        check(route_pixel, "drawing current prefix preserves following corner segment");
        sample_cell(125, 131);
        check(!route_pixel, "prefix and future route do not create lateral connector");
        $display("CHECKS: %0d", checks);
        $display("METRICS: nav_map_view_tb cycles=%0d errors=%0d", cycles, errors);
        if (errors == 0) $display("PASS: nav_map_view_tb");
        $finish;
    end
    initial begin
        #30000000;
        $display("FAIL: nav_map_view_tb timeout");
        $finish;
    end
endmodule
