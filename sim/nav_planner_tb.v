`timescale 1ns/1ps
// 同一室内地图分别检查原始 BFS 与实际开启居中的航点输出。
module nav_planner_check #(
    parameter integer CENTER_EN = 0
) (
    output reg done = 0,
    output integer checks = 0,
    output integer errors = 0
);
    reg clk = 0;
    always #10 clk = ~clk;
    reg reset_n = 0, req_valid = 0, escape = 0;
    reg [15:0] sx = 0, sy = 0, gx = 0, gy = 0;
    wire req_ready, mrd_valid, wp_valid, wp_last, busy, success, no_path;
    wire [15:0] mrd_x, mrd_y, wp_x, wp_y, path_len;
    reg rsp_valid = 0, rsp_cost = 0, rsp_raw = 0, pending = 0;
    integer cycles = 0, delay_left = 0, pending_index = 0;
    wire mrd_ready = !pending && (cycles % 5 != 0);
    wire wp_ready = (cycles % 7 >= 2);
    reg [1:0] map [0:65535];
    reg [15:0] reference [0:4095];
    reg collecting = 0, geometry_ok, reference_ok, stable_ok, bounds_ok;
    reg held = 0, held_last;
    reg [15:0] held_x, held_y, occupancy;
    integer current_x, current_y, step_x, step_y, next_x, next_y, dx, dy;
    integer last_dir, direction, steps, turns, points, expanded, peak_queue;
    integer manifest, metrics, waypoints, cases, scan_result, case_index;
    integer start_x, start_y, goal_x, goal_y, allow_escape, expected_steps, expected_turns;
    integer case_begin, wait_cycles, path_pos, i, index;
    reg [2047:0] fixture_dir, file_name;

    nav_planner #(.GRID_W(256), .GRID_H(256), .CENTER_EN(CENTER_EN)) dut (
        .clk(clk), .reset_n(reset_n),
        .req_valid(req_valid), .req_ready(req_ready), .allow_start_escape(escape),
        .start_x(sx), .start_y(sy), .goal_x(gx), .goal_y(gy),
        .mrd_valid(mrd_valid), .mrd_ready(mrd_ready), .mrd_x(mrd_x), .mrd_y(mrd_y),
        .mrd_rsp_valid(rsp_valid), .mrd_cost(rsp_cost), .mrd_raw(rsp_raw),
        .wp_limit(16'd512), .wp_valid(wp_valid), .wp_ready(wp_ready),
        .wp_x(wp_x), .wp_y(wp_y), .wp_last(wp_last),
        .busy(busy), .success(success), .no_path(no_path), .path_len(path_len)
    );

    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: planner center=%0d case=%0d %0s", CENTER_EN, case_index, message);
            end else $display("pass planner center=%0d case=%0d %0s", CENTER_EN, case_index, message);
        end
    endtask

    // 地图握手带不同读延迟，航点接收也定期背压。
    always @(posedge clk) begin
        cycles <= cycles + 1;
        rsp_valid <= 0;
        if (pending) begin
            if (delay_left == 0) begin
                rsp_valid <= 1;
                rsp_cost <= map[pending_index][0];
                rsp_raw <= map[pending_index][1];
                pending <= 0;
            end else delay_left <= delay_left - 1;
        end
        if (mrd_valid && mrd_ready) begin
            pending <= 1;
            pending_index <= mrd_y * 256 + mrd_x;
            delay_left <= cycles % 4;
            if (mrd_x > 255 || mrd_y > 255) bounds_ok = 0;
        end
        if (collecting) begin
            occupancy = dut.q_tail - dut.q_head;
            if (dut.state >= 6'd3 && dut.state <= 6'd7 && occupancy > peak_queue)
                peak_queue = occupancy;
            if (dut.state == 6'd3 && dut.q_head != dut.q_tail && (sx != gx || sy != gy))
                expanded = expanded + 1;
            if (held && (!wp_valid || wp_x !== held_x || wp_y !== held_y || wp_last !== held_last))
                stable_ok = 0;
            held = wp_valid && !wp_ready;
            held_x = wp_x;
            held_y = wp_y;
            held_last = wp_last;
            if (wp_valid && wp_ready) begin
                points = points + 1;
                $fdisplay(waypoints, "%0d,%0d,%0d,%0d,%0d", case_index, points, wp_x, wp_y, wp_last);
                dx = $signed({1'b0, wp_x}) - current_x;
                dy = $signed({1'b0, wp_y}) - current_y;
                if (dx != 0 && dy != 0) geometry_ok = 0;
                if (wp_x > 255 || wp_y > 255) bounds_ok = 0;
                step_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
                step_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
                if (dx != 0 || dy != 0) begin
                    direction = dx > 0 ? 0 : (dx < 0 ? 1 : (dy > 0 ? 2 : 3));
                    if (last_dir >= 0 && last_dir != direction) turns = turns + 1;
                    last_dir = direction;
                end
                for (i = 0; i < (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy); i = i + 1) begin
                    next_x = current_x + step_x;
                    next_y = current_y + step_y;
                    index = next_y * 256 + next_x;
                    if (next_x < 0 || next_x > 255 || next_y < 0 || next_y > 255) bounds_ok = 0;
                    else if (map[index][1] || (map[index][0] &&
                             !(escape && map[start_y*256+start_x][0] &&
                               next_x >= start_x-4 && next_x <= start_x+4 &&
                               next_y >= start_y-4 && next_y <= start_y+4))) geometry_ok = 0;
                    path_pos = path_pos + 1;
                    if (CENTER_EN == 0 && (path_pos >= 4096 || reference[path_pos] !== index[15:0]))
                        reference_ok = 0;
                    current_x = next_x;
                    current_y = next_y;
                    steps = steps + 1;
                end
                if (wp_last && (wp_x != gx || wp_y != gy)) geometry_ok = 0;
            end
        end
    end

    initial begin
        if (!$value$plusargs("FIXTURE_DIR=%s", fixture_dir)) fixture_dir = "../logs/bfs_straight/fixtures";
        $sformat(file_name, "%0s/cases.txt", fixture_dir);
        manifest = $fopen(file_name, "r");
        if (!manifest) begin
            check(0, "fixture manifest exists");
            done = 1;
        end else begin
            $sformat(file_name, "%0s/rtl_center_%0d.csv", fixture_dir, CENTER_EN);
            metrics = $fopen(file_name, "w");
            $fdisplay(metrics, "case,cycles,expanded,peak_queue,steps,turns,points,success,no_path");
            $sformat(file_name, "%0s/waypoints_center_%0d.csv", fixture_dir, CENTER_EN);
            waypoints = $fopen(file_name, "w");
            $fdisplay(waypoints, "case,point,x,y,last");
            scan_result = $fscanf(manifest, "%d\n", cases);
            repeat (4) @(negedge clk);
            reset_n = 1;
            repeat (2) @(negedge clk);
            repeat (cases) begin
                scan_result = $fscanf(manifest, "%d %d %d %d %d %d %d %d\n",
                                      case_index, start_x, start_y, goal_x, goal_y,
                                      allow_escape, expected_steps, expected_turns);
                check(scan_result == 8, "complete test case");
                $sformat(file_name, "%0s/map_%0d.hex", fixture_dir, case_index);
                $readmemh(file_name, map);
                if (expected_steps >= 0) begin
                    $sformat(file_name, "%0s/path_%0d.hex", fixture_dir, case_index);
                    $readmemh(file_name, reference, 0, expected_steps);
                end
                sx = start_x;
                sy = start_y;
                gx = goal_x;
                gy = goal_y;
                escape = allow_escape;
                current_x = start_x;
                current_y = start_y;
                last_dir = -1;
                steps = 0;
                turns = 0;
                points = 0;
                expanded = 0;
                peak_queue = 0;
                path_pos = 0;
                held = 0;
                geometry_ok = 1;
                reference_ok = 1;
                stable_ok = 1;
                bounds_ok = 1;
                collecting = 1;
                case_begin = cycles;
                req_valid = 1;
                @(negedge clk);
                req_valid = 0;
                wait_cycles = 0;
                while (busy && wait_cycles < 8000000) begin
                    @(negedge clk);
                    wait_cycles = wait_cycles + 1;
                end
                collecting = 0;
                check(!busy && (expected_steps >= 0 ? success && !no_path : no_path && !success),
                      "completion agrees with independent reachability");
                check(geometry_ok && bounds_ok, "all output segments are axial and traversable");
                check(stable_ok, "waypoint remains stable under backpressure");
                if (expected_steps >= 0) begin
                    check(points > 0 && current_x == goal_x && current_y == goal_y,
                          "final waypoint reaches requested goal");
                    check(dut.wk_steps == expected_steps, "BFS walk keeps shortest grid distance");
                    if (CENTER_EN == 0)
                        check(reference_ok && steps == expected_steps && turns == expected_turns,
                              "every BFS path cell and turn matches independent reference");
                    else check(steps >= expected_steps && steps <= expected_steps + 20,
                               "centering respects added-length budget");
                end else check(points == 0, "no route emitted on blocked map");
                check(peak_queue < 8192, "queue capacity not exceeded");
                if (case_index == 0) check(dut.q_tail > 8192, "queue ring wraps safely during room search");
                $fdisplay(metrics, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d", case_index,
                          cycles-case_begin, expanded, peak_queue, steps, turns, points, success, no_path);
                $display("METRICS: planner center=%0d case=%0d cycles=%0d expanded=%0d peak=%0d steps=%0d turns=%0d",
                         CENTER_EN, case_index, cycles-case_begin, expanded, peak_queue, steps, turns);
                repeat (3) @(negedge clk);
            end
            $fclose(manifest);
            $fclose(metrics);
            $fclose(waypoints);
            done = 1;
        end
    end
endmodule

module nav_planner_tb;
    wire done_raw, done_center;
    wire [31:0] checks_raw, checks_center, errors_raw, errors_center;
    nav_planner_check #(.CENTER_EN(0)) raw_path(done_raw, checks_raw, errors_raw);
    nav_planner_check #(.CENTER_EN(1)) centered_path(done_center, checks_center, errors_center);
    initial begin
        wait (done_raw && done_center);
        $display("CHECKS: %0d", checks_raw + checks_center);
        $display("METRICS: nav_planner_tb cycles=%0d errors=%0d", $time/20, errors_raw + errors_center);
        if (errors_raw + errors_center == 0) $display("PASS: nav_planner_tb");
        $finish;
    end
    initial begin
        #2000000000;
        $display("FAIL: nav_planner_tb timeout");
        $finish;
    end
endmodule
