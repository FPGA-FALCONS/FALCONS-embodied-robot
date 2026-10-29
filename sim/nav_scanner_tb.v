`timescale 1ns/1ps
// 独立浮点格子/线段求交参考，检查原始命中、遮挡、10 m 截止和背压。
module nav_scanner_tb;
    reg clk = 0, reset_n = 0, valid = 0, enabled = 1, hit = 1;
    always #10 clk = ~clk;
    reg signed [15:0] gx = 0, gy = 0;
    reg signed [31:0] ox = 0, oy = 0, dx = 0, dy = 0;
    reg [13:0] distance = 1000;
    wire ready, wv, value, shadow, busy;
    wire [15:0] wx, wy;
    integer cycles = 0, errors = 0, checks = 0, tests = 0;
    integer count = 0, hits = 0, shadows = 0, start_cycle, max_cycles = 0;
    integer x, y, a, expected_count, expected_hits, expected_shadows, mismatch;
    integer seen [0:65535];
    reg [1:0] expected [0:65535];
    reg stalled = 0, old_value, old_shadow;
    reg [15:0] old_x, old_y;
    wire wr_ready = cycles % 17 < 11;
    real theta, vx, vy, limit, low, high, lo, hi, temp, enter_t;
    integer start_x, start_y, end_x, end_y;
    nav_scanner dut (
        .clk(clk), .reset_n(reset_n), .obs_valid(valid), .obs_ready(ready),
        .grid_x(gx), .grid_y(gy), .grid_valid(hit), .origin_x_mm(ox), .origin_y_mm(oy),
        .ray_dx_q16(dx), .ray_dy_q16(dy), .range_mm(distance), .map_enable(enabled),
        .wr_valid(wv), .wr_ready(wr_ready), .wr_val(value), .wr_shadow(shadow),
        .wr_x(wx), .wr_y(wy), .busy(busy), .accepted(), .dropped(), .map_dropped()
    );
    always @(posedge clk) begin
        cycles = cycles + 1;
        if (reset_n) begin
            if (stalled && (!wv || wx != old_x || wy != old_y ||
                            value != old_value || shadow != old_shadow)) begin
                errors = errors + 1;
                $display("FAIL: scanner payload changed under backpressure");
            end
            stalled = wv && !wr_ready;
            old_x = wx; old_y = wy; old_value = value; old_shadow = shadow;
            if (wv && wr_ready) begin
                a = wy * 256 + wx;
                seen[a] = seen[a] + 1;
                count = count + 1;
                if (value) hits = hits + 1;
                if (shadow) shadows = shadows + 1;
                if ({shadow, value} !== expected[a] || value && shadow) begin
                    errors = errors + 1;
                    $display("FAIL: scanner event test=%0d cell=(%0d,%0d) flags=%b%b expected=%b",
                             tests, wx, wy, shadow, value, expected[a]);
                end
            end
        end
    end
    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin errors = errors + 1; $display("FAIL: %0s", message); end
            else $display("pass %0s", message);
        end
    endtask
    task ray;
        input integer origin_x, origin_y, angle, r;
        input obstacle;
        begin
            @(negedge clk);
            while (!ready) @(negedge clk);
            ox = origin_x; oy = origin_y; distance = r; hit = obstacle;
            theta = angle * 3.141592653589793 / 18000.0;
            dx = $rtoi($cos(theta) * r * 65536.0);
            dy = $rtoi($sin(theta) * r * 65536.0);
            vx = $itor(dx) / 65536.0; vy = $itor(dy) / 65536.0;
            gx = $rtoi($floor((origin_x + vx + 6400.0) / 50.0));
            gy = $rtoi($floor((origin_y + vy + 6400.0) / 50.0));
            limit = obstacle ? 10000.0 / r : 1.0;
            start_x = (origin_x + 6400) / 50;
            start_y = (origin_y + 6400) / 50;
            // 遮挡截止允许记录恰在 10 m 格界处进入的格，不沿边扩展整片。
            end_x = $rtoi($floor((origin_x + vx * (limit + 1e-10) + 6400.0) / 50.0));
            end_y = $rtoi($floor((origin_y + vy * (limit + 1e-10) + 6400.0) / 50.0));
            if (!obstacle) begin end_x = gx; end_y = gy; end
            expected_count = 0; expected_hits = 0; expected_shadows = 0;
            for (y = 0; y < 256; y = y + 1)
                for (x = 0; x < 256; x = x + 1) begin
                    a = y * 256 + x;
                    seen[a] = 0; expected[a] = 3;
                    low = 0.0; high = limit;
                    if (dx == 0) begin
                        if (x != start_x) high = -1.0;
                    end else begin
                        lo = (x * 50.0 - 6400.0 - origin_x) / vx;
                        hi = (x * 50.0 - 6350.0 - origin_x) / vx;
                        if (lo > hi) begin temp = lo; lo = hi; hi = temp; end
                        if (lo > low) low = lo;
                        if (hi < high) high = hi;
                    end
                    if (dy == 0) begin
                        if (y != start_y) high = -1.0;
                    end else begin
                        lo = (y * 50.0 - 6400.0 - origin_y) / vy;
                        hi = (y * 50.0 - 6350.0 - origin_y) / vy;
                        if (lo > hi) begin temp = lo; lo = hi; hi = temp; end
                        if (lo > low) low = lo;
                        if (hi < high) high = hi;
                    end
                    if (high > low + 1e-10 || (x == start_x && y == start_y) ||
                        (x == end_x && y == end_y)) begin
                        if (obstacle && x == gx && y == gy) begin
                            expected[a] = 1; expected_hits = expected_hits + 1;
                        end else if (obstacle && low >= 1.0 - 1e-10) begin
                            expected[a] = 2; expected_shadows = expected_shadows + 1;
                        end else expected[a] = 0;
                        expected_count = expected_count + 1;
                    end
                end
            count = 0; hits = 0; shadows = 0; tests = tests + 1;
            start_cycle = cycles;
            valid = 1;
            @(negedge clk); valid = 0;
            while (!ready) @(negedge clk);
            if (cycles - start_cycle > max_cycles) max_cycles = cycles - start_cycle;
            mismatch = 0;
            for (a = 0; a < 65536; a = a + 1)
                if (seen[a] != (expected[a] != 3)) mismatch = mismatch + 1;
            check(!mismatch && count == expected_count && hits == expected_hits && shadows == expected_shadows,
                  "all free/hit/shadow cells match independent floating-point intersections");
        end
    endtask
    initial begin
        repeat (4) @(negedge clk); reset_n = 1;
        ray(25, 25, 0, 1000, 1);
        ray(25, 25, 9000, 1000, 1);
        ray(25, 25, 18000, 1000, 1);
        ray(25, 25, 27000, 1000, 1);
        ray(25, 25, 4500, 1000, 1);
        ray(25, 25, 13500, 1000, 1);
        ray(25, 25, 22500, 1000, 1);
        ray(25, 25, 31500, 1000, 1);
        ray(17, -31, 1730, 777, 1);
        ray(17, -31, 25891, 4321, 1);
        ray(0, 0, 18000, 1000, 1);
        ray(-6200, 25, 0, 500, 1);
        ray(6200, 25, 18000, 500, 1);
        ray(-6399, -6399, 4500, 1, 1);
        ray(0, 0, 4500, 10000, 0);
        ray(-6200, 25, 0, 10000, 0);
        ray(17, -31, 25891, 10000, 0);
        @(negedge clk); enabled = 0; valid = 1;
        @(negedge clk); valid = 0;
        repeat (4) @(negedge clk);
        check(ready && !wv, "disabled mapping drops the observation without emitting protection");
        $display("CHECKS: %0d", checks);
        $display("METRICS: nav_scanner cycles=%0d rays=%0d max_ray_cycles=%0d errors=%0d", cycles, tests, max_cycles, errors);
        if (!errors) $display("PASS: nav_scanner_tb");
        $finish;
    end
    initial begin #10000000; $display("FAIL: nav_scanner_tb timeout"); $finish; end
endmodule
