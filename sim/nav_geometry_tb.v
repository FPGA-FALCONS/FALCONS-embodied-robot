`timescale 1ns/1ps
// 固定 256×256、50 mm 地图的毫米 / 格号边界及 10 m 射线几何。
module nav_geometry_tb;
    reg clk = 0, reset_n = 0, in_valid = 0, out_ready = 0;
    always #10 clk = ~clk;
    reg signed [31:0] px = 0, py = 0, yaw = 0;
    reg [15:0] angle = 0, distance = 0;
    wire in_ready, out_valid, hit, rejected;
    wire signed [15:0] gx, gy;
    wire signed [31:0] ox, oy, dx_q16, dy_q16;
    integer heading_index, sweep_angle, sweep_range, sweep_checks = 0, sweep_errors = 0;
    real theta, expected_dx, expected_dy, error_dx, error_dy;
    integer cycles = 0, checks = 0, errors = 0, mm, expected, mismatches = 0;
    always @(posedge clk) cycles <= cycles + 1;
    nav_polar dut (
        .clk(clk), .reset_n(reset_n), .in_valid(in_valid), .in_scan_start(1'b0),
        .in_ready(in_ready), .in_angle_cdeg(angle), .in_distance_mm(distance),
        .in_pose_valid(1'b1), .in_pose_x_mm(px), .in_pose_y_mm(py), .in_pose_yaw_cdeg(yaw),
        .out_valid(out_valid), .out_scan_start(), .out_ready(out_ready),
        .out_hit(hit), .out_hit_x(gx), .out_hit_y(gy),
        .out_origin_x_mm(ox), .out_origin_y_mm(oy), .out_ray_dx_q16(dx_q16), .out_ray_dy_q16(dy_q16),
        .out_range_mm(), .out_rejected(rejected)
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
    task ray;
        input signed [31:0] x, y, heading;
        input [15:0] a, d;
        input expected_hit;
        input signed [15:0] expected_x, expected_y;
        begin
            @(negedge clk);
            while (!in_ready) @(negedge clk);
            px = x; py = y; yaw = heading; angle = a; distance = d; in_valid = 1;
            @(negedge clk); in_valid = 0;
            wait (out_valid);
            repeat (3) @(negedge clk);
            check(hit == expected_hit && gx == expected_x && gy == expected_y && ox == x && oy == y,
                  "millimeter endpoint and sampled origin remain exact under backpressure");
            out_ready = 1;
            @(negedge clk); out_ready = 0;
        end
    endtask
    // 普通角度必须检验位移；四个正轴的精确值分支不能覆盖 CORDIC。
    task sweep_ray;
        input integer a, d;
        begin
            @(negedge clk);
            while (!in_ready) @(negedge clk);
            angle = a;
            distance = d;
            in_valid = 1;
            @(negedge clk);
            in_valid = 0;
            wait (out_valid);
            repeat (3) @(negedge clk);
            theta = ($itor(yaw) - a) * 3.141592653589793 / 18000.0;
            expected_dx = d * $cos(theta);
            expected_dy = d * $sin(theta);
            error_dx = $itor(dx_q16) / 65536.0 - expected_dx;
            error_dy = $itor(dy_q16) / 65536.0 - expected_dy;
            sweep_checks = sweep_checks + 1;
            if ((^dx_q16 === 1'bx) || (^dy_q16 === 1'bx) ||
                error_dx * error_dx + error_dy * error_dy >= 1.0) begin
                sweep_errors = sweep_errors + 1;
                if (sweep_errors <= 8)
                    $display("RAY_MISMATCH yaw=%0d angle=%0d distance=%0d dx=%0d dy=%0d",
                             yaw, a, d, dx_q16, dy_q16);
            end
            out_ready = 1;
            @(negedge clk);
            out_ready = 0;
        end
    endtask
    initial begin
        repeat (4) @(negedge clk); reset_n = 1;
        ray(0, 0, 0, 0, 49, 1, 128, 128);
        ray(0, 0, 0, 0, 50, 1, 129, 128);
        ray(0, 0, 0, 0, 51, 1, 129, 128);
        ray(0, 0, 0, 18000, 1, 1, 127, 128);
        ray(0, 0, 0, 18000, 50, 1, 127, 128);
        ray(0, 0, 0, 18000, 51, 1, 126, 128);
        ray(0, 0, 0, 27000, 50, 1, 128, 129);
        ray(0, 0, 9000, 9000, 50, 1, 129, 128);
        ray(-6400, 0, 0, 0, 1, 1, 0, 128);
        ray(-6351, 0, 0, 0, 1, 1, 1, 128);
        ray(6398, 0, 0, 0, 1, 1, 255, 128);
        ray(6399, 0, 0, 0, 1, 0, 256, 128);
        ray(-6400, 0, -18000, 0, 1, 0, -1, 128);
        ray(0, 0, 0, 0, 10000, 0, 328, 128);
        ray(0, 0, 0, 0, 0, 0, 328, 128);

        // 独立整数除法检查整个可达量化输入范围，包括负的图外坐标。
        for (mm = -10000; mm <= 22800; mm = mm + 1) begin
            expected = mm < 0 ? -((-mm + 49) / 50) : mm / 50;
            if ($signed(dut.signed_cell(mm)) != expected) mismatches = mismatches + 1;
        end
        check(mismatches == 0, "50 mm quantization matches exact floor for all 32801 offsets");
        @(negedge clk); px = 6400; distance = 50; in_valid = 1;
        @(negedge clk); in_valid = 0;
        check(rejected && !out_valid && in_ready, "positive map position limit is excluded");
        @(negedge clk); px = -6401; in_valid = 1;
        @(negedge clk); in_valid = 0;
        check(rejected && !out_valid && in_ready, "position below negative map limit is rejected");
        px = 0;
        py = 0;
        for (heading_index = 0; heading_index < 5; heading_index = heading_index + 1) begin
            case (heading_index)
                0: yaw = 0;
                1: yaw = 4500;
                2: yaw = -4500;
                3: yaw = 17999;
                4: yaw = -18000;
            endcase
            for (sweep_range = 4000; sweep_range <= 10000; sweep_range = sweep_range + 6000)
                for (sweep_angle = 0; sweep_angle < 36000; sweep_angle = sweep_angle + 37)
                    sweep_ray(sweep_angle, sweep_range);
        end
        check(sweep_errors == 0 && sweep_checks == 9730,
              "quadrants and five headings match trigonometry within 1 mm");
        $display("METRICS: angular_sweep samples=%0d errors=%0d", sweep_checks, sweep_errors);
        $display("CHECKS: %0d", checks);
        $display("METRICS: nav_geometry cycles=%0d quantized_offsets=32801 errors=%0d", cycles, errors);
        if (!errors) $display("PASS: nav_geometry_tb");
        $finish;
    end
    initial begin #10000000; $display("FAIL: nav_geometry_tb timeout"); $finish; end
endmodule
