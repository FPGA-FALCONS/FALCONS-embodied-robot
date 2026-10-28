`timescale 1ns/1ps
// 真实规划器到控制输出：内部短段、动态最远点、后台起点及暂停恢复。
module nav_lock_tb;
    reg clk = 0, reset_n = 0, motion = 1, normal_done = 0;
    always #10 clk = ~clk;
    reg signed [31:0] px = -175, py = 25;
    reg [7:0] window = 5;
    reg done = 0, check_anchor = 1;
    reg [15:0] goal_y = 8;
    wire command, emergency;
    wire signed [31:0] tx, ty;
    wire [3:0] nav_state;
    wire [15:0] route_id, length;
    integer cycles = 0, checks = 0, errors = 0, stop_count = 0;
    integer saved_index, saved_x, saved_y, old_id, first_cycles, scan_cycles, max_scan = 0, anchored = 0;
    reg scan_q = 0;

    nav_top #(
        .GRID_W(64), .GRID_H(32), .HALF_MM(400), .MAX_WP(64),
        .REPLAN_PERIOD(23'd25000)
    ) dut (
        .clk(clk), .reset_n(reset_n), .w_max(window),
        .forward_check_cells(8'd4), .vehicle_width_mm(16'd50),
        .pose_valid(1'b1), .pose_x_mm(px), .pose_y_mm(py),
        .goal_valid(1'b1), .goal_x(16'd28), .goal_y(goal_y),
        .plan_enable(1'b1), .motion_enable(motion),
        .display_enable(1'b0), .map_enable(1'b0),
        .ctrl_done(done), .ctrl_linear_active(command),
        .obs_valid(1'b0), .obs_scan_start(1'b0), .obs_ready(),
        .obs_grid_x(16'sd0), .obs_grid_y(16'sd0), .obs_grid_valid(1'b0),
        .obs_origin_x_mm(32'sd0), .obs_origin_y_mm(32'sd0),
        .obs_ray_dx_q16(32'sd0), .obs_ray_dy_q16(32'sd0),
        .scan_valid(1'b1), .stream_error(1'b0),
        .cmd_valid(command), .emergency_stop(emergency),
        .target_x_mm(tx), .target_y_mm(ty), .nav_state(nav_state),
        .st_route_id(route_id), .st_route_length(length)
    );

    always @(posedge clk) begin
        cycles <= cycles + 1;
        scan_q <= dut.drive_valid;
        if (dut.publish_route || (!dut.drive_valid && scan_q)) scan_cycles = cycles;
        if (dut.drive_valid && !scan_q && cycles - scan_cycles > max_scan)
            max_scan = cycles - scan_cycles;
        if (check_anchor && reset_n && dut.pl_req_valid && dut.pl_req_ready && dut.candidate_commit) begin
            anchored = anchored + 1;
            if (dut.plan_x != 8 || dut.plan_y != 8) begin
                errors = errors + 1;
                $display("FAIL: background origin moved away from the locked short endpoint");
            end
        end
    end

    // 控制器停车四相握手，正常完成由测试显式触发。
    always @(negedge clk) begin
        if (!reset_n) begin done = 0; stop_count = 0; end
        else if (emergency) begin
            if (stop_count < 3) begin stop_count = stop_count + 1; done = 0; end
            else done = 1;
        end else begin stop_count = 0; done = normal_done; end
    end

    task check;
        input condition;
        input [767:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s", message);
            end else $display("pass %0s", message);
        end
    endtask

    initial begin
        scan_cycles = 0;
        repeat (4) @(negedge clk); reset_n = 1;
        wait (command && tx == 1025);
        @(negedge clk); first_cycles = cycles;
        check(dut.r_target_x == 8 && dut.r_target_y == 8 && length == 6,
              "W_MAX=5 retains the first four-cell segment and all six relay endpoints");
        check(tx == 1025 && ty == 25 && dut.drive_index == 5,
              "controller receives the farthest collinear point, not the first relay");
        old_id = route_id;
        wait (route_id != old_id && dut.drive_valid && dut.commit_valid);
        repeat (5) @(negedge clk);
        check(dut.plan_x == 8 && dut.plan_y == 8 && command && !emergency,
              "background replans from the short endpoint while the long command remains active");

        // 位姿越过内部短段、且有横向误差，后台仍信任控制器，不改从实际位置规划。
        px = 250; py = 65; window = 3; old_id = route_id;
        wait (route_id != old_id && dut.drive_valid && length == 11);
        repeat (5) @(negedge clk);
        check(dut.r_target_x == 8 && dut.plan_x == 8 && dut.plan_y == 8,
              "crossing the short endpoint with pose error does not extend or move its lock");
        check(command && !emergency && tx == 1025 && ty == 25 && dut.drive_index == 10,
              "new window changes future relay count without changing the controller endpoint");
        check(anchored >= 2, "multiple real planner requests use the short endpoint");

        // 完成不复核实车是否到目标；最终完成沿用控制器握手。
        normal_done = 1;
        wait (!command);
        normal_done = 0;
        wait (nav_state == 3);
        check(!emergency, "final completion trusts the controller despite deliberate pose error");
        // 两段长度均大于默认短段，窗口变更后同一转弯目标对应不同索引。
        check_anchor = 0; reset_n = 0; normal_done = 0;
        px = -175; py = 25; window = 5; goal_y = 20;
        repeat (4) @(negedge clk); reset_n = 1;
        wait (command && dut.drive_valid);
        repeat (4) @(negedge clk);
        saved_x = tx; saved_y = ty; saved_index = dut.drive_index;
        check(!dut.final_target && saved_index >= 2,
              "a real bent route starts with multiple short segments before its first turn");
        old_id = route_id; window = 3;
        wait (route_id != old_id && dut.drive_valid && dut.commit_valid);
        repeat (5) @(negedge clk);
        check(tx == saved_x && ty == saved_y && dut.drive_index != saved_index,
              "publication retains the same controller endpoint with a different internal index");
        saved_index = dut.drive_index;
        normal_done = 1;
        wait (!command);
        normal_done = 0;
        wait (dut.target_index == saved_index + 1);
        @(negedge clk);
        check(dut.route_ready, "completion skips the full collinear run using the new route index");
        // 发布后同步读尚未完成，此时旧目标完成，不能套用旧编号继续走新路线。
        reset_n = 0; normal_done = 0; window = 5;
        repeat (4) @(negedge clk); reset_n = 1;
        wait (command && dut.drive_valid);
        old_id = route_id; window = 3;
        wait (route_id != old_id && !dut.drive_valid && dut.commit_valid);
        @(negedge clk); px = -125; py = 75; normal_done = 1;
        wait (!command);
        normal_done = 0;
        repeat (3) @(negedge clk);
        check(!dut.route_ready, "completion during route refresh revokes an unmappable old task");
        wait (dut.pl_req_valid && dut.pl_req_ready && !dut.candidate_commit);
        @(negedge clk);
        check(dut.plan_x == 5 && dut.plan_y == 9,
              "after confirmed stop an unmappable completion replans from the actual pose");
        $display("CHECKS: %0d", checks);
        $display("METRICS: nav_lock_tb cycles=%0d first_command=%0d max_lookup=%0d anchored=%0d errors=%0d",
                 cycles, first_cycles, max_scan, anchored, errors);
        if (!errors) $display("PASS: nav_lock_tb");
        $finish;
    end
    initial begin #100000000; $display("FAIL: nav_lock_tb timeout"); $finish; end
endmodule
