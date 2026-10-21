`timescale 1ns/1ps
// 接实际 odom 流水，确认同一次测速窗口的坐标被导航采到。
module nav_pose_tb;
    reg clk = 0, rst_n = 0, tick = 0, clear = 0, ready = 0;
    always #10 clk = ~clk;
    wire signed [31:0] ox, oy, px, py;
    wire valid, pose_tick;
    integer pose_events = 0;
    reg signed [31:0] control_x;
    always @(posedge clk) begin
        if (pose_tick) begin
            pose_events <= pose_events + 1;
            control_x <= px;
        end
    end
    odom odometry (
        .clk(clk), .rst_n(rst_n), .i_delta0(32'sd10000), .i_delta1(32'sd10000),
        .i_delta2(32'sd10000), .i_delta3(32'sd10000), .i_delta_valid(tick),
        .i_yaw(32'sd0), .i_yaw_valid(1'b1), .i_clear(clear),
        .o_x_mm(ox), .o_y_mm(oy), .o_yaw_cont(), .o_v_mms(), .o_omega_enc()
    );
    nav_pose dut (
        .clk(clk), .rst_n(rst_n), .clear(clear), .ready(ready), .position_tick(tick),
        .x_mm(ox), .y_mm(oy), .yaw_cdeg(32'sd0), .pose_valid(valid),
        .pose_x_mm(px), .pose_y_mm(py), .pose_yaw_cdeg(), .pose_tick(pose_tick)
    );
    integer cycles = 0, checks = 0, errors = 0;
    always @(posedge clk) cycles <= cycles + 1;
    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s odom=%0d pose=%0d", message, ox, px);
            end else $display("pass %0s", message);
        end
    endtask
    task update;
        begin
            @(negedge clk); tick = 1;
            @(negedge clk); tick = 0;
            repeat (5) @(negedge clk);
        end
    endtask
    initial begin
        repeat (5) @(negedge clk); rst_n = 1;
        repeat (4) @(negedge clk);
        check(!valid, "initialization holds navigation pose invalid");
        ready = 1; update();
        check(valid && px == ox && px > 0 && py == oy, "first window uses updated odometry position");
        check(pose_events == 1 && control_x == ox, "controller tick observes newly captured pose");
        update();
        check(valid && px == ox && px > 50, "next window does not lag one odometry period");
        check(pose_events == 2 && !pose_tick, "one controller tick per odometry window");
        clear = 1;
        repeat (2) @(negedge clk);
        check(!valid, "coordinate clear invalidates navigation pose");
        clear = 0; update();
        check(valid && px == ox, "clear resumes from same origin as odometry");
        ready = 0;
        repeat (2) @(negedge clk);
        check(!valid, "loss of ready invalidates pose");
        $display("CHECKS: %0d", checks);
        $display("METRICS: nav_pose_tb cycles=%0d errors=%0d", cycles, errors);
        if (!errors) $display("PASS: nav_pose_tb");
        $finish;
    end
endmodule
