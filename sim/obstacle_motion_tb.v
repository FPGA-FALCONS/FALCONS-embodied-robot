`timescale 1ns/1ps
// 应用闭环：真实 UI、BFS 和目标点控制器。传感器边界在本台用有效射线输入；
// UART / 极坐标转换另由 radar_map_tb 覆盖。50 MHz 周期不压缩，运动每 10 ms 推进。
// 理想差速底盘瞬时响应速度 / 刹车，仅验证连接和流程，不代表实车制动性能。
module obstacle_motion_tb;
    `include "nav_obstacle_states.vh"
    reg clk = 0, rst_n = 0, active = 0, manual = 0;
    always #10 clk = ~clk;
    reg confirm = 0, back = 0, down = 0;
    reg signed [11:0] steer = 0, throttle = 0;
    reg signed [31:0] x = 25, y = 25, yaw = 0, velocity = 0;
    wire run, paused, goal, request, estop, done, linear, fault, auto_valid;
    wire [7:0] gx, gy;
    wire [3:0] nav_state;
    wire [2:0] ui_state;
    wire signed [31:0] tx, ty, center, diff;
    wire [31:0] rounds;
    integer cycles = 0, checks = 0, errors = 0, tick_count = 0, ticks = 0;
    integer manual_tick, old_x;
    reg tick = 0;
    real px = 25.0, py = 25.0, heading = 0.0, speed, rotation;
    always @(posedge clk) cycles <= cycles + 1;
    always @(negedge clk) begin
        tick = 0;
        if (rst_n) begin
            if (tick_count == 499999) begin
                tick_count = 0;
                ticks = ticks + 1;
                speed = manual ? 750.0 : auto_valid && !estop ? $itor(center) * 0.33617 : 0.0;
                rotation = auto_valid && !estop ? $itor(diff) * 0.23853 : 0.0;
                heading = heading + rotation * 0.01;
                if (heading >= 180.0) heading = heading - 360.0;
                if (heading < -180.0) heading = heading + 360.0;
                px = px + speed * $cos(heading * 0.0174532925199433) * 0.01;
                py = py + speed * $sin(heading * 0.0174532925199433) * 0.01;
                x = $rtoi(px);
                y = $rtoi(py);
                yaw = $rtoi(heading * 100.0);
                velocity = $rtoi(speed);
                tick = 1;
                if (ticks % 100 == 0)
                    $display("PROGRESS: t=%0dms xy=(%0d,%0d) yaw=%0d nav=%0d cmd=%0b stop=%0b done=%0b", ticks*10, x,y,yaw,nav_state,request,estop,done);
            end else tick_count = tick_count + 1;
        end
    end

    nav_obstacle_ui #(.CLK_HZ(1000)) ui (
        .clk(clk), .rst_n(rst_n), .i_active(active), .i_clear(1'b0),
        .i_x_mm(x), .i_y_mm(y), .i_manual(manual), .i_steer(steer), .i_throttle(throttle),
        .i_confirm(confirm), .i_back(back), .i_aux_up(1'b0), .i_aux_down(down),
        .i_nav_state(nav_state), .i_command_idle(!request && !estop && !done),
        .i_pause(1'b0), .i_ctrl_fault(fault),
        .o_cur_x_mm(), .o_cur_y_mm(), .o_aux_sel(), .o_pt_count(), .o_nav_state(ui_state),
        .o_nav_ui_active(), .o_nav_run(run), .o_nav_paused(paused),
        .o_goal_valid(goal), .o_goal_x_cell(gx), .o_goal_y_cell(gy),
        .o_pt_px(), .o_pt_py(), .o_pt_v(), .o_car_px(), .o_car_py(), .o_cur_px(), .o_cur_py()
    );

    reg obs_valid = 0, hit = 0, scan_start = 0;
    reg signed [15:0] hit_x = 0, hit_y = 0;
    reg signed [31:0] origin_x = 0, origin_y = 0, ray_x = 0;
    wire obs_ready;
    // 与整车相同：普通运动许可寄存，前探急停仍直连控制器。
    reg motion_enable;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) motion_enable <= 0;
        else motion_enable <= run;
    end
    nav_top core (
        .forward_check_cells(8'd4), .vehicle_width_mm(16'd180),
        .clk(clk), .reset_n(rst_n), .pose_valid(1'b1), .pose_x_mm(x), .pose_y_mm(y),
        .goal_valid(goal), .goal_x({8'd0, gx}), .goal_y({8'd0, gy}),
        .plan_enable(goal), .motion_enable(motion_enable), .display_enable(1'b0), .map_enable(1'b1),
        .ctrl_done(done), .ctrl_linear_active(linear), .obs_scan_start(scan_start), .obs_valid(obs_valid), .obs_ready(obs_ready),
        .obs_grid_x(hit_x), .obs_grid_y(hit_y), .obs_grid_valid(hit),
        .obs_origin_x_mm(origin_x), .obs_origin_y_mm(origin_y),
        .obs_ray_dx_q16(ray_x), .obs_ray_dy_q16(32'sd0),
        .scan_valid(1'b1), .stream_error(1'b0), .cmd_valid(request), .emergency_stop(estop),
        .target_x_mm(tx), .target_y_mm(ty), .nav_state(nav_state), .st_plan_rounds(rounds)
    );
    nav_point_ctrl #(.STOP_SETTLE_TICKS(2)) controller (
        .clk(clk), .rst_n(rst_n), .i_cmd_valid(request), .i_emergency_stop(estop),
        .i_target_x_mm(tx), .i_target_y_mm(ty), .i_tick(tick),
        .i_x_mm(x), .i_y_mm(y), .i_yaw_centi(yaw), .i_v_mms(velocity), .i_stall(4'd0),
        .i_v_cruise(32'sd1000), .i_d_slow(32'sd60), .i_align_in(32'sd300),
        .i_align_out(32'sd3500), .i_max_diff_cruise(32'sd126), .i_kp_yaw(32'sd40),
        .i_ki_yaw(32'sd3), .i_kd_yaw(32'sd6), .i_i_limit(32'sd120), .i_k_slow_q10(32'sd17066),
        .o_auto_valid(auto_valid), .o_center(center), .o_diff(diff),
        .o_done(done), .o_linear_active(linear), .o_fault(fault)
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
    task key;
        input integer which;
        begin
            @(negedge clk);
            case (which) 0: confirm = 1; 1: down = 1; 2: back = 1; endcase
            @(negedge clk);
            confirm = 0; down = 0; back = 0;
            repeat (5) @(negedge clk);
        end
    endtask
    task observation;
        input obstacle;
        begin
            @(negedge clk);
            while (!obs_ready) @(negedge clk);
            hit = obstacle;
            hit_x = (x + 6400) / 50 + 4;
            hit_y = (y + 6400) / 50;
            origin_x = x; origin_y = y;
            ray_x = obstacle ? 32'sd13107200 : 32'sd65536000;
            obs_valid = 1;
            @(negedge clk);
            obs_valid = 0;
            while (!obs_ready) @(negedge clk);
        end
    endtask
    integer stop_x, start_cycles;
    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1; active = 1;
        repeat (10) @(negedge clk);
        // UI 仅加速光标，导航 / 车辆 / 规划的实际时间不加速。
        steer = -1000;
        repeat (800) @(negedge clk);
        steer = 0;
        repeat (5) @(negedge clk);
        key(0);
        throttle = 1000;
        repeat (800) @(negedge clk);
        throttle = 0;
        repeat (5) @(negedge clk);
        key(0); key(1); key(1); key(0);
        wait (goal);
        check(gx == 136 && gy == 128 && run, "first user destination submitted");
        start_cycles = cycles;
        wait (x > 110);
        check(center > 0 && linear && !estop, "actual BFS starts point execution");
        key(2);
        wait (estop && done);
        check(paused && goal && center == 0, "pause preserves destination and acknowledges stopped");
        stop_x = x;
        repeat (500000) @(negedge clk);
        check(x == stop_x && velocity == 0, "paused vehicle remains stopped while planning advances");
        // 正常手动行驶 100 ms，跨越至少一格；定位与规划仍按真实周期推进。
        manual=1; manual_tick=ticks; old_x=x;
        wait(ticks>=manual_tick+10);
        manual=0;
        repeat(10) @(negedge clk);
        check(paused && x>=old_x+60 && gx==136 && gy==128,
              "manual movement crosses cells without consuming destination or resuming");
        key(0);
        wait (!estop && center > 0);
        check(run && tx>x, "resume replans forward from moved pose");
        observation(1);
        wait (estop && done);
        check(center == 0 && velocity == 0 && goal, "near raw obstacle stops vehicle without cancelling destination");
        scan_start = 1;
        observation(0);
        observation(0);
        scan_start = 0;
        wait (!estop && center > 0);
        check(goal && run, "clear observation permits replanning and resumption");
        wait (gx == 136 && gy == 136);
        check(goal && run, "arrival advances to second user destination");
        wait (nav_state == 3);
        check(x >= 395 && x <= 455 && y >= 395 && y <= 455 && center == 0,
              "second destination reached and stopped after corner turn");
        wait(ui_state==OBS_DONE);
        check(!goal && !run && !request, "final completion releases goal and control request");
        key(0);
        check(ui_state==OBS_DONE && !goal, "confirm cannot restart finished navigation");
        key(2);
        repeat(5) @(negedge clk);
        steer=-1000;
        repeat(400) @(negedge clk);
        steer=0;
        repeat(5) @(negedge clk);
        key(0); key(1); key(1); key(0);
        wait(goal);
        check(run && gx>136, "return-to-editor submits an independent new task");
        wait(ui_state==OBS_DONE);
        check(!goal && !run && center==0, "second complete navigation stops and releases its task");
        check(rounds > 2, "background planner runs throughout vehicle motion");
        $display("CHECKS: %0d", checks);
        $display("METRICS: obstacle_motion_tb cycles=%0d motion_cycles=%0d ticks=%0d rounds=%0d errors=%0d", cycles, cycles-start_cycles, ticks, rounds, errors);
        if (!errors) $display("PASS: obstacle_motion_tb");
        $finish;
    end
    initial begin
        #1.0e10;
        $display("FAIL: obstacle_motion_tb timeout xy=(%0d,%0d) nav=%0d", x, y, nav_state);
        $finish;
    end
endmodule
