//////////////////////////////////////////////////////////////////////////////////
// chassis_ctrl_tb —— 底盘运动控制单元测试（差速分解 + 四路速度环 + 安全逻辑）
//
// 【覆盖】
//   1. 中位指令                    → 左右目标 = 0
//   2. 满油门 throttle=+1000       → 目标 = +3072（= 3146×1000>>10，Q10 系数）
//   3. 满舵 steer=+1000, 油门 0    → 左 −612 / 右 +612（差速系数 627）
//   4. 后退 throttle=−1000         → −3072
//   5. 自动模式 + 过大给定         → 单轮目标被限幅到 MAX_DELTA=4096
//   6. i_enable=0                  → duty 全 0、o_enable 全 0
//   7. 急停 i_estop=1              → duty 全 0、o_brake 全 1（短路刹车）
//   8. 堵转 i_stall 任一路         → duty 全 0、o_enable 全 0（滑行停机，不刹车）
//   9. 闭环方向：目标>0 且反馈=0   → 四路 dir=0（正转）、duty 从 0 单调上升
//  10. rpm 换算：delta=1024       → rpm=100（÷10.24）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module chassis_ctrl_tb;

    localparam integer PWM_BITS = 11;

    reg  clk = 1'b0;
    reg  rst_n = 1'b0;

    reg               i_enable;
    reg               i_estop;
    reg               i_manual;
    reg signed [11:0] i_steer;
    reg signed [11:0] i_throttle;
    reg               i_auto_valid;
    reg signed [31:0] i_auto_center;
    reg signed [31:0] i_auto_diff;
    reg signed [15:0] i_kp, i_ki, i_kd;
    reg signed [31:0] i_delta0, i_delta1, i_delta2, i_delta3;
    reg               i_delta_valid;
    reg  [3:0]        i_stall;

    wire [4*PWM_BITS-1:0] o_duty;
    wire [3:0]            o_dir, o_enable, o_brake;
    wire signed [31:0]    o_tgt_left, o_tgt_right;
    wire signed [31:0]    o_rpm0, o_rpm1, o_rpm2, o_rpm3;
    wire [3:0]            o_sat;
    wire                  o_running;

    wire [PWM_BITS-1:0] d0 = o_duty[0*PWM_BITS +: PWM_BITS];
    wire [PWM_BITS-1:0] d1 = o_duty[1*PWM_BITS +: PWM_BITS];
    wire [PWM_BITS-1:0] d2 = o_duty[2*PWM_BITS +: PWM_BITS];
    wire [PWM_BITS-1:0] d3 = o_duty[3*PWM_BITS +: PWM_BITS];

    integer errors = 0;

    chassis_ctrl #(
        .PWM_BITS        (PWM_BITS),
        .CENTER_K_Q10    (3146),
        .DIFF_K_Q10      (627),
        .MAX_DELTA       (4096),
        .CNT_PER_RPM_Q10 (10486),
        .KP_INIT         (26),
        .KI_INIT         (3),
        .KD_INIT         (0)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_enable      (i_enable),
        .i_estop       (i_estop),
        .i_manual      (i_manual),
        .i_steer       (i_steer),
        .i_throttle    (i_throttle),
        .i_auto_valid  (i_auto_valid),
        .i_auto_center (i_auto_center),
        .i_auto_diff   (i_auto_diff),
        .i_kp          (i_kp),
        .i_ki          (i_ki),
        .i_kd          (i_kd),
        .i_delta0      (i_delta0),
        .i_delta1      (i_delta1),
        .i_delta2      (i_delta2),
        .i_delta3      (i_delta3),
        .i_delta_valid (i_delta_valid),
        .i_stall       (i_stall),
        .o_duty        (o_duty),
        .o_dir         (o_dir),
        .o_enable      (o_enable),
        .o_brake       (o_brake),
        .o_tgt_left    (o_tgt_left),
        .o_tgt_right   (o_tgt_right),
        .o_rpm0        (o_rpm0),
        .o_rpm1        (o_rpm1),
        .o_rpm2        (o_rpm2),
        .o_rpm3        (o_rpm3),
        .o_sat         (o_sat),
        .o_running     (o_running)
    );

    always #50 clk = ~clk;              // 10 MHz，便于观察流水

    // ------------------------------------------------------------------
    function integer absi;
        input integer v;
        begin absi = (v < 0) ? -v : v; end
    endfunction

    task check_int;
        input [8*44-1:0] name;
        input integer    got;
        input integer    exp;
        input integer    tol;
        begin
            if (absi(got - exp) > tol) begin
                errors = errors + 1;
                $display("  FAIL  %0s: got %0d, expect %0d (±%0d)", name, got, exp, tol);
            end else begin
                $display("  pass  %0s = %0d", name, got);
            end
        end
    endtask

    // 指令流水 2 级（乘法 1 拍 + 差速 1 拍）+ 余量
    task settle;
        begin repeat (6) @(posedge clk); end
    endtask

    // 模拟一次测速窗口（100 Hz 下的 i_delta_valid 脉冲）
    task fb_window;
        input signed [31:0] a, b, c, d;
        begin
            i_delta0 = a; i_delta1 = b; i_delta2 = c; i_delta3 = d;
            @(posedge clk);
            i_delta_valid = 1'b1;
            @(posedge clk);
            i_delta_valid = 1'b0;
            @(posedge clk);
        end
    endtask

    // ------------------------------------------------------------------
    initial begin
        $display("======== chassis_ctrl 单元测试（差速分解 + 4×速度环）========");

        i_enable = 1'b0; i_estop = 1'b0; i_manual = 1'b1;
        i_steer = 12'sd0; i_throttle = 12'sd0;
        i_auto_valid = 1'b0; i_auto_center = 32'sd0; i_auto_diff = 32'sd0;
        i_kp = 16'sd26; i_ki = 16'sd3; i_kd = 16'sd0;
        i_delta0 = 32'sd0; i_delta1 = 32'sd0; i_delta2 = 32'sd0; i_delta3 = 32'sd0;
        i_delta_valid = 1'b0; i_stall = 4'b0000;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        i_enable = 1'b1;
        settle;

        // ---------------- 1. 中位 ----------------
        $display("-- 1. 中位指令：目标应为 0 --");
        i_throttle = 12'sd0; i_steer = 12'sd0;
        settle;
        check_int("tgt_left_center", o_tgt_left,  0, 1);
        check_int("tgt_right_center", o_tgt_right, 0, 1);

        // ---------------- 2. 满油门前进 ----------------
        $display("-- 2. 满油门 +1000：目标应 = +3072（3146×1000>>10）--");
        i_throttle = 12'sd1000; i_steer = 12'sd0;
        settle;
        check_int("tgt_left_full_thr", o_tgt_left,  3072, 2);
        check_int("tgt_right_full_thr", o_tgt_right, 3072, 2);

        // ---------------- 3. 满舵原地转向 ----------------
        $display("-- 3. 满舵 steer=+1000、油门 0：左 −612 / 右 +612 --");
        i_throttle = 12'sd0; i_steer = 12'sd1000;
        settle;
        check_int("tgt_left_full_steer", o_tgt_left,  -612, 2);
        check_int("tgt_right_full_steer", o_tgt_right,  612, 2);

        // ---------------- 4. 后退 ----------------
        $display("-- 4. 油门 −1000：目标应 = −3072 --");
        i_throttle = -12'sd1000; i_steer = 12'sd0;
        settle;
        check_int("tgt_left_reverse", o_tgt_left,  -3072, 2);
        check_int("tgt_right_reverse", o_tgt_right, -3072, 2);

        // ---------------- 5. 自动模式限幅 ----------------
        $display("-- 5. 自动模式给定 center=5000/diff=2000：右轮应被限幅到 4096 --");
        i_manual = 1'b0; i_auto_valid = 1'b1;
        i_auto_center = 32'sd5000; i_auto_diff = 32'sd2000;
        settle;
        check_int("left_clamped", o_tgt_left,  3000, 2);        // 5000−2000
        check_int("right_clamped", o_tgt_right, 4096, 2);        // 7000 → 限幅 4096
        i_manual = 1'b1; i_auto_valid = 1'b0;
        i_auto_center = 32'sd0; i_auto_diff = 32'sd0;

        // ---------------- 6. 未使能 ----------------
        $display("-- 6. i_enable=0：duty 全 0、o_enable 全 0 --");
        i_enable = 1'b0; i_throttle = 12'sd1000;
        settle;
        check_int("duty0_disabled", d0, 0, 0);
        check_int("oenable_disabled", o_enable, 4'b0000, 0);

        // ---------------- 7. 急停 ----------------
        $display("-- 7. 急停：duty 全 0 且 o_brake=1111（短路刹车）--");
        i_enable = 1'b1; i_estop = 1'b1;
        settle;
        check_int("duty0_estop", d0, 0, 0);
        check_int("obrake_estop", o_brake, 4'b1111, 0);
        i_estop = 1'b0;
        settle;

        // ---------------- 8. 堵转 ----------------
        $display("-- 8. 一路堵转：全部停机（滑行，不刹车）--");
        i_stall = 4'b0001;
        settle;
        check_int("duty0_stall", d0, 0, 0);
        check_int("oenable_stall", o_enable, 4'b0000, 0);
        check_int("obrake_stall_0", o_brake, 4'b0000, 0);
        i_stall = 4'b0000;
        settle;

        // ---------------- 9. 闭环方向与上升 ----------------
        $display("-- 9. 闭环：目标>0、反馈=0 → dir=0（正转），duty 单调上升 --");
        i_throttle = 12'sd1000; i_steer = 12'sd0;
        settle;
        fb_window(32'sd0, 32'sd0, 32'sd0, 32'sd0);
        check_int("dir0_fwd", o_dir[0], 0, 0);
        check_int("duty0_rising", (d0 > 0) ? 1 : 0, 1, 0);
        begin : ramp
            integer prev;
            integer rises;
            integer k;
            prev = d0;
            rises = 0;
            for (k = 0; k < 6; k = k + 1) begin
                fb_window(32'sd0, 32'sd0, 32'sd0, 32'sd0);
                if (d0 > prev) rises = rises + 1;
                prev = d0;
            end
            check_int("ramp_6_windows", rises, 6, 0);
        end
        check_int("running_1", o_running, 1, 0);

        // 反馈跟上目标 → 输出不再上升
        $display("-- 9b. 反馈=目标 → 输出应停止上升（增量式 PID 特性）--");
        begin : hold
            integer duty_before;
            integer k;
            duty_before = d0;
            for (k = 0; k < 4; k = k + 1) fb_window(32'sd3072, 32'sd3072, 32'sd3072, 32'sd3072);
            check_int("duty_holds_at_target", (d0 > duty_before) ? 1 : 0, 0, 0);
        end

        // ---------------- 10. rpm 换算 ----------------
        $display("-- 10. 反馈换算：delta=1024 → rpm=100（÷10.24）--");
        fb_window(32'sd1024, 32'sd1024, 32'sd1024, 32'sd1024);
        check_int("rpm0", o_rpm0, 100, 1);

        // ---------------- 汇总 ----------------
        $display("======== 结果 ========");
        if (errors == 0) $display("PASS: chassis_ctrl 全部检查通过");
        else             $display("FAIL: %0d 项检查未通过", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("FAIL: 仿真超时");
        $finish;
    end

endmodule
