//////////////////////////////////////////////////////////////////////////////////
// odom_tb —— 航位推算单元测试（覆盖各方向移动与旋转）
//
// 【坐标约定（需求方 2026-10-18 明确）】
//   · 初始车头朝 **+y**；车辆**右侧**为 **+x**；**向右转 ⇒ 航向角增大**
//   ⇒ 车头单位向量 = ( sin(yaw), cos(yaw) )
//      yaw=0     → (0,+1) 朝 +y
//      yaw=+90°  → (+1,0) 朝 +x（右转 90°）
//      yaw=±180° → (0,−1) 朝 −y
//      yaw=−90°  → (−1,0) 朝 −x
//
// 【数值基准】轮半径 32mm ⇒ D=64mm、61440 counts/轮转
//   1 count = πD/61440 = 3.2726e-3 mm；1024 counts/窗口 ⇒ 3.351 mm
//   100 窗口 ⇒ **335 mm**；对角方向各分量为 335/√2 ≈ **237 mm**
//
// 覆盖：静止 / 4 个正交方向 / 对角 / 倒车 / 角度解缠（跨 ±180°）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module odom_tb;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    reg signed [31:0] d0, d1, d2, d3;
    reg               dvalid = 1'b0;
    reg signed [31:0] yaw = 32'sd0;
    reg               yvalid = 1'b0;
    reg               clr = 1'b0;

    wire signed [31:0] ox, oy, oyaw, ov, oom;

    integer errors = 0;

    odom #(
        .CLK_HZ      (1_000_000),
        .WINDOW_MS   (10),
        .CNT_PER_REV (61440),
        .WHEEL_R_MM  (32),
        .TRACK_MM10  (1615),
        // ★ 本台显式传【理论值 3431】，用来验证算法本身（335mm 等期望值都按它算）。
        //   顶层生产配置用的是实测校正值 3525（见 odom.v 参数区），
        //   由于是同一乘数的线性缩放，算法正确性不受影响。
        .DS_K_Q10    (3431)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_delta0    (d0), .i_delta1 (d1), .i_delta2 (d2), .i_delta3 (d3),
        .i_delta_valid(dvalid),
        .i_yaw       (yaw),
        .i_yaw_valid (yvalid),
        .i_clear     (clr),
        .o_x_mm      (ox),
        .o_y_mm      (oy),
        .o_yaw_cont  (oyaw),
        .o_v_mms     (ov),
        .o_omega_enc (oom)
    );

    always #500 clk = ~clk;          // 1 MHz

    task chk;
        input [8*28-1:0] name;
        input integer    got;
        input integer    exp;
        input integer    tol;
        begin
            if ((got - exp <= tol) && (exp - got <= tol)) begin
                $display("  pass  %0s = %0d (exp %0d ±%0d)", name, got, exp, tol);
            end else begin
                errors = errors + 1;
                $display("  FAIL  %0s = %0d (exp %0d ±%0d)", name, got, exp, tol);
            end
        end
    endtask

    // 一个 10ms 窗口：10 拍 yaw 更新 + 一个 delta_valid
    task one_window;
        input signed [31:0] dyaw_per_ms;
        input signed [31:0] cnt;
        integer k;
        begin
            for (k = 0; k < 10; k = k + 1) begin
                yaw    = yaw + dyaw_per_ms;
                yvalid = 1'b1;
                @(posedge clk); #1;
            end
            yvalid = 1'b0;
            d0 = cnt; d1 = cnt; d2 = cnt; d3 = cnt;
            dvalid = 1'b1;
            @(posedge clk); #1;
            dvalid = 1'b0;
            d0 = 0; d1 = 0; d2 = 0; d3 = 0;
            @(posedge clk); #1;
        end
    endtask

    // ★ 航位用【相对航向】：清零时刻的车头方向恒为 +y。
    //   所以流程是"先把 yaw 归到基准并清零 → 再转到相对角 ang"。
    task reset_and_turn;
        input signed [31:0] ang;
        begin
            yaw = 32'sd0; yvalid = 1'b1; @(posedge clk); #1; yvalid = 1'b0;
            repeat (2) @(posedge clk);
            clr = 1'b1; @(posedge clk); #1; clr = 1'b0;
            repeat (2) @(posedge clk);
            // 清零后转到相对角（yaw_wrap 会从 0 加上 ang）
            yaw = ang; yvalid = 1'b1; @(posedge clk); #1; yvalid = 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    initial begin
        $display("======== odom 单元测试（坐标约定：yaw=0 朝 +y，右转 yaw 增大）========");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ---- 1. 静止 ----
        $display("-- 1. 静止 10 窗口：x/y 保持 0 --");
        reset_and_turn(32'sd0);
        repeat (10) one_window(32'sd0, 32'sd0);
        chk("idle_x", ox, 0, 0);
        chk("idle_y", oy, 0, 0);
        chk("idle_v", ov, 0, 1);

        // ---- 2. yaw=0 前进：位移应全部落在 +y ----
        $display("-- 2. 清零后直接前进 100 窗口（车头=+y）：x≈0, y≈+335 --");
        reset_and_turn(32'sd0);
        repeat (100) one_window(32'sd0, 32'sd1024);
        chk("north_x", ox, 0, 2);
        chk("north_y", oy, 335, 3);
        chk("north_v", ov, 335, 3);

        // ---- 3. yaw=0 倒车：位移全部落在 −y ----
        $display("-- 3. 清零后倒车 100 窗口：x≈0, y≈-335 --");
        reset_and_turn(32'sd0);
        repeat (100) one_window(32'sd0, -32'sd1024);
        chk("back_x", ox, 0, 2);
        chk("back_y", oy, -335, 3);

        // ---- 4. yaw=+90°（右转 90°）前进：全部落在 +x ----
        $display("-- 4. 清零后右转 90° 再前进 100 窗口：x≈+335, y≈0 --");
        reset_and_turn(32'sd9000);
        repeat (100) one_window(32'sd0, 32'sd1024);
        chk("east_x", ox, 335, 8);
        chk("east_y", oy, 0, 8);

        // ---- 5. yaw=+180° 前进：全部落在 −y ----
        $display("-- 5. 清零后转 180° 再前进 100 窗口：x≈0, y≈-335 --");
        reset_and_turn(32'sd18000);
        repeat (100) one_window(32'sd0, 32'sd1024);
        chk("south_x", ox, 0, 2);
        chk("south_y", oy, -335, 3);

        // ---- 6. yaw=-90°（左转 90°）前进：全部落在 −x ----
        $display("-- 6. 清零后左转 90° 再前进 100 窗口：x≈-335, y≈0 --");
        reset_and_turn(-32'sd9000);
        repeat (100) one_window(32'sd0, 32'sd1024);
        chk("west_x", ox, -335, 8);
        chk("west_y", oy, 0, 8);

        // ---- 7. yaw=+45° 前进：x 与 y 同为正、各 ≈237 ----
        $display("-- 7. 清零后右转 45° 再前进 100 窗口：x≈+237, y≈+237 --");
        reset_and_turn(32'sd4500);
        repeat (100) one_window(32'sd0, 32'sd1024);
        chk("ne_x", ox, 237, 6);
        chk("ne_y", oy, 237, 6);

        // ---- 8. ★ 角度解缠：跨过 +180° 回绕 ----
        $display("-- 8. 解缠：yaw 从 +17900 起持续 +50/ms，跨过 +18000 --");
        yaw = 32'sd17900; yvalid = 1'b1; @(posedge clk); #1; yvalid = 1'b0;
        repeat (2) @(posedge clk);
        clr = 1'b1; @(posedge clk); #1; clr = 1'b0;
        repeat (2) @(posedge clk);
        chk("unwrap_start", oyaw, 0, 0);
        begin : wrap_test
            integer before;
            before = oyaw;
            repeat (4) one_window(32'sd50, 32'sd0);   // 4×10ms×50 = +2000
            chk("unwrap_delta", oyaw - before, 2000, 20);
        end

        $display("======== 结果 ========");
        if (errors == 0) $display("PASS: odom all checks passed");
        else             $display("FAIL: %0d checks failed", errors);
        $finish;
    end

    initial begin
        #80_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule
