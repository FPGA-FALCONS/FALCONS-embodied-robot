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
//       ＋ **★ 移动中同时转向（中点积分）**（2026-10-18 新增，第 9 组）
//   ★ 第 9 组是唯一能区分「窗口端点积分」与「窗口中点积分」的用例：
//     端点法给出 (286, −7)、中点法给出 (285, +13)（期望 10 ±8），两者差 20 mm。
//     前面 8 组直行段的 dyaw_per_ms 全是 0，两种实现都 PASS —— 所以只跑旧用例
//     无法证明积分方法改对了。改 odom 的积分方式务必跑第 9 组。
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
            // ★ 2026-10-18 新增：再跑一个 **ds=0 的空窗口**把 `dyaw_acc` 清干净。
            //   为什么需要：上面"一步跳到 ang"是测试台的抽象（真实 1 kHz 采样下
            //   单拍不可能跳 180°）。而中点积分要在 `delta_valid` 拍才知道
            //   "本窗口转了多少"，于是这一跳会被记成"窗口内转了 ang"，
            //   中点 = ang/2 —— 第一个直行窗口的方向就偏了半个 ang。
            //   补一个 ds=0 的窗口：不产生位移（ds=0），但让 `dyaw_acc` 正常清零，
            //   后续窗口的 yaw_mid 才等于 yaw_wrap。
            one_window(32'sd0, 32'sd0);
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

        // ---- 9. ★ 移动中同时转向：验证「中点积分」（2026-10-18 新增）----
        //   为什么必须单独加这一条：前面 1~8 项直行段的 `dyaw_per_ms` 全是 0，
        //   也就是"移动中同时转向"这个工况**零覆盖** —— 端点法和中点法都判 PASS。
        //   本例让航向边走边转，两种实现会给出相差约 20 mm 的结果，才真正能区分。
        //
        //   参数：每窗口转 8°（dyaw_per_ms=80 × 10）、走 19.995 mm（cnt=6111）
        //         ⇒ 轨迹半径 R = 19.995 / 8° = 143.2 mm，22 窗口共 176°
        //   期望终点（中点法，连续解）：
        //         (R(1−cos176°), R·sin176°) = (143.2×1.99756, 143.2×0.06976)
        //                                   = (286.0, 10.0) mm
        //   ★ 端点法会给出 (286, **−10**) —— 只有 y 差 20 mm，所以 arc_mid_y 是
        //     这条用例的判别项；容差 ±8 mm 既能区分（20 mm），又容得下 256 点查表的
        //     量化残差（实测 y 落在 13，即偏 +3 mm）。
        $display("-- 9. ★中点积分：边走边转 22 窗口（8°/窗口、19.995mm/窗口，共 176°）--");
        $display("     期望 (286, 10)；端点法会给 (286, -10)");
        reset_and_turn(32'sd0);
        repeat (22) one_window(32'sd80, 32'sd6111);
        chk("arc_mid_x", ox, 286, 8);
        chk("arc_mid_y", oy, 10, 8);

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
