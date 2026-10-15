`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// yaw_integrate 测试平台
//
// 验证内容：
//   1. 复位后冻结（i_enable=0）时 yaw 恒为 0
//   2. 静止时 yaw 不漂（gz=0）
//   3. 恒速转动角度正确：90°/s 转 1 秒 → 90.00°（分 4 段累计验证线性）
//   4. 回绕正确：转满 +180° 折回 -180°，继续反向转 -270° 等价于 +90°
//   5. 静止死区：|gz| < 阈值 归零；|gz| == 阈值 不归零（含负向）
//   6. 样本锁存（ZOH）：只发一个采样脉冲后 i_gyro_z 变化不影响积分
//   7. i_clear 清零、i_enable 冻结/解冻从 0 重新积分
//
// 【关于参数】CLK_FREQ_HZ 只用于计算分频系数，与仿真时钟周期无关。
//   为加速仿真取 CLK_FREQ_HZ = 2^18、INT_RATE_HZ = 1024，分频系数恰为整数
//   256，故仿真中积分节拍严格精确，与真实 50MHz/1024Hz 的比值一致。
//   节拍数用 dut.int_tick 计数，因此所有期望值都是节拍数的精确倍数，无舍入。
//////////////////////////////////////////////////////////////////////////////////

module yaw_integrate_tb;

    reg  clk;
    reg  rst_n;

    reg  signed [31:0] gz;
    reg                sample_valid;
    reg                enable;
    reg                clear;

    wire signed [31:0] yaw;
    wire               yaw_valid;
    wire               dz_hit;
    wire signed [31:0] gz_eff;

    integer errors;

    yaw_integrate #(
        .CLK_FREQ_HZ (262_144),         // 2^18
        .INT_RATE_HZ (1024),
        .DEADZONE_EN (1),
        .DEADZONE_TH (30)               // 0.3°/s
    ) dut (
        .i_clock        (clk),
        .i_reset_n      (rst_n),
        .i_gyro_z       (gz),
        .i_sample_valid (sample_valid),
        .i_enable       (enable),
        .i_clear        (clear),
        .o_yaw          (yaw),
        .o_yaw_valid    (yaw_valid),
        .o_deadzone_hit (dz_hit),
        .o_gz_eff       (gz_eff)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;               // 10ns 周期（仅作仿真节拍）

    //---------------------------------------------------------------
    // 激励辅助
    //---------------------------------------------------------------
    // 让 i_gyro_z 持续送同一值（采样保持常高，gz_now 直通）
    task hold_gz;
        input signed [31:0] v;
        begin
            @(negedge clk);
            gz           = v;
            sample_valid = 1'b1;
        end
    endtask

    // 单个采样脉冲后撤掉 i_sample_valid，用于验证 ZOH 锁存
    task pulse_gz;
        input signed [31:0] v;
        begin
            @(negedge clk);
            gz           = v;
            sample_valid = 1'b1;
            @(negedge clk);
            sample_valid = 1'b0;
        end
    endtask

    // 清零脉冲。返回时位于"clear 被采样的那个时钟沿之后、下一个时钟沿之前"，
    // 因此返回瞬间 acc 必然已是 0，可以立刻比较。
    task do_clear;
        begin
            @(negedge clk);
            clear = 1'b1;
            @(negedge clk);
            clear = 1'b0;
        end
    endtask

    // 等待 n 个积分节拍
    //   int_tick 由 tick_cnt 变 0 组合产生，其上升沿比"本次积分写入 acc"早一拍，
    //   故必须再等一个时钟沿让 acc 更新，否则每次测量都会少算一个节拍
    //   （表现为期望 2250 实测 2241，差值恰为一个节拍的增量 9000>>10）。
    task wait_ticks;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(posedge dut.int_tick);    // 第 k+1 个积分节拍到来
                @(posedge clk);             // 该节拍的积分写入 acc
            end
            @(negedge clk);                 // 避开时钟沿竞争，读稳定值
        end
    endtask

    task check_yaw;
        input signed [31:0] expect;
        input [255:0]       name;
        begin
            if (yaw !== expect) begin
                $display("  FAIL: %0s 期望 %0d，实际 %0d", name, $signed(expect), $signed(yaw));
                errors = errors + 1;
            end
        end
    endtask

    task check_bit;
        input        val;
        input [255:0] name;
        begin
            if (val !== 1'b1) begin
                $display("  FAIL: %0s 应为 1，实际 %b", name, val);
                errors = errors + 1;
            end
        end
    endtask

    //---------------------------------------------------------------
    // 主流程
    //---------------------------------------------------------------
    initial begin
        errors       = 0;
        clk          = 1'b0;
        rst_n        = 1'b0;
        gz           = 32'sd0;
        sample_valid = 1'b0;
        enable       = 1'b0;
        clear        = 1'b0;

        repeat (20) @(posedge clk);
        rst_n = 1'b1;

        //===========================================================
        $display("---- 用例1：冻结（i_enable=0）时 yaw 恒为 0 ----");
        hold_gz(32'sd9000);                 // 90°/s
        wait_ticks(100);
        check_yaw(32'sd0, "冻结时 yaw");
        check_bit(~yaw_valid, "冻结时 o_yaw_valid 应为 0");

        //===========================================================
        $display("---- 用例2：静止（gz=0）时 yaw 不漂 ----");
        enable = 1'b1;
        hold_gz(32'sd0);
        do_clear();
        wait_ticks(256);
        check_yaw(32'sd0, "静止 256 节拍");

        //===========================================================
        $display("---- 用例3：恒速 90°/s 积分（分 4 段累计到 90.00°）----");
        hold_gz(32'sd9000);                 // 9000 × 0.01°/s = 90°/s
        do_clear();
        wait_ticks(256);                    // 256/1024 s = 0.25s → 22.5°
        check_yaw(32'sd2250, "0.25s 累计（22.50°）");
        wait_ticks(256);                    // 0.5s → 45°
        check_yaw(32'sd4500, "0.50s 累计（45.00°）");
        wait_ticks(256);                    // 0.75s → 67.5°
        check_yaw(32'sd6750, "0.75s 累计（67.50°）");
        wait_ticks(256);                    // 1.0s → 90°
        check_yaw(32'sd9000, "1.00s 累计（90.00°）");

        //===========================================================
        $display("---- 用例4：回绕，转满 +180° 折回 -180° ----");
        wait_ticks(1024);                   // 再转 1s → 累计 180°
        check_yaw(-32'sd18000, "转到 +180° 处应回绕为 -180.00°");

        //===========================================================
        $display("---- 用例5：反向转动跨越回绕边界 ----");
        hold_gz(-32'sd9000);                // -90°/s
        wait_ticks(1024);                   // -180° - 90° = -270° ≡ +90°
        check_yaw(32'sd9000, "-270° 应等价于 +90.00°");

        //===========================================================
        $display("---- 用例6：静止死区（|gz| < 0.3°/s 归零）----");
        hold_gz(32'sd20);                   // 0.2°/s，低于阈值
        do_clear();
        wait_ticks(256);
        check_yaw(32'sd0, "0.2°/s 应被死区吞掉");
        check_bit(dz_hit, "0.2°/s 时 o_deadzone_hit");

        //===========================================================
        $display("---- 用例7：死区边界（|gz| == 阈值 不归零）----");
        hold_gz(32'sd30);                   // 恰为 0.3°/s
        do_clear();
        wait_ticks(256);
        check_yaw(32'sd7, "0.3°/s 不应被死区吞掉（30×256>>10 = 7）");

        //===========================================================
        $display("---- 用例8：负向死区与负向越过阈值 ----");
        hold_gz(-32'sd20);                  // -0.2°/s，低于阈值
        do_clear();
        wait_ticks(256);
        check_yaw(32'sd0, "-0.2°/s 应被死区吞掉");

        hold_gz(-32'sd40);                  // -0.4°/s，越过阈值
        do_clear();
        wait_ticks(256);
        check_yaw(-32'sd10, "-0.4°/s 应正常积分（-10）");

        //===========================================================
        $display("---- 用例9：样本锁存（ZOH），脉冲后改输入不影响积分 ----");
        pulse_gz(32'sd4096);                // 锁存 4096
        gz = 32'sd0;                        // 输入改为 0 但不再发脉冲
        sample_valid = 1'b0;
        do_clear();
        wait_ticks(256);
        check_yaw(32'sd1024, "应按锁存的 4096 积分（4096×256>>10 = 1024）");

        //===========================================================
        $display("---- 用例10：i_clear 清零 ----");
        hold_gz(32'sd9000);
        do_clear();
        wait_ticks(256);
        check_yaw(32'sd2250, "清零前应有值");
        do_clear();
        check_yaw(32'sd0, "i_clear 后应立即为 0");

        //===========================================================
        $display("---- 用例11：冻结后解冻从 0 重新积分 ----");
        enable = 1'b0;
        hold_gz(32'sd9000);
        wait_ticks(100);
        check_yaw(32'sd0, "冻结期间 yaw 应为 0");
        enable = 1'b1;
        wait_ticks(256);
        check_yaw(32'sd2250, "解冻后 0.25s 应为 22.50°");

        //===========================================================
        if (errors == 0) begin
            $display("PASS: yaw_integrate 积分、死区、回绕、清零、冻结全部测试通过");
        end else begin
            $display("FAIL: 共 %0d 个错误", errors);
        end
        $finish;
    end

    // 总超时保护
    initial begin
        #50_000_000;
        $display("FAIL: 仿真总超时");
        $finish;
    end

endmodule
