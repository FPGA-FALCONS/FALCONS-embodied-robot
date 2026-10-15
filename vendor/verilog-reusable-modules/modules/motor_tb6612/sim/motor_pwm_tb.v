//////////////////////////////////////////////////////////////////////////////////
// motor_pwm_tb —— 单路电机 PWM 驱动测试平台
//
// 覆盖：复位安全态 / 滑行停止 / 短路刹车 / 正反转 / 占空比精度 /
//       duty=0 视为停止 / INV_DIR
//
// 参数取小值以加速仿真：CLK=1MHz，PWM 周期 2^4 = 16 拍（等效 62.5kHz）
// —— 只影响等待的拍数，不影响逻辑正确性。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module motor_pwm_tb;

    localparam integer CLK_FREQ_HZ = 1_000_000;
    localparam integer PWM_BITS    = 4;
    localparam integer CLK_PERIOD  = 1000;              // 1MHz
    localparam integer PWM_PERIOD  = (1 << PWM_BITS);   // 16 拍

    reg  clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg                    reset_n;
    reg                    enable, dir, brake;
    reg  [PWM_BITS-1:0]    duty;
    wire                   in1, in2, pwm, active;

    // 第二个例化：验证 INV_DIR = 1 时方向语义反转
    wire                   in1_inv, in2_inv, pwm_inv, active_inv;

    integer errors = 0;

    //---------------------------------------------------------------
    // 被测模块
    //---------------------------------------------------------------
    motor_pwm #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_BITS    (PWM_BITS),
        .INV_DIR  (0)
    ) dut (
        .i_clock   (clk),
        .i_reset_n (reset_n),
        .i_enable  (enable),
        .i_dir     (dir),
        .i_brake   (brake),
        .i_duty    (duty),
        .o_in1     (in1),
        .o_in2     (in2),
        .o_pwm     (pwm),
        .o_active  (active)
    );

    motor_pwm #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_BITS    (PWM_BITS),
        .INV_DIR  (1)
    ) dut_inv (
        .i_clock   (clk),
        .i_reset_n (reset_n),
        .i_enable  (enable),
        .i_dir     (dir),
        .i_brake   (brake),
        .i_duty    (duty),
        .o_in1     (in1_inv),
        .o_in2     (in2_inv),
        .o_pwm     (pwm_inv),
        .o_active  (active_inv)
    );

    //---------------------------------------------------------------
    // 检查任务
    //---------------------------------------------------------------
    task checki;
        input [8*48-1:0] label;
        input integer    got;
        input integer    exp;
        begin
            if (got == exp) begin
                $display("OK  : %0s = %0d", label, got);
            end else begin
                $display("FAIL: %0s 期望 %0d，实测 %0d", label, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    //---------------------------------------------------------------
    // 在 cycles 个时钟内统计 o_pwm 为高的拍数
    //   cycles 取 PWM 周期的整数倍时，占空比测量精确无对齐误差
    //---------------------------------------------------------------
    task meas_high;
        input  integer cycles;
        output integer high_cnt;
        integer i;
        begin
            high_cnt = 0;
            for (i = 0; i < cycles; i = i + 1) begin
                @(posedge clk);
                #1;
                if (pwm) high_cnt = high_cnt + 1;
            end
        end
    endtask

    integer high_cnt;
    integer n;

    //---------------------------------------------------------------
    // 主流程
    //---------------------------------------------------------------
    initial begin
        $display("");
        $display("======== motor_pwm_tb ========");

        reset_n = 1'b0;
        enable  = 1'b0;
        dir     = 1'b0;
        brake   = 1'b0;
        duty    = {PWM_BITS{1'b0}};

        //---- 用例 1：复位期间必须安全（滑行、不出 PWM）----
        repeat (4) @(posedge clk);
        #1;
        $display("--- 用例1：复位期间的安全态 ---");
        checki("复位时 IN1", in1, 0);
        checki("复位时 IN2", in2, 0);
        checki("复位时 PWM", pwm, 0);

        reset_n = 1'b1;
        repeat (4) @(posedge clk);
        #1;

        //---- 用例 2：使能关闭 + 滑行 ----
        $display("--- 用例2：enable=0, brake=0 → 滑行停止 ---");
        enable = 1'b0; brake = 1'b0; duty = 4'd8;
        repeat (4) @(posedge clk); #1;
        checki("IN1", in1, 0);
        checki("IN2", in2, 0);
        checki("PWM", pwm, 0);
        checki("active", active, 0);

        //---- 用例 3：使能关闭 + 刹车 ----
        $display("--- 用例3：enable=0, brake=1 → 短路刹车 ---");
        brake = 1'b1;
        repeat (4) @(posedge clk); #1;
        checki("IN1", in1, 1);
        checki("IN2", in2, 1);
        checki("PWM", pwm, 0);

        //---- 用例 4：正转方向 ----
        $display("--- 用例4：正转方向（dir=0）---");
        enable = 1'b1; brake = 1'b0; dir = 1'b0; duty = 4'd8;
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("正转 IN1", in1, 1);
        checki("正转 IN2", in2, 0);
        checki("active", active, 1);

        //---- 用例 5：反转方向 ----
        $display("--- 用例5：反转方向（dir=1）---");
        dir = 1'b1;
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("反转 IN1", in1, 0);
        checki("反转 IN2", in2, 1);

        //---- 用例 6：占空比精度（测 16 个完整周期 = 256 拍）----
        $display("--- 用例6：占空比精度（256 拍内的高电平拍数）---");
        dir = 1'b0;

        duty = 4'd8;  repeat (PWM_PERIOD) @(posedge clk); meas_high(256, high_cnt);
        checki("duty=8,  50.0% 应为 128", high_cnt, 128);

        duty = 4'd4;  repeat (PWM_PERIOD) @(posedge clk); meas_high(256, high_cnt);
        checki("duty=4,  25.0% 应为 64 ", high_cnt, 64);

        duty = 4'd12; repeat (PWM_PERIOD) @(posedge clk); meas_high(256, high_cnt);
        checki("duty=12, 75.0% 应为 192", high_cnt, 192);

        duty = 4'd15; repeat (PWM_PERIOD) @(posedge clk); meas_high(256, high_cnt);
        checki("duty=15, 93.75% 应为 240", high_cnt, 240);

        duty = 4'd1;  repeat (PWM_PERIOD) @(posedge clk); meas_high(256, high_cnt);
        checki("duty=1,  6.25% 应为 16 ", high_cnt, 16);

        //---- 用例 7：duty=0 视为停止（不能停留在"PWM 恒 0 却 IN 有效"）----
        $display("--- 用例7：duty=0 → 停止 ---");
        duty = 4'd0; enable = 1'b1; brake = 1'b0;
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("duty=0 且 brake=0 时 IN1", in1, 0);
        checki("duty=0 且 brake=0 时 IN2", in2, 0);
        checki("duty=0 时 PWM", pwm, 0);
        checki("duty=0 时 active", active, 0);

        brake = 1'b1;
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("duty=0 且 brake=1 时 IN1", in1, 1);
        checki("duty=0 且 brake=1 时 IN2", in2, 1);

        //---- 用例 8：INV_DIR=1 时方向语义反转 ----
        $display("--- 用例8：INV_DIR=1（另一例化）---");
        enable = 1'b1; brake = 1'b0; dir = 1'b0; duty = 4'd8;
        repeat (PWM_PERIOD) @(posedge clk); #1;
        checki("INV_DIR=1, dir=0 时 IN1", in1_inv, 0);
        checki("INV_DIR=1, dir=0 时 IN2", in2_inv, 1);
        checki("两例化的 PWM 应一致", pwm_inv, pwm);

        //---- 汇总 ----
        $display("");
        if (errors == 0) begin
            $display("PASS: motor_pwm 单路驱动测试全部通过");
        end else begin
            $display("FAIL: motor_pwm 测试共 %0d 处错误", errors);
        end
        $display("");
        $finish;
    end

endmodule
