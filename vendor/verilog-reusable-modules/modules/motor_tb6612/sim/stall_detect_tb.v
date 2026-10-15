//////////////////////////////////////////////////////////////////////////////////
// stall_detect_tb —— 堵转检测与保护测试平台
//
// ★ 本测试的核心是验证判据的【分辨能力】：
//     堵转振动（+167/-167 来回）  → 必须判为堵转
//     低速转动（每窗口 +100 单向）→ 必须【不】误报
//   这两个场景的单窗口增量幅值是同一量级（167 vs 100），正是"单窗口阈值
//   判据做不到、净位移判据才做得到"的地方。
//
// 参数取小值以加速仿真：宽限期 4 窗口、判定 5 窗口、净位移阈值 300 计数
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module stall_detect_tb;

    localparam integer STARTUP_WINDOWS = 4;
    localparam integer STALL_WINDOWS   = 5;
    localparam integer DUTY_TH         = 200;
    localparam integer MOVE_TH         = 300;

    localparam integer CLK_PERIOD = 1000;      // 1MHz

    reg  clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg                reset_n;
    reg                enable;
    reg  [15:0]        duty;
    reg  signed [31:0] delta_in;
    reg                valid_in;
    reg                stall_clear;

    wire               stall;
    wire               enable_out;

    integer errors = 0;
    integer n;

    //---------------------------------------------------------------
    stall_detect #(
        .STARTUP_WINDOWS (STARTUP_WINDOWS),
        .STALL_WINDOWS   (STALL_WINDOWS),
        .DUTY_TH         (DUTY_TH),
        .MOVE_TH         (MOVE_TH)
    ) dut (
        .i_clock       (clk),
        .i_reset_n     (reset_n),
        .i_enable      (enable),
        .i_duty        (duty),
        .i_delta       (delta_in),
        .i_delta_valid (valid_in),
        .i_stall_clear (stall_clear),
        .o_stall       (stall),
        .o_enable_out  (enable_out)
    );

    //---------------------------------------------------------------
    task checki;
        input [8*52-1:0] label;
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

    // 推进一个测速窗口，本窗口的增量为 d
    task win;
        input signed [31:0] d;
        begin
            delta_in = d;
            valid_in = 1'b1;
            @(posedge clk);
            valid_in = 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    // 连续 n 个窗口给同样的增量
    task win_n;
        input signed [31:0] d;
        input integer      n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) win(d);
        end
    endtask

    // 清除堵转
    task do_clear;
        begin
            stall_clear = 1'b1;
            @(posedge clk);
            stall_clear = 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    // 复位整块
    task do_reset;
        begin
            reset_n     = 1'b0;
            enable      = 1'b0;
            duty        = 16'd0;
            delta_in    = 32'sd0;
            valid_in    = 1'b0;
            stall_clear = 1'b0;
            repeat (8) @(posedge clk);
            reset_n = 1'b1;
            repeat (4) @(posedge clk);
        end
    endtask

    //---------------------------------------------------------------
    initial begin
        $display("");
        $display("======== stall_detect_tb ========");
        $display("（宽限期 %0d 窗口，判定 %0d 窗口，净位移阈值 %0d 计数）",
                 STARTUP_WINDOWS, STALL_WINDOWS, MOVE_TH);

        do_reset;

        //---- 用例 1：复位后 ----
        $display("--- 用例1：复位后 ---");
        checki("复位后 stall", stall, 0);
        checki("复位后 enable_out", enable_out, 0);

        //---- 用例 2：使能但占空比低于阈值 → 不检测 ----
        $display("--- 用例2：duty 低于阈值，不判定（低速/停止不算堵转）---");
        enable = 1'b1;
        duty   = DUTY_TH - 1;
        win_n(32'sd0, 20);
        checki("低占空比下 stall", stall, 0);
        checki("低占空比下 enable_out", enable_out, 1);   // 使能照常放行

        //---- 用例 3：★ 正常转动不得误报 ----
        $display("--- 用例3：★ 正常转动（每窗口 +100）不得误报 ---");
        do_clear;
        duty = 16'd512;
        win_n(32'sd100, 30);          // 3000 计数，远大于阈值
        checki("正常转动后 stall", stall, 0);
        checki("正常转动后 enable_out", enable_out, 1);

        //---- 用例 4：★ 堵转（完全静止）----
        $display("--- 用例4：★ 堵转 —— 完全静止（每窗口 0）---");
        do_clear;
        win_n(32'sd0, STARTUP_WINDOWS + STALL_WINDOWS + 4);
        checki("静止堵转后 stall", stall, 1);
        checki("堵转后 enable_out（应被切断）", enable_out, 0);

        //---- 用例 5：锁存 —— 恢复转动也不解除 ----
        $display("--- 用例5：锁存 —— 之后恢复转动也不解除 ---");
        win_n(32'sd500, 20);
        checki("恢复转动后 stall（应仍锁存）", stall, 1);
        checki("锁存期间 enable_out", enable_out, 0);

        //---- 用例 6：清除后恢复 ----
        $display("--- 用例6：i_stall_clear 清除锁存 ---");
        do_clear;
        checki("清除后 stall", stall, 0);
        checki("清除后 enable_out", enable_out, 1);

        //---- 用例 7：★ 堵转（±1° 振动）—— 核心用例 ----
        //   60000 计数/转时 1° = 167 计数。振动的单窗口增量与低速转动同量级，
        //   但净位移来回抵消，所以必须仍被判为堵转。
        $display("--- 用例7：★ 堵转 —— ±1° 振动（+167/-167 交替）---");
        do_clear;
        win_n(32'sd0, STARTUP_WINDOWS);          // 先过宽限期
        repeat (12) begin                        // 交替振动
            win(32'sd167);
            win(-32'sd167);
        end
        checki("振动堵转后 stall（核心：必须为 1）", stall, 1);
        checki("振动堵转后 enable_out", enable_out, 0);

        //---- 用例 8：★ 极低速转动（每窗口 +20）—— 会被判为堵转（设计意图）----
        //   +20 计数/窗口 = 2000 计数/s = 2 转/分钟，这种"转速"对小车等同不动
        $display("--- 用例8：极低速（每窗口 +20）按设计会被判堵转 ---");
        do_clear;
        win_n(32'sd20, STARTUP_WINDOWS + STALL_WINDOWS + 4);
        checki("极低速下 stall（设计如此）", stall, 1);

        //---- 用例 9：中途关使能 → 回到初始态，不残留堵转 ----
        $display("--- 用例9：中途关使能，回到初始态 ---");
        do_clear;
        enable = 1'b0;
        win_n(32'sd0, 20);
        checki("关使能后 stall", stall, 0);
        checki("关使能后 enable_out", enable_out, 0);

        //---- 用例 10：重新使能后能正常检测 ----
        $display("--- 用例10：重新使能后检测功能仍正常 ---");
        enable = 1'b1;
        duty   = 16'd512;
        win_n(32'sd0, STARTUP_WINDOWS + STALL_WINDOWS + 4);
        checki("重新使能后静止堵转", stall, 1);

        //---- 汇总 ----
        $display("");
        if (errors == 0) begin
            $display("PASS: stall_detect 堵转检测测试全部通过");
        end else begin
            $display("FAIL: stall_detect 测试共 %0d 处错误", errors);
        end
        $display("");
        $finish;
    end

endmodule
