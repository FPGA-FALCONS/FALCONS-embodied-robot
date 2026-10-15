//////////////////////////////////////////////////////////////////////////////////
// quad_decoder_tb —— 正交编码器解码测试平台
//
// 覆盖：正转/反转计数与方向 / 4 倍频标度 / 位置清零 /
//       ★ 去毛刺（窄脉冲不计数）/ ★ 非法双位跳变不计数 / INV_DIR
//
// 编码器状态序列（{A,B}）：
//     正转：00 → 10 → 11 → 01 → 00
//     反转：00 → 01 → 11 → 10 → 00
//
// 参数取小值：CLK=1MHz，FILTER_BITS=2 → 需连续 4 拍一致才认可
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module quad_decoder_tb;

    localparam integer CLK_FREQ_HZ = 1_000_000;
    localparam integer FILTER_BITS = 2;        // 4 拍确认
    localparam integer CLK_PERIOD  = 1000;
    localparam integer SETTLE      = 8;        // 每次改电平后的等待拍数（>4）

    reg  clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg         reset_n;
    reg         enc_a, enc_b;
    reg         count_clear;

    wire signed [31:0] count_out;
    wire               dir_out;
    wire               step_out;

    // 方向反相的第二个例化
    wire signed [31:0] count_inv;
    wire               dir_inv;
    wire               step_inv;

    integer errors = 0;
    integer n;

    //---------------------------------------------------------------
    quad_decoder #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .COUNT_BITS  (32),
        .FILTER_BITS (FILTER_BITS),
        .INV_DIR  (0)
    ) dut (
        .i_clock   (clk),
        .i_reset_n (reset_n),
        .i_enc_a   (enc_a),
        .i_enc_b   (enc_b),
        .i_clear   (count_clear),
        .o_count   (count_out),
        .o_dir     (dir_out),
        .o_step    (step_out)
    );

    quad_decoder #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .COUNT_BITS  (32),
        .FILTER_BITS (FILTER_BITS),
        .INV_DIR  (1)
    ) dut_inv (
        .i_clock   (clk),
        .i_reset_n (reset_n),
        .i_enc_a   (enc_a),
        .i_enc_b   (enc_b),
        .i_clear   (count_clear),
        .o_count   (count_inv),
        .o_dir     (dir_inv),
        .o_step    (step_inv)
    );

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

    // 设置 A/B 电平并等待滤波器确认
    task set_ab;
        input a, b;
        begin
            enc_a = a;
            enc_b = b;
            repeat (SETTLE) @(posedge clk);
        end
    endtask

    // 正转一步 = 4 个计数
    task step_fwd;
        begin
            set_ab(1'b1, 1'b0);   // 00→10
            set_ab(1'b1, 1'b1);   // 10→11
            set_ab(1'b0, 1'b1);   // 11→01
            set_ab(1'b0, 1'b0);   // 01→00
        end
    endtask

    // 反转一步 = 4 个计数
    task step_rev;
        begin
            set_ab(1'b0, 1'b1);   // 00→01
            set_ab(1'b1, 1'b1);   // 01→11
            set_ab(1'b1, 1'b0);   // 11→10
            set_ab(1'b0, 1'b0);   // 10→00
        end
    endtask

    //---------------------------------------------------------------
    initial begin
        $display("");
        $display("======== quad_decoder_tb ========");

        reset_n     = 1'b0;
        enc_a       = 1'b0;
        enc_b       = 1'b0;
        count_clear = 1'b0;

        repeat (8) @(posedge clk);
        #1;

        //---- 用例 1：复位后计数为 0 ----
        $display("--- 用例1：复位后 ---");
        n = count_out;
        checki("复位后计数", n, 0);

        reset_n = 1'b1;
        repeat (SETTLE) @(posedge clk);

        //---- 用例 2：正转 1 步 = 4 个计数（4 倍频）----
        $display("--- 用例2：正转 1 步（4 倍频，应为 +4）---");
        step_fwd;
        n = count_out;
        checki("正转一步后计数", n, 4);
        checki("方向位（1=递增）", dir_out, 1);
        checki("反相例化计数", count_inv, -4);

        //---- 用例 3：再正转 2 步 ----
        $display("--- 用例3：再正转 2 步（应为 +12）---");
        step_fwd;
        step_fwd;
        n = count_out;
        checki("累计计数", n, 12);

        //---- 用例 4：反转 1 步 ----
        $display("--- 用例4：反转 1 步（应为 +8）---");
        step_rev;
        n = count_out;
        checki("反转一步后计数", n, 8);
        checki("方向位（0=递减）", dir_out, 0);

        //---- 用例 5：反转 3 步（跨零）----
        $display("--- 用例5：再反转 3 步（应为 -4，跨零）---");
        step_rev;
        step_rev;
        step_rev;
        n = count_out;
        checki("跨零后计数", n, -4);

        //---- 用例 6：位置清零 ----
        $display("--- 用例6：位置清零 ---");
        count_clear = 1'b1;
        @(posedge clk);
        count_clear = 1'b0;
        repeat (SETTLE) @(posedge clk);
        n = count_out;
        checki("清零后计数", n, 0);

        //---- 用例 7：★ 去毛刺（1 拍窄脉冲不得计数）----
        $display("--- 用例7：★ 去毛刺（1 拍毛刺）---");
        set_ab(1'b0, 1'b0);
        n = count_out;
        enc_a = 1'b1;              // 毛刺：A 拉高
        @(posedge clk);
        enc_a = 1'b0;              // 仅 1 拍
        repeat (SETTLE*2) @(posedge clk);
        checki("毛刺后计数（应不变）", count_out, n);

        // 反相也测一次（B 相毛刺）
        enc_b = 1'b1;
        @(posedge clk);
        enc_b = 1'b0;
        repeat (SETTLE*2) @(posedge clk);
        checki("B 相毛刺后计数（应不变）", count_out, n);

        //---- 用例 8：★ 非法双位跳变（00 → 11）不计数 ----
        $display("--- 用例8：★ 非法双位跳变（00→11）---");
        set_ab(1'b0, 1'b0);
        n = count_out;
        enc_a = 1'b1;
        enc_b = 1'b1;              // A、B 同时翻转
        repeat (SETTLE*2) @(posedge clk);
        checki("双位跳变后计数（应不变）", count_out, n);
        set_ab(1'b0, 1'b0);
        checki("跳回 00 后计数（应不变）", count_out, n);

        //---- 用例 9：去毛刺之后的正常计数仍然工作 ----
        $display("--- 用例9：毛刺测试后正常计数仍可用 ---");
        step_fwd;
        checki("再次正转一步", count_out, n + 4);

        //---- 汇总 ----
        $display("");
        if (errors == 0) begin
            $display("PASS: quad_decoder 正交解码测试全部通过");
        end else begin
            $display("FAIL: quad_decoder 测试共 %0d 处错误", errors);
        end
        $display("");
        $finish;
    end

endmodule
