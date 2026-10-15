//////////////////////////////////////////////////////////////////////////////////
// speed_measure_tb —— 定窗口测速测试平台
//
// 覆盖：窗口长度 / 首个窗口 / 正增量 / 负增量 / ★ 清零时基准复位（不留假增量）
//
// 参数取小值：CLK=1MHz，WINDOW_MS=1 → 窗口 1000 拍（1ms），仿真很快
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module speed_measure_tb;

    localparam integer CLK_FREQ_HZ  = 1_000_000;
    localparam integer WINDOW_MS    = 1;
    localparam integer COUNT_BITS   = 32;
    localparam integer CLK_PERIOD   = 1000;
    localparam integer WINDOW_TICKS = (CLK_FREQ_HZ / 1000) * WINDOW_MS;   // 1000

    reg  clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg                    reset_n;
    reg  signed [31:0]     count_in;
    reg                    clear_in;

    wire signed [31:0]     delta_out;
    wire                   delta_valid;

    integer errors = 0;
    integer n;

    // 自由计数器：用于测量窗口长度
    reg [31:0] tick_cnt = 32'd0;
    always @(posedge clk) if (reset_n) tick_cnt = tick_cnt + 32'd1;

    integer tick_a, tick_b;

    //---------------------------------------------------------------
    speed_measure #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .WINDOW_MS   (WINDOW_MS),
        .COUNT_BITS  (COUNT_BITS)
    ) dut (
        .i_clock       (clk),
        .i_reset_n     (reset_n),
        .i_count       (count_in),
        .i_clear       (clear_in),
        .o_delta       (delta_out),
        .o_delta_valid (delta_valid)
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

    // 等待下一个窗口结束脉冲
    task wait_valid;
        begin
            @(posedge clk); #1;
            while (delta_valid !== 1'b1) begin
                @(posedge clk); #1;
            end
        end
    endtask

    //---------------------------------------------------------------
    initial begin
        $display("");
        $display("======== speed_measure_tb ========");

        reset_n  = 1'b0;
        count_in = 32'sd0;
        clear_in = 1'b0;

        repeat (8) @(posedge clk);
        #1;
        $display("--- 用例1：复位后 ---");
        n = delta_out;
        checki("复位后增量", n, 0);
        checki("复位后 valid", delta_valid, 0);

        reset_n = 1'b1;

        //---- 用例 2：窗口长度应为 1000 拍 ----
        $display("--- 用例2：窗口长度 ---");
        wait_valid;
        tick_a = tick_cnt;
        wait_valid;
        tick_b = tick_cnt;
        checki("窗口长度(拍)", tick_b - tick_a, WINDOW_TICKS);
        checki("首窗口增量(计数未变)", delta_out, 0);

        //---- 用例 3：正增量 ----
        $display("--- 用例3：正增量 ---");
        count_in = 32'sd100;
        wait_valid;
        checki("计数 0→100", delta_out, 100);

        //---- 用例 4：再正增量 ----
        $display("--- 用例4：再正增量 ---");
        count_in = 32'sd250;
        wait_valid;
        checki("计数 100→250", delta_out, 150);

        //---- 用例 5：负增量（反向转动）----
        $display("--- 用例5：负增量 ---");
        count_in = 32'sd200;
        wait_valid;
        checki("计数 250→200", delta_out, -50);

        //---- 用例 6：计数不变时为 0 ----
        $display("--- 用例6：计数不变 ---");
        wait_valid;
        checki("计数保持不变", delta_out, 0);

        //---- 用例 7：★ 清零（基准必须同步复位，否则会冒出一个巨大假增量）----
        $display("--- 用例7：★ 清零时的基准复位 ---");
        count_in = 32'sd5000;
        repeat (50) @(posedge clk);          // 让计数先跑起来（窗口未结束）
        count_in = 32'sd0;                   // 与 quad_decoder 同步清零
        clear_in = 1'b1;
        @(posedge clk);
        clear_in = 1'b0;
        wait_valid;
        checki("清零后首个窗口增量（应为 0，而不是 5000）", delta_out, 0);

        //---- 用例 8：清零后恢复正常计数 ----
        $display("--- 用例8：清零后恢复正常 ---");
        count_in = 32'sd42;
        wait_valid;
        checki("计数 0→42", delta_out, 42);

        //---- 汇总 ----
        $display("");
        if (errors == 0) begin
            $display("PASS: speed_measure 定窗口测速测试全部通过");
        end else begin
            $display("FAIL: speed_measure 测试共 %0d 处错误", errors);
        end
        $display("");
        $finish;
    end

endmodule
