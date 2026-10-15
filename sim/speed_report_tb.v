//////////////////////////////////////////////////////////////////////////////////
// speed_report_tb —— 串口 CSV 上报格式测试（12 列）
//
// 【为什么需要这个 tb】
//   speed_report 的输出是"给人看的 CSV"，格式错了不会让任何 RTL 报错，
//   只会让上位机/Excel 里少一列或多一列，而且极难从波形上定位。
//   列数从 8 扩到 12 时最容易出的错：
//     * 忘记改 S_SEP 里"最后一个字段"的判断 → 列还没发完就换行（少列）
//     * 忘记改 field 位宽 → 字段号被截断，高位字段永远发不出来
//     * 忘记给新增端口接线 → 发出来的是 0 或 x
//   所以这里直接【逐字符比对整行文本】，格式类错误一抓一个准。
//
// 【为什么用 1 MHz 时钟 + 100 kbaud】
//   真实配置是 50 MHz / 115200，一行 42 字符要 3.6 ms 仿真时间；
//   本 tb 只验证"格式与数值"，与时钟频率无关，于是把时钟降到 1 MHz、
//   波特率设成 100 kbaud（位宽 10 拍），一行只要 4.2 µs —— 快 1000 倍。
//
// 【覆盖】
//   1. 12 列齐全（含 11 个逗号），典型值含负号
//   2. 全零 / 负数 / 单位数边界（−1、0）
//   3. 7 位大数（验证数字权重从 10^6 起、没有丢高位）
//   4. 行尾必须是 CR LF（0x0D 0x0A）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module speed_report_tb;

    localparam integer CLK_HZ  = 1_000_000;             // 1 µs 周期
    localparam integer BAUD    = 100_000;               // 位宽 10 拍 = 10 µs
    localparam integer BIT_NS  = (1_000_000_000 / BAUD);// 10000 ns

    reg         clk     = 1'b0;
    reg         rst_n   = 1'b0;
    reg         force_i = 1'b0;

    reg signed [31:0] v_tgt, v_act, v_duty, v_kp, v_ki, v_kd, v_flags, v_err;
    reg signed [31:0] v_steer, v_throttle, v_mode, v_sigok;

    wire        uart_tx;

    integer errors = 0;

    speed_report #(
        .CLK_FREQ_HZ     (CLK_HZ),
        .BAUD_RATE       (BAUD),
        .REPORT_DIV      (10_000_000),      // 周期上报放到很慢（10 s），只靠 i_force 触发
        .FIFO_DEPTH_LOG2 (7)                // 与顶层一致
    ) dut (
        .i_clock    (clk),
        .i_reset_n  (rst_n),
        .i_tgt      (v_tgt),
        .i_act      (v_act),
        .i_duty     (v_duty),
        .i_kp       (v_kp),
        .i_ki       (v_ki),
        .i_kd       (v_kd),
        .i_flags    (v_flags),
        .i_err      (v_err),
        .i_steer    (v_steer),
        .i_throttle (v_throttle),
        .i_mode     (v_mode),
        .i_sig_ok   (v_sigok),
        .i_force    (force_i),
        .o_uart_tx  (uart_tx)
    );

    always #500 clk = ~clk;                 // 周期 1 µs

    // ------------------------------------------------------------------
    // UART 接收进程：收一个字符 = 起始位 + 8 数据位（LSB 先）+ 停止位
    //   ★ 不要在采样过程中再去等 negedge：字符数据位里也有 1→0 的下降沿
    //     （例如 ',' = 0x2C，位3→位4 就是下降沿），边采边等沿必然错位。
    //     本实现"进 forever → 等起始沿 → 连续采 8 位 → 回到起点"，
    //     采样窗口内不响应任何边沿，因此不会错位。
    // ------------------------------------------------------------------
    reg [7:0] rxbuf [0:255];
    integer   rxlen     = 0;
    integer   line_cnt  = 0;
    reg       line_done = 1'b0;

    integer   bi;
    reg [7:0] rb;

    initial begin : uart_rx_proc
        forever begin
            @(negedge uart_tx);                     // 起始位下降沿
            #(BIT_NS + BIT_NS / 2);                 // 对齐到第 0 位中心
            for (bi = 0; bi < 8; bi = bi + 1) begin
                rb[bi] = uart_tx;
                #(BIT_NS);
            end
            // 此刻位于停止位区间内（起始沿 + 9.5 位宽）
            if (rb == 8'h0D) begin
                // CR：忽略
            end else if (rb == 8'h0A) begin
                line_cnt  = line_cnt + 1;           // LF：一行结束
                line_done = 1'b1;
            end else begin
                if (rxlen < 256) begin
                    rxbuf[rxlen] = rb;
                    rxlen        = rxlen + 1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 逐字符比对整行
    //   exp 是右对齐的字符串字面量 → 第 k 个字符在 exp[8*(explen-1-k) +: 8]
    //   长度【自动求】：从最高字节往下找第一个非 0 字节。硬编码长度太容易数错
    //   （本项目已经因为手数 12 个字段的字符数错过两次）。
    // ------------------------------------------------------------------
    task expect_line;
        input [8*64-1:0] exp;
        input [8*40-1:0] name;
        integer k;
        integer bad;
        integer explen;
        begin
            explen = 0;
            for (k = 63; k >= 0; k = k - 1) begin
                if (exp[8*k +: 8] != 8'h00) begin
                    explen = k + 1;
                    k      = -1;                // 跳出循环
                end
            end

            bad = 0;
            if (rxlen != explen) begin
                bad = 1;
                $display("  FAIL  %0s: 收到 %0d 字符，期望 %0d", name, rxlen, explen);
            end else begin
                for (k = 0; k < explen; k = k + 1) begin
                    if (rxbuf[k] !== exp[8*(explen-1-k) +: 8]) begin
                        bad = 1;
                        $display("  FAIL  %0s: 第 %0d 字符 got 0x%02x exp 0x%02x",
                                 name, k, rxbuf[k], exp[8*(explen-1-k) +: 8]);
                    end
                end
            end
            if (bad == 0) begin
                $display("  pass  %0s (%0d 字符)", name, rxlen);
            end else begin
                errors = errors + 1;
                $write("        实收: ");
                for (k = 0; k < rxlen; k = k + 1) $write("%c", rxbuf[k]);
                $write("\n");
            end
        end
    endtask

    // ------------------------------------------------------------------
    // 等 UART 发送线空闲
    //   判据：连续 300 拍为高。一个字符内最长连续高电平 < 10 拍（位宽 10 拍），
    //   所以连续 300 拍空闲一定意味着"没有字符正在发送"。
    // ------------------------------------------------------------------
    task wait_tx_idle;
        integer n;
        begin
            n = 0;
            while (n < 300) begin
                @(posedge clk);
                if (uart_tx) n = n + 1;
                else         n = 0;
            end
        end
    endtask

    // ------------------------------------------------------------------
    // 触发一行并等它收完
    //   ★ i_force 在 S_IDLE 里是【或】条件：若保持高电平，发完一行回到 S_IDLE
    //     后会【再发一行】。最初写成"拉高 → 等 line_done → 拉低"，每次都会
    //     多留 1 行残留，下一次检查读到的就是上一次触发的那一行 —— 表现为整体
    //     滞后一行，且每行长度都对不上（实测 39 / 42 / 42 / 42）。
    //     所以这里只给两个时钟的脉冲，确保只发这一行。
    // ------------------------------------------------------------------
    task send_one;
        begin
            rxlen     = 0;
            line_done = 1'b0;
            force_i   = 1'b1;
            @(posedge clk);
            @(posedge clk);
            force_i   = 1'b0;                       // 立刻拉低 → 只发这一行
            wait (line_done == 1'b1);
            repeat (4) @(posedge clk);
        end
    endtask

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    initial begin
        $display("======== speed_report 串口 CSV 上报测试（12 列）========");

        v_tgt = 0; v_act = 0; v_duty = 0; v_kp = 0; v_ki = 0; v_kd = 0;
        v_flags = 0; v_err = 0; v_steer = 0; v_throttle = 0; v_mode = 0; v_sigok = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ---------------- 0. 丢弃复位后 DUT 自发的一行 ----------------
        //   speed_report 的 tick_cnt 复位为 0，而 report_tick = (tick_cnt == 0)
        //   且 state 复位后就是 S_IDLE，所以【复位一释放就自发上报一行】。
        //   这是 RTL 的既定行为（好处：上位机一上电就能看到数据），保留不动；
        //   但本 tb 必须先把这一行吃掉、把接收缓冲清干净，否则后面每次检查都会
        //   读到上一次的行（整体滞后一行）。
        $display("-- 0. 丢弃复位后 DUT 自发的一行（tick_cnt 复位值 0 所致，属正常行为）--");
        wait_tx_idle;                       // 等它发完
        rxlen     = 0;                      // 丢弃已收到的内容
        line_done = 1'b0;
        line_cnt  = 0;                      // 行计数也从这里重新起算
        repeat (4) @(posedge clk);

        // ---------------- 1. 典型一行 ----------------
        $display("-- 1. 典型值：12 列 / 11 个逗号 / 负数带减号 --");
        v_tgt = 1234;  v_act = -56;   v_duty = 2047; v_kp = 256;
        v_ki  = 8;     v_kd  = 0;     v_flags = 1;   v_err = 1290;
        v_steer = -1000; v_throttle = 500; v_mode = 1; v_sigok = 1;
        send_one;
        expect_line("1234,-56,2047,256,8,0,1,1290,-1000,500,1,1", "row_typical");

        // ---------------- 2. 单位数与负数边界 ----------------
        $display("-- 2. 边界：-1 / 0（应输出 1 位，不留前导零）--");
        v_tgt = -1; v_act = 0; v_duty = 0; v_kp = 0; v_ki = 0; v_kd = 0;
        v_flags = 0; v_err = -1; v_steer = 0; v_throttle = -1000;
        v_mode = 0; v_sigok = 0;
        send_one;
        expect_line("-1,0,0,0,0,0,0,-1,0,-1000,0,0", "row_boundary");

        // ---------------- 3. 7 位大数 ----------------
        $display("-- 3. 大数：9999999（数字权重最高位为 10^6，共 7 位）--");
        v_tgt = 9999999; v_act = 7; v_duty = 7; v_kp = 7; v_ki = 7; v_kd = 7;
        v_flags = 7; v_err = 7; v_steer = 7; v_throttle = 7; v_mode = 7; v_sigok = 7;
        send_one;
        expect_line("9999999,7,7,7,7,7,7,7,7,7,7,7", "row_bignum");

        // ---------------- 4. 负的 7 位大数 ----------------
        $display("-- 4. 负数大数：-9999999 --");
        v_tgt = -9999999; v_act = -7; v_duty = -7; v_kp = -7; v_ki = -7; v_kd = -7;
        v_flags = -7; v_err = -7; v_steer = -7; v_throttle = -7; v_mode = -7; v_sigok = -7;
        send_one;
        expect_line("-9999999,-7,-7,-7,-7,-7,-7,-7,-7,-7,-7,-7", "row_bignum_neg");

        // ---------------- 汇总 ----------------
        $display("======== 结果 ========");
        $display("  共收到 %0d 行（应为 4）", line_cnt);
        if (line_cnt != 4) begin
            errors = errors + 1;
            $display("  FAIL  line_count: %0d", line_cnt);
        end
        if (errors == 0) $display("PASS: speed_report 全部检查通过");
        else             $display("FAIL: %0d 项检查未通过", errors);
        $finish;
    end

    // 超时保护（4 行 × 47 字符 × 100 µs ≈ 19 ms 仿真时间）
    initial begin
        #500_000_000;                       // 500 ms
        $display("FAIL: 仿真超时（500 ms），line_done=%b rxlen=%0d", line_done, rxlen);
        $finish;
    end

endmodule
