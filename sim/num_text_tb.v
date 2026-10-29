//////////////////////////////////////////////////////////////////////////////////
// num_text_tb —— 实时数值格式化单元测试
//
// ★ 2026-10-18：字段宽 4 → 5（"符号 + 千位 + 百位 + 十位 + 个位"）。
//   渲染端两类消费者共用这份 5 格结果：
//     4 字符字段（航向角/四轮转速）→ 从 base+1 读 4 格
//     5 字符字段（角速度/中心速度/航位）→ 从 base+0 读 5 格
//   所以本台断言的是**完整 5 格**，地址 = 字段号 × 5。
//
// 字库索引 = ASCII − 32 → 空格=0  '-'=13  '0'=16 …
//   前导零抑制按"高位是否已开始"：400 → "  400"（十位的 0 必须显示），
//   全零只留个位：0 → "    0"。
//   ★ 这一条是 2026-10-18 预览图抓出的 bug：第一版按"该位是否为 0"判断，
//     结果 400 显示成 " 4 0"、-200 显示成 "-2 0"。下面是它的回归用例。
//   ★ 下面给的值只用于验证【格式化】，不代表真实字段语义（真实映射见 num_text.v 头注释）。
//
// 【时钟与刷新周期】
//   REFRESH_DIV = (CLK_HZ/1000) × REFRESH_MS，这里取 10 MHz + 1 ms → 1 万拍一轮，
//   仿真很快；逻辑与 50 MHz/100 ms 完全一致。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module num_text_tb;

    localparam integer CH_SPACE = 0;
    localparam integer CH_MINUS = 13;
    localparam integer CH_0     = 16;      // '0'

    localparam integer FW = 5;             // 每字段字符数（与 RTL 默认一致）

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    reg signed [31:0] v0, v1, v2, v3, v4, v5, v6, v7, v8;
    reg signed [31:0] v9 = 32'sd0, v10 = 32'sd0, v11 = 32'sd0;
    reg signed [31:0] v12 = 4, v13 = 180;
    // ★ 2026-10-20：哪些字段带小数。默认 = 数据页配置（字段 6..10 带小数）。
    reg [15:0] frac_mask = 14'b00_0111_1100_0000;

    wire        o_we;
    wire [6:0]  o_addr;
    wire [7:0]  o_data;

    integer errors = 0;
    integer i;

    // 抓取写入的字符（64 深，避免 o_addr 越界）
    reg [7:0] cap [0:127];

    num_text #(
        .CLK_HZ     (10_000_000),
        .REFRESH_MS (1),
        .FIELD_W    (FW),
        // ★ 2026-10-20：9 → 12，覆盖新增的字段 11（参数整定页速度环 Kd）
        .NUM_FIELDS (16)
    ) dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_v0   (v0), .i_v1 (v1), .i_v2 (v2),
        .i_v3   (v3), .i_v4 (v4), .i_v5 (v5), .i_v6 (v6),
        .i_v7   (v7), .i_v8 (v8), .i_v9 (v9), .i_v10 (v10), .i_v11 (v11),
        .i_v12 (v12), .i_v13 (v13), .i_v14(32'sd128), .i_v15(32'sd30),
        .i_frac_mask (frac_mask),
        .o_we   (o_we),
        .o_addr (o_addr),
        .o_data (o_data)
    );

    always #50 clk = ~clk;                  // 10 MHz

    // 记录所有写入
    always @(posedge clk) begin
        if (o_we) cap[o_addr] = o_data;
    end

    task check_ch;
        input [8*32-1:0] name;
        input integer    got;
        input integer    exp;
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL  %0s: got idx %0d, expect %0d", name, got, exp);
            end else begin
                $display("  pass  %0s = idx %0d", name, got);
            end
        end
    endtask

    // 等一轮完整刷新（1 ms = 10000 拍）
    task wait_refresh;
        begin repeat (12000) @(posedge clk); end
    endtask

    initial begin
        $display("======== num_text 单元测试（实时数值格式化，5 字符/字段）========");

        for (i = 0; i < 128; i = i + 1) cap[i] = 8'hFF;   // 用 FF 标记"未写入"

        v0 = 32'sd300;      // → "  300"
        v1 = -32'sd45;      // → "  -45"（负号贴最高位左侧一格）
        v2 = 32'sd0;        // → "    0"
        v3 = 32'sd999;      // → "  999"
        v4 = 32'sd12;       // → "   12"
        v5 = -32'sd7;       // → "   -7"
        v6 = -32'sd180;     // → " -180"（3 位数字）
        v7 = -32'sd260;     // → " -260"
        v8 = 32'sd12345;    // → " 9999"（**饱和**测试：>9999 时不许溢出成乱码）

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        wait_refresh;

        $display("-- 字段 0：v0=300 → 空格 空格 '3' '0' '0' --");
        check_ch("f0[0]", cap[0*FW+0], CH_SPACE);
        check_ch("f0[1]", cap[0*FW+1], CH_SPACE);
        check_ch("f0[2]", cap[0*FW+2], CH_0 + 3);
        check_ch("f0[3]", cap[0*FW+3], CH_0 + 0);
        check_ch("f0[4]", cap[0*FW+4], CH_0 + 0);

        $display("-- 字段 1：v1=-45 → 空格 空格 '-' '4' '5'（负号贴最高位）--");
        check_ch("f1[0]", cap[1*FW+0], CH_SPACE);
        check_ch("f1[1]", cap[1*FW+1], CH_SPACE);
        check_ch("f1[2]", cap[1*FW+2], CH_MINUS);
        check_ch("f1[3]", cap[1*FW+3], CH_0 + 4);
        check_ch("f1[4]", cap[1*FW+4], CH_0 + 5);

        $display("-- 字段 2：v2=0 → 空格 x4 + '0' --");
        check_ch("f2[0]", cap[2*FW+0], CH_SPACE);
        check_ch("f2[1]", cap[2*FW+1], CH_SPACE);
        check_ch("f2[2]", cap[2*FW+2], CH_SPACE);
        check_ch("f2[3]", cap[2*FW+3], CH_SPACE);
        check_ch("f2[4]", cap[2*FW+4], CH_0);

        $display("-- 字段 3：v3=999 → 空格 空格 '9' '9' '9' --");
        //   注意每字段布局是 [符号][千][百][十][个]：v3 为正 → 符号位与千位是空格，
        //   百位才是第一个 '9'（下面 f3[2] 断言的正是百位）。
        check_ch("f3[0]", cap[3*FW+0], CH_SPACE);
        check_ch("f3[1]", cap[3*FW+1], CH_SPACE);
        check_ch("f3[2]", cap[3*FW+2], CH_0 + 9);
        check_ch("f3[3]", cap[3*FW+3], CH_0 + 9);
        check_ch("f3[4]", cap[3*FW+4], CH_0 + 9);

        $display("-- 字段 4：v4=12 → 空格 x3 '1' '2' --");
        check_ch("f4[0]", cap[4*FW+0], CH_SPACE);
        check_ch("f4[1]", cap[4*FW+1], CH_SPACE);
        check_ch("f4[2]", cap[4*FW+2], CH_SPACE);
        check_ch("f4[3]", cap[4*FW+3], CH_0 + 1);
        check_ch("f4[4]", cap[4*FW+4], CH_0 + 2);

        $display("-- 字段 5：v5=-7 → 空格 x3 '-' '7'（负号贴最高位）--");
        check_ch("f5[0]", cap[5*FW+0], CH_SPACE);
        check_ch("f5[1]", cap[5*FW+1], CH_SPACE);
        check_ch("f5[2]", cap[5*FW+2], CH_SPACE);
        check_ch("f5[3]", cap[5*FW+3], CH_MINUS);
        check_ch("f5[4]", cap[5*FW+4], CH_0 + 7);

        $display("-- 字段 6：v6=-180 → 空格 '-' '1' '8' '0' --");
        check_ch("f6[0]", cap[6*FW+0], CH_SPACE);
        check_ch("f6[1]", cap[6*FW+1], CH_MINUS);
        check_ch("f6[2]", cap[6*FW+2], CH_0 + 1);
        check_ch("f6[3]", cap[6*FW+3], CH_0 + 8);
        check_ch("f6[4]", cap[6*FW+4], CH_0 + 0);

        $display("-- 字段 7：v7=-260 → 空格 '-' '2' '6' '0' --");
        check_ch("f7[0]", cap[7*FW+0], CH_SPACE);
        check_ch("f7[1]", cap[7*FW+1], CH_MINUS);
        check_ch("f7[2]", cap[7*FW+2], CH_0 + 2);
        check_ch("f7[3]", cap[7*FW+3], CH_0 + 6);
        check_ch("f7[4]", cap[7*FW+4], CH_0 + 0);

        $display("-- 字段 8：v8=12345 → 饱和成 ' 9999'（>9999 不许溢出成乱码）--");
        check_ch("f8[0]", cap[8*FW+0], CH_SPACE);
        check_ch("f8[1]", cap[8*FW+1], CH_0 + 9);
        check_ch("f8[2]", cap[8*FW+2], CH_0 + 9);
        check_ch("f8[3]", cap[8*FW+3], CH_0 + 9);
        check_ch("f8[4]", cap[8*FW+4], CH_0 + 9);

        // 负号不影响后续字段（说明字段边界正确）
        $display("-- 字段边界：字段 2 首字符应是空格而不是 '-' --");
        check_ch("f2[0]_not_minus", (cap[2*FW+0] == CH_MINUS) ? 1 : 0, 0);

        // ---- 改值后下一轮应更新：-123 ----
        $display("-- 改值 -123 → 空格 '-' '1' '2' '3' --");
        v0 = -32'sd123;
        wait_refresh;
        check_ch("f0[0]_after_change", cap[0*FW+0], CH_SPACE);
        check_ch("f0[1]_after_change", cap[0*FW+1], CH_MINUS);
        check_ch("f0[2]_after_change", cap[0*FW+2], CH_0 + 1);
        check_ch("f0[3]_after_change", cap[0*FW+3], CH_0 + 2);
        check_ch("f0[4]_after_change", cap[0*FW+4], CH_0 + 3);

        // ---- ★ 低位补零回归（2026-10-18 预览图抓出的 bug）----
        //   400 的十位是 0，但百位已非零 → 十位必须显示 '0'，不能当空格。
        $display("-- 回归：v0=400 → 空格 空格 '4' '0' '0'（十位的 0 必须显示）--");
        v0 = 32'sd400;
        wait_refresh;
        check_ch("f0_400[0]", cap[0*FW+0], CH_SPACE);
        check_ch("f0_400[1]", cap[0*FW+1], CH_SPACE);
        check_ch("f0_400[2]", cap[0*FW+2], CH_0 + 4);
        check_ch("f0_400[3]", cap[0*FW+3], CH_0 + 0);   // ← 十位补零
        check_ch("f0_400[4]", cap[0*FW+4], CH_0 + 0);

        $display("-- 回归：v0=-200 → 空格 '-' '2' '0' '0' --");
        v0 = -32'sd200;
        wait_refresh;
        check_ch("f0_n200[0]", cap[0*FW+0], CH_SPACE);
        check_ch("f0_n200[1]", cap[0*FW+1], CH_MINUS);
        check_ch("f0_n200[2]", cap[0*FW+2], CH_0 + 2);
        check_ch("f0_n200[3]", cap[0*FW+3], CH_0 + 0);   // ← 十位补零
        check_ch("f0_n200[4]", cap[0*FW+4], CH_0 + 0);

        $display("-- 回归：v0=1000 → 空格 '1' '0' '0' '0'（千位的 1，后面三个 0）--");
        v0 = 32'sd1000;
        wait_refresh;
        check_ch("f0_1000[0]", cap[0*FW+0], CH_SPACE);
        check_ch("f0_1000[1]", cap[0*FW+1], CH_0 + 1);
        check_ch("f0_1000[2]", cap[0*FW+2], CH_0 + 0);
        check_ch("f0_1000[3]", cap[0*FW+3], CH_0 + 0);
        check_ch("f0_1000[4]", cap[0*FW+4], CH_0 + 0);

        $display("-- 回归：v0=-9999 → '-' '9' '9' '9' '9'（4 位负数用满 5 格）--");
        v0 = -32'sd9999;
        wait_refresh;
        check_ch("f0_n9999[0]", cap[0*FW+0], CH_MINUS);
        check_ch("f0_n9999[1]", cap[0*FW+1], CH_0 + 9);
        check_ch("f0_n9999[2]", cap[0*FW+2], CH_0 + 9);
        check_ch("f0_n9999[3]", cap[0*FW+3], CH_0 + 9);
        check_ch("f0_n9999[4]", cap[0*FW+4], CH_0 + 9);

        // ---- ★ 小数位必须补 0（2026-10-18 需求方指出）----
        //   字段 6/7/8 在 ui_render 里 dec = 2 → num_text 的 min_dig = 3，
        //   即整数部分至少 1 位。否则 0.00 会显示成 "   .0"（小数点左边全是空格）。
        $display("-- 回归：v6=0（中心速度 0.00）→ 空格 空格 '0' '0' '0' --");
        v6 = 32'sd0;
        wait_refresh;
        check_ch("f6_zero[0]", cap[6*FW+0], CH_SPACE);
        check_ch("f6_zero[1]", cap[6*FW+1], CH_SPACE);
        check_ch("f6_zero[2]", cap[6*FW+2], CH_0);
        check_ch("f6_zero[3]", cap[6*FW+3], CH_0);
        check_ch("f6_zero[4]", cap[6*FW+4], CH_0);

        $display("-- 回归：v6=5（0.05 m/s）→ 空格 空格 '0' '0' '5' --");
        v6 = 32'sd5;
        wait_refresh;
        check_ch("f6_005[0]", cap[6*FW+0], CH_SPACE);
        check_ch("f6_005[1]", cap[6*FW+1], CH_SPACE);
        check_ch("f6_005[2]", cap[6*FW+2], CH_0);
        check_ch("f6_005[3]", cap[6*FW+3], CH_0);
        check_ch("f6_005[4]", cap[6*FW+4], CH_0 + 5);

        $display("-- 回归：v6=-5（-0.05 m/s）→ 空格 '-' '0' '0' '5' --");
        v6 = -32'sd5;
        wait_refresh;
        check_ch("f6_n005[0]", cap[6*FW+0], CH_SPACE);
        check_ch("f6_n005[1]", cap[6*FW+1], CH_MINUS);
        check_ch("f6_n005[2]", cap[6*FW+2], CH_0);
        check_ch("f6_n005[3]", cap[6*FW+3], CH_0);
        check_ch("f6_n005[4]", cap[6*FW+4], CH_0 + 5);

        $display("-- 回归：v8=0（航位 0.00）→ 空格 空格 '0' '0' '0' --");
        v8 = 32'sd0;
        wait_refresh;
        check_ch("f8_zero[0]", cap[8*FW+0], CH_SPACE);
        check_ch("f8_zero[1]", cap[8*FW+1], CH_SPACE);
        check_ch("f8_zero[2]", cap[8*FW+2], CH_0);
        check_ch("f8_zero[3]", cap[8*FW+3], CH_0);
        check_ch("f8_zero[4]", cap[8*FW+4], CH_0);

        $display("-- 对照：v5=-7（角速度，dec=0）→ 空格 x3 '-' '7'（不受 min_dig 影响）--");
        //   ★ 若有人把 min_dig 做成全局 3 位，这一条会立刻红 —— 它锁住"轮速/角速度不受影响"。
        v5 = -32'sd7;
        wait_refresh;
        check_ch("f5_keep[0]", cap[5*FW+0], CH_SPACE);
        check_ch("f5_keep[1]", cap[5*FW+1], CH_SPACE);
        check_ch("f5_keep[2]", cap[5*FW+2], CH_SPACE);
        check_ch("f5_keep[3]", cap[5*FW+3], CH_MINUS);
        check_ch("f5_keep[4]", cap[5*FW+4], CH_0 + 7);

        // ---------------------------------------------------------------
        // ★ 2026-10-20（综合报错后补的两条覆盖）
        //   背景：参数整定页复用字段 0..11，但参数全是整数。
        //   原 min_dig 写死 "fidx>=6 → 3 位"，会让航向 Kd=6 显示成 "  006"；
        //   ui_render 的 field_dec 还会在百位后画点 → "  0.06"。
        //   现改为由顶层按页面给 i_frac_mask。下面两条锁住这个语义。
        // ---------------------------------------------------------------
        $display("-- 掩码语义：v11=6 且 bit11=0（参数页）→ 空格 x4 '6'（无前导零）--");
        frac_mask = 14'b00_0111_1100_0000;   // bit11 = 0
        v11 = 32'sd6;
        wait_refresh;
        check_ch("f11_nofrac[0]", cap[11*FW+0], CH_SPACE);
        check_ch("f11_nofrac[1]", cap[11*FW+1], CH_SPACE);
        check_ch("f11_nofrac[2]", cap[11*FW+2], CH_SPACE);
        check_ch("f11_nofrac[3]", cap[11*FW+3], CH_SPACE);
        check_ch("f11_nofrac[4]", cap[11*FW+4], CH_0 + 6);

        $display("-- 掩码语义：v6=0 且 bit6=0（参数页）→ 空格 x4 '0'（不是 000）--");
        frac_mask = 14'b00_0000_0000_0000;   // 参数页：全 0
        v6 = 32'sd0;
        wait_refresh;
        check_ch("f6_frac_off[0]", cap[6*FW+0], CH_SPACE);
        check_ch("f6_frac_off[1]", cap[6*FW+1], CH_SPACE);
        check_ch("f6_frac_off[2]", cap[6*FW+2], CH_SPACE);
        check_ch("f6_frac_off[3]", cap[6*FW+3], CH_SPACE);
        check_ch("f6_frac_off[4]", cap[6*FW+4], CH_0);

        // 新字段跨过 6 位地址边界，地址 64 不得回绕覆盖字段 0。
        check_ch("front_addr64", cap[64], CH_0 + 4);
        check_ch("width_addr65", cap[65], CH_SPACE);
        check_ch("width_addr66", cap[66], CH_SPACE);
        check_ch("width_addr67", cap[67], CH_0 + 1);
        check_ch("width_addr68", cap[68], CH_0 + 8);
        check_ch("width_addr69", cap[69], CH_0);

        $display("======== 结果 ========");
        check_ch("window_addr70", cap[70], CH_SPACE);
        check_ch("window_addr71", cap[71], CH_SPACE);
        check_ch("window_addr72", cap[72], CH_0+1);
        check_ch("window_addr73", cap[73], CH_0+2);
        check_ch("window_addr74", cap[74], CH_0+8);
        check_ch("inspect_turn_tens", cap[78], CH_0+3);
        check_ch("inspect_turn_units", cap[79], CH_0);
        if (errors == 0) $display("PASS: num_text all checks passed");
        else             $display("FAIL: %0d checks failed", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule
