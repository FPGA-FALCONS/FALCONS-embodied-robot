//////////////////////////////////////////////////////////////////////////////////
// num_text_tb —— 实时数值格式化单元测试
//
// 检查 **9 个字段**的字符输出（每字段 4 字符：符号 + 百位 + 十位 + 个位）
//   字库索引 = ASCII − 32 → 空格=0  '-'=13  '0'=16 …
//   前导零写空格：0 → "   0"、12 → "  12"、-45 → "- 45"、-7 → "-  7"
//   ★ 2026-10-16：字段数 7 → 9（顶层与 RTL 默认都是 9，本台原先只传 7，
//     于是字段 7/8（航位 x,y）那条路径从未被激励 —— 上板若显示乱字符查不出来）。
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

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    reg signed [31:0] v0, v1, v2, v3, v4, v5, v6, v7, v8;

    wire        o_we;
    wire [5:0]  o_addr;
    wire [7:0]  o_data;

    integer errors = 0;
    integer i;

    // 抓取写入的字符（64 深，避免 o_addr 越界）
    reg [7:0] cap [0:63];

    num_text #(
        .CLK_HZ     (10_000_000),
        .REFRESH_MS (1),
        .FIELD_W    (4),
        .NUM_FIELDS (9)
    ) dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_v0   (v0), .i_v1 (v1), .i_v2 (v2),
        .i_v3   (v3), .i_v4 (v4), .i_v5 (v5), .i_v6 (v6),
        .i_v7   (v7), .i_v8 (v8),
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
        $display("======== num_text 单元测试（实时数值格式化）========");

        for (i = 0; i < 64; i = i + 1) cap[i] = 8'hFF;   // 用 FF 标记"未写入"

        v0 = 32'sd300;      // → " 300"
        v1 = -32'sd45;      // → "- 45"
        v2 = 32'sd0;        // → "   0"
        v3 = 32'sd999;      // → " 999"
        v4 = 32'sd12;       // → "  12"
        v5 = -32'sd7;       // → "-  7"
        v6 = -32'sd180;     // → "-180"（验证 3 位数字够用）
        v7 = -32'sd260;     // → "-260"（字段 7 = 航位 x）
        v8 = 32'sd1234;     // → " 999"（**饱和**测试：>999 时不许溢出成乱码）

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        wait_refresh;

        $display("-- 字段 0：v0=300 → 空格 '3' '0' '0' --");
        check_ch("f0[0]", cap[0],  CH_SPACE);
        check_ch("f0[1]", cap[1],  CH_0 + 3);
        check_ch("f0[2]", cap[2],  CH_0 + 0);
        check_ch("f0[3]", cap[3],  CH_0 + 0);

        $display("-- 字段 1：v1=-45 → '-' 空格 '4' '5' --");
        check_ch("f1[0]", cap[4],  CH_MINUS);
        check_ch("f1[1]", cap[5],  CH_SPACE);
        check_ch("f1[2]", cap[6],  CH_0 + 4);
        check_ch("f1[3]", cap[7],  CH_0 + 5);

        $display("-- 字段 2：v2=0 → 空格 空格 空格 '0' --");
        check_ch("f2[0]", cap[8],  CH_SPACE);
        check_ch("f2[1]", cap[9],  CH_SPACE);
        check_ch("f2[2]", cap[10], CH_SPACE);
        check_ch("f2[3]", cap[11], CH_0);

        $display("-- 字段 3：v3=999 → '9' '9' '9' --");
        check_ch("f3[1]", cap[13], CH_0 + 9);
        check_ch("f3[2]", cap[14], CH_0 + 9);
        check_ch("f3[3]", cap[15], CH_0 + 9);

        $display("-- 字段 4：v4=12 → 空格 空格 '1' '2' --");
        check_ch("f4[2]", cap[18], CH_0 + 1);
        check_ch("f4[3]", cap[19], CH_0 + 2);

        $display("-- 字段 5：v5=-7 → '-' 空格 空格 '7' --");
        check_ch("f5[0]", cap[20], CH_MINUS);
        check_ch("f5[3]", cap[23], CH_0 + 7);

        $display("-- 字段 6：v6=-180 → '-' '1' '8' '0' --");
        check_ch("f6[0]", cap[24], CH_MINUS);
        check_ch("f6[1]", cap[25], CH_0 + 1);
        check_ch("f6[2]", cap[26], CH_0 + 8);
        check_ch("f6[3]", cap[27], CH_0 + 0);

        $display("-- 字段 7：v7=-260 → '-' '2' '6' '0' --");
        check_ch("f7[0]", cap[28], CH_MINUS);
        check_ch("f7[1]", cap[29], CH_0 + 2);
        check_ch("f7[2]", cap[30], CH_0 + 6);
        check_ch("f7[3]", cap[31], CH_0 + 0);

        $display("-- 字段 8：v8=1234 → 饱和显示 ' 999'（>999 不许溢出成乱码）--");
        check_ch("f8[0]", cap[32], CH_SPACE);
        check_ch("f8[1]", cap[33], CH_0 + 9);
        check_ch("f8[2]", cap[34], CH_0 + 9);
        check_ch("f8[3]", cap[35], CH_0 + 9);

        // 负号不影响后续字段（说明字段边界正确）
        $display("-- 字段边界：字段 2 首字符应是空格而不是 '-' --");
        check_ch("f2[0]_not_minus", (cap[8] == CH_MINUS) ? 1 : 0, 0);

        // 改值后下一轮应更新
        $display("-- 改值 -123 → 下一轮应刷成 '-' '1' '2' '3' --");
        v0 = -32'sd123;
        wait_refresh;
        check_ch("f0[0]_after_change", cap[0], CH_MINUS);
        check_ch("f0[1]_after_change", cap[1], CH_0 + 1);
        check_ch("f0[2]_after_change", cap[2], CH_0 + 2);
        check_ch("f0[3]_after_change", cap[3], CH_0 + 3);

        $display("======== 结果 ========");
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
