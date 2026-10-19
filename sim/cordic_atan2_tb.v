//////////////////////////////////////////////////////////////////////////////////
// 测试台：cordic_atan2
//
// 覆盖 8 个必测点（需求方 2026-10-19 定：四象限 + 正前后 + 零向量 + 对角）：
//   ① (+,0)   → 0      ② (0,+)  → +9000   ③ (−,0) → +18000  ④ (0,−) → −9000
//   ⑤ (+,+)   → +4500  ⑥ (−,+) → +13500  ⑦ (+,−) → −4500   ⑧ (0,0) → o_zero
//
// 容差 ±30（0.3°）：13 级迭代的理论残差 ≤0.01°，留足余量给表量化与移位截断。
// 输入是 Q8 mm：100 mm → 100<<8 = 25600，幅值足够大，x>>i 不会提前变成 0。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps

module cordic_atan2_tb;

    reg                clk = 1'b0;
    reg                rst_n = 1'b0;
    reg                start = 1'b0;
    reg  signed [31:0] ix = 32'sd0, iy = 32'sd0;
    wire signed [15:0] angle;
    wire               valid, zero;

    always #10 clk = ~clk;      // 50 MHz

    cordic_atan2 #(.ITER(13)) dut (
        .clk(clk), .rst_n(rst_n), .i_start(start),
        .i_x(ix), .i_y(iy),
        .o_angle(angle), .o_valid(valid), .o_zero(zero)
    );

    integer errors = 0;
    integer checks = 0;

    task chk_int;
        input [255:0] name;
        input signed [31:0] got;
        input signed [31:0] exp;
        input signed [31:0] tol;
        begin
            checks = checks + 1;
            if ((got > exp + tol) || (got < exp - tol)) begin
                errors = errors + 1;
                $display("FAIL %0s: got %0d, expect %0d +/- %0d", name, got, exp, tol);
            end else begin
                $display("ok   %0s: %0d (expect %0d)", name, got, exp);
            end
        end
    endtask

    // 一次转换：给 x/y，等 valid，检查角度；返回后 zero 也可查
    task do_conv;
        input signed [31:0] x;
        input signed [31:0] y;
        begin
            @(posedge clk);
            ix = x; iy = y; start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            // 等 valid（最多 100 拍）
            begin : wait_v
                integer n;
                for (n = 0; n < 100; n = n + 1) begin
                    @(posedge clk);
                    if (valid) disable wait_v;
                end
                errors = errors + 1;
                $display("FAIL timeout waiting valid");
            end
        end
    endtask

    initial begin
        $display("=== cordic_atan2_tb ===");
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ---- ① 正前方 ----
        do_conv(32'sd25600, 32'sd0);
        chk_int("q1_posx", angle, 32'sd0, 32'sd30);
        chk_int("q1_zero0", {31'd0, zero}, 0, 0);

        // ---- ② 正左（+y，左） ----
        do_conv(32'sd0, 32'sd25600);
        chk_int("q2_posy", angle, 32'sd9000, 32'sd30);

        // ---- ③ 正后（−x） ----
        do_conv(-32'sd25600, 32'sd0);
        chk_int("q3_negx", angle, 32'sd18000, 32'sd30);

        // ---- ④ 正右（−y，右） ----
        do_conv(32'sd0, -32'sd25600);
        chk_int("q4_negy", angle, -32'sd9000, 32'sd30);

        // ---- ⑤ 左前 45° ----
        do_conv(32'sd25600, 32'sd25600);
        chk_int("q5_pp", angle, 32'sd4500, 32'sd30);

        // ---- ⑥ 左后 135° ----
        do_conv(-32'sd25600, 32'sd25600);
        chk_int("q6_np", angle, 32'sd13500, 32'sd30);

        // ---- ⑦ 右前 −45° ----
        do_conv(32'sd25600, -32'sd25600);
        chk_int("q7_pn", angle, -32'sd4500, 32'sd30);

        // ---- ⑧ 零向量 ----
        do_conv(32'sd0, 32'sd0);
        chk_int("q8_zero_flag", {31'd0, zero}, 1, 0);
        chk_int("q8_zero_ang",  angle, 32'sd0, 32'sd30);

        // ---- 小幅值（10 mm → 2560，验证小信号不塌） ----
        do_conv(32'sd2560, 32'sd2560);
        chk_int("small_45", angle, 32'sd4500, 32'sd60);

        // ---- 大幅值（10 m → 2.56e6，验证不溢出） ----
        do_conv(32'sd2560000, 32'sd2560000);
        chk_int("big_45", angle, 32'sd4500, 32'sd30);

        $display("=== cordic_atan2_tb: %0d checks, %0d errors ===", checks, errors);
        if (errors != 0) $display("FAIL: %0d / %0d 项未通过", errors, checks);
        else             $display("PASS: cordic_atan2_tb %0d 项全过", checks);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("FAIL: global timeout");
        $finish;
    end

endmodule
