`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// gyro_tilt_comp 测试平台
//
// 验证内容：
//   1. 水平安装（a=[0,0,1000]）        → k = 1.0，输出直通
//   2. 倾斜 36.87°（a=[600,0,800]）    → k = 1.25，输出 ×1.25
//      选 3-4-5 直角三角形：|a| = 1000，1000×65536/800 = 81920 恰好整除，
//      期望值无舍入歧义，能精确验证整条定点链路（平方和→开方→除法）
//   3. 倾斜 15°（a=[259,0,966]）       → 验证非整除情况下的增益
//   4. 横向分量在 y 轴（a=[0,600,800]）→ 与用例 2 结果相同
//   5. 倒置安装（a=[600,0,-800]）      → k = −1.25，符号自动翻转
//   6. 倾斜超过 60°（a=[1000,0,500]）  → k 限幅到 2.0 并置 tilt_fail
//   7. 标定样本不足                    → 不补偿（k = 1.0）并置 tilt_fail
//   8. 计算期间 o_calib_busy 状态      → 观察状态机确实在跑
//   9. 多轴平均                        → 交替样本求平均后再算倾角
//
// 期望值算法与 RTL 严格一致（isqrt 为向下取整），故可精确比对。
//////////////////////////////////////////////////////////////////////////////////

module gyro_tilt_comp_tb;

    reg  clk;
    reg  rst_n;

    reg  signed [15:0] ax, ay, az;
    reg  signed [31:0] gz;
    reg                sample_valid;
    reg                calib_start;

    wire signed [31:0] gyro_out;
    wire               ready;
    wire               tilt_fail;
    wire [31:0]        gain_q16;
    wire               calib_busy;

    integer errors;

    gyro_tilt_comp #(
        .CALIB_SAMPLES_LG (8),          // 256 个样本
        .GAIN_MIN_Q16     (32768),      // 0.5
        .GAIN_MAX_Q16     (131072)      // 2.0 → 倾斜 60°
    ) dut (
        .i_clock        (clk),
        .i_reset_n      (rst_n),
        .i_acc_x        (ax),
        .i_acc_y        (ay),
        .i_acc_z        (az),
        .i_gyro_z       (gz),
        .i_sample_valid (sample_valid),
        .i_calib_start  (calib_start),
        .o_gyro_z       (gyro_out),
        .o_ready        (ready),
        .o_tilt_fail    (tilt_fail),
        .o_gain_q16     (gain_q16),
        .o_calib_busy   (calib_busy)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;               // 10ns 周期

    //---------------------------------------------------------------
    // 激励辅助
    //---------------------------------------------------------------
    task do_reset;
        begin
            rst_n        = 1'b0;
            ax           = 16'sd0;
            ay           = 16'sd0;
            az           = 16'sd1000;
            gz           = 32'sd0;
            sample_valid = 1'b0;
            calib_start  = 1'b0;
            repeat (10) @(negedge clk);
            rst_n = 1'b1;
            repeat (3) @(negedge clk);
        end
    endtask

    // 送 n 个恒定样本并完成标定（样本数 n < 256 时用于验证"样本不足"）
    task run_calib;
        input signed [15:0] t_ax;
        input signed [15:0] t_ay;
        input signed [15:0] t_az;
        input integer       n;
        integer k;
        begin
            ax = t_ax;
            ay = t_ay;
            az = t_az;
            @(negedge clk);
            calib_start = 1'b1;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk);
                sample_valid = 1'b1;
                @(negedge clk);
                sample_valid = 1'b0;
            end
            @(negedge clk);
            calib_start = 1'b0;         // 下降沿触发计算
            // 等计算结束（o_ready 置位）
            k = 0;
            while ((ready !== 1'b1) && (k < 5000)) begin
                @(negedge clk);
                k = k + 1;
            end
            @(negedge clk);
        end
    endtask

    // 送交替样本（验证求平均），avg_ax 为期望均值
    task run_calib_alt;
        input signed [15:0] ax_lo;
        input signed [15:0] ax_hi;
        input signed [15:0] t_ay;
        input signed [15:0] t_az;
        integer k;
        begin
            ay = t_ay;
            az = t_az;
            @(negedge clk);
            calib_start = 1'b1;
            for (k = 0; k < 256; k = k + 1) begin
                ax = (k % 2 == 0) ? ax_lo : ax_hi;
                @(negedge clk);
                sample_valid = 1'b1;
                @(negedge clk);
                sample_valid = 1'b0;
            end
            @(negedge clk);
            calib_start = 1'b0;
            k = 0;
            while ((ready !== 1'b1) && (k < 5000)) begin
                @(negedge clk);
                k = k + 1;
            end
            @(negedge clk);
        end
    endtask

    task check_gain;
        input [31:0]    expect;
        input [255:0]   name;
        begin
            if (gain_q16 !== expect) begin
                $display("  FAIL: %0s 增益期望 %0d，实际 %0d", name, expect, gain_q16);
                errors = errors + 1;
            end
        end
    endtask

    task check_out;
        input signed [31:0] gz_in;
        input signed [31:0] expect;
        input [255:0]       name;
        begin
            gz = gz_in;
            #1;                          // 等组合乘法稳定
            if (gyro_out !== expect) begin
                $display("  FAIL: %0s 输入 %0d 期望输出 %0d，实际 %0d",
                         name, $signed(gz_in), $signed(expect), $signed(gyro_out));
                errors = errors + 1;
            end
        end
    endtask

    task check_fail;
        input        expect;
        input [255:0] name;
        begin
            if (tilt_fail !== expect) begin
                $display("  FAIL: %0s tilt_fail 期望 %b，实际 %b", name, expect, tilt_fail);
                errors = errors + 1;
            end
        end
    endtask

    task check_ready;
        input        expect;
        input [255:0] name;
        begin
            if (ready !== expect) begin
                $display("  FAIL: %0s ready 期望 %b，实际 %b", name, expect, ready);
                errors = errors + 1;
            end
        end
    endtask

    //---------------------------------------------------------------
    // 主流程
    //---------------------------------------------------------------
    integer wait_cnt;

    initial begin
        errors = 0;

        //===========================================================
        $display("---- 用例1：水平安装 a=[0,0,1000] → k=1.0 ----");
        do_reset();
        check_ready(1'b0, "标定前 ready 应为 0");
        check_out(32'sd8000, 32'sd8000, "标定前应直通");
        run_calib(16'sd0, 16'sd0, 16'sd1000, 256);
        check_ready(1'b1, "标定后 ready 应为 1");
        check_gain(32'd65536, "水平增益（应为 1.0 = 65536）");
        check_fail(1'b0, "水平不应报倾斜失败");
        check_out(32'sd8000, 32'sd8000, "水平输出直通");

        //===========================================================
        $display("---- 用例2：倾斜 36.87° a=[600,0,800] → k=1.25（3-4-5 精确）----");
        do_reset();
        run_calib(16'sd600, 16'sd0, 16'sd800, 256);
        // |a| = isqrt(600²+800²) = isqrt(1000000) = 1000
        // k = (1000<<16)/800 = 81920（1.25）
        check_gain(32'd81920, "倾斜增益（应为 1.25 = 81920）");
        check_fail(1'b0, "36.87° 不应报失败");
        check_out(32'sd8000, 32'sd10000, "8000 → 10000（×1.25）");
        check_out(-32'sd8000, -32'sd10000, "负角速度同样补偿");

        //===========================================================
        $display("---- 用例3：倾斜 15° a=[259,0,966] ----");
        do_reset();
        run_calib(16'sd259, 16'sd0, 16'sd966, 256);
        // |a| = isqrt(259²+966²) = isqrt(1000237) = 1000（向下取整）
        // k = (1000<<16)/966 = 65536000/966 = 67842（余 628）
        check_gain(32'd67842, "15° 倾斜增益");
        check_fail(1'b0, "15° 不应报失败");
        // 输出 = 8000 × 67842 >> 16 = 542736000 >> 16 = 8281
        check_out(32'sd8000, 32'sd8281, "15° 补偿输出");

        //===========================================================
        $display("---- 用例4：横向分量在 y 轴 a=[0,600,800] ----");
        do_reset();
        run_calib(16'sd0, 16'sd600, 16'sd800, 256);
        check_gain(32'd81920, "y 轴倾斜增益应与 x 轴一致");

        //===========================================================
        $display("---- 用例5：倒置安装 a=[600,0,-800] → k=-1.25 ----");
        do_reset();
        run_calib(16'sd600, 16'sd0, -16'sd800, 256);
        // |a_z| = 800，|a| = 1000，因 a_z < 0 故 k 取负
        check_gain(-32'd81920, "倒置增益（应为 -1.25）");
        check_fail(1'b0, "倒置不应报失败");
        check_out(32'sd8000, -32'sd10000, "倒置时符号自动翻转");

        //===========================================================
        $display("---- 用例6：倾斜超过 60° a=[1000,0,500] → 限幅 ----");
        do_reset();
        run_calib(16'sd1000, 16'sd0, 16'sd500, 256);
        // |a| = isqrt(1000²+500²) = isqrt(1250000) = 1118（1119²=1252161 超出）
        // k = (1118<<16)/500 = 146538 > 131072 → 限幅到 2.0 并报失败
        check_gain(32'd131072, "超范围增益应限幅到 2.0");
        check_fail(1'b1, "倾斜 63° 应报 tilt_fail");
        check_out(32'sd8000, 32'sd16000, "限幅后 ×2.0");

        //===========================================================
        $display("---- 用例7：标定样本不足（只给 100 个）----");
        do_reset();
        run_calib(16'sd600, 16'sd0, 16'sd800, 100);
        check_ready(1'b1, "样本不足也应结束标定");
        check_fail(1'b1, "样本不足应报 tilt_fail");
        check_gain(32'd65536, "样本不足时增益应保持 1.0（不补偿）");
        check_out(32'sd8000, 32'sd8000, "不补偿时输出直通");

        //===========================================================
        $display("---- 用例8：计算期间 o_calib_busy 应拉起 ----");
        do_reset();
        ax = 16'sd600; ay = 16'sd0; az = 16'sd800;
        @(negedge clk);
        calib_start = 1'b1;
        for (wait_cnt = 0; wait_cnt < 256; wait_cnt = wait_cnt + 1) begin
            @(negedge clk);
            sample_valid = 1'b1;
            @(negedge clk);
            sample_valid = 1'b0;
        end
        @(negedge clk);
        calib_start = 1'b0;
        // 计算期间 busy 应被拉起（几拍内就能观察到）
        wait_cnt = 0;
        while ((calib_busy !== 1'b1) && (wait_cnt < 20)) begin
            @(negedge clk);
            wait_cnt = wait_cnt + 1;
        end
        if (calib_busy !== 1'b1) begin
            $display("  FAIL: 计算期间 o_calib_busy 未拉起");
            errors = errors + 1;
        end
        wait_cnt = 0;
        while ((calib_busy !== 1'b0) && (wait_cnt < 5000)) begin
            @(negedge clk);
            wait_cnt = wait_cnt + 1;
        end

        //===========================================================
        $display("---- 用例9：交替样本求平均 a_x = 590/610 交替 → 均值 600 ----");
        do_reset();
        run_calib_alt(16'sd590, 16'sd610, 16'sd0, 16'sd800);
        // 均值 = (590+610)/2 = 600 → 与用例2 相同结果
        check_gain(32'd81920, "交替样本平均后增益应与恒定 600 一致");

        //===========================================================
        if (errors == 0) begin
            $display("PASS: gyro_tilt_comp 倾斜补偿（增益计算/符号/限幅/退化）全部测试通过");
        end else begin
            $display("FAIL: 共 %0d 个错误", errors);
        end
        $finish;
    end

    // 总超时保护
    initial begin
        #20_000_000;
        $display("FAIL: 仿真总超时");
        $finish;
    end

endmodule
