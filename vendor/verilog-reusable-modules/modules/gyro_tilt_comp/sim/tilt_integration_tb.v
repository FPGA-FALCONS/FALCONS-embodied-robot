`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// 集成测试：imu_top + gyro_tilt_comp
//
// 【为什么需要这个测试平台】
//   单元测试 gyro_tilt_comp_tb 是直接给 i_calib_start 电平 + i_sample_valid
//   脉冲的，一切正常。但真实系统里 imu_top 在【零偏标定期间不产生 o_data_valid】
//   （标定期 gyro 输出尚未更新，只在 RUN 态才产生）。
//   若把 gyro_tilt_comp 的 i_sample_valid 接到 o_data_valid，标定期间一个样本
//   都采不到 → 倾斜增益永远停在复位值 65536，而单元测试完全发现不了。
//
//   这个平台把两个模块【按真实连接方式】接起来，专门覆盖"标定窗口内能否采到
//   加速度样本"。imu_top 为此新增了 o_acc_valid（标定态与 RUN 态都产生）。
//
// 【用例】
//   1. 倾斜安装 a=[600,0,800] mg  → 标定后 gain 应为 81920（1.25）
//   2. 水平安装 a=[0,0,1000] mg   → 标定后 gain 应为 65536（1.0，不误补偿）
//   3. 负 az（倒置）a=[600,0,-800] → gain 应为 -81920
//
// 【换算依据】±8g 量程下 acc_mg = raw × 1000 >> 12，故
//   600 mg → raw 2458 (0x099A)   800 mg → raw 3277 (0x0CCD)   1000 mg → raw 4096
//   与 bmi270_model 的小端寄存器一致。
//
// 说明：为缩短仿真时间，例化 imu_top 时把 CLK_FREQ_HZ 声明为 1MHz、
//       SCK_HALF_DIV 取 1、SAMPLE_DIV 取 1000（与 imu_top_tb 相同）。
//////////////////////////////////////////////////////////////////////////////////

module tilt_integration_tb;

    reg  clk;
    reg  rst_n;

    wire imu_sck;
    wire imu_mosi;
    wire imu_miso;
    wire imu_cs_n;

    wire        init_done;
    wire        err_flag;
    wire        calib_busy;
    wire        calib_done;
    wire        calib_fail;
    wire signed [31:0] gyro_x, gyro_y, gyro_z;
    wire signed [15:0] acc_x,  acc_y,  acc_z;
    wire               data_valid;
    wire               acc_valid;

    wire signed [31:0] comp_gz;
    wire               tilt_ready;
    wire               tilt_fail;
    wire signed [31:0] tilt_gain;      // 声明为 signed，否则 %d 会按无符号打印负增益

    integer errors;
    integer wait_cnt;

    //------------------------------------------------------------------
    // 被测链路（与 imu_demo_top 中的连接方式一致）
    //------------------------------------------------------------------
    imu_top #(
        .CLK_FREQ_HZ   (1_000_000),
        .SCK_HALF_DIV  (1),
        .SAMPLE_DIV    (1_000),
        .PWR_WAIT_MS   (20),
        .CHK_RETRY_MAX (10)
    ) u_imu (
        .i_clock      (clk),
        .i_reset_n    (rst_n),
        .o_imu_sck    (imu_sck),
        .o_imu_mosi   (imu_mosi),
        .i_imu_miso   (imu_miso),
        .o_imu_cs_n   (imu_cs_n),
        .o_init_busy  (),
        .o_init_done  (init_done),
        .o_err_flag   (err_flag),
        .o_err_code   (),
        .o_state_dbg  (),
        .o_calib_busy (calib_busy),
        .o_calib_done (calib_done),
        .o_calib_fail (calib_fail),
        .o_bias_x     (),
        .o_bias_y     (),
        .o_bias_z     (),
        .o_gyro_x     (gyro_x),
        .o_gyro_y     (gyro_y),
        .o_gyro_z     (gyro_z),
        .o_acc_x      (acc_x),
        .o_acc_y      (acc_y),
        .o_acc_z      (acc_z),
        .o_data_valid (data_valid),
        .o_acc_valid  (acc_valid)
    );

    // ★ 关键连接：用 acc_valid（标定态也产生），不能用 data_valid
    gyro_tilt_comp #(
        .CALIB_SAMPLES_LG (8),
        .GAIN_MIN_Q16     (32768),
        .GAIN_MAX_Q16     (131072)
    ) u_tilt (
        .i_clock        (clk),
        .i_reset_n      (rst_n),
        .i_acc_x        (acc_x),
        .i_acc_y        (acc_y),
        .i_acc_z        (acc_z),
        .i_gyro_z       (gyro_z),
        .i_sample_valid (acc_valid),
        .i_calib_start  (calib_busy),
        .o_gyro_z       (comp_gz),
        .o_ready        (tilt_ready),
        .o_tilt_fail    (tilt_fail),
        .o_gain_q16     (tilt_gain),
        .o_calib_busy   ()
    );

    bmi270_model u_bmi270 (
        .sck  (imu_sck),
        .mosi (imu_mosi),
        .miso (imu_miso),
        .cs_n (imu_cs_n)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    //------------------------------------------------------------------
    // 跑一个用例：设置加速度寄存器 → 复位 → 等标定 → 检查增益
    //------------------------------------------------------------------
    task run_case;
        input [7:0]     ax_l, ax_h, az_l, az_h;
        input [31:0]    expect_gain;
        input [255:0]   name;
        begin
            // 先复位再改寄存器，避免改动被初始化流程干扰
            rst_n = 1'b0;
            repeat (20) @(posedge clk);

            u_bmi270.regfile[8'h0C] = ax_l;     // acc_x 低字节
            u_bmi270.regfile[8'h0D] = ax_h;     // acc_x 高字节
            u_bmi270.regfile[8'h0E] = 8'h00;    // acc_y
            u_bmi270.regfile[8'h0F] = 8'h00;
            u_bmi270.regfile[8'h10] = az_l;     // acc_z 低字节
            u_bmi270.regfile[8'h11] = az_h;     // acc_z 高字节

            rst_n = 1'b1;

            // 等零偏标定结束（成功或失败都会结束）
            wait_cnt = 0;
            while ((calib_done !== 1'b1) && (calib_fail !== 1'b1) && (wait_cnt < 3_000_000)) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end
            if ((calib_done !== 1'b1) && (calib_fail !== 1'b1)) begin
                $display("  FAIL: %0s 零偏标定超时未结束", name);
                errors = errors + 1;
                disable run_case;
            end
            if (calib_fail === 1'b1) begin
                $display("  FAIL: %0s 零偏标定失败（本用例不应失败）", name);
                errors = errors + 1;
            end

            // 等倾斜增益算完
            wait_cnt = 0;
            while ((tilt_ready !== 1'b1) && (wait_cnt < 100_000)) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end

            if (tilt_ready !== 1'b1) begin
                $display("  FAIL: %0s 倾斜增益计算未结束（o_ready 未置位）", name);
                errors = errors + 1;
            end else if (tilt_gain !== expect_gain) begin
                $display("  FAIL: %0s 倾斜增益期望 %0d (0x%08h)，实际 %0d (0x%08h)",
                         name, $signed(expect_gain), expect_gain, tilt_gain, tilt_gain);
                if (tilt_gain == 32'd65536)
                    $display("        增益停在复位默认值 → 标定窗口内没采到加速度样本！");
                errors = errors + 1;
            end else begin
                $display("  OK  : %0s 倾斜增益 = %0d  (实际读到 acc=[%0d,%0d,%0d] mg)",
                         name, tilt_gain, acc_x, acc_y, acc_z);
            end

            if (tilt_fail !== 1'b0) begin
                $display("  FAIL: %0s 不应报 tilt_fail（读到 acc=[%0d,%0d,%0d] mg）",
                         name, acc_x, acc_y, acc_z);
                errors = errors + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        errors      = 0;
        rst_n       = 1'b0;
        wait_cnt    = 0;

        $display("");
        $display("==== 用例1：倾斜安装 a=[600,0,800] mg → 期望 gain=81920 ====");
        // 600 mg → raw 2458 = 0x099A；800 mg → raw 3277 = 0x0CCD
        run_case(8'h9A, 8'h09, 8'hCD, 8'h0C, 32'd81920, "倾斜安装");

        $display("");
        $display("==== 用例2：水平安装 a=[0,0,1000] mg → 期望 gain=65536 ====");
        // 1000 mg → raw 4096 = 0x1000
        run_case(8'h00, 8'h00, 8'h00, 8'h10, 32'd65536, "水平安装");

        $display("");
        $display("==== 用例3：倒置安装 a=[600,0,-800] mg → 期望 gain=-81920 ====");
        // -800 mg → raw -3276 = 0xF334。注意不能用 -3277：换算 acc_mg = raw*1000>>12
        // 对负数是向下取整，-3277 会得到 -801 mg，反而让期望值变成 81817。
        run_case(8'h9A, 8'h09, 8'h34, 8'hF3, -32'd81920, "倒置安装");

        $display("");
        if (errors == 0) begin
            $display("PASS: imu_top + gyro_tilt_comp 集成测试通过（标定窗口内能正确采到加速度样本）");
        end else begin
            $display("FAIL: 共 %0d 个错误", errors);
        end
        $finish;
    end

    // 总超时保护
    initial begin
        #200_000_000;
        $display("FAIL: 仿真总超时");
        $finish;
    end

endmodule
