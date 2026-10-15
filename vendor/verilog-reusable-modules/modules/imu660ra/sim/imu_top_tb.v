`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// imu_top 测试平台
//
// 三个用例（每个用例都从复位开始，独立验证）：
//   用例1  静止 + 零偏 + 噪声            → 标定必须成功，bias 正确、扣除后输出≈0
//   用例2  匀速转动（角速度恒定且很大）  → 标定必须失败（绝对门禁拦住）
//   用例3  剧烈抖动                      → 标定必须失败（抖动判据拦住）
//
// 用例2 是本测试平台的重点：纯抖动判据对"匀速转动"完全无效（此时抖动为 0），
// 必须靠"三轴 |读数| 之和 < CALIB_ABS_LIMIT"的绝对门禁才能拦住。
// （早期版本的模型用恒定 250°/s 当"静止"，恰好掩盖了这个缺陷。）
//
// 说明：为了缩短仿真时间，例化 imu_top 时把 CLK_FREQ_HZ 声明为 1MHz、
//       SCK_HALF_DIV 取 1、SAMPLE_DIV 取 1000。这些只影响"周期数"，
//       不影响逻辑正确性（真实工程用 50MHz / 1MHz SCK / 1kHz 采样）。
//////////////////////////////////////////////////////////////////////////////////

module imu_top_tb;

    reg  clk;
    reg  rst_n;

    wire imu_sck;
    wire imu_mosi;
    wire imu_miso;
    wire imu_cs_n;

    wire        init_busy;
    wire        init_done;
    wire        err_flag;
    wire [7:0]  err_code;
    wire [7:0]  state_dbg;
    wire        calib_busy;
    wire        calib_done;
    wire        calib_fail;
    wire signed [31:0] bias_x, bias_y, bias_z;

    wire signed [31:0] gyro_x, gyro_y, gyro_z;
    wire signed [15:0] acc_x,  acc_y,  acc_z;
    wire               data_valid;

    integer errors;
    integer wait_cnt;
    integer mism;
    integer i;

    //------------------------------------------------------------------
    // 被测模块
    //------------------------------------------------------------------
    imu_top #(
        .CLK_FREQ_HZ   (1_000_000),     // 声明值，仅用于计算延时周期数
        .SCK_HALF_DIV  (1),
        .SAMPLE_DIV    (1_000),
        .PWR_WAIT_MS   (20),
        .CHK_RETRY_MAX (10)
    ) dut (
        .i_clock      (clk),
        .i_reset_n    (rst_n),
        .o_imu_sck    (imu_sck),
        .o_imu_mosi   (imu_mosi),
        .i_imu_miso   (imu_miso),
        .o_imu_cs_n   (imu_cs_n),
        .o_init_busy  (init_busy),
        .o_init_done  (init_done),
        .o_err_flag   (err_flag),
        .o_err_code   (err_code),
        .o_state_dbg  (state_dbg),
        .o_calib_busy (calib_busy),
        .o_calib_done (calib_done),
        .o_calib_fail (calib_fail),
        .o_bias_x     (bias_x),
        .o_bias_y     (bias_y),
        .o_bias_z     (bias_z),
        .o_gyro_x     (gyro_x),
        .o_gyro_y     (gyro_y),
        .o_gyro_z     (gyro_z),
        .o_acc_x      (acc_x),
        .o_acc_y      (acc_y),
        .o_acc_z      (acc_z),
        .o_data_valid (data_valid)
    );

    bmi270_model u_bmi270 (
        .sck  (imu_sck),
        .mosi (imu_mosi),
        .miso (imu_miso),
        .cs_n (imu_cs_n)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;               // 周期 10ns

    //------------------------------------------------------------------
    // 噪声/运动注入
    //   mode 0 = 关闭
    //   mode 1 = 小噪声：gyro_x 在 65~96 间跳（±15 raw ≈ ±0.9°/s）
    //            模拟"静止但有噪声"，应能通过抖动判据
    //   mode 2 = 剧烈抖动：gyro_x 在 0~255 间跳（约 ±12°/s）
    //            超过抖动判据，应判失败
    //------------------------------------------------------------------
    reg [1:0] noise_mode = 2'd0;
    reg [7:0] noise_cnt  = 8'd0;

    always @(posedge clk) begin
        if (noise_mode != 2'd0 && calib_busy) begin
            noise_cnt <= noise_cnt + 8'd1;
            if (noise_mode == 2'd1) begin
                u_bmi270.regfile[8'h12] <= 8'h41 + {3'b000, noise_cnt[4:0]};  // 65~96
            end else begin
                u_bmi270.regfile[8'h12] <= noise_cnt;                          // 0~255
            end
        end
    end

    //------------------------------------------------------------------
    // 采样观测
    //------------------------------------------------------------------
    integer valid_cnt;
    reg signed [31:0] last_gyro_x;
    reg signed [15:0] last_acc_z;

    always @(posedge clk) begin
        if (data_valid) begin
            valid_cnt   <= valid_cnt + 1;
            last_gyro_x <= gyro_x;
            last_acc_z  <= acc_z;
        end
    end

    //------------------------------------------------------------------
    // 检查任务
    //------------------------------------------------------------------
    task check_reg;
        input [7:0]   addr;
        input [7:0]   expect;
        input [127:0] name;
        begin
            if (u_bmi270.regfile[addr] !== expect) begin
                $display("  FAIL: 寄存器 0x%02h(%0s) 期望 %02h，实际 %02h",
                         addr, name, expect, u_bmi270.regfile[addr]);
                errors = errors + 1;
            end
        end
    endtask

    task do_reset;
        begin
            rst_n = 1'b0;
            repeat (10) @(negedge clk);
            rst_n = 1'b1;
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        errors      = 0;
        valid_cnt   = 0;
        last_gyro_x = 32'sd0;
        last_acc_z  = 16'sd0;
        rst_n       = 1'b0;

        //==============================================================
        // 用例 1：静止 + 零偏 + 噪声 → 标定应成功
        //==============================================================
        $display("");
        $display("==== 用例1：静止（零偏 4.88°/s + 噪声）→ 应标定成功 ====");
        noise_mode = 2'd1;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;

        wait_cnt = 0;
        while ((init_done !== 1'b1 || calib_done !== 1'b1) && wait_cnt < 9_000_000) begin
            @(negedge clk);
            wait_cnt = wait_cnt + 1;
        end
        noise_mode = 2'd0;
        // 停止注入后把读数放回名义零偏值：否则它会冻结在噪声的最后一个采样上，
        // 与"512 次采样的均值"相比偏差可达 ±15 raw，看起来像扣除不准
        u_bmi270.regfile[8'h12] = 8'h50;    // 80 raw ≈ 4.88°/s

        if (calib_done !== 1'b1) begin
            $display("  FAIL: 初始化/标定超时，state_dbg=%0d calib_fail=%b", state_dbg, calib_fail);
            errors = errors + 1;
        end else begin
            $display("  OK  : 初始化与标定完成（用时约 %0d 拍）", wait_cnt);
        end

        if (calib_fail !== 1'b0) begin
            $display("  FAIL: 静止工况不应判为标定失败");
            errors = errors + 1;
        end

        if (u_bmi270.burst_cnt != 16'd8192) begin
            $display("  FAIL: INIT_DATA 写入字节数应为 8192，实际 %0d", u_bmi270.burst_cnt);
            errors = errors + 1;
        end else begin
            $display("  OK  : INIT_DATA 完整写入 8192 字节");
        end

        mism = 0;
        for (i = 0; i < 8192; i = i + 1) begin
            if (u_bmi270.cfg_ram[i] !== dut.u_cfg_rom.rom[i]) begin
                if (mism < 3) begin
                    $display("  FAIL: 配置字节 %0d 不符：发送 %02h，应为 %02h",
                             i, u_bmi270.cfg_ram[i], dut.u_cfg_rom.rom[i]);
                end
                mism = mism + 1;
            end
        end
        if (mism == 0) $display("  OK  : 8192 字节配置逐字节一致");
        else begin
            $display("  FAIL: %0d 个配置字节不一致", mism);
            errors = errors + 1;
        end

        check_reg(8'h7C, 8'h00, "PWR_CONF");
        check_reg(8'h59, 8'h01, "INIT_CTRL");
        check_reg(8'h41, 8'h02, "ACC_RANGE");
        check_reg(8'h40, 8'hA7, "ACC_CONF");
        check_reg(8'h43, 8'h00, "GYR_RANGE");
        check_reg(8'h42, 8'hA9, "GYR_CONF");
        check_reg(8'h7D, 8'h0E, "PWR_CTRL");

        // bias ≈ 80 raw × 25000/4096 = 488（噪声使均值略偏，容差 ±30）
        if ((bias_x > 32'sd518) || (bias_x < 32'sd458)) begin
            $display("  FAIL: bias_x 期望约 488，实际 %0d", bias_x);
            errors = errors + 1;
        end else begin
            $display("  OK  : bias_x = %0d (0.01°/s) ≈ 4.88°/s，已正确测出零偏", bias_x);
        end

        if (bias_y !== 32'sd0 || bias_z !== 32'sd0) begin
            $display("  FAIL: bias_y/z 期望 0，实际 %0d / %0d", bias_y, bias_z);
            errors = errors + 1;
        end

        wait_cnt = 0;
        while (valid_cnt < 3 && wait_cnt < 1_000_000) begin
            @(negedge clk);
            wait_cnt = wait_cnt + 1;
        end
        if (valid_cnt < 3) begin
            $display("  FAIL: 未收到足够数据帧");
            errors = errors + 1;
        end

        if ((last_gyro_x > 32'sd10) || (last_gyro_x < -32'sd10)) begin
            $display("  FAIL: 扣除零偏后 gyro_x 期望≈0，实际 %0d", last_gyro_x);
            errors = errors + 1;
        end else begin
            $display("  OK  : 扣除零偏后 gyro_x = %0d（≈0）", last_gyro_x);
        end

        if (last_acc_z !== 16'sd1000) begin
            $display("  FAIL: acc_z 期望 1000 mg，实际 %0d", last_acc_z);
            errors = errors + 1;
        end else begin
            $display("  OK  : acc_z = 1000 mg");
        end

        //==============================================================
        // 用例 2：匀速转动 → 必须判失败（绝对门禁）
        //==============================================================
        $display("");
        $display("==== 用例2：匀速转动（恒定 250°/s，抖动为0）→ 应标定失败 ====");
        do_reset;
        u_bmi270.regfile[8'h12] = 8'h00;    // 0x1000 = 4096 raw = 250°/s
        u_bmi270.regfile[8'h13] = 8'h10;

        wait_cnt = 0;
        while (calib_fail !== 1'b1 && wait_cnt < 9_000_000) begin
            @(negedge clk);
            wait_cnt = wait_cnt + 1;
        end

        if (calib_fail !== 1'b1) begin
            $display("  FAIL: 匀速转动应判失败，但 calib_fail 未置位（绝对门禁失效！）");
            errors = errors + 1;
        end else begin
            $display("  OK  : 匀速转动被判标定失败（绝对门禁生效，未写入错误零偏）");
        end
        if (bias_x !== 32'sd0 || bias_y !== 32'sd0 || bias_z !== 32'sd0) begin
            $display("  FAIL: 标定失败时零偏应保持 0，实际 %0d/%0d/%0d", bias_x, bias_y, bias_z);
            errors = errors + 1;
        end

        //==============================================================
        // 用例 3：剧烈抖动 → 必须判失败（抖动判据）
        //==============================================================
        $display("");
        $display("==== 用例3：剧烈抖动（±12°/s）→ 应标定失败 ====");
        do_reset;
        u_bmi270.regfile[8'h12] = 8'h50;    // 回到 80
        u_bmi270.regfile[8'h13] = 8'h00;
        noise_mode = 2'd2;

        wait_cnt = 0;
        while (calib_fail !== 1'b1 && wait_cnt < 9_000_000) begin
            @(negedge clk);
            wait_cnt = wait_cnt + 1;
        end
        noise_mode = 2'd0;

        if (calib_fail !== 1'b1) begin
            $display("  FAIL: 剧烈抖动应判失败，但 calib_fail 未置位");
            errors = errors + 1;
        end else begin
            $display("  OK  : 剧烈抖动被判标定失败（抖动判据生效）");
        end

        //==============================================================
        $display("");
        if (errors == 0)
            $display("PASS: imu_top 初始化、零偏标定（含匀速转动防护）与数据读取测试全部通过");
        else
            $display("FAIL: 共 %0d 个错误", errors);
        $finish;
    end

    initial begin
        #300_000_000;
        $display("FAIL: 仿真总超时");
        $finish;
    end

endmodule
