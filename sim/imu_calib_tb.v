`timescale 1ns/1ps

// 在 SPI 接收字节接口注入采样，执行真实标定状态机及提交流水。
// 初始化配置传输由原驱动负责，本台从配置完成处进入，不模拟芯片固件上传。
module imu_calib_tb;
    reg clk = 0;
    reg rst_n = 0;
    always #10 clk = ~clk;

    reg rx_valid = 0;
    reg [15:0] rx_index = 0;
    reg [7:0] rx_byte = 0;
    reg done = 0;
    wire calib_done, calib_fail, acc_valid, data_valid;
    wire signed [31:0] bias_x, bias_y, bias_z, gx, gy, gz;
    wire signed [15:0] ax, ay, az;
    integer checks = 0, errors = 0, cycles = 0;
    integer acc_pulses = 0, gyro_pulses = 0, n;
    integer capture_cycle = 0, min_latency = 100, max_latency = 0;

    imu_top dut (
        .i_clock(clk), .i_reset_n(rst_n), .i_imu_miso(1'b0),
        .o_calib_done(calib_done), .o_calib_fail(calib_fail),
        .o_bias_x(bias_x), .o_bias_y(bias_y), .o_bias_z(bias_z),
        .o_gyro_x(gx), .o_gyro_y(gy), .o_gyro_z(gz),
        .o_acc_x(ax), .o_acc_y(ay), .o_acc_z(az),
        .o_acc_valid(acc_valid), .o_data_valid(data_valid)
    );

    always @(posedge clk) begin
        cycles = cycles + 1;
        if (done && ((dut.state == dut.S_CALIB) || (dut.state == dut.S_RUN)))
            capture_cycle = cycles;
        #1;
        if (acc_valid) begin
            if (cycles - capture_cycle < min_latency) min_latency = cycles - capture_cycle;
            if (cycles - capture_cycle > max_latency) max_latency = cycles - capture_cycle;
        end
        if (acc_valid) acc_pulses = acc_pulses + 1;
        if (data_valid) gyro_pulses = gyro_pulses + 1;
    end

    task check;
        input [255:0] name;
        input condition;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s cycle=%0d", name, cycles);
            end
        end
    endtask

    task reset_calibration;
        begin
            @(negedge clk); rst_n = 0; done = 0; rx_valid = 0;
            repeat (3) @(negedge clk);
            rst_n = 1;
            // 只跳过初始化；后续接收、计数、均值、拒收及完成全部运行 RTL。
            dut.state = dut.S_CALIB;
            dut.cmd_type = dut.C_RDN;
            acc_pulses = 0;
            gyro_pulses = 0;
        end
    endtask

    task byte_in;
        input integer index;
        input [7:0] value;
        begin
            @(negedge clk);
            rx_valid = 1;
            rx_index = index + 2;
            rx_byte = value;
        end
    endtask

    task sample;
        input signed [15:0] x, y, z;
        begin
            byte_in(0, 8'h00); byte_in(1, 8'h10);  // +1000 mg
            byte_in(2, 8'h00); byte_in(3, 8'hF0);  // -1000 mg
            byte_in(4, 8'h00); byte_in(5, 8'h20);  // +2000 mg
            byte_in(6, x[7:0]); byte_in(7, x[15:8]);
            byte_in(8, y[7:0]); byte_in(9, y[15:8]);
            byte_in(10, z[7:0]); byte_in(11, z[15:8]);
            @(negedge clk); rx_valid = 0; done = 1;
            @(negedge clk); done = 0;
            // 下一份接收数据改变原始缓冲，不能污染已经锁存的本份样本。
            byte_in(6, 8'hFF);
            byte_in(7, 8'h7F);
            @(negedge clk); rx_valid = 0;
            repeat (2) @(negedge clk);
        end
    endtask

    initial begin
        force dut.spi_rx_valid = rx_valid;
        force dut.spi_rx_idx = rx_index;
        force dut.spi_rx_byte = rx_byte;
        force dut.spi_done = done;
        force dut.sample_tick = 1'b0;  // 手工注入采样，不启动物理 SPI。
        reset_calibration;
        sample(16, -16, 32);  // 换算后 97、-98、195。
        check("first_sample_count", dut.calib_cnt == 1);
        check("first_sample_sum", dut.calib_sum_x == 97 && dut.calib_sum_y == -98);
        check("coherent_acceleration", ax == 1000 && ay == -1000 && az == 2000);
        check("calibration_valid_only_acc", acc_pulses == 1 && gyro_pulses == 0);
        for (n = 1; n < 15; n = n + 1) sample(16, -16, 32);
        check("before_seed_boundary", !dut.calib_seed_done && dut.calib_cnt == 15);
        sample(16, -16, 32);
        check("seed_count", dut.calib_seed_done && dut.calib_cnt == 16);
        check("seed_mean", dut.calib_mean_x == 97 && dut.calib_mean_y == -98 &&
                           dut.calib_mean_z == 195);
        sample(2000, -16, 32);
        check("moving_sample_rejected", dut.calib_cnt == 16 && !calib_done);
        for (n = 16; n < 511; n = n + 1) sample(16, -16, 32);
        check("before_completion", dut.calib_cnt == 511 && !calib_done);
        dut.calib_tick = 3000;  // 最后一份有效样本优先于同拍超时。
        sample(16, -16, 32);
        check("completion_count", dut.calib_cnt == 512 && calib_done && !calib_fail);
        check("bias_same_sample", bias_x == 97 && bias_y == -98 && bias_z == 195);
        check("completion_state", dut.state == dut.S_RUN);
        check("all_acc_pulses", acc_pulses == 513 && gyro_pulses == 0);
        sample(32, -32, 48);
        check("running_bias_subtraction", gx == 98 && gy == -98 && gz == 97);
        check("running_pulses", acc_pulses == 514 && gyro_pulses == 1);

        reset_calibration;
        for (n = 0; n < 16; n = n + 1) sample(0, 0, 0);
        dut.calib_tick = 3000;
        sample(2000, 0, 0);
        check("timeout_rejected_sample", calib_fail && !calib_done && dut.calib_cnt == 16);
        check("timeout_no_bias", bias_x == 0 && bias_y == 0 && bias_z == 0);
        check("timeout_running_state", dut.state == dut.S_RUN);

        reset_calibration;
        for (n = 0; n < 16; n = n + 1) sample(1000, 0, 0);
        sample(1000, 0, 0);  // 偏差为零，但绝对读数超限，不能计入。
        check("absolute_gate_after_seed", dut.calib_cnt == 16 && !calib_done && !calib_fail);

        reset_calibration;
        dut.state = dut.S_RUN;
        sample(-32768, 32767, -1);
        check("signed_extremes", gx == -200000 && gy == 199993 && gz == -7);
        @(negedge clk); done = 1;
        @(negedge clk); done = 0; rst_n = 0;
        repeat (4) @(negedge clk);
        check("reset_drops_pending_sample", !acc_valid && !data_valid && !calib_done);
        check("two_cycle_commit_latency", min_latency == 2 && max_latency == 2);
        $display("CYCLES: %0d; sample commit latency min=%0d max=%0d", cycles, min_latency, max_latency);
        if (errors == 0) $display("PASS: imu_calib_tb %0d checks", checks);
        else $display("FAIL: imu_calib_tb %0d errors / %0d checks", errors, checks);
        $finish;
    end

    initial begin
        #1000000;
        $display("FAIL: imu_calib_tb timeout");
        $finish;
    end
endmodule
