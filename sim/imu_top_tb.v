//////////////////////////////////////////////////////////////////////////////
// sim\imu_top_tb.v -- imu_top 角速度/加速度换算的模块级回归
//
// 历史说明：本 tb 最初是为 2026-10-20 那次「rx_buf 与 ×25000 之间插三级打拍
//          寄存器 (gyro_x/y/z_raw_r)」的改动写的。该改动在 22:05 综合复核后
//          确认并未改善时序（同域违例 1 条 → 3 条，最差 −1.297 → −2.310），
//          已按用户指令回退，`gyro_*_raw_r` 三级寄存器不复存在。
//          本 tb 因此收窄为「换算通路回归」，不再检查打拍延迟。
//
// 目的：锁定 `rx_buf -> gyro_*_cs` 这段换算的数值语义，防止将来改动时
//       破坏增益系数、符号扩展或算术右移的取整方向。
//         · gyro: raw × 25000 >>> 12   (25000 = 0x61A8，6 个 1 的移位加)
//         · acc : raw × 1000  >>> 12
//
// 手段：SPI 总线交互太长（需要模拟 IMU660RA 芯片的字节响应），
//       这里用 `force` 直接压 `rx_buf`（imu_top 内部的 reg 数组），
//       只验证「从 rx_buf 到 gyro_*_cs / acc_*_mg」这一段。
//       打拍回退后这段是纯组合，force 之后 #1 即可采样。
//
// 检查项：
//   G1 复位后 gyro_*_cs 全 0
//   G2 正数：rx_buf[9:8] = 0x12,0x34 -> raw = 4660 -> cs = 4660*25000>>12 = 28442
//   G3 负数：0xFFFF -> raw = -1     -> cs = -7      （算术右移向下取整）
//            0x8000 -> raw = -32768 -> cs = -200000
//   G4 三轴独立：x/y/z 各压不同值，互不串扰
//   G5 加速度通路：acc_y = 0x1000 -> acc_y_mg = 1000
//////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module imu_top_tb;

    localparam integer CLK_PERIOD = 20;          // 50 MHz

    reg         clk;
    reg         rst_n;

    // SPI 物理接口
    wire        imu_sck, imu_mosi, imu_cs_n;
    reg         imu_miso;

    // 状态
    wire        init_busy, init_done, err_flag;
    wire [7:0]  err_code, state_dbg;
    wire        calib_busy, calib_done, calib_fail;
    wire signed [31:0] bias_x, bias_y, bias_z;

    // 数据
    wire signed [31:0] gyro_x, gyro_y, gyro_z;
    wire signed [15:0] acc_x, acc_y, acc_z;
    wire               data_valid, acc_valid;

    integer checks = 0;
    integer errors = 0;

    task chk_int(input [255:0] name, input integer got, input integer exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL %0s: got %0d expect %0d (t=%0t)", name, got, exp, $time);
            end
        end
    endtask

    task step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    imu_top #(
        .CALIB_APPLY   (0)        // 关掉零偏扣除，直接看原始换算
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
        .o_data_valid (data_valid),
        .o_acc_valid  (acc_valid)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    // MISO 恒 0：本 tb 不关心 SPI 内容，只借用它的时序
    initial imu_miso = 1'b0;

    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;

        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) step;

        // ================= G1 复位后 =================
        // rx_buf 全 0（未 force），换算结果应为 0
        chk_int("g1_cs_x_zero", dut.gyro_x_cs, 0);
        chk_int("g1_cs_y_zero", dut.gyro_y_cs, 0);
        chk_int("g1_cs_z_zero", dut.gyro_z_cs, 0);

        // ================= G2 正数换算 =================
        force dut.rx_buf[8]  = 8'h34;      // gyro_y 低字节
        force dut.rx_buf[9]  = 8'h12;      // gyro_y 高字节  -> 0x1234 = 4660
        #1;                                 // 组合直通
        chk_int("g2_raw_y",        dut.gyro_y_raw, 16'sh1234);
        chk_int("g2_cs_y_positive", dut.gyro_y_cs, 28442);   // 4660*25000>>12 = 28442.9 -> 28442

        // ================= G3 负数换算 =================
        force dut.rx_buf[8]  = 8'hFF;
        force dut.rx_buf[9]  = 8'hFF;      // -> 0xFFFF = -1
        #1;
        chk_int("g3_raw_y_neg1",   dut.gyro_y_raw, -1);
        chk_int("g3_cs_y_neg1",    dut.gyro_y_cs, -7);       // -25000>>12 = -7（算术右移向下取整）

        force dut.rx_buf[8]  = 8'h00;
        force dut.rx_buf[9]  = 8'h80;      // -> 0x8000 = -32768（最负）
        #1;
        chk_int("g3_raw_y_min",    dut.gyro_y_raw, -32768);
        chk_int("g3_cs_y_min",     dut.gyro_y_cs, -200000);  // -32768*25000/4096 = -200000

        // ================= G4 三轴独立 =================
        force dut.rx_buf[6]  = 8'h00; force dut.rx_buf[7]  = 8'h10;   // gyro_x = 0x1000 = 4096
        force dut.rx_buf[8]  = 8'h00; force dut.rx_buf[9]  = 8'h20;   // gyro_y = 0x2000 = 8192
        force dut.rx_buf[10] = 8'h00; force dut.rx_buf[11] = 8'h30;   // gyro_z = 0x3000 = 12288
        #1;
        chk_int("g4_raw_x", dut.gyro_x_raw, 16'sh1000);
        chk_int("g4_raw_y", dut.gyro_y_raw, 16'sh2000);
        chk_int("g4_raw_z", dut.gyro_z_raw, 16'sh3000);
        chk_int("g4_cs_x",  dut.gyro_x_cs, 25000);    // 4096*25000>>12  = 25000
        chk_int("g4_cs_y",  dut.gyro_y_cs, 50000);    // 8192*25000>>12  = 50000
        chk_int("g4_cs_z",  dut.gyro_z_cs, 75000);    // 12288*25000>>12 = 75000

        // ================= G5 加速度通路 =================
        force dut.rx_buf[2] = 8'h00; force dut.rx_buf[3] = 8'h10;   // acc_y = 0x1000 = 4096
        #1;
        chk_int("g5_acc_y_comb", dut.acc_y_mg, 1000);   // 4096*1000>>12 = 1000
        // 注意：不检查输出端口 acc_y —— `o_acc_y = acc_y_r` 只在 `op_done` 时更新
        // （imu_top.v:741 `acc_y_r <= acc_y_mg[15:0]`），本 tb 不跑真实 SPI 交互，
        // op_done 永不置位。检查到 acc_y_mg 这一层即可。

        release dut.rx_buf[2];  release dut.rx_buf[3];
        release dut.rx_buf[6];  release dut.rx_buf[7];
        release dut.rx_buf[8];  release dut.rx_buf[9];
        release dut.rx_buf[10]; release dut.rx_buf[11];

        $display("");
        if (errors == 0) $display("PASS: imu_top_tb %0d checks, 0 errors", checks);
        else             $display("FAIL: imu_top_tb %0d checks, %0d errors", checks, errors);
        $finish;
    end

endmodule
