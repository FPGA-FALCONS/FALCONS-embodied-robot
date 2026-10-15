//////////////////////////////////////////////////////////////////////////////////
// 模块：imu_top —— IMU660RA(BMI270) 驱动模块
//
// 功能：
//   1) 上电后自动完成 BMI270 初始化：读 CHIP_ID → 关省电 → 灌 8192 字节
//      config file → 校验 INTERNAL_STATUS → 配置量程/ODR → 使能传感器
//   2) 初始化完成后执行开机零偏标定（可用 CALIB_EN 关闭）：
//      按采样节拍连续读陀螺，用"与均值的偏差"判据挑选静止样本
//      （阶段 A 前 16 个样本无条件建立均值，阶段 B 偏差在阈值内才计入），
//      收满 CALIB_SAMPLES 个有效样本后求均值作为零偏；
//      超时（CALIB_TIMEOUT_MS）则判为失败，零偏保持 0
//   3) 标定完成后以固定周期 burst 读 12 字节（加速度 6 + 陀螺 6）
//   4) 原始值换算为物理量输出，并按 CALIB_APPLY 扣除零偏
//
// 为什么要用"偏差判据"而不是"绝对值阈值"：
//   这类模块的陀螺零偏典型可达 ±3~6°/s（三轴合计可超 12°/s）。用绝对阈值
//   要么把正常零偏拦掉，要么阈值放宽到会放行缓慢转动。偏差判据只看数据稳
//   不稳，与零偏大小无关，更可靠。
//   标定失败时宁可退化为"未标定"（bias=0），也不能写进一个错误值。
//
// 初始化序列（依据逐飞 zf_device_imu660ra.c:214-338，顺序做了安全性调整：
// 先配量程/ODR，最后再写 PWR_CTRL 使能）：
//   上电等 20ms → RD 0x00(CHIP_ID 应=0x24) → WR 0x7C=0x00(关高级省电)
//   → 等 1ms → WR 0x59=0x00(配置开始) → BURST 写 0x5E + 8192 字节
//   → WR 0x59=0x01(配置结束) → 等 20ms → RD 0x21(应=0x01)
//   → WR 0x41=0x02(±8g) → WR 0x40=0xA7(ACC 50Hz) → WR 0x43=0x00(±2000dps)
//   → WR 0x42=0xA9(GYR 200Hz) → WR 0x7D=0x0E(使能 acc/gyr/temp) → RUN
//
// SPI 时序要点（BMI270）：
//   * Mode0，MSB first
//   * 读寄存器：地址字节须置读位 bit7=1，其后紧跟 1 个 dummy 字节，再是数据
//   * 连续写：地址后连续写数据，片选全程保持低（8KB 配置约 6.6ms 不抬 CS）
//
// 输出数据格式（模式A：寄存器直出 + 单周期 valid）：
//   o_gyro_x/y/z  int32，单位 0.01°/s（±2000dps 满量程需 ±200000，故用 32 位）
//   o_acc_x/y/z   int16，单位 mg（±8g 满量程为 ±8000，16 位足够）
//
// 参数：
//   CLK_FREQ_HZ    系统时钟频率(Hz)              默认 50_000_000
//   SCK_HALF_DIV   SCK 半周期时钟数              默认 3 → 8.33MHz
//   SAMPLE_DIV     采样周期时钟数                默认 50_000 → 1kHz
//   PWR_WAIT_MS    上电等待(ms)                  默认 20
//   CHK_RETRY_MAX  CHIP_ID 读取最大重试次数      默认 10
//
// 端口：
//   i_clock        输入  系统时钟
//   i_reset_n      输入  异步低电平复位
//   o_imu_sck      输出  SPI 时钟
//   o_imu_mosi     输出  SPI 主机输出
//   i_imu_miso     输入  SPI 主机输入（接模块 SA0/SDO）
//   o_imu_cs_n     输出  SPI 片选，低有效，空闲为高
//   o_init_busy    输出  初始化进行中
//   o_init_done    输出  初始化完成，进入周期采样
//   o_err_flag     输出  初始化失败（需复位重来）
//   o_state_dbg    输出  当前状态机状态（接 GAO/LED 调试）
//   o_calib_busy   输出  零偏标定进行中
//   o_calib_done   输出  标定完成，零偏有效（此时 CALIB_APPLY 才开始生效）
//   o_calib_fail   输出  标定失败（未静止/超时），零偏保持 0
//   o_bias_x/y/z   输出  标定得到的零偏，0.01°/s（供观察/保存）
//   o_calib_busy   输出  零偏标定进行中
//   o_calib_done   输出  标定完成，零偏有效（此时 CALIB_APPLY 才开始生效）
//   o_calib_fail   输出  标定失败（未静止/超时），零偏保持 0
//   o_bias_x/y/z   输出  标定得到的零偏，0.01°/s（供观察/保存）
//   o_gyro_x/y/z   输出  角速度，0.01°/s
//   o_acc_x/y/z    输出  加速度，mg
//   o_data_valid   输出  单周期脉冲：本拍输出为新一帧数据（仅 RUN 态产生）
//   o_acc_valid    输出  单周期脉冲：本拍 acc_x/y/z 为新值（标定态与 RUN 态都产生）
//                        供 gyro_tilt_comp 做倾斜标定使用；标定态 gyro 输出尚未
//                        更新，故不能复用 o_data_valid
//////////////////////////////////////////////////////////////////////////////////

module imu_top #(
    parameter integer CLK_FREQ_HZ   = 50_000_000,
    parameter integer SCK_HALF_DIV  = 3,
    parameter integer SAMPLE_DIV    = 50_000,
    parameter integer PWR_WAIT_MS   = 20,
    parameter integer CHK_RETRY_MAX = 10,
    // ---- 零偏标定 ----
    parameter integer CALIB_EN         = 1,      // 1 = 初始化后执行开机零偏标定
    parameter integer CALIB_APPLY      = 1,      // 1 = 输出角速度中扣除零偏
    parameter integer CALIB_SAMPLES    = 512,    // 需要的有效样本数（须为 2 的幂，便于移位求均值）
    parameter integer CALIB_TIMEOUT_MS = 3000,   // 标定超时(ms)，超时判为失败
    parameter integer CALIB_DEV_TH     = 100,    // 抖动判据阈值(0.01°/s)，100 = 1°/s
                                                 // 取值参考：BMI270 陀螺噪声密度约
                                                 // 0.008°/s/√Hz，200Hz ODR 下短时噪声
                                                 // 可达 0.1~0.5°/s；阈值过小会把静止
                                                 // 样本全部拒掉导致标定超时失败
    parameter integer CALIB_ABS_LIMIT  = 2000    // 绝对门禁：三轴 |读数| 之和上限
                                                 // 2000 = 20°/s。零偏的物理范围是有界的
                                                 // （这类模块典型 ±3~6°/s，最坏 ±20°/s），
                                                 // 而真实运动可以任意大，故用绝对门禁拦住
                                                 // "匀速转动"——纯抖动判据对此完全无效
                                                 // （匀速转动时抖动为 0，会被误判为静止）
                                                 // 设置为 0 可关闭该门禁
)(
    input  wire        i_clock,
    input  wire        i_reset_n,

    // ---- IMU660RA 物理接口 ----
    output wire        o_imu_sck,
    output wire        o_imu_mosi,
    input  wire        i_imu_miso,
    output wire        o_imu_cs_n,

    // ---- 状态输出 ----
    output wire        o_init_busy,
    output wire        o_init_done,
    output wire        o_err_flag,
    output wire [7:0]  o_err_code,      // 失败原因：1=CHIP_ID 校验失败（通信问题）
                                        //           2=INTERNAL_STATUS 校验失败（配置问题）
    output wire [7:0]  o_state_dbg,

    // ---- 零偏标定 ----
    output wire        o_calib_busy,    // 标定进行中
    output wire        o_calib_done,    // 标定完成，bias 有效
    output wire        o_calib_fail,    // 标定失败（未满足静止条件或超时，bias 保持 0）
    output wire signed [31:0] o_bias_x, // 标定得到的零偏，0.01°/s
    output wire signed [31:0] o_bias_y,
    output wire signed [31:0] o_bias_z,

    // ---- 数据输出（模式A）----
    output wire signed [31:0] o_gyro_x,
    output wire signed [31:0] o_gyro_y,
    output wire signed [31:0] o_gyro_z,
    output wire signed [15:0] o_acc_x,
    output wire signed [15:0] o_acc_y,
    output wire signed [15:0] o_acc_z,
    output wire               o_data_valid,
    output wire               o_acc_valid
);

    localparam integer MS_TICKS = (CLK_FREQ_HZ >= 1000) ? (CLK_FREQ_HZ / 1000) : 1;

    // 命令类型
    localparam [1:0] C_WR1   = 2'd0;    // 写 1 个寄存器：地址 + 数据（2 字节）
    localparam [1:0] C_RDN   = 2'd1;    // 读 N 字节：地址|0x80 + dummy + N（2+N 字节）
    localparam [1:0] C_BURST = 2'd2;    // 连续写：地址 + N 字节数据（1+N 字节）

    // 状态编码（同时用于 o_state_dbg）
    localparam [7:0] S_PWR_WAIT     = 8'd0;
    localparam [7:0] S_CHK_ID       = 8'd1;
    localparam [7:0] S_WR_PWR_CONF  = 8'd2;
    localparam [7:0] S_DLY_1MS      = 8'd3;
    localparam [7:0] S_WR_INIT_0    = 8'd4;
    localparam [7:0] S_BURST_INIT   = 8'd5;
    localparam [7:0] S_WR_INIT_1    = 8'd6;
    localparam [7:0] S_DLY_20MS     = 8'd7;
    localparam [7:0] S_CHK_STATUS   = 8'd8;
    localparam [7:0] S_CFG_ACC_RNG  = 8'd9;
    localparam [7:0] S_CFG_ACC_CFG  = 8'd10;
    localparam [7:0] S_CFG_GYR_RNG  = 8'd11;
    localparam [7:0] S_CFG_GYR_CFG  = 8'd12;
    localparam [7:0] S_WR_PWR_CTRL  = 8'd13;
    localparam [7:0] S_RUN          = 8'd14;
    localparam [7:0] S_ERROR        = 8'd15;
    localparam [7:0] S_CALIB        = 8'd16;    // 零偏标定（单独编码，不占用 0~15，
                                                // 以便 LED 用 0xB 单独表示"标定中"）

    // 标定相关常量
    localparam integer CALIB_SEED_N     = 16;   // 建立初值用的样本数
    localparam integer CALIB_SAMPLES_LG = 9;    // log2(CALIB_SAMPLES)
    localparam integer CALIB_TIMEOUT_TK = CALIB_TIMEOUT_MS;  // 按采样节拍计（1kHz 时即毫秒数）

    reg [7:0]  state;
    reg [7:0]  retry_cnt;

    //---------------------------------------------------------------
    // SPI 引擎
    //---------------------------------------------------------------
    reg         spi_start_r;
    reg  [1:0]  cmd_type;
    reg  [7:0]  cmd_addr;
    reg  [7:0]  cmd_data;
    reg  [15:0] cmd_len;

    wire [15:0] spi_byte_idx;
    wire [7:0]  spi_tx_byte;
    wire [7:0]  spi_rx_byte;
    wire [15:0] spi_rx_idx;
    wire        spi_rx_valid;
    wire        spi_busy;
    wire        spi_done;
    wire [7:0]  rom_data;

    // 发送字节：由上层按当前字节索引组合给出
    reg [7:0] spi_tx_byte_c;
    always @(*) begin
        case (cmd_type)
            C_WR1  : spi_tx_byte_c = (spi_byte_idx == 16'd0) ? cmd_addr : cmd_data;
            C_BURST: spi_tx_byte_c = (spi_byte_idx == 16'd0) ? cmd_addr : rom_data;
            default: spi_tx_byte_c = (spi_byte_idx == 16'd0) ? {1'b1, cmd_addr[6:0]} : 8'h00;
        endcase
    end
    assign spi_tx_byte = spi_tx_byte_c;

    spi_master_mode0 #(
        .SCK_HALF_DIV (SCK_HALF_DIV)
    ) u_spi (
        .i_clock    (i_clock),
        .i_reset_n  (i_reset_n),
        .i_start    (spi_start_r),
        .i_byte_num (cmd_len),
        .o_byte_idx (spi_byte_idx),
        .i_tx_byte  (spi_tx_byte),
        .o_rx_byte  (spi_rx_byte),
        .o_rx_idx   (spi_rx_idx),
        .o_rx_valid (spi_rx_valid),
        .o_busy     (spi_busy),
        .o_done     (spi_done),
        .o_sck      (o_imu_sck),
        .o_mosi     (o_imu_mosi),
        .i_miso     (i_imu_miso),
        .o_cs_n     (o_imu_cs_n)
    );

    //---------------------------------------------------------------
    // 8KB 配置 ROM（地址 = 字节索引 - 1）
    //---------------------------------------------------------------
    wire [12:0] rom_addr = spi_byte_idx[12:0] - 13'd1;

    bmi270_cfg_rom u_cfg_rom (
        .i_clock (i_clock),
        .i_addr  (rom_addr),
        .o_data  (rom_data)
    );

    //---------------------------------------------------------------
    // 接收缓冲：读命令时，丢弃前 2 字节（地址 + dummy），收集数据
    //---------------------------------------------------------------
    reg [7:0] rx_buf [0:15];
    integer   bi;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            for (bi = 0; bi < 16; bi = bi + 1) begin
                rx_buf[bi] <= 8'd0;
            end
        end else if (spi_rx_valid) begin
            if ((cmd_type == C_RDN) && (spi_rx_idx >= 16'd2)) begin
                rx_buf[spi_rx_idx[3:0] - 4'd2] <= spi_rx_byte;
            end
        end
    end

    //---------------------------------------------------------------
    // 命令执行辅助：op_fired 防止同一状态重复发起，op_busy 表示传输进行中
    //---------------------------------------------------------------
    reg  op_fired;
    reg  op_busy;
    wire op_done = spi_done;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            op_fired <= 1'b0;
            op_busy  <= 1'b0;
        end else begin
            if (spi_start_r)   op_busy  <= 1'b1;
            else if (op_done)  op_busy  <= 1'b0;

            if (op_done)       op_fired <= 1'b0;
            else if (spi_start_r) op_fired <= 1'b1;
        end
    end

    //---------------------------------------------------------------
    // 延时计数器（初始化各步骤之间的等待）
    //---------------------------------------------------------------
    reg  [31:0] dly_cnt;
    reg  [31:0] dly_target;
    reg         dly_load;
    reg         dly_armed;
    wire        dly_done = (dly_cnt == 32'd0);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            dly_cnt <= 32'd0;
        end else if (dly_load) begin
            dly_cnt <= dly_target;
        end else if (dly_cnt != 32'd0) begin
            dly_cnt <= dly_cnt - 32'd1;
        end
    end

    //---------------------------------------------------------------
    // 采样节拍（RUN 与 CALIB 两个状态都需要按节拍读数据）
    //---------------------------------------------------------------
    reg  [31:0] sample_cnt;
    wire        sample_tick = ((state == S_RUN) || (state == S_CALIB)) &&
                              (sample_cnt == 32'd0);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            sample_cnt <= 32'd0;
        end else if ((state != S_RUN) && (state != S_CALIB)) begin
            sample_cnt <= 32'd0;
        end else if (sample_cnt == 32'd0) begin
            sample_cnt <= SAMPLE_DIV;
        end else begin
            sample_cnt <= sample_cnt - 32'd1;
        end
    end

    //---------------------------------------------------------------
    // 原始值拼装与单位换算
    //   acc  ±8g     : mg      = raw * 8000  / 32768 = raw * 1000  / 4096
    //   gyro ±2000dps: 0.01°/s = raw * 200000/ 32768 = raw * 25000 / 4096
    //   （乘数带符号扩展到 32 位，最大 32767*25000 ≈ 8.2e8，32 位内不溢出）
    //---------------------------------------------------------------
    wire signed [15:0] acc_x_raw  = {rx_buf[1],  rx_buf[0]};
    wire signed [15:0] acc_y_raw  = {rx_buf[3],  rx_buf[2]};
    wire signed [15:0] acc_z_raw  = {rx_buf[5],  rx_buf[4]};
    wire signed [15:0] gyro_x_raw = {rx_buf[7],  rx_buf[6]};
    wire signed [15:0] gyro_y_raw = {rx_buf[9],  rx_buf[8]};
    wire signed [15:0] gyro_z_raw = {rx_buf[11], rx_buf[10]};

    wire signed [31:0] acc_x_mg  = ($signed(acc_x_raw)  * 32'sd1000)  >>> 12;
    wire signed [31:0] acc_y_mg  = ($signed(acc_y_raw)  * 32'sd1000)  >>> 12;
    wire signed [31:0] acc_z_mg  = ($signed(acc_z_raw)  * 32'sd1000)  >>> 12;
    wire signed [31:0] gyro_x_cs = ($signed(gyro_x_raw) * 32'sd25000) >>> 12;
    wire signed [31:0] gyro_y_cs = ($signed(gyro_y_raw) * 32'sd25000) >>> 12;
    wire signed [31:0] gyro_z_cs = ($signed(gyro_z_raw) * 32'sd25000) >>> 12;

    //---------------------------------------------------------------
    // 零偏标定
    //
    // 流程：初始化完成后进入 S_CALIB，按采样节拍连续读取陀螺数据。
    //   阶段 A（前 CALIB_SEED_N 个样本）：无条件累加，用于建立均值初值。
    //   阶段 B：用"与均值的偏差"作为静止判据——
    //           |sample - mean| 三轴之和 < CALIB_DEV_TH 才计入。
    //   收满 CALIB_SAMPLES 个有效样本 → bias = 累加和 >> log2(CALIB_SAMPLES)。
    //   超过 CALIB_TIMEOUT_MS 仍未收满 → 判为失败，bias 保持 0。
    //
    // 为什么用"偏差判据"而不是逐飞那种"绝对值之和 < 阈值"：
    //   这类模块的陀螺零偏典型可达 ±3~6 °/s（三轴合计可超 12 °/s），
    //   用绝对阈值要么拦掉正常零偏、要么放行缓慢转动。
    //   偏差判据只看"稳不稳"，与零偏大小无关，更可靠。
    //---------------------------------------------------------------
    reg  [31:0]       calib_cnt;                        // 已收集的有效样本数
    reg  [31:0]       calib_tick;                       // 超时计数（按采样节拍）
    reg  signed [47:0] calib_sum_x, calib_sum_y, calib_sum_z;
    reg  signed [31:0] calib_mean_x, calib_mean_y, calib_mean_z;
    reg               calib_seed_done;
    reg               calib_done_r;
    reg               calib_fail_r;
    reg  signed [31:0] bias_x_r, bias_y_r, bias_z_r;

    // 与均值的偏差（取绝对值）
    wire signed [31:0] dev_x = gyro_x_cs - calib_mean_x;
    wire signed [31:0] dev_y = gyro_y_cs - calib_mean_y;
    wire signed [31:0] dev_z = gyro_z_cs - calib_mean_z;
    wire        [31:0] adev_x = dev_x[31] ? (~dev_x + 32'sd1) : dev_x;
    wire        [31:0] adev_y = dev_y[31] ? (~dev_y + 32'sd1) : dev_y;
    wire        [31:0] adev_z = dev_z[31] ? (~dev_z + 32'sd1) : dev_z;

    // 读数的绝对值（用于绝对门禁）
    wire        [31:0] abs_gx = gyro_x_cs[31] ? (~gyro_x_cs + 32'sd1) : gyro_x_cs;
    wire        [31:0] abs_gy = gyro_y_cs[31] ? (~gyro_y_cs + 32'sd1) : gyro_y_cs;
    wire        [31:0] abs_gz = gyro_z_cs[31] ? (~gyro_z_cs + 32'sd1) : gyro_z_cs;

    // 绝对门禁：三轴读数之和必须落在零偏的合理范围内。
    // 这一步专门拦"匀速转动"——它抖动为 0，纯偏差判据看不出任何异常。
    // CALIB_ABS_LIMIT=0 时关闭该门禁。
    wire               calib_in_range = (CALIB_ABS_LIMIT == 0) ||
                                        ((abs_gx + abs_gy + abs_gz) < CALIB_ABS_LIMIT[31:0]);

    // 静止判据 = 绝对门禁 AND 抖动判据
    wire calib_still = calib_in_range &&
                       ((adev_x + adev_y + adev_z) < CALIB_DEV_TH[31:0]);
    wire calib_timeout = (calib_tick >= CALIB_TIMEOUT_TK[31:0]);

    // 本拍是否计入样本（阶段 A 无条件；阶段 B 需满足抖动判据）
    wire        calib_accum    = (!calib_seed_done) || calib_still;
    wire [31:0] calib_cnt_next = calib_cnt + 32'd1;

    // 累加和的"下一拍值"（供求均值/零偏时使用，避免表达式位宽截断警告）
    wire signed [47:0] seed_sum_x = calib_sum_x + gyro_x_cs;
    wire signed [47:0] seed_sum_y = calib_sum_y + gyro_y_cs;
    wire signed [47:0] seed_sum_z = calib_sum_z + gyro_z_cs;

    // 标定超时计数（按采样节拍计；1 kHz 采样时即毫秒数）
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)              calib_tick <= 32'd0;
        else if (state != S_CALIB)   calib_tick <= 32'd0;
        else if (sample_tick)        calib_tick <= calib_tick + 32'd1;
    end

    assign o_calib_busy = (state == S_CALIB);
    assign o_calib_done = calib_done_r;
    assign o_calib_fail = calib_fail_r;
    assign o_bias_x     = bias_x_r;
    assign o_bias_y     = bias_y_r;
    assign o_bias_z     = bias_z_r;

    //---------------------------------------------------------------
    // 输出寄存器
    //---------------------------------------------------------------
    reg signed [31:0] gyro_x_r, gyro_y_r, gyro_z_r;
    reg signed [15:0] acc_x_r,  acc_y_r,  acc_z_r;
    reg               data_valid_r;
    reg               acc_valid_r;
    reg               init_done_r;
    reg               err_flag_r;
    reg [7:0]         err_code_r;      // 失败原因：1=CHIP_ID 校验失败，2=INTERNAL_STATUS 校验失败

    assign o_gyro_x     = gyro_x_r;
    assign o_gyro_y     = gyro_y_r;
    assign o_gyro_z     = gyro_z_r;
    assign o_acc_x      = acc_x_r;
    assign o_acc_y      = acc_y_r;
    assign o_acc_z      = acc_z_r;
    assign o_data_valid = data_valid_r;
    assign o_acc_valid  = acc_valid_r;
    assign o_init_done  = init_done_r;
    assign o_err_flag   = err_flag_r;
    assign o_err_code   = err_code_r;
    assign o_init_busy  = ~init_done_r & ~err_flag_r;
    assign o_state_dbg  = state;

    //---------------------------------------------------------------
    // 主状态机
    //---------------------------------------------------------------
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state        <= S_PWR_WAIT;
            retry_cnt    <= 8'd0;
            spi_start_r  <= 1'b0;
            dly_load     <= 1'b0;
            dly_armed    <= 1'b0;
            dly_target   <= 32'd0;
            cmd_type     <= C_RDN;
            cmd_addr     <= 8'h00;
            cmd_data     <= 8'h00;
            cmd_len      <= 16'd3;
            data_valid_r <= 1'b0;
            acc_valid_r  <= 1'b0;
            init_done_r  <= 1'b0;
            err_flag_r   <= 1'b0;
            err_code_r   <= 8'd0;
            calib_cnt    <= 32'd0;
            // calib_tick 由独立的计数块负责复位与递增（见"标定超时计数"），
            // 此处不能再赋值，否则构成多驱动（Gowin 报 EX2000）
            calib_sum_x  <= 48'sd0;
            calib_sum_y  <= 48'sd0;
            calib_sum_z  <= 48'sd0;
            calib_mean_x <= 32'sd0;
            calib_mean_y <= 32'sd0;
            calib_mean_z <= 32'sd0;
            calib_seed_done <= 1'b0;
            calib_done_r <= 1'b0;
            calib_fail_r <= 1'b0;
            bias_x_r     <= 32'sd0;
            bias_y_r     <= 32'sd0;
            bias_z_r     <= 32'sd0;
            gyro_x_r <= 32'sd0;
            gyro_y_r <= 32'sd0;
            gyro_z_r <= 32'sd0;
            acc_x_r  <= 16'sd0;
            acc_y_r  <= 16'sd0;
            acc_z_r  <= 16'sd0;
        end else begin
            spi_start_r  <= 1'b0;       // 单周期脉冲
            dly_load     <= 1'b0;
            data_valid_r <= 1'b0;
            acc_valid_r  <= 1'b0;

            case (state)
                //-----------------------------------------------
                S_PWR_WAIT: begin
                    if (!dly_armed) begin
                        dly_target <= PWR_WAIT_MS * MS_TICKS;
                        dly_load   <= 1'b1;
                        dly_armed  <= 1'b1;
                    end else if (dly_done) begin
                        dly_armed <= 1'b0;
                        state     <= S_CHK_ID;
                        retry_cnt <= 8'd0;
                    end
                end
                //-----------------------------------------------
                // 读 CHIP_ID(0x00)，期望 0x24；本操作同时把芯片切到 SPI 模式
                S_CHK_ID: begin
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_RDN;
                        cmd_addr    <= 8'h00;
                        cmd_len     <= 16'd3;
                    end else if (op_done) begin
                        if (rx_buf[0] == 8'h24) begin
                            state <= S_WR_PWR_CONF;
                        end else if (retry_cnt >= CHK_RETRY_MAX) begin
                            state      <= S_ERROR;
                            err_flag_r <= 1'b1;
                            err_code_r <= 8'd1;         // CHIP_ID 校验失败（通信问题）
                        end else begin
                            retry_cnt <= retry_cnt + 8'd1;
                        end
                    end
                end
                //-----------------------------------------------
                S_WR_PWR_CONF: begin                 // 关高级省电 0x7C = 0x00
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h7C;
                        cmd_data    <= 8'h00;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_DLY_1MS;
                    end
                end
                //-----------------------------------------------
                S_DLY_1MS: begin
                    if (!dly_armed) begin
                        dly_target <= MS_TICKS;
                        dly_load   <= 1'b1;
                        dly_armed  <= 1'b1;
                    end else if (dly_done) begin
                        dly_armed <= 1'b0;
                        state     <= S_WR_INIT_0;
                    end
                end
                //-----------------------------------------------
                S_WR_INIT_0: begin                   // 配置开始 0x59 = 0x00
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h59;
                        cmd_data    <= 8'h00;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_BURST_INIT;
                    end
                end
                //-----------------------------------------------
                S_BURST_INIT: begin                  // 连续写 0x5E + 8192 字节
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_BURST;
                        cmd_addr    <= 8'h5E;
                        cmd_len     <= 16'd8193;     // 地址 + 8192 字节
                    end else if (op_done) begin
                        state <= S_WR_INIT_1;
                    end
                end
                //-----------------------------------------------
                S_WR_INIT_1: begin                   // 配置结束 0x59 = 0x01
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h59;
                        cmd_data    <= 8'h01;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_DLY_20MS;
                    end
                end
                //-----------------------------------------------
                S_DLY_20MS: begin
                    if (!dly_armed) begin
                        dly_target <= 20 * MS_TICKS;
                        dly_load   <= 1'b1;
                        dly_armed  <= 1'b1;
                    end else if (dly_done) begin
                        dly_armed <= 1'b0;
                        state     <= S_CHK_STATUS;
                    end
                end
                //-----------------------------------------------
                S_CHK_STATUS: begin                  // 校验 0x21 应=0x01
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_RDN;
                        cmd_addr    <= 8'h21;
                        cmd_len     <= 16'd3;
                    end else if (op_done) begin
                        if (rx_buf[0] == 8'h01) begin
                            state <= S_CFG_ACC_RNG;
                        end else begin
                            state      <= S_ERROR;
                            err_flag_r <= 1'b1;
                            err_code_r <= 8'd2;         // INTERNAL_STATUS 校验失败（配置问题）
                        end
                    end
                end
                //-----------------------------------------------
                S_CFG_ACC_RNG: begin                 // 加速度量程 ±8g
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h41;
                        cmd_data    <= 8'h02;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_CFG_ACC_CFG;
                    end
                end
                //-----------------------------------------------
                S_CFG_ACC_CFG: begin                 // 加速度 50Hz，性能模式
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h40;
                        cmd_data    <= 8'hA7;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_CFG_GYR_RNG;
                    end
                end
                //-----------------------------------------------
                S_CFG_GYR_RNG: begin                 // 陀螺量程 ±2000dps
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h43;
                        cmd_data    <= 8'h00;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_CFG_GYR_CFG;
                    end
                end
                //-----------------------------------------------
                S_CFG_GYR_CFG: begin                 // 陀螺 200Hz，性能模式
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h42;
                        cmd_data    <= 8'hA9;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        state <= S_WR_PWR_CTRL;
                    end
                end
                //-----------------------------------------------
                S_WR_PWR_CTRL: begin                 // 使能 acc/gyr/temp
                    if (!op_fired && !op_busy) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_WR1;
                        cmd_addr    <= 8'h7D;
                        cmd_data    <= 8'h0E;
                        cmd_len     <= 16'd2;
                    end else if (op_done) begin
                        init_done_r <= 1'b1;
                        if (CALIB_EN != 0) begin
                            // 先做零偏标定，再进入周期采样
                            // 注意：calib_tick 由下方独立的计数块维护（离开 S_CALIB
                            // 时自动清零、在 S_CALIB 内按采样节拍递增），此处不可
                            // 再赋值，否则构成多驱动：ModelSim 容忍，Gowin 报 EX2000。
                            state           <= S_CALIB;
                            calib_cnt       <= 32'd0;
                            calib_sum_x     <= 48'sd0;
                            calib_sum_y     <= 48'sd0;
                            calib_sum_z     <= 48'sd0;
                            calib_seed_done <= 1'b0;
                            calib_done_r    <= 1'b0;
                            calib_fail_r    <= 1'b0;
                            bias_x_r        <= 32'sd0;
                            bias_y_r        <= 32'sd0;
                            bias_z_r        <= 32'sd0;
                        end else begin
                            state <= S_RUN;
                        end
                    end
                end
                //-----------------------------------------------
                // 零偏标定：连续读取陀螺，满足静止判据的样本才计入
                S_CALIB: begin
                    if (!op_fired && !op_busy && sample_tick) begin
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_RDN;
                        cmd_addr    <= 8'h0C;
                        cmd_len     <= 16'd14;
                    end else if (op_done) begin
                        //---- 累加有效样本 ----
                        if (calib_accum) begin
                            calib_sum_x <= calib_sum_x + gyro_x_cs;
                            calib_sum_y <= calib_sum_y + gyro_y_cs;
                            calib_sum_z <= calib_sum_z + gyro_z_cs;
                            calib_cnt   <= calib_cnt_next;
                        end

                        //---- 输出本帧数据 ----
                        // 标定期间读的同样是 0x0C 起 12 字节（acc 6 + gyro 6），
                        // 加速度数据完全有效，故更新 acc 并给出 acc_valid：
                        // 下游 gyro_tilt_comp 的倾斜标定就是靠它采加速度计的，
                        // 若这里不产生脉冲，倾斜增益会永远停在 1.0。
                        //
                        // 注意这里【不】置 data_valid：标定期 gyro 输出并未更新
                        // （仍是复位值 0），给出 data_valid 会让下游把无效的角速度
                        // 当成有效数据，也会让 imu_top_tb 的"扣除零偏后≈0"检查
                        // 退化成"0≈0"。两个信号语义不同，必须分开。
                        acc_x_r     <= acc_x_mg[15:0];
                        acc_y_r     <= acc_y_mg[15:0];
                        acc_z_r     <= acc_z_mg[15:0];
                        acc_valid_r <= 1'b1;

                        //---- 阶段 A 结束：建立均值初值 ----
                        // 用位选取移位结果的低 32 位，避免综合器报"表达式位宽截断"
                        // （>>>4 得 44 位 → 取 [35:4]；>>>9 得 39 位 → 取 [40:9]）
                        if ((!calib_seed_done) && (calib_cnt_next == CALIB_SEED_N)) begin
                            calib_mean_x    <= seed_sum_x[35:4];
                            calib_mean_y    <= seed_sum_y[35:4];
                            calib_mean_z    <= seed_sum_z[35:4];
                            calib_seed_done <= 1'b1;
                        end

                        //---- 收满样本：计算零偏 ----
                        if (calib_accum && (calib_cnt_next >= CALIB_SAMPLES)) begin
                            bias_x_r     <= seed_sum_x[40:9];
                            bias_y_r     <= seed_sum_y[40:9];
                            bias_z_r     <= seed_sum_z[40:9];
                            calib_done_r <= 1'b1;
                            state        <= S_RUN;
                        end
                        //---- 超时：判为失败，零偏保持 0 ----
                        else if (calib_timeout) begin
                            calib_fail_r <= 1'b1;
                            state        <= S_RUN;
                        end
                    end
                end
                //-----------------------------------------------
                S_RUN: begin
                    if (!op_fired && !op_busy && sample_tick) begin
                        // 读 0x0C 起 12 字节：加速度 6 + 陀螺 6（连续区）
                        spi_start_r <= 1'b1;
                        cmd_type    <= C_RDN;
                        cmd_addr    <= 8'h0C;
                        cmd_len     <= 16'd14;       // 地址 + dummy + 12
                    end else if (op_done) begin
                        // 角速度：按参数决定是否扣除标定得到的零偏
                        if ((CALIB_APPLY != 0) && calib_done_r) begin
                            gyro_x_r <= gyro_x_cs - bias_x_r;
                            gyro_y_r <= gyro_y_cs - bias_y_r;
                            gyro_z_r <= gyro_z_cs - bias_z_r;
                        end else begin
                            gyro_x_r <= gyro_x_cs;
                            gyro_y_r <= gyro_y_cs;
                            gyro_z_r <= gyro_z_cs;
                        end
                        acc_x_r      <= acc_x_mg[15:0];
                        acc_y_r      <= acc_y_mg[15:0];
                        acc_z_r      <= acc_z_mg[15:0];
                        data_valid_r <= 1'b1;
                        acc_valid_r  <= 1'b1;
                    end
                end
                //-----------------------------------------------
                S_ERROR: begin
                    err_flag_r <= 1'b1;
                end
                //-----------------------------------------------
                default: state <= S_ERROR;
            endcase
        end
    end

endmodule
