//////////////////////////////////////////////////////////////////////////////////
// 模块：imu_uart_top —— IMU660RA 最小上板示例顶层
//
// 功能：
//   * 按键复位（同步 + 去抖，内联实现，避免示例额外依赖其它模块）
//   * imu_top：完成 BMI270 初始化并以 1 kHz 读六轴数据
//   * uart_report：按 50 Hz 输出 CSV 文本 "gx,gy,gz,ax,ay,az\r\n"
//   * LED：8 位诊断显示
//
// 板卡：ACG720 / GW5AT-138B，系统时钟 50 MHz
// 串口：115200 8N1
//
// LED 诊断（高电平点亮）：
//   led[0] 常亮      = 初始化成功
//   led[1] 闪烁      = 初始化失败
//   led[2] 闪烁      = 片选在动（SPI 正在传输）
//   led[3] 闪烁      = SCK 在动
//   led[7:4]         = 正常时显示状态机状态；失败时显示错误码
//                      错误码 1 = CHIP_ID 校验失败（通信问题）
//                            2 = INTERNAL_STATUS 校验失败（配置问题）
//
// 依赖：模块库的 uart/rtl/uart_tx_queue.v（+ sync_fifo.v + uart_byte_tx.v）
//////////////////////////////////////////////////////////////////////////////////

module imu_uart_top #(
    parameter integer CLK_FREQ_HZ = 50_000_000
)(
    input  wire       i_clock,          // 50 MHz（Y18）
    input  wire       i_key_n,          // 按键 S0（F15），低有效

    output wire       o_imu_sck,
    output wire       o_imu_mosi,
    input  wire       i_imu_miso,
    output wire       o_imu_cs_n,

    output wire       o_uart_tx,        // M15
    output wire [7:0] o_led
);

    //---------------------------------------------------------------
    // 复位：按键同步 + 去抖（内联，示例自包含）
    //---------------------------------------------------------------
    localparam integer MS_TICKS   = (CLK_FREQ_HZ >= 1000) ? (CLK_FREQ_HZ / 1000) : 1;
    localparam integer DBNC_TICKS = MS_TICKS * 20;

    reg [1:0]  key_sync;
    reg [31:0] dbnc_cnt;
    reg        key_stable = 1'b0;      // 上电默认复位态，去抖确认后释放

    always @(posedge i_clock) begin
        key_sync <= {key_sync[0], i_key_n};
    end

    always @(posedge i_clock) begin
        if (key_sync[1] == key_stable) begin
            dbnc_cnt <= 32'd0;
        end else if (dbnc_cnt >= DBNC_TICKS) begin
            key_stable <= key_sync[1];
            dbnc_cnt   <= 32'd0;
        end else begin
            dbnc_cnt <= dbnc_cnt + 32'd1;
        end
    end

    wire reset_n = key_stable;

    //---------------------------------------------------------------
    // IMU 驱动
    //---------------------------------------------------------------
    wire        init_done;
    wire        err_flag;
    wire [7:0]  err_code;
    wire [7:0]  state_dbg;
    wire        calib_busy;
    wire        calib_done;
    wire        calib_fail;
    wire signed [31:0] gyro_x, gyro_y, gyro_z;
    wire signed [15:0] acc_x,  acc_y,  acc_z;

    imu_top #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .SCK_HALF_DIV  (25),            // SCK = 50MHz/50 = 1MHz（杜邦线建议 1~5MHz）
        .SAMPLE_DIV    (50_000),        // 1 kHz
        .PWR_WAIT_MS   (20),
        .CHK_RETRY_MAX (10),
        .CALIB_DEV_TH  (100)            // 零偏标定的抖动判据 1°/s
                                        // 若 led[7:4]=C（标定失败），说明静止时
                                        // 噪声超过该值，把它调大即可
    ) u_imu (
        .i_clock      (i_clock),
        .i_reset_n    (reset_n),
        .o_imu_sck    (o_imu_sck),
        .o_imu_mosi   (o_imu_mosi),
        .i_imu_miso   (i_imu_miso),
        .o_imu_cs_n   (o_imu_cs_n),
        .o_init_busy  (),
        .o_init_done  (init_done),
        .o_err_flag   (err_flag),
        .o_err_code   (err_code),
        .o_state_dbg  (state_dbg),
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
        .o_data_valid (),
        .o_acc_valid  ()            // 本示例不做倾斜标定，悬空即可；
                                    // 若下游要采标定窗口内的加速度样本，接这个
    );

    //---------------------------------------------------------------
    // 串口上报（50 Hz CSV）
    //---------------------------------------------------------------
    uart_report #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .BAUD_RATE       (115_200),
        .REPORT_DIV      (1_000_000),
        .FIFO_DEPTH_LOG2 (5)
    ) u_report (
        .i_clock   (i_clock),
        .i_reset_n (reset_n),
        .i_gyro_x  (gyro_x),
        .i_gyro_y  (gyro_y),
        .i_gyro_z  (gyro_z),
        .i_acc_x   (acc_x),
        .i_acc_y   (acc_y),
        .i_acc_z   (acc_z),
        .o_uart_tx (o_uart_tx)
    );

    //---------------------------------------------------------------
    // LED 诊断
    //---------------------------------------------------------------
    reg [23:0] blink_cnt;
    reg        blink;

    always @(posedge i_clock or negedge reset_n) begin
        if (!reset_n) begin
            blink_cnt <= 24'd0;
            blink     <= 1'b0;
        end else if (blink_cnt == 24'd4_999_999) begin     // 5 Hz
            blink_cnt <= 24'd0;
            blink     <= ~blink;
        end else begin
            blink_cnt <= blink_cnt + 24'd1;
        end
    end

    // led[0]：初始化 + 零偏标定都完成才常亮（上电后约 0.6s）
    assign o_led[0]   = init_done & calib_done;
    // led[1]：初始化失败或标定失败时闪烁
    assign o_led[1]   = (err_flag | calib_fail) & blink;
    assign o_led[2]   = ~o_imu_cs_n;            // 片选活动
    assign o_led[3]   = o_imu_sck;              // SCK 活动
    // led[7:4]：错误码 / 标定状态 / 状态机状态
    //   1 = CHIP_ID 失败（通信）    2 = INTERNAL_STATUS 失败（配置）
    //   B = 零偏标定中              C = 零偏标定失败
    //   其它 = 状态机状态（14=RUN，见 imu_top 文件头对照表）
    assign o_led[7:4] = err_flag   ? err_code[3:0]
                      : calib_fail ? 4'hC
                      : calib_busy ? 4'hB
                      : state_dbg[3:0];

endmodule
