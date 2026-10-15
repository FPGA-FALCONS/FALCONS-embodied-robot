//////////////////////////////////////////////////////////////////////////////////
// 模块：uart_byte_rx —— 可配置的单字节UART接收器
// 特点：两级输入同步，在每个UART位的中点采样，并检查校验位和停止位
//
// 参数：
//   CLK_FREQ_HZ  系统时钟频率(Hz)       默认50_000_000
//   BAUD_RATE    UART波特率             默认9_600
//   DATA_BITS    有效数据位数           5～8，默认8
//   PARITY_EN    校验使能               0=无校验，1=有校验
//   PARITY_TYPE  校验类型               0=偶校验，1=奇校验
//   RECOVERY_EN  启动/错误恢复           默认0；1等待1.5位高电平并用下降沿起收
//
// 端口：
//   i_clock       输入  系统时钟
//   i_reset_n     输入  异步低电平复位
//   i_uart_rx     输入  UART串行接收线
//   o_rx_data     输出  接收到的数据；无效高位补0
//   o_rx_valid    输出  接收数据有效，握手前保持为1
//   i_rx_ready    输入  下游可以接收数据
//   o_rx_error    输出  单周期错误脉冲：校验、停止位或输出溢出错误
//
// 最小示例（9600 8N1，始终立即取走数据）：
//   uart_byte_rx u_rx (
//       .i_clock(clk), .i_reset_n(reset_n), .i_uart_rx(uart_rx),
//       .o_rx_data(rx_data), .o_rx_valid(rx_valid),
//       .i_rx_ready(1'b1), .o_rx_error(rx_error)
//   );
//   // i_rx_ready固定为1，因此每当rx_valid为1，本时钟沿就完成一次握手
//   always @(posedge clk or negedge reset_n) begin
//       if (!reset_n)
//           saved_data <= 8'd0;
//       else if (rx_valid)
//           saved_data <= rx_data;               // 保存刚接收到的字节
//   end
//////////////////////////////////////////////////////////////////////////////////
module uart_byte_rx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 9_600,
    parameter integer DATA_BITS   = 8,
    parameter integer PARITY_EN   = 0,
    parameter integer PARITY_TYPE = 0,
    parameter integer RECOVERY_EN = 0
)(
    input  wire       i_clock,
    input  wire       i_reset_n,
    input  wire       i_uart_rx,

    output reg  [7:0] o_rx_data,
    output reg        o_rx_valid,
    input  wire       i_rx_ready,
    output reg        o_rx_error
);

    localparam integer CLKS_PER_BIT =
        (CLK_FREQ_HZ + BAUD_RATE / 2) / BAUD_RATE;
    localparam integer HALF_BIT_CLKS = CLKS_PER_BIT / 2;
    localparam integer RECOVERY_HIGH_CLKS = CLKS_PER_BIT + HALF_BIT_CLKS;

    localparam [2:0] IDLE   = 3'd0;
    localparam [2:0] START  = 3'd1;
    localparam [2:0] DATA   = 3'd2;
    localparam [2:0] PARITY = 3'd3;
    localparam [2:0] STOP   = 3'd4;
    localparam [2:0] RECOVER = 3'd5;

    reg rx_sync_0;
    reg rx_sync_1;
    reg rx_sync_1_d;
    reg [2:0]  state;
    reg [31:0] baud_cnt;
    reg [2:0]  bit_index;
    reg [7:0]  data_shift;
    reg        parity_bad;

    // 异步串口输入先经过两级寄存器同步，降低亚稳态风险
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            rx_sync_0 <= 1'b1;
            rx_sync_1 <= 1'b1;
            rx_sync_1_d <= 1'b1;
        end else begin
            rx_sync_0 <= i_uart_rx;
            rx_sync_1 <= rx_sync_0;
            rx_sync_1_d <= rx_sync_1;
        end
    end

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state       <= (RECOVERY_EN != 0) ? RECOVER : IDLE;
            baud_cnt    <= 32'd0;
            bit_index   <= 3'd0;
            data_shift  <= 8'd0;
            parity_bad  <= 1'b0;
            o_rx_data  <= 8'd0;
            o_rx_valid <= 1'b0;
            o_rx_error <= 1'b0;
        end else begin
            o_rx_error <= 1'b0;

            // 下游完成握手后，撤销当前输出有效标志
            if (o_rx_valid && i_rx_ready)
                o_rx_valid <= 1'b0;

            case (state)
                IDLE: begin
                    // RECOVERY_EN=1时只在下降沿起收；默认0保留低电平起收方式。
                    if (!rx_sync_1 && ((RECOVERY_EN == 0) || rx_sync_1_d)) begin
                        baud_cnt <= HALF_BIT_CLKS - 1;
                        state <= START;
                    end
                end

                START: begin
                    if (baud_cnt == 0) begin
                        if (!rx_sync_1) begin
                            baud_cnt   <= CLKS_PER_BIT - 1;
                            bit_index  <= 3'd0;
                            data_shift <= 8'd0;
                            parity_bad <= 1'b0;
                            state      <= DATA;
                        end else begin
                            // 中点已经回到高电平，说明刚才只是干扰而非真正起始位
                            state <= IDLE;
                        end
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                DATA: begin
                    if (baud_cnt == 0) begin
                        data_shift[bit_index] <= rx_sync_1;
                        baud_cnt <= CLKS_PER_BIT - 1;

                        if (bit_index == DATA_BITS - 1) begin
                            if (PARITY_EN != 0)
                                state <= PARITY;
                            else
                                state <= STOP;
                        end else
                            bit_index <= bit_index + 1'b1;
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                PARITY: begin
                    if (baud_cnt == 0) begin
                        baud_cnt <= CLKS_PER_BIT - 1;
                        if (PARITY_TYPE == 0)
                            parity_bad <= (rx_sync_1 != ^data_shift[DATA_BITS-1:0]);
                        else
                            parity_bad <= (rx_sync_1 != ~^data_shift[DATA_BITS-1:0]);
                        state <= STOP;
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                STOP: begin
                    if (baud_cnt == 0) begin
                        state <= IDLE;

                        if (!rx_sync_1 || parity_bad) begin
                            // 停止位必须为1，同时校验位必须正确
                            o_rx_error <= 1'b1;
                            if (RECOVERY_EN != 0) begin
                                state <= RECOVER;
                                baud_cnt <= 0;
                            end
                        end else if (!o_rx_valid || i_rx_ready) begin
                            o_rx_data  <= data_shift;
                            o_rx_valid <= 1'b1;
                        end else begin
                            // 上一个数据还未被取走，新数据只能丢弃
                            o_rx_error <= 1'b1;
                        end
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                RECOVER: begin
                    // 上电可能发生在连续串口流中途。错误后立即在低电平
                    // 再起收，会在某些数据模式中永久错位。等待连续 1.5 位
                    // 高电平后才重新允许下降沿起收；M10 的 A5 尾部和
                    // FA/FB 都提供此恢复机会。正常接收无需字节间空闲。
                    // break 持续为低时停在这里，只报告首次停止位错误。
                    if (rx_sync_1) begin
                        if (baud_cnt >= RECOVERY_HIGH_CLKS - 1) begin
                            state <= IDLE;
                            baud_cnt <= 0;
                        end else
                            baud_cnt <= baud_cnt + 1'b1;
                    end else
                        baud_cnt <= 0;
                end

                default: begin
                    state <= (RECOVERY_EN != 0) ? RECOVER : IDLE;
                    baud_cnt <= 0;
                end
            endcase
        end
    end

endmodule
