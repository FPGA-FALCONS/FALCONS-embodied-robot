//////////////////////////////////////////////////////////////////////////////////
// 模块：uart_rx_queue —— 带FIFO缓存的UART接收队列
// UART收到的正确数据自动进入FIFO，用户通过valid/ready按顺序读取。
//
// 参数：
//   CLK_FREQ_HZ      系统时钟频率(Hz)
//   BAUD_RATE        UART波特率
//   DATA_BITS        有效数据位数，建议5～8
//   PARITY_EN        0=无校验，1=有校验
//   PARITY_TYPE      0=偶校验，1=奇校验
//   FIFO_DEPTH_LOG2  FIFO地址位宽；4表示可缓存16字节
//
// 端口：
//   i_clock       输入  系统时钟
//   i_reset_n     输入  异步低电平复位
//   i_uart_rx     输入  UART串行接收线
//   o_rx_data     输出  当前队首数据
//   o_rx_valid    输出  队首数据有效，握手前保持为1
//   i_rx_ready    输入  用户准备取走数据
//   o_rx_error    输出  单周期错误脉冲：校验、停止位或缓冲溢出错误
//
// 最小示例（115200 8N1、16字节队列）：
//   uart_rx_queue #(.BAUD_RATE(115_200), .FIFO_DEPTH_LOG2(4)) u_rxq (
//       .i_clock(clk), .i_reset_n(reset_n), .i_uart_rx(uart_rx),
//       .o_rx_data(rx_data), .o_rx_valid(rx_valid),
//       .i_rx_ready(rx_ready), .o_rx_error(rx_error)
//   );
//   // 需要一个字节时拉高ready；valid与ready同时为1即完成读取
//   always @(posedge clk or negedge reset_n) begin
//       if (!reset_n) begin
//           rx_ready  <= 1'b0;
//           saved_data <= 8'd0;
//       end else begin
//           rx_ready <= read_request;
//           if (rx_valid && rx_ready)
//               saved_data <= rx_data;           // 数据已从接收FIFO取出
//       end
//   end
//////////////////////////////////////////////////////////////////////////////////
module uart_rx_queue #(
    parameter integer CLK_FREQ_HZ     = 50_000_000,
    parameter integer BAUD_RATE       = 9_600,
    parameter integer DATA_BITS       = 8,
    parameter integer PARITY_EN       = 0,
    parameter integer PARITY_TYPE     = 0,
    parameter integer FIFO_DEPTH_LOG2 = 4
)(
    input  wire       i_clock,
    input  wire       i_reset_n,
    input  wire       i_uart_rx,
    output wire [7:0] o_rx_data,
    output wire       o_rx_valid,
    input  wire       i_rx_ready,
    output wire       o_rx_error
);

    wire [7:0] rx_data;
    wire rx_valid;
    wire rx_ready;
    wire rx_error;

    uart_byte_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE),
        .DATA_BITS(DATA_BITS), .PARITY_EN(PARITY_EN),
        .PARITY_TYPE(PARITY_TYPE)
    ) receiver (
        .i_clock(i_clock), .i_reset_n(i_reset_n), .i_uart_rx(i_uart_rx),
        .o_rx_data(rx_data), .o_rx_valid(rx_valid),
        .i_rx_ready(rx_ready), .o_rx_error(rx_error)
    );

    sync_fifo #(
        .DATA_WIDTH(8),
        .ADDR_WIDTH(FIFO_DEPTH_LOG2)
    ) fifo (
        .i_clock(i_clock), .i_reset_n(i_reset_n),
        .i_write_data(rx_data), .i_write_valid(rx_valid),
        .o_write_ready(rx_ready),
        .o_read_data(o_rx_data), .o_read_valid(o_rx_valid),
        .i_read_ready(i_rx_ready)
    );

    assign o_rx_error = rx_error;

endmodule
