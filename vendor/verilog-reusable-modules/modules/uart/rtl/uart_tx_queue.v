//////////////////////////////////////////////////////////////////////////////////
// 模块：uart_tx_queue —— 带FIFO缓存的UART发送队列
// 数据完成valid/ready握手后进入FIFO，并由UART按照写入顺序自动发送。
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
//   i_tx_data     输入  要加入发送队列的数据
//   i_tx_valid    输入  写入数据有效，握手前保持为1
//   o_tx_ready    输出  FIFO有空间，可以接收数据
//   o_uart_tx     输出  UART串行发送线
//
// 最小示例（115200 8N1、16字节队列）：
//   uart_tx_queue #(.BAUD_RATE(115_200), .FIFO_DEPTH_LOG2(4)) u_txq (
//       .i_clock(clk), .i_reset_n(reset_n),
//       .i_tx_data(send_data), .i_tx_valid(send_valid),
//       .o_tx_ready(send_ready), .o_uart_tx(uart_tx)
//   );
//   // 向队列写入一次；FIFO暂时满时，send_valid会自动保持
//   always @(posedge clk or negedge reset_n) begin
//       if (!reset_n)
//           send_valid <= 1'b0;
//       else begin
//           if (send_valid && send_ready)
//               send_valid <= 1'b0;              // 数据已进入发送FIFO
//           if (send_request && (!send_valid || send_ready)) begin
//               send_data  <= 8'h55;
//               send_valid <= 1'b1;
//           end
//       end
//   end
//////////////////////////////////////////////////////////////////////////////////
module uart_tx_queue #(
    parameter integer CLK_FREQ_HZ     = 50_000_000,
    parameter integer BAUD_RATE       = 9_600,
    parameter integer DATA_BITS       = 8,
    parameter integer PARITY_EN       = 0,
    parameter integer PARITY_TYPE     = 0,
    parameter integer FIFO_DEPTH_LOG2 = 4
)(
    input  wire       i_clock,
    input  wire       i_reset_n,
    input  wire [7:0] i_tx_data,
    input  wire       i_tx_valid,
    output wire       o_tx_ready,
    output wire       o_uart_tx
);

    wire [7:0] fifo_data;
    wire fifo_valid;
    wire fifo_ready;

    sync_fifo #(
        .DATA_WIDTH(8),
        .ADDR_WIDTH(FIFO_DEPTH_LOG2)
    ) fifo (
        .i_clock(i_clock), .i_reset_n(i_reset_n),
        .i_write_data(i_tx_data), .i_write_valid(i_tx_valid),
        .o_write_ready(o_tx_ready),
        .o_read_data(fifo_data), .o_read_valid(fifo_valid),
        .i_read_ready(fifo_ready)
    );

    uart_byte_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE),
        .DATA_BITS(DATA_BITS), .PARITY_EN(PARITY_EN),
        .PARITY_TYPE(PARITY_TYPE)
    ) transmitter (
        .i_clock(i_clock), .i_reset_n(i_reset_n),
        .i_tx_data(fifo_data), .i_tx_valid(fifo_valid),
        .o_tx_ready(fifo_ready), .o_uart_tx(o_uart_tx)
    );

endmodule
