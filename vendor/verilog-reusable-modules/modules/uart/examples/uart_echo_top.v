//////////////////////////////////////////////////////////////////////////////////
// 最小上板示例：UART FIFO回显
// 电脑发送的数据经过接收FIFO和发送FIFO后原样返回。
// 串口配置：115200波特率、8位数据、无校验、1位停止位。
//////////////////////////////////////////////////////////////////////////////////
module uart_echo_top(
    input  wire i_clock,
    input  wire i_reset_n,
    input  wire i_uart_rx,
    output wire o_uart_tx,
    output wire o_error_led
);

    wire [7:0] received_data;
    wire       received_valid;
    wire       receive_ready;
    wire       receive_error;
    wire       transmit_ready;
    reg        error_latched;

    uart_rx_queue #(
        .CLK_FREQ_HZ(50_000_000), .BAUD_RATE(115_200),
        .DATA_BITS(8), .PARITY_EN(0), .PARITY_TYPE(0),
        .FIFO_DEPTH_LOG2(4)
    ) receive_queue (
        .i_clock(i_clock), .i_reset_n(i_reset_n), .i_uart_rx(i_uart_rx),
        .o_rx_data(received_data), .o_rx_valid(received_valid),
        .i_rx_ready(receive_ready), .o_rx_error(receive_error)
    );

    uart_tx_queue #(
        .CLK_FREQ_HZ(50_000_000), .BAUD_RATE(115_200),
        .DATA_BITS(8), .PARITY_EN(0), .PARITY_TYPE(0),
        .FIFO_DEPTH_LOG2(4)
    ) transmit_queue (
        .i_clock(i_clock), .i_reset_n(i_reset_n),
        .i_tx_data(received_data), .i_tx_valid(received_valid),
        .o_tx_ready(transmit_ready), .o_uart_tx(o_uart_tx)
    );

    // 发送FIFO有空间时，从接收FIFO取出一个字节并完成队列间握手
    assign receive_ready = transmit_ready;

    // 错误脉冲锁存到LED，复位后清除
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)
            error_latched <= 1'b0;
        else if (receive_error)
            error_latched <= 1'b1;
    end

    assign o_error_led = error_latched;

endmodule
