`timescale 1ns/1ps

module uart_queue_loopback_tb;
    reg        clock;
    reg        reset_n;
    reg  [7:0] tx_data;
    reg        tx_valid;
    wire       tx_ready;
    wire       serial_line;
    wire [7:0] rx_data;
    wire       rx_valid;
    reg        rx_ready;
    wire       rx_error;
    integer    received_count;
    integer    errors;

    uart_tx_queue #(
        .CLK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000),
        .DATA_BITS(8), .PARITY_EN(1), .PARITY_TYPE(0),
        .FIFO_DEPTH_LOG2(2)
    ) tx_queue (
        .i_clock(clock), .i_reset_n(reset_n),
        .i_tx_data(tx_data), .i_tx_valid(tx_valid),
        .o_tx_ready(tx_ready), .o_uart_tx(serial_line)
    );

    uart_rx_queue #(
        .CLK_FREQ_HZ(1_000_000), .BAUD_RATE(100_000),
        .DATA_BITS(8), .PARITY_EN(1), .PARITY_TYPE(0),
        .FIFO_DEPTH_LOG2(2)
    ) rx_queue (
        .i_clock(clock), .i_reset_n(reset_n), .i_uart_rx(serial_line),
        .o_rx_data(rx_data), .o_rx_valid(rx_valid),
        .i_rx_ready(rx_ready), .o_rx_error(rx_error)
    );

    initial clock = 1'b0;
    always #500 clock = ~clock;

    task enqueue;
        input [7:0] value;
        begin
            @(negedge clock);
            tx_data  = value;
            tx_valid = 1'b1;
            while (!tx_ready) @(negedge clock);
            @(negedge clock);
            tx_valid = 1'b0;
        end
    endtask

    always @(posedge clock) begin
        if (rx_error) begin
            $display("错误：UART接收器报告帧错误");
            errors = errors + 1;
        end

        if (rx_valid && rx_ready) begin
            case (received_count)
                0: if (rx_data !== 8'h11) errors = errors + 1;
                1: if (rx_data !== 8'h22) errors = errors + 1;
                2: if (rx_data !== 8'h33) errors = errors + 1;
                3: if (rx_data !== 8'hA5) errors = errors + 1;
            endcase
            received_count = received_count + 1;
        end
    end

    initial begin
        reset_n = 1'b0;
        tx_data = 8'd0;
        tx_valid = 1'b0;
        rx_ready = 1'b1;
        received_count = 0;
        errors = 0;
        repeat (5) @(posedge clock);
        reset_n = 1'b1;

        enqueue(8'h11);
        enqueue(8'h22);
        enqueue(8'h33);
        enqueue(8'hA5);

        wait (received_count == 4);
        repeat (10) @(posedge clock);
        if (errors == 0)
            $display("PASS：UART FIFO回环测试通过");
        else
            $display("FAIL：UART FIFO回环测试共有%0d个错误", errors);
        $finish;
    end
endmodule
