`timescale 1ns/1ps

module sync_fifo_tb;
    reg        clock;
    reg        reset_n;
    reg  [7:0] write_data;
    reg        write_valid;
    wire       write_ready;
    wire [7:0] read_data;
    wire       read_valid;
    reg        read_ready;
    integer    errors;

    sync_fifo #(
        .DATA_WIDTH(8),
        .ADDR_WIDTH(2)       // 深度为4，便于快速测满
    ) dut (
        .i_clock(clock), .i_reset_n(reset_n),
        .i_write_data(write_data), .i_write_valid(write_valid),
        .o_write_ready(write_ready),
        .o_read_data(read_data), .o_read_valid(read_valid),
        .i_read_ready(read_ready)
    );

    initial clock = 1'b0;
    always #5 clock = ~clock;

    task push;
        input [7:0] value;
        begin
            @(negedge clock);
            write_data  = value;
            write_valid = 1'b1;
            while (!write_ready) @(negedge clock);
            @(negedge clock);
            write_valid = 1'b0;
        end
    endtask

    task pop_and_check;
        input [7:0] expected;
        begin
            @(negedge clock);
            read_ready = 1'b1;
            while (!read_valid) @(negedge clock);
            if (read_data !== expected) begin
                $display("错误：期望%h，实际%h", expected, read_data);
                errors = errors + 1;
            end
            @(negedge clock);
            read_ready = 1'b0;
        end
    endtask

    initial begin
        reset_n = 1'b0;
        write_data = 8'd0;
        write_valid = 1'b0;
        read_ready = 1'b0;
        errors = 0;

        repeat (4) @(posedge clock);
        reset_n = 1'b1;

        // 写满4项，验证FIFO满后停止接收
        push(8'h11);
        push(8'h22);
        push(8'h33);
        push(8'h44);
        @(negedge clock);
        if (write_ready !== 1'b0) begin
            $display("错误：FIFO写满后write_ready仍为1");
            errors = errors + 1;
        end

        // 验证先进先出顺序
        pop_and_check(8'h11);
        pop_and_check(8'h22);
        pop_and_check(8'h33);
        pop_and_check(8'h44);
        @(negedge clock);
        if (read_valid !== 1'b0) begin
            $display("错误：FIFO读空后read_valid仍为1");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS：sync_fifo测试通过");
        else
            $display("FAIL：sync_fifo共有%0d个错误", errors);
        $finish;
    end
endmodule
