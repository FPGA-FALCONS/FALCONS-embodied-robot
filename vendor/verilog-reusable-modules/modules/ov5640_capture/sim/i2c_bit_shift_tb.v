`timescale 1ns/1ps

module i2c_bit_shift_tb;

    localparam [5:0] CMD_WR  = 6'b000001;
    localparam [5:0] CMD_STA = 6'b000010;
    localparam [5:0] CMD_STO = 6'b001000;

    reg clk;
    reg reset_n;
    reg [5:0] cmd;
    reg go;
    reg [7:0] tx_data;
    wire [7:0] rx_data;
    wire trans_done;
    wire ack_o;
    wire i2c_sclk;
    tri1 i2c_sdat;

    reg slave_pull_sda_low;
    reg transaction_active;
    reg slave_should_ack;
    integer scl_rise_count;
    integer error_count;

    assign i2c_sdat = slave_pull_sda_low ? 1'b0 : 1'bz;

    i2c_bit_shift #(
        .SYS_CLOCK (40_000_000),
        .SCL_CLOCK (1_000_000)
    ) dut (
        .Clk        (clk),
        .Rst_n      (reset_n),
        .Cmd        (cmd),
        .Go         (go),
        .Rx_DATA    (rx_data),
        .Tx_DATA    (tx_data),
        .Trans_Done (trans_done),
        .ack_o      (ack_o),
        .i2c_sclk   (i2c_sclk),
        .i2c_sdat   (i2c_sdat)
    );

    always #12.5 clk = ~clk;

    always @(posedge i2c_sclk)
        if(transaction_active)
            scl_rise_count = scl_rise_count + 1;

    // Pull SDA low after the eighth data-bit clock. It is then stable before
    // the controller raises SCL for the ACK phase.
    always @(negedge i2c_sclk)
        if(transaction_active && slave_should_ack && scl_rise_count == 8)
            slave_pull_sda_low = 1'b1;

    task run_write;
        input expect_ack;
        begin
            scl_rise_count = 0;
            slave_should_ack = expect_ack;
            slave_pull_sda_low = 1'b0;
            transaction_active = 1'b1;

            @(posedge clk);
            cmd = CMD_WR | CMD_STA | CMD_STO;
            go = 1'b1;
            @(posedge clk);
            go = 1'b0;

            wait(trans_done === 1'b1);
            #1;
            if(ack_o !== (expect_ack ? 1'b0 : 1'b1))begin
                $display("ERROR: ACK result mismatch, expected %0d got %0d", !expect_ack, ack_o);
                error_count = error_count + 1;
            end

            @(posedge clk);
            #1;
            if(trans_done !== 1'b0)begin
                $display("ERROR: Trans_Done lasted more than one Clk cycle");
                error_count = error_count + 1;
            end

            transaction_active = 1'b0;
            slave_pull_sda_low = 1'b0;
            repeat(4) @(posedge clk);
        end
    endtask

    initial begin
        clk = 1'b0;
        reset_n = 1'b0;
        cmd = 6'd0;
        go = 1'b0;
        tx_data = 8'hA5;
        slave_pull_sda_low = 1'b0;
        transaction_active = 1'b0;
        slave_should_ack = 1'b0;
        scl_rise_count = 0;
        error_count = 0;

        repeat(5) @(posedge clk);
        reset_n = 1'b1;
        repeat(4) @(posedge clk);

        run_write(1'b1);
        run_write(1'b0);

        if(error_count == 0)
            $display("PASS: i2c_bit_shift ACK/NACK and Trans_Done checks passed");
        else
            $fatal(1, "FAIL: i2c_bit_shift detected %0d errors", error_count);
        $finish;
    end

    initial begin
        repeat(20000) @(posedge clk);
        $fatal(1, "FAIL: i2c_bit_shift test timed out");
    end

endmodule
