`timescale 1ns/1ps
module tb_tof_i2c_reg_master;
    parameter integer CLK_HZ = 4000000;
    localparam integer CLOCK_HALF_NS = 500000000 / CLK_HZ;
    reg clk = 1'b0;
    always #(CLOCK_HALF_NS) clk = ~clk;
    reg rst_n = 1'b0, cmd_valid = 1'b0, cmd_read = 1'b0;
    reg [7:0] cmd_reg = 8'h14, cmd_wdata = 8'ha6;
    wire cmd_ready, rsp_valid;
    wire [7:0] rsp_rdata;
    wire [2:0] rsp_error;
    tri1 scl, sda;
    reg slave_sda_low = 1'b0, slave_scl_low = 1'b0;
    assign sda = slave_sda_low ? 1'b0 : 1'bz;
    assign scl = slave_scl_low ? 1'b0 : 1'bz;
    integer failures = 0, checks = 0, responses = 0;
    tof_i2c_reg_master #(.CLK_HZ(CLK_HZ), .I2C_TIMEOUT_US(20)) dut (
        .clk(clk), .rst_n(rst_n), .cmd_valid(cmd_valid),
        .cmd_read(cmd_read), .cmd_reg(cmd_reg), .cmd_wdata(cmd_wdata),
        .cmd_ready(cmd_ready), .rsp_valid(rsp_valid),
        .rsp_rdata(rsp_rdata), .rsp_error(rsp_error), .scl(scl), .sda(sda)
    );
    always @(posedge clk) if (rsp_valid) responses = responses + 1;
    task check;
        input condition;
        input [511:0] description;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                failures = failures + 1;
                $display("FAIL: %0s at %0t", description, $time);
            end
        end
    endtask
    task wait_start;
        begin : detect_start
            forever begin
                @(negedge sda);
                if (scl === 1'b1) disable detect_start;
            end
        end
    endtask
    task wait_stop;
        begin : detect_stop
            forever begin
                @(posedge sda);
                if (scl === 1'b1) disable detect_stop;
            end
        end
    endtask
    task slave_receive;
        input [7:0] expected;
        input nack;
        input stretch;
        reg [7:0] received;
        integer b;
        begin
            received = 8'd0;
            for (b = 7; b >= 0; b = b - 1) begin
                @(posedge scl);
                received[b] = sda;
            end
            check(received == expected, "slave received expected byte");
            @(negedge scl);
            slave_sda_low = !nack;
            if (stretch) begin
                slave_scl_low = 1'b1;
                #12000;
                slave_scl_low = 1'b0;
            end
            @(posedge scl);
            @(negedge scl);
            slave_sda_low = 1'b0;
        end
    endtask
    task slave_transmit;
        input [7:0] value;
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                slave_sda_low = !value[b];
                @(posedge scl);
                @(negedge scl);
            end
            slave_sda_low = 1'b0;
            @(posedge scl);
            check(sda === 1'b1, "master NACKs final read byte");
            @(negedge scl);
        end
    endtask
    task slave_transaction;
        input rd;
        input integer nack_phase;
        input stretch;
        begin : transaction
            wait_start;
            slave_receive(8'h52, nack_phase == 0, stretch);
            if (nack_phase == 0) begin wait_stop; disable transaction; end
            slave_receive(8'h14, nack_phase == 1, 1'b0);
            if (nack_phase == 1) begin wait_stop; disable transaction; end
            if (rd) begin
                wait_start;
                slave_receive(8'h53, nack_phase == 3, 1'b0);
                if (nack_phase == 3) begin wait_stop; disable transaction; end
                slave_transmit(8'hd3);
            end else begin
                slave_receive(8'ha6, nack_phase == 2, 1'b0);
            end
            wait_stop;
        end
    endtask
    task host_transaction;
        input rd;
        input [2:0] expected_error;
        integer before_count;
        begin
            before_count = responses;
            @(negedge clk);
            check(cmd_ready, "ready before command");
            cmd_read = rd;
            cmd_valid = 1'b1;
            @(negedge clk);
            cmd_valid = 1'b0;
            check(!cmd_ready, "not ready during command");
            wait (rsp_valid);
            #1;
            check(rsp_error == expected_error, "response error matches");
            if (rd && expected_error == 3'd0)
                check(rsp_rdata == 8'hd3, "read data equals slave data");
            repeat (10) @(negedge clk);
            check(responses == before_count + 1, "exactly one response");
            check(cmd_ready, "ready restored");
        end
    endtask
    task normal_case;
        input rd;
        input integer nack_phase;
        input stretch;
        begin
            fork
                slave_transaction(rd, nack_phase, stretch);
                host_transaction(rd, nack_phase < 0 ? 3'd0 : 3'd1);
            join
        end
    endtask
    initial begin
        #10000000;
        $display("FAIL: global timeout");
        $fatal(1);
    end
    initial begin
        repeat (5) @(negedge clk);
        check(scl === 1'b1 && sda === 1'b1, "reset releases pins");
        rst_n = 1'b1;
        normal_case(1'b0, -1, 1'b0);
        normal_case(1'b1, -1, 1'b0);
        normal_case(1'b0, 0, 1'b0);
        normal_case(1'b0, 1, 1'b0);
        normal_case(1'b0, 2, 1'b0);
        normal_case(1'b1, 3, 1'b0);
        normal_case(1'b1, -1, 1'b1);
        slave_sda_low = 1'b1;
        host_transaction(1'b0, 3'd3);
        slave_sda_low = 1'b0;
        slave_scl_low = 1'b1;
        host_transaction(1'b0, 3'd2);
        slave_scl_low = 1'b0;
        // Stretch beyond the deadline during the first ACK.
        fork
            begin
                wait_start;
                repeat (8) @(posedge scl);
                @(negedge scl);
                slave_scl_low = 1'b1;
                #40000;
                slave_scl_low = 1'b0;
            end
            host_transaction(1'b0, 3'd2);
        join
        // Reset in the middle of a transaction cancels it without a response.
        @(negedge clk);
        cmd_valid = 1'b1;
        @(negedge clk);
        cmd_valid = 1'b0;
        repeat (20) @(negedge clk);
        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        check(scl === 1'b1 && sda === 1'b1, "mid-command reset releases bus");
        check(!rsp_valid, "reset cancels response");
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
        normal_case(1'b1, -1, 1'b0);
        $display("I2C TEST clk=%0d: %0d checks, %0d failures", CLK_HZ, checks, failures);
        if (failures != 0) $fatal(1);
        $finish;
    end
endmodule
