`timescale 1ns/1ps
module inspection_ddr_deadline_tb;
    reg clk=0,rst=0;always #10 clk=~clk;
    reg camera=1,read_request=0,write_ready=0;
    wire lr,lwr,ar,men;integer reads=0,checks=0,errors=0;
    inspection_ddr_arb dut(.clk(clk),.rst_n(rst),.i_initialized(1'b1),
        .i_legacy_want(camera),.i_legacy_en(camera && lr && lwr),.i_legacy_cmd(3'd0),
        .i_legacy_address(28'd0),.i_legacy_write_data(128'd0),.o_legacy_ready(lr),.o_legacy_write_ready(lwr),
        .i_write_valid(1'b0),.i_write_address(28'd0),.i_write_data(128'd0),
        .i_read_valid(read_request),.i_read_address(28'h100000),.o_read_ready(ar),
        .i_memory_ready(1'b1),.i_memory_write_ready(write_ready),.i_memory_read_valid(1'b0),
        .i_memory_read_data(128'd0),.o_memory_en(men));
    always @(posedge clk)if(rst && ar && read_request)reads=reads+1;
    task check;input condition;input [767:0] label;
        begin checks=checks+1;if(condition)$display("pass %0s",label);
        else begin errors=errors+1;$display("FAIL: %0s",label);end end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        read_request=1;repeat(6)@(negedge clk);read_request=0;
        check(reads>0,"AI display read bypasses camera write-data backpressure before FIFO deadline");
        write_ready=1;repeat(5)@(negedge clk);
        check(lr && lwr && men,"camera-only path resumes after AI request withdraws");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_ddr_deadline %0d checks",checks);
        $finish;
    end
endmodule
