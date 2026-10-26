`timescale 1ns/1ps
module inspection_fx2_clock_tb;
    reg clk=0,fclk=0,rst=0,clock_enabled=1;
    real half_period=8.333333;
    always #5 clk=~clk;
    always begin #(half_period);if(clock_enabled)fclk=~fclk;end
    wire [31:0] hz;
    wire valid,alive;
    integer checks=0,errors=0;
    inspection_fx2_clock #(.CLK_HZ(1000)) dut(.clk(clk),.rst_n(rst),.fx2_clk(fclk),.o_hz(hz),.o_valid(valid),.o_alive(alive));
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        #100;rst=1;
        repeat(2100)@(negedge clk);
        check(valid && alive && hz>=599 && hz<=601,"Gray counter independently measures the 30MHz-equivalent interface ratio");
        half_period=5.208333;repeat(2100)@(negedge clk);
        check(valid && alive && hz>=959 && hz<=961,"48MHz-equivalent clock is distinguishable from required 30MHz");
        clock_enabled=0;repeat(30)@(negedge clk);
        check(!alive,"missing IFCLK is detected in the independent system domain");
        clock_enabled=1;repeat(20)@(negedge clk);
        check(alive,"clock return is detected without relying on a stopped-domain reset release");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_fx2_clock %0d checks",checks);else $display("FAIL: inspection_fx2_clock %0d",errors);
        $finish;
    end
endmodule
