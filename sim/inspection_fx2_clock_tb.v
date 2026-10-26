`timescale 1ns/1ps
module inspection_fx2_clock_tb;
    reg clk=0,fclk=0,rst=0,clock_enabled=1,measure=0;
    real half_period=8.333333;
    always #5 clk=~clk;
    always begin #(half_period);if(clock_enabled)fclk=~fclk;end
    wire [31:0] hz;
    wire valid,alive;
    integer checks=0,errors=0;
    inspection_fx2_clock #(.CLK_HZ(1000)) dut(.clk(clk),.rst_n(rst),.fx2_clk(fclk),.i_measure(measure),.o_hz(hz),.o_valid(valid),.o_alive(alive));
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
        check(!alive && !valid,"missing IFCLK is detected in the independent system domain");
        clock_enabled=1;repeat(20)@(negedge clk);
        check(alive && !valid,"clock return is detected without relying on a stopped-domain reset release");
        repeat(1100)@(negedge clk);
        check(valid && hz>=959 && hz<=961,"clock return requires a complete new measurement window");
        measure=1;@(negedge clk);measure=0;
        check(!valid,"a new diagnostic explicitly invalidates the previous frequency");
        repeat(500)@(negedge clk);check(!valid,"partial measurement cannot authorize a diagnostic reply");
        repeat(600)@(negedge clk);check(valid && hz>=959 && hz<=961,"fresh challenge receives a complete independent measurement");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_fx2_clock %0d checks",checks);else $display("FAIL: inspection_fx2_clock %0d",errors);
        $finish;
    end
endmodule
