`timescale 1ns/1ps
module hdmi_tx_reset_tb;
    reg clk=0,run_clock=1,reset_n=0,locked=0;
    wire tx_reset_n;
    integer checks=0,errors=0;
    always #2 if(run_clock)clk=~clk;
    hdmi_tx_reset dut(.serial_clk(clk),.reset_n(reset_n),.pll_lock(locked),.tx_reset_n(tx_reset_n));
    task check;
        input condition;
        input [511:0] name;
        begin
            checks=checks+1;
            if(condition===1'b1)$display("pass %0s",name);
            else begin errors=errors+1;$display("FAIL: %0s",name);end
        end
    endtask
    task release_three_edges;
        begin
            @(posedge clk);#0.1;check(!tx_reset_n,"first serial edge remains in reset");
            @(posedge clk);#0.1;check(!tx_reset_n,"second serial edge remains in reset");
            @(posedge clk);#0.1;check(tx_reset_n,"third serial edge releases transmitter");
        end
    endtask
    initial begin
        #1;reset_n=1;#0.1;check(!tx_reset_n,"unlocked PLL holds transmitter reset");
        @(negedge clk);#0.7;locked=1;
        release_three_edges;
        #0.6;reset_n=0;#0.1;check(!tx_reset_n,"reset assertion does not wait for serial edge");
        @(negedge clk);#0.7;reset_n=1;
        release_three_edges;
        @(negedge clk);run_clock=0;#0.4;locked=0;#0.1;
        check(!tx_reset_n,"PLL loss asserts reset even with stopped clock");
        locked=1;#12;check(!tx_reset_n,"relocking cannot release reset without clock edges");
        run_clock=1;release_three_edges;
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: hdmi_tx_reset %0d checks",checks);
        else $display("FAIL: hdmi_tx_reset %0d errors",errors);
        $finish;
    end
endmodule
