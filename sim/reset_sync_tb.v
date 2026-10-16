`timescale 1us/1ns
module reset_sync_tb;
    reg clk=0,key=1;
    always #0.5 clk=~clk;
    wire rst_n;
    integer errors=0,checks=0;
    reset_sync #(.CLK_FREQ_HZ(1000000),.DEBOUNCE_MS(1)) dut(clk,key,rst_n);
    task check;
        input [8*60-1:0] label;
        input integer got,want;
        begin checks=checks+1;
            if(got!==want) begin errors=errors+1;$display("FAIL %0s got=%0d expected=%0d",label,got,want);end
            else $display("pass %0s",label);
        end
    endtask
    initial begin
        #10;check("cold startup remains reset",rst_n,0);
        #1050;check("released button boots without initial key press",rst_n,1);
        @(negedge clk);key=0;#100;key=1;#100;
        check("short reset bounce ignored",rst_n,1);
        @(negedge clk);key=0;#1050;check("held key asserts reset",rst_n,0);
        key=1;#100;key=0;#100;check("short release bounce ignored",rst_n,0);
        key=1;#1050;check("stable release exits reset",rst_n,1);
        $display("CHECKS: %0d",checks);
        if(errors!=0)$fatal(1,"FAIL: reset_sync");
        $display("PASS: reset_sync");$finish;
    end
    initial begin #6000;$fatal(1,"FAIL: watchdog");end
endmodule
