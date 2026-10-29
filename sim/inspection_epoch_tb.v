`timescale 1ns/1ps
module inspection_epoch_tb;
    reg clk=0,rst=0,clear=0,done=0;
    wire [31:0] epoch;
    integer checks=0,errors=0;
    always #10 clk=~clk;
    inspection_epoch_tracker dut(.clk(clk),.rst_n(rst),.i_origin_clear(clear),.i_calib_done(done),.o_epoch(epoch));
    task check;input ok;input [639:0] name;
        begin checks=checks+1;if(ok===1'b1)$display("pass %0s",name);else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        #100;rst=1;repeat(10)@(negedge clk);check(epoch==1,"initial epoch is stable and nonzero");
        done=1;repeat(10000)@(negedge clk);check(epoch==2,"held IMU calibration-valid level advances exactly once");
        clear=1;repeat(20)@(negedge clk);check(epoch==3,"held origin-clear level advances once while calibration remains valid");
        clear=0;done=0;repeat(3)@(negedge clk);done=1;clear=1;repeat(20)@(negedge clk);
        check(epoch==4,"new simultaneous localization events coalesce without repeated invalidation");
        done=0;clear=0;@(negedge clk);rst=0;@(negedge clk);rst=1;repeat(3)@(negedge clk);
        check(epoch==1,"system reset starts a new handshake epoch; host session distinguishes reboot");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_epoch %0d checks",checks);else $display("FAIL: inspection_epoch %0d",errors);
        $finish;
    end
endmodule
