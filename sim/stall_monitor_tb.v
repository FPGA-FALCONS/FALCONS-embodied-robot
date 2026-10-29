`timescale 1ns/1ps
module stall_monitor_tb;
    reg clk=0,rst=0,en=0,valid=0,clear=0;
    reg [15:0] duty=500;
    reg signed [31:0] delta=0;
    wire stall,enable_out;
    integer errors=0,checks=0;
    always #10 clk=~clk;
    stall_detect #(.STARTUP_WINDOWS(1),.STALL_WINDOWS(2),.DUTY_TH(200),.MOVE_TH(3),.PIPE_MONITOR_INPUT(1)) dut(
        .i_clock(clk),.i_reset_n(rst),.i_enable(en),.i_duty(duty),.i_delta(delta),.i_delta_valid(valid),
        .i_stall_clear(clear),.o_stall(stall),.o_enable_out(enable_out));
    task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
    task sample;input signed [31:0] value;
        begin delta=value;valid=1;tick;valid=0;tick;end
    endtask
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    initial begin
        tick;rst=1;en=1;tick;
        sample(0);sample(0);sample(0);
        check(!stall,"startup grace and first stationary monitor window remain distinct");
        sample(0);
        check(stall && !enable_out,"stationary driven motor still latches stall after configured windows");
        clear=1;tick;clear=0;tick;
        check(!stall && enable_out,"explicit stall clear re-arms the monitor");
        sample(0);sample(4);repeat(8)sample(4);
        check(!stall,"coherent pipelined movement samples prevent false stall");
        en=0;#1;
        check(!enable_out,"live disable inhibits PWM immediately without waiting for monitor pipeline");
        sample(0);sample(0);sample(0);sample(0);
        check(!stall,"disabled monitor cannot accumulate stationary windows");
        en=1;valid=1;delta=0;clear=1;tick;valid=0;clear=0;tick;
        check(dut.state==0,"clear discards a simultaneous sampled window");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: stall_monitor %0d checks",checks);else $display("FAIL: stall_monitor %0d",errors);
        $finish;
    end
endmodule
