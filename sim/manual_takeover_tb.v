`timescale 1ns/1ps
module manual_takeover_tb;
    reg clk=0,rst=0,manual=0,signal_ok=1,estop=0,sample=0;
    reg signed [11:0] throttle=0,steer=0;
    reg signed [31:0] d0=0,d1=0,d2=0,d3=0;
    wire ready,brake,run;
    wire [3:0] motor_enable,motor_brake;
    wire signed [31:0] target_left,target_right;
    integer checks=0,errors=0;
    always #5 clk=~clk;
    manual_takeover #(.CLK_HZ(1000),.HOLD_MS(5)) dut(
        .clk(clk),.rst_n(rst),.i_manual(manual),.i_signal_valid(signal_ok),.i_estop(estop),
        .i_throttle(throttle),.i_steer(steer),.i_sample_valid(sample),
        .i_delta0(d0),.i_delta1(d1),.i_delta2(d2),.i_delta3(d3),.o_ready(ready),.o_brake(brake));
    rc_drive_guard guard(.clk(clk),.rst_n(rst),.i_manual(manual),
        .i_signal_valid(signal_ok && ready),.i_estop(estop),.i_stop_pulse(1'b0),
        .i_throttle(throttle),.o_enable(run),.o_arm_hint());
    // Exercise the real chassis safety outputs against a continuously nonzero
    // autonomous command. Neither UART nor a host response is involved.
    chassis_ctrl chassis(.clk(clk),.rst_n(rst),.i_enable(manual?run:1'b1),
        .i_estop(estop || brake || !signal_ok),.i_manual(manual),.i_steer(steer),.i_throttle(throttle),
        .i_auto_valid(1'b1),.i_auto_center(32'sd900),.i_auto_diff(32'sd100),
        .i_kp(16'sd26),.i_ki(16'sd3),.i_kd(16'sd0),
        .i_delta0(d0),.i_delta1(d1),.i_delta2(d2),.i_delta3(d3),.i_delta_valid(sample),.i_stall(4'd0),
        .o_duty(),.o_dir(),.o_enable(motor_enable),.o_brake(motor_brake),
        .o_tgt_left(target_left),.o_tgt_right(target_right),.o_rpm0(),.o_rpm1(),.o_rpm2(),.o_rpm3(),.o_sat(),.o_running());
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task tick;begin @(negedge clk);end endtask
    task feedback;begin sample=1;tick;sample=0;tick;end endtask
    initial begin
        repeat(3)tick;rst=1;repeat(2)tick;
        check(!ready && !brake && !run,"cursor leaves manual guard inactive");
        manual=1;#1;check(brake && !run,"SWC manual immediately brakes before sticks");
        check(motor_brake==15 && motor_enable==0,"actual chassis brakes override ongoing autonomous target");
        d0=20;repeat(8)feedback;check(!ready && brake,"time alone cannot release while wheel moving");
        d0=0;feedback;feedback;check(!ready,"two stopped samples are insufficient");
        steer=300;feedback;check(!ready,"steering must also return to center");
        steer=0;feedback;repeat(3)tick;check(ready && run && !brake,"fresh stopped feedback and centered sticks release handover");
        check(target_left==0 && target_right==0,"held autonomous commands cannot leak into centered manual control");
        throttle=600;repeat(6)tick;check(run,"normal stick driving remains armed");
        check(motor_enable==15 && motor_brake==0 && target_left>900 && target_right>900,"after stop the real chassis follows manual sticks");
        manual=0;tick;check(!ready && !run && !brake,"leaving manual discards handover readiness");
        manual=1;repeat(8)feedback;check(!ready && brake,"held throttle cannot drive on manual reentry");
        throttle=0;feedback;repeat(3)tick;check(run,"center releases new manual takeover");
        estop=1;tick;check(!run && !ready && brake,"physical estop still overrides manual");
        estop=0;repeat(8)tick;check(!ready,"estop recovery cannot use stale encoder feedback");
        repeat(3)feedback;repeat(3)tick;check(run,"fresh stop confirmation permits manual after recovery");
        signal_ok=0;#1;check(!run,"RC loss immediately disables manual");
        tick;signal_ok=1;repeat(8)tick;check(!ready && brake,"RC recovery requires fresh stopped samples");
        $display("CHECKS: %0d",checks);$display("%0d FAIL",errors);$display("PASS: manual_takeover");$finish;
    end
endmodule
