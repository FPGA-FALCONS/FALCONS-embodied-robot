`timescale 1ns/1ps
module nav_point_ctrl_tb;
    reg clk=0,rst_n=0,request=0,estop=0,tick=0;
    always #10 clk=~clk;
    reg signed [31:0] x=0,y=0,yaw=0,v=0,tx=1000,ty=0;
    reg [3:0] stall=0;
    wire active,done,linear,fault;
    wire signed [31:0] center,diff;
    nav_point_ctrl #(.STOP_SETTLE_TICKS(3),.TIMEOUT_TICKS(200)) dut(
        .clk(clk),.rst_n(rst_n),.i_cmd_valid(request),.i_emergency_stop(estop),
        .i_target_x_mm(tx),.i_target_y_mm(ty),.i_tick(tick),.i_x_mm(x),.i_y_mm(y),
        .i_yaw_centi(yaw),.i_v_mms(v),.i_stall(stall),
        .i_v_cruise(32'sd300),.i_d_slow(32'sd100),.i_align_in(32'sd800),.i_align_out(32'sd3500),
        .i_max_diff_cruise(32'sd126),.i_kp_yaw(32'sd40),.i_ki_yaw(32'sd3),.i_kd_yaw(32'sd6),
        .i_i_limit(32'sd120),.i_k_slow_q10(32'sd3072),
        .o_auto_valid(active),.o_center(center),.o_diff(diff),.o_done(done),.o_linear_active(linear),.o_fault(fault));
    integer cycles=0,checks=0,errors=0,n;
    always @(posedge clk) cycles<=cycles+1;
    task check;input c;input[511:0]msg;
        begin checks=checks+1;if(c!==1'b1)begin errors=errors+1;$display("FAIL: %0s",msg);end
        else $display("pass %0s",msg);end
    endtask
    task step;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(78)@(negedge clk);end endtask
    task steps;input integer count;integer k;begin for(k=0;k<count;k=k+1)step();end endtask
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        estop=1;steps(1);check(!done,"startup stop first observes done low");steps(3);
        check(done && center==0 && diff==0,"startup stop is acknowledged without a target");
        repeat(10)@(negedge clk);check(done,"stop completion is held");
        estop=0;repeat(3)@(negedge clk);check(!done,"stop release clears completion");
        yaw=9000;request=1;steps(4);check(center==0 && diff<0 && !linear,"side target turns before translating");
        yaw=0;steps(5);check(linear && center>0 && !done,"aligned task starts translating");
        tx=1500;x=1000;steps(5);check(!done && center>0,"live target extension does not complete old target");
        tx=1030;steps(4);check(center<=300,"live target shortening changes deceleration");
        x=tx;v=0;steps(7);check(done && !linear && center==0,"arrival includes stopped confirmation");
        steps(2);check(done,"normal completion is held until request withdrawn");
        estop=1;@(negedge clk);repeat(2)@(negedge clk);check(!done,"new emergency clears stale normal completion");
        steps(3);check(done,"new emergency gets its own stopped confirmation");
        request=0;estop=0;repeat(3)@(negedge clk);check(!done,"normal and emergency request release");
        x=0;tx=1000;request=1;steps(4);v=200;
        // 在计算中间而非 i_tick 到来时抢占。
        @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);estop=1;
        repeat(2)@(negedge clk);check(center==0 && diff==0 && !done,"emergency preempts in-flight calculation");
        steps(5);check(!done && linear,"decelerating car remains in linear phase");
        v=0;steps(3);check(done && !linear,"linear phase clears only after stopped");
        estop=0;request=0;repeat(4)@(negedge clk);
        request=1;steps(3);stall=1;steps(1);check(!active && center==0 && diff==0,"stall revokes propulsion");
        $display("CHECKS: %0d",checks);$display("METRICS: nav_point_ctrl cycles=%0d errors=%0d",cycles,errors);
        if(!errors)$display("PASS: nav_point_ctrl_tb");$finish;
    end
    initial begin #10000000;$display("FAIL: nav_point_ctrl timeout");$finish;end
endmodule
