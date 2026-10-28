`timescale 1ns/1ps
module inspection_speed_tb;
    reg clk=0,rst=0,owned=0,tick=0;
    reg [15:0] cruise=300;
    reg signed [31:0] base_cruise=450,dslow=100,base_gain=4608;
    wire signed [31:0] selected_cruise,gain,center,diff;
    integer checks=0,errors=0,n;
    always #10 clk=~clk;
    inspection_speed_params params(.clk(clk),.rst_n(rst),.i_owned(owned),.i_cruise_mm_s(cruise),
        .i_base_cruise(base_cruise),.i_d_slow(dslow),.i_base_k_slow(base_gain),
        .o_cruise(selected_cruise),.o_k_slow(gain));
    nav_point_ctrl nav(.clk(clk),.rst_n(rst),.i_cmd_valid(owned),.i_emergency_stop(1'b0),
        .i_target_x_mm(32'sd2000),.i_target_y_mm(32'sd0),.i_tick(tick),
        .i_x_mm(32'sd0),.i_y_mm(32'sd0),.i_yaw_centi(32'sd0),.i_v_mms(32'sd0),.i_stall(4'd0),
        .i_v_cruise(selected_cruise),.i_d_slow(dslow),.i_align_in(32'sd800),.i_align_out(32'sd3500),
        .i_max_diff_cruise(32'sd126),.i_kp_yaw(32'sd16),.i_ki_yaw(32'sd0),.i_kd_yaw(32'sd0),
        .i_i_limit(32'sd120),.i_k_slow_q10(gain),.o_auto_valid(),.o_center(center),.o_diff(diff),
        .o_done(),.o_linear_active(),.o_fault());
    task check;input condition;input [639:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task ticks;input integer count;
        begin for(n=0;n<count;n=n+1)begin
            @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(100)@(negedge clk);
        end end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        check(selected_cruise==450 && gain==4608,"base navigation retains FPGA cruise and gain without RK ownership");
        owned=1;repeat(35)@(negedge clk);
        check(selected_cruise==300 && gain==3072,"300mm/s and its deceleration gain arrive without 8-bit truncation");
        ticks(4);
        $display("RK_CRUISE_OUTPUT center=%0d diff=%0d cruise=%0d gain=%0d",center,diff,selected_cruise,gain);
        check(center==892 && center>120 && diff>=-1 && diff<=1,"real point controller executes 300mm/s without old 120-count clamp");
        base_cruise=600;base_gain=6144;repeat(35)@(negedge clk);
        check(selected_cruise==300 && gain==3072,"FPGA cruise knob does not replace RK inspection cruise");
        cruise=180;repeat(35)@(negedge clk);ticks(2);
        check(selected_cruise==180 && gain==1843 && center==535,"RK speed update also changes real controller output and slow gain");
        dslow=200;repeat(35)@(negedge clk);
        check(gain==921,"FPGA deceleration-distance tuning recalculates gain with RK speed");
        cruise=400;repeat(4)@(negedge clk);cruise=500;repeat(35)@(negedge clk);
        check(selected_cruise==500 && gain==2560,"a newer speed restarts the iterative parameter calculation");
        owned=0;repeat(3)@(negedge clk);
        check(selected_cruise==600 && gain==6144,"releasing AI restores standalone base-navigation parameters");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_speed %0d checks",checks);else $display("FAIL: inspection_speed %0d",errors);
        $finish;
    end
    initial begin #200000;$display("FAIL: inspection_speed timeout");$finish;end
endmodule
