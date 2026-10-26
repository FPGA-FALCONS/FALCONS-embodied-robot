`timescale 1ns/1ps
module obstacle_plan_tb;
    reg clk=0,rst_n=0,active=0,manual=0,clear=0,confirm=0,back=0,up=0,down=0,pause=0;
    always #10 clk=~clk;
    reg signed [11:0] steer=0,throttle=0;
    reg [3:0] nav_status=0;
    reg command_idle=1;
    wire [3:0] feedback;
    wire [2:0] state;
    wire [4:0] count;
    wire run,paused,ui,goal;
    wire [7:0] gx,gy;
    wire signed [31:0] cx,cy;
    nav_obstacle_ui #(.CLK_HZ(1000)) dut(
        .clk(clk),.rst_n(rst_n),.i_active(active),.i_clear(clear),
        .i_x_mm(32'sd1000),.i_y_mm(32'sd0),.i_manual(manual),
        .i_steer(steer),.i_throttle(throttle),.i_confirm(confirm),.i_back(back),
        .i_aux_up(up),.i_aux_down(down),.i_nav_state(feedback),
        .i_command_idle(command_idle),.i_pause(pause),.i_ctrl_fault(1'b0),
        .o_cur_x_mm(cx),.o_cur_y_mm(cy),.o_aux_sel(),.o_pt_count(count),
        .o_nav_state(state),.o_nav_ui_active(ui),.o_nav_run(run),.o_nav_paused(paused),
        .o_goal_valid(goal),.o_goal_x_cell(gx),.o_goal_y_cell(gy),
        .o_pt_px(),.o_pt_py(),.o_pt_v(),
        .o_car_px(),.o_car_py(),.o_cur_px(),.o_cur_py());
    integer cycles=0,checks=0,errors=0,wait_cycles;
    always @(posedge clk) cycles<=cycles+1;
    task check; input condition;input [511:0] msg;
        begin checks=checks+1; if(condition!==1'b1)begin errors=errors+1;$display("FAIL: %0s",msg);end
        else $display("pass %0s",msg);end
    endtask
    task key;input integer which;
        begin @(negedge clk);case(which)0:confirm=1;1:down=1;2:back=1;3:up=1;endcase
        @(negedge clk);confirm=0;down=0;back=0;up=0;repeat(5)@(negedge clk);end
    endtask
    task finish_test;
        begin $display("CHECKS: %0d",checks);$display("METRICS: obstacle_plan_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors)$display("PASS: obstacle_plan_tb");$finish;end
    endtask
    wire [15:0] route_length;
    nav_top core(
        .forward_check_cells(8'd4), .vehicle_width_mm(16'd180),.clk(clk),.reset_n(rst_n),.pose_valid(1'b1),.pose_x_mm(32'sd1000),.pose_y_mm(32'sd0),
        .goal_valid(goal),.goal_x({8'd0,gx}),.goal_y({8'd0,gy}),.plan_enable(goal),.motion_enable(1'b0),
        .display_enable(1'b0),.map_enable(1'b1),.ctrl_done(1'b0),.ctrl_linear_active(1'b0),
        .obs_scan_start(1'b0),.obs_valid(1'b0),.obs_grid_x(16'sd0),.obs_grid_y(16'sd0),.obs_grid_valid(1'b0),
        .obs_origin_x_mm(32'sd0),.obs_origin_y_mm(32'sd0),.obs_ray_dx_q16(32'sd0),.obs_ray_dy_q16(32'sd0),.obs_range_mm(14'd10000),
        .scan_valid(1'b1),.stream_error(1'b0),.nav_state(feedback),.st_route_length(route_length));

    initial begin
        repeat(4)@(negedge clk);rst_n=1;active=1;repeat(8)@(negedge clk);
        key(0);key(1);key(1);key(0);
        wait_cycles=0;
        while(route_length==0 && wait_cycles<1000000) begin @(negedge clk);wait_cycles=wait_cycles+1;end
        check(goal && gx==148 && gy==128,"UI target reaches actual planner");
        check(feedback==2,"preplanning is paused without motion grant");
        check(route_length>0,"BFS publishes a route on initialized free map");
        key(2);check(paused && goal,"pause keeps planner goal");
        key(2);repeat(10)@(negedge clk);check(!goal && feedback==0,"cancel propagates to planner");
        finish_test();
    end
    initial begin #100000000;$display("FAIL: obstacle_plan_tb timeout");$finish;end
endmodule
