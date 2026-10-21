`timescale 1ns/1ps
module obstacle_page_tb;
    reg clk=0,rst_n=0,active=0,manual=0,clear=0,confirm=0,back=0,up=0,down=0,pause=0;
    always #10 clk=~clk;
    reg signed [11:0] steer=0,throttle=0;
    reg [3:0] nav_status=0;
    reg command_idle=1;
    wire [3:0] feedback;
    wire [1:0] state;
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
    integer cycles=0,checks=0,errors=0;
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
        begin $display("CHECKS: %0d",checks);$display("METRICS: obstacle_page_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors)$display("PASS: obstacle_page_tb");$finish;end
    endtask
    assign feedback=nav_status;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;active=1;repeat(8)@(negedge clk);
        check(ui && count==0,"page enters setup");
        key(1);key(1);key(0);check(!run,"empty list cannot start");
        key(3);key(3);key(0);check(count==1,"confirm stores first destination");
        throttle=1000;repeat(2000)@(negedge clk);throttle=0;repeat(4)@(negedge clk);
        key(0);check(count==2,"second destination uses existing list");
        key(1);key(1);command_idle=0;key(0);
        check(run && !goal,"start waits for released old request");
        command_idle=1;repeat(6)@(negedge clk);
        check(goal && gx==138 && gy==128,"first goal is converted to map cell");
        nav_status=4;repeat(6)@(negedge clk);check(goal && gx==138 && gy==128,"no path does not skip goal");
        nav_status=0;key(2);check(paused && goal,"pause retains destination");
        manual=1;key(0);check(paused,"manual mode cannot resume");manual=0;key(0);check(run,"explicit resume");
        nav_status=3;command_idle=0;repeat(3)@(negedge clk);check(!goal,"arrival withdraws old goal");
        nav_status=0;command_idle=1;repeat(6)@(negedge clk);
        check(goal && gx==138 && gy==137,"next destination advances after release");
        nav_status=3;repeat(8)@(negedge clk);check(goal && dut.goal_index==1,"last goal retains arrived result");
        key(2);key(2);repeat(3)@(negedge clk);check(!ui && !goal,"cancel releases goal");
        key(0);key(1);key(0);check(count==0,"clear menu clears the list");
        steer=1000;repeat(30000)@(negedge clk);steer=0;repeat(4)@(negedge clk);
        check(cx==-3400,"cursor clamps at left boundary");
        steer=-1000;repeat(40000)@(negedge clk);steer=0;repeat(4)@(negedge clk);
        check(cx==10183,"cursor clamps at right boundary");
        throttle=1000;repeat(15000)@(negedge clk);throttle=0;repeat(4)@(negedge clk);
        check(cy==5000,"cursor clamps at upper boundary");
        throttle=-1000;repeat(25000)@(negedge clk);throttle=0;repeat(4)@(negedge clk);
        check(cy==-4983,"cursor clamps at lower boundary");
        key(1);check(count==0,"auxiliary selection does not create a point");
        finish_test();
    end
    initial begin #100000000;$display("FAIL: obstacle_page_tb timeout");$finish;end
endmodule
