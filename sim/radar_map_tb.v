`timescale 1ns/1ps
module radar_map_tb;
    reg clk=0, rst_n=0, clear=0, ready=0, tick=0, rx=1;
    always #10 clk=~clk;
    reg signed [31:0] x=0,y=0,yaw=0;
    wire valid;
    wire signed [31:0] px,py,pa;
    nav_pose pose(.clk(clk),.rst_n(rst_n),.clear(clear),.ready(ready),
        .position_tick(tick),.x_mm(x),.y_mm(y),.yaw_cdeg(yaw),
        .pose_valid(valid),.pose_x_mm(px),.pose_y_mm(py),.pose_yaw_cdeg(pa));
    radar_navigation dut(.clk(clk),.reset_n(rst_n && !clear),
        .map_enable(valid),.plan_enable(1'b0),.motion_enable(1'b0),
        .radar_rx(rx),.pose_valid(valid),.pose_x_mm(px),.pose_y_mm(py),.pose_yaw_cdeg(pa),
        .goal_valid(1'b0),.goal_x_cell(8'd0),.goal_y_cell(8'd0),
        .ctrl_done(1'b0),.ctrl_linear_active(1'b0),.display_enable(1'b0),
        .cmd_valid(),.emergency_stop(),.target_x_mm(),.target_y_mm(),.nav_state(),
        .map_disp_valid(),.map_disp_occupied(),.map_disp_frame_start(),.map_disp_frame_done(),
        .route_disp_valid(),.route_disp_x_cell(),.route_disp_y_cell(),
        .route_disp_frame_start(),.route_disp_frame_done(),
        .cloud_valid(),.cloud_angle_cdeg(),.cloud_distance_mm());
    integer cycles=0,errors=0,checks=0,j;
    always @(posedge clk) cycles<=cycles+1;
    task check;
        input condition; input [255:0] msg;
        begin checks=checks+1; if(condition!==1'b1) begin errors=errors+1; $display("FAIL: %0s",msg); end else $display("pass %0s",msg); end
    endtask
    task pose_update;
        input signed [31:0] angle;
        begin @(negedge clk); yaw=angle;tick=1; @(negedge clk); tick=0; repeat(3) @(negedge clk); end
    endtask
    task byte_out;
        input [7:0] b; integer k;
        begin
            rx=0;repeat(109) @(negedge clk);
            for(k=0;k<8;k=k+1) begin rx=b[k];repeat(109) @(negedge clk);end
            rx=1;repeat(109) @(negedge clk);
        end
    endtask
    task packet;
        begin
            byte_out(8'ha5);byte_out(8'h5a);byte_out(0);byte_out(0);
            byte_out(0);byte_out(60);
            byte_out(8'h02);byte_out(8'h26);
            for(j=1;j<42;j=j+1) begin byte_out(8'hff);byte_out(8'hff);end
            repeat(10) byte_out(0);
            byte_out(8'hfa);byte_out(8'hfb);
            repeat(3000) @(negedge clk);
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        repeat(66000) @(negedge clk);
        check(!valid,"pose held invalid before ready");ready=1;
        pose_update(0);check(valid && px==0 && py==0 && pa==0,"coherent initial pose");
        packet();check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+133],"forward radar maps +x");
        pose_update(9000);packet();check(dut.u_adapter.u_navigation.u_nav.u_map.raw[133*256+128],"left yaw maps +y");
        pose_update(18000);check(pa==-18000,"positive 180 normalized");
        packet();check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+122],"rear heading maps -x");
        pose_update(-9000);packet();check(dut.u_adapter.u_navigation.u_nav.u_map.raw[122*256+128],"right yaw maps -y");
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+133],"history retained across scans");
        clear=1;repeat(2) @(negedge clk);check(!valid,"clear invalidates pose");clear=0;
        repeat(66000) @(negedge clk);
        check(!dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+133],"clear resets old map");
        $display("CHECKS: %0d",checks);
        $display("METRICS: radar_map cycles=%0d checks=%0d errors=%0d",cycles,checks,errors);
        if(!errors) $display("PASS: radar_map");
        $finish;
    end
    initial begin #30000000;$display("FAIL: radar_map timeout");$finish;end
endmodule
