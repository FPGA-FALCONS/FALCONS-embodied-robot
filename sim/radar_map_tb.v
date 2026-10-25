`timescale 1ns/1ps
module radar_map_tb;
    reg clk=0, rst_n=0, clear=0, ready=0, tick=0, rx=1;
    always #10 clk=~clk;
    reg signed [31:0] x=0,y=0,yaw=0;
    wire valid;
    wire signed [31:0] px,py,pa;
    nav_pose pose(.clk(clk),.rst_n(rst_n),.clear(clear),.ready(ready),
        .position_tick(tick),.x_mm(x),.y_mm(y),.yaw_cdeg(yaw),
        .pose_valid(valid),.pose_tick(),.pose_x_mm(px),.pose_y_mm(py),.pose_yaw_cdeg(pa));
    radar_navigation dut(
        .forward_check_cells(8'd4), .vehicle_width_mm(16'd180),.clk(clk),.reset_n(rst_n && !clear),
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
        input condition; input [511:0] msg;
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
        input [15:0] angle, distance;
        input invalid_first;
        begin
            byte_out(8'ha5);byte_out(8'h5a);byte_out(angle[15:8]);byte_out(angle[7:0]);
            byte_out(0);byte_out(60);
            for(j=0;j<42;j=j+1) begin
                if(j == (invalid_first ? 1 : 0)) begin
                    byte_out(distance[15:8]);byte_out(distance[7:0]);
                end else begin byte_out(8'hff);byte_out(8'hff);end
            end
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
        packet(0,550,0);check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+139],"forward radar maps +x");
        pose_update(9000);packet(0,550,0);check(dut.u_adapter.u_navigation.u_nav.u_map.raw[139*256+128],"left yaw maps +y");
        pose_update(18000);check(pa==-18000,"positive 180 normalized");
        packet(0,550,0);check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+117],"rear heading maps -x");
        pose_update(-9000);packet(0,550,0);check(dut.u_adapter.u_navigation.u_nav.u_map.raw[117*256+128],"right yaw maps -y");
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+139],"history retained across scans");
        pose_update(0);
        packet(34500,16'hffff,0);
        packet(0,550,1);
        packet(1500,1500,0);
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[125*256+138],
              "near hit survives farther same-circle ray");
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[120*256+156],
              "far hit enters map immediately without waiting for next circle");
        packet(34500,16'hffff,0);
        packet(0,1500,1);
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[125*256+138],
              "clear evidence waits until its circle ends");
        packet(34500,16'hffff,0);
        packet(0,16'hffff,0);
        // 换圈首包全是填充槽，下一包的有效射线仍必须携带换圈标记。
        packet(1500,1500,0);
        repeat(500000) @(negedge clk);
        check(!dut.u_adapter.u_navigation.u_nav.u_map.raw[125*256+138],
              "next completed circle clears old hit despite invalid boundary slots");
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[120*256+156],
              "new far obstacle survives deferred cleanup");
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[139*256+128],
              "unobserved side obstacle remains across circle cleanup");
        clear=1;repeat(2) @(negedge clk);check(!valid,"clear invalidates pose");clear=0;
        repeat(66000) @(negedge clk);
        check(!dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+139],"clear resets old map");
        pose_update(0);
        packet(0,49,0);
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+128],
              "49 mm front hit belongs to center cell");
        check(dut.u_adapter.u_navigation.u_nav.u_map.cost[132*256+132] &&
              !dut.u_adapter.u_navigation.u_nav.u_map.cost[128*256+133],
              "inflation extends four cells (200 mm), not five");
        packet(0,50,0);
        check(dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+129],
              "50 mm front hit enters next cell");
        packet(0,10000,0);
        check(!dut.u_adapter.u_navigation.u_nav.u_map.raw[128*256+255],
              "10 m no-echo ray clips at map edge without creating boundary hit");
        $display("CHECKS: %0d",checks);
        $display("METRICS: radar_map cycles=%0d checks=%0d errors=%0d",cycles,checks,errors);
        if(!errors) $display("PASS: radar_map");
        $finish;
    end
    initial begin #100000000;$display("FAIL: radar_map timeout");$finish;end
endmodule
