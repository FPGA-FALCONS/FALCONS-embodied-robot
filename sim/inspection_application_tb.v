`timescale 1ns/1ps
module inspection_application_tb;
    reg clk=0,rst=0,rxvalid=0,permission=0,manual=0,safety=1,done=0;
    reg [7:0] rxdata=0,vector[0:371];
    reg local_pause=0,confirm=0,back=0,ui_open=0;
    reg [15:0] token=0;
    wire ready,owned,goal,motion,force_stop,pending,stopped,txvalid;
    wire [7:0] gx,gy,status,txdata;
    wire [15:0] cruise;
    wire ui_close;
    integer checks=0,errors=0,i,stop_wait=0,txbytes=0,closes=0,input_snapshots=0;
    always #10 clk=~clk;
    // Model the point controller's STOP completion and release, not the RK
    // transport's bytes-written acknowledgment.
    always @(posedge clk)begin
        if(force_stop && !manual)begin
            if(stop_wait<6)stop_wait<=stop_wait+1;
            else done<=1;
        end else begin stop_wait<=0;done<=0;end
        if(txvalid)txbytes=txbytes+1;
        if(ui_close)closes=closes+1;
        if(dut.tx_start && dut.tx_kind==8'h83)input_snapshots=input_snapshots+1;
    end
    inspection_application #(.CLK_HZ(1_000_000),.UI_ENABLE(1)) dut(
        .clk(clk),.rst_n(rst),.i_rx_data(rxdata),.i_rx_valid(rxvalid),.o_rx_ready(ready),
        .i_permission(permission),.i_manual(manual),.i_safety(safety),.i_link(1'b1),.i_local_pause(local_pause),
        .i_user_token(token),.i_epoch(32'd1),.i_pose_tick(1'b0),.i_pose_valid(1'b1),
        .i_ui_open(ui_open),.i_ui_master(1'b1),.i_ui_record(1'b0),.i_ui_rc_valid(1'b1),
        .i_ui_confirm(confirm),.i_ui_back(back),.i_ui_view(2'd1),.i_ui_steer(-12'sd500),.i_ui_throttle(12'sd800),
        .i_vehicle_width_mm(16'd180),.i_forward_check_cells(8'd4),.o_ui_close(ui_close),
        .i_x_mm(32'sd0),.i_y_mm(32'sd0),.i_yaw_cdeg(32'sd0),
        .i_ctrl_done(done),.i_command_idle(!done),.i_ctrl_fault(1'b0),.i_nav_state(4'd1),
        .o_owned(owned),.o_goal_valid(goal),.o_motion_enable(motion),.o_force_stop(force_stop),.o_pending_motion(pending),
        .o_goal_x(gx),.o_goal_y(gy),.o_layer_begin(),.o_layer_commit(),.o_layer_word_valid(),
        .o_cruise_mm_s(cruise),
        .o_layer_epoch(),.o_layer_crc(),.o_layer_word(),.o_layer_version(),.o_layer_word_index(),
        .i_layer_ready(1'b1),.i_layer_valid(1'b1),.i_layer_committed(1'b0),.i_layer_error(1'b0),.i_layer_busy(1'b0),
        .i_layer_version(16'd1),.i_layer_error_code(4'd0),.o_stopped(stopped),.o_status(status),
        .o_tx_data(txdata),.o_tx_valid(txvalid),.o_tx_last(),.i_tx_ready(1'b1));
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task feed;input integer first,last;
        begin for(i=first;i<=last;i=i+1)begin
            wait(ready);@(negedge clk);rxdata=vector[i];rxvalid=1;
            @(negedge clk);rxvalid=0;repeat(8)@(negedge clk);
        end repeat(100)@(negedge clk);end
    endtask
    initial begin
        $readmemh("vectors/inspection_application.hex",vector);
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        feed(0,35);check(!owned && txbytes>0,"host HELLO produces capability transaction without authority");
        permission=1;ui_open=1;repeat(25)@(negedge clk);token=1;confirm=1;@(negedge clk);confirm=0;
        check(owned && stopped && !motion,"task entry completes actual stop before host goal");
        feed(36,155);check(motion && goal && gx==140 && gy==128 && cruise==300,"Python transaction reaches registered goal and 16-bit 300mm/s cruise");
        check(input_snapshots>0 && dut.confirms==1 && dut.backs==0,"physical input snapshot retains confirmation and joystick axes");
        feed(156,227);check(closes==0 && motion,"host cannot close child page while chassis is moving");
        local_pause=1;@(negedge clk);local_pause=0;repeat(25)@(negedge clk);
        feed(156,227);check(closes==1 && stopped,"rejected moving close revision can retry after actual stopped");
        ui_open=0;repeat(3)@(negedge clk);ui_open=1;repeat(3)@(negedge clk);
        feed(156,227);check(closes==1,"old page close cannot close a newly entered page");
        manual=1;repeat(3)@(negedge clk);
        check(!motion && !goal,"manual revokes application output locally");
        manual=0;repeat(25)@(negedge clk);check(stopped && !motion,"cursor return holds current task stopped");
        safety=0;repeat(3)@(negedge clk);check(!motion,"lost sensor health cannot release motion");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_application %0d checks",checks);else $display("FAIL: inspection_application %0d",errors);
        $finish;
    end
    initial begin #2000000;$display("FAIL: inspection_application timeout");$finish;end
endmodule
