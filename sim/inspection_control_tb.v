`timescale 1ns/1ps
module inspection_control_tb;
    reg clk=0,rst=0,permission=0,manual=0,safety=1,link=1;
    reg [31:0] epoch=1,session=5,task_id=1,command=0,request_epoch=1;
    reg [15:0] user_token=0,token=0,requested_layer=1;
    reg [15:0] cruise=300;
    wire [15:0] accepted_cruise;
    reg hello=0,control=0,heartbeat=0,layer_begin=0,commissioned=0,internal_pause=0;
    reg [7:0] op=1,gx=140,gy=128;
    reg done=0,idle=1,fault=0,local_pause=0;reg [3:0] nav_state=1;
    reg layer_valid=1,layer_committed=0,layer_error=0;
    wire owned,goal,motion,force_stop,stopped,upload,caps;
    wire [7:0] goal_x,goal_y,status,error_code;
    wire [31:0] current_session,current_task,current_command;
    integer checks=0,errors=0;
    always #10 clk=~clk;
    inspection_nav_control #(.LEASE_CYCLES(1000)) dut(
        .clk(clk),.rst_n(rst),.i_permission(permission),.i_manual(manual),.i_safety(safety),.i_link(link),.i_local_pause(local_pause),
        .i_epoch(epoch),.i_user_token(user_token),.i_hello(hello),.i_control(control),.i_heartbeat(heartbeat),.i_layer_begin(layer_begin),
        .i_session(session),.i_task(task_id),.i_command(command),.i_request_epoch(request_epoch),
        .i_op(op),.i_cruise_mm_s(cruise),.o_cruise_mm_s(accepted_cruise),.i_commissioned(commissioned),.i_internal_pause(internal_pause),
        .i_token(token),.i_requested_layer(requested_layer),.i_goal_x(gx),.i_goal_y(gy),
        .i_ctrl_done(done),.i_command_idle(idle),.i_ctrl_fault(fault),.i_nav_state(nav_state),
        .i_layer_valid(layer_valid),.i_layer_committed(layer_committed),.i_layer_error(layer_error),
        .i_layer_version(16'd1),.i_layer_error_code(4'd0),
        .o_owned(owned),.o_goal_valid(goal),.o_motion_enable(motion),.o_goal_x(goal_x),.o_goal_y(goal_y),
        .o_force_stop(force_stop),.o_stopped(stopped),.o_upload_allowed(upload),.o_session(current_session),
        .o_task(current_task),.o_command(current_command),.o_status(status),.o_error_code(error_code),.o_caps_request(caps));
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send;input [7:0] operation;
        begin @(negedge clk);command=command+1;op=operation;control=1;@(negedge clk);control=0;repeat(2)@(negedge clk);end
    endtask
    task actual_stop;
        begin done=0;repeat(3)@(negedge clk);done=1;repeat(3)@(negedge clk);end
    endtask
    task release_stop;
        begin done=0;repeat(4)@(negedge clk);end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        hello=1;@(negedge clk);hello=0;repeat(2)@(negedge clk);
        check(current_session==5 && !owned && !motion,"hello cannot claim motor authority outside AI task");
        permission=1;repeat(3)@(negedge clk);check(owned && force_stop && !goal,"page entry claims zero and requests real stop");
        actual_stop();check(stopped && upload,"actual done low-high allows stationary upload");
        user_token=1;token=1;send(1);check(!motion && status==5 && error_code==17,"uncommissioned goal is refused");
        commissioned=1;user_token=2;token=2;send(1);
        check(!motion && !force_stop,"valid goal waits for old stop handshake release");
        release_stop();check(motion && goal && goal_x==140 && accepted_cruise==300,"physical confirm admits goal and full 300mm/s cruise");
        nav_state=3;actual_stop();check(stopped && status==3,"first waypoint arrival reports actual stopped");
        nav_state=1;gx=150;send(1);release_stop();
        check(motion && goal_x==150 && current_task==1,"second waypoint reuses authorized task and physical token");
        nav_state=3;actual_stop();check(stopped && status==3,"second waypoint also waits for actual stopped");
        nav_state=1;gx=160;send(1);release_stop();
        check(motion && goal_x==160 && current_task==1,"third waypoint continues without another physical confirm");
        internal_pause=1;send(2);actual_stop();check(stopped && !motion,"internal visual pause waits for actual stopped");
        internal_pause=0;send(1);release_stop();check(motion,"internal layer pause may continue same authorized task");
        task_id=2;send(1);actual_stop();check(!motion && status==5,"armed old task cannot authorize a new task with reused token");
        user_token=5;token=5;send(1);release_stop();check(motion,"new task requires fresh physical confirmation");
        local_pause=1;@(negedge clk);local_pause=0;actual_stop();
        check(!motion && stopped,"local pause revokes goal without a host round trip");
        user_token=6;token=6;send(1);release_stop();check(motion,"fresh confirmation resumes locally paused task");
        manual=1;@(negedge clk);check(!motion && !goal,"manual revokes autonomous output without RK round trip");
        done=0;repeat(3)@(negedge clk);manual=0;actual_stop();
        check(!motion && stopped,"cursor return never resumes old task");
        send(1);check(!motion && status==5,"old physical token cannot resume after manual");
        user_token=3;token=3;send(1);release_stop();check(motion,"new confirmation can resume after manual");
        send(2);actual_stop();check(!motion && stopped,"operator pause still requests a real stop");
        user_token=4;token=4;cruise=500;send(1);release_stop();check(motion && accepted_cruise==500,"next accepted goal updates RK cruise without a speed-code gate");
        repeat(1005)@(negedge clk);check(!motion && force_stop,"local lease timeout brakes independently");
        actual_stop();epoch=2;request_epoch=1;@(negedge clk);repeat(2)@(negedge clk);
        send(1);check(!motion,"old localization epoch cannot authorize movement");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_control %0d checks",checks);else $display("FAIL: inspection_control %0d",errors);
        $finish;
    end
    initial begin #1000000;$display("FAIL: inspection_control timeout");$finish;end
endmodule
