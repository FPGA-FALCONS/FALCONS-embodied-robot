`timescale 1ns/1ps
module obstacle_page_tb;
    `include "nav_obstacle_states.vh"
    reg clk = 0, rst_n = 0, active = 0, manual = 0, clear = 0;
    reg confirm = 0, back = 0, up = 0, down = 0, pause = 0, fault = 0;
    always #10 clk = ~clk;
    reg signed [11:0] steer = 0, throttle = 0;
    reg signed [31:0] x = 1000, y = 0;
    reg [3:0] nav_status = 0;
    reg command_idle = 1;
    wire [2:0] state, aux;
    wire [4:0] count;
    wire [15:0] points;
    wire run, paused, ui, goal;
    wire [7:0] gx, gy;
    wire [9:0] car_px,car_py,cur_px,cur_py;
    wire signed [31:0] cx, cy;
    integer cycles = 0, checks = 0, errors = 0, n;
    reg signed [31:0] saved_x;
    always @(posedge clk) cycles <= cycles + 1;
    nav_obstacle_ui #(.CLK_HZ(1000)) dut (
        .clk(clk), .rst_n(rst_n), .i_active(active), .i_clear(clear),
        .i_x_mm(x), .i_y_mm(y), .i_manual(manual),
        .i_steer(steer), .i_throttle(throttle), .i_confirm(confirm), .i_back(back),
        .i_aux_up(up), .i_aux_down(down), .i_nav_state(nav_status),
        .i_command_idle(command_idle), .i_pause(pause), .i_ctrl_fault(fault),
        .o_cur_x_mm(cx), .o_cur_y_mm(cy), .o_aux_sel(aux), .o_pt_count(count),
        .o_nav_state(state), .o_nav_ui_active(ui), .o_nav_run(run), .o_nav_paused(paused),
        .o_goal_valid(goal), .o_goal_x_cell(gx), .o_goal_y_cell(gy),
        .o_pt_px(), .o_pt_py(), .o_pt_v(points),
        .o_car_px(car_px), .o_car_py(car_py), .o_cur_px(cur_px), .o_cur_py(cur_py)
    );
    task tick;
        input integer clocks;
        begin repeat (clocks) @(negedge clk); end
    endtask
    task check;
        input condition;
        input [639:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s state=%0d count=%0d goal=%0b", message, state, count, goal);
            end else $display("pass %0s", message);
        end
    endtask
    task key;
        input integer which;
        begin
            @(negedge clk);
            case (which) 0: confirm=1; 1: down=1; 2: back=1; 3: up=1; endcase
            @(negedge clk); confirm=0; down=0; back=0; up=0;
            tick(5);
        end
    endtask
    task arrive;
        begin nav_status=3; tick(3); nav_status=0; tick(6); end
    endtask
    task start;
        begin key(1); key(1); key(0); tick(6); end
    endtask
    initial begin
        tick(4); rst_n=1; active=1; tick(8);
        check(state==OBS_EDIT && ui && count==0 && aux==0, "page enters a clean editor");
        check(cx==1000 && cy==0, "cursor initializes from current pose");
        check(car_px==471 && car_py==300 && cur_px==471 && cur_py==300, "pose and cursor share sixty pixels per metre");
        start(); check(!run && !goal, "empty task cannot start");
        key(3); key(3); key(0);
        check(count==1 && points==1, "confirm stores exactly one destination");
        throttle=1000; tick(2000); throttle=0; tick(4); key(0);
        check(count==2 && points==3, "second point keeps contiguous validity mask");
        key(1); key(1); command_idle=0; key(0);
        check(run && !goal, "start waits for old command release");
        command_idle=1; tick(6);
        check(goal && gx==148 && gy==128, "first goal converts to map cells");
        nav_status=4; tick(4);
        check(state==OBS_NO_PATH && run && goal, "no path retains running task");
        nav_status=1; tick(4);
        check(state==OBS_RUN, "new route recovers without extra confirmation");
        saved_x=cx; steer=1000; tick(100); steer=0; key(1);
        check(cx==saved_x && aux==2 && count==2, "running locks cursor and editor");
        key(0); check(paused && goal, "confirm pauses while preserving goal");
        pause=1; key(0); check(paused, "pause condition blocks resume");
        pause=0; tick(6); check(paused, "removing stop condition does not auto resume");
        manual=1; key(0); check(paused, "manual mode cannot resume");
        x=1750; y=500; tick(6); manual=0; tick(6);
        check(paused && goal && gx==148 && gy==128, "manual movement preserves destination, not an arrival");
        key(0); check(run, "explicit confirmation resumes existing task");
        nav_status=3; command_idle=0; tick(3);
        check(!goal, "intermediate arrival withdraws old goal");
        tick(8); check(!goal, "next goal still waits for command release");
        nav_status=0; command_idle=1; tick(6);
        check(goal && gx==148 && gy==147, "next destination follows old release");
        fault=1; tick(1); fault=0; tick(6);
        check(state==OBS_FAULT && !run && goal && count==2, "fault holds current task and reports fault");
        manual=1; key(0); check(state==OBS_FAULT, "manual switch keeps fault state");
        manual=0; pause=1; key(0); check(state==OBS_FAULT, "stop condition blocks retry");
        pause=0; key(0); tick(6);
        check(run && goal && gy==147 && dut.goal_index==1, "fault retry stays at second destination");
        arrive();
        check(state==OBS_DONE && !goal && !run && ui && count==2, "final arrival releases task and preserves read-only result");
        saved_x=cx; steer=1000; throttle=1000; tick(100); steer=0; throttle=0;
        key(1); key(0); key(3);
        check(state==OBS_DONE && count==2 && cx==saved_x && !goal, "done ignores cursor, confirm and auxiliary keys");
        manual=1; tick(4); manual=0; tick(4);
        check(state==OBS_DONE, "manual switch cannot turn a finished task into pause");
        key(2);
        check(state==OBS_EDIT && ui && count==0 && aux==0 && cx==1750 && cy==500,
              "one back opens a clean editor at current pose");
        key(0); start();
        check(run && goal && gx==163 && gy==138, "second navigation submits new destination without old points");
        arrive(); check(state==OBS_DONE && !goal, "second task finishes independently");
        key(2); key(0); start();
        manual=1; tick(4);
        check(paused && goal, "running manual takeover pauses");
        manual=0; tick(4); check(paused, "returning to cursor mode stays paused");
        key(2);
        check(state==OBS_BROWSE && !ui && !goal && count==0 && aux==0, "cancel clears all task state and exits interaction");
        key(0);
        check(state==OBS_EDIT && count==0 && aux==0, "reenter only opens editor, no duplicate click");
        key(0); key(1); key(0);
        check(count==0 && points==0, "undo removes the final point");
        key(2); key(0); key(0); start();
        fault=1; tick(1); fault=0; tick(4); key(2);
        check(state==OBS_BROWSE && !goal && count==0, "fault cancellation clears task");
        key(0);
        for (n=0; n<17; n=n+1) key(0);
        check(count==16 && points==16'hffff, "full list ignores seventeenth destination");
        key(3); key(0);
        check(count==0 && points==0, "clear menu empties list");
        key(1); steer=1000; tick(30000); steer=0; tick(4);
        check(cx==-3400, "cursor clamps at left boundary");
        steer=-1000; tick(45000); steer=0; tick(4);
        check(cx==6375, "cursor clamps at right boundary");
        throttle=1000; tick(15000); throttle=0; tick(4);
        check(cy==5000, "cursor clamps at upper boundary");
        throttle=-1000; tick(25000); throttle=0; tick(4);
        check(cy==-4983, "cursor clamps at lower boundary");
        key(0); start(); active=0; tick(5);
        check(state==OBS_BROWSE && !run && !goal && count==0, "forced page change cancels task");
        active=1; tick(6);
        check(state==OBS_EDIT && count==0 && aux==0, "page return has the same fresh-editor behavior");
        clear=1; tick(1); clear=0; tick(4);
        check(state==OBS_BROWSE && !ui && !goal && count==0 && cx==0 && cy==0, "global clear releases UI and origin cursor");
        $display("CHECKS: %0d", checks);
        $display("METRICS: obstacle_page_tb cycles=%0d errors=%0d", cycles, errors);
        if (!errors) $display("PASS: obstacle_page_tb");
        $finish;
    end
    initial begin #100000000; $display("FAIL: obstacle_page_tb timeout"); $finish; end
endmodule
