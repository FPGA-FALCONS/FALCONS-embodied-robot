`timescale 1ns/1ps
module nav_route_tb;
    reg clk=0,reset_n=0,start=0,abort=0,publish=0;
    always #10 clk=~clk;
    reg in_valid=0,in_last=0,prefix=1;
    reg [15:0] in_x=128,in_y=128,index=0;
    reg [2:0] direction=0;
    wire in_ready,sv,sr,sl,complete,active,drive_valid;
    wire [15:0] sx,sy,length,target_x,target_y,drive_x,drive_y,drive_index,route_id;
    wire [2:0] sd,target_dir;
    integer cycles=0,checks=0,errors=0,d,dx,dy,nx,ny;
    always @(posedge clk) cycles<=cycles+1;
    nav_segments segments (
        .clk(clk),.reset_n(reset_n),.start(start),.cancel(abort),
        .start_x(16'd128),.start_y(16'd128),.start_dir_valid(prefix),.start_dir(direction),
        .in_valid(in_valid),.in_ready(in_ready),.in_x(in_x),.in_y(in_y),.in_last(in_last),
        .out_valid(sv),.out_ready(sr),.out_x(sx),.out_y(sy),.out_last(sl),.out_dir(sd),.busy()
    );
    nav_route #(.MAX_WP(16)) route (
        .clk(clk),.reset_n(reset_n),.load_begin(start),.load_abort(abort),
        .load_valid(sv),.load_ready(sr),.load_last(sl),.load_x(sx),.load_y(sy),.load_dir(sd),
        .publish(publish),.active_valid(active),.active_length(length),.route_id(route_id),
        .target_index(index),.target_x(target_x),.target_y(target_y),.target_dir(target_dir),
        .drive_x(drive_x),.drive_y(drive_y),.drive_index(drive_index),.final_target(),.drive_valid(drive_valid),
        .candidate_complete(complete),.candidate_length(),.candidate_index(16'd0),
        .candidate_x(),.candidate_y(),.candidate_valid(),.load_full(),
        .display_enable(1'b0),.display_refresh(1'b0),.display_route_valid(1'b0),
        .route_disp_valid(),.route_disp_x_cell(),.route_disp_y_cell(),
        .route_disp_frame_start(),.route_disp_frame_done()
    );
    task check;
        input condition;
        input [511:0] text;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin errors=errors+1; $display("FAIL: route dir=%0d %0s",direction,text); end
            else $display("pass route dir=%0d %0s",direction,text);
        end
    endtask
    task point;
        input [15:0] x,y;
        input last;
        begin
            @(negedge clk);
            in_x=x; in_y=y; in_last=last; in_valid=1;
            while(!in_ready) @(negedge clk);
            @(negedge clk); in_valid=0;
        end
    endtask
    wire edge_one_done, edge_five_done, edge_full_done;
    wire [31:0] edge_one_checks, edge_five_checks, edge_full_checks;
    wire [31:0] edge_one_errors, edge_five_errors, edge_full_errors;
    nav_route_address_check #(.CAPACITY(1)) edge_one(edge_one_done, edge_one_checks, edge_one_errors);
    nav_route_address_check #(.CAPACITY(5)) edge_five(edge_five_done, edge_five_checks, edge_five_errors);
    nav_route_address_check #(.CAPACITY(1024)) edge_full(edge_full_done, edge_full_checks, edge_full_errors);
    initial begin
        repeat(3) @(negedge clk); reset_n=1;
        for(d=0;d<8;d=d+1) begin
            @(negedge clk); direction=d; start=1; index=0;
            @(negedge clk); start=0;
            case(d) 0,1,7:dx=1; 3,4,5:dx=-1; default:dx=0; endcase
            case(d) 1,2,3:dy=1; 5,6,7:dy=-1; default:dy=0; endcase
            case((d+1)%8) 0,1,7:nx=1; 3,4,5:nx=-1; default:nx=0; endcase
            case((d+1)%8) 1,2,3:ny=1; 5,6,7:ny=-1; default:ny=0; endcase
            point(128,128,0);
            point(128+4*dx,128+4*dy,0);
            point(128+8*dx,128+8*dy,0);
            point(128+8*dx+4*nx,128+8*dy+4*ny,1);
            wait(complete);
            if (d > 0) wait(drive_valid);
            @(negedge clk); publish=1;
            #1;
            if (d > 0) check(drive_valid, "publication request does not feed back into the current descriptor");
            @(negedge clk); publish=0;
            wait(drive_valid);
            @(negedge clk);
            check(active && length==4 && target_x==128 && target_y==128 && target_dir==d,
                  "locked prefix remains a separate waypoint in every direction");
            check(drive_x==128+8*dx && drive_y==128+8*dy && drive_index==2,
                  "output extends to farthest collinear point without merging locked prefix");
            index=1;
            @(negedge clk);
            wait(drive_valid);
            @(negedge clk);
            check(target_x==128+4*dx && target_y==128+4*dy && drive_index==2,
                  "advancing the short-segment index retains its own endpoint");
            index=3;
            @(negedge clk);
            wait(drive_valid);
            @(negedge clk);
            check(target_x==128+8*dx+4*nx && target_y==128+8*dy+4*ny && target_dir==(d+1)%8,
                  "next corner retains its independent direction");
            start=1;
            @(negedge clk); start=0;
            point(100,100,0);
            @(negedge clk); abort=1;
            @(negedge clk); abort=0;
            repeat(3) @(negedge clk);
            check(active && length==4 && target_dir==(d+1)%8 && route_id==d+1,
                  "aborted candidate never overwrites the active route");
        end
        // 初次规划没有注入前缀，也必须保留所有限长中继；查找不能只处理前两点。
        @(negedge clk); prefix=0; start=1; index=0;
        @(negedge clk); start=0;
        for(d=1;d<=8;d=d+1) point(128+4*d,128,d==8);
        wait(complete);
        @(negedge clk); publish=1;
        @(negedge clk); publish=0;
        wait(drive_valid);
        @(negedge clk);
        check(length==8 && target_x==132 && drive_x==160 && drive_index==7,
              "initial long straight keeps eight short segments but outputs the far endpoint");
        index=4;
        @(negedge clk);
        wait(drive_valid);
        @(negedge clk);
        check(target_x==148 && drive_x==160 && drive_index==7,
              "lookahead also works from a middle short segment");
        wait(edge_one_done && edge_five_done && edge_full_done);
        checks=checks+edge_one_checks+edge_five_checks+edge_full_checks;
        errors=errors+edge_one_errors+edge_five_errors+edge_full_errors;
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_route_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors) $display("PASS: nav_route_tb");
        $finish;
    end
    initial begin #1000000; $display("FAIL: route timeout"); $finish; end
endmodule

// 容量 1、非二次幂和生产容量：核对地址缩位后的末点、满深度和最后索引。
module nav_route_address_check #(
    parameter integer CAPACITY = 1
) (
    output reg done = 0,
    output integer checks = 0,
    output integer errors = 0
);
    reg clk=0, reset_n=0, begin_load=0, valid=0, last=0, publish=0;
    always #10 clk=~clk;
    reg [15:0] x=0, index=0;
    wire ready, complete, active, drive_valid, final_target;
    wire [15:0] length, target_x, drive_x, drive_index;
    integer n;
    nav_route #(.MAX_WP(CAPACITY)) dut (
        .clk(clk), .reset_n(reset_n), .load_begin(begin_load), .load_abort(1'b0),
        .load_valid(valid), .load_last(last), .load_ready(ready), .load_full(),
        .load_x(x), .load_y(16'd0), .load_dir(3'd0), .publish(publish),
        .active_valid(active), .active_length(length), .route_id(), .target_index(index),
        .target_x(target_x), .target_y(), .target_dir(), .final_target(final_target),
        .drive_x(drive_x), .drive_y(), .drive_index(drive_index), .drive_valid(drive_valid),
        .candidate_complete(complete), .candidate_length(), .candidate_index(16'd0),
        .candidate_x(), .candidate_y(), .candidate_valid(),
        .display_enable(1'b0), .display_refresh(1'b0), .display_route_valid(1'b0),
        .route_disp_valid(), .route_disp_x_cell(), .route_disp_y_cell(),
        .route_disp_frame_start(), .route_disp_frame_done()
    );
    task check;
        input condition;
        input [511:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin errors=errors+1; $display("FAIL: capacity=%0d %0s",CAPACITY,message); end
            else $display("pass capacity=%0d %0s",CAPACITY,message);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); reset_n=1; begin_load=1;
        @(negedge clk); begin_load=0;
        @(negedge clk);
        for(n=1;n<=CAPACITY;n=n+1) begin
            x=n; valid=1; last=n==CAPACITY;
            while(!ready) @(negedge clk);
            @(negedge clk);
        end
        valid=0;
        wait(complete);
        @(negedge clk); publish=1;
        @(negedge clk); publish=0;
        wait(drive_valid);
        @(negedge clk);
        check(active && length==CAPACITY && target_x==1 && drive_x==CAPACITY &&
              drive_index==CAPACITY-1 && final_target,
              "full-depth forward scan registers the terminal point without wrapping");
        index=CAPACITY-1;
        @(negedge clk);
        wait(drive_valid);
        @(negedge clk);
        check(target_x==CAPACITY && drive_x==CAPACITY && final_target,
              "last target index reads the final point directly");
        done=1;
    end
endmodule
