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
            @(negedge clk); publish=1;
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
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_route_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors) $display("PASS: nav_route_tb");
        $finish;
    end
    initial begin #1000000; $display("FAIL: route timeout"); $finish; end
endmodule
