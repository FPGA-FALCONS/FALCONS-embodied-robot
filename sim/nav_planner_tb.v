`timescale 1ns/1ps
// 独立参考逐格检查八方向压缩航点：相邻点只能共轴或呈45°，斜步检查两侧格。
module nav_planner_tb;
    reg clk=0, reset_n=0, request=0, escape=0;
    always #10 clk=~clk;
    reg [15:0] sx,sy,gx,gy,limit=512;
    wire ready,busy,success,no_path,rd,valid,last;
    wire [15:0] rx,ry,x,y,length;
    reg response=0,raw=0,cost=0,pending=0;
    reg [1:0] map [0:65535];
    reg [15:0] reference [0:4095];
    integer cycles=0,checks=0,errors=0,delay_left,address;
    integer fd,result,count,index,ex,position,begin_cycle,points,wait_count;
    integer px,py,dx,dy,step_x,step_y,n,waypoints,metrics;
    reg geometry_ok,reference_ok,stable_ok,held=0;
    reg [32:0] hold_data;
    reg [2047:0] fixture_dir,file_name;
    wire rd_ready=!pending && cycles%5!=0;
    wire take=cycles%7>=2;
    nav_planner #(.GRID_W(256),.GRID_H(256)) dut (
        .clk(clk),.reset_n(reset_n),.req_valid(request),.req_ready(ready),
        .allow_start_escape(escape),.start_x(sx),.start_y(sy),.goal_x(gx),.goal_y(gy),
        .mrd_valid(rd),.mrd_ready(rd_ready),.mrd_x(rx),.mrd_y(ry),
        .mrd_rsp_valid(response),.mrd_raw(raw),.mrd_cost(cost),
        .wp_valid(valid),.wp_ready(take),.wp_x(x),.wp_y(y),.wp_last(last),
        .wp_limit(limit),.busy(busy),.success(success),.no_path(no_path),.path_len(length)
    );
    function allowed;
        input integer ax,ay;
        begin
            if (ax<0 || ax>255 || ay<0 || ay>255) allowed=0;
            else allowed=!map[ay*256+ax][1] && (!map[ay*256+ax][0] ||
                (escape && map[sy*256+sx][0] && ax>=sx-4 && ax<=sx+4 && ay>=sy-4 && ay<=sy+4));
        end
    endfunction
    task check;
        input condition;
        input [511:0] text;
        begin
            checks=checks+1;
            if (condition!==1'b1) begin errors=errors+1; $display("FAIL: planner case=%0d %0s",index,text); end
            else $display("pass planner case=%0d %0s",index,text);
        end
    endtask
    always @(posedge clk) begin
        cycles<=cycles+1;
        response<=0;
        if (pending) begin
            if (delay_left==0) begin
                response<=1;
                raw<=map[address][1];
                cost<=map[address][0];
                pending<=0;
            end else delay_left<=delay_left-1;
        end
        if (rd && rd_ready) begin
            pending<=1;
            address<=ry*256+rx;
            delay_left<=cycles%4;
            if (rx>255 || ry>255) begin $display("FAIL: planner out-of-map read"); errors=errors+1; end
        end
        if (held && (!valid || {last,y,x}!==hold_data)) stable_ok=0;
        held=valid && !take;
        if (held) hold_data={last,y,x};
        if (valid && take) begin
            points=points+1;
            $fdisplay(waypoints,"%0d,%0d,%0d,%0d,%0d",index,points,x,y,last);
            dx=$signed({1'b0,x})-px;
            dy=$signed({1'b0,y})-py;
            step_x=dx==0 ? 0 : dx>0 ? 1 : -1;
            step_y=dy==0 ? 0 : dy>0 ? 1 : -1;
            if (dx!=0 && dy!=0 && (dx<0 ? -dx : dx)!=(dy<0 ? -dy : dy)) geometry_ok=0;
            n=(dx<0 ? -dx : dx)>(dy<0 ? -dy : dy) ? (dx<0 ? -dx : dx) : (dy<0 ? -dy : dy);
            repeat(n) begin
                if (step_x!=0 && step_y!=0 && (!allowed(px+step_x,py) || !allowed(px,py+step_y))) geometry_ok=0;
                px=px+step_x;
                py=py+step_y;
                position=position+1;
                if (!allowed(px,py)) geometry_ok=0;
                if (position>4095 || reference[position]!==py*256+px) reference_ok=0;
            end
        end
    end
    initial begin
        fixture_dir="../logs/eight_direction/fixtures";
        if ($value$plusargs("FIXTURE_DIR=%s",fixture_dir)) begin end
        file_name={fixture_dir,"/cases.txt"}; fd=$fopen(file_name,"r");
        waypoints=$fopen("../logs/eight_direction/waypoints_center_0.csv","w");
        metrics=$fopen("../logs/eight_direction/rtl_center_0.csv","w");
        $fdisplay(waypoints,"case,point,x,y,last");
        $fdisplay(metrics,"case,cycles,steps,points,success,no_path");
        if (!fd) begin $display("FAIL: planner missing fixtures"); $finish; end
        result=$fscanf(fd,"%d\n",count);
        repeat(4) @(negedge clk);
        reset_n=1;
        repeat(count) begin
            result=$fscanf(fd,"%d %d %d %d %d %d %d\n",index,sx,sy,gx,gy,escape,ex);
            $sformat(file_name,"%0s/map_%0d.hex",fixture_dir,index); $readmemh(file_name,map);
            if (ex>=0) begin
                $sformat(file_name,"%0s/path_%0d.hex",fixture_dir,index); $readmemh(file_name,reference,0,ex);
            end
            position=0; points=0; px=sx; py=sy;
            geometry_ok=1; reference_ok=1; stable_ok=1;
            @(negedge clk); request=1; begin_cycle=cycles;
            @(negedge clk); request=0;
            wait_count=0;
            while (busy && wait_count<10000000) begin @(negedge clk); wait_count=wait_count+1; end
            check(!busy && (ex<0 ? no_path && !success : success && !no_path),"reachability matches independent BFS");
            check(ex<0 ? points==0 : position==ex && length==ex && px==gx && py==gy,"compressed route keeps shortest step count and goal");
            check(reference_ok && geometry_ok,"complete segments match reference and reject corner cuts");
            check(stable_ok,"waypoints remain stable under backpressure");
            $fdisplay(metrics,"%0d,%0d,%0d,%0d,%0d,%0d",index,cycles-begin_cycle,length,points,success,no_path);
            $display("METRICS: planner case=%0d cycles=%0d steps=%0d points=%0d",index,cycles-begin_cycle,length,points);
            repeat(3) @(negedge clk);
        end
        $fclose(fd);
        $fclose(metrics);
        $fclose(waypoints);
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_planner_tb cycles=%0d errors=%0d",cycles,errors);
        if (!errors) $display("PASS: nav_planner_tb");
        $finish;
    end
    initial begin #4000000000; $display("FAIL: planner timeout"); $finish; end
endmodule
