`timescale 1ns/1ps
module nav_drive_tb;
    reg clk=0, reset_n=0, pose_valid=1;
    always #10 clk=~clk;
    reg [2:0] heading=0;
    reg [7:0] check_cells=4;
    reg [15:0] width_mm=180;
    reg signed [31:0] px=25,py=25;
    reg hit=0,response=0,raw=0,pending=0;
    integer hit_x=0,hit_y=0,address_x,address_y,delay_left;
    integer cycles=0,checks=0,errors=0,d,dx,dy,nx,ny,begin_cycle,max_scan=0,wait_count;
    wire request,clear,blocked,busy;
    wire [15:0] x,y;
    wire ready=!pending && cycles%5!=0;
    nav_drive dut (
        .forward_check_cells(check_cells), .vehicle_width_mm(width_mm),
        .clk(clk),.reset_n(reset_n),.pose_valid(pose_valid),.check_enable(1'b1),
        .pose_x_mm(px),.pose_y_mm(py),.heading(heading),
        .mrd_valid(request),.mrd_ready(ready),.mrd_x(x),.mrd_y(y),
        .mrd_rsp_valid(response),.mrd_raw(raw),.forward_clear(clear),.segment_blocked(blocked),.busy(busy)
    );
    always @(posedge clk) begin
        cycles<=cycles+1;
        response<=0;
        if(pending) begin
            if(delay_left==0) begin
                response<=1;
                raw<=hit && address_x==hit_x && address_y==hit_y;
                pending<=0;
            end else delay_left<=delay_left-1;
        end
        if(request && ready) begin
            address_x<=x;
            address_y<=y;
            pending<=1;
            delay_left<=cycles%4;
        end
    end
    task check;
        input condition;
        input [511:0] text;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin errors=errors+1; $display("FAIL: drive dir=%0d %0s",heading,text); end
            else $display("pass drive dir=%0d %0s",heading,text);
        end
    endtask
    task scan;
        begin
            if(busy) wait(!busy);
            wait(busy);
            begin_cycle=cycles;
            wait(!busy);
            @(negedge clk);
            if(cycles-begin_cycle>max_scan) max_scan=cycles-begin_cycle;
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); reset_n=1;
        for(d=0;d<8;d=d+1) begin
            @(negedge clk); heading=d; hit=0;
            case(d) 0,1,7:dx=1; 3,4,5:dx=-1; default:dx=0; endcase
            case(d) 1,2,3:dy=1; 5,6,7:dy=-1; default:dy=0; endcase
            nx=-dy; ny=dx;
            scan();
            check(clear && !blocked,"free front region permits motion");
            @(negedge clk); hit=1; hit_x=128+2*dx; hit_y=128+2*dy;
            scan();
            check(blocked && !clear,"raw obstacle ahead stops motion");
            @(negedge clk); hit_x=128+2*dx+8*nx; hit_y=128+2*dy+8*ny;
            scan();
            check(clear && !blocked,"lateral obstacle outside front region is ignored");
            @(negedge clk); hit_x=128-5*dx; hit_y=128-5*dy;
            scan();
            check(clear && !blocked,"obstacle behind car is ignored");
        end
        // 车中心附近的真实障碍也检查，不再跳过原来的前缘偏移。
        @(negedge clk); heading=0; hit=1; hit_x=128; hit_y=128;
        scan();
        check(blocked && !clear,"raw obstacle at center is checked");
        @(negedge clk); hit_x=131; hit_y=128; check_cells=2;
        scan();
        check(clear && !blocked,"two-cell lookahead ignores obstacle three cells ahead");
        @(negedge clk); check_cells=4;
        scan();
        check(blocked && !clear,"four-cell lookahead includes same obstacle");
        @(negedge clk); hit_x=133;
        scan();
        check(clear && !blocked,"obstacle beyond four cells is ignored");
        @(negedge clk); hit_x=132;
        scan();
        check(blocked && !clear,"cell intersecting front endpoint is checked");
        @(negedge clk); hit_x=130; hit_y=131; width_mm=180;
        scan();
        check(clear && !blocked,"width has no additional lateral margin");
        @(negedge clk); width_mm=300;
        scan();
        check(blocked && !clear,"runtime width increase includes lateral obstacle");
        @(negedge clk); width_mm=100;
        scan();
        check(clear && !blocked,"runtime width decrease excludes lateral obstacle");
        // 配置只影响下一次扫描，中途不能混用新旧尺寸。
        @(negedge clk); check_cells=2; width_mm=180; hit_x=131; hit_y=128;
        wait(busy);
        @(negedge clk); check_cells=4; width_mm=300;
        wait(!busy);
        @(negedge clk);
        check(clear && !blocked,"in-flight scan retains captured configuration");
        scan();
        check(blocked && !clear,"next scan uses updated configuration");
        // UI 上限在八方向全部能于 1 ms 内完成，覆盖最长、最宽矩形。
        @(negedge clk); check_cells=16; width_mm=500; hit=0;
        for(d=0;d<8;d=d+1) begin
            @(negedge clk); heading=d;
            scan();
            check(clear && !blocked,"largest UI dimensions complete without timeout");
            check(cycles-begin_cycle<50000,"scan latency stays below one millisecond");
        end
        @(negedge clk); heading=0; px=6300; check_cells=4; width_mm=180;
        scan();
        check(blocked && !clear,"front rectangle cannot cross map boundary");
        @(negedge clk); pose_valid=0;
        repeat(3) @(negedge clk);
        check(!clear,"invalid pose never grants front clearance");
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_drive_tb cycles=%0d max_scan=%0d errors=%0d",cycles,max_scan,errors);
        if(!errors) $display("PASS: nav_drive_tb");
        $finish;
    end
    initial begin #100000000; $display("FAIL: drive timeout"); $finish; end
endmodule
