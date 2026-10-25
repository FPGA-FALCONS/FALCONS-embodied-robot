`timescale 1ns/1ps
module nav_dijkstra_tb;
    reg clk = 0, reset_n = 0, request = 0, escape = 0;
    always #10 clk = ~clk;
    reg [15:0] sx,sy,gx,gy;
    wire ready,busy,success,no_path,rd,valid,last;
    wire [15:0] rx,ry,x,y,length;
    wire [2:0] dir;
    reg response = 0,raw = 0,cost = 0,pending = 0;
    reg [1:0] map [0:65535];
    reg [15:0] reference [0:4095];
    integer cycles = 0,checks = 0,errors = 0,delay_left,address;
    integer fd,result,count,index,ex,position,begin_cycle,peak,wait_count;
    integer px,py,dx,dy,expected_cost,actual_cost,k,improved=0,stale=0,ties=0,wraps=0,expected_turns,old_direction,direction_now,saturation_cases=0;
    reg geometry_ok,reference_ok,stable_ok,held = 0;
    reg [35:0] hold_data;
    reg [2047:0] fixture_dir,file_name;
    wire rd_ready = !pending && cycles%5 != 0;
    wire take = cycles%7 >= 2;
    nav_dijkstra dut (
        .clk(clk),.reset_n(reset_n),.req_valid(request),.req_ready(ready),
        .allow_start_escape(escape),.start_x(sx),.start_y(sy),.goal_x(gx),.goal_y(gy),
        .mrd_valid(rd),.mrd_ready(rd_ready),.mrd_x(rx),.mrd_y(ry),
        .mrd_rsp_valid(response),.mrd_raw(raw),.mrd_cost(cost),
        .path_valid(valid),.path_ready(take),.path_x(x),.path_y(y),
        .path_dir(dir),.path_last(last),.busy(busy),.success(success),.no_path(no_path),.path_len(length),.start_relaxed()
    );
    function allowed;
        input integer ax,ay;
        begin
            if (ax<0 || ax>255 || ay<0 || ay>255) allowed = 0;
            else allowed = !map[ay*256+ax][1] && (!map[ay*256+ax][0] || (escape && map[sy*256+sx][0] && ax >= sx-4 && ax <= sx+4 && ay >= sy-4 && ay <= sy+4));
        end
    endfunction
    task check;
        input condition;
        input [511:0] text;
        begin
            checks = checks+1;
            if (condition !== 1'b1) begin errors = errors+1; $display("FAIL: dijkstra case=%0d %0s",index,text); end
            else $display("pass dijkstra case=%0d %0s",index,text);
        end
    endtask
    always @(posedge clk) begin
        cycles <= cycles+1;
        response <= 0;
        if (pending) begin
            if (delay_left == 0) begin
                response <= 1;
                raw <= map[address][1];
                cost <= map[address][0];
                pending <= 0;
            end else delay_left <= delay_left-1;
        end
        if (rd && rd_ready) begin
            pending <= 1;
            address <= ry*256+rx;
            delay_left <= cycles%4;
            if (rx>255 || ry>255) begin $display("FAIL: dijkstra out-of-map read"); errors = errors+1; end
        end
        if (dut.state>=7 && dut.state<=19 && 2048-dut.free_count>peak) peak = 2048-dut.free_count;
        if (dut.state == 7 && dut.bucket_empty != 4'hf && dut.bucket_empty[dut.current_bucket] && dut.current_bucket == 3) wraps = wraps+1;
        if (dut.state == 24) ties = ties+1;
        if (dut.state == 23 && dut.shorter && dut.node_q[2]) improved = improved + 1;
        if (dut.state == 11 && (!dut.node_q[2] || dut.node_q[1:0] != dut.current_bucket)) stale = stale + 1;
        if (held && (!valid || {last,dir,y,x} !== hold_data)) stable_ok = 0;
        held = valid && !take;
        if (held) hold_data = {last,dir,y,x};
        if (valid && take) begin
            if (position>4095 || reference[position] !== y*256+x) reference_ok = 0;
            if (!allowed(x,y)) geometry_ok = 0;
            if (position>0) begin
                actual_cost = actual_cost + (x != px && y != py ? 3 : 2);
                dx = $signed({1'b0,x})-px;
                dy = $signed({1'b0,y})-py;
                if (dx>1 || dx< -1 || dy>1 || dy< -1 || (dx == 0 && dy == 0)) geometry_ok = 0;
                if (dx != 0 && dy != 0 && (!allowed(px+dx,py) || !allowed(px,py+dy))) geometry_ok = 0;
            end
            px = x;
            py = y;
            position = position+1;
        end
    end
    initial begin
        fixture_dir = "../logs/dijkstra/fixtures";
        if ($value$plusargs("FIXTURE_DIR = %s",fixture_dir)) begin end
        file_name = {fixture_dir,"/cases.txt"}; fd = $fopen(file_name,"r");
        if (!fd) begin $display("FAIL: dijkstra missing fixtures"); $finish; end
        result = $fscanf(fd,"%d\n",count);
        repeat(4) @(negedge clk);
        reset_n = 1;
        repeat(count) begin
            result = $fscanf(fd,"%d %d %d %d %d %d %d\n",index,sx,sy,gx,gy,escape,ex);
            $sformat(file_name,"%0s/map_%0d.hex",fixture_dir,index); $readmemh(file_name,map);
            if (ex >= 0) begin
                $sformat(file_name,"%0s/path_%0d.hex",fixture_dir,index); $readmemh(file_name,reference,0,ex);
            end
            expected_cost = 0;
            for (k=1;k<=ex;k=k+1) expected_cost = expected_cost +
                (reference[k]%256 != reference[k-1]%256 && reference[k]/256 != reference[k-1]/256 ? 3 : 2);
            expected_turns = 0;
            old_direction = -1;
            for (k=1;k<=ex;k=k+1) begin
                direction_now = reference[k]-reference[k-1];
                if (k>1 && direction_now != old_direction) expected_turns=expected_turns+1;
                old_direction=direction_now;
            end
            actual_cost = 0;
            position = 0;
            peak = 0;
            geometry_ok = 1;
            reference_ok = 1;
            stable_ok = 1;
            @(negedge clk);
            request = 1;
            begin_cycle = cycles;
            @(negedge clk);
            request = 0;
            wait_count = 0;
            while (busy && wait_count<10000000) begin @(negedge clk);
            wait_count = wait_count+1;
            end
            check(!busy && (ex<0 ? no_path && !success : success && !no_path),"reachability matches independent Dijkstra");
            check(ex<0 ? position == 0 : position == ex+1 && length == ex && actual_cost == expected_cost,"minimum 2/3 cost and reference steps");
            check(reference_ok && geometry_ok,"all cells match straight-first reference and reject corner cutting");
            check(stable_ok && peak<=2048,"backpressure and queue bound");
            if (ex>=0) begin
                check(dut.node_tag[sy*256+sx] == 3'b001 &&
                      dut.turns[sy*256+sx] == (expected_turns>255 ? 255 : expected_turns),
                      "settled state and saturating turn count");
                if (expected_turns>255) saturation_cases=saturation_cases+1;
            end
            $display("METRICS: dijkstra case=%0d cycles=%0d steps=%0d peak=%0d",index,cycles-begin_cycle,length,peak);
            repeat(3) @(negedge clk);
        end
        $fclose(fd);
        check(improved > 0 && stale > 0 && ties > 0 && wraps > 0 && saturation_cases > 0,"maps exercise distance improvements, turn ties and stale entries");
        $display("METRICS: improvements=%0d stale=%0d ties=%0d wraps=%0d saturation=%0d",improved,stale,ties,wraps,saturation_cases);
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_dijkstra_tb cycles=%0d errors=%0d",cycles,errors);
        if (!errors) $display("PASS: nav_dijkstra_tb");
        $finish;
    end
    initial begin #4000000000; $display("FAIL: nav_dijkstra timeout"); $finish; end
endmodule
