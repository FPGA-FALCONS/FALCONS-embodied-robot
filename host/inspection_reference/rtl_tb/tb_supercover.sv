`timescale 1ns/1ps
module tb_supercover;
    reg clk=0; always #5 clk=~clk;
    reg reset_n=0, cancel=0, start_valid=0, result_ready=0;
    reg [15:0] start_x, start_y, end_x, end_y;
    reg [31:0] map_revision=1;
    wire start_ready, map_req_valid, result_valid, busy;
    reg map_req_ready=0, map_resp_valid=0, map_blocked=0;
    wire [7:0] map_x, map_y;
    wire [2:0] result_code;
    wire [31:0] result_revision;
    wire [10:0] checked_cells;
    nav_supercover_check dut(.*);
    reg expected [0:65535];
    reg seen [0:65535];
    integer addresses [0:1023];
    integer cycle=0, pending=0, delay_count=0, requests=0;
    integer address_q=0, blocked_address=-1, expected_code=0, aborted=0, abort_state=2;
    integer address_now;
    // Memory/scoreboard runs independently of stimulus. At least one cycle
    // response latency and deterministic variable backpressure and delays.
    always @(posedge clk) if (reset_n) begin
        cycle=cycle+1;
        if (map_resp_valid) begin
            if (!pending) $fatal(1,"response without request");
            pending=0;
        end
        if (map_req_valid && map_req_ready) begin
            if (pending) $fatal(1,"multiple outstanding reads");
            address_now={map_y,map_x};
            if (!expected[address_now]) $fatal(1,"unexpected cell %d",address_now);
            if (seen[address_now]) $fatal(1,"duplicate cell %d",address_now);
            seen[address_now]=1;
            address_q=address_now; pending=1;
            delay_count=cycle%4+2; requests=requests+1;
        end
    end
    always @(negedge clk) begin
        map_req_ready=reset_n && cycle%5!=0;
        map_resp_valid=0;
        if (pending) begin
            if (delay_count==0) begin
                map_resp_valid=1;
                map_blocked=address_q==blocked_address;
            end else delay_count=delay_count-1;
        end
        if (!aborted && dut.state==abort_state && (expected_code==2 || expected_code==4)) begin
            aborted=1;
            if (expected_code==2) map_revision=map_revision+1;
            else cancel=1;
        end
    end

    integer fd, count, code, bx, by, n, i, j, got, limit, cases=0, total_reads=0;
    integer x0,y0,x1,y1;
    reg [2:0] held_code;
    reg [31:0] held_revision;
    reg [10:0] held_count;
    initial begin
        for(i=0;i<65536;i=i+1) begin expected[i]=0; seen[i]=0; end
        repeat(3) @(negedge clk);
        reset_n=1;
        fd=$fopen("vectors.txt","r");
        if (!fd) $fatal(1,"cannot open vectors");
        while (!$feof(fd)) begin
            got=$fscanf(fd,"%d %d %d %d %d %d %d %d %d\n",x0,y0,x1,y1,count,code,bx,by,abort_state);
            if(got!=9) $fatal(1,"bad vector header %d",got);
            for(j=0;j<count;j=j+1) begin
                got=$fscanf(fd,"%d\n",n);
                if(got!=1) $fatal(1,"bad cell vector");
                addresses[j]=n; expected[n]=1; seen[n]=0;
            end
            @(negedge clk);
            expected_code=code; blocked_address=bx<0 ? -1 : by*256+bx;
            requests=0; aborted=0; cancel=0;
            start_x=x0; start_y=y0; end_x=x1; end_y=y1;
            start_valid=1;
            if (!start_ready) $fatal(1,"start not ready");
            @(negedge clk); start_valid=0;
            limit=0;
            while(!result_valid) begin
                @(negedge clk); limit=limit+1;
                if(limit>20000) $fatal(1,"timeout case %d",cases);
            end
            if (pending) $fatal(1,"result before read drained");
            if (result_code!==code) $fatal(1,"code got %d expected %d case %d",result_code,code,cases);
            if (checked_cells!==requests) $fatal(1,"request count mismatch");
            if (code==0) begin
                if(requests!=count) $fatal(1,"missing cells %d vs %d",requests,count);
                for(j=0;j<count;j=j+1)
                    if(!seen[addresses[j]]) $fatal(1,"missing expected cell");
            end
            if (code==3 && requests!=0) $fatal(1,"invalid endpoint read memory");
            // Hold completion; revision can change afterwards. Result must
            // retain the old revision so a consumer can reject stale clearance.
            held_code=result_code; held_revision=result_revision; held_count=checked_cells;
            map_revision=map_revision+1;
            repeat(7) begin
                @(negedge clk);
                if (!result_valid || result_code!==held_code || result_revision!==held_revision ||
                    checked_cells!==held_count || map_req_valid || start_ready)
                    $fatal(1,"unstable completion under backpressure");
            end
            result_ready=1;
            @(negedge clk); result_ready=0; cancel=0;
            for(j=0;j<count;j=j+1) expected[addresses[j]]=0;
            total_reads=total_reads+requests; cases=cases+1;
        end
        $fclose(fd);
        $display("PASS cases=%0d accepted_reads=%0d",cases,total_reads);
        $finish;
    end
endmodule
