`timescale 1ns/1ps
// 中继在居中后插入；容量达到时发布可执行前缀，排空后仍能接受下一请求。
module nav_planner_limit_tb;
    reg clk=0,reset_n=0,request=0,response=0;
    always #10 clk=~clk;
    reg [15:0] gx=14,gy=14;
    wire ready,rd,valid,last,busy,success,no_path;
    wire [15:0] rx,ry,x,y,length;
    integer cycles=0,checks=0,errors=0,n,count,px,py,wait_count;
    reg shape_ok;
    wire take=cycles%5>=2;
    always @(posedge clk) begin
        cycles<=cycles+1;
        response<=rd;
        if(valid && take) begin
            count=count+1;
            if(n==0 && (x!=1+count*3 || y!=1+count*3)) shape_ok=0;
            if(n==1 && (x!=1 || y!=1+count*3)) shape_ok=0;
            if(n==2 && (x!=1 || y!=1)) shape_ok=0;
            if(last!=(n==2 || count==2)) shape_ok=0;
        end
    end
    nav_planner #(.GRID_W(16),.GRID_H(16),.W_MAX(4)) dut (
        .clk(clk),.reset_n(reset_n),.req_valid(request),.req_ready(ready),
        .allow_start_escape(1'b0),.start_x(16'd1),.start_y(16'd1),.goal_x(gx),.goal_y(gy),
        .mrd_valid(rd),.mrd_ready(1'b1),.mrd_x(rx),.mrd_y(ry),
        .mrd_rsp_valid(response),.mrd_raw(1'b0),.mrd_cost(1'b0),
        .wp_limit(16'd2),.wp_valid(valid),.wp_ready(take),.wp_x(x),.wp_y(y),.wp_last(last),
        .busy(busy),.success(success),.no_path(no_path),.path_len(length)
    );
    task check;
        input condition;
        input [511:0] text;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin errors=errors+1; $display("FAIL: prefix %0s",text); end
            else $display("pass prefix %0s",text);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); reset_n=1;
        for(n=0;n<3;n=n+1) begin
            count=0; shape_ok=1;
            @(negedge clk); gx=n==0 ? 14 : 1; gy=n==2 ? 1 : 14; request=1;
            @(negedge clk); request=0;
            wait_count=0;
            while(busy && wait_count<100000) begin @(negedge clk); wait_count=wait_count+1; end
            check(!busy && success && !no_path && ready,"capacity prefix drains pipeline and accepts next request");
            check(shape_ok && count==(n==2 ? 1 : 2),"relay spacing and truncated last flag are correct");
        end
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_planner_limit_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors) $display("PASS: nav_planner_limit_tb");
        $finish;
    end
    initial begin #10000000; $display("FAIL: prefix timeout"); $finish; end
endmodule
