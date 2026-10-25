`timescale 1ns/1ps
module nav_line_cells_tb;
    reg clk=0, reset_n=0, start=0;
    always #10 clk=~clk;
    reg [15:0] x0=10,y0=10,x1=10,y1=10;
    wire valid,last,invalid;
    wire [15:0] x,y;
    integer cycles=0,checks=0,errors=0,d,n,dx,dy,k,count;
    reg [1023:0] expected,observed;
    reg held=0,stable;
    reg [32:0] held_data;
    wire ready=cycles%5>=2;
    always @(posedge clk) begin
        cycles<=cycles+1;
        if (held && (!valid || {last,y,x}!==held_data)) stable=0;
        held=valid && !ready;
        if (held) held_data={last,y,x};
        if (valid && ready) begin
            if (x>31 || y>31) begin $display("FAIL: line coordinate overflow"); errors=errors+1; end
            else observed[y*32+x]=1;
            count=count+1;
        end
    end
    nav_line_cells dut (
        .clk(clk),.reset_n(reset_n),.start(start),.cancel(1'b0),
        .x0(x0),.y0(y0),.x1(x1),.y1(y1),.valid(valid),.ready(ready),
        .x(x),.y(y),.last(last),.invalid(invalid)
    );
    task check;
        input condition;
        input [511:0] text;
        begin
            checks=checks+1;
            if (condition!==1'b1) begin errors=errors+1; $display("FAIL: line %0s",text); end
            else $display("pass line %0s",text);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); reset_n=1;
        for(d=0;d<8;d=d+1) begin
            case(d) 0,1,7:dx=1; 3,4,5:dx=-1; default:dx=0; endcase
            case(d) 1,2,3:dy=1; 5,6,7:dy=-1; default:dy=0; endcase
            expected=0; observed=0; count=0; stable=1;
            for(k=0;k<=7;k=k+1) begin
                expected[(10+dy*k)*32+10+dx*k]=1;
                if(k<7 && dx!=0 && dy!=0) begin
                    expected[(10+dy*k)*32+10+dx*(k+1)]=1;
                    expected[(10+dy*(k+1))*32+10+dx*k]=1;
                end
            end
            @(negedge clk); x1=10+dx*7; y1=10+dy*7; start=1;
            @(negedge clk); start=0;
            wait(valid);
            wait(!valid);
            @(negedge clk);
            check(expected===observed && count==(dx!=0 && dy!=0 ? 22 : 8),"all eight directions include both endpoints and diagonal flank cells");
            check(stable,"cell coordinates hold under backpressure");
        end
        @(negedge clk); x1=14; y1=13; start=1;
        @(negedge clk); start=0;
        repeat(3) @(negedge clk);
        check(invalid && !valid,"non-eight-direction segment is rejected");
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_line_cells_tb cycles=%0d errors=%0d",cycles,errors);
        if(!errors) $display("PASS: nav_line_cells_tb");
        $finish;
    end
    initial begin #100000; $display("FAIL: line timeout"); $finish; end
endmodule
