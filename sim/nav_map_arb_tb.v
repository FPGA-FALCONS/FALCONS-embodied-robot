`timescale 1ns/1ps
module nav_map_arb_tb;
    reg clk=0,rst=0;
    always #10 clk=~clk;
    reg [2:0] req=0,en=7;
    reg [47:0] x=48'h0030_0020_0010,y=0;
    reg ready=0,response=0;
    wire [2:0] grant,rsp;
    wire valid,busy,fault;
    wire [15:0] mx,my;
    integer errors=0,checks=0;
    nav_map_arb #(.TMO(5)) dut(.clk(clk),.reset_n(rst),.req(req),.enable(en),.req_x(x),.req_y(y),
        .gnt(grant),.rsp_valid(rsp),.rsp_cost(),.rsp_raw(),.map_valid(valid),.map_x(mx),.map_y(my),
        .map_ready(ready),.map_rsp_valid(response),.map_raw(1'b1),.map_cost(1'b1),.busy_o(busy),.err_timeout(fault));
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
    initial begin
        tick;rst=1;req=7;#1;
        check(!valid && grant==0,"priority selection has a registered boundary");
        tick;
        check(valid && mx==16'h10 && !busy,"selected address is held before acceptance; unaccepted request is not in flight");
        req[0]=0;#1;
        check(!valid && grant==0,"withdrawal before memory acceptance never produces a grant");
        tick;tick;ready=1;#1;
        check(grant==2 && mx==16'h20,"next surviving priority client receives its own grant");
        tick;req=0;#1;
        check(busy && grant==0,"accepted owner excludes all new requests until response");
        req=1;response=1;#1;
        check(rsp==2,"late response remains bound to accepted owner despite competing request");
        tick;response=0;ready=0;tick;
        x[15:0]=16'h11;#1;
        check(!valid,"address changes cancel an unaccepted registered selection");
        tick;tick;ready=1;tick;req=0;
        repeat(6)tick;
        check(fault && busy && grant==0,"accepted response timeout latches fault and prevents reassignment");
        response=1;tick;response=0;req=4;repeat(3)tick;
        check(fault && !valid && grant==0,"a late drained response cannot reopen a faulted arbiter");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: nav_map_arb %0d checks",checks);else $display("FAIL: nav_map_arb %0d",errors);
        $finish;
    end
endmodule
