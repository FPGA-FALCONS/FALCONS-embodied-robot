`timescale 1ns/1ps
module inspection_route_export_tb;
    reg clk=0,rst=0,valid=0,start=0,done=0,release_front=0,read=0;
    reg [7:0] x=0,y=0;reg [6:0] address=0;
    reg [31:0] context=7;
    wire published,error;wire [31:0] crc,sequence;wire [6:0] words;wire [15:0] data;
    integer checks=0,errors=0,device_errors=0,k;
    always #10 clk=~clk;
    always @(posedge clk)if(error)device_errors=device_errors+1;
    inspection_route_export dut(.clk(clk),.rst_n(rst),.i_enabled(1'b1),.i_valid(valid),.i_start(start),.i_done(done),.i_x(x),.i_y(y),
        .i_route_id(16'd5),.i_map_version(16'd6),.i_session(32'd1),.i_task(32'd2),.i_epoch(32'd3),.i_context(context),.i_view(2'd1),
        .o_valid(published),.i_release(release_front),.o_session(),.o_task(),.o_epoch(),.o_context(),.o_sequence(sequence),.o_crc(crc),
        .o_view(),.o_words(words),.i_read(read),.i_address(address),.o_data(data),.o_error(error));
    task check;input ok;input [639:0] name;
        begin checks=checks+1;if(ok===1'b1)$display("pass %0s",name);else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task point;input [7:0] px,py;input first,last;
        begin @(negedge clk);valid=1;start=first;done=last;x=px;y=py;@(negedge clk);valid=0;start=0;done=0;end
    endtask
    task get;input [6:0] index;input [15:0] expected;
        begin @(negedge clk);address=index;read=1;@(negedge clk);read=0;check(data==expected,"synchronous held route RAM word order");end
    endtask
    task release_bank;begin @(negedge clk);release_front=1;@(negedge clk);release_front=0;end endtask
    initial begin
        #100;rst=1;point(128,128,1,0);point(138,128,0,0);point(138,138,0,1);
        wait(published);@(negedge clk);
        check(words==4 && crc==32'heb2b2be0 && sequence==32'h00050006,"complete published prefix has independent host CRC and source route/map versions");
        get(0,3);get(1,16'h8080);get(2,16'h808a);get(3,16'h8a8a);
        point(1,2,1,1);repeat(200)@(negedge clk);
        check(words==4 && crc==32'heb2b2be0,"held front cannot be overwritten by a new prefix");
        release_bank();repeat(10)@(negedge clk);check(published && words==2,"back prefix publishes after release");
        release_bank();point(1,2,1,0);context=8;point(3,4,0,1);repeat(200)@(negedge clk);
        check(!published,"context change aborts an incomplete prefix");
        @(negedge clk);start=1;done=1;@(negedge clk);start=0;done=0;
        wait(published);@(negedge clk);check(words==1 && crc==32'h41d912ff,"empty route is an explicit CRC-protected clear");
        release_bank();
        for(k=0;k<65;k=k+1)point(k,k,k==0,k==64);
        repeat(200)@(negedge clk);check(!published && device_errors>0,"overflow never publishes a truncated prefix");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_route_export %0d checks",checks);else $display("FAIL: inspection_route_export %0d",errors);
        $finish;
    end
    initial begin #500000;$display("FAIL: inspection_route_export timeout");$finish;end
endmodule
