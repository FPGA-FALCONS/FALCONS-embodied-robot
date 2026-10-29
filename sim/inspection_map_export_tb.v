`timescale 1ns/1ps
module inspection_map_export_tb;
    reg clk=0,rst=0,enabled=1,valid=0,occupied=0,start=0,done=0,release_map=0,read=0;
    reg [15:0] version=1;
    reg [31:0] context=7;
    reg [11:0] address=0;
    wire published,error;
    wire [31:0] sequence,crc;
    wire [15:0] data;
    integer errors=0,checks=0,errors_reported=0,n,k;
    always #10 clk=~clk;
    always @(posedge clk)if(error)errors_reported=errors_reported+1;
    inspection_map_export dut(.clk(clk),.rst_n(rst),.i_enabled(enabled),.i_valid(valid),.i_occupied(occupied),
        .i_start(start),.i_done(done),.i_version(version),.i_session(32'd1),.i_task(32'd2),.i_epoch(32'd3),.i_context(context),.i_view(2'd1),
        .o_valid(published),.i_release(release_map),.o_session(),.o_task(),.o_epoch(),.o_context(),.o_sequence(sequence),.o_crc(crc),.o_view(),
        .i_read(read),.i_address(address),.o_data(data),.o_error(error));
    function pattern;input integer index;
        begin pattern=(index%17==3 || index%251==8);end
    endfunction
    function [15:0] expected;input integer index;input invert;integer b;
        begin for(b=0;b<16;b=b+1)expected[b]=pattern(index*16+b)^invert;end
    endfunction
    task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
    task sweep;input invert;input change_version;input change_context;integer index;
        begin
            for(index=0;index<65536;index=index+1)begin
                valid=1;occupied=pattern(index)^invert;start=index==0;done=index==65535;
                if(change_version && index==1000)version=2;
                if(change_context && index==1000)context=context+1;
                tick;valid=0;start=0;done=0;tick;
            end
            repeat(4)tick;
        end
    endtask
    task check;input condition;input [767:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task check_ram;input invert;integer index;
        begin
            read=1;
            for(index=0;index<4096;index=index+1)begin
                address=index;@(posedge clk);#1;
                if(data!==expected(index,invert))begin errors=errors+1;$display("FAIL: map packed word %0d",index);end
                @(negedge clk);
            end
            read=0;
        end
    endtask
    initial begin
        repeat(3)tick;rst=1;tick;
        sweep(0,1,0);
        check(published && sequence==32'h00010002 && crc==32'h983d8789,"complete raster CRC matches independent Python zlib and reports version range");
        check_ram(0);check(!errors,"every cell keeps x-first LSB order through synchronous RAM");
        version=3;sweep(1,0,0);
        check(published && sequence==32'h00010002 && crc==32'h983d8789,"back-bank capture cannot change held front metadata");
        check_ram(0);
        release_map=1;tick;release_map=0;tick;
        check(published && sequence==32'h00030003 && crc==32'hf4e02a28,"pending complete map replaces front only after explicit release");
        check_ram(1);
        sweep(0,0,1);release_map=1;tick;release_map=0;repeat(4)tick;
        check(!published && !errors,"context change aborts partial raster without publishing old task data");
        valid=1;start=1;occupied=0;tick;start=0;done=1;tick;valid=0;done=0;repeat(4)tick;
        check(!published && errors_reported==1,"early frame-end never creates a padded or incomplete map");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_map_export %0d checks",checks);else $display("FAIL: inspection_map_export %0d",errors);
        $finish;
    end
endmodule
