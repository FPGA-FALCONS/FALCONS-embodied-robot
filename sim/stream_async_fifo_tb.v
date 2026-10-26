`timescale 1ns/1ps
module stream_async_fifo_tb;
    reg wclk=0,rclk=0,rst=0;
    integer wi=0,ri=0,wcycles=0,rcycles=0,checks=0,errors=0;
    wire wready,rvalid;
    wire [16:0] rdata;
    wire wvalid=wi<2000 && wcycles%5!=2;
    wire rready=rcycles%11<4;
    wire [16:0] wdata=17'h10000+wi;
    always #7 wclk=~wclk;
    always #11 rclk=~rclk;
    stream_async_fifo #(.WIDTH(17),.DEPTH(16)) dut(
        .wclk(wclk),.wrst_n(rst),.i_valid(wvalid),.i_data(wdata),.o_ready(wready),
        .rclk(rclk),.rrst_n(rst),.o_valid(rvalid),.o_data(rdata),.i_ready(rready));
    always @(posedge wclk)begin wcycles=wcycles+1;if(rst && wvalid && wready)wi<=wi+1;end
    always @(posedge rclk)begin
        rcycles=rcycles+1;
        if(rst && rvalid && rready)begin
            if(rdata!==(17'h10000+ri))begin errors=errors+1;$display("FAIL: FIFO order at %0d got %h",ri,rdata);end
            ri<=ri+1;
        end
    end
    initial begin
        #100;rst=1;
        wait(ri==2000);repeat(20)@(negedge rclk);
        checks=checks+1;if(wi==2000 && ri==2000 && !rvalid && !errors)$display("pass asynchronous stream preserves 2000 tagged words through full/backpressure");
        else begin errors=errors+1;$display("FAIL: FIFO counts");end
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: stream_async_fifo %0d checks",checks);else $display("FAIL: stream_async_fifo %0d",errors);
        $finish;
    end
    initial begin #5000000;$display("FAIL: stream_async_fifo timeout");$finish;end
endmodule
