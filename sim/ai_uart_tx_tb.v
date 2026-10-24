`timescale 1ns/1ps
module ai_uart_tx_tb;
    reg clk=0,rst=0,valid=0,link_active=1;reg [7:0] data;
    always #500 clk=~clk;
    wire ready,tx,rx_valid;wire [7:0] rx_data;
    reg [7:0] received[0:63];integer count=0,checks=0,errors=0,i;
    reg [15:0] c;
    ai_uart_tx #(.CLK_HZ(1_000_000),.BAUD_RATE(100_000),.REPORT_CYCLES(5000)) dut(
        .clk(clk),.rst_n(rst),.i_csv_data(data),.i_csv_valid(valid),.o_csv_ready(ready),
        .i_flags(16'd19),.i_controls(8'd18),.i_ai_link_active(link_active),.o_uart_tx(tx));
    uart_byte_rx #(.CLK_FREQ_HZ(1_000_000),.BAUD_RATE(100_000),.RECOVERY_EN(0)) rx(
        .i_clock(clk),.i_reset_n(rst),.i_uart_rx(tx),.o_rx_data(rx_data),
        .o_rx_valid(rx_valid),.i_rx_ready(1'b1),.o_rx_error());
    always @(posedge clk)if(rx_valid && count<64)begin received[count]=rx_data;count=count+1;end
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] value;integer n;reg [15:0] x;
        begin x=old^{value,8'd0};for(n=0;n<8;n=n+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc_byte=x;end
    endfunction
    task check;input condition;input [255:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task send;input [7:0] value;
        begin @(negedge clk);data=value;valid=1;@(posedge clk);while(!ready)@(posedge clk);@(negedge clk);valid=0;end
    endtask
    initial begin
        #4000;rst=1;#4000;
        send("1");send(",");send("2");send(13);send(10);
        wait(count>=17);#1000;
        check({received[0],received[1],received[2],received[3]}==32'h4d4f5431,"control envelope magic");
        check(received[5]==8'h50 && received[7]==19 && received[8]==18,"physical controls snapshot");
        c=16'hffff;for(i=0;i<10;i=i+1)c=crc_byte(c,received[i]);
        check({received[10],received[11]}==c,"control CRC");
        check({received[12],received[13],received[14],received[15],received[16]}==40'h312c320d0a,"CSV line remains whole");
        link_active=0;rst=0;#4000;count=0;rst=1;#4000;
        send("1");send(",");send("2");send(13);send(10);#6000000;
        check(count==5 && {received[0],received[1],received[2],received[3],received[4]}==40'h312c320d0a,"RK absent preserves original plain CSV");
        $display("CHECKS: %0d",checks);$display("%0d FAIL",errors);$display("PASS: ai_uart_tx");$finish;
    end
    initial begin #20000000;$display("FAIL: UART test timeout count=%0d",count);$finish;end
endmodule
