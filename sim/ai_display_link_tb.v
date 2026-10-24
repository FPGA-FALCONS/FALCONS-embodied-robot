`timescale 1ns/1ps
module ai_display_link_tb;
    reg clk=0,pclk=0,rst=0,valid=0;reg [7:0] data=0;
    always #10 clk=~clk;always #11 pclk=~pclk;
    wire glyph,connected,host_alive,bank;wire [7:0] ack;
    integer checks=0,errors=0;
    ai_display_link #(.CLK_HZ(1000)) dut(.clk(clk),.rst_n(rst),.rx_data(data),.rx_valid(valid),
        .pixel_clk(pclk),.pixel_rst_n(rst),.pixel_x(12'd0),.pixel_y(12'd0),
        .selected(2'd0),.o_glyph(glyph),.o_connected(connected),.o_local_region(),
        .o_host_alive(host_alive),.o_display_ack(ack),.o_display_bank(bank));
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] value;integer n;reg [15:0] c;
        begin c=old^{value,8'd0};for(n=0;n<8;n=n+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
    endfunction
    task check;input condition;input [255:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task packet;input [7:0] seq,cmd;input [15:0] value;input [7:0] speed,flags;input bad;
        reg [7:0] b[0:11];reg [15:0] c;integer i;
        begin
            b[0]=8'h4d;b[1]=8'h4f;b[2]=8'h54;b[3]=8'h31;b[4]=seq;b[5]=cmd;
            b[6]=value[15:8];b[7]=value[7:0];b[8]=speed;b[9]=flags;c=16'hffff;
            for(i=0;i<10;i=i+1)c=crc_byte(c,b[i]);b[10]=c[15:8];b[11]=c[7:0]^{7'd0,bad};
            for(i=0;i<12;i=i+1)begin @(negedge clk);data=b[i];valid=1;@(negedge clk);valid=0;end
            #100;
        end
    endtask
    task font;input [7:0] seq;input [8:0] slot;integer row;
        begin for(row=0;row<16;row=row+1)packet(seq,8'h44,16'hffff,slot[7:0],{3'd0,slot[8],row[3:0]},0);end
    endtask
    initial begin
        #100;rst=1;#100;check(!connected,"reset disconnected");
        packet(1,8'h40,2,0,0,0);font(1,33);font(1,34);packet(1,8'h41,33,0,0,0);packet(1,8'h41,34,1,0,0);
        check(dut.read_count==0,"uncommitted text invisible");
        packet(1,8'h42,2,0,0,0);#300;
        check(dut.read_count==2 && dut.read_bank==1,"atomic text commit");
        check(dut.mem[1024]==33 && dut.mem[1025]==34,"glyph data in committed bank");
        check(ack==1 && bank==1,"host receives only committed display transaction and bank");
        packet(2,8'h40,1,0,0,0);packet(2,8'h41,35,0,0,1);packet(2,8'h42,1,0,0,0);
        check(dut.read_bank==1,"bad CRC cannot commit");
        packet(3,8'h40,1,0,0,0);packet(4,8'h41,35,0,0,0);packet(3,8'h42,1,0,0,0);
        check(dut.read_bank==1,"transaction mismatch rejected");
        packet(5,8'h40,2,0,0,0);packet(5,8'h41,35,1,0,0);packet(5,8'h42,2,0,0,0);
        check(dut.read_bank==1,"missing or reordered glyph rejected");
        packet(6,8'h40,1,0,0,0);font(6,35);packet(6,8'h41,35,0,0,0);packet(6,8'h42,1,0,0,0);#300;
        check(dut.read_bank==0 && dut.read_count==1,"next complete transaction commits");
        packet(7,1,800,30,0,0);check(dut.read_count==1,"motion does not edit display");
        packet(0,8'h43,0,0,0,0);check(connected,"UI ping maintains display lease");
        packet(8,8'h40,1,0,0,0);packet(8,8'h41,100,0,0,0);packet(8,8'h42,1,0,0,0);
        check(ack==6 && bank==0,"uncached glyph cannot commit uninitialized RAM");
        packet(9,8'h40,1,0,0,0);packet(9,8'h44,16'hffff,100,0,0);packet(9,8'h44,16'hffff,100,2,0);
        packet(9,8'h41,100,0,0,0);packet(9,8'h42,1,0,0,0);
        check(ack==6,"missing font row cannot expose partial glyph");
        #17000;check(!host_alive,"FPGA detects host loss before content display lease");
        #70000;check(!connected,"missing host blanks stale display");
        $display("CHECKS: %0d",checks);$display("%0d FAIL",errors);$display("PASS: ai_display_link");$finish;
    end
endmodule
