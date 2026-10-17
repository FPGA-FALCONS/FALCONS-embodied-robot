`timescale 1ns/1ps
module tb_tof_formatter;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,range_valid=0,initialized=0;
    reg [15:0] distance_mm=0;
    reg [4:0] fault_code=0;
    wire [63:0] glyphs;
    wire warning,stop;
    integer checks=0,groups=0,i,j;
    reg [63:0] before_change;
    tof_display_formatter dut(.*);
    function [7:0] digit;
      input integer n;
      begin
        case(n)
        0:digit=8'hc0;1:digit=8'hf9;2:digit=8'ha4;3:digit=8'hb0;
        4:digit=8'h99;5:digit=8'h92;6:digit=8'h82;7:digit=8'hf8;
        8:digit=8'h80;9:digit=8'h90;default:digit=8'hxx;
        endcase
      end
    endfunction
    function [63:0] expected;
      input integer mm;
      reg [31:0] left,right;
      begin
        if(mm<100) left=32'h9287a38c;
        else if(mm<200) left=32'he388afab;
        else left=32'h92888e86;
        if(mm>9999) right=32'hffff89f9;
        else right={digit((mm/1000)%10),digit((mm/100)%10),digit((mm/10)%10),digit(mm%10)};
        expected={left,right};
      end
    endfunction
    task check;
      input condition;
      input [511:0] label;
      begin
        checks=checks+1;
        if(condition!==1'b1) $fatal(1,"FORMAT FAIL %0s mm=%0d got=%h time=%0t",label,distance_mm,glyphs,$time);
      end
    endtask
    task settle;
      begin repeat(40) @(negedge clk); end
    endtask
    task check_frame;
      input [63:0] frame;
      integer byte_i;
      begin
        for(byte_i=0;byte_i<8;byte_i=byte_i+1)
          check(glyphs[byte_i*8+:8]===frame[byte_i*8+:8],"each segment byte matches independently encoded frame");
      end
    endtask
    initial begin
      repeat(4) @(negedge clk); rst_n=1;settle;
      check_frame(64'hf9abf987bfbfbfbf);check(!warning&&!stop,"startup has no obstacle decision");groups=groups+1;
      initialized=1;settle;check_frame(64'hbfbfbfbfbfbfbfbf);groups=groups+1;
      range_valid=1;
      // Independent arithmetic oracle covers every 16-bit distance, both exact
      // thresholds, four-digit overflow, all decimal carries and segment bytes.
      for(i=0;i<65536;i=i+1) begin
        distance_mm=i[15:0];settle;check_frame(expected(i));
        check(warning==((i>=100)&&(i<200)),"WARN interval is [100,200) mm");
        check(stop==(i<100),"STOP interval is [0,100) mm");
      end
      groups=groups+1;
      distance_mm=80;settle;range_valid=0;@(negedge clk);
      check(!stop&&!warning,"invalid cancels live decision immediately");settle;
      check_frame(64'hbfbfbfbfbfbfbfbf);groups=groups+1;
      fault_code=10;settle;check_frame(64'hbfbfbfbfbfbfbfbf);groups=groups+1;
      for(i=1;i<=9;i=i+1) begin
        fault_code=i[4:0];initialized=0;range_valid=1;settle;
        check_frame({32'hff86afaf,digit(0),digit(0),digit(0),digit(i)});
        check(!stop&&!warning,"fatal overrides stale valid input");
      end
      groups=groups+1;
      fault_code=0;initialized=1;range_valid=1;distance_mm=1234;settle;
      // Change faster than the BCD converter: no half-old decimal digits or
      // mismatched warning word may appear at any clock.
      before_change=expected(1234);
      for(i=0;i<1000;i=i+1) begin
        distance_mm=(i%2)==0 ? 16'd9876:16'd80;
        @(negedge clk);
        check(glyphs===before_change || glyphs===expected(9876) || glyphs===expected(80),"rapid input commits only complete captured frames");
      end
      distance_mm=199;settle;check_frame(expected(199));groups=groups+1;
      distance_mm=9999;repeat(3) @(negedge clk);rst_n=0;
      @(negedge clk);check_frame(64'hbfbfbfbfbfbfbfbf);
      initialized=0;range_valid=0;rst_n=1;settle;check_frame(64'hf9abf987bfbfbfbf);
      initialized=1;range_valid=1;distance_mm=200;settle;check_frame(expected(200));groups=groups+1;
      $display("PASS tb_tof_formatter groups=%0d checks=%0d exhaustive_distances=65536",groups,checks);
      $finish;
    end
    initial begin #100000000;$fatal(1,"formatter watchdog");end
endmodule
