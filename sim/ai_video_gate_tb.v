`timescale 1ns/1ps
module ai_video_gate_tb;
    reg clk=0,rst=0,page=1,master=0,link=1,manual=0,open=0;
    reg [1:0] mode=0,view=1;
    wire enabled;wire [1:0] video_view;
    integer checks=0,errors=0;
    always #10 clk=~clk;
    ai_video_gate dut(.clk(clk),.rst_n(rst),.i_ai_page(page),.i_master(master),.i_link(link),
        .i_manual(manual),.i_inspection_open(open),.i_mode(mode),.i_inspection_view(view),
        .o_enabled(enabled),.o_view(video_view));
    task check;input ok;input [639:0] name;
        begin repeat(3)@(negedge clk);checks=checks+1;
        if(ok===1'b1)$display("pass %0s",name);else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task settle;begin repeat(3)@(negedge clk);end endtask
    initial begin
        #100;rst=1;settle;check(!enabled,"master off never displays RK frames");
        master=1;settle;check(!enabled,"semantic navigation retains its text page");
        mode=1;settle;check(enabled && video_view==0 && !open,"standalone recognition displays camera without opening inspection");
        manual=1;settle;check(!enabled,"manual takeover disables video overlay");
        manual=0;mode=2;settle;check(!enabled,"chatbot retains its text page");
        mode=3;settle;check(!enabled,"inspection selector stays visible until child opens");
        open=1;view=2;settle;check(enabled && video_view==2,"inspection keeps its independently selected view");
        link=0;settle;check(!enabled,"RK loss disables video overlay");
        link=1;page=0;settle;check(!enabled,"original FPGA pages cannot receive AI video overlay");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: ai_video_gate %0d checks",checks);else $display("FAIL: ai_video_gate %0d",errors);
        $finish;
    end
endmodule
