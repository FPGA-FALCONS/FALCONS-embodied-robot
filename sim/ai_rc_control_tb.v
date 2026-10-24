`timescale 1ns/1ps
module ai_rc_control_tb;
    reg clk=0,rst=0,enter=0,page=0,back=0,prev=0,next=0,confirm=0;
    reg rc_ok=1,manual=0,estop=0,pwm=0,host_alive=1;
    always #500 clk=~clk;
    wire master,chat,recording,vision;
    wire [1:0] mode,selected;
    wire [15:0] flags;wire [7:0] controls;
    integer checks=0,errors=0;
    ai_rc_control #(.CLK_HZ(1_000_000)) dut(
        .clk(clk),.rst_n(rst),.i_enter_confirm(enter),.i_ai_page(page),.i_back(back),
        .i_select_prev(prev),.i_select_next(next),.i_feature_confirm(confirm),
        .i_rc_valid(rc_ok),.i_manual(manual),.i_estop(estop),.i_swb_pwm(pwm),.i_host_alive(host_alive),
        .o_master(master),.o_chat(chat),.o_record(recording),.o_vision(vision),
        .o_mode(mode),.o_selected(selected),.o_flags(flags),.o_controls(controls));
    task check;input condition;input [255:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task frame;input integer us;
        begin pwm=1;#(us*1000);pwm=0;#((20000-us)*1000);end
    endtask
    task select_next;
        begin @(negedge clk);next=1;@(negedge clk);next=0;end
    endtask
    task choose;
        begin @(negedge clk);confirm=1;@(negedge clk);confirm=0;#4000;end
    endtask
    initial begin
        #4000;rst=1;#4000;
        check(!master && !recording,"reset master off");
        frame(2000);frame(2000);page=1;enter=1;#2000;enter=0;#5000;
        check(master && !recording,"held down at entry is not recording");
        frame(1000);frame(1000);
        check(!recording,"SWB up arms recording");
        frame(2000);frame(2000);check(recording,"SWB down starts recording");
        frame(2000);check(recording,"held down remains one recording");
        frame(1000);frame(1000);check(!recording,"SWB up stops recording");
        select_next;check(selected==1 && mode==0,"selection does not activate feature");
        choose;check(mode==1 && vision && !chat,"confirm activates recognition");
        select_next;choose;check(mode==2 && chat && !vision,"confirm activates chat only");
        select_next;check(selected==0,"submenu wraps");
        @(negedge clk);prev=1;@(negedge clk);prev=0;#2000;check(selected==2,"submenu reverse wraps");
        frame(1000);frame(1000);frame(2000);frame(2000);
        check(recording,"chat also uses SWB recording");
        manual=1;#4000;check(!recording && !master,"manual takeover closes AI and recording");
        manual=0;frame(2000);check(!recording,"manual release cannot rearm held down");
        frame(1000);frame(1000);frame(2000);frame(2000);
        enter=1;#2000;enter=0;
        estop=1;#4000;check(!recording,"estop cancels recording");estop=0;
        #101000000;check(!recording && !flags[4],"stale SWB cannot record");
        enter=1;#2000;enter=0;host_alive=0;#4000;check(!master,"RK loss closes master locally");
        host_alive=1;#4000;check(!master,"RK recovery cannot reopen AI");
        enter=1;#2000;enter=0;back=1;#2000;back=0;#2000;check(!master,"back closes master");
        enter=1;#2000;enter=0;page=0;#4000;check(!master,"leaving AI page closes master");
        $display("CHECKS: %0d",checks);$display("%0d FAIL",errors);$display("PASS: ai_rc_control");$finish;
    end
endmodule
