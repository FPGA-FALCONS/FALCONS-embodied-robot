`timescale 1ns/1ps
module inspection_ui_tb;
    reg clk=0,rst=0,enter=0,back=0,prev=0,next=0,confirm=0,manual=0,close_child=0,host_alive=1;
    wire master,recording,vision,chat,open_task;wire [1:0] mode,selected,view;
    integer checks=0,errors=0;
    always #500 clk=~clk;
    ai_rc_control #(.CLK_HZ(1_000_000),.INSPECTION_ENABLE(1)) dut(
        .clk(clk),.rst_n(rst),.i_enter_confirm(enter),.i_ai_page(1'b1),.i_back(back),
        .i_select_prev(prev),.i_select_next(next),.i_feature_confirm(confirm),
        .i_rc_valid(1'b1),.i_manual(manual),.i_estop(1'b0),.i_swb_pwm(1'b0),.i_host_alive(host_alive),
        .i_inspection_close(close_child),.o_inspection_open(open_task),.o_inspection_view(view),
        .o_master(master),.o_record(recording),.o_chat(chat),.o_vision(vision),.o_mode(mode),.o_selected(selected),.o_flags(),.o_controls());
    task check;input condition;input [511:0] name;
        begin checks=checks+1;if(condition===1'b1)$display("pass %0s",name);
        else begin errors=errors+1;$display("FAIL: %0s",name);end end
    endtask
    task step;
        begin @(negedge clk);next=1;@(negedge clk);next=0;@(negedge clk);end
    endtask
    task choose;
        begin @(negedge clk);confirm=1;@(negedge clk);confirm=0;@(negedge clk);end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;repeat(3)@(negedge clk);
        enter=1;@(negedge clk);enter=0;step();choose();
        check(master && !open_task && mode==1,"original recognition remains independent of inspection");
        step();step();choose();
        check(master && open_task && mode==3 && selected==3 && view==1,"fourth feature enters independent inspection map view");
        step();check(view==2 && selected==3 && mode==3,"CH8 changes child view without switching top feature");
        step();check(view==0,"child views wrap through camera map and list");
        choose();check(open_task && mode==3,"child confirmation cannot exit or change feature");
        back=1;@(negedge clk);back=0;@(negedge clk);
        check(master && open_task,"child back preserves AI master while host handles task state");
        manual=1;step();choose();check(master && open_task && view==0 && !recording,"manual preserves task page and blocks its controls");
        manual=0;@(negedge clk);check(master && open_task && view==0,"cursor return preserves current child view");
        close_child=1;@(negedge clk);close_child=0;@(negedge clk);
        check(master && !open_task && selected==3,"stationary child exit returns to overview with master enabled");
        step();choose();check(master && !open_task && mode==0 && !chat,"original semantic navigation remains selectable after inspection");
        step();step();choose();check(master && !open_task && mode==2 && chat,"original chatbot remains selectable after leaving inspection");
        back=1;@(negedge clk);back=0;@(negedge clk);check(!master,"overview back closes AI master");
        enter=1;@(negedge clk);enter=0;step();step();step();choose();host_alive=0;@(negedge clk);@(negedge clk);
        check(!master && !open_task,"host loss closes child and master locally");
        $display("CHECKS: %0d",checks);
        if(!errors)$display("PASS: inspection_ui %0d checks",checks);else $display("FAIL: inspection_ui %0d",errors);
        $finish;
    end
endmodule
