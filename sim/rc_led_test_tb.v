`timescale 1us/1ns
// Real board-test top + production HT10A capture, key debounce, LEDs and UART.
module rc_led_test_tb;
    reg clk=0,key_reset=1,key_view=1;
    always #0.5 clk=~clk;
    reg p0=0,p1=0,p2=0,p3=0,p4=0;
    wire [5:0] pins={1'b0,p4,p3,p2,p1,p0}; // J6.6 left disconnected.
    wire tx,stby;
    wire [3:0] in1,in2,pwm;
    wire [7:0] led;
    integer errors=0,checks=0;
    rc_led_test_top #(.CLK_HZ(1000000),.BAUD(100000),.REPORT_HZ(10),
        .KEY_DEBOUNCE_MS(1),.RESET_DEBOUNCE_MS(1),.EVENT_HOLD_MS(8)) dut(
        .i_clk_50m(clk),.i_key_reset_n(key_reset),.i_key1_n(key_view),.i_rc_pwm(pins),
        .o_uart_tx(tx),.o_stby(stby),.o_motor_in1(in1),.o_motor_in2(in2),.o_motor_pwm(pwm),.o_led(led));
    // Also check the constant motor pins before the reset chain initializes.
    always @(posedge clk) begin
        #0.01;
        if({stby,in1,in2,pwm}!==13'b0) begin errors=errors+1;$display("FAIL motor output must stay zero");end
    end
    task check;
        input [8*65-1:0] label;
        input integer got,want;
        begin checks=checks+1;
            if(got!==want) begin errors=errors+1;$display("FAIL %0s got=%0d expected=%0d",label,got,want);end
            else $display("pass %0s",label);
        end
    endtask
    task frame;
        input integer a,b,c,d,e;
        begin
            @(negedge clk);#0.2;p0=(a>0);p1=(b>0);p2=(c>0);p3=(d>0);p4=(e>0);
            fork
                begin #(a) p0=0;end begin #(b) p1=0;end begin #(c) p2=0;end
                begin #(d) p3=0;end begin #(e) p4=0;end
            join
            #30;
        end
    endtask
    task settled;
        input integer a,b,c,d,e;
        begin frame(a,b,c,d,e);frame(a,b,c,d,e);frame(a,b,c,d,e);end
    endtask
    task toggle_view;
        begin @(negedge clk);key_view=0;#1100;key_view=1;#1100;end
    endtask
    initial begin
        #1100;check("cold boot releases reset without S0",dut.rst_n,1);
        check("initial view is sticks",dut.view,0);check("no signal LED6 off",led[6],0);
        settled(1500,1500,1000,1000,1000);
        check("5 路活 + 第 6 路(CH8)未接仍可解锁（默认掩码 011111）",led[6],1);check("manual mode LED5 off",led[5],0);
        check("neutral direction LEDs off",led[4:1],0);
        settled(1500,2000,1000,1000,1000);check("CH2 forward LED4",led[4:1],8);
        settled(1500,1000,1000,1000,1000);check("CH2 reverse LED3",led[4:1],4);
        settled(2000,1500,1000,1000,1000);check("CH4 positive LED2",led[4:1],2);
        settled(1000,1500,1000,1000,1000);check("CH4 negative LED1",led[4:1],1);
        settled(1500,1500,1000,2000,1000);check("cursor mode LED5 on",led[5],1);
        toggle_view();check("S1 selects event view",dut.view,1);check("event view marker LED0",led[0],1);
        settled(1500,1500,2000,2000,1000);check("CH5 confirm stretches LED1",led[1],1);
        check("event does not leak to select",led[2],0);
        #9000;check("event display expires while signal stays fresh",led[1],0);
        settled(1500,1500,2000,2000,1000);check("held confirm does not repeat",led[1],0);
        settled(1500,1500,1000,2000,1000);
        settled(1500,1500,1000,2000,2000);check("CH6(闲置槽位4) select 事件不影响导航",led[2],1);
        settled(1500,1500,1000,2000,1000);#9000;
        settled(1500,2000,1000,2000,1000);check("menu up stretches LED3",led[3],1);
        settled(1500,1500,1000,2000,1000);#9000;
        settled(1500,1000,1000,2000,1000);check("menu down stretches LED4",led[4],1);
        frame(1500,1500,500,2000,1000);check("bad confirm channel invalidates LED6",led[6],0);
        check("invalid input clears all event LEDs",led[4:1],0);
        settled(1500,1500,1000,1000,1000);#101000;
        check("receiver disconnect makes LED6 off",led[6],0);
        check("motors inhibited after full test",{stby,in1,in2,pwm},0);
        check("view retained across signal loss",dut.view,1);
        key_reset=0;#1100;check("S0 clears view",dut.view,0);check("S0 clears valid LED",led[6],0);
        $display("CHECKS: %0d",checks);
        if(errors!=0)$fatal(1,"FAIL: rc_led_test");
        $display("PASS: rc_led_test");$finish;
    end
    initial begin #400000;$fatal(1,"FAIL: watchdog");end
endmodule
