`timescale 1us/1ns
// Integration: actual HT10A profile + unchanged vendor pulse capture + manual interlock.
module ht10a_input_tb;
    reg clk=0, rst_n=0;
    always #0.5 clk=~clk;
    reg ch4=0,ch2=0,ch5=0,ch7=0,ch6=0,ch8=0;
    wire [5:0] pins={ch8,ch6,ch7,ch5,ch2,ch4};
    wire signed [11:0] steer,throttle;
    wire [1:0] mode;
    wire sel,confirm,back,up,down,good;
    wire [95:0] widths;
    wire [5:0] expired,valid;
    reg estop=0,stop=0;
    wire enable,hint;
    integer errors=0,checks=0,selects=0,confirms=0,backs=0,ups=0,downs=0;
    reg last_sel=0,last_confirm=0,last_back=0,last_up=0,last_down=0;
    ht10a_input #(.CLK_FREQ_HZ(1000000),.TIMEOUT_US(10000)) dut(
        .clk(clk),.rst_n(rst_n),.rc_pwm(pins),.o_steer(steer),.o_throttle(throttle),.o_mode(mode),
        .o_btn_select(sel),.o_btn_confirm(confirm),.o_btn_back(back),.o_menu_up(up),.o_menu_down(down),
        .o_signal_valid(good),.o_chan_us(widths),.o_chan_timeout(expired),.o_chan_valid(valid));
    rc_drive_guard guard(.clk(clk),.rst_n(rst_n),.i_manual(mode==0),.i_signal_valid(good),
        .i_estop(estop),.i_stop_pulse(stop),.i_throttle(throttle),.o_enable(enable),.o_arm_hint(hint));
    // Independent calibrated/reversed instance verifies parameters actually affect the arithmetic.
    wire signed [11:0] cal_s,cal_t;
    wire cal_good;
    wire [1:0] cal_mode;
    wire cal_select;
    ht10a_input #(.CLK_FREQ_HZ(1000000),.TIMEOUT_US(10000),
        .STEER_CENTER_US(1480),.THROTTLE_CENTER_US(1520),
        .STEER_HALF_RANGE_US(400),.THROTTLE_HALF_RANGE_US(400),
        .STEER_DEADZONE_US(20),.THROTTLE_DEADZONE_US(20),
        .STEER_REVERSE(1),.THROTTLE_REVERSE(1),.MODE_REVERSE(1),.SELECT_ACTIVE_LOW(1)) calibrated(
        .clk(clk),.rst_n(rst_n),.rc_pwm(pins),.o_steer(cal_s),.o_throttle(cal_t),.o_mode(cal_mode),
        .o_signal_valid(cal_good),.o_btn_select(cal_select),.o_btn_confirm(),.o_btn_back(),
        .o_menu_up(),.o_menu_down(),.o_chan_us(),.o_chan_timeout(),.o_chan_valid());
    integer cal_selects=0;
    always @(posedge clk) begin
        #0.01;
        if (sel) selects=selects+1;
        if (confirm) confirms=confirms+1;
        if (back) backs=backs+1;
        if (up) ups=ups+1;
        if (down) downs=downs+1;
        if (cal_select) cal_selects=cal_selects+1;
        if ((sel&&last_sel)||(confirm&&last_confirm)||(back&&last_back)||(up&&last_up)||(down&&last_down)) begin
            errors=errors+1; $display("FAIL: event wider than one clock");
        end
        last_sel=sel;last_confirm=confirm;last_back=back;last_up=up;last_down=down;
    end
    task check;
        input [8*70-1:0] label;
        input integer actual,expected;
        begin
            checks=checks+1;
            if (actual!==expected) begin errors=errors+1;$display("FAIL: %0s got=%0d expected=%0d",label,actual,expected);end
            else $display("pass %0s",label);
        end
    endtask
    // Receiver channel numbers, NOT slot numbers, make mapping mistakes visible in tests.
    task frame;
        input integer w4,w2,w5,w7,w6,w8;
        begin
            @(negedge clk); #0.2;
            ch4=(w4>0);ch2=(w2>0);ch5=(w5>0);ch7=(w7>0);ch6=(w6>0);ch8=(w8>0);
            fork
                begin #(w4) ch4=0;end
                begin #(w2) ch2=0;end
                begin #(w5) ch5=0;end
                begin #(w7) ch7=0;end
                begin #(w6) ch6=0;end
                begin #(w8) ch8=0;end
            join
            #30;
        end
    endtask
    task settled;
        input integer w4,w2,w5,w7,w6,w8;
        begin frame(w4,w2,w5,w7,w6,w8);frame(w4,w2,w5,w7,w6,w8);frame(w4,w2,w5,w7,w6,w8); end
    endtask
    integer before_s,before_c,before_b,before_u,before_d;
    initial begin
        #10;check("reset signal invalid",good,0);check("reset mode safe cursor",mode,1);check("reset no drive",enable,0);
        @(negedge clk);rst_n=1;#100;
        check("no initial pulses is invalid before timeout",good,0);
        // Startup/reconnect with switches already held must NOT click or drive.
        settled(1500,2000,2000,2000,1000,2000);
        check("valid six channel capture",good,1);check("manual acquired",mode,0);
        check("startup noncenter throttle cannot arm",enable,0);check("arming hint",hint,1);
        check("held startup confirm suppressed",confirms,0);check("manual menu suppressed",ups+downs,0);
        settled(1500,1500,1000,1000,1000,1000);
        check("neutral arms manual",enable,1);
        settled(1500,2000,1000,1000,1000,1000);
        check("CH2 forward full",throttle,1000);check("CH2 does not steer",steer,0);
        check("leaving center keeps drive enabled",enable,1);
        settled(2000,1500,1000,1000,1000,1000);
        check("CH4 right full",steer,1000);check("CH4 does not throttle",throttle,0);
        settled(1000,1000,1000,1000,1000,1000);
        check("simultaneous CH4 negative",steer,-1000);check("simultaneous CH2 negative",throttle,-1000);
        check("raw slot zero is CH4",widths[15:0],1000);check("raw slot one is CH2",widths[31:16],1000);
        settled(1525,1475,1000,1000,1000,1000);
        check("positive deadzone boundary",steer,0);check("negative deadzone boundary",throttle,0);
        settled(1526,1474,1000,1000,1000,1000);
        check("positive first step",steer,2);check("negative first step symmetric",throttle,-2);
        settled(1750,1250,1000,1000,1000,1000);
        check("calibrated linear positive midpoint",steer,473);check("calibrated linear negative midpoint",throttle,-473);
        settled(2100,900,1000,1000,1000,1000);
        check("valid extended endpoint clamps positive",steer,1000);check("valid extended endpoint clamps negative",throttle,-1000);
        @(negedge clk);stop=1;#0.1;check("S1 stops immediately",enable,0);@(negedge clk);stop=0;#2;
        check("S1 remains stopped",enable,0);
        @(negedge clk);stop=1;@(negedge clk);stop=0;#2;
        check("S1 release at noncenter stays disarmed",enable,0);
        settled(1500,1500,1000,1000,1000,1000);check("S1 rearm after center",enable,1);
        estop=1;#0.1;check("estop immediate disable",enable,0);#2;
        settled(1500,2000,1000,1000,1000,1000);estop=0;#2;
        check("estop recovery noncenter blocked",enable,0);
        settled(1500,1500,1000,1000,1000,1000);
        settled(1500,1500,1000,1000,2000,1000);
        check("CH6 cursor mode",mode,1);check("cursor cannot drive",enable,0);
        before_u=ups;before_d=downs;
        settled(1500,2000,1000,1000,2000,1000);
        check("CH2 cursor up once",ups-before_u,1);check("cursor up does not down",downs-before_d,0);
        settled(1500,2000,1000,1000,2000,1000);check("held stick no repeat",ups-before_u,1);
        settled(1500,1000,1000,1000,2000,1000);check("endpoint jump is not recenter",downs-before_d,0);
        settled(1500,1500,1000,1000,2000,1000);
        settled(1500,1000,1000,1000,2000,1000);check("center then down once",downs-before_d,1);
        settled(1500,1500,1000,1000,2000,1000);
        before_s=selects;before_c=confirms;before_b=backs;
        settled(1500,1500,2000,2000,2000,2000);
        check("CH5 confirm one click",confirms-before_c,1);check("CH7 back one click",backs-before_b,1);
        check("CH8 select one click",selects-before_s,1);
        settled(1500,1500,2000,2000,2000,2000);check("held button no repeat",selects-before_s,1);
        settled(1500,1500,1000,1000,2000,1000);
        before_c=confirms;
        frame(1500,1500,2000,1000,2000,1000);
        frame(1500,1500,1000,1000,2000,1000);check("one frame switch glitch ignored",confirms,before_c);
        settled(1500,1500,1000,1000,1000,1000);check("manual baseline before hysteresis",mode,0);
        settled(1500,1500,1000,1000,1310,1000);check("mode hysteresis near 1300 holds manual",mode,0);
        settled(1500,1500,1000,1000,1400,1000);check("mode leaves manual past hysteresis",mode,1);
        // Invalid *auxiliary* input also invalidates and zeros both driving commands.
        settled(2000,2000,1000,1000,2000,1000);
        frame(2000,2000,500,1000,2000,1000);
        check("illegal auxiliary pulse invalidates whole input",good,0);
        check("invalid auxiliary zeros steer",steer,0);check("invalid auxiliary zeros throttle",throttle,0);
        check("invalid mode safe cursor",mode,1);
        settled(2000,2000,2000,2000,2000,2000);
        check("reconnect held confirm no phantom",confirms,before_c);
        // Recover through release, then a real press is accepted again.
        settled(1500,1500,1000,1000,2000,1000);before_c=confirms;
        settled(1500,1500,2000,1000,2000,1000);check("release then press after reconnect",confirms-before_c,1);
        #11000;check("timeout invalid",good,0);check("timeout zeros command",throttle,0);check("timeout stops drive",enable,0);
        before_c=confirms;settled(1500,1500,2000,1000,2000,1000);
        check("timeout reconnect held button no phantom",confirms,before_c);
        // Clock continues but a stuck-high CH4 must invalidate, not wrap into legality.
        @(negedge clk);ch4=1;#2200;
        check("stuck high invalid",good,0);check("stuck high zeros steer",steer,0);ch4=0;#10;
        // Calibrated center/travel and reverse are independent for the two axes.
        settled(1480,1520,1000,1000,1000,2000);
        check("custom steer center",cal_s,0);check("custom throttle center",cal_t,0);
        check("mode reverse low is cursor",cal_mode,1);
        settled(1880,1120,1000,1000,1000,2000);
        check("custom steer travel reversed",cal_s,-1000);check("custom throttle travel reversed",cal_t,1000);
        before_s=cal_selects;settled(1480,1520,1000,1000,1000,1000);
        check("active low select",cal_selects-before_s,1);
        settled(1480,1520,1000,1000,2000,1000);check("mode reverse high is manual",cal_mode,0);
        // Missing CH8 refresh while the other five continue => all-good drops by timeout.
        settled(1500,1500,1000,1000,1000,1000);
        repeat(7) frame(1500,2000,1000,1000,1000,0);
        check("single channel timeout invalidates all",good,0);check("single channel timeout stops",enable,0);
        check("single channel timeout mask",expired[5],1);
        settled(1500,2000,1000,1000,1000,1000);
        check("loss recovery at noncenter remains disarmed",enable,0);
        settled(1500,1500,1000,1000,1000,1000);
        check("loss recovery centered rearms",enable,1);
        @(negedge clk);rst_n=0;#0.1;
        check("runtime reset disables immediately",enable,0);check("runtime reset invalid immediately",good,0);
        #2;check("runtime reset commands zero",throttle,0);check("runtime reset safe mode",mode,1);
        $display("CHECKS: %0d",checks);
        if(errors==0) $display("PASS: ht10a_input integration");
        else $fatal(1,"FAIL: ht10a_input errors=%0d",errors);
        $finish;
    end
    initial begin #1000000;$fatal(1,"FAIL: watchdog");end
endmodule
