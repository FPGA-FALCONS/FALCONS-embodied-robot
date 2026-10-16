`timescale 1ns/1ps
module rc_probe_report_tb;
    reg clk=0,rst_n=0;
    always #500 clk=~clk;
    reg view=0,good=1;
    reg [1:0] mode=1;
    reg signed [11:0] steer=-1000,throttle=500;
    reg [95:0] us={16'd0,16'd1000,16'd2000,16'd1500,16'd2000,16'd1500};
    reg [5:0] valid=31,timeout=32;
    reg [3:0] events=5;
    wire tx;
    integer checks=0,errors=0,len=0,n;
    reg [7:0] ch;
    reg [8*160-1:0] line;
    rc_probe_report #(.CLK_HZ(1000000),.BAUD(100000),.REPORT_HZ(20)) dut(
        .clk(clk),.rst_n(rst_n),.i_view(view),.i_signal_valid(good),.i_mode(mode),.i_steer(steer),
        .i_throttle(throttle),.i_chan_us(us),.i_valid(valid),.i_timeout(timeout),.i_events(events),.o_uart_tx(tx));
    task check;
        input [8*60-1:0] label;
        input integer got,want;
        begin checks=checks+1;
            if(got!==want) begin errors=errors+1;$display("FAIL %0s got=%0d expected=%0d",label,got,want);end
            else $display("pass %0s",label);
        end
    endtask
    task byte_rx;
        output [7:0] value;
        integer bitno;
        begin
            @(negedge tx);#15000;
            for(bitno=0;bitno<8;bitno=bitno+1) begin value[bitno]=tx;#10000;end
            if(tx!==1) begin errors=errors+1;$display("FAIL UART stop bit");end
        end
    endtask
    task row_rx;
        begin
            line=0;len=0;ch=0;
            while(ch!=10 && len<160) begin
                byte_rx(ch);line={line[8*159-1:0],ch};len=len+1;
            end
        end
    endtask
    initial begin
        #2000;check("UART idle during reset",tx,1);@(negedge clk);rst_n=1;
        fork
            row_rx();
            begin
                #1000000; // Mid-line changes must not corrupt the captured row.
                view=1;good=0;mode=0;steer=0;throttle=-1000;
                us={16'd65535,16'd0,16'd900,16'd2100,16'd1500,16'd1000};
                valid=0;timeout=63;events=0;
            end
        join
        check("first row length",len,102);
        check("first row atomic snapshot",line=={"RC,+00000,+00001,+00001,-01000,+00500,+01500,+02000,+01500,+02000,+01000,+00000,+00031,+00032,+00005",8'h0d,8'h0a},1);
        row_rx();check("second row length",len,102);
        check("second row updated values and 65535",line=={"RC,+00001,+00000,+00000,+00000,-01000,+01000,+01500,+02100,+00900,+00000,+65535,+00000,+00063,+00000",8'h0d,8'h0a},1);
        $display("CHECKS: %0d",checks);
        if(errors!=0)$fatal(1,"FAIL: rc_probe_report");
        $display("PASS: rc_probe_report");$finish;
    end
    initial begin #100000000;$fatal(1,"FAIL: watchdog");end
endmodule
