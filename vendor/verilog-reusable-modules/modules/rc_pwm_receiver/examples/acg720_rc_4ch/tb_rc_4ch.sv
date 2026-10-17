`timescale 1ns/1ps
module tb_rc_4ch;
    reg clk=0;
    always #125 clk=~clk;
    reg rst_n=0;
    reg [3:0] rc_pwm=0;
    reg sw0=1;
    wire ds,sh_cp,st_cp;
    wire [7:0] led;
    integer width_us[0:3];
    reg [3:0] enabled=0;
    integer checks=0,failures=0,latches=0;
    rc_4ch_top #(.CLK_HZ(4000000),.TIMEOUT_US(30000),
        .CHANNEL_US(40000),.HEARTBEAT_US(10000),.DIGIT_US(100),
        .SNAPSHOT_US(500),.SCLK_DIVIDER(2)) dut (
        .clk(clk),.rst_n(rst_n),.rc_pwm(rc_pwm),.sw0(sw0),
        .ds(ds),.sh_cp(sh_cp),.st_cp(st_cp),.led(led));

    genvar c;
    generate for(c=0;c<4;c=c+1) begin: sources
        initial forever begin
            if(enabled[c]) begin
                rc_pwm[c]=1;
                #(width_us[c]*1000);
                rc_pwm[c]=0;
                #((20000-width_us[c])*1000);
            end else begin
                rc_pwm[c]=0;
                #1000;
            end
        end
    end endgenerate

    // Decode actual serial pins, exactly as two cascaded HC595 devices do.
    reg [15:0] shifted=0;
    integer serial_bits=0;
    integer digits[0:7];
    integer stamp[0:7];
    integer i,which;
    always @(posedge sh_cp) begin
        shifted={shifted[14:0],ds};
        serial_bits=serial_bits+1;
    end
    function integer decode;
        input [7:0] seg;
        begin
            case(seg)
                8'hc0:decode=0; 8'hf9:decode=1; 8'ha4:decode=2;
                8'hb0:decode=3; 8'h99:decode=4; 8'h92:decode=5;
                8'h82:decode=6; 8'hf8:decode=7; 8'h80:decode=8;
                8'h90:decode=9; 8'hbf:decode=-1;
                default:decode=-99;
            endcase
        end
    endfunction
    task check;
        input condition;
        input [511:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1) begin
                failures=failures+1;
                $display("FAIL t=%0t: %0s",$time,message);
            end
        end
    endtask
    always @(posedge st_cp) begin
        if(rst_n) begin
            check(serial_bits==16,"595 latch contains exactly 16 bits");
            which=-1;
            for(integer k=0;k<8;k=k+1)
                if(shifted[7:0]==(8'b1<<k)) which=k;
            check(which>=0,"595 digit select is one-hot");
            if(which>=0) begin
                digits[which]=decode(shifted[15:8]);
                stamp[which]=latches;
                check(digits[which]!=-99,"595 segment encoding recognized");
            end
            latches=latches+1;
        end
        serial_bits=0;
    end
    always @(negedge rst_n) begin
        serial_bits=0;
        shifted=0;
    end
    task display_check;
        input integer channel_number;
        input integer pulse;
        integer actual;
        begin
            // Longer than complete scan + snapshot; selection must be frozen.
            #3000000;
            actual=digits[7]*1000+digits[6]*100+digits[5]*10+digits[4];
            check(actual==channel_number,"physical left display identifies selected CH");
            for(integer j=0;j<8;j=j+1)
                check(latches-stamp[j]<=9,"every physical digit refreshed recently");
            if(pulse<0) begin
                check(digits[3]==-1 && digits[2]==-1 && digits[1]==-1 && digits[0]==-1,
                    "invalid selected channel displays four dashes");
            end else begin
                actual=digits[3]*1000+digits[2]*100+digits[1]*10+digits[0];
                if(actual!=pulse) $display("display actual=%0d expected=%0d",actual,pulse);
                check(actual==pulse,"physical right display equals measured PWM microseconds");
            end
        end
    endtask
    task select_channel;
        input [1:0] target;
        begin
            if(led[5:4]!=target) begin
                sw0=0;
                wait(led[5:4]==target);
                sw0=1;
                #10000;
            end
            check(led[5:4]==target,"SW0 freeze retains desired channel");
        end
    endtask
    reg heartbeat;
    initial begin
        for(i=0;i<8;i=i+1) begin digits[i]=-99;stamp[i]=-100;end
        width_us[0]=1000;width_us[1]=1500;width_us[2]=2000;width_us[3]=1250;
        #3000;rst_n=1;
        #3000;
        check(led[3:0]==0,"reset starts with all channels invalid");
        display_check(1,-1);
        enabled=4'b1111;
        #24000000;
        check(led[3:0]==15,"all four independent PWM inputs become valid");
        for(i=0;i<4;i=i+1) begin
            select_channel(i[1:0]);
            display_check(i+1,width_us[i]);
            check(led[6]==(i!=1),"activity LED reflects selected neutral/deviation");
        end
        heartbeat=led[7];#10000000;
        check(led[7]!=heartbeat,"heartbeat toggles at configured interval");
        // Independent change must appear on the physical display.
        select_channel(2'd1);width_us[1]=1750;
        #23000000;display_check(2,1750);
        check(led[6],"changed selected axis activates deviation LED");
        // Freeze for longer than selection period, then resume automatic scan.
        #45000000;check(led[5:4]==1,"frozen channel does not rotate");
        sw0=0;wait(led[5:4]==2);sw0=1;#10000;
        display_check(3,2000);
        // Invalid pulses affect only their channel.
        width_us[2]=500;#24000000;
        check(led[3:0]==4'b1011,"illegal pulse invalidates only CH3");
        check(!led[6],"invalid channel does not claim movement");
        display_check(3,-1);
        width_us[2]=2000;#24000000;
        check(led[3:0]==15,"valid pulse recovers illegal channel");
        display_check(3,2000);
        enabled[2]=0;#55000000;
        check(led[3:0]==4'b1011,"one missing channel times out while other three remain valid");
        display_check(3,-1);
        select_channel(2'd0);display_check(1,1000);
        enabled[2]=1;#24000000;
        check(led[3:0]==15,"timed-out channel recovers without global reset");
        // Reset while PWM streams continue: a partial first pulse must not survive reset.
        rst_n=0;#3000;
        check(led[3:0]==0,"asserted reset clears validity");
        rst_n=1;#45000000;
        check(led[3:0]==15,"all channels reacquire after reset");
        select_channel(2'd0);display_check(1,1000);
        $display("RC_4CH_RESULT checks=%0d failures=%0d physical_595_words=%0d",checks,failures,latches);
        if(failures!=0) $fatal(1,"four-channel verification failed");
        $finish;
    end
    initial begin #1500000000;$fatal(1,"testbench watchdog");end
endmodule
