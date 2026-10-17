`timescale 1ns/1ps
// Four MHz acceleration only changes clock-count parameters; all RC times are us.
// Display scoreboard decodes external DS/SH_CP/ST_CP, never DUT internals.
module tb_rc_debug;
    reg clk = 0;
    always #125 clk = ~clk;
    reg rst_n = 0, steering_pwm = 0, throttle_pwm = 0, sw0 = 0;
    wire ds, sh_cp, st_cp;
    wire [7:0] led;
    rc_debug_top #(.CLK_HZ(4000000), .TIMEOUT_US(15000),
        .SNAPSHOT_US(100), .DIGIT_US(100), .SCLK_DIVIDER(2),
        .HEARTBEAT_US(1000)) dut (
        .clk(clk), .rst_n(rst_n), .rc_steering_pwm(steering_pwm),
        .rc_throttle_pwm(throttle_pwm), .sw0(sw0), .ds(ds),
        .sh_cp(sh_cp), .st_cp(st_cp), .led(led)
    );
    integer width_steering = 1500, width_throttle = 1500;
    reg enable_steering = 0, enable_throttle = 0;
    integer checks = 0, groups = 0, latches = 0, heartbeat_edges = 0;
    integer bits_shifted = 0, index, scan_i, last_index = -1;
    reg [15:0] shift_word = 0;
    reg [7:0] observed [0:7];
    reg [7:0] seen = 0;
    time last_shift_rise = 0, last_shift_fall = 0, last_latch_rise = 0;
    task require;
        input condition;
        input [511:0] label;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) $fatal(1,"FAIL %0t: %0s",$time,label);
        end
    endtask
    function [7:0] character;
        input [7:0] segments;
        begin
            case (segments)
                8'hc0: character="0"; 8'hf9: character="1";
                8'ha4: character="2"; 8'hb0: character="3";
                8'h99: character="4"; 8'h92: character="5";
                8'h82: character="6"; 8'hf8: character="7";
                8'h80: character="8"; 8'h90: character="9";
                8'hbf: character="-"; default: character="?";
            endcase
        end
    endfunction
    always @(posedge sh_cp) if (rst_n) begin
        require(st_cp===0,"no shifting during latch");
        require(ds===0 || ds===1,"known serial data");
        if (last_shift_fall!=0) require($time-last_shift_fall>=500,"595 low time");
        last_shift_rise=$time;
        shift_word={shift_word[14:0],ds};
        bits_shifted=bits_shifted+1;
        require(bits_shifted<=16,"at most 16 shifts");
    end
    always @(negedge sh_cp) if (rst_n && last_shift_rise!=0) begin
        require($time-last_shift_rise>=500,"595 high time");
        last_shift_fall=$time;
    end
    always @(posedge st_cp) if (rst_n) begin
        require(sh_cp===0,"latch while shift clock low");
        require(bits_shifted==16,"16 bits per physical latch");
        require($time-last_shift_fall>=500,"595 shift to latch separation");
        require(shift_word[7:0]!=0 &&
            (shift_word[7:0] & (shift_word[7:0]-8'd1))==0,"one hot digit");
        require(shift_word[15]===1,"decimal point disabled");
        require(character(shift_word[15:8])!="?","legal segment code");
        index=0;
        for (scan_i=0;scan_i<8;scan_i=scan_i+1)
            if (shift_word[scan_i]) index=scan_i;
        if(last_index>=0) begin
            require(index==((last_index+1)%8),"scan order");
            require($time-last_latch_rise==100000,"100us scan cadence");
        end
        last_index=index;
        last_latch_rise=$time;
        observed[index]=character(shift_word[15:8]);
        seen[index]=1;
        bits_shifted=0;
        latches=latches+1;
    end
    always @(negedge st_cp) if(rst_n && last_latch_rise!=0)
        require($time-last_latch_rise>=500,"595 latch pulse width");
    always @(led[7]) if(rst_n) heartbeat_edges=heartbeat_edges+1;
    // Independent continuously running inputs. New requested values are latched
    // at the following frame boundary, as a real transmitter/receiver would do.
    initial forever begin
        @(negedge clk);
        fork
            begin
                if(enable_steering) begin
                    steering_pwm=1; #(width_steering*1000); steering_pwm=0;
                end
                #5000000;
            end
            begin
                if(enable_throttle) begin
                    throttle_pwm=1; #(width_throttle*1000); throttle_pwm=0;
                end
                #5000000;
            end
        join
    end
    task expect_observation;
        input [63:0] expected;
        input [6:0] expected_led;
        input [511:0] label;
        integer ch;
        begin
            // Covers next 7ms maximum generator period, capture, normalization,
            // 100us BCD snapshot and at least two complete 800us scans.
            #12000000;
            require(seen==8'hff,"all external digits observed");
            for(ch=0;ch<8;ch=ch+1) begin
                if(observed[ch]!==expected[ch*8 +:8]) begin
                    $display("Expected %0s got %c%c%c%c%c%c%c%c",expected,
                        observed[7],observed[6],observed[5],observed[4],
                        observed[3],observed[2],observed[1],observed[0]);
                    require(0,label);
                end
                require(1,"external digit matches expected");
            end
            require(led[6:0]===expected_led,label);
            groups=groups+1;
            $display("PASS %0d %0s display=%0s led=%b",groups,label,expected,led[6:0]);
        end
    endtask
    initial begin
        for(scan_i=0;scan_i<8;scan_i=scan_i+1) observed[scan_i]="?";
        #2000; @(negedge clk); rst_n=1;
        expect_observation("--------",7'b0000000,"startup invalid and no direction");
        enable_steering=1; width_steering=1000;
        expect_observation("1000----",7'b0100000,"raw one channel independently valid");
        enable_throttle=1; width_throttle=2000;
        expect_observation("10002000",7'b1001001,"raw limits also drive real command LEDs");
        sw0=1;
        expect_observation("10001000",7'b1011001,"negative steering positive throttle");
        width_steering=2000; width_throttle=1000;
        expect_observation("10001000",7'b1010110,"positive steering negative throttle");
        width_steering=1500; width_throttle=1500;
        expect_observation("00000000",7'b1010000,"valid center distinctly zero");
        sw0=0;
        expect_observation("15001500",7'b1000000,"raw center on both channels");
        sw0=1;
        expect_observation("00000000",7'b1010000,"return to normalized center");
        width_steering=1750; width_throttle=1250;
        sw0=0;
        expect_observation("17501250",7'b1000110,"switch to raw during live input change");
        width_steering=1250; width_throttle=1750;
        sw0=1;
        expect_observation("04730473",7'b1011001,"switch to command while both inputs change");
        enable_throttle=0;
        #17000000;
        expect_observation("--------",7'b0110000,"one lost channel clears commands and directions");
        sw0=0;
        expect_observation("1250----",7'b0100000,"raw surviving channel remains visible");
        enable_throttle=1; width_throttle=1500; width_steering=1500;
        sw0=1;
        expect_observation("00000000",7'b1010000,"recovery after real timeout");
        width_steering=400;
        expect_observation("--------",7'b0010000,"illegal width invalidates command pair");
        width_steering=2000; width_throttle=2000;
        expect_observation("10001000",7'b1011010,"recovery from illegal pulse");
        require(heartbeat_edges>10,"heartbeat is alive");
        $display("PASS tb_rc_debug groups=%0d checks=%0d physical_latches=%0d",groups,checks,latches);
        $finish;
    end
    initial begin #400000000; $fatal(1,"testbench watchdog"); end
endmodule

