`timescale 1ns/1ps
// Boundary fault injection targets capture-to-command handshaking; the final
// section releases every force and exercises real 50 Hz / 330 Hz input PWM.
module rc_receiver_pipeline_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg pwm1 = 0, pwm2 = 0;
    wire signed [11:0] steering, throttle;
    wire signal_valid;
    wire [15:0] width1, width2;
    wire valid1, valid2, timeout1, timeout2;
    reg [15:0] injected_width1 = 1500, injected_width2 = 1500;
    reg injected_valid1 = 0, injected_valid2 = 0;
    reg injected_update1 = 0, injected_update2 = 0;
    reg injected_timeout1 = 0, injected_timeout2 = 0;
    integer checks = 0, groups = 0, index;
    reg require_continuous = 0;
    rc_receiver dut (
        .clk(clk), .rst_n(rst_n), .rc_ch1(pwm1), .rc_ch2(pwm2),
        .steering(steering), .throttle(throttle), .signal_valid(signal_valid),
        .ch1_width_us(width1), .ch2_width_us(width2),
        .ch1_valid(valid1), .ch2_valid(valid2),
        .ch1_timeout(timeout1), .ch2_timeout(timeout2)
    );
    task require;
        input condition;
        input [639:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1)
                $fatal(1,"FAIL at %0t: %0s; valid=%b pair=%0d/%0d",$time,message,
                    signal_valid,$signed(steering),$signed(throttle));
        end
    endtask
    always @(posedge clk) begin
        #2;
        if (!signal_valid && (steering !== 0 || throttle !== 0))
            $fatal(1,"invalid command not zero");
        if (require_continuous && !signal_valid)
            $fatal(1,"legal fresh PWM caused a false signal-loss indication");
    end
    task request_pair;
        input [15:0] new1, new2;
        begin
            @(negedge clk);
            injected_width1 = new1; injected_width2 = new2;
            injected_valid1 = 1; injected_valid2 = 1;
            injected_update1 = 1; injected_update2 = 1;
            @(posedge clk); #1;
            @(negedge clk); injected_update1 = 0; injected_update2 = 0;
        end
    endtask
    task expect_pair;
        input integer expected1, expected2;
        begin
            require(signal_valid,"committed commands valid");
            require($signed(steering)==expected1 && $signed(throttle)==expected2,
                    "atomic latest command pair");
        end
    endtask
    task wait_commit;
        begin
            repeat (28) @(posedge clk);
            #1;
        end
    endtask
    task real_pair;
        input integer duration1, duration2;
        begin
            @(negedge clk);
            fork
                begin #3; pwm1=1; #(duration1*1000); pwm1=0; end
                begin #7; pwm2=1; #(duration2*1000); pwm2=0; end
            join
            #2000;
        end
    endtask
    initial begin
        force dut.ch1_width_us = injected_width1;
        force dut.ch2_width_us = injected_width2;
        force dut.ch1_valid = injected_valid1;
        force dut.ch2_valid = injected_valid2;
        force dut.ch1_update = injected_update1;
        force dut.ch2_update = injected_update2;
        force dut.ch1_timeout = injected_timeout1;
        force dut.ch2_timeout = injected_timeout2;
        #100; rst_n = 1; #100;
        request_pair(1000,2000);
        for(index=1; index<=27; index=index+1) begin
            @(posedge clk); #1;
            require(!signal_valid,"initial commands cannot be valid before commit");
        end
        @(posedge clk); #1; expect_pair(-1000,1000);
        groups=groups+1;
        require_continuous=1;
        request_pair(1800,1200);
        repeat (27) begin @(posedge clk); #1; expect_pair(-1000,1000); end
        // done is now asserted, but a fresh update wins over the old result.
        @(negedge clk);
        injected_width1=2000; injected_width2=1000;
        injected_update1=1; injected_update2=1;
        @(posedge clk); #1; expect_pair(-1000,1000);
        @(negedge clk); injected_update1=0; injected_update2=0;
        repeat (27) begin @(posedge clk); #1; expect_pair(-1000,1000); end
        @(posedge clk); #1; expect_pair(1000,-1000);
        groups=groups+1;
        // CH1 updates alone: CH2 must retain its latest captured snapshot.
        @(negedge clk); injected_width1=1500; injected_update1=1;
        @(posedge clk); #1; expect_pair(1000,-1000);
        @(negedge clk); injected_update1=0;
        wait_commit; expect_pair(0,-1000);
        groups=groups+1;
        request_pair(1000,2000);
        repeat (27) @(posedge clk);
        require_continuous=0;
        @(negedge clk); injected_valid1=0;
        #1; require(!signal_valid && steering==0 && throttle==0,
                    "invalid capture immediately masks completed but uncommitted result");
        repeat (35) begin @(posedge clk); #1;
            require(!signal_valid,"invalid session cannot resurrect old result"); end
        request_pair(2000,1500);
        repeat (27) begin @(posedge clk); #1;
            require(!signal_valid,"recovery waits for new session calculation"); end
        @(posedge clk); #1; expect_pair(1000,0);
        groups=groups+1;
        request_pair(1000,2000);
        repeat (27) @(posedge clk);
        @(negedge clk); injected_timeout2=1;
        #1; require(!signal_valid && steering==0 && throttle==0,
                    "timeout wins over same-cycle completed calculation");
        repeat (30) @(posedge clk);
        @(negedge clk); injected_timeout2=0;
        request_pair(1500,1500); wait_commit; expect_pair(0,0);
        groups=groups+1;
        request_pair(1000,2000);
        repeat (26) @(posedge clk);
        @(negedge clk); rst_n=0; #1;
        require(!signal_valid && steering==0 && throttle==0,
                "asynchronous reset masks in-flight results immediately");
        release dut.ch1_width_us; release dut.ch2_width_us;
        release dut.ch1_valid; release dut.ch2_valid;
        release dut.ch1_update; release dut.ch2_update;
        release dut.ch1_timeout; release dut.ch2_timeout;
        // Toggle reset after releasing forced capture registers.
        #100; rst_n=1; #100; rst_n=0; #100; rst_n=1; #1000;
        real_pair(1000,2000); expect_pair(-1000,1000);
        require_continuous=1;
        for(index=0; index<3; index=index+1) begin
            #17998000;
            if(index==1) begin real_pair(2000,1000); expect_pair(1000,-1000); end
            else begin real_pair(1000,2000); expect_pair(-1000,1000); end
        end
        groups=groups+1;
        for(index=0; index<8; index=index+1) begin
            #1028300;
            if(index[0]) begin real_pair(2000,1000); expect_pair(1000,-1000); end
            else begin real_pair(1000,2000); expect_pair(-1000,1000); end
        end
        groups=groups+1;
        require_continuous=0;
        #100100000;
        require(!signal_valid && timeout1 && timeout2,
                "production 100 ms timeout invalidates last command");
        real_pair(1500,1500); expect_pair(0,0);
        groups=groups+1;
        $display("PASS rc_receiver_pipeline: %0d groups, %0d checks; atomic commit, collision/cancel/recovery, real 50/330 Hz continuous valid",groups,checks);
        $finish;
    end
    initial begin #300000000; $fatal(1,"watchdog"); end
endmodule
