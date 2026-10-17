`timescale 1ns/1ps

// Self-checking tests; no board pins or Gowin device primitives are required.
// Compile with Icarus Verilog -g2012 for $fatal (RTL remains Verilog 2001).
// The main DUT uses 10 ms timeout to shorten tests. A separate capture instance
// checks the production/default 100 ms timeout at the same 50 MHz clock.
module rc_receiver_tb;
    reg clk;
    reg rst_n;
    reg rc_ch1;
    reg rc_ch2;
    wire signed [11:0] steering;
    wire signed [11:0] throttle;
    wire signal_valid;
    wire [15:0] ch1_width_us;
    wire [15:0] ch2_width_us;
    wire ch1_valid;
    wire ch2_valid;
    wire ch1_timeout;
    wire ch2_timeout;

    wire [15:0] monitor_width_us;
    wire monitor_valid;
    wire monitor_timeout;
    wire monitor_update;

    reg default_rst_n;
    reg default_pwm;
    wire [15:0] default_width_us;
    wire default_valid;
    wire default_timeout;
    wire default_update;

    integer checks;
    integer groups;
    integer monitor_updates;
    integer default_updates;
    integer saved_updates;
    integer frame_index;
    reg previous_monitor_update;
    reg previous_default_update;

    rc_receiver #(
        .CLK_HZ(50000000),
        .MIN_PULSE_US(900),
        .MAX_PULSE_US(2100),
        .TIMEOUT_US(10000),
        .CH1_MIN_US(1000),
        .CH1_CENTER_US(1500),
        .CH1_MAX_US(2000),
        .CH1_DEADZONE_US(25),
        .CH2_MIN_US(1000),
        .CH2_CENTER_US(1500),
        .CH2_MAX_US(2000),
        .CH2_DEADZONE_US(25),
        .CH1_REVERSE(0),
        .CH2_REVERSE(0)
    ) dut (
        .clk(clk), .rst_n(rst_n), .rc_ch1(rc_ch1), .rc_ch2(rc_ch2),
        .steering(steering), .throttle(throttle),
        .signal_valid(signal_valid),
        .ch1_width_us(ch1_width_us), .ch2_width_us(ch2_width_us),
        .ch1_valid(ch1_valid), .ch2_valid(ch2_valid),
        .ch1_timeout(ch1_timeout), .ch2_timeout(ch2_timeout)
    );

    // Direct access to capture's pulse_update, without hierarchy assumptions.
    rc_pwm_capture #(
        .CLK_HZ(50000000), .MIN_PULSE_US(900),
        .MAX_PULSE_US(2100), .TIMEOUT_US(10000)
    ) capture_monitor (
        .clk(clk), .rst_n(rst_n), .rc_pwm(rc_ch1),
        .pulse_width_us(monitor_width_us), .pulse_valid(monitor_valid),
        .timed_out(monitor_timeout), .pulse_update(monitor_update)
    );

    // Omitting TIMEOUT_US exercises the real default value, 100000 us.
    rc_pwm_capture #(
        .CLK_HZ(50000000), .MIN_PULSE_US(900), .MAX_PULSE_US(2100)
    ) default_capture (
        .clk(clk), .rst_n(default_rst_n), .rc_pwm(default_pwm),
        .pulse_width_us(default_width_us), .pulse_valid(default_valid),
        .timed_out(default_timeout), .pulse_update(default_update)
    );

    initial clk = 1'b0;
    always #10 clk = ~clk; // 20 ns period = 50 MHz.

    // Dump only useful low-rate ports; dumping 50 MHz counters/clock makes a
    // 100 ms timeout test needlessly large and slow.
    initial begin
        $dumpfile("rc_receiver_tb.vcd");
        $dumpvars(0, rst_n, rc_ch1, rc_ch2, steering, throttle, signal_valid,
            ch1_width_us, ch2_width_us, ch1_valid, ch2_valid,
            ch1_timeout, ch2_timeout, monitor_update,
            default_rst_n, default_pwm, default_width_us,
            default_valid, default_timeout, default_update);
    end

    initial begin
        #900000000;
        $fatal(1, "FAIL: global watchdog expired after 900 ms");
    end

    // Observe after nonblocking assignments have settled.
    always @(posedge clk) begin
        #1;
        if (!rst_n) begin
            previous_monitor_update = 1'b0;
            monitor_updates = 0;
        end else begin
            if ((^{steering, throttle, signal_valid, ch1_valid, ch2_valid,
                    ch1_timeout, ch2_timeout, monitor_update}) === 1'bx)
                $fatal(1, "FAIL: unknown public output at %0t", $time);
            if (!signal_valid && ((steering !== 12'sd0) ||
                                  (throttle !== 12'sd0)))
                $fatal(1, "FAIL: invalid receiver must output zero at %0t", $time);
            if (signal_valid && !(ch1_valid && ch2_valid))
                $fatal(1, "FAIL: signal_valid without two valid captures at %0t", $time);
            if (monitor_update && previous_monitor_update)
                $fatal(1, "FAIL: pulse_update lasted more than one clock");
            if (monitor_update) begin
                if (!monitor_valid || monitor_timeout)
                    $fatal(1, "FAIL: pulse_update without a valid capture");
                monitor_updates = monitor_updates + 1;
            end
            previous_monitor_update = monitor_update;
        end

        if (!default_rst_n) begin
            previous_default_update = 1'b0;
            default_updates = 0;
        end else begin
            if (default_update && previous_default_update)
                $fatal(1, "FAIL: default capture pulse_update lasted > 1 clock");
            if (default_update)
                default_updates = default_updates + 1;
            previous_default_update = default_update;
        end
    end

    task require;
        input condition;
        input [639:0] description;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                $display("FAIL at %0t: %0s", $time, description);
                $display("  CH1 width=%0d valid=%b timeout=%b; CH2 width=%0d valid=%b timeout=%b",
                    ch1_width_us, ch1_valid, ch1_timeout,
                    ch2_width_us, ch2_valid, ch2_timeout);
                $display("  steering=%0d throttle=%0d signal_valid=%b",
                    $signed(steering), $signed(throttle), signal_valid);
                $fatal(1, "Self-check failed");
            end
        end
    endtask

    task pass_group;
        input [639:0] description;
        begin
            groups = groups + 1;
            $display("PASS %0d: %0s", groups, description);
        end
    endtask

    task expect_safe;
        begin
            require(signal_valid === 1'b0, "receiver invalid");
            require(steering === 12'sd0, "invalid steering is safe zero");
            require(throttle === 12'sd0, "invalid throttle is safe zero");
        end
    endtask

    task expect_pair;
        input integer width1;
        input integer width2;
        input integer expected_steering;
        input integer expected_throttle;
        begin
            require(ch1_valid === 1'b1, "CH1 valid");
            require(ch2_valid === 1'b1, "CH2 valid");
            require(ch1_timeout === 1'b0, "CH1 has no timeout");
            require(ch2_timeout === 1'b0, "CH2 has no timeout");
            require(signal_valid === 1'b1, "both channels make receiver valid");
            require(ch1_width_us == width1, "CH1 captured width in microseconds");
            require(ch2_width_us == width2, "CH2 captured width in microseconds");
            require($signed(steering) == expected_steering, "steering mapping");
            require($signed(throttle) == expected_throttle, "throttle mapping");
            require(monitor_valid === 1'b1, "standalone capture valid");
            require(monitor_width_us == width1, "standalone captured width");
        end
    endtask

    // Use unequal phases deliberately: receiver signals are asynchronous to
    // the FPGA clock and need not rise/fall together.
    task drive_pair_ns;
        input integer width1_ns;
        input integer width2_ns;
        input integer phase1_ns;
        input integer phase2_ns;
        begin
            require((rc_ch1 === 1'b0) && (rc_ch2 === 1'b0), "pulse starts from low");
            @(negedge clk);
            fork
                begin
                    #(phase1_ns); rc_ch1 = 1'b1;
                    #(width1_ns); rc_ch1 = 1'b0;
                end
                begin
                    #(phase2_ns); rc_ch2 = 1'b1;
                    #(width2_ns); rc_ch2 = 1'b0;
                end
            join
            #1000;
        end
    endtask

    task drive_pair;
        input integer width1_us;
        input integer width2_us;
        begin
            drive_pair_ns(width1_us * 1000, width2_us * 1000, 7, 133);
        end
    endtask

    task drive_ch1;
        input integer width_us;
        begin
            @(negedge clk);
            #7; rc_ch1 = 1'b1;
            #(width_us * 1000); rc_ch1 = 1'b0;
            #1000;
        end
    endtask

    task drive_ch2;
        input integer width_us;
        begin
            @(negedge clk);
            #13; rc_ch2 = 1'b1;
            #(width_us * 1000); rc_ch2 = 1'b0;
            #1000;
        end
    endtask

    task reset_low;
        begin
            @(negedge clk);
            rst_n = 1'b0;
            rc_ch1 = 1'b0;
            rc_ch2 = 1'b0;
            #1000;
            expect_safe;
            require(ch1_width_us == 0 && ch2_width_us == 0, "reset clears widths");
            require(!ch1_valid && !ch2_valid, "reset clears channel valid");
            require(!ch1_timeout && !ch2_timeout, "reset clears timeout flags");
            rst_n = 1'b1;
            #1000;
        end
    endtask

    task invalid_ch1;
        input integer width_us;
        begin
            drive_pair(1500, 1500);
            saved_updates = monitor_updates;
            drive_pair(width_us, 1500);
            require(ch1_valid === 1'b0, "illegal CH1 clears valid");
            require(ch2_valid === 1'b1, "illegal CH1 does not invalidate CH2");
            require(ch2_width_us == 1500, "CH2 still captures legal pulse");
            require(monitor_updates == saved_updates, "illegal capture gives no update");
            expect_safe;
            drive_pair(1500, 1500);
            expect_pair(1500, 1500, 0, 0);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        rc_ch1 = 1'b0;
        rc_ch2 = 1'b0;
        default_rst_n = 1'b0;
        default_pwm = 1'b0;
        checks = 0;
        groups = 0;
        monitor_updates = 0;
        default_updates = 0;
        previous_monitor_update = 1'b0;
        previous_default_update = 1'b0;

        #200;
        expect_safe;
        require(ch1_width_us == 0 && ch2_width_us == 0, "initial reset widths zero");
        require(!ch1_valid && !ch2_valid, "initial reset valid zero");
        require(!ch1_timeout && !ch2_timeout, "initial reset timeout zero");
        @(negedge clk); rst_n = 1'b1;
        #200;
        expect_safe;
        pass_group("reset and startup outputs");

        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        drive_pair(1000, 2000);
        expect_pair(1000, 2000, -1000, 1000);
        drive_pair(2000, 1000);
        expect_pair(2000, 1000, 1000, -1000);
        require(monitor_updates == 3, "one update per valid CH1 pulse");
        pass_group("1000 / 1500 / 2000 us and signed endpoint mapping");

        drive_pair(1500, 1000);
        expect_pair(1500, 1000, 0, -1000);
        drive_pair(2000, 1000);
        expect_pair(2000, 1000, 1000, -1000);
        drive_pair(2000, 1500);
        expect_pair(2000, 1500, 1000, 0);
        drive_pair_ns(1000000, 2000000, 911, 19);
        expect_pair(1000, 2000, -1000, 1000);
        pass_group("independent channel changes and asynchronous phases");

        drive_pair(1474, 1526);
        expect_pair(1474, 1526, -2, 2);
        drive_pair(1475, 1525);
        expect_pair(1475, 1525, 0, 0);
        drive_pair(1525, 1475);
        expect_pair(1525, 1475, 0, 0);
        drive_pair(1526, 1474);
        expect_pair(1526, 1474, 2, -2);
        pass_group("inclusive +/-25 us deadzone and first values outside it");

        drive_pair(900, 2100);
        expect_pair(900, 2100, -1000, 1000);
        drive_pair(2100, 900);
        expect_pair(2100, 900, 1000, -1000);
        pass_group("900 / 2100 us legal capture bounds and command saturation");

        invalid_ch1(899);
        invalid_ch1(2101);
        invalid_ch1(100);
        invalid_ch1(500);
        invalid_ch1(2200);
        drive_pair(1500, 500);
        require(ch1_valid && !ch2_valid, "CH2 independently rejects short pulse");
        expect_safe;
        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        pass_group("899 / 2101 / 100 / 500 / 2200 us invalid pulses and recovery");

        saved_updates = monitor_updates;
        @(negedge clk); #7; rc_ch1 = 1'b1;
        #2101000;
        require(!ch1_valid && ch2_valid, "overlong CH1 invalid before falling edge");
        require(!monitor_valid, "standalone overlong capture invalid before fall");
        require(monitor_updates == saved_updates, "overlong pulse has no early update");
        expect_safe;
        rc_ch1 = 1'b0;
        #200;
        require(monitor_updates == saved_updates, "overlong falling edge has no update");
        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        pass_group("overlong pulse fails before fall and cannot restore valid");

        drive_pair(1000, 2000);
        repeat (4) begin
            #2000000;
            drive_ch2(1500);
        end
        require(ch1_timeout && !ch1_valid, "CH1 times out independently");
        require(ch2_valid && !ch2_timeout, "CH2 remains live while CH1 stops");
        require(monitor_timeout && !monitor_valid, "direct capture CH1 timeout");
        expect_safe;
        drive_ch1(1000);
        expect_pair(1000, 1500, -1000, 0);
        pass_group("CH1 loss with live CH2, both commands safe, CH1 recovery");

        repeat (4) begin
            #2000000;
            drive_ch1(2000);
        end
        require(ch2_timeout && !ch2_valid, "CH2 times out independently");
        require(ch1_valid && !ch1_timeout, "CH1 remains live while CH2 stops");
        expect_safe;
        drive_ch2(1000);
        expect_pair(2000, 1000, 1000, -1000);
        pass_group("CH2 loss with live CH1, both commands safe, CH2 recovery");

        reset_low;
        drive_pair(1500, 1500);
        saved_updates = monitor_updates;
        #3000000;
        drive_pair(500, 1500);
        #2000000;
        drive_pair(500, 1500);
        #2500000;
        require(ch1_timeout && monitor_timeout, "bad pulses do not refresh last-good timer");
        require(!ch1_valid && ch2_valid, "CH2 live during bad CH1 timeout");
        require(monitor_updates == saved_updates, "bad pulses give no update");
        expect_safe;
        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        pass_group("repeated bad pulses cannot keep last-good watchdog alive");

        #10100000;
        require(ch1_timeout && ch2_timeout, "both stuck-low inputs time out");
        require(!ch1_valid && !ch2_valid, "both stuck-low inputs invalid");
        expect_safe;
        pass_group("both inputs stuck low");

        drive_pair(1500, 1500);
        @(negedge clk); #7; rc_ch1 = 1'b1;
        #2200000;
        require(!ch1_valid, "stuck-high input rejected at overlong limit");
        expect_safe;
        #8000000;
        require(ch1_timeout && monitor_timeout, "stuck-high input eventually times out");
        rc_ch1 = 1'b0;
        #200;
        require(!ch1_valid, "falling edge of stuck-high cannot become valid");
        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        pass_group("stuck-high input, timeout and recovery");

        // A reset released during high must not accept the remaining high tail
        // as a complete pulse; this also exercises power-up with high inputs.
        @(negedge clk);
        rst_n = 1'b0; rc_ch1 = 1'b1; rc_ch2 = 1'b1;
        #200;
        require(ch1_width_us == 0 && ch2_width_us == 0, "reset clears prior pulse widths");
        expect_safe;
        rst_n = 1'b1;
        #1500000;
        rc_ch1 = 1'b0; rc_ch2 = 1'b0;
        #200;
        require(!ch1_valid && !ch2_valid, "initial high tail is not a complete pulse");
        require(monitor_updates == 0, "initial high tail has no pulse_update");
        expect_safe;
        drive_pair(1000, 2000);
        expect_pair(1000, 2000, -1000, 1000);
        pass_group("reset released while high: no false capture, then low arms input");

        @(negedge clk); rc_ch1 = 1'b1; rc_ch2 = 1'b1;
        #500000;
        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;
        #1000000;
        rc_ch1 = 1'b0; rc_ch2 = 1'b0;
        #200;
        require(!ch1_valid && !ch2_valid, "reset in middle of pulse discards remaining high");
        require(monitor_updates == 0, "partial post-reset pulse has no update");
        expect_safe;
        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        pass_group("mid-pulse reset discards incomplete pulses");

        reset_low;
        #10100000;
        require(ch1_timeout && ch2_timeout, "never-seen signal still times out");
        expect_safe;
        drive_pair(1500, 1500);
        expect_pair(1500, 1500, 0, 0);
        pass_group("no valid pulse since reset: startup timeout and recovery");

        // Synchronous samples quantize asynchronous edges at 20 ns resolution.
        // Do not demand sub-cycle classification at the 900 / 2100 us bounds.
        drive_pair_ns(1000007, 1999993, 1, 19);
        require(ch1_valid && ch2_valid, "non-integer-clock widths inside valid range");
        require(ch1_width_us == 1000, "1000.007 us reports floor 1000 us");
        require(ch2_width_us >= 1999 && ch2_width_us <= 2000,
            "1999.993 us quantizes within one microsecond");
        require($signed(steering) == -1000, "lower endpoint remains saturated");
        require($signed(throttle) >= 997 && $signed(throttle) <= 1000,
            "upper endpoint quantization produces bounded command");

        drive_pair_ns(899999, 1500000, 7, 13);
        require(ch2_valid, "CH2 legal while CH1 near lower sample boundary");
        if (ch1_valid) begin
            require(ch1_width_us == 900, "near lower bound accepted only at sampled 900 us");
            require($signed(steering) == -1000, "accepted near-minimum pulse saturated");
        end else begin
            expect_safe;
        end
        drive_pair_ns(2100001, 1500000, 7, 13);
        require(ch2_valid, "CH2 legal while CH1 near upper sample boundary");
        if (ch1_valid) begin
            require(ch1_width_us == 2100, "near upper bound accepted only at sampled 2100 us");
            require($signed(steering) == 1000, "accepted near-maximum pulse saturated");
        end else begin
            expect_safe;
        end
        drive_pair_ns(899980, 1500000, 7, 13);
        require(!ch1_valid && ch2_valid, "one whole cycle below minimum is invalid");
        expect_safe;
        drive_pair_ns(2100020, 1500000, 7, 13);
        require(!ch1_valid && ch2_valid, "one whole cycle above maximum is invalid");
        expect_safe;
        drive_pair_ns(900020, 2099980, 7, 13);
        expect_pair(900, 2099, -1000, 1000);
        pass_group("20 ns sampling resolution, off-clock phases and capture boundary cycles");

        // Default/production 100 ms timeout: valid just before the deadline,
        // invalid and neutral shortly after it, then recovers on the next pulse.
        @(negedge clk); default_rst_n = 1'b1;
        #200;
        require(!default_valid && !default_timeout, "default capture startup flags");
        for (frame_index = 0; frame_index < 3; frame_index = frame_index + 1) begin
            @(negedge clk); #7; default_pwm = 1'b1;
            #1500000; default_pwm = 1'b0;
            #1000;
            require(default_valid && !default_timeout, "default capture sees legal 50 Hz frame");
            require(default_width_us == 1500, "default capture width");
            require(default_updates == frame_index + 1, "one update per default 50 Hz frame");
            if (frame_index < 2)
                #18499800; // 1.5 ms high + 18.5 ms low ~= 20 ms frame.
        end
        #99900000;
        require(default_valid && !default_timeout, "default signal live at 99.9 ms");
        #200000;
        require(!default_valid && default_timeout, "default signal expires after 100 ms");
        @(negedge clk); #13; default_pwm = 1'b1;
        #1000000; default_pwm = 1'b0;
        #200;
        require(default_valid && !default_timeout, "default timeout recovery");
        require(default_width_us == 1000 && default_updates == 4,
            "default recovered width and update count");
        pass_group("three 50 Hz frames, production/default 100 ms timeout and recovery");

        $display("TOTAL: %0d groups PASS; %0d assertions PASS; 0 FAIL", groups, checks);
        $display("PASS: public command safety and one-clock pulse_update were continuously monitored.");
        $finish;
    end
endmodule


