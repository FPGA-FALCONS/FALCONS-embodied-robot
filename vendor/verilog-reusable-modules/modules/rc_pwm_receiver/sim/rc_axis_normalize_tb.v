`timescale 1ns/1ps
module rc_axis_normalize_tb;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg start = 0;
    reg cancel = 0;
    reg [15:0] width_us = 0;
    wire signed [11:0] command_default;
    wire signed [11:0] command_wide_reverse;
    wire busy_default, done_default, busy_wide, done_wide;
    integer width_index, cycle_index, checks = 0;
    rc_axis_normalize normal (
        .clk(clk), .rst_n(rst_n), .start(start), .cancel(cancel),
        .width_us(width_us), .command(command_default),
        .busy(busy_default), .done(done_default)
    );
    rc_axis_normalize #(
        .MIN_US(1), .CENTER_US(12345), .MAX_US(65535),
        .DEADZONE_US(17), .REVERSE(1)
    ) wide (
        .clk(clk), .rst_n(rst_n), .start(start), .cancel(cancel),
        .width_us(width_us), .command(command_wide_reverse),
        .busy(busy_wide), .done(done_wide)
    );
    function integer reference;
        input integer width_value, low, center, high, deadzone, reverse_axis;
        integer value;
        begin
            if (width_value <= low) value = -1000;
            else if (width_value >= high) value = 1000;
            else if (width_value < center-deadzone)
                value = -((center-deadzone-width_value)*1000 /
                          (center-deadzone-low));
            else if (width_value > center+deadzone)
                value = (width_value-center-deadzone)*1000 /
                        (high-center-deadzone);
            else value = 0;
            reference = reverse_axis ? -value : value;
        end
    endfunction
    task require;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1)
                $fatal(1,"FAIL width=%0d cycle=%0d: %0s",width_us,cycle_index,message);
        end
    endtask
    task launch;
        input [15:0] width_value;
        begin
            @(negedge clk); width_us = width_value; start = 1;
            @(posedge clk); #1;
            require(busy_default && busy_wide && !done_default && !done_wide,
                    "request accepted without early done");
            @(negedge clk); start = 0;
        end
    endtask
    task check_completion;
        begin
            for (cycle_index=1; cycle_index<=27; cycle_index=cycle_index+1) begin
                @(posedge clk); #1;
                if (cycle_index < 27)
                    require(busy_default && busy_wide && !done_default && !done_wide,
                            "fixed latency: no early done");
                else begin
                    require(!busy_default && !busy_wide && done_default && done_wide,
                            "exactly 27 clocks to done");
                    require($signed(command_default) == reference(width_us,1000,1500,2000,25,0),
                            "default calibration reference");
                    require($signed(command_wide_reverse) == reference(width_us,1,12345,65535,17,1),
                            "wide asymmetric reverse calibration reference");
                end
            end
            @(posedge clk); #1;
            require(!done_default && !done_wide,"done is one cycle");
        end
    endtask
    initial begin
        #100; rst_n = 1;
        // Exhaustive entire 16-bit width space includes all deadzone borders,
        // every fractional quotient, saturation, and the largest numerator.
        for (width_index=0; width_index<=65535; width_index=width_index+1) begin
            launch(width_index[15:0]);
            check_completion;
        end
        launch(1200);
        repeat (10) @(negedge clk);
        launch(1800);
        check_completion;
        launch(1100);
        repeat (26) @(negedge clk);
        cancel = 1;
        @(posedge clk); #1;
        require(!busy_default && !done_default && command_default == 0,
                "cancel wins at final divide step");
        @(negedge clk); cancel = 0;
        repeat (35) begin
            @(posedge clk); #1;
            require(!done_default && command_default == 0,"canceled result never returns");
        end
        launch(1474); check_completion;
        @(negedge clk); rst_n = 0; #1;
        require(command_default == 0 && !done_default && !busy_default,
                "asynchronous reset clears committed normalizer result");
        $display("PASS rc_axis_normalize: 65536 exhaustive widths x 2 calibrations, restart/cancel/reset; %0d checks",checks);
        $finish;
    end
    initial begin #100000000; $fatal(1,"watchdog"); end
endmodule
