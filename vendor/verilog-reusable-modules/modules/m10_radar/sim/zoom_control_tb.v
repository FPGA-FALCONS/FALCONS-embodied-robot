`timescale 1ns/1ps
module zoom_control_tb;
    reg clk = 0;
    always #5 clk = ~clk;
    reg reset_n = 0, zoom_in_n = 1, zoom_out_n = 1, frame_start = 0;
    wire [1:0] zoom_level;
    wire plot_clear;
    integer clears = 0, checks = 0;
    reg frame_start_d = 0;
    zoom_control #(.DEBOUNCE_CLKS(4)) dut (
        .clk(clk), .reset_n(reset_n), .zoom_in_n(zoom_in_n),
        .zoom_out_n(zoom_out_n), .frame_start(frame_start),
        .zoom_level(zoom_level), .plot_clear(plot_clear)
    );
    always @(posedge clk) begin
        frame_start_d <= frame_start;
        if (reset_n && plot_clear) begin
            if (!frame_start_d) $fatal(1,"clear occurred outside frame boundary");
            clears = clears + 1;
        end
    end
    task apply_frame;
        input [1:0] expected_level;
        input integer expected_clear;
        integer previous_clears;
        begin
            previous_clears = clears;
            @(negedge clk); frame_start = 1;
            @(negedge clk);
            if (zoom_level !== expected_level || plot_clear !== (expected_clear != 0))
                $fatal(1,"wrong scale/clear at frame boundary");
            frame_start = 0;
            @(negedge clk);
            if (plot_clear || clears != previous_clears + expected_clear)
                $fatal(1,"plot clear must last exactly one clock");
            checks = checks + 1;
        end
    endtask
    task press;
        input integer select_key;
        begin
            @(negedge clk);
            if (select_key == 0) zoom_in_n = 0;
            else zoom_out_n = 0;
            repeat(12) @(negedge clk);
        end
    endtask
    task release_keys;
        begin
            zoom_in_n = 1; zoom_out_n = 1;
            repeat(12) @(negedge clk);
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);
        reset_n = 1;
        apply_frame(2,0);
        // Bounces shorter than the stable threshold cannot trigger zoom.
        zoom_in_n = 0; @(negedge clk); zoom_in_n = 1;
        repeat(2) @(negedge clk);
        zoom_in_n = 0; repeat(2) @(negedge clk); zoom_in_n = 1;
        repeat(12) @(negedge clk);
        apply_frame(2,0);
        press(0);
        if (zoom_level != 2 || plot_clear) $fatal(1,"scale changed outside a frame");
        apply_frame(1,1);
        repeat(40) @(negedge clk); apply_frame(1,0); // No long-press repeat.
        release_keys(); press(0); apply_frame(0,1); release_keys();
        press(0); apply_frame(0,0); release_keys(); // Minimum clamp.
        press(1); apply_frame(1,1); release_keys();
        press(1); apply_frame(2,1); release_keys();
        press(1); apply_frame(3,1); release_keys();
        press(1); apply_frame(3,0); release_keys(); // Maximum clamp.
        zoom_in_n = 0; zoom_out_n = 0;
        repeat(12) @(negedge clk); apply_frame(3,0); release_keys();
        press(0); apply_frame(2,1); release_keys();
        press(1); // Pending change is discarded by reset.
        reset_n = 0; repeat(4) @(negedge clk); release_keys(); reset_n = 1;
        apply_frame(2,0);
        $display("PASS zoom_control_tb: %0d checks, debounce, frame apply, limits, hold, simultaneous keys and reset",checks);
        $finish;
    end
    initial begin
        #100000;
        $fatal(1,"zoom control test timeout");
    end
endmodule
