`timescale 1ns/1ps
// 实际 raw/cost 地图参与验证，包含上一圈清空与下一圈新命中的竞争。
module nav_scan_accumulator_tb;
    reg clk = 0, reset_n = 0;
    always #10 clk = ~clk;
    reg scan_begin = 0, in_valid = 0, in_value = 0, allow_map = 1;
    reg [15:0] in_x = 0, in_y = 0;
    wire scan_ready, in_ready, wr_valid, wr_ready, wr_value, map_ready;
    wire [15:0] wr_x, wr_y;
    wire map_idle;
    integer cycles = 0, checks = 0, errors = 0, start_cycle, empty_cycles, clear_cycles;
    always @(posedge clk) cycles <= cycles + 1;

    nav_scan_accumulator #(.GRID_W(16), .GRID_H(16)) dut (
        .clk(clk), .reset_n(reset_n), .scan_begin(scan_begin), .scan_ready(scan_ready),
        .in_valid(in_valid), .in_ready(in_ready), .in_value(in_value), .in_x(in_x), .in_y(in_y),
        .wr_valid(wr_valid), .wr_ready(wr_ready), .wr_value(wr_value), .wr_x(wr_x), .wr_y(wr_y)
    );
    assign wr_ready = map_ready && allow_map;
    nav_map #(.GRID_W(16), .GRID_H(16), .INFLATE(1)) map (
        .clk(clk), .reset_n(reset_n), .wr_valid(wr_valid && allow_map), .wr_ready(map_ready),
        .wr_val(wr_value), .wr_x(wr_x), .wr_y(wr_y),
        .rd_valid(1'b0), .rd_x(16'd0), .rd_y(16'd0), .rd_ready(),
        .rd_rsp_valid(), .rd_raw(), .rd_cost(), .idle(map_idle), .occupied_cells(), .map_version()
    );

    reg full_begin = 0, full_done = 0;
    wire full_ready;
    integer full_start, full_merge_cycles, full_init_cycles;
    nav_scan_accumulator full_map (
        .clk(clk), .reset_n(reset_n), .scan_begin(full_begin), .scan_ready(full_ready),
        .in_valid(1'b0), .in_ready(), .in_value(1'b0), .in_x(16'd0), .in_y(16'd0),
        .wr_valid(), .wr_ready(1'b1), .wr_value(), .wr_x(), .wr_y()
    );
    initial begin
        wait (reset_n);
        wait (full_ready);
        @(negedge clk);
        full_init_cycles = cycles - 4;
        full_begin = 1;
        @(negedge clk);
        full_begin = 0;
        full_start = cycles;
        wait (full_ready);
        @(negedge clk);
        full_merge_cycles = cycles - full_start;
        full_done = 1;
    end

    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL: %0s", message);
            end else $display("pass %0s", message);
        end
    endtask
    task observation;
        input value;
        input [15:0] x, y;
        begin
            @(negedge clk);
            in_valid = 1; in_value = value; in_x = x; in_y = y;
            @(posedge clk);
            while (!in_ready) @(posedge clk);
            @(negedge clk);
            in_valid = 0;
        end
    endtask
    task rotate;
        begin
            @(negedge clk);
            while (!scan_ready) @(negedge clk);
            scan_begin = 1;
            @(negedge clk);
            scan_begin = 0;
        end
    endtask
    task settled;
        begin
            @(negedge clk);
            while (!scan_ready || !map_idle) @(negedge clk);
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        reset_n = 1;
        settled;
        check(cycles >= 260 && cycles <= 261, "both memories initialize together in 256 clocks");
        start_cycle = cycles;
        rotate; settled;
        empty_cycles = cycles - start_cycle;
        check(empty_cycles < 780, "empty-circle merge has bounded linear scan cost");

        observation(1, 5, 5); wait (map_idle); @(negedge clk);
        check(map.raw[85], "new hit enters raw map before circle end");
        observation(0, 5, 5);
        observation(0, 6, 5);
        observation(1, 6, 5);
        observation(1, 12, 12);
        rotate; settled;
        check(map.raw[85] && map.raw[86], "hit wins in both hit/free arrival orders");

        observation(0, 5, 5);
        check(map.raw[85], "free ray cannot erase previous hit before circle end");
        start_cycle = cycles;
        rotate; settled;
        clear_cycles = cycles - start_cycle;
        check(!map.raw[85] && map.raw[86], "next circle clears only observed free cell");
        check(map.cost[85], "clearing keeps overlapping inflation from neighboring hit");
        check(map.raw[204], "unobserved historical obstacle is retained");
        observation(0, 6, 5);
        rotate; settled;
        check(!map.raw[86] && !map.cost[85], "last neighboring hit removal rebuilds inflation");

        observation(1, 8, 8);
        rotate; settled;
        observation(0, 8, 8);
        allow_map = 0;
        rotate;
        wait (dut.state == 4 && wr_x == 8 && wr_y == 8);
        @(negedge clk);
        check(!scan_ready && !wr_value, "previous circle clear waits under map backpressure");
        // 当前圈的命中抢占等待中的旧清空；释放后旧清空不能追着删掉新命中。
        in_valid = 1; in_value = 1; in_x = 8; in_y = 8;
        repeat (4) @(negedge clk);
        check(!in_ready && wr_value && wr_x == 8, "pending new hit has priority over old clear");
        allow_map = 1;
        @(negedge clk); in_valid = 0;
        settled;
        check(map.raw[136], "new-circle hit survives previous-circle pending clear");

        observation(0, 8, 8);
        rotate; settled;
        check(map.raw[136], "free after protected hit still cannot erase same-circle hit");
        observation(0, 8, 8);
        rotate;
        check(!scan_ready, "bank cannot be reused before merge finishes");
        observation(1, 15, 15);
        wait (map_idle); @(negedge clk);
        check(!scan_ready && map.raw[255], "new hit is accepted while previous circle merges");
        settled;
        check(!map.raw[136] && map.raw[255], "subsequent clear works after bank reuse");

        wait (full_done);
        check(full_init_cycles == 65536, "full 256x256 circle buffers initialize in 65536 clocks");
        check(full_merge_cycles == 196608, "full empty circle scan completes in 196608 clocks (3.93216 ms)");
        $display("METRICS: full_grid init=%0d merge=%0d", full_init_cycles, full_merge_cycles);
        $display("CHECKS: %0d", checks);
        $display("METRICS: scan_accumulator cycles=%0d empty_merge=%0d one_clear_merge=%0d errors=%0d", cycles, empty_cycles, clear_cycles, errors);
        if (!errors) $display("PASS: nav_scan_accumulator_tb");
        $finish;
    end
    initial begin
        #20000000;
        $display("FAIL: nav_scan_accumulator_tb timeout state=%0d", dut.state);
        $finish;
    end
endmodule
