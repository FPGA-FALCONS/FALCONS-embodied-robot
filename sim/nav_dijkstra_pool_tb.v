`timescale 1ns/1ps
// 小容量共享池：满池失败、释放后复用、重复启动和搜索中复位。
module nav_dijkstra_pool_tb;
    reg clk = 0, reset_n = 0, request = 0, blocked = 0;
    always #10 clk = ~clk;
    reg [15:0] sx = 0, sy = 0, gx = 4, gy = 4;
    wire ready, busy, success, no_path, rd, valid;
    wire [15:0] x, y, length;
    reg response = 0, raw = 0;
    integer cycles = 0, checks = 0, errors = 0, points = 0;
    nav_dijkstra #(.GRID_W(8), .GRID_H(8), .QMAX(8)) dut (
        .clk(clk), .reset_n(reset_n), .req_valid(request), .req_ready(ready),
        .allow_start_escape(1'b0), .start_x(sx), .start_y(sy), .goal_x(gx), .goal_y(gy),
        .mrd_valid(rd), .mrd_ready(1'b1), .mrd_x(), .mrd_y(),
        .mrd_rsp_valid(response), .mrd_raw(raw), .mrd_cost(1'b0),
        .path_valid(valid), .path_ready(1'b1), .path_x(x), .path_y(y),
        .path_dir(), .path_last(), .busy(busy), .success(success),
        .no_path(no_path), .path_len(length), .start_relaxed()
    );
    always @(posedge clk) begin
        cycles <= cycles + 1;
        response <= reset_n && rd;
        raw <= blocked;
        if (valid) points = points + 1;
        if (dut.free_count > 8) begin
            $display("FAIL: shared pool free count overflow");
            errors = errors + 1;
        end
    end
    task check;
        input condition;
        input [511:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                $display("FAIL: %0s",message);
                errors = errors + 1;
            end else $display("pass %0s",message);
        end
    endtask
    task start_round;
        begin
            wait (ready);
            @(negedge clk);
            points = 0;
            request = 1;
            @(negedge clk);
            request = 0;
        end
    endtask
    initial begin
        repeat (4) @(negedge clk);
        reset_n = 1;
        start_round;
        wait (!busy);
        check(no_path && !success && points == 0,"full pool fails without publishing a partial path");
        sx = 4; sy = 4;
        start_round;
        wait (!busy);
        check(success && !no_path && points == 1 && length == 0,"same-cell request reinitializes full pool");
        sx = 5;
        start_round;
        wait (!busy);
        check(success && !no_path && points == 2 && length == 1,"settled adjacent goal after pool reuse");
        blocked = 1;
        start_round;
        wait (!busy);
        check(no_path && !success && points == 0,"blocked start clears prior success");
        blocked = 0; sx = 0; sy = 0;
        start_round;
        wait (dut.state == 12);
        @(negedge clk); reset_n = 0;
        repeat (3) @(negedge clk);
        check(!busy && !valid,"reset cancels search and pending output");
        reset_n = 1; sx = 4; sy = 4;
        start_round;
        wait (!busy);
        check(success && points == 1,"fresh round after reset rebuilds all pool links");
        $display("CHECKS: %0d",checks);
        $display("METRICS: nav_dijkstra_pool_tb cycles=%0d errors=%0d",cycles,errors);
        if (errors == 0) $display("PASS: nav_dijkstra_pool_tb");
        $finish;
    end
    initial begin
        #2000000;
        $display("FAIL: pool timeout");
        $finish;
    end
endmodule
