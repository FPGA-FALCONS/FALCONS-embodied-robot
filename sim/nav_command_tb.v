`timescale 1ns/1ps
// 验证显示轴向始终跟随控制器当前任务，不能提前跟随后台描述符换向。
module nav_command_tb;
    reg clk = 0, reset_n = 0, enable = 0, ctrl_done = 0;
    always #10 clk = ~clk;
    reg [2:0] segment_dir = 0;
    reg signed [31:0] segment_x_mm = 500, segment_y_mm = 25;
    wire cmd_valid, emergency_stop;
    wire [1:0] target_line_dir;
    wire signed [31:0] target_x_mm, target_y_mm;
    integer cycles = 0, checks = 0, errors = 0, direction;
    always @(posedge clk) cycles <= cycles + 1;

    nav_command dut (
        .clk(clk), .reset_n(reset_n), .enable(enable),
        .sensors_valid(1'b1), .route_valid(1'b1),
        .segment_valid(1'b1), .segment_last(1'b0), .segment_final(1'b0),
        .segment_dir(segment_dir), .segment_x_mm(segment_x_mm),
        .segment_y_mm(segment_y_mm),
        .forward_clear(1'b1), .ctrl_linear_active(1'b1), .ctrl_done(ctrl_done),
        .cmd_valid(cmd_valid), .target_x_mm(target_x_mm), .target_y_mm(target_y_mm),
        .target_line_dir(target_line_dir), .emergency_stop(emergency_stop),
        .advance(), .arrived(), .at_target(), .stopped()
    );

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

    initial begin
        for (direction = 0; direction < 8; direction = direction + 1) begin
            @(negedge clk);
            reset_n = 0;
            enable = 0;
            ctrl_done = 0;
            segment_dir = direction;
            segment_x_mm = 500;
            segment_y_mm = 25;
            repeat (3) @(negedge clk);
            reset_n = 1;
            enable = 1;
            wait (emergency_stop);
            repeat (3) @(negedge clk);
            ctrl_done = 1;
            wait (!emergency_stop);
            @(negedge clk);
            ctrl_done = 0;
            wait (cmd_valid);
            @(negedge clk);
            check(target_line_dir == (direction % 4) &&
                  target_x_mm == 500 && target_y_mm == 25,
                  "accepted task axis matches coordinates for each direction");

            case (direction)
                0,1,7: segment_x_mm = segment_x_mm+300;
                3,4,5: segment_x_mm = segment_x_mm-300;
            endcase
            case (direction)
                1,2,3: segment_y_mm = segment_y_mm+300;
                5,6,7: segment_y_mm = segment_y_mm-300;
            endcase
            repeat (4) @(negedge clk);
            check(cmd_valid && target_line_dir == (direction % 4) &&
                  target_x_mm == segment_x_mm && target_y_mm == segment_y_mm,
                  "same-direction target update preserves active axis");

            segment_dir = (direction + 1) % 8;
            @(negedge clk);
            check(cmd_valid && target_line_dir == (direction % 4),
                  "incoming direction does not change active command axis early");
            repeat (3) @(negedge clk);
            check(emergency_stop && !cmd_valid && target_line_dir == (direction % 4),
                  "incompatible direction still stops before replacing command");
            ctrl_done = 1;
            wait (!emergency_stop);
            @(negedge clk);
            ctrl_done = 0;
            wait (cmd_valid);
            @(negedge clk);
            check(target_line_dir == ((direction + 1) % 4),
                  "new task axis changes only after stop handshake");
        end
        $display("CHECKS: %0d", checks);
        $display("METRICS: nav_command_tb cycles=%0d errors=%0d", cycles, errors);
        if (errors == 0) $display("PASS: nav_command_tb");
        $finish;
    end
    initial begin
        #100000;
        $display("FAIL: nav_command_tb timeout");
        $finish;
    end
endmodule
