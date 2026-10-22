`timescale 1ns/1ps
module display_mm_to_cm_tb;
    reg clk = 0, reset_n = 0;
    always #10 clk = ~clk;
    reg signed [31:0] mm = 0;
    wire signed [31:0] cm;
    reg signed [31:0] expected [0:2];
    integer cycles = 0, errors = 0, n;
    reg [31:0] random_state = 32'h291a73d5;

    display_mm_to_cm dut (.clk(clk), .reset_n(reset_n), .mm(mm), .cm(cm));

    always @(posedge clk) begin
        cycles = cycles + 1;
        if (!reset_n) begin
            expected[0] = 0;
            expected[1] = 0;
            expected[2] = 0;
        end else begin
            expected[2] = expected[1];
            expected[1] = expected[0];
            expected[0] = mm / 32'sd10;
        end
        #1;
        if (cm !== expected[2]) begin
            errors = errors + 1;
            if (errors < 8) $display("FAIL: cycle=%0d cm=%0d expected=%0d", cycles, cm, expected[2]);
        end
    end

    task sample;
        input signed [31:0] value;
        begin
            @(negedge clk);
            mm = value;
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        reset_n = 1;
        // 连续输入覆盖符号、每个除十余数、量程边界与流水中途复位。
        for (n = -1000; n <= 1000; n = n + 1) sample(n);
        for (n = 0; n < 100; n = n + 1) begin
            sample(32'sh7fffffff - n);
            sample(32'sh80000000 + n);
        end
        for (n = 0; n < 20000; n = n + 1) begin
            random_state = random_state ^ (random_state << 13);
            random_state = random_state ^ (random_state >> 17);
            random_state = random_state ^ (random_state << 5);
            sample(random_state);
        end
        @(negedge clk); reset_n = 0;
        repeat (3) @(negedge clk);
        reset_n = 1;
        sample(-2147483648);
        sample(2147483647);
        sample(0);
        repeat (5) @(negedge clk);
        $display("CYCLES: %0d", cycles);
        if (errors == 0) $display("PASS: display_mm_to_cm_tb 1 checks");
        else $display("FAIL: display_mm_to_cm_tb %0d errors", errors);
        $finish;
    end
endmodule
