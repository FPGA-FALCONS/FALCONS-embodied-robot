//////////////////////////////////////////////////////////////////////////////////
// debug_rc_tb —— 最小调试台：单个 rc_pwm_capture 到底能不能测到脉宽
//   （rc_input_tb 出现"所有通道都收不到脉冲"时，用它把问题范围缩到"模块用法"
//     还是"多路例化的接线"，不要靠猜）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module debug_rc_tb;

    localparam integer CLK_HZ = 1_000_000;      // 1 MHz → 1 µs/拍

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         rc = 1'b0;

    wire [15:0] w;
    wire        v, to, upd;

    integer upd_cnt = 0;

    rc_pwm_capture #(
        .CLK_HZ       (CLK_HZ),
        .MIN_PULSE_US (900),
        .MAX_PULSE_US (2100),
        .TIMEOUT_US   (5000)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .rc_pwm         (rc),
        .pulse_width_us (w),
        .pulse_valid    (v),
        .timed_out      (to),
        .pulse_update   (upd)
    );

    always #500 clk = ~clk;                     // 1 MHz

    always @(posedge clk) if (upd) upd_cnt = upd_cnt + 1;

    initial begin
        $display("==== debug rc_pwm_capture ====");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
        $display("  DBG init:        w=%0d v=%b to=%b", w, v, to);

        // 发 1500 µs 脉冲
        rc = 1'b1; #1_500_000; rc = 1'b0;
        repeat (100) @(posedge clk);
        $display("  DBG after1500us: w=%0d v=%b to=%b upd_cnt=%0d", w, v, to, upd_cnt);

        // 发 2000 µs 脉冲
        #100_000;
        rc = 1'b1; #2_000_000; rc = 1'b0;
        repeat (100) @(posedge clk);
        $display("  DBG after2000us: w=%0d v=%b to=%b upd_cnt=%0d", w, v, to, upd_cnt);

        // 发 300 µs 非法脉冲（应被过滤）
        #100_000;
        rc = 1'b1; #300_000; rc = 1'b0;
        repeat (100) @(posedge clk);
        $display("  DBG after300us:  w=%0d v=%b to=%b upd_cnt=%0d (illegal)", w, v, to, upd_cnt);

        // 停发超过 TIMEOUT_US
        #6_000_000;
        $display("  DBG after6ms_idle: w=%0d v=%b to=%b", w, v, to);

        $finish;
    end

endmodule
