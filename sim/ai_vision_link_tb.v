`timescale 1ns/1ps
// ai_vision_link_tb —— AI 视觉帧解析/安全校验/链路超时
// 运行：python host/vision_bridge/verify_handoff.py
module ai_vision_link_tb;
    initial begin
        #100_000_000;
        $fatal(1, "AI vision link test exceeded bounded time");
    end
    localparam integer BIT_NS = 8680; // 115200 8N1 @ 50 MHz

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg uart_rx = 1'b1;
    always #10 clk = ~clk;

    wire [15:0] o_seq, o_x1, o_y1, o_x2, o_y2;
    wire [7:0]  o_flags, o_class, o_status, o_confidence;
    wire o_target_valid, o_frame_valid, o_crc_error, o_frame_error;
    wire o_policy_violation, o_replay_error, o_heartbeat, o_link_alive;

    ai_vision_link #(
        .TIMEOUT_CYCLES(500_000), // 10 ms @ 50 MHz，测试用
        .MIN_CONF_Q8(77)
    ) dut (
        .i_clock(clk), .i_reset_n(rst_n), .i_uart_rx(uart_rx),
        .o_seq(o_seq), .o_flags(o_flags), .o_class(o_class), .o_status(o_status),
        .o_confidence(o_confidence), .o_x1(o_x1), .o_y1(o_y1), .o_x2(o_x2), .o_y2(o_y2),
        .o_target_valid(o_target_valid), .o_frame_valid(o_frame_valid),
        .o_crc_error(o_crc_error), .o_frame_error(o_frame_error),
        .o_policy_violation(o_policy_violation), .o_replay_error(o_replay_error), .o_heartbeat(o_heartbeat),
        .o_link_alive(o_link_alive)
    );

    integer frame_count = 0, crc_count = 0, err_count = 0, policy_count = 0, replay_count = 0, hb_count = 0, fails = 0;
    always @(posedge clk) begin
        if (o_frame_valid) frame_count <= frame_count + 1;
        if (o_crc_error) crc_count <= crc_count + 1;
        if (o_frame_error) err_count <= err_count + 1;
        if (o_policy_violation) policy_count <= policy_count + 1;
        if (o_replay_error) replay_count <= replay_count + 1;
        if (o_heartbeat) hb_count <= hb_count + 1;
    end

    task uart_byte(input [7:0] b);
        integer i;
        begin
            uart_rx = 1'b0; #BIT_NS;
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = b[i]; #BIT_NS;
            end
            uart_rx = 1'b1; #BIT_NS;
        end
    endtask

    task send_valid; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h01); uart_byte(8'h01); uart_byte(8'h03);
        uart_byte(8'h02); uart_byte(8'hF5); uart_byte(8'h00); uart_byte(8'h0A);
        uart_byte(8'h00); uart_byte(8'h14); uart_byte(8'h01); uart_byte(8'h2C);
        uart_byte(8'h01); uart_byte(8'h90); uart_byte(8'h77); uart_byte(8'hA7);
    end endtask

    task send_tracking; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h02); uart_byte(8'h01); uart_byte(8'h00);
        uart_byte(8'h00); uart_byte(8'hC8); uart_byte(8'h00); uart_byte(8'h05);
        uart_byte(8'h00); uart_byte(8'h06); uart_byte(8'h00); uart_byte(8'h64);
        uart_byte(8'h00); uart_byte(8'hC8); uart_byte(8'h54); uart_byte(8'h07);
    end endtask

    task send_bad_crc; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h01); uart_byte(8'h01); uart_byte(8'h03);
        uart_byte(8'h02); uart_byte(8'hF5); uart_byte(8'h00); uart_byte(8'h0A);
        uart_byte(8'h00); uart_byte(8'h14); uart_byte(8'h01); uart_byte(8'h2C);
        uart_byte(8'h01); uart_byte(8'h90); uart_byte(8'h77); uart_byte(8'h58);
    end endtask

    task send_uncertain; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h03); uart_byte(8'h02); uart_byte(8'hFF);
        uart_byte(8'h03); uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00);
        uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00);
        uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h7E); uart_byte(8'h66);
    end endtask

    task send_policy_violation; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h06); uart_byte(8'h01); uart_byte(8'h03);
        uart_byte(8'h03); uart_byte(8'hC8); uart_byte(8'h00); uart_byte(8'h01);
        uart_byte(8'h00); uart_byte(8'h02); uart_byte(8'h00); uart_byte(8'h03);
        uart_byte(8'h00); uart_byte(8'h04); uart_byte(8'hD6); uart_byte(8'h64);
    end endtask

    task send_bad_class; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h07); uart_byte(8'h01); uart_byte(8'h09);
        uart_byte(8'h02); uart_byte(8'hC8); uart_byte(8'h00); uart_byte(8'h01);
        uart_byte(8'h00); uart_byte(8'h02); uart_byte(8'h00); uart_byte(8'h03);
        uart_byte(8'h00); uart_byte(8'h04); uart_byte(8'hFE); uart_byte(8'h7F);
    end endtask

    task send_heartbeat; begin
        uart_byte(8'hAA); uart_byte(8'h55); uart_byte(8'h0E);
        uart_byte(8'h00); uart_byte(8'h08); uart_byte(8'h02); uart_byte(8'hFF);
        uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00);
        uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00);
        uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h0B); uart_byte(8'h56);
    end endtask

    task check(input condition, input [255:0] msg);
        begin
            if (!condition) begin
                fails = fails + 1;
                $display("FAIL: %0s", msg);
            end
        end
    endtask

    initial begin
        $display("======== AI vision link test ========");
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
        #20000; // let uart_byte_rx RECOVERY_EN see >1.5 idle bits

        send_valid; #20000;
        check(frame_count == 1, "valid frame count");
        check(o_target_valid === 1'b1, "valid target");
        check(o_seq === 16'd1, "seq");
        check(o_class === 8'd3, "class");
        check(o_status === 8'd2, "status");
        check(o_confidence === 8'd245, "confidence");
        check(o_x1 === 16'd10 && o_y1 === 16'd20 && o_x2 === 16'd300 && o_y2 === 16'd400, "bbox");
        check(o_link_alive === 1'b1, "link alive after valid frame");

        send_bad_crc; #20000;
        check(crc_count == 1, "CRC error counted");
        check(frame_count == 1, "CRC error must not update frame");
        check(o_target_valid === 1'b0, "CRC error revokes target");

        send_valid; #20000;
        check(replay_count == 1, "duplicate seq replay counted");
        check(o_target_valid === 1'b0, "replay must not re-arm target");
        check(frame_count == 1, "replay must not count as new frame");

        send_tracking; #20000;
        check(frame_count == 2, "tracking frame count");
        check(o_target_valid === 1'b1, "tracking target valid");
        check(o_class === 8'd0, "tracking class");

        send_uncertain; #20000;
        check(frame_count == 3, "UNCERTAIN frame count");
        check(o_target_valid === 1'b0, "UNCERTAIN target invalid");
        check(o_status === 8'd3, "UNCERTAIN status");

        send_policy_violation; #20000;
        check(policy_count == 1, "policy violation counted");
        check(o_target_valid === 1'b0, "policy violation target invalid");

        send_bad_class; #20000;
        check(err_count == 1, "bad class frame error");

        send_heartbeat; #20000;
        check(hb_count == 2, "heartbeat counted (UNCERTAIN + explicit heartbeat)");
        check(o_target_valid === 1'b0, "heartbeat not a target");

        #11_000_000;
        check(o_link_alive === 1'b0, "link timeout");
        if (fails == 0) $display("PASS: all AI vision link checks");
        else $fatal(1, "FAIL: %0d checks failed", fails);
        $finish;
    end
endmodule
