`timescale 1ns/1ps
// Current checkout's standalone ai_vision_link is compiled by verify_handoff.py.
// True UART bits, 115200 8N1, 50 MHz. No physical UART port is used.
module vision_serial_bridge_tb;
    localparam integer BIT_NS=8680;
    localparam integer MAX_BYTES=262144;
    reg clk=0, rst_n=0, uart_rx=1;
    always #10 clk=~clk;
    reg [7:0] mem[0:MAX_BYTES-1];
    reg [4095:0] hex_path, expect_path;
    wire [15:0] seq;
    wire [7:0] flags, cls, status, conf;
    wire raw_target_valid, frame_valid, crc_error, frame_error, policy_error, replay_error, link_alive;
    // Keep the consumer's explicit live-link gate as an interface invariant.
    wire effective_target_valid=raw_target_valid && link_alive;
    ai_vision_link #(.TIMEOUT_CYCLES(500000), .MIN_CONF_Q8(77)) dut(
        .i_clock(clk), .i_reset_n(rst_n), .i_uart_rx(uart_rx),
        .o_seq(seq), .o_flags(flags), .o_class(cls), .o_status(status), .o_confidence(conf),
        .o_x1(), .o_y1(), .o_x2(), .o_y2(), .o_target_valid(raw_target_valid),
        .o_frame_valid(frame_valid), .o_crc_error(crc_error), .o_frame_error(frame_error),
        .o_policy_violation(policy_error), .o_replay_error(replay_error), .o_heartbeat(), .o_link_alive(link_alive));
    integer frames=0, crc_errors=0, frame_errors=0, policy_errors=0, replay_errors=0;
    always @(posedge clk) begin
        #1;
        if (frame_valid) frames=frames+1;
        if (crc_error) crc_errors=crc_errors+1;
        if (frame_error) frame_errors=frame_errors+1;
        if (policy_error) policy_errors=policy_errors+1;
        if (replay_error) replay_errors=replay_errors+1;
    end
    integer nframes, fd, i, k, n, v_expected, class_expected, status_expected, seq_expected, conf_expected;
    integer checks=0, failures=0, targets=0, last_valid_index=-1, previous_frames;
    task check;
        input cond;
        input [8*100-1:0] text;
        begin
            checks=checks+1;
            if (!cond) begin failures=failures+1; $display("FAIL %0s",text); end
        end
    endtask
    task uart_byte;
        input [7:0] value;
        integer b;
        begin
            uart_rx=0; #BIT_NS;
            for (b=0;b<8;b=b+1) begin uart_rx=value[b]; #BIT_NS; end
            uart_rx=1; #BIT_NS;
        end
    endtask
    task packet;
        input integer which;
        begin
            for(k=0;k<19;k=k+1) uart_byte(mem[which*19+k]);
            #4000;
        end
    endtask
    initial begin
        #2000000000;
        $fatal(1,"serial bridge simulation exceeded bounded time");
    end
    initial begin
        if (!$value$plusargs("HEX=%s",hex_path) || !$value$plusargs("EXPECT=%s",expect_path)
            || !$value$plusargs("FRAMES=%d",nframes)) $fatal(1,"missing replay arguments");
        if (nframes<=0 || nframes*19>MAX_BYTES) $fatal(1,"frame count outside replay memory bound");
        $readmemh(hex_path,mem,0,nframes*19-1);
        fd=$fopen(expect_path,"r");
        if (!fd) $fatal(1,"cannot open expectations");
        repeat(10) @(negedge clk);
        rst_n=1;
        #20000;
        check(!effective_target_valid && !link_alive,"reset has no live target");
        for(i=0;i<nframes;i=i+1) begin
            n=$fscanf(fd,"%d %d %d %d %d\n",seq_expected,v_expected,class_expected,status_expected,conf_expected);
            if(n!=5) $fatal(1,"missing expectation record");
            packet(i);
            check(frames==i+1,"exactly one valid CRC frame received per normalized observation");
            check(seq==seq_expected,"wire sequence retained");
            check(raw_target_valid==(v_expected!=0),"target validity follows safe bridge");
            check(cls==class_expected && status==status_expected && conf==conf_expected,"class status confidence match encoded observation");
            check(link_alive && effective_target_valid==(v_expected!=0),"effective target requires live link");
            check(crc_errors==0 && frame_errors==0 && policy_errors==0,"safe bridge causes no parser errors");
            if(v_expected!=0) begin targets=targets+1; last_valid_index=i; end
        end
        $fclose(fd);
        if(last_valid_index<0) $fatal(1,"no valid model observation reached candidate parser");
        // Old valid observations must not re-arm the receiver after EOF revoke.
        previous_frames=frames;
        packet(last_valid_index);
        check(frames==previous_frames && !raw_target_valid && replay_errors==1,"old wire sequence is rejected and target stays revoked");
        previous_frames=frames;
        for(k=0;k<18;k=k+1) uart_byte(mem[last_valid_index*19+k]);
        uart_byte(mem[last_valid_index*19+18]^8'h01);
        #4000;
        check(frames==previous_frames && crc_errors==1,"CRC-corrupted copy cannot update receiver state");
        #11000000;
        check(!link_alive && !raw_target_valid && !effective_target_valid,"link watchdog clears target validity");
        previous_frames=frames;
        packet(last_valid_index);
        check(frames==previous_frames+1 && link_alive && raw_target_valid,"fresh session after timeout can accept a valid observation");
        #11000000;
        check(!link_alive && !raw_target_valid,"active valid target is cleared when producer goes silent");
        $display("RAW_TARGET_AFTER_TIMEOUT=%0d EFFECTIVE_TARGET_AFTER_TIMEOUT=%0d",raw_target_valid,effective_target_valid);
        $display("SERIAL_MODEL_RECORDS=%0d SERIAL_TARGETS=%0d CHECKS=%0d FAILURES=%0d",nframes,targets,checks,failures);
        if(failures) $fatal(1,"serial bridge integration failed");
        $display("PASS safe bridge to UART parser, observation only");
        $finish;
    end
endmodule
