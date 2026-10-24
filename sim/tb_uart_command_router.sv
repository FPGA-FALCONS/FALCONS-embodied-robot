`timescale 1ns/1ps
module tb_uart_command_router;
    reg clk=0, rst_n=0;
    always #5 clk=~clk;
    reg [7:0] data=0;
    reg valid=0;
    wire [7:0] ascii_data, motion_data;
    wire ascii_valid, motion_valid, abort_ascii;
    wire kp_valid, ki_valid, kd_valid, count_clear, stall_clear, report_req;
    wire signed [15:0] kp_value;
    wire tgt_valid, mode_valid, duty_valid, cmd_error;
    uart_command_router #(.CLK_FREQ_HZ(1000),.BYTE_TIMEOUT_MS(20)) dut (
        .i_clock(clk),.i_reset_n(rst_n),.i_rx_data(data),.i_rx_valid(valid),
        .o_ascii_data(ascii_data),.o_ascii_valid(ascii_valid),.o_ascii_abort(abort_ascii),
        .o_motion_data(motion_data),.o_motion_valid(motion_valid));
    uart_cmd_rx parser (
        .i_clock(clk),.i_reset_n(rst_n),.i_rx_data(ascii_data),
        .i_rx_valid(ascii_valid),.i_abort(abort_ascii),
        .o_kp_valid(kp_valid),.o_kp_value(kp_value),.o_ki_valid(ki_valid),
        .o_kd_valid(kd_valid),.o_count_clear(count_clear),.o_stall_clear(stall_clear),
        .o_report_req(report_req),.o_tgt_valid(tgt_valid),.o_mode_valid(mode_valid),
        .o_duty_valid(duty_valid),.o_cmd_error(cmd_error));
    reg [7:0] expected_ascii[0:2047], expected_motion[0:65535];
    integer na=0,nm=0,ra=0,rm=0,aborts=0,kps=0;
    integer allowed_count=0,allowed_stall=0,allowed_report=0;
    integer counts=0,stalls=0,reports=0;
    reg expect_binary_only=0;
    always @(posedge clk) begin
        if (rst_n) begin
            if (dut.count > 8) $fatal(1,"queue overflow");
            if (ascii_valid) begin
                if (ra >= na || ascii_data !== expected_ascii[ra])
                    $fatal(1,"ASCII mismatch index %0d got %02x",ra,ascii_data);
                ra=ra+1;
            end
            if (motion_valid) begin
                if (rm >= nm || motion_data !== expected_motion[rm])
                    $fatal(1,"motion mismatch index %0d got %02x",rm,motion_data);
                rm=rm+1;
            end
            if (abort_ascii) aborts=aborts+1;
            if (kp_valid) kps=kps+1;
            if (count_clear) counts=counts+1;
            if (stall_clear) stalls=stalls+1;
            if (report_req) reports=reports+1;
            if (expect_binary_only && (kp_valid || ki_valid || kd_valid ||
                count_clear || stall_clear || report_req || tgt_valid || mode_valid || duty_valid || cmd_error))
                $fatal(1,"binary triggered ASCII parser action/error");
        end
    end
    task send_byte(input [7:0] b);
        begin @(negedge clk); valid=1; data=b; end
    endtask
    task idle(input integer n);
        begin @(negedge clk); valid=0; repeat(n) @(negedge clk); end
    endtask
    task ascii(input string text);
        integer i;
        begin
            for(i=0;i<text.len();i=i+1) begin
                expected_ascii[na]=text[i];na=na+1;
                send_byte(text[i]);
            end
        end
    endtask
    task motion_byte(input [7:0] b);
        begin expected_motion[nm]=b;nm=nm+1;send_byte(b);end
    endtask
    reg [7:0] frame[0:11];
    task make_frame(input integer seqno,input integer command,input bit corrupt);
        integer i,j;
        reg [15:0] crc;
        begin
            frame[0]="M";frame[1]="O";frame[2]="T";frame[3]="1";
            frame[4]=seqno[7:0];frame[5]=command[7:0];
            frame[6]=0;frame[7]=0;frame[8]=0;frame[9]=0;
            crc=16'hffff;
            for(i=0;i<10;i=i+1) begin
                crc=crc ^ {frame[i],8'b0};
                for(j=0;j<8;j=j+1) crc=crc[15] ? (crc<<1)^16'h1021 : crc<<1;
            end
            frame[10]=crc[15:8];frame[11]=crc[7:0]^{7'b0,corrupt};
        end
    endtask
    task send_frame;
        integer i;
        begin for(i=0;i<12;i=i+1) motion_byte(frame[i]);end
    endtask
    task drained;
        begin
            idle(25);
            if (ra!=na || rm!=nm) $fatal(1,"undelivered stream %0d/%0d %0d/%0d",ra,na,rm,nm);
        end
    endtask
    integer seqno,command,i;
    integer before_kps,before_aborts,before_stalls,before_counts,before_reports;
    initial begin
        repeat(3) @(negedge clk);rst_n=1;
        ascii("P26\n");drained();
        if(kps!=1 || kp_value!=26) $fatal(1,"normal PID broken");
        // Every legal command and all seq values, with no idle between frames.
        expect_binary_only=1;
        for(command=0;command<=8;command=command+1)
            for(seqno=0;seqno<256;seqno=seqno+1) begin
                make_frame(seqno,command,0);
                if(command==0 && seqno==32 && {frame[10],frame[11]}!=16'hcc78)
                    $fatal(1,"STOP seq32 fixture CRC wrong");
                send_frame();
            end
        // Bad CRC, unknown command and ASCII-looking binary body stay isolated.
        make_frame(90,255,1);frame[6]="X";frame[7]="Z";frame[8]=10;frame[9]="P";send_frame();
        drained();expect_binary_only=0;
        if(aborts!=2305 || counts!=0 || stalls!=0 || reports!=0 || kps!=1)
            $fatal(1,"binary leaked into parser");
        // Preserve unmatched/restarted prefixes and prefix timeout as ASCII.
        ascii("MOP1\nMOTX\nMMO\n");drained();
        ascii("MO");drained();
        before_kps=kps;
        // A binary boundary must discard a half-entered P command.
        ascii("P12");make_frame(1,0,0);send_frame();ascii("3\n");drained();
        if(kps!=before_kps) $fatal(1,"partial PID crossed binary boundary");
        ascii("P37\n");drained();
        if(kps!=before_kps+1 || kp_value!=37) $fatal(1,"PID did not recover");
        // Truncated claimed frame times out; new frame and ASCII recover.
        make_frame(2,0,0);
        for(i=0;i<7;i=i+1) motion_byte(frame[i]);
        drained();make_frame(3,0,0);send_frame();ascii("P41\n");drained();
        if(kps!=before_kps+2 || kp_value!=41) $fatal(1,"truncation recovery failed");
        // Immediate commands remain functional after binary traffic.
        before_stalls=stalls;before_counts=counts;before_reports=reports;
        ascii("XZR");drained();
        if(stalls!=before_stalls+1 || counts!=before_counts+1 || reports!=before_reports+1)
            $fatal(1,"immediate commands broken");
        // A prior complete PID must commit even while the output queue is busy.
        before_kps=kps;
        make_frame(4,0,0);send_frame();ascii("P52\n");
        make_frame(5,0,0);send_frame();drained();
        if(kps!=before_kps+1 || kp_value!=52)
            $fatal(1,"abort overtook a preceding complete PID");
        $display("PASS: uart_command_router 2304 legal MOT1 frames plus fault/ASCII checks");
        $finish;
    end
    initial begin #2000000;$fatal(1,"test timeout");end
endmodule
