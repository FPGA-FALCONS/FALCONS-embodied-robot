`timescale 1ns/1ps
module tb_voice_motion_bridge;
    localparam integer CLK_HZ=100_000, MS=100;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, rx_valid=0, enable=1, estop=0, manual=0;
    reg [7:0] rx_data=0;
    reg signed [31:0] yaw=0;
    reg yaw_valid=1, imu_sample=0, imu_healthy=1, nav_ready=0;
    wire valid, active, owned, error, patrol_start, patrol_stop;
    wire signed [31:0] center, diff;
    wire [7:0] seq, cmd, flags;
    integer errors=0, starts=0, stops=0, checks=0;
    always @(posedge clk) begin
        if(error) errors=errors+1;
        if(patrol_start) starts=starts+1;
        if(patrol_stop) stops=stops+1;
    end
    voice_motion_bridge #(.CLK_HZ(CLK_HZ), .MAX_CENTER(100),
        .WATCHDOG_MS(500), .TIMEOUT_MS(600)) dut (
        .clk(clk),.rst_n(rst_n),.i_rx_data(rx_data),.i_rx_valid(rx_valid),
        .i_enable(enable),.i_estop(estop),.i_manual(manual),
        .i_yaw_deg(yaw),.i_yaw_valid(yaw_valid),
        .i_imu_sample_valid(imu_sample),.i_imu_healthy(imu_healthy),
        .i_nav_release_ready(nav_ready),.o_auto_valid(valid),.o_auto_center(center),
        .o_auto_diff(diff),.o_active(active),.o_owned(owned),
        .o_last_seq(seq),.o_last_cmd(cmd),.o_last_flags(flags),
        .o_cmd_error(error),.o_patrol_start(patrol_start),.o_patrol_stop(patrol_stop)
    );
    function automatic [15:0] crcbyte(input [15:0] c0,input [7:0] b);
        reg[15:0] c; integer j;
        begin
            c=c0^{b,8'h00};
            for(j=0;j<8;j=j+1)c=c[15]?((c<<1)^16'h1021):(c<<1);
            crcbyte=c;
        end
    endfunction
    task automatic check(input bit ok,input string msg);
        begin checks=checks+1; if(!ok)$fatal(1,"%s",msg); $display("pass %s",msg); end
    endtask
    task automatic cycles(input integer n);
        begin repeat(n)@(negedge clk); end
    endtask
    task automatic ms(input integer n); begin cycles(n*MS); end endtask
    task automatic reset;
        begin
            rst_n=0; rx_valid=0; enable=1; estop=0; manual=0;
            yaw=0; yaw_valid=1; imu_sample=0; imu_healthy=1; nav_ready=0;
            cycles(4); rst_n=1; cycles(4);
        end
    endtask
    task automatic sample(input signed [31:0] angle);
        begin @(negedge clk); yaw=angle; imu_sample=1;
            @(negedge clk); imu_sample=0; cycles(3); end
    endtask
    task automatic frame(input[7:0] code,input signed[15:0] value,
        input[7:0] speed,input bit bad,input bit collision);
        reg[7:0] b[0:11];reg[15:0] crc;integer j;
        begin
            b[0]="M";b[1]="O";b[2]="T";b[3]="1";
            b[4]=8'h42;b[5]=code;b[6]=value[15:8];b[7]=value[7:0];
            b[8]=speed;b[9]=0;crc=16'hffff;
            for(j=0;j<10;j=j+1)crc=crcbyte(crc,b[j]);
            b[10]=crc[15:8];b[11]=crc[7:0]^{7'd0,bad};
            for(j=0;j<12;j=j+1)begin
                @(negedge clk);rx_data=b[j];rx_valid=1;
                @(negedge clk);rx_valid=0;
                if(j==11 && collision)begin
                    // Deterministically exercise the former NBA overwrite:
                    // new action and the old action's final ms tick together.
                    dut.ms_div=MS-1; dut.remaining_ms=1;
                end
                cycles(3);
            end
        end
    endtask
    task automatic hold_zero;
        begin check(owned && valid && !active && center==0 && diff==0,
                    "voice must retain zero ownership"); end
    endtask
    integer i, e;
    initial begin
        reset(); check(!owned && !valid,"reset must not seize navigation");
        manual=1; frame(0,0,0,0,0);
        check(owned && !valid && !active,"manual STOP must acquire persistent zero ownership");
        manual=0;cycles(3);hold_zero();
        reset();enable=0;frame(0,0,0,0,0);
        check(owned && !valid && !active,"RC-disabled STOP must acquire persistent zero ownership");
        enable=1;cycles(3);hold_zero();
        reset();estop=1;frame(7,0,0,0,0);
        check(owned && !valid && !active && !patrol_start,"unsafe PATROL_STOP acquires stop only");
        estop=0;cycles(3);hold_zero();
        reset();manual=1;frame(1,20,40,0,0);
        check(!owned && !active && !valid,"unsafe motion must not acquire or execute");
        manual=0;cycles(3);check(!owned && !active,"unsafe motion must not resume");
        frame(0,0,0,0,0);hold_zero();
        frame(8,0,0,0,0);hold_zero();
        frame(1,20,40,0,0);check(active && center==100,"bounded forward speed");
        ms(22);hold_zero();
        frame(2,20,255,0,0);check(active && center == -100,"bounded reverse speed");
        ms(22);hold_zero();
        frame(1,100,40,0,0);
        frame(2,100,40,0,1);
        check(active && center==-100 && dut.remaining_ms>=99,"new command lost to old tick");
        frame(0,0,0,0,0);hold_zero();
        frame(1,100,40,1,0);hold_zero();
        check(errors>0,"bad CRC must flag error");
        e=errors;frame(99,0,0,0,0);hold_zero();check(errors>e,"unknown opcode error");
        frame(3,90,40,0,0);hold_zero(); // no actual IMU sample yet
        sample(0);frame(3,0,40,0,0);hold_zero();
        frame(3,361,40,0,0);hold_zero();
        frame(4,-90,40,0,0);hold_zero();
        frame(5,-32768,40,0,0);hold_zero();
        frame(1,5001,40,0,0);hold_zero();
        frame(1,10,0,0,0);hold_zero();
        frame(3,90,40,0,0);check(active && diff>0,"left turn start");
        sample(-1);check(active && dut.accum==-1,"reverse 1 degree falsely completed left");
        sample(89);check(active,"left must wait for target");
        sample(90);hold_zero();
        sample(0);frame(4,90,40,0,0);
        sample(1);check(active && dut.accum==1,"reverse 1 degree falsely completed right");
        sample(-90);hold_zero();
        sample(179);frame(3,4,40,0,0);
        sample(-179);check(active && dut.accum==2,"positive wrap delta");
        sample(-177);hold_zero();
        sample(-179);frame(4,4,40,0,0);
        sample(179);check(active && dut.accum==-2,"negative wrap delta");
        sample(177);hold_zero();
        sample(0);frame(5,360,40,0,0);
        sample(-90);sample(-180);sample(90);check(active,"360 stopped early");sample(0);hold_zero();
        sample(0);frame(5,-360,40,0,0);
        sample(90);sample(180);sample(-90);sample(0);hold_zero();
        sample(0);frame(3,90,40,0,0);ms(201);hold_zero();
        frame(3,90,40,0,0);hold_zero(); // latched valid cannot freshen sensor
        sample(0);frame(3,90,40,0,0);imu_healthy=0;cycles(2);hold_zero();
        imu_healthy=1;cycles(2);frame(3,90,40,0,0);hold_zero();
        sample(0);frame(3,90,40,0,0);yaw_valid=0;cycles(2);hold_zero();
        yaw_valid=1;sample(0);frame(3,90,40,0,0);yaw=181;cycles(2);hold_zero();
        yaw=0;sample(0);frame(3,90,40,0,0);
        for(i=0;i<7;i=i+1)begin ms(90);sample(0);frame(8,0,0,0,0);end
        hold_zero(); // turn timeout despite fresh samples and heartbeats
        frame(1,5000,40,0,0);ms(501);hold_zero();
        frame(1,5000,40,0,0);
        for(i=0;i<6;i=i+1)begin ms(90);frame(1,5000,40,1,0);end
        hold_zero(); // bad CRC flood does not keep motion alive
        frame(1,5000,40,0,0);
        for(i=0;i<6;i=i+1)begin ms(90);frame(99,0,0,0,0);end
        hold_zero(); // CRC-valid unknown frames cannot keep motion alive
        frame(1,5000,40,0,0);
        for(i=0;i<6;i=i+1)begin ms(90);frame(3,0,40,0,0);end
        hold_zero(); // semantic rejects do not refresh watchdog
        frame(1,5000,40,0,0);ms(300);frame(8,0,0,0,0);ms(300);
        check(active,"valid heartbeat must keep live action");frame(0,0,0,0,0);hold_zero();
        for(i=0;i<3;i=i+1)begin
            frame(1,100,40,0,0);
            if(i==0)manual=1;else if(i==1)estop=1;else enable=0;
            #1;check(!valid && center==0 && diff==0,"safety must mask immediately");
            cycles(3);manual=0;estop=0;enable=1;cycles(3);hold_zero();
        end
        frame(6,0,0,0,0);hold_zero();check(starts==0,"unready patrol must not pulse/release");
        nav_ready=1;frame(8,0,0,0,0);hold_zero();
        frame(6,0,0,0,0);check(!owned && !valid && starts==1,"explicit ready patrol releases");
        nav_ready=0;frame(6,0,0,0,0);hold_zero();
        frame(7,0,0,0,0);hold_zero();check(stops==1,"patrol stop must pulse and own stop");
        frame(8,0,0,0,0);hold_zero();
        // Truncated header/body must recover without requiring a reset or
        // sacrificing the next good MOT1 frame.
        @(negedge clk); rx_data="M";rx_valid=1;
        @(negedge clk);rx_data="O";
        @(negedge clk);rx_data="T";
        @(negedge clk);rx_data="1";
        @(negedge clk);rx_data=8'h42;
        @(negedge clk);rx_valid=0;
        ms(21);check(dut.magic_idx==0,"truncated frame timeout resets parser");
        frame(1,20,40,0,0);check(active && center>0,"first frame after timeout accepted");
        frame(0,0,0,0,0);hold_zero();
        $display("CHECKS: %0d",checks);
        $display("PASS: tb_voice_motion_bridge %0d checks: ownership/safety/IMU freshness/yaw wraps/reverse jitter/semantic limits/watchdog/priority/patrol",checks);
        $finish;
    end
    initial begin #20_000_000;$fatal(1,"global timeout");end
endmodule
