`timescale 1ns/1ps
// Production router -> bridge -> ownership mux, against continuously nonzero nav.
module voice_motion_integration_tb;
    reg clk=0; always #5000 clk=~clk;
    reg rst_n=0, enable=1, manual=0, estop=0, healthy=1, samples=1,local_release=0;
    reg [7:0] rx=0; reg rx_valid=0;
    reg signed [31:0] yaw=0;
    wire [7:0] ascii_data,motion_data;
    wire ascii_valid,ascii_abort,motion_valid;
    wire owned,valid,active; wire signed [31:0] center,diff;
    wire selected_valid; wire signed [31:0] selected_center,selected_diff;
    wire kp_valid,count_clear,stall_clear;
    integer checks=0, failures=0;
    uart_command_router #(.CLK_FREQ_HZ(100000),.BYTE_TIMEOUT_MS(20)) router (
        .i_clock(clk),.i_reset_n(rst_n),.i_rx_data(rx),.i_rx_valid(rx_valid),
        .o_ascii_data(ascii_data),.o_ascii_valid(ascii_valid),.o_ascii_abort(ascii_abort),
        .o_motion_data(motion_data),.o_motion_valid(motion_valid));
    uart_cmd_rx ascii_parser (
        .i_clock(clk),.i_reset_n(rst_n),.i_abort(ascii_abort),
        .i_rx_data(ascii_data),.i_rx_valid(ascii_valid),
        .o_tgt_valid(),.o_tgt_sel(),.o_tgt_value(),.o_kp_valid(kp_valid),.o_kp_value(),
        .o_ki_valid(),.o_ki_value(),.o_kd_valid(),.o_kd_value(),
        .o_mode_valid(),.o_closed_loop(),.o_duty_valid(),.o_duty_value(),
        .o_count_clear(count_clear),.o_stall_clear(stall_clear),.o_report_req(),.o_cmd_error());
    voice_motion_bridge #(.CLK_HZ(100000),.WATCHDOG_MS(500),.LOCAL_RELEASE_EN(1)) bridge (
        .clk(clk),.rst_n(rst_n),.i_rx_data(motion_data),.i_rx_valid(motion_valid),
        .i_enable(enable),.i_estop(estop),.i_manual(manual),
        .i_yaw_deg(yaw),.i_yaw_valid(1'b1),.i_imu_sample_valid(samples),
        .i_imu_healthy(healthy),.i_nav_release_ready(1'b0),.i_local_release(local_release),
        .o_owned(owned),.o_auto_valid(valid),.o_auto_center(center),.o_auto_diff(diff),
        .o_active(active),.o_last_seq(),.o_last_cmd(),.o_last_flags(),
        .o_cmd_error(),.o_patrol_start(),.o_patrol_stop());
    voice_nav_arbiter arbiter (
        .i_voice_owned(owned),.i_voice_valid(valid),.i_voice_center(center),.i_voice_diff(diff),
        .i_nav_valid(1'b1),.i_nav_center(32'sd900),.i_nav_diff(32'sd100),
        .o_valid(selected_valid),.o_center(selected_center),.o_diff(selected_diff));
    task check(input bit ok,input [511:0] message);
        begin checks=checks+1; if(ok) $display("pass %0s",message);
        else begin failures=failures+1;$display("FAIL: %0s",message);end end
    endtask
    function [15:0] crc_byte(input [15:0] old,input [7:0] data);
        reg [15:0] c; integer j; begin c=old^{data,8'd0};
        for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;
        crc_byte=c;end
    endfunction
    task byte_tx(input [7:0] b);
        begin @(negedge clk);rx=b;rx_valid=1;@(negedge clk);rx_valid=0;repeat(2)@(negedge clk);end
    endtask
    task command(input [7:0] cmd,input signed [15:0] value);
        reg[7:0] bytes[0:9];reg[15:0] crc;integer j;
        begin bytes[0]="M";bytes[1]="O";bytes[2]="T";bytes[3]="1";
        bytes[4]=32;bytes[5]=cmd;bytes[6]=value[15:8];bytes[7]=value[7:0];
        bytes[8]=30;bytes[9]=0;crc=16'hffff;
        for(j=0;j<10;j=j+1)begin crc=crc_byte(crc,bytes[j]);byte_tx(bytes[j]);end
        byte_tx(crc[15:8]);byte_tx(crc[7:0]);repeat(20)@(negedge clk);end
    endtask
    task hold_check(input [511:0] message);
        begin check(owned && selected_valid && selected_center==0 && selected_diff==0 && !active,message);end
    endtask
    integer forbidden_events=0;
    always @(posedge clk) if(rst_n && (kp_valid || count_clear || stall_clear)) forbidden_events<=forbidden_events+1;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;repeat(10)@(negedge clk);
        check(!owned && selected_center==900,"original navigation selected before voice owns");
        command(1,10);check(owned && active && selected_center>0,"forward crosses real router into production arbiter");
        repeat(1200)@(negedge clk);hold_check("completion holds zero despite continuously nonzero navigation");
        command(0,0);repeat(1000)@(negedge clk);hold_check("STOP owns and brakes navigation");
        command(6,0);hold_check("patrol start without readiness cannot resume old UI route");
        command(1,1000);repeat(51000)@(negedge clk);hold_check("watchdog leaves persistent zero authority");
        command(1,1000);manual=1;repeat(4)@(negedge clk);
        check(owned&&!selected_valid&&selected_center==0,"manual takeover suppresses voice and navigation leak");
        manual=0;repeat(4)@(negedge clk);hold_check("return from manual never resumes prior motion");
        command(1,1000);estop=1;repeat(4)@(negedge clk);
        check(owned&&!selected_valid&&selected_center==0,"emergency stop suppresses voice and navigation leak");
        estop=0;repeat(4)@(negedge clk);hold_check("emergency release never resumes prior motion");
        command(3,90);check(active && selected_diff>0,"left turn accepted with healthy sampled yaw");
        healthy=0;repeat(4)@(negedge clk);hold_check("IMU fault during turn holds zero against navigation");
        healthy=1;repeat(4)@(negedge clk);command(3,90);samples=0;
        repeat(20500)@(negedge clk);hold_check("constant yaw_valid cannot mask missing real IMU samples");
        samples=1;command(0,0);check(forbidden_events==0,"all binary commands isolated from ASCII PID and clear events");
        byte_tx("M");byte_tx("O");byte_tx("T");byte_tx("1");byte_tx(8'd42);byte_tx(8'd1);
        repeat(2100)@(negedge clk);command(1,10);
        check(active && selected_center>0,"first complete frame after both parser timeouts recovers");
        command(0,0);hold_check("STOP after truncated-frame recovery holds authority");
        enable=0;repeat(10)@(negedge clk);check(owned && !selected_valid && selected_center==0 && selected_diff==0 && !active,"RK disconnect never exposes stale local navigation");
        local_release=1;manual=1;repeat(3)@(negedge clk);check(owned,"manual cannot release autonomous ownership");
        manual=0;estop=1;repeat(3)@(negedge clk);check(owned,"estop blocks local navigation handoff");
        local_release=0;estop=0;repeat(3)@(negedge clk);check(owned && !selected_valid && selected_center==0 && !active,"recovery alone never resumes a task");
        local_release=1;repeat(3)@(negedge clk);local_release=0;
        check(!owned && selected_center==900,"fresh local start restores original FPGA navigation without RK");
        command(0,0);check(!owned && selected_center==900,"late RK STOP cannot lock out explicitly restored local navigation");
        enable=1;repeat(3)@(negedge clk);command(1,10);
        check(owned && active,"new AI session can own motion after explicit local handoff");
        local_release=1;repeat(3)@(negedge clk);local_release=0;check(owned,"active AI rejects local ownership release");
        command(0,0);
        $display("CHECKS: %0d",checks);
        if(failures!=0)$fatal(1,"integration failures=%0d",failures);
        $display("PASS: voice motion integration complete");$finish;
    end
    initial begin #3000000000;$fatal(1,"integration timeout");end
endmodule
