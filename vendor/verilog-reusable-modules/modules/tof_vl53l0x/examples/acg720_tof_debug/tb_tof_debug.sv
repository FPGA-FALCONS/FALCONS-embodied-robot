`timescale 1ns/1ps
// Real ToF core + physical open-drain sensor model + physical 595 scoreboard.
// Only timing parameters are shortened; no module or internal signal is forced.
module tb_tof_debug;
    reg clk=0;always #125 clk=~clk;
    reg rst_n=0;
    tri1 tof_scl,tof_sda,tof_xshut_n;
    wire ds,sh_cp,st_cp;
    wire [7:0] led;
    reg [3:0] mode=0,status=11;
    reg [15:0] requested=300;
    wire integer model_checks,clears,starts;
    tof_debug_top #(.CLK_HZ(4000000),.I2C_HZ(400000),
      .I2C_TIMEOUT_US(30),.BOOT_DELAY_US(10),.RESET_LOW_US(10),
      .POLL_INTERVAL_US(5),.INIT_TIMEOUT_US(2000),.MEAS_TIMEOUT_US(15000),
      .SAMPLE_PERIOD_US(2500),.STALE_TIMEOUT_US(8000),
      .DIGIT_US(50),.SCLK_DIVIDER(2),.HEARTBEAT_US(1000)) dut(.*);
    vl53l0x_sensor_model sensor(.xshut_n(tof_xshut_n),.scl(tof_scl),.sda(tof_sda),
      .mode(mode),.next_distance(requested),.next_status(status),.aperture(1'b1),
      .checks(model_checks),.clears(clears),.starts(starts));
    integer checks=0,groups=0,latches=0,frames=0,heartbeat_edges=0,sample_edges=0;
    integer shifted=0,index,i,last_index=-1;
    reg [15:0] serial=0;
    reg [63:0] assembling=64'h0,observed=64'h0;
    time last_shift_rise=0,last_shift_fall=0,last_latch=0;
    function [7:0] digit;
      input integer n;
      begin case(n)
        0:digit=8'hc0;1:digit=8'hf9;2:digit=8'ha4;3:digit=8'hb0;
        4:digit=8'h99;5:digit=8'h92;6:digit=8'h82;7:digit=8'hf8;
        8:digit=8'h80;9:digit=8'h90;default:digit=8'hxx;
      endcase end
    endfunction
    function [63:0] expected;
      input integer mm;
      reg [31:0] left,right;
      begin
        if(mm<100)left=32'h9287a38c;
        else if(mm<200)left=32'he388afab;
        else left=32'h92888e86;
        if(mm>9999)right=32'hffff89f9;
        else right={digit((mm/1000)%10),digit((mm/100)%10),digit((mm/10)%10),digit(mm%10)};
        expected={left,right};
      end
    endfunction
    // Immediate invalidation may replace the remaining high-numbered digits
    // with dashes halfway through one scan. Recovery may not revive old data
    // in that scan. Normal-to-normal changes must remain fully atomic.
    function valid_or_invalidated;
      input [63:0] frame,reference;
      integer n;
      reg invalid_tail;
      begin
        valid_or_invalidated=1;invalid_tail=0;
        for(n=0;n<8;n=n+1)begin
          if(frame[n*8+:8]!==reference[n*8+:8])invalid_tail=1;
          if(invalid_tail && frame[n*8+:8]!==8'hbf)valid_or_invalidated=0;
        end
      end
    endfunction
    function legal_frame;
      input [63:0] frame;
      begin
        legal_frame=valid_or_invalidated(frame,64'hf9abf987bfbfbfbf) || frame===64'hbfbfbfbfbfbfbfbf ||
          valid_or_invalidated(frame,expected(300))||valid_or_invalidated(frame,expected(150))||valid_or_invalidated(frame,expected(80))||
          valid_or_invalidated(frame,expected(99))||valid_or_invalidated(frame,expected(100))||valid_or_invalidated(frame,expected(199))||valid_or_invalidated(frame,expected(200))||
          frame===64'hff86afafc0c0c0f9;
      end
    endfunction
    task check;
      input condition;
      input [511:0] label;
      begin checks=checks+1;
        if(condition!==1'b1)$fatal(1,"TOP FAIL %0s time=%0t frame=%h led=%b",label,$time,observed,led);
      end
    endtask
    always @(negedge rst_n) begin
      shifted=0;last_index=-1;last_shift_rise=0;last_shift_fall=0;last_latch=0;
      assembling=0;observed=0;
    end
    always @(posedge sh_cp) if(rst_n) begin
      check(st_cp===0,"595 never shifts during latch");
      check(ds===0||ds===1,"known serial bit");
      if(last_shift_fall)check($time-last_shift_fall>=500,"595 low time at least two system clocks");
      last_shift_rise=$time;serial={serial[14:0],ds};shifted=shifted+1;
      check(shifted<=16,"no extra serial clocks");
    end
    always @(negedge sh_cp) if(rst_n&&last_shift_rise)begin
      check($time-last_shift_rise>=500,"595 high time");last_shift_fall=$time;
    end
    always @(posedge st_cp) if(rst_n)begin
      check(sh_cp===0,"latch with shift clock low");
      check(shifted==16,"exactly 16 clocks per 595 word");
      check($time-last_shift_fall>=500,"shift to latch setup");
      check(serial[7:0]!=0 && (serial[7:0]&(serial[7:0]-8'd1))==0,"one-hot digit select");
      check(serial[15]===1,"decimal point off");
      index=0;for(i=0;i<8;i=i+1)if(serial[i])index=i;
      if(last_index>=0)begin
        check(index==(last_index+1)%8,"digits scan right to left");
        check($time-last_latch==50000,"50us digit scan cadence");
      end
      last_index=index;last_latch=$time;
      assembling[index*8+:8]=serial[15:8];
      if(index==7)begin
        observed=assembling;frames=frames+1;
        check(legal_frame(observed),"each complete physical scan is one coherent allowed frame");
      end
      shifted=0;latches=latches+1;
    end
    always @(negedge st_cp)if(rst_n&&last_latch)
      check($time-last_latch>=500,"595 latch pulse width");
    always @(led[7])if(rst_n)heartbeat_edges=heartbeat_edges+1;
    always @(led[5])if(rst_n)sample_edges=sample_edges+1;
    task await_frame;
      input [63:0] frame;
      input [511:0] label;
      integer limit,frame_start;
      begin
        limit=0;frame_start=frames;
        while((observed!==frame || frames==frame_start)&&limit<160000)begin
          @(negedge clk);limit=limit+1;
        end
        check(observed===frame&&frames>frame_start,label);
        groups=groups+1;$display("PASS %0d %0s frame=%h led=%b",groups,label,observed,led);
      end
    endtask
    task verify_distance;
      input [15:0] mm;
      begin
        requested=mm;await_frame(expected(mm),"distance threshold and all eight physical segment bytes");
        check(led[0]&&led[1]&&!led[6],"initialized valid and no fault LEDs");
        check(led[2]==((mm>=100)&&(mm<200)),"WARN LED exact threshold");
        check(led[3]==(mm<100),"STOP LED exact threshold");
      end
    endtask
    task reset_board;
      begin
        @(negedge clk);rst_n=0;repeat(8)@(negedge clk);
        check(tof_scl===1&&tof_sda===1,"S4 releases physical I2C bus");
        rst_n=1;
      end
    endtask
    initial begin
      repeat(8)@(negedge clk);rst_n=1;
      await_frame(64'hf9abf987bfbfbfbf,"startup init and invalid distance");
      verify_distance(300);verify_distance(150);verify_distance(80);
      verify_distance(99);verify_distance(100);verify_distance(199);verify_distance(200);
      status=4;await_frame(64'hbfbfbfbfbfbfbfbf,"sensor raw invalid status blanks distance");
      check(led[0]&&!led[1]&&!led[2]&&!led[3]&&!led[6],"invalid range never shows safe/stop decision");
      status=11;verify_distance(150);
      mode=6;await_frame(64'hbfbfbfbfbfbfbfbf,"stale sample blanks display before measurement timeout");
      check(led[6]&&!led[1]&&!led[2]&&!led[3],"stale fault LED and no obsolete obstacle decision");
      mode=0;verify_distance(80);
      mode=1;reset_board;
      await_frame(64'hff86afafc0c0c0f9,"missing device NACK displays Err 0001");
      check(led[6]&&!led[0]&&!led[1]&&!led[2]&&!led[3],"fatal fault LEDs");
      check(tof_xshut_n===0,"fatal fault physically holds XSHUT low");
      mode=0;reset_board;verify_distance(300);
      check(heartbeat_edges>10,"heartbeat changes throughout real I2C traffic");
      check(sample_edges>8,"sample activity LED toggles on repeated acquisitions");
      check(model_checks>1000,"actual initialization and physical I2C model were exercised");
      $display("PASS tb_tof_debug groups=%0d checks=%0d model_checks=%0d physical_latches=%0d complete_frames=%0d",groups,checks,model_checks,latches,frames);
      $finish;
    end
    initial begin #500000000;$fatal(1,"board test watchdog");end
endmodule
