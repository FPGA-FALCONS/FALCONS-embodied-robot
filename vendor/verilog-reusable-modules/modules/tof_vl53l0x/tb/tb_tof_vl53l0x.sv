`timescale 1ns/1ps
module tb_tof_vl53l0x;
    reg clk=0; always #125 clk=~clk;
    reg rst_n=0,retry=0;
    tri1 scl,sda,xshut_n;
    reg hold_scl=0,hold_sda=0;
    assign scl=hold_scl ? 1'b0:1'bz;
    assign sda=hold_sda ? 1'b0:1'bz;
    reg [3:0] mode=0,status=11;
    reg [15:0] requested=16'h1234;
    reg aperture=1;
    wire [15:0] distance;
    wire valid,pulse,initialized,busy;
    wire [3:0] raw;
    wire [4:0] fault;
    wire integer model_checks,clears,starts;
    integer checks=0,failures=0,groups=0,pulses=0,last_clear=0;
    reg previous_pulse=0;
    reg [15:0] previous_distance=0;
    tof_vl53l0x #(.CLK_HZ(4000000),.I2C_HZ(400000),
      .I2C_TIMEOUT_US(30),.BOOT_DELAY_US(10),.RESET_LOW_US(10),
      .POLL_INTERVAL_US(5),.INIT_TIMEOUT_US(2000),.MEAS_TIMEOUT_US(2000),
      .SAMPLE_PERIOD_US(2500),.STALE_TIMEOUT_US(8000)) dut (
      .clk(clk),.rst_n(rst_n),.retry(retry),.scl(scl),.sda(sda),.xshut_n(xshut_n),
      .distance_mm(distance),.range_valid(valid),.sample_pulse(pulse),
      .initialized(initialized),.busy(busy),.raw_range_status(raw),.fault_code(fault));
    vl53l0x_sensor_model sensor(.xshut_n(xshut_n),.scl(scl),.sda(sda),
      .mode(mode),.next_distance(requested),.next_status(status),.aperture(aperture),
      .checks(model_checks),.clears(clears),.starts(starts));
    task check;
      input condition;
      input [511:0] description;
      begin
        checks=checks+1;
        if(condition!==1'b1) begin failures=failures+1;$display("FAIL %0s time=%0t",description,$time);end
      end
    endtask
    always @(negedge clk) begin
      if(!rst_n || retry || !initialized) begin
        previous_pulse=0;previous_distance=distance;last_clear=0;
      end else begin
        if(pulse) begin
          check(!previous_pulse,"sample pulse is exactly one cycle");
          check(clears>last_clear,"published sample follows physical IRQ clear");
          last_clear=clears;pulses=pulses+1;
        end
        if(distance!=previous_distance) check(pulse,"distance changes atomically with sample pulse");
        previous_distance=distance;previous_pulse=pulse;
      end
    end
    task restart;
      input [3:0] new_mode;
      begin
        @(negedge clk);rst_n=0;mode=new_mode;hold_scl=0;hold_sda=0;
        repeat(8) @(negedge clk);
        rst_n=1;
      end
    endtask
    task await_sample;
      input [15:0] expected;
      input expected_valid;
      integer limit;
      begin
        limit=0;
        while(!pulse && limit<600000) begin @(posedge clk);#1;limit=limit+1;end
        check(pulse,"bounded sample completion");
        check(distance==expected,"big-endian distance decoded");
        check(valid==expected_valid,"range validity follows raw status 11");
        check(raw==status,"raw sensor status is exposed");
        check(initialized,"sample requires completed initialization");
        @(negedge clk);@(negedge clk);
      end
    endtask
    task await_fault;
      input [4:0] expected;
      integer limit;
      begin
        limit=0;
        while(fault!=expected && limit<600000) begin @(negedge clk);limit=limit+1;end
        check(fault==expected,"bounded specific fault");
        check(!valid && !initialized,"fault invalidates sample and initialization");
        check(xshut_n===0,"fatal fault holds sensor in reset");
        repeat(20) @(negedge clk);
        check(!pulse && fault==expected,"fatal state remains latched");
      end
    endtask
    initial begin
      #1000000000;$display("FAIL global watchdog");$fatal(1);
    end
    initial begin
      restart(0);groups=groups+1;
      await_sample(16'h1234,1);
      check(sensor.tuning_done && sensor.cal_stage==2,"all 80 tuning writes and both calibrations checked");
      requested=16'habcd;await_sample(16'habcd,1);
      status=4;requested=16'h4321;await_sample(16'h4321,0);
      status=11;requested=16'd99;await_sample(16'd99,1);
      groups=groups+1;mode=6;
      await_fault(8);
      // Retry, without global reset, must recover a live sensor and new data.
      groups=groups+1;mode=0;requested=777;
      @(negedge clk);retry=1;@(negedge clk);retry=0;
      await_sample(777,1);
      groups=groups+1;aperture=0;restart(0);await_sample(777,1);
      check(sensor.written_map==sensor.expected_map,"non-aperture map supported");
      groups=groups+1;restart(1);await_fault(1);
      groups=groups+1;restart(2);await_fault(4);
      groups=groups+1;restart(3);await_fault(5);
      groups=groups+1;restart(4);await_fault(6);
      groups=groups+1;restart(5);await_fault(7);
      groups=groups+1;restart(7);await_fault(9);
      groups=groups+1;restart(9);await_fault(9);
      groups=groups+1;restart(8);await_fault(1);
      check(distance==0 && clears==0,"IRQ-clear NACK suppresses atomic sample publication");
      groups=groups+1;restart(0);hold_scl=1;await_fault(2);hold_scl=0;
      groups=groups+1;restart(0);hold_sda=1;await_fault(3);hold_sda=0;
      // Reset halfway through a physical I2C byte, then complete initialization.
      groups=groups+1;restart(0);
      @(negedge sda);repeat(3) @(posedge scl);
      @(negedge clk);rst_n=0;
      repeat(8) @(negedge clk);
      check(scl===1 && sda===1 && !valid,"reset aborts transaction and releases bus");
      rst_n=1;await_sample(777,1);
      $display("TOF END-TO-END: %0d groups, %0d checks, %0d model checks, %0d failures",groups,checks,model_checks,failures);
      if(failures) $fatal(1);
      $finish;
    end
endmodule
