`timescale 1ns/1ps
module tb_tof_stale;
    reg clk=0;always #125 clk=~clk;
    reg rst_n=0;
    tri1 scl,sda,xshut_n;
    wire [15:0] distance;
    wire valid,pulse,initialized,busy;
    wire [3:0] raw;
    wire [4:0] fault;
    wire integer model_checks,clears,starts;
    integer checks=0;
    tof_vl53l0x #(.CLK_HZ(4000000),.I2C_HZ(400000),
      .I2C_TIMEOUT_US(30),.BOOT_DELAY_US(10),.RESET_LOW_US(10),
      .POLL_INTERVAL_US(5),.INIT_TIMEOUT_US(2000),.MEAS_TIMEOUT_US(2000),
      .SAMPLE_PERIOD_US(10000),.STALE_TIMEOUT_US(2000)) dut (
      .clk(clk),.rst_n(rst_n),.retry(1'b0),.scl(scl),.sda(sda),.xshut_n(xshut_n),
      .distance_mm(distance),.range_valid(valid),.sample_pulse(pulse),
      .initialized(initialized),.busy(busy),.raw_range_status(raw),.fault_code(fault));
    vl53l0x_sensor_model sensor(.xshut_n(xshut_n),.scl(scl),.sda(sda),
      .mode(4'd0),.next_distance(16'd321),.next_status(4'd11),.aperture(1'b1),
      .checks(model_checks),.clears(clears),.starts(starts));
    task check;
      input condition;
      input [511:0] description;
      begin
        checks=checks+1;
        if(condition!==1'b1) begin $display("FAIL %0s",description);$fatal(1);end
      end
    endtask
    initial begin #100000000;$display("FAIL stale watchdog");$fatal(1);end
    initial begin
      repeat(8) @(negedge clk);rst_n=1;
      wait(pulse);@(negedge clk);
      check(valid && distance==321 && fault==0,"fresh sample valid");
      wait(fault==10);@(negedge clk);
      check(!valid && initialized && xshut_n===1,"stale invalidates without fatal shutdown");
      check(distance==321,"stale sample value retained for diagnostics");
      wait(pulse);@(negedge clk);
      check(valid && fault==0 && distance==321,"next valid sample automatically clears stale fault");
      $display("TOF STALE: %0d checks, %0d model checks, 0 failures",checks,model_checks);
      $finish;
    end
endmodule
