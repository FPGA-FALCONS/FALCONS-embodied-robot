`timescale 1ns/1ps
// Behavioral protocol model: interacts exclusively with open-drain pins.
// Register expectations are independently transcribed from the cited Pololu
// initialization sequence, not generated from the DUT instruction function.
module vl53l0x_sensor_model (
    input wire xshut_n, inout wire scl, inout wire sda,
    input wire [3:0] mode,
    input wire [15:0] next_distance,
    input wire [3:0] next_status,
    input wire aperture,
    output integer checks = 0,
    output integer clears = 0,
    output integer starts = 0
);
    reg sda_low = 0;
    assign sda = sda_low ? 1'b0 : 1'bz;
    reg [7:0] memory [0:2047];
    reg [7:0] page, pointer;
    reg [47:0] factory_map, expected_map, written_map;
    integer tuning_index, kind, ready_reads, start_reads, read_stage;
    integer cal_stage, stop_reads, restore_writes, i, enabled;
    reg tuning_active, tuning_done;
    reg [15:0] captured_distance;
    reg [3:0] captured_status;
    function [15:0] tuning;
      input integer index;
      begin
        case (index)
        0: tuning = 16'hff01;
        1: tuning = 16'h0000;
        2: tuning = 16'hff00;
        3: tuning = 16'h0900;
        4: tuning = 16'h1000;
        5: tuning = 16'h1100;
        6: tuning = 16'h2401;
        7: tuning = 16'h25ff;
        8: tuning = 16'h7500;
        9: tuning = 16'hff01;
        10: tuning = 16'h4e2c;
        11: tuning = 16'h4800;
        12: tuning = 16'h3020;
        13: tuning = 16'hff00;
        14: tuning = 16'h3009;
        15: tuning = 16'h5400;
        16: tuning = 16'h3104;
        17: tuning = 16'h3203;
        18: tuning = 16'h4083;
        19: tuning = 16'h4625;
        20: tuning = 16'h6000;
        21: tuning = 16'h2700;
        22: tuning = 16'h5006;
        23: tuning = 16'h5100;
        24: tuning = 16'h5296;
        25: tuning = 16'h5608;
        26: tuning = 16'h5730;
        27: tuning = 16'h6100;
        28: tuning = 16'h6200;
        29: tuning = 16'h6400;
        30: tuning = 16'h6500;
        31: tuning = 16'h66a0;
        32: tuning = 16'hff01;
        33: tuning = 16'h2232;
        34: tuning = 16'h4714;
        35: tuning = 16'h49ff;
        36: tuning = 16'h4a00;
        37: tuning = 16'hff00;
        38: tuning = 16'h7a0a;
        39: tuning = 16'h7b00;
        40: tuning = 16'h7821;
        41: tuning = 16'hff01;
        42: tuning = 16'h2334;
        43: tuning = 16'h4200;
        44: tuning = 16'h44ff;
        45: tuning = 16'h4526;
        46: tuning = 16'h4605;
        47: tuning = 16'h4040;
        48: tuning = 16'h0e06;
        49: tuning = 16'h201a;
        50: tuning = 16'h4340;
        51: tuning = 16'hff00;
        52: tuning = 16'h3403;
        53: tuning = 16'h3544;
        54: tuning = 16'hff01;
        55: tuning = 16'h3104;
        56: tuning = 16'h4b09;
        57: tuning = 16'h4c05;
        58: tuning = 16'h4d04;
        59: tuning = 16'hff00;
        60: tuning = 16'h4400;
        61: tuning = 16'h4520;
        62: tuning = 16'h4708;
        63: tuning = 16'h4828;
        64: tuning = 16'h6700;
        65: tuning = 16'h7004;
        66: tuning = 16'h7101;
        67: tuning = 16'h72fe;
        68: tuning = 16'h7600;
        69: tuning = 16'h7700;
        70: tuning = 16'hff01;
        71: tuning = 16'h0d01;
        72: tuning = 16'hff00;
        73: tuning = 16'h8001;
        74: tuning = 16'h01f8;
        75: tuning = 16'hff01;
        76: tuning = 16'h8e01;
        77: tuning = 16'h0001;
        78: tuning = 16'hff00;
        79: tuning = 16'h8000;
        default: tuning = 16'hffff;
        endcase
      end
    endfunction
    task verify;
      input condition;
      input [511:0] description;
      begin
        checks = checks + 1;
        if (condition !== 1'b1) begin
          $display("SENSOR MODEL FAIL: %0s page=%0h reg=%0h time=%0t",description,page,pointer,$time);
          $fatal(1);
        end
      end
    endtask
    task wait_start;
      begin : detect_start
        forever begin
          @(negedge sda);
          if (scl === 1'b1) disable detect_start;
        end
      end
    endtask
    task wait_stop;
      begin : detect_stop
        forever begin
          @(posedge sda);
          if (scl === 1'b1) disable detect_stop;
        end
      end
    endtask
    task receive;
      output [7:0] value;
      integer b;
      begin
        for (b=7;b>=0;b=b-1) begin @(posedge scl); value[b]=sda; end
        @(negedge scl); sda_low=1;
        @(posedge scl); @(negedge scl); sda_low=0;
      end
    endtask
    task transmit;
      input [7:0] value;
      integer b;
      begin
        for (b=7;b>=0;b=b-1) begin
          sda_low=!value[b]; @(posedge scl); @(negedge scl);
        end
        sda_low=0;
        @(posedge scl); verify(sda === 1'b1,"final read byte must be NACKed");
        @(negedge scl);
      end
    endtask
    task read_register;
      output [7:0] value;
      begin
        verify(page<8,"legal page on read");
        value=memory[{page[2:0],pointer}];
        case ({page,pointer})
          16'h00c0: value = mode==2 ? 8'hab : 8'hee;
          16'h0191: begin
            verify(memory[8'h80]==1 && memory[256]==0,"stop variable access bank unlock");
            value=8'h6d; stop_reads=stop_reads+1;
          end
          16'h0783: value= mode==3 ? 0 : 1;
          16'h0792: value = {aperture,mode==9 ? 7'd0 : 7'd7};
          16'h0013: begin
            verify(kind!=0,"ready is read only after measurement/calibration start");
            ready_reads=ready_reads+1;
            if ((kind==3 && mode==6) || (kind!=3 && mode==4) || ready_reads==1) value=0;
            else value=7;
          end
          16'h0000: begin
            if (kind==3) begin
              start_reads=start_reads+1;
              value=(mode==5 || start_reads==1) ? 1 : 0;
            end
          end
          16'h0014: begin
            verify(kind==3 && ready_reads>=2 && read_stage==0,"range status first");
            value={1'b0,captured_status,3'b0}; read_stage=1;
          end
          16'h001e: begin
            verify(read_stage==1,"distance high byte before low");
            value=captured_distance[15:8]; read_stage=2;
          end
          16'h001f: begin
            verify(read_stage==2,"distance low byte after high");
            value=captured_distance[7:0]; read_stage=3;
          end
          default: begin end
        endcase
      end
    endtask
    task write_register;
      input [7:0] value;
      begin
        verify(page<8,"legal page on write");
        if (tuning_active) begin
          verify({pointer,value}==tuning(tuning_index),"80 tuning writes match independent reference");
          tuning_index=tuning_index+1;
          if(tuning_index==80) begin tuning_active=0; tuning_done=1; end
        end
        if(page==0 && pointer>=8'hb0 && pointer<=8'hb5) begin
          written_map[(pointer-8'hb0)*8+:8]=value;
          if(pointer==8'hb5) begin
            verify(written_map==expected_map,"SPAD map preserves factory holes and aperture offset/count");
            tuning_active=1; tuning_index=0;
          end
        end
        if(page==1 && pointer==8'h91) begin
          verify(value==8'h6d && stop_reads==1,"restore actual factory stop variable");
          verify(memory[8'h80]==1 && memory[256]==0,"stop restore page/unlock order");
          restore_writes=restore_writes+1;
        end
        if(page==0 && pointer==0 && value[0] && tuning_done) begin
          if(memory[1]==1) begin
            verify(memory[8'h84]==8'hef,"active-low IRQ RMW preserves unrelated bits");
            verify(value==8'h41 && cal_stage==0,"VHV calibration first");
            kind=1;
          end else if(memory[1]==2) begin
            verify(value==1 && cal_stage==1,"phase calibration second");
            kind=2;
          end else if(memory[1]==8'he8) begin
            verify(cal_stage==2 && restore_writes==starts+1,"single shot after calibrations and stopvar restore");
            kind=3; starts=starts+1;
            captured_distance=next_distance; captured_status=next_status;
          end else verify(0,"unexpected ranging sequence configuration");
          ready_reads=0; start_reads=0; read_stage=0;
        end
        if(page==0 && pointer==8'h0b && value==1 && kind!=0) begin
          if(kind==3) begin
            verify(read_stage==3,"clear IRQ only after complete status/high/low read");
            clears=clears+1;
          end else cal_stage=cal_stage+1;
          kind=0;
        end
        memory[{page[2:0],pointer}]=value;
        if(pointer==8'hff) page=value;
      end
    endtask
    task transaction;
      reg [7:0] address,value;
      reg read_detected;
      begin : transfer
        wait_start;
        if(mode==1) begin wait_stop; disable transfer; end
        receive(address);
        verify(address==8'h52,"write address is 0x29");
        receive(pointer);
        read_detected=0;
        // Distinguish a read's repeated START from a write's next SCL edge.
        fork : choose
          begin
            wait_start;
            read_detected=1;
            receive(address); verify(address==8'h53,"repeated-start read address");
            read_register(value); transmit(value); wait_stop;
          end
          begin : write_part
            @(posedge scl);
            // The write data's first bit; remaining seven arrive on later edges.
            value[7]=sda;
            @(negedge scl);
            if(read_detected) begin
              // Keep this branch alive until the read branch finishes.
              wait_stop;
              disable write_part;
            end
            for(integer b=6;b>=0;b=b-1) begin @(posedge scl); value[b]=sda; end
            @(negedge scl);
            if(mode==8 && page==0 && pointer==8'h0b && kind==3) begin
              // Fault at the final IRQ clear: all data was read, but none may
              // become public until this write has actually been acknowledged.
              sda_low=0;
            end else begin
              write_register(value);
              sda_low=1;
            end
            @(posedge scl); @(negedge scl); sda_low=0;
            wait_stop;
          end
        join_any
        disable choose;
      end
    endtask
    initial forever begin
      wait(xshut_n===1'b1);
      page=0;pointer=0;tuning_index=0;tuning_active=0;tuning_done=0;
      kind=0;cal_stage=0;stop_reads=0;restore_writes=0;starts=0;clears=0;
      ready_reads=0;start_reads=0;read_stage=0;sda_low=0;written_map=0;
      for(i=0;i<2048;i=i+1) memory[i]=0;
      factory_map=mode==7 ? 48'h000000001000 : 48'hf0fff0f0a55a;
      expected_map=0;enabled=0;
      for(i=0;i<48;i=i+1)
        if((!aperture || i>=12) && factory_map[i] && enabled<7) begin
          expected_map[i]=1;enabled=enabled+1;
        end
      for(i=0;i<6;i=i+1) memory[8'hb0+i]=factory_map[i*8+:8];
      memory[8'h89]=8'ha4;memory[8'h60]=8'h81;memory[8'h84]=8'hff;
      fork : session
        forever transaction;
        begin wait(xshut_n===1'b0); end
      join_any
      disable session;
      sda_low=0;
    end
endmodule
