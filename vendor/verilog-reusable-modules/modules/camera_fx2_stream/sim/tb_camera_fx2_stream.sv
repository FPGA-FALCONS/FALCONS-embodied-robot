`timescale 1ns/1ps
module tb_camera_fx2_stream;
  parameter integer SOURCE_WIDTH=8, SOURCE_HEIGHT=6, DOWNSAMPLE=2;
  localparam integer WIDTH=SOURCE_WIDTH/DOWNSAMPLE;
  localparam integer HEIGHT=SOURCE_HEIGHT/DOWNSAMPLE;
  localparam integer NPIX=SOURCE_WIDTH*SOURCE_HEIGHT;
  localparam integer PAYLOAD=WIDTH*HEIGHT*2;
  localparam integer FRAME_BYTES=24+PAYLOAD;
  reg cam_clk=0, ifclk=0;
  always #7 cam_clk=~cam_clk;
  always #5 ifclk=~ifclk;
  reg reset_n=0, cam_vsync=0, cam_valid=0;
  reg [15:0] cam_rgb565=0;
  reg fx2_flagc=0;
  wire [15:0] fx2_fd;
  wire [1:0] fx2_faddr;
  wire fx2_slwr_n, fx2_sloe_n, fx2_slrd_n, fx2_pktend_n;
  wire capture_busy;
  wire [31:0] frames_sent, frames_dropped, frames_malformed;
  camera_fx2_stream #(.SOURCE_WIDTH(SOURCE_WIDTH),.SOURCE_HEIGHT(SOURCE_HEIGHT),.DOWNSAMPLE(DOWNSAMPLE)) dut (.*);

  reg [7:0] expected [0:65535];
  integer expected_len=0, seen=0, wire_words=0, stall_cycles=0;
  integer gap_since_write=1000, packets=0, completed=0;
  integer pktend_stall_cycles=0;
  integer bin_fd, hex_fd, records_fd;
  reg random_stalls=0, was_stalled=0;
  reg [15:0] stalled_data;
  reg [31:0] rng=32'h31415926;
  reg previous_pktend=0;
  string out_dir;

  function automatic [31:0] crc_byte(input [31:0] crc_in,input [7:0] b);
    reg [31:0] c;
    integer k;
    begin
      c=crc_in ^ b;
      for(k=0;k<8;k=k+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
      crc_byte=c;
    end
  endfunction

  task automatic put16(input integer at,input integer value);
    begin expected[at]=value&255; expected[at+1]=(value>>8)&255; end
  endtask
  task automatic put32(input integer at,input reg [31:0] value);
    begin expected[at]=value[7:0]; expected[at+1]=value[15:8]; expected[at+2]=value[23:16]; expected[at+3]=value[31:24]; end
  endtask
  task automatic expect_frame(input integer tag,input integer seq,input integer keep_record);
    integer x,y,j,pixel;
    reg [31:0] crc;
    begin
      if(expected_len!=0 && seen!=expected_len) $fatal(1,"Overwriting incomplete scoreboard");
      expected_len=FRAME_BYTES; seen=0;
      expected[0]="F"; expected[1]="I"; expected[2]="M"; expected[3]="G";
      expected[4]=1; expected[5]=1;
      put16(6,24); put16(8,WIDTH); put16(10,HEIGHT);
      put32(12,seq); put32(16,PAYLOAD);
      j=24;
      for(y=0;y<SOURCE_HEIGHT;y=y+DOWNSAMPLE)
        for(x=0;x<SOURCE_WIDTH;x=x+DOWNSAMPLE) begin
          pixel=((tag<<8)+(y*SOURCE_WIDTH+x))&65535;
          put16(j,pixel); j=j+2;
        end
      crc=32'hffffffff;
      for(j=0;j<20;j=j+1) crc=crc_byte(crc,expected[j]);
      for(j=24;j<FRAME_BYTES;j=j+1) crc=crc_byte(crc,expected[j]);
      put32(20,~crc);
      if(keep_record) $fdisplay(records_fd,"%0d %0d %0d %0d %0d %0d %0d",seq,tag,SOURCE_WIDTH,SOURCE_HEIGHT,DOWNSAMPLE,WIDTH,HEIGHT);
    end
  endtask

  // Model FX2 readiness changes only at the falling edge: accepted writes are
  // evaluated at IFCLK rising edges, matching the synchronous slave-FIFO bus.
  always @(negedge ifclk) begin
    if(random_stalls && reset_n) begin
      rng = {rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
      fx2_flagc = (rng[3:0]>4);
    end
  end
  always @(posedge ifclk) begin
    if(!reset_n) begin
      was_stalled=0; previous_pktend=0; gap_since_write=1000;
    end else begin
      gap_since_write=gap_since_write+1;
      if(fx2_faddr!==2'b10 || fx2_sloe_n!==1 || fx2_slrd_n!==1)
        $fatal(1,"FX2 fixed bus direction/endpoint mismatch");
      if(!fx2_flagc && !fx2_slwr_n) $fatal(1,"SLWR asserted while FLAGC full");
      if(was_stalled && fx2_fd!==stalled_data) $fatal(1,"FD changed while transfer stalled");
      was_stalled=(!fx2_flagc && (dut.state==dut.TX_SEND || dut.state==dut.TX_HEADER_SEND));
      stalled_data=fx2_fd;
      if(!fx2_flagc && seen<expected_len) stall_cycles=stall_cycles+1;
      if(!fx2_slwr_n) begin
        if(seen+1>=expected_len) $fatal(1,"Unexpected write seen=%0d expected=%0d",seen,expected_len);
        if(fx2_fd!=={expected[seen+1],expected[seen]})
          $fatal(1,"Wire mismatch at byte%0d got%04x expected%02x%02x",seen,fx2_fd,expected[seen+1],expected[seen]);
        $fwrite(bin_fd,"%c%c",fx2_fd[7:0],fx2_fd[15:8]);
        $fdisplay(hex_fd,"%02x\n%02x",fx2_fd[7:0],fx2_fd[15:8]);
        wire_words=wire_words+1; seen=seen+2; gap_since_write=0;
        if(seen==expected_len) completed=completed+1;
      end
      if(!fx2_pktend_n) begin
        if(previous_pktend) $fatal(1,"PKTEND longer than one IFCLK");
        if(!fx2_flagc || !fx2_slwr_n || seen!=expected_len || FRAME_BYTES%512==0 || gap_since_write<2)
          $fatal(1,"PKTEND without short-frame completion and full guard");
        packets=packets+1;
      end
      previous_pktend=!fx2_pktend_n;
    end
  end

  always @(posedge cam_clk)
    if(reset_n && dut.cam_write_enable && dut.req_toggle!=dut.ack_toggle)
      $fatal(1,"Camera writes RAM owned by transport");

  task automatic boundary;
    begin
      @(negedge cam_clk); cam_valid=0; cam_vsync=1;
      @(negedge cam_clk); cam_vsync=0;
      repeat(4) @(negedge cam_clk);
    end
  endtask
  task automatic boundary_with_ignored_pixel;
    begin
      @(negedge cam_clk); cam_valid=1; cam_rgb565=16'hdead; cam_vsync=1;
      @(negedge cam_clk); cam_valid=0; cam_vsync=0;
      repeat(4) @(negedge cam_clk);
    end
  endtask
  task automatic pixels(input integer tag,input integer count);
    integer p;
    begin
      for(p=0;p<count;p=p+1) begin
        @(negedge cam_clk); cam_valid=1; cam_rgb565=((tag<<8)+p)&65535;
        if(p%11==3) begin @(negedge cam_clk); cam_valid=0; end
      end
      @(negedge cam_clk); cam_valid=0;
    end
  endtask
  task automatic reset_device;
    begin
      random_stalls=0;
      @(negedge ifclk); reset_n=0; cam_valid=0; cam_vsync=0; fx2_flagc=0;
      repeat(10) @(negedge cam_clk);
      expected_len=0; seen=0;
      reset_n=1;
      repeat(12) @(negedge cam_clk);
    end
  endtask
  task automatic wait_drain(input integer count);
    integer timeout;
    begin
      timeout=0;
      while(frames_sent<count && timeout<200000) begin @(negedge ifclk); timeout=timeout+1; end
      if(timeout==200000) $fatal(1,"Drain timeout");
      repeat(15) @(negedge cam_clk);
      if(seen!=expected_len) $fatal(1,"Frame missing bytes");
    end
  endtask

  integer old_words, old_completed, old_packets;
  initial begin
    if(!$value$plusargs("OUT_DIR=%s",out_dir)) out_dir=".";
    bin_fd=$fopen({out_dir,"/wire.bin"},"wb");
    hex_fd=$fopen({out_dir,"/wire.hex"},"w");
    records_fd=$fopen({out_dir,"/expected_frames.txt"},"w");
    if(!bin_fd || !hex_fd || !records_fd) $fatal(1,"Cannot open output evidence");
    reset_device();
    // Incomplete startup pixels are ignored before the first frame boundary.
    pixels(99,NPIX/2);
    boundary_with_ignored_pixel(); pixels(1,NPIX); expect_frame(1,0,1); boundary_with_ignored_pixel();
    // Freeze first frame in RAM while a complete second frame arrives.
    pixels(2,NPIX); boundary();
    repeat(80) @(negedge ifclk);
    if(wire_words!=0) $fatal(1,"Transport ignored long-full stall");
    if(frames_dropped==0) $fatal(1,"Busy frame was not counted dropped");
    random_stalls=1; wait_drain(1);
    boundary(); pixels(3,NPIX); expect_frame(3,1,1); boundary(); wait_drain(2);
    old_words=wire_words;
    boundary(); pixels(4,NPIX-1); boundary();
    repeat(60) @(negedge cam_clk);
    boundary(); pixels(5,NPIX+1); boundary();
    repeat(60) @(negedge cam_clk);
    if(wire_words!=old_words || frames_malformed<2) $fatal(1,"Malformed frames transmitted or not counted");
    // Abort one published frame after a partial wire prefix. A later complete
    // frame must resynchronize in the host parser; expected manifest excludes it.
    random_stalls=0; fx2_flagc=0;
    boundary(); pixels(6,NPIX); expect_frame(6,2,0); boundary();
    fx2_flagc=1;
    wait(seen>=10); reset_device();
    // Abort an unfinished camera frame, then recover to another complete frame.
    boundary(); pixels(7,NPIX/2); reset_device();
    boundary(); pixels(8,NPIX); expect_frame(8,0,1); boundary();
    random_stalls=1; wait_drain(1);
    repeat(20) @(negedge ifclk);
    if(completed!=3) $fatal(1,"Expected three completed valid frames, got%0d",completed);
    if(FRAME_BYTES%512==0 && packets!=0) $fatal(1,"Exact packet frame emitted PKTEND");
    if(FRAME_BYTES%512!=0 && packets!=3) $fatal(1,"Expected three short-frame PKTENDs, got%0d",packets);
    if(FRAME_BYTES%512!=0 && pktend_stall_cycles<7) $fatal(1,"PACKET_END FLAGC stall coverage missing");
    if(stall_cycles<80) $fatal(1,"Stall coverage missing");
    $fclose(bin_fd); $fclose(hex_fd); $fclose(records_fd);
    $display("PASS completed=%0d bytes=%0d pktend=%0d stalls=%0d pktend_stalls=%0d output=%0dx%0d",completed,wire_words*2,packets,stall_cycles,pktend_stall_cycles,WIDTH,HEIGHT);
    $finish;
  end
  initial begin
    // FLAGC can fall after the guard has selected PACKET_END. It must still
    // prevent the commit at the next IFCLK edge, and permit exactly one pulse
    // when writable again. Exercise this separately from random SEND stalls.
    if(FRAME_BYTES%512!=0) begin
      wait(reset_n);
      wait(dut.state==dut.PACKET_END);
      @(negedge ifclk); random_stalls=0; fx2_flagc=0;
      repeat(7) begin
        @(negedge ifclk);
        if(fx2_pktend_n!==1 || dut.state!=dut.PACKET_END)
          $fatal(1,"PACKET_END committed while FLAGC full");
        pktend_stall_cycles=pktend_stall_cycles+1;
      end
      fx2_flagc=1;
      repeat(2) @(negedge ifclk);
      random_stalls=1;
    end
  end
  initial begin #100000000; $fatal(1,"Global watchdog timeout"); end
endmodule
