`timescale 1ns/1ps
module tb_camera_fx2_live;
  parameter integer SOURCE_WIDTH=8, SOURCE_HEIGHT=8;
  parameter integer FULL_TEST=0, LINE_CYCLES=100, SERVICE_WORDS=2500;
  localparam integer NPIX=SOURCE_WIDTH*SOURCE_HEIGHT;
  reg cam_clk=0,ifclk=0;
  // 52.5 MHz pixel clock and an independent 48 MHz FX2 IFCLK.
  always #9.524 cam_clk=~cam_clk;
  always #10.417 ifclk=~ifclk;
  reg reset_n=0,cam_vsync=0,cam_valid=0,fx2_flagc=0;
  reg [15:0] cam_rgb565=0;
  wire [15:0] fx2_fd;
  wire [1:0] fx2_faddr;
  wire fx2_slwr_n,fx2_sloe_n,fx2_slrd_n,fx2_pktend_n,capture_busy;
  wire [31:0] frames_sent,frames_dropped,frames_malformed;
  camera_fx2_live #(.SOURCE_WIDTH(SOURCE_WIDTH),.SOURCE_HEIGHT(SOURCE_HEIGHT),.LINE_BANKS(4),.DESC_DEPTH(16)) dut (.*);
  integer bin_fd,manifest_fd,phase=0,words=0,commits=0;
  integer run_words=0,max_run_words=0,full_cycles=0;
  integer forced_full=0,random_mode=0;
  reg [31:0] rng=32'h9e3779b9;
  reg previous_commit=0;
  realtime first_write_ns=0,last_write_ns=0,test_start_ns;
  string out_dir;

  always @(negedge ifclk) begin
    phase=(phase+1)%6000;
    rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
    if(!reset_n || forced_full) fx2_flagc=0;
    else if(FULL_TEST) fx2_flagc=(phase<SERVICE_WORDS);
    else if(random_mode) fx2_flagc=(rng[3:0]>3);
    else fx2_flagc=1;
  end
  always @(posedge ifclk) begin
    if(reset_n) begin
      if(fx2_faddr!==2'b10 || fx2_sloe_n!==1 || fx2_slrd_n!==1) $fatal(1,"FX2 direction/endpoint");
      if(!fx2_flagc && (!fx2_slwr_n || !fx2_pktend_n)) $fatal(1,"FX2 write/commit while full");
      if(!fx2_flagc) full_cycles=full_cycles+1;
      if(!fx2_slwr_n) begin
        $fwrite(bin_fd,"%c%c",fx2_fd[7:0],fx2_fd[15:8]);
        words=words+1; run_words=run_words+1;
        if(run_words>max_run_words) max_run_words=run_words;
        if(first_write_ns==0) first_write_ns=$realtime;
        last_write_ns=$realtime;
      end else run_words=0;
      if(!fx2_pktend_n) begin
        if(previous_commit || !fx2_slwr_n) $fatal(1,"PKTEND pulse/SLWR overlap");
        commits=commits+1;
      end
      previous_commit=!fx2_pktend_n;
    end else begin previous_commit=0; run_words=0; end
  end

  task automatic reset_device;
    begin
      @(negedge cam_clk); reset_n=0; cam_valid=0; cam_vsync=0;
      repeat(15) @(negedge cam_clk);
      reset_n=1;
      repeat(15) @(negedge cam_clk);
    end
  endtask
  task automatic boundary;
    begin
      @(negedge cam_clk); cam_vsync=1; cam_valid=1; cam_rgb565=16'hdead;
      @(negedge cam_clk); cam_vsync=0; cam_valid=0;
      repeat(6) @(negedge cam_clk);
    end
  endtask
  task automatic frame_pixels(input integer tag,input integer count);
    integer i;
    begin
      for(i=0;i<count;i=i+1) begin
        @(negedge cam_clk); cam_valid=1; cam_rgb565=(tag*257+i)&65535;
        @(negedge cam_clk); cam_valid=0;
        if(i%SOURCE_WIDTH==SOURCE_WIDTH-1)
          repeat(LINE_CYCLES-2*SOURCE_WIDTH) @(negedge cam_clk);
      end
      @(negedge cam_clk); cam_valid=0;
    end
  endtask
  task automatic idle_cycles(input integer n);
    begin repeat(n) @(negedge cam_clk); end
  endtask
  task automatic expected(input integer frameid,input integer tag);
    begin $fdisplay(manifest_fd,"%0d %0d %0d %0d",frameid,tag,SOURCE_WIDTH,SOURCE_HEIGHT); end
  endtask
  task automatic drain(input integer sent);
    integer timeout;
    begin
      timeout=0;
      while(frames_sent<sent && timeout<2000000) begin @(negedge ifclk); timeout=timeout+1; end
      if(timeout==2000000) $fatal(1,"Drain timeout sent=%0d target=%0d dropped=%0d malformed=%0d",frames_sent,sent,frames_dropped,frames_malformed);
      idle_cycles(100);
    end
  endtask
  integer f,before_words;
  initial begin
    if(!$value$plusargs("OUT_DIR=%s",out_dir)) out_dir=".";
    bin_fd=$fopen({out_dir,"/wire.bin"},"wb");
    manifest_fd=$fopen({out_dir,"/expected_frames.txt"},"w");
    if(!bin_fd || !manifest_fd) $fatal(1,"Cannot open evidence");
    reset_device(); test_start_ns=$realtime;
    if(FULL_TEST) begin
      boundary();
      for(f=0;f<3;f=f+1) begin
        frame_pixels(f+1,NPIX);
        idle_cycles(30*LINE_CYCLES);
        expected(f,f+1); boundary();
      end
      drain(3);
      if(frames_dropped!=0 || frames_malformed!=0) $fatal(1,"Sustained target lost frames dropped=%0d malformed=%0d",frames_dropped,frames_malformed);
      if(max_run_words<SOURCE_WIDTH) $fatal(1,"Payload not continuous one word per IFCLK, max=%0d",max_run_words);
    end else begin
      random_mode=1;
      // Ignore incomplete startup pixels, then exact frame0.
      frame_pixels(99,NPIX/2); boundary(); frame_pixels(1,NPIX); expected(0,1); boundary(); drain(1);
      // Frame1 truncated and frame2 overlong must ABORT, never END.
      frame_pixels(2,NPIX-1); boundary();
      frame_pixels(3,NPIX+1); boundary(); idle_cycles(2000);
      if(frames_malformed<2) $fatal(1,"Malformed counter missing");
      // Frame3 suffers a long USB pause exceeding four line banks.
      forced_full=1; before_words=words;
      frame_pixels(4,NPIX); boundary(); idle_cycles(1000);
      if(words!=before_words) $fatal(1,"Long USB pause wrote data");
      forced_full=0; idle_cycles(3000);
      if(frames_dropped==0) $fatal(1,"Long USB pause did not abort");
      // Frame4 entered while full, so start afresh at frame5 after drain.
      boundary(); frame_pixels(5,NPIX); expected(5,5); boundary(); drain(2);
      // Reset discards a partial camera frame, then frame0 recovers.
      frame_pixels(6,NPIX/2); reset_device();
      boundary(); frame_pixels(7,NPIX); expected(0,7); boundary(); drain(1);
      // Reset during a queued transport: inject partial frame8, then reset.
      forced_full=1; frame_pixels(8,NPIX/2); forced_full=0;
      wait(!fx2_slwr_n); reset_device();
      boundary(); frame_pixels(9,NPIX); expected(0,9); boundary(); drain(1);
      idle_cycles(1000);
    end
    $fclose(bin_fd); $fclose(manifest_fd);
    $display("PASS live words=%0d bytes=%0d commits=%0d max_burst_words=%0d full_cycles=%0d elapsed_ns=%0.3f first_write_ns=%0.3f last_write_ns=%0.3f sent=%0d dropped=%0d malformed=%0d",words,words*2,commits,max_run_words,full_cycles,$realtime-test_start_ns,first_write_ns,last_write_ns,frames_sent,frames_dropped,frames_malformed);
    $finish;
  end
  initial begin #1000000000; $fatal(1,"Global watchdog"); end
endmodule
