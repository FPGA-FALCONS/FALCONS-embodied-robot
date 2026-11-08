`timescale 1ns/1ps
module camera_frame_buffer_tb #(parameter FULL_SIZE=0);
 localparam PIXELS=FULL_SIZE ? 1024*600 : 64;
 localparam BLOCKS=PIXELS/8;
 reg ui=0,cam=0,pix=0,rst=0;always #10 ui=~ui;always #(FULL_SIZE ? 9.524 : 19) cam=~cam;
 integer pix_half=FULL_SIZE ? 10 : 11;always begin #(pix_half)pix=~pix;end
 reg sof=0,de=0,tick=0,req=0;reg [15:0] camera_data=0;
 wire want,en,we,wend;wire [2:0] cmd;wire [27:0] addr;wire [127:0] wd;
 wire ready,wready,resp;wire [127:0] resp_data;wire [15:0] pixel;wire full,empty;
 camera_frame_buffer #(.PIXELS(PIXELS),.READ_TARGET(FULL_SIZE ? 128 : 16)) dut(.ui_clk(ui),.rst_n(rst),.init_calib_complete(rst),
 .app_rdy(ready),.app_wdf_rdy(wready),.app_rd_data_valid(resp),.app_rd_data(resp_data),
 .app_addr(addr),.o_request(want),.app_en(en),.app_wdf_wren(we),.app_wdf_end(wend),.app_cmd(cmd),.app_wdf_data(wd),
 .rd_load(tick),.wr_load(sof),.wr_clk(cam),.wfifo_wren(de),.wfifo_din(camera_data),.wrfifo_full(full),
 .rd_clk(pix),.rfifo_rden(req),.rdfifo_empty(empty),.rfifo_dout(pixel));
 wire men,mwe;wire [2:0] mc;wire [27:0] ma;wire [127:0] mw;
 reg mr=0,mwr=0,mrv=0;reg [127:0] mdata=0;wire fault;integer cycles=0;
 reg ai_read=0,ai_write=0;wire ai_rr,ai_wr,ai_rv;
 inspection_ddr_arb arb(.clk(ui),.rst_n(rst),.i_initialized(rst),.i_legacy_want(want),.i_legacy_en(en),
 .i_legacy_cmd(cmd),.i_legacy_address(addr),.i_legacy_write_data(wd),.o_legacy_ready(ready),.o_legacy_write_ready(wready),
 .o_legacy_read_valid(resp),.o_legacy_read_data(resp_data),
 .i_write_valid(ai_write),.i_write_address(28'h100000),.i_write_data(128'h1234),.o_write_ready(ai_wr),
 .i_read_valid(ai_read),.i_read_address(28'h100000),.o_read_ready(ai_rr),.o_read_valid(ai_rv),.o_read_data(),
 .i_memory_ready(mr),.i_memory_write_ready(mwr),.i_memory_read_valid(mrv),.i_memory_read_data(mdata),
 .o_memory_en(men),.o_memory_write_en(mwe),.o_memory_write_end(),.o_memory_cmd(mc),.o_memory_address(ma),
 .o_memory_write_data(mw),.o_protocol_fault(fault));
 reg [127:0] memory[0:3*BLOCKS-1],qdata[0:65535];integer due[0:65535];integer head=0,tail=0;
 integer bank,offset,mi,checks=0,comparisons=0,errors=0,frames=0,good_frames=0,published=0;
 reg invalid_frame[0:255];integer fi,rejected=0;
 initial for(fi=0;fi<256;fi=fi+1)invalid_frame[fi]=0;
 integer short_frame=0;reg stall_write=0,stall_read=0;
 task verify;input ok;input [1023:0] label;begin comparisons=comparisons+1;if(ok!==1'b1)begin errors=errors+1;$display("FAIL: %0s",label);end end endtask
 task check;input ok;input [1023:0] label;begin checks=checks+1;if(ok===1'b1)$display("pass %0s",label);else begin errors=errors+1;$display("FAIL: %0s",label);end end endtask
 always @(negedge ui)begin
  if(!rst)begin cycles=0;mr=0;mwr=0;mrv=0;head=0;tail=0;end
  else begin
   cycles=cycles+1;mr=(cycles%7!=0);mwr=!stall_write && cycles%5!=0;mrv=0;
   ai_read=cycles%13<4;ai_write=cycles%19<3;
   if(!stall_read && head<tail && due[head%65536]<=cycles && cycles%3!=0)begin mrv=1;mdata=qdata[head%65536];head=head+1;end
  end
 end
 always @(posedge ui)if(rst)begin
  if(fault)begin $display("FAIL: DDR owner tag fault");$fatal;end
  if(dut.ws==3 && dut.capture_bad)rejected=rejected+1;
  if(dut.writer_publish)begin
   published=published+1;
   verify(invalid_frame[memory[dut.write_bank*BLOCKS][15:8]]===1'b0,"truncated/overflowed capture is never published");
  end
  if(men)begin
   verify(mr,"every command accepted only when DDR ready");
   if(ma>=28'h200000)begin
    bank=ma[22:20]-2;offset=ma[19:3];mi=bank*BLOCKS+offset;
    verify(bank>=0 && bank<3 && offset<BLOCKS,"camera address stays within its three banks");
    if(mc==0)begin
     verify(!dut.front_valid || dut.write_bank!=dut.front_bank,"writer never overwrites display front bank");
     verify(mwr,"write data ready");memory[mi]=mw;
    end else begin qdata[tail%65536]=memory[mi];due[tail%65536]=cycles+11+(cycles%17);tail=tail+1;end
   end else begin
    verify(ma==28'h100000,"AI memory range remains separate");
    if(mc==1)begin qdata[tail%65536]=128'hdead;due[tail%65536]=cycles+11;tail=tail+1;end
   end
  end
 end
 integer camera_frame=0,k,limit;
 initial begin
  wait(rst);
  forever begin
   @(negedge cam);sof=1;de=0;
   repeat(40)@(negedge cam);sof=0;
   camera_frame=camera_frame+1;limit=PIXELS;
   if(short_frame)begin limit=40;short_frame=0;invalid_frame[camera_frame%256]=1;end
   for(k=0;k<limit;k=k+1)begin
    @(negedge cam);de=1;camera_data={camera_frame[7:0],k[7:0]};
    @(negedge cam);de=0;
   end
   repeat(FULL_SIZE ? 1000 : 30)@(negedge cam);
  end
 end
 integer n,frame_id,valid_pixels,blanks;reg started;
 initial begin
  wait(rst);
  forever begin
   @(negedge pix);tick=1;req=0;
   repeat(FULL_SIZE ? 2000 : 160)@(negedge pix);tick=0;
   frames=frames+1;started=0;valid_pixels=0;blanks=0;frame_id=0;
   for(n=0;n<PIXELS;n=n+1)begin
    req=1;@(posedge pix);
    if(pixel!=0)begin
     if(!started)begin started=1;frame_id=pixel[15:8];end
     verify(pixel[15:8]==frame_id,"all displayed pixels belong to one capture frame");
     verify(pixel[7:0]==n[7:0],"no dropped/repeated/shifted pixels within a displayed frame");valid_pixels=valid_pixels+1;
    end else blanks=blanks+1;
    @(negedge pix);
   end
   req=0;if(valid_pixels==PIXELS)good_frames=good_frames+1;
   if(!stall_read && started)verify(valid_pixels==PIXELS,"normal display frame is complete");
   repeat(40)@(negedge pix);
  end
 end
 integer saved,first_good;
 initial begin
  repeat(10)@(negedge ui);rst=1;
  if(FULL_SIZE)begin
   wait(frames>=12);@(negedge ui);
   check(good_frames>=3,"full 1024x600 frames retain pixel order under shared DDR traffic");
   check(published>=2,"full-resolution capture writes all 76800 blocks before publishing");
   check(errors==0,"all full-resolution pixel and bank ownership comparisons pass");
   $display("COMPARISONS: %0d",comparisons);$display("CHECKS: %0d",checks);
   if(errors==0)$display("PASS: camera_frame_buffer_full");else $display("FAIL: full frame errors=%0d",errors);
   $finish;
  end
  wait(frames>=20);check(good_frames>10,"startup and faster display repeat complete frames");
  pix_half=47;first_good=good_frames;
  wait(frames>=40);check(good_frames-first_good>=18,"faster camera replaces pending frames without tearing");
  wait(dut.waiting_frame);short_frame=1;repeat(1500)@(negedge ui);
  check(rejected>=1,"short capture is explicitly discarded");
  check(good_frames>first_good,"incomplete capture does not corrupt the front frame");
  wait(dut.capturing && dut.pixel_count==16);
  invalid_frame[camera_frame%256]=1;
  force dut.wrfifo_full=1'b1;repeat(8)@(negedge cam);release dut.wrfifo_full;
  repeat(1500)@(negedge ui);
  check(rejected>=2,"overflowed capture is explicitly discarded");
  check(!dut.display_bad,"capture overflow recovery preserves displayed pixel alignment");
  // Leave accepted reads in flight across a display ticket. The front bank
  // must remain owned until all tagged responses have returned.
  wait(dut.outstanding>0);stall_read=1;saved=dut.front_bank;
  wait(dut.rs==1);repeat(40)@(negedge ui);
  check(dut.front_bank==saved,"old bank retained while responses are outstanding");
  // A response arriving during active scan must not cause a mid-frame switch.
  wait(!dut.blank_window);repeat(10)@(negedge ui);stall_read=0;
  repeat(50)@(negedge ui);
  check(dut.front_bank==saved,"late read responses cannot switch a frame during active scan");
  wait(frames>=60);check(published>10,"complete captured frames published repeatedly");
  check(!fault,"shared AI/camera DDR tag ownership remains coherent");
  rst=0;repeat(20)@(negedge ui);rst=1;
  repeat(5000)@(negedge ui);check(dut.front_valid,"reset restarts capture and publication");
  check(errors==0,"all asynchronous pixel and ownership comparisons pass");
  $display("COMPARISONS: %0d",comparisons);$display("CHECKS: %0d",checks);
  if(errors==0)$display("PASS: camera_frame_buffer %0d checks",checks);
  else $display("FAIL: camera_frame_buffer errors=%0d",errors);
  $finish;
 end
 initial begin #(FULL_SIZE ? 500000000 : 2000000);$display("FAIL: test timeout");$finish;end
endmodule

module camera_frame_buffer_full_tb; camera_frame_buffer_tb #(.FULL_SIZE(1)) test(); endmodule
