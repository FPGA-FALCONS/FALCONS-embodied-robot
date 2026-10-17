`timescale 1ns/1ps
module tb_rc_6ch_led;
reg clk=0; always #125 clk=~clk;
reg rst_n=0, sw0=1;
reg [5:0] pwm=0;
wire ds,sh_cp,st_cp; wire [7:0] led,low_led;
wire low_ds,low_sh,low_st;
rc_6ch_led_top #(.CLK_HZ(4000000),.TIMEOUT_US(12000),.CHANNEL_US(10000),.DIGIT_US(100),.SNAPSHOT_US(100),.SCLK_DIVIDER(2),.HEARTBEAT_US(1000)) dut
(.clk(clk),.rst_n(rst_n),.sw0(sw0),.rc_pwm(pwm),.ds(ds),.sh_cp(sh_cp),.st_cp(st_cp),.led(led));
rc_6ch_led_top #(.CLK_HZ(4000000),.TIMEOUT_US(12000),.CHANNEL_US(10000),.DIGIT_US(100),.SNAPSHOT_US(100),.SCLK_DIVIDER(2),.HEARTBEAT_US(1000),.CH5_ACTIVE_LOW(1),.CH6_ACTIVE_LOW(1)) low_dut
(.clk(clk),.rst_n(rst_n),.sw0(sw0),.rc_pwm(pwm),.ds(low_ds),.sh_cp(low_sh),.st_cp(low_st),.led(low_led));
integer checks=0,words=0,bits_seen=0;
reg [15:0] shifted=0; reg [7:0] segments[0:7]; reg [7:0] seen=0;
integer d;
always @(posedge sh_cp) begin shifted={shifted[14:0],ds};bits_seen=bits_seen+1;end
always @(posedge st_cp) begin
 if(rst_n) begin
  if(bits_seen!=16) $fatal(1,"595 bit count %0d",bits_seen);
  if(shifted[7:0]==0 || ((shifted[7:0] & (shifted[7:0]-1))!=0)) $fatal(1,"bad digit select");
  for(d=0;d<8;d=d+1) if(shifted[d]) begin segments[d]=shifted[15:8];seen[d]=1;end
  words=words+1;
 end
 bits_seen=0;
end
task check(input bit ok,input string label_text); begin checks++;if(!ok)$fatal(1,"FAIL %s high=%h low=%h",label_text,led,low_led);end endtask
task pulse(input integer channel,input integer us);begin
 @(negedge clk);pwm[channel]=1;#(us*1000);pwm[channel]=0;#3000;
end endtask
task both(input integer us);begin
 @(negedge clk);pwm[5:4]=3;#(us*1000);pwm[5:4]=0;#3000;
end endtask
task buttons(input [1:0] hi,input [1:0] lo,input string label_text);begin check(led[5:4]===hi,label_text);check(low_led[5:4]===lo,label_text);end endtask
initial begin
 #2000;check(led===0 && low_led===0,"reset clears all");rst_n=1;#3000;
 pulse(0,1500);pulse(1,1500);pulse(2,1500);pulse(3,1500);
 check(led[3:0]===4'hf,"four axes valid");check(led[6]===0,"center direction clear");
 #1200000;check(seen===8'hff,"all physical digits latched");
 check(segments[0]==8'hc0 && segments[1]==8'hc0 && segments[2]==8'h92 && segments[3]==8'hf9,"physical right 1500");
 check(segments[4]==8'hf9 && segments[5]==8'hc0 && segments[6]==8'hc0 && segments[7]==8'hc0,"physical left 0001");
 both(1500);buttons(0,0,"center off");
 pulse(4,2000);buttons(1,0,"CH5 alone pressed");
 pulse(5,2000);buttons(3,0,"both pressed");
 pulse(4,1500);buttons(2,0,"CH5 release independent");
 pulse(5,1500);buttons(0,0,"both release");
 both(1750);buttons(0,0,"1750 strict off");both(1751);buttons(3,0,"1751 on");
 both(1250);buttons(0,0,"1250 low strict off");both(1249);buttons(0,3,"1249 low on");
 both(1000);buttons(0,3,"1000 low on");
 both(500);buttons(0,0,"illegal short clears");
 both(2000);buttons(3,0,"recover after illegal");both(2300);buttons(0,0,"overlong clears");
 both(2000);buttons(3,0,"recover after overlong");#12010000;buttons(0,0,"timeout clears");
 both(1000);buttons(0,3,"recover after timeout");
 rst_n=0;#2000;buttons(0,0,"reset pressed clears");pwm=0;rst_n=1;#3000;
 both(2000);buttons(3,0,"recover after reset");
 pulse(0,2000);check(led[0] && led[6],"axis direction unaffected");
 // Freeze keeps channel 1; release SW0 enables rotation to channel 2.
 pulse(0,1500);#1000000;check(segments[4]==8'hf9,"freeze preserves channel1");
 sw0=0;#11000000;check(segments[4]==8'ha4,"automatic channel2 display");
 check(words>100,"physical serial traffic continued");
 $display("PASS tb_rc_6ch_led checks=%0d words=%0d",checks,words);$finish;
end
initial begin #150000000;$fatal(1,"watchdog");end
endmodule
