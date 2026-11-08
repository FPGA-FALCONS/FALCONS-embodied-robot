`timescale 1ns/1ps
// Behavioral models reproduce the existing IPs' mixed-width, first-word output.
// They are simulation-only; production still uses the original Gowin FIFO IPs.
module wr_data_fifo(input [15:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [12:0] Wnum,output [9:0] Rnum,output Almost_Empty,Almost_Full,
 output [127:0] Q,output Empty,Full);
 reg [15:0] mem[0:255];integer w=0,r=0,k;
 assign Wnum=w-r;assign Rnum=(w-r)/8;assign Empty=w-r<8;assign Full=w-r>=256;
 assign Almost_Empty=Empty;assign Almost_Full=Full;
 genvar i;generate for(i=0;i<8;i=i+1)begin assign Q[i*16+:16]=mem[(r+i)%256];end endgenerate
 always @(posedge WrClk or posedge Reset)if(Reset)w<=0;else if(WrEn && !Full)begin mem[w%256]<=Data;w<=w+1;end
 always @(posedge RdClk or posedge Reset)if(Reset)r<=0;else if(RdEn && !Empty)r<=r+8;
endmodule
module rd_data_fifo(input [127:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [9:0] Wnum,output [12:0] Rnum,output Almost_Empty,Almost_Full,
 output [15:0] Q,output Empty,Full);
 reg [15:0] mem[0:4095];integer w=0,r=0,k;
 assign Wnum=(w-r+7)/8;assign Rnum=w-r;assign Empty=w==r;assign Full=w-r>=4088;
 assign Almost_Empty=Empty;assign Almost_Full=Full;assign Q=mem[r%4096];
 always @(posedge WrClk or posedge Reset)if(Reset)w<=0;else if(WrEn && !Full)begin
  for(k=0;k<8;k=k+1)mem[(w+k)%4096]<=Data[k*16+:16];w<=w+8;end
 always @(posedge RdClk or posedge Reset)if(Reset)r<=0;else if(RdEn && !Empty)r<=r+1;
endmodule
