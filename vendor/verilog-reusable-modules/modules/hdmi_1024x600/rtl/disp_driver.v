/*============================================================================
*
*  LOGIC CORE:          显示设备驱动模块		
*  MODULE NAME:         disp_driver()
*  COMPANY:             武汉芯路恒科技有限公司
*                       http://xiaomeige.taobao.com
*  author:              小梅哥
*  Website:             www.corecourse.cn
*  REVISION HISTORY:  
*
*  Revision 1.0  04/10/2019
*  Description: Initial Release.
*
*  FUNCTIONAL DESCRIPTION:
===========================================================================*/
`include "disp_parameter_cfg.vh"

module disp_driver(
	ClkDisp,
	Rst_n,
	
	Data,
	DataReq,
	
	H_Addr,
	V_Addr,
	
	Disp_HS,
	Disp_VS,
	Disp_Red,
	Disp_Green,
	Disp_Blue,
    Frame_Begin,
	Disp_DE,
	Disp_PCLK
);

	input ClkDisp;
	input Rst_n;
	input [`HDMI_1024X600_RED_BITS + `HDMI_1024X600_GREEN_BITS + `HDMI_1024X600_BLUE_BITS - 1:0] Data;
	output DataReq;
	
	output [11:0] H_Addr;
	output [11:0] V_Addr;
	
	output reg Disp_HS;
	output reg Disp_VS;

	output reg [`HDMI_1024X600_RED_BITS - 1 :0]Disp_Red;
	output reg [`HDMI_1024X600_GREEN_BITS - 1 :0]Disp_Green;
	output reg [`HDMI_1024X600_BLUE_BITS - 1 :0]Disp_Blue;
	
    output reg Frame_Begin; //一帧图像起始标识信号，ClkDisp时钟域
	output reg Disp_DE;
	output Disp_PCLK;
	
	wire hcount_ov;
	wire vcount_ov;
	
	//----------------内部寄存器定义----------------
	reg [11:0] hcount_r;     //行扫描计数器
	reg [11:0] vcount_r;     //场扫描计数器

	assign Disp_PCLK = ClkDisp;

	assign DataReq = Disp_DE;
	
	parameter hdat_begin = `HDMI_1024X600_H_SYNC + `HDMI_1024X600_H_BACK_PORCH + `HDMI_1024X600_H_LEFT_BORDER - 1'b1;
	parameter hdat_end = `HDMI_1024X600_H_TOTAL - `HDMI_1024X600_H_RIGHT_BORDER - `HDMI_1024X600_H_FRONT_PORCH - 1'b1;
	
	parameter vdat_begin = `HDMI_1024X600_V_SYNC + `HDMI_1024X600_V_BACK_PORCH + `HDMI_1024X600_V_TOP_BORDER - 1'b1;
	parameter vdat_end = `HDMI_1024X600_V_TOTAL - `HDMI_1024X600_V_BOTTOM_BORDER - `HDMI_1024X600_V_FRONT_PORCH - 1'b1;
	
	// Disp_DE is registered from the previous hcount_r value, while hcount_r
	// has already advanced when the consumer observes DataReq. Compensate that
	// pipeline stage so the active-line address is 0 through 1023.
	assign H_Addr = Disp_DE?(hcount_r - hdat_begin - 1'b1):12'd0;
	assign V_Addr = Disp_DE?(vcount_r - vdat_begin):12'd0;

	//行扫描
	assign hcount_ov = (hcount_r >= `HDMI_1024X600_H_TOTAL - 1);
	
	always@(posedge ClkDisp or negedge Rst_n)
	if(!Rst_n)
		hcount_r <= 0;
	else if(hcount_ov)
		hcount_r <= 0;
	else
		hcount_r <= hcount_r + 1'b1;

	//场扫描
	assign vcount_ov = (vcount_r >= `HDMI_1024X600_V_TOTAL - 1);

	always@(posedge ClkDisp or negedge Rst_n)
	if(!Rst_n)
		vcount_r <= 0;
	else if(hcount_ov) begin
		if(vcount_ov)
			vcount_r <= 0;
		else
			vcount_r <= vcount_r + 1'd1;
	end
	else 
		vcount_r <= vcount_r;
	
	always@(posedge ClkDisp)
		Disp_DE <= ((hcount_r >= hdat_begin)&&(hcount_r < hdat_end))
					&&((vcount_r >= vdat_begin)&&(vcount_r < vdat_end));
	
	always@(posedge ClkDisp)	begin			
		Disp_HS <= (hcount_r > `HDMI_1024X600_H_SYNC - 1);
		Disp_VS <= (vcount_r > `HDMI_1024X600_V_SYNC - 1);
		{Disp_Red,Disp_Green,Disp_Blue} <= (Disp_DE)?Data:1'd0;
	end
	
  /*******************************************/
  reg Disp_VS_dly1;
  always@(posedge ClkDisp)
  begin
    Disp_VS_dly1 <= Disp_VS;
  end

  always@(posedge ClkDisp or negedge Rst_n)
  begin
    if(!Rst_n)
      Frame_Begin <= 1'b0;
    else if(!Disp_VS_dly1 && Disp_VS)
      Frame_Begin <= 1'b1;
    else
      Frame_Begin <= 1'b0;
  end
  /*******************************************/

endmodule
