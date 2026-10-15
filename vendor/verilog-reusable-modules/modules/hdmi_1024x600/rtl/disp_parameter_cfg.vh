/*============================================================================
*
*  Original display timing source:
*    Wuhan CoreCourse Technology / XiaoMeiGe, 2019
*
*  This maintained configuration keeps only the hardware-tested MPI7002
*  1024x600 timing used by hdmi_1024x600.
*
===========================================================================*/
`ifndef HDMI_1024X600_DISP_PARAMETER_CFG_VH
`define HDMI_1024X600_DISP_PARAMETER_CFG_VH

`define HDMI_1024X600_RED_BITS       8
`define HDMI_1024X600_GREEN_BITS     8
`define HDMI_1024X600_BLUE_BITS      8

// MPI7002 1024x600 timing, 49 MHz pixel clock.
`define HDMI_1024X600_H_TOTAL        12'd1312
`define HDMI_1024X600_H_RIGHT_BORDER 12'd0
`define HDMI_1024X600_H_FRONT_PORCH  12'd5
`define HDMI_1024X600_H_SYNC         12'd13
`define HDMI_1024X600_H_BACK_PORCH   12'd270
`define HDMI_1024X600_H_LEFT_BORDER  12'd0

`define HDMI_1024X600_V_TOTAL        12'd622
`define HDMI_1024X600_V_BOTTOM_BORDER 12'd0
`define HDMI_1024X600_V_FRONT_PORCH  12'd2
`define HDMI_1024X600_V_SYNC         12'd3
`define HDMI_1024X600_V_BACK_PORCH   12'd17
`define HDMI_1024X600_V_TOP_BORDER   12'd0

`endif
