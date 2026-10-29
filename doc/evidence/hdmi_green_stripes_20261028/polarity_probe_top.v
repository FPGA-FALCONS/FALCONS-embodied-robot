`timescale 1ns/1ps
// Standalone display diagnosis; no motor, DDR, camera or RK state machine.
module hdmi_polarity_probe (
    input wire i_clk_50m,
    input wire i_key4_n,
    input wire i_key3_n,
    output wire o_tmds_clk_p,o_tmds_clk_n,
    output wire [2:0] o_tmds_data_p,o_tmds_data_n,
    output wire o_stby,
    output wire [3:0] o_motor_in1,o_motor_in2,o_motor_pwm,
    output wire o_uart_tx,
    output wire [7:0] o_led,
    output wire camera_pwdn,camera_rst_n,camera_sclk,
    inout wire camera_sdat,
    output wire o_fx2_slrd_n,o_fx2_slwr_n,o_fx2_sloe_n,o_fx2_pktend_n,o_fx2_slcs_n,o_fx2_pa0
);
    assign o_stby=0;
    assign o_motor_in1=0,o_motor_in2=0,o_motor_pwm=0;
    assign o_uart_tx=1;
    assign camera_pwdn=1,camera_rst_n=0,camera_sclk=1;
    assign camera_sdat=1'bz;
    assign o_fx2_slrd_n=1,o_fx2_slwr_n=1,o_fx2_sloe_n=1,o_fx2_pktend_n=1,o_fx2_slcs_n=1,o_fx2_pa0=0;
    wire pixel_clk,pll_lock,de;
    wire [11:0] x,y;
    reg lock1=0,lock2=0;
    reg [3:0] power_count=0;
    always @(posedge pixel_clk)begin
        lock1<=pll_lock;lock2<=lock1;
        if(!lock2)power_count<=0;
        else if(power_count!=15)power_count<=power_count+1'b1;
    end
    reg [1:0] key_sync=2'b11;
    reg frozen=0;
    reg [1:0] polarity_key=2'b11;
    reg vs_positive=0;
    wire frame_start;
    always @(posedge pixel_clk)begin
        key_sync<={key_sync[0],i_key4_n};
        polarity_key<={polarity_key[0],i_key3_n};
        if(frame_start)begin frozen<=!key_sync[1];vs_positive<=!polarity_key[1];end
    end
    reg text_row;
    integer row;
    always @*begin
        text_row=0;
        for(row=0;row<7;row=row+1)
            if(y>=48+row*72 && y<72+row*72)text_row=1;
    end
    wire stroke=text_row && x>=24 && x<192 && (x[3:0]<3 || y[3:0]<2);
    reg [23:0] color;
    always @*begin
        if(x<208)begin
            color=frozen?24'h0b1019:24'h101828;
            if(y>=468 && y<516)color=24'h1d4ed8;
            if(stroke)color=frozen?24'h5a6170:24'he5e7eb;
        end else color=(y>=60 && y<112)||(y>=120 && y<540)?24'h101820:24'h000000;
    end
    assign o_led={4'b1010,1'b0,vs_positive,frozen,pll_lock};
    hdmi_1024x600 hdmi(
        .clk_50m(i_clk_50m),.vs_positive(vs_positive),.reset_n(power_count[3]),.pixel_rgb888(color),
        .pixel_clk(pixel_clk),.pixel_req(),.pixel_x(x),.pixel_y(y),
        .pixel_x_ui(),.pixel_y_ui(),.pixel_x_ai(),.pixel_y_ai(),
        .frame_start(frame_start),.frame_sync(),.video_active(de),.pll_lock(pll_lock),
        .tmds_clk_p(o_tmds_clk_p),.tmds_clk_n(o_tmds_clk_n),
        .tmds_data_p(o_tmds_data_p),.tmds_data_n(o_tmds_data_n));
endmodule
