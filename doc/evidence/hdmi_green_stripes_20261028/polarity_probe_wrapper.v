module hdmi_1024x600(
    input             clk_50m,
    input             reset_n,
    input             vs_positive,
    input      [23:0] pixel_rgb888,

    output            pixel_clk,
    output            pixel_req,
    output     [11:0] pixel_x,
    output     [11:0] pixel_y,
    output     [11:0] pixel_x_ui, pixel_y_ui,
    output     [11:0] pixel_x_ai, pixel_y_ai,
    output            frame_start,
    output            frame_sync,
    output            video_active,
    output            pll_lock,

    output            tmds_clk_p,
    output            tmds_clk_n,
    output     [2:0]  tmds_data_p,
    output     [2:0]  tmds_data_n
);

    wire clk_pixel;
    wire clk_serial_5x;
    wire video_hs;
    wire video_vs;
    wire [7:0] video_r;
    wire [7:0] video_g;
    wire [7:0] video_b;

    assign pixel_clk   = clk_pixel;
    assign frame_sync  = video_vs;

    display_pll u_display_pll (
        .lock    (pll_lock),
        .clkout0 (clk_pixel),
        .clkout1 (clk_serial_5x),
        .clkin   (clk_50m)
    );

    disp_driver u_disp_driver (
        .ClkDisp     (clk_pixel),
        .Rst_n       (reset_n & pll_lock),
        .Data        (pixel_rgb888),
        .DataReq     (pixel_req),
        .H_Addr      (pixel_x),
        .V_Addr      (pixel_y),
        .H_Addr_UI   (pixel_x_ui), .V_Addr_UI(pixel_y_ui),
        .H_Addr_AI   (pixel_x_ai), .V_Addr_AI(pixel_y_ai),
        .Disp_HS     (video_hs),
        .Disp_VS     (video_vs),
        .Disp_Red    (video_r),
        .Disp_Green  (video_g),
        .Disp_Blue   (video_b),
        .Frame_Begin (frame_start),
        .Disp_DE     (video_active),
        .Disp_PCLK   ()
    );

    DVI_TX_Top u_dvi_tx (
        .I_rst_n       (reset_n & pll_lock),
        .I_serial_clk  (clk_serial_5x),
        .I_rgb_clk     (clk_pixel),
        .I_rgb_vs      (vs_positive ? ~video_vs : video_vs),
        .I_rgb_hs      (video_hs),
        .I_rgb_de      (video_active),
        .I_rgb_r       (video_r),
        .I_rgb_g       (video_g),
        .I_rgb_b       (video_b),
        .O_tmds_clk_p  (tmds_clk_p),
        .O_tmds_clk_n  (tmds_clk_n),
        .O_tmds_data_p (tmds_data_p),
        .O_tmds_data_n (tmds_data_n)
    );

endmodule
