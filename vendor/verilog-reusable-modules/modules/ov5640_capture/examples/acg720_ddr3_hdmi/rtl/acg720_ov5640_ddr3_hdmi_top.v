module acg720_ov5640_ddr3_hdmi_top #(
    parameter integer IMAGE_WIDTH  = 1024,
    parameter integer IMAGE_HEIGHT = 600
)(
    input             clk50M,
    input             reset_n,

    output            camera_sclk,
    inout             camera_sdat,
    input             camera_vsync,
    input             camera_href,
    input             camera_pclk,
    input      [7:0]  camera_data,
    output            camera_rst_n,
    output            camera_pwdn,

    output     [3:0]  led,

    output            tmds_clk_p,
    output            tmds_clk_n,
    output     [2:0]  tmds_data_p,
    output     [2:0]  tmds_data_n,

    output     [13:0] O_ddr_addr,
    output     [2:0]  O_ddr_ba,
    output            O_ddr_cs_n,
    output            O_ddr_ras_n,
    output            O_ddr_cas_n,
    output            O_ddr_we_n,
    output            O_ddr_clk,
    output            O_ddr_clk_n,
    output            O_ddr_cke,
    output            O_ddr_odt,
    output            O_ddr_reset_n,
    output     [1:0]  O_ddr_dqm,
    inout      [15:0] IO_ddr_dq,
    inout      [1:0]  IO_ddr_dqs,
    inout      [1:0]  IO_ddr_dqs_n
);

    localparam [27:0] APP_ADDR_MAX = IMAGE_WIDTH * IMAGE_HEIGHT;
    localparam [7:0]  BURST_LEN    = IMAGE_WIDTH / 8;

    wire clk_200m;
    wire clk_50m;
    wire system_pll_lock;

    wire camera_init_done;
    wire camera_pixel_clk;
    wire camera_capture_clear;
    wire camera_pixel_valid;
    wire [15:0] camera_pixel_rgb565;
    wire camera_frame_sync;

    wire hdmi_pixel_clk;
    wire hdmi_pixel_req;
    wire hdmi_frame_sync;
    wire hdmi_pll_lock;
    wire [23:0] hdmi_pixel_rgb888;

    wire ddr_init_done;
    wire [15:0] ddr_pixel_rgb565;

    assign hdmi_pixel_rgb888 = {
        ddr_pixel_rgb565[15:11], 3'b000,
        ddr_pixel_rgb565[10:5],  2'b00,
        ddr_pixel_rgb565[4:0],   3'b000
    };

    assign led = {
        hdmi_pll_lock,
        camera_init_done,
        ddr_init_done,
        system_pll_lock
    };

    Gowin_PLL u_system_pll (
        .lock    (system_pll_lock),
        .clkout0 (clk_200m),
        .clkout1 (clk_50m),
        .clkin   (clk50M),
        .reset   (~reset_n)
    );

    ov5640_capture #(
        .IMAGE_WIDTH  (IMAGE_WIDTH),
        .IMAGE_HEIGHT (IMAGE_HEIGHT)
    ) u_camera (
        .cfg_clk       (clk_50m),
        .reset_n       (reset_n & system_pll_lock),
        .camera_sclk   (camera_sclk),
        .camera_sdat   (camera_sdat),
        .camera_vsync  (camera_vsync),
        .camera_href   (camera_href),
        .camera_pclk   (camera_pclk),
        .camera_data   (camera_data),
        .camera_rst_n  (camera_rst_n),
        .camera_pwdn   (camera_pwdn),
        .init_done     (camera_init_done),
        .init_error    (),
        .pixel_clk     (camera_pixel_clk),
        .capture_clear (camera_capture_clear),
        .pixel_valid   (camera_pixel_valid),
        .pixel_rgb565  (camera_pixel_rgb565),
        .line_valid    (),
        .frame_sync    (camera_frame_sync),
        .pixel_x       (),
        .pixel_y       ()
    );

    ddr3_ctrl_2port u_frame_buffer (
        .clk                 (clk_50m),
        .pll_lock            (system_pll_lock),
        .clk_200m            (clk_200m),
        .sys_rst_n           (reset_n),
        .init_calib_complete (ddr_init_done),

        .rd_load             (hdmi_frame_sync),
        .wr_load             (camera_frame_sync),
        .app_addr_rd_min     (28'd0),
        .app_addr_rd_max     (APP_ADDR_MAX),
        .rd_bust_len         (BURST_LEN),
        .app_addr_wr_min     (28'd0),
        .app_addr_wr_max     (APP_ADDR_MAX),
        .wr_bust_len         (BURST_LEN),

        .wr_clk              (camera_pixel_clk),
        .wfifo_wren          (camera_pixel_valid),
        .wfifo_din           (camera_pixel_rgb565),
        .rd_clk              (hdmi_pixel_clk),
        .rfifo_rden          (hdmi_pixel_req),
        .rfifo_dout          (ddr_pixel_rgb565),

        .ddr3_dq             (IO_ddr_dq),
        .ddr3_dqs_n          (IO_ddr_dqs_n),
        .ddr3_dqs_p          (IO_ddr_dqs),
        .ddr3_addr           (O_ddr_addr),
        .ddr3_ba             (O_ddr_ba),
        .ddr3_ras_n          (O_ddr_ras_n),
        .ddr3_cas_n          (O_ddr_cas_n),
        .ddr3_we_n           (O_ddr_we_n),
        .ddr3_reset_n        (O_ddr_reset_n),
        .ddr3_ck_p           (O_ddr_clk),
        .ddr3_ck_n           (O_ddr_clk_n),
        .ddr3_cke            (O_ddr_cke),
        .ddr3_cs_n           (O_ddr_cs_n),
        .ddr3_dm             (O_ddr_dqm),
        .ddr3_odt            (O_ddr_odt)
    );

    hdmi_1024x600 u_hdmi (
        .clk_50m       (clk_50m),
        .reset_n       (reset_n & system_pll_lock),
        .pixel_rgb888  (hdmi_pixel_rgb888),
        .pixel_clk     (hdmi_pixel_clk),
        .pixel_req     (hdmi_pixel_req),
        .pixel_x       (),
        .pixel_y       (),
        .frame_start   (),
        .frame_sync    (hdmi_frame_sync),
        .video_active  (),
        .pll_lock      (hdmi_pll_lock),
        .tmds_clk_p    (tmds_clk_p),
        .tmds_clk_n    (tmds_clk_n),
        .tmds_data_p   (tmds_data_p),
        .tmds_data_n   (tmds_data_n)
    );

endmodule
