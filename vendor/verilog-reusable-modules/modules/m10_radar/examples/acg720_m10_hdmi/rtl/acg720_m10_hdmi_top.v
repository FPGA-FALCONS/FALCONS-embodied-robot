// ACG720 / GW5AT-LV138PG484AC1/I0, 50 MHz board oscillator.
module acg720_m10_hdmi_top #(
    parameter integer REFRESH_FRAMES = 12
)(
    input wire i_clock,
    input wire i_reset_n,
    input wire i_radar_rx,
    input wire i_zoom_in_n,
    input wire i_zoom_out_n,
    output wire o_error_led,
    output wire o_debug_uart_tx,
    output wire tmds_clk_p,
    output wire tmds_clk_n,
    output wire [2:0] tmds_data_p,
    output wire [2:0] tmds_data_n
);
    wire pixel_clk, serial_clk, pll_lock;
    wire async_reset_n = i_reset_n & pll_lock;
    reg [1:0] reset_sync;
    reg [1:0] dvi_reset_sync;
    wire reset_n = reset_sync[1];
    wire [11:0] video_x, video_y;
    wire video_de, video_hs, video_vs, frame_start;
    wire [23:0] rgb888;
    wire out_de, out_hs, out_vs;
    wire data_active, error_sticky, overflow_sticky, bitmap_ready;
    wire [15:0] front_points;

    display_pll u_display_pll (
        .lock(pll_lock), .clkout0(pixel_clk), .clkout1(serial_clk), .clkin(i_clock)
    );
    // Assert asynchronously on button/PLL unlock, release on a pixel clock edge.
    always @(posedge pixel_clk or negedge async_reset_n) begin
        if (!async_reset_n)
            reset_sync <= 0;
        else
            reset_sync <= {reset_sync[0], 1'b1};
    end
    // The encrypted DVI core contains both pixel logic and 245 MHz OSER10
    // cells. A separate, local reset chain keeps its reset fanout short and
    // remains aligned to PCLK. Two extra cycles make the final stage unique
    // so synthesis cannot merge it with the whole-core reset synchronizer.
    always @(posedge pixel_clk or negedge async_reset_n) begin
        if (!async_reset_n)
            dvi_reset_sync <= 0;
        else
            dvi_reset_sync <= {dvi_reset_sync[0], reset_n};
    end
    // Reuse the proven MPI7002 timing. Its RGB registers are bypassed:
    // synchronous bitmap reads need RGB and all sync signals delayed together.
    disp_driver u_timing (
        .ClkDisp(pixel_clk), .Rst_n(reset_n), .Data(24'd0), .DataReq(),
        .H_Addr(video_x), .V_Addr(video_y), .Disp_HS(video_hs),
        .Disp_VS(video_vs), .Disp_Red(), .Disp_Green(), .Disp_Blue(),
        .Frame_Begin(frame_start), .Disp_DE(video_de), .Disp_PCLK()
    );
    m10_point_cloud_core #(
        .REFRESH_FRAMES(REFRESH_FRAMES)
    ) u_core (
        .clk(pixel_clk), .reset_n(reset_n), .radar_rx(i_radar_rx),
        .zoom_in_n(i_zoom_in_n), .zoom_out_n(i_zoom_out_n),
        .video_x(video_x), .video_y(video_y), .video_de(video_de),
        .video_hs(video_hs), .video_vs(video_vs), .frame_start(frame_start),
        .rgb888(rgb888), .out_de(out_de), .out_hs(out_hs), .out_vs(out_vs),
        .debug_uart_tx(o_debug_uart_tx),
        .data_active(data_active), .error_sticky(error_sticky),
        .overflow_sticky(overflow_sticky), .front_points(front_points),
        .bitmap_ready(bitmap_ready)
    );
    DVI_TX_Top u_dvi_tx (
        .I_rst_n(dvi_reset_sync[1]), .I_serial_clk(serial_clk), .I_rgb_clk(pixel_clk),
        .I_rgb_vs(out_vs), .I_rgb_hs(out_hs), .I_rgb_de(out_de),
        .I_rgb_r(rgb888[23:16]), .I_rgb_g(rgb888[15:8]), .I_rgb_b(rgb888[7:0]),
        .O_tmds_clk_p(tmds_clk_p), .O_tmds_clk_n(tmds_clk_n),
        .O_tmds_data_p(tmds_data_p), .O_tmds_data_n(tmds_data_n)
    );
    // Board LED is active high. Error stays latched until S0 reset.
    assign o_error_led = error_sticky | overflow_sticky;
endmodule
