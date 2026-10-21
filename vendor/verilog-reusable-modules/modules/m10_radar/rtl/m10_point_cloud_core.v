// All logic runs on the 49 MHz pixel clock; no multi-bit CDC is required.
module m10_point_cloud_core #(
    parameter integer CLK_FREQ_HZ = 49_000_000,
    parameter integer KEY_DEBOUNCE_CLKS = CLK_FREQ_HZ / 50,
    parameter integer REFRESH_FRAMES = 12
)(
    input wire clk,
    input wire reset_n,
    input wire radar_rx,
    input wire zoom_in_n,
    input wire zoom_out_n,
    input wire [11:0] video_x,
    input wire [11:0] video_y,
    input wire video_de,
    input wire video_hs,
    input wire video_vs,
    input wire frame_start,
    output wire [23:0] rgb888,
    output wire out_de,
    output wire out_hs,
    output wire out_vs,
    output wire debug_uart_tx,
    output wire data_active,
    output wire error_sticky,
    output wire overflow_sticky,
    output wire [15:0] front_points,
    output wire bitmap_ready
);
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    wire debug_tx_ready;
    wire point_valid, point_ready;
    wire [15:0] point_angle_cdeg, point_distance_mm;
    wire [5:0] point_index;
    wire packet_valid, packet_error, packet_overflow;
    wire [2:0] packet_error_reason;
    wire [15:0] packet_speed;
    wire pixel_valid, pixel_ready;
    wire polar_point_ready;
    wire [1:0] zoom_level;
    wire plot_clear;
    wire plot_reset_n = reset_n && !plot_clear;
    assign point_ready = plot_reset_n && polar_point_ready;
    wire [9:0] pixel_x, pixel_y;
    wire [31:0] packet_count;
    wire [15:0] rx_byte_count, uart_error_count, bad_packet_count;
    wire [7:0] last_rx_byte;
    wire [2:0] last_error_reason;
    m10_radar #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_radar (
        .clk(clk), .reset_n(reset_n), .radar_rx(radar_rx),
        .capture_context(97'd0), .point_context(),
        .point_valid(point_valid), .point_ready(point_ready),
        .point_angle_cdeg(point_angle_cdeg), .point_distance_mm(point_distance_mm),
        .point_index(point_index), .packet_valid(packet_valid),
        .packet_error(packet_error), .packet_overflow(packet_overflow),
        .packet_error_reason(packet_error_reason), .packet_speed(packet_speed),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_error(rx_error),
        .data_active(data_active), .error_sticky(error_sticky),
        .overflow_sticky(overflow_sticky), .packet_count(packet_count),
        .rx_byte_count(rx_byte_count), .uart_error_count(uart_error_count),
        .bad_packet_count(bad_packet_count), .last_rx_byte(last_rx_byte),
        .last_error_reason(last_error_reason)
    );
    // Mirror decoded radar bytes to the board USB-UART. At twice the input
    // baud rate, each transmit finishes before the next received byte; no
    // packet filtering or text formatting hides a malformed frame.
    uart_byte_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(921600)) u_debug_tx (
        .i_clock(clk), .i_reset_n(reset_n), .i_tx_data(rx_data),
        .i_tx_valid(rx_valid), .o_tx_ready(debug_tx_ready),
        .o_uart_tx(debug_uart_tx)
    );
    zoom_control #(.DEBOUNCE_CLKS(KEY_DEBOUNCE_CLKS)) u_zoom (
        .clk(clk), .reset_n(reset_n), .zoom_in_n(zoom_in_n),
        .zoom_out_n(zoom_out_n), .frame_start(frame_start),
        .zoom_level(zoom_level), .plot_clear(plot_clear)
    );
    polar_to_xy u_polar (
        .clk(clk), .reset_n(plot_reset_n), .zoom_level(zoom_level),
        .point_valid(point_valid && plot_reset_n),
        .point_ready(polar_point_ready), .angle_cdeg(point_angle_cdeg),
        .distance_mm(point_distance_mm), .pixel_valid(pixel_valid),
        .pixel_ready(pixel_ready), .pixel_x(pixel_x), .pixel_y(pixel_y)
    );
    point_cloud_video #(.REFRESH_FRAMES(REFRESH_FRAMES)) u_video (
        .clk(clk), .reset_n(reset_n), .zoom_level(zoom_level),
        .plot_clear(plot_clear), .point_valid(pixel_valid && plot_reset_n),
        .point_ready(pixel_ready), .point_x(pixel_x), .point_y(pixel_y),
        .video_x(video_x), .video_y(video_y), .video_de(video_de),
        .video_hs(video_hs), .video_vs(video_vs), .frame_start(frame_start),
        .status_data_active(data_active), .status_error(error_sticky),
        .status_overflow(overflow_sticky),
        .diagnostic_rx_bytes(rx_byte_count),
        .diagnostic_uart_errors(uart_error_count),
        .diagnostic_bad_packets(bad_packet_count),
        .diagnostic_good_packets(packet_count[15:0]),
        .diagnostic_last_byte(last_rx_byte),
        .diagnostic_last_error(last_error_reason), .rgb888(rgb888),
        .out_de(out_de), .out_hs(out_hs), .out_vs(out_vs),
        .front_points(front_points), .bitmap_ready(bitmap_ready)
    );

endmodule
