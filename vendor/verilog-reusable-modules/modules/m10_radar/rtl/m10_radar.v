// Independent M10_GPS radar reader: fixed 460800 baud, 8 data bits, no parity,
// one stop bit, and the 102-byte A5 5A ... FA FB packet format.
// The point stream uses valid/ready; raw UART byte/error pulses are also exposed.
// Dependencies: ../uart/rtl/uart_byte_rx.v and rtl/m10_packet_parser.v.
module m10_radar #(
    parameter integer CLK_FREQ_HZ = 50_000_000
) (
    input wire clk,
    input wire reset_n,
    input wire radar_rx,
    input wire [96:0] capture_context,
    output wire [96:0] point_context,
    output wire point_valid,
    input wire point_ready,
    output wire [15:0] point_angle_cdeg,
    output wire [15:0] point_distance_mm,
    output wire [5:0] point_index,
    output wire packet_valid,
    output wire packet_error,
    output wire packet_overflow,
    output wire [2:0] packet_error_reason,
    output wire [15:0] packet_speed,
    output wire [7:0] rx_data,
    output wire rx_valid,
    output wire rx_error,
    output reg data_active,
    output reg error_sticky,
    output reg overflow_sticky,
    output reg [31:0] packet_count,
    output reg [15:0] rx_byte_count,
    output reg [15:0] uart_error_count,
    output reg [15:0] bad_packet_count,
    output reg [7:0] last_rx_byte,
    output reg [2:0] last_error_reason
);
    reg [31:0] silence_clks;

    // Reception continues even when the point consumer stalls. The parser's
    // two packet banks retain validated packets and report full-queue drops.
    // Recovery waits for an idle line after reset and a malformed stop bit.
    uart_byte_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(460800),
        .DATA_BITS(8), .PARITY_EN(0), .RECOVERY_EN(1)
    ) u_uart (
        .i_clock(clk), .i_reset_n(reset_n), .i_uart_rx(radar_rx),
        .o_rx_data(rx_data), .o_rx_valid(rx_valid), .i_rx_ready(1'b1),
        .o_rx_error(rx_error)
    );
    m10_packet_parser #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_parser (
        .clk(clk), .rst_n(reset_n), .rx_data(rx_data), .rx_valid(rx_valid),
        .rx_error(rx_error), .capture_context(capture_context), .point_context(point_context),
        .point_valid(point_valid), .point_ready(point_ready),
        .point_angle_cdeg(point_angle_cdeg), .point_distance_mm(point_distance_mm),
        .point_index(point_index), .packet_valid(packet_valid),
        .packet_error(packet_error), .packet_overflow(packet_overflow),
        .packet_error_reason(packet_error_reason), .packet_speed(packet_speed)
    );

    // Activity expires after one second without a validated packet. Error
    // indicators remain latched until reset. Byte/good-packet counters wrap;
    // error counters saturate so an extended fault cannot appear as zero.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            silence_clks <= 0;
            packet_count <= 0;
            rx_byte_count <= 0;
            uart_error_count <= 0;
            bad_packet_count <= 0;
            last_rx_byte <= 0;
            last_error_reason <= 0;
            data_active <= 0;
            error_sticky <= 0;
            overflow_sticky <= 0;
        end else begin
            if (rx_valid) begin
                rx_byte_count <= rx_byte_count + 1'b1;
                last_rx_byte <= rx_data;
            end
            if (rx_error && uart_error_count != 16'hFFFF)
                uart_error_count <= uart_error_count + 1'b1;
            if (packet_error) begin
                if (bad_packet_count != 16'hFFFF)
                    bad_packet_count <= bad_packet_count + 1'b1;
                last_error_reason <= packet_error_reason;
            end
            if (packet_valid) begin
                silence_clks <= 0;
                data_active <= 1;
                packet_count <= packet_count + 1'b1;
            end else if (silence_clks < CLK_FREQ_HZ) begin
                silence_clks <= silence_clks + 1'b1;
            end else begin
                data_active <= 0;
            end
            if (rx_error || packet_error)
                error_sticky <= 1;
            if (packet_overflow)
                overflow_sticky <= 1;
        end
    end
endmodule
