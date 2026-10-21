// M10_GPS UART packet decoder: exactly 102 bytes with A5 5A, start angle,
// speed, 42 big-endian distances, ten time bytes at 90..99 and FA FB at
// 100/101. There is no checksum in this format.
// Point angles are centidegrees; distances are millimetres.
// Two packet banks retain complete packets while the point consumer stalls.
// A packet received while both banks are occupied is discarded in its entirety.
// An opaque 97-bit acquisition context is captured at each distance low byte.
// It is buffered with that slot, read synchronously, and held until acceptance.
module m10_packet_parser #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BYTE_TIMEOUT_CLKS = CLK_FREQ_HZ / 100
) (
    input wire clk,
    input wire rst_n,
    input wire [7:0] rx_data,
    input wire rx_valid,
    input wire rx_error,
    input wire [96:0] capture_context,
    output wire [96:0] point_context,
    output wire point_valid,
    input wire point_ready,
    output wire [15:0] point_angle_cdeg,
    output wire [15:0] point_distance_mm,
    output wire [5:0] point_index,
    output reg packet_valid,
    output reg packet_error,
    output reg [2:0] packet_error_reason,
    output reg packet_overflow,
    output reg [15:0] packet_speed
);
    function integer counter_width;
        input integer value;
        integer work_value;
        begin
            counter_width = 0;
            work_value = value - 1;
            while (work_value > 0) begin
                counter_width = counter_width + 1;
                work_value = work_value >> 1;
            end
            if (counter_width < 1) counter_width = 1;
        end
    endfunction

    // Constant quotient/remainder table for a 15-degree sector. This avoids a
    // run-time divider when M10_GPS pads the final slots with invalid FFFF.
    function [16:0] gps_step_parts;
        input [5:0] count;
        begin
            case (count)
                1: gps_step_parts = {11'd1500, 6'd0};
                2: gps_step_parts = {11'd750, 6'd0};
                3: gps_step_parts = {11'd500, 6'd0};
                4: gps_step_parts = {11'd375, 6'd0};
                5: gps_step_parts = {11'd300, 6'd0};
                6: gps_step_parts = {11'd250, 6'd0};
                7: gps_step_parts = {11'd214, 6'd2};
                8: gps_step_parts = {11'd187, 6'd4};
                9: gps_step_parts = {11'd166, 6'd6};
                10: gps_step_parts = {11'd150, 6'd0};
                11: gps_step_parts = {11'd136, 6'd4};
                12: gps_step_parts = {11'd125, 6'd0};
                13: gps_step_parts = {11'd115, 6'd5};
                14: gps_step_parts = {11'd107, 6'd2};
                15: gps_step_parts = {11'd100, 6'd0};
                16: gps_step_parts = {11'd93, 6'd12};
                17: gps_step_parts = {11'd88, 6'd4};
                18: gps_step_parts = {11'd83, 6'd6};
                19: gps_step_parts = {11'd78, 6'd18};
                20: gps_step_parts = {11'd75, 6'd0};
                21: gps_step_parts = {11'd71, 6'd9};
                22: gps_step_parts = {11'd68, 6'd4};
                23: gps_step_parts = {11'd65, 6'd5};
                24: gps_step_parts = {11'd62, 6'd12};
                25: gps_step_parts = {11'd60, 6'd0};
                26: gps_step_parts = {11'd57, 6'd18};
                27: gps_step_parts = {11'd55, 6'd15};
                28: gps_step_parts = {11'd53, 6'd16};
                29: gps_step_parts = {11'd51, 6'd21};
                30: gps_step_parts = {11'd50, 6'd0};
                31: gps_step_parts = {11'd48, 6'd12};
                32: gps_step_parts = {11'd46, 6'd28};
                33: gps_step_parts = {11'd45, 6'd15};
                34: gps_step_parts = {11'd44, 6'd4};
                35: gps_step_parts = {11'd42, 6'd30};
                36: gps_step_parts = {11'd41, 6'd24};
                37: gps_step_parts = {11'd40, 6'd20};
                38: gps_step_parts = {11'd39, 6'd18};
                39: gps_step_parts = {11'd38, 6'd18};
                40: gps_step_parts = {11'd37, 6'd20};
                41: gps_step_parts = {11'd36, 6'd24};
                42: gps_step_parts = {11'd35, 6'd30};
                default: gps_step_parts = 17'd0;
            endcase
        end
    endfunction

    localparam integer TIMEOUT_WIDTH = counter_width(BYTE_TIMEOUT_CLKS);
    localparam [3:0] S_HEADER_0 = 0, S_HEADER_1 = 1,
                     S_ANGLE_HI = 2, S_ANGLE_LO = 3,
                     S_SPEED_HI = 4, S_SPEED_LO = 5,
                     S_DISTANCE_HI = 6, S_DISTANCE_LO = 7,
                     S_TAIL_0 = 8, S_TAIL_1 = 9, S_TIME = 10;
    localparam [2:0] E_NONE = 0, E_UART = 1, E_TAIL = 2,
                     E_ANGLE = 3, E_TIMEOUT = 4, E_OVERFLOW = 5;

    reg [3:0] state;
    reg [TIMEOUT_WIDTH-1:0] timeout_count;
    reg [7:0] byte_hi;
    reg [15:0] received_angle;
    reg [15:0] received_speed;
    reg [5:0] received_index;
    reg [3:0] time_index;
    reg [5:0] valid_distance_count;
    reg previous_distance_invalid;
    reg receive_keep;
    reg write_bank;
    reg read_bank;
    reg [1:0] bank_pending;
    reg [15:0] bank_first_angle [0:1];
    reg [5:0] bank_denominator [0:1];
    reg [10:0] bank_step_base [0:1];
    reg [5:0] bank_step_remainder [0:1];
    reg [15:0] distances [0:83];
    reg [96:0] contexts [0:83];
    reg output_loaded;
    reg [15:0] output_distance;
    reg [96:0] output_context;
    reg output_active;
    reg [5:0] output_index;
    reg [15:0] output_angle;
    reg [5:0] output_fraction;
    reg [5:0] output_denominator;
    reg [10:0] output_step_base;
    reg [5:0] output_step_remainder;

    wire [15:0] normalized_angle = (received_angle == 16'd36000)
                                           ? 16'd0 : received_angle;
    wire [15:0] raw_distance = {byte_hi, rx_data};
    // In M10_GPS bit 15 is intensity; FFFF remains invalid. Match the SDK's
    // slot-40 invalidation when slot 39 is invalid or its high byte is >= F5.
    wire received_distance_invalid = (raw_distance == 16'hFFFF) ||
                ((received_index == 6'd40) &&
                 (previous_distance_invalid || (byte_hi >= 8'hF5)));
    wire [15:0] received_distance = received_distance_invalid
                ? 16'hFFFF : {1'b0, byte_hi[6:0], rx_data};
    wire [16:0] received_step_parts = gps_step_parts(valid_distance_count);
    wire [6:0] fraction_sum = {1'b0, output_fraction} + {1'b0, output_step_remainder};
    wire fraction_carry = (fraction_sum >= {1'b0, output_denominator});
    wire [15:0] angle_increment = output_step_base + fraction_carry;
    wire [16:0] next_output_angle = {1'b0, output_angle} + angle_increment;
    wire [6:0] write_address = (write_bank ? 7'd42 : 7'd0) + received_index;
    wire [6:0] read_address = (read_bank ? 7'd42 : 7'd0) + output_index;

    // Standard valid/ready interface: data and valid remain stable while stalled.
    assign point_valid = output_active && output_loaded;
    assign point_angle_cdeg = output_angle;
    assign point_distance_mm = output_distance;
    assign point_context = output_context;
    assign point_index = output_index;

    // Registered reads infer synchronous RAM, rather than a 97-bit-wide
    // combinational multiplexer across all 84 slot contexts. A one-clock bubble
    // between points loads both memories before valid is asserted.
    always @(posedge clk) begin
        if (rst_n && rx_valid && !rx_error && state == S_DISTANCE_LO && receive_keep) begin
            distances[write_address] <= received_distance;
            contexts[write_address] <= capture_context;
        end
        if (rst_n && output_active && !output_loaded) begin
            output_distance <= distances[read_address];
            output_context <= contexts[read_address];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_HEADER_0;
            timeout_count <= 0;
            byte_hi <= 0;
            received_angle <= 0;
            received_speed <= 0;
            received_index <= 0;
            time_index <= 0;
            valid_distance_count <= 0;
            previous_distance_invalid <= 0;
            receive_keep <= 0;
            write_bank <= 0;
            read_bank <= 0;
            bank_pending <= 0;
            output_active <= 0;
            output_loaded <= 0;
            output_index <= 0;
            output_angle <= 0;
            output_fraction <= 0;
            output_denominator <= 1;
            output_step_base <= 0;
            output_step_remainder <= 0;
            packet_valid <= 0;
            packet_error <= 0;
            packet_error_reason <= E_NONE;
            packet_overflow <= 0;
            packet_speed <= 0;
        end else begin
            packet_valid <= 0;
            packet_error <= 0;
            packet_overflow <= 0;

            // Pending banks form a two-entry FIFO. Bank ownership is retained
            // until all 42 points have completed their ready/valid handshake.
            if (!output_active) begin
                if (bank_pending[read_bank]) begin
                    output_active <= 1;
                    output_loaded <= 0;
                    output_index <= 0;
                    output_angle <= bank_first_angle[read_bank];
                    output_fraction <= 0;
                    output_denominator <= bank_denominator[read_bank];
                    output_step_base <= bank_step_base[read_bank];
                    output_step_remainder <= bank_step_remainder[read_bank];
                end
            end else if (!output_loaded) begin
                output_loaded <= 1;
            end else if (point_ready) begin
                output_loaded <= 0;
                if (output_index == 6'd41) begin
                    output_active <= 0;
                    bank_pending[read_bank] <= 0;
                    read_bank <= ~read_bank;
                end else begin
                    output_index <= output_index + 1'b1;
                    if (next_output_angle >= 17'd36000)
                        output_angle <= next_output_angle - 17'd36000;
                    else
                        output_angle <= next_output_angle[15:0];
                    output_fraction <= fraction_carry
                                     ? fraction_sum - output_denominator
                                     : fraction_sum[5:0];
                end
            end

            // A framing error invalidates only the packet being received;
            // already validated packets remain available to the consumer.
            if (rx_error) begin
                state <= S_HEADER_0;
                timeout_count <= 0;
                receive_keep <= 0;
                packet_error <= 1;
                packet_error_reason <= E_UART;
            end else if (rx_valid) begin
                timeout_count <= 0;
                case (state)
                    S_HEADER_0: begin
                        if (rx_data == 8'hA5) state <= S_HEADER_1;
                    end
                    S_HEADER_1: begin
                        if (rx_data == 8'h5A) begin
                            state <= S_ANGLE_HI;
                            received_index <= 0;
                            valid_distance_count <= 0;
                            previous_distance_invalid <= 0;
                            receive_keep <= !bank_pending[write_bank];
                        end else if (rx_data != 8'hA5) begin
                            state <= S_HEADER_0;
                        end
                        // A5 A5 5A recognizes the overlapping second header.
                    end
                    S_ANGLE_HI: begin
                        byte_hi <= rx_data;
                        state <= S_ANGLE_LO;
                    end
                    S_ANGLE_LO: begin
                        received_angle <= {byte_hi, rx_data};
                        state <= S_SPEED_HI;
                    end
                    S_SPEED_HI: begin
                        byte_hi <= rx_data;
                        state <= S_SPEED_LO;
                    end
                    S_SPEED_LO: begin
                        received_speed <= {byte_hi, rx_data};
                        state <= S_DISTANCE_HI;
                    end
                    S_DISTANCE_HI: begin
                        byte_hi <= rx_data;
                        state <= S_DISTANCE_LO;
                    end
                    S_DISTANCE_LO: begin
                        previous_distance_invalid <= received_distance_invalid;
                        if (!received_distance_invalid)
                            valid_distance_count <= valid_distance_count + 1'b1;
                        if (received_index == 6'd41) begin
                            time_index <= 0;
                            state <= S_TIME;
                        end else begin
                            received_index <= received_index + 1'b1;
                            state <= S_DISTANCE_HI;
                        end
                    end
                    S_TIME: begin
                        // Time bytes may contain header or tail markers; only
                        // their fixed ten-byte length determines the real tail.
                        if (time_index == 4'd9) state <= S_TAIL_0;
                        else time_index <= time_index + 1'b1;
                    end
                    S_TAIL_0: begin
                        if (rx_data == 8'hFA) begin
                            state <= S_TAIL_1;
                        end else begin
                            packet_error <= 1;
                            packet_error_reason <= E_TAIL;
                            receive_keep <= 0;
                            state <= (rx_data == 8'hA5) ? S_HEADER_1 : S_HEADER_0;
                        end
                    end
                    S_TAIL_1: begin
                        receive_keep <= 0;
                        if ((rx_data == 8'hFB) && (received_angle <= 16'd36000)) begin
                            if (receive_keep) begin
                                bank_first_angle[write_bank] <= normalized_angle;
                                bank_denominator[write_bank] <= (valid_distance_count == 0)
                                                                ? 6'd1 : valid_distance_count;
                                bank_step_base[write_bank] <= received_step_parts[16:6];
                                bank_step_remainder[write_bank] <= received_step_parts[5:0];
                                bank_pending[write_bank] <= 1;
                                write_bank <= ~write_bank;
                                packet_speed <= received_speed;
                                packet_valid <= 1;
                            end else begin
                                packet_error <= 1;
                                packet_error_reason <= E_OVERFLOW;
                                packet_overflow <= 1;
                            end
                        end else begin
                            packet_error <= 1;
                            packet_error_reason <= (rx_data != 8'hFB)
                                                 ? E_TAIL : E_ANGLE;
                        end
                        state <= (rx_data == 8'hA5) ? S_HEADER_1 : S_HEADER_0;
                    end
                    default: begin
                        state <= S_HEADER_0;
                        receive_keep <= 0;
                    end
                endcase
            end else if (state != S_HEADER_0) begin
                if (timeout_count == BYTE_TIMEOUT_CLKS - 1) begin
                    state <= S_HEADER_0;
                    timeout_count <= 0;
                    receive_keep <= 0;
                    // A lone header prefix expires silently; a recognized
                    // A5 5A header followed by a truncated packet is an error.
                    if (state != S_HEADER_1) begin
                        packet_error <= 1;
                        packet_error_reason <= E_TIMEOUT;
                    end
                end else begin
                    timeout_count <= timeout_count + 1'b1;
                end
            end else begin
                timeout_count <= 0;
            end
        end
    end
endmodule
