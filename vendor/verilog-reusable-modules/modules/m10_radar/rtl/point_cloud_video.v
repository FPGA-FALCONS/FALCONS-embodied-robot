// 1024x600 radar UI. RGB and all three video control signals have exactly
// three rising-edge stages of latency, including the synchronous bitmap read.
// Character selection and font lookup are separated for 49 MHz timing.
module point_cloud_video #(
    parameter integer REFRESH_FRAMES = 12
) (
    input             clk,
    input             reset_n,
    input             point_valid,
    output            point_ready,
    input      [9:0]  point_x,
    input      [9:0]  point_y,
    input     [11:0]  video_x,
    input     [11:0]  video_y,
    input             video_de,
    input             video_hs,
    input             video_vs,
    input             frame_start,
    input      [1:0]  zoom_level,
    input             plot_clear,
    input             status_data_active,
    input             status_error,
    input             status_overflow,
    input      [15:0] diagnostic_rx_bytes,
    input      [15:0] diagnostic_uart_errors,
    input      [15:0] diagnostic_bad_packets,
    input      [15:0] diagnostic_good_packets,
    input      [7:0]  diagnostic_last_byte,
    input      [2:0]  diagnostic_last_error,
    output reg [23:0] rgb888,
    output reg        out_de,
    output reg        out_hs,
    output reg        out_vs,
    output     [15:0] front_points,
    output            bitmap_ready
);
    // Both strings are constant per zoom level; no decimal divider is needed.
    reg [95:0] zoom_scale_text, zoom_range_text;
    always @* begin
        case (zoom_level)
            2'd0: begin
                zoom_scale_text = "010 MM/PX   "; zoom_range_text = "2.5 M       ";
            end
            2'd1: begin
                zoom_scale_text = "020 MM/PX   "; zoom_range_text = "5 M         ";
            end
            2'd2: begin
                zoom_scale_text = "040 MM/PX   "; zoom_range_text = "10 M        ";
            end
            default: begin
                zoom_scale_text = "080 MM/PX   "; zoom_range_text = "20 M        ";
            end
        endcase
    end
    wire input_inside = point_x >= 10'd256 && point_x < 10'd768 &&
                        point_y >= 10'd44 && point_y < 10'd556;
    wire [9:0] local_point_x = point_x - 10'd256;
    wire [9:0] local_point_y = point_y - 10'd44;
    wire bitmap_point_ready, bitmap_pixel, bitmap_pixel_valid;
    assign point_ready = input_inside ? bitmap_point_ready : 1'b1;
    point_bitmap #(.REFRESH_FRAMES(REFRESH_FRAMES)) u_bitmap (
        .clk(clk), .reset_n(reset_n && !plot_clear),
        .point_valid(point_valid && input_inside),
        .point_ready(bitmap_point_ready),
        .point_x(local_point_x[8:0]), .point_y(local_point_y[8:0]),
        .video_x(video_x), .video_y(video_y), .video_de(video_de),
        .frame_start(frame_start), .pixel_bit(bitmap_pixel),
        .pixel_valid(bitmap_pixel_valid), .front_points(front_points),
        .bitmap_ready(bitmap_ready)
    );

    // Binary-to-decimal runs only after a displayed accumulation changes.
    // Shift/add-3 avoids five parallel run-time division circuits for digits.
    reg [15:0] decimal_source, decimal_shift;
    reg [19:0] decimal_work, decimal_digits;
    reg [4:0] decimal_step;
    reg decimal_busy;
    reg [19:0] decimal_adjusted;
    integer digit;
    always @* begin
        decimal_adjusted = decimal_work;
        for (digit = 0; digit < 5; digit = digit + 1)
            if (decimal_work[digit*4 +: 4] >= 5)
                decimal_adjusted[digit*4 +: 4] = decimal_work[digit*4 +: 4] + 4'd3;
    end
    wire [19:0] decimal_next = {decimal_adjusted[18:0], decimal_shift[15]};
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            decimal_source <= 0;
            decimal_shift <= 0;
            decimal_work <= 0;
            decimal_digits <= 0;
            decimal_step <= 0;
            decimal_busy <= 0;
        end else if (!decimal_busy) begin
            if (decimal_source != front_points) begin
                decimal_source <= front_points;
                decimal_shift <= front_points;
                decimal_work <= 0;
                decimal_step <= 0;
                decimal_busy <= 1'b1;
            end
        end else begin
            decimal_work <= decimal_next;
            decimal_shift <= {decimal_shift[14:0], 1'b0};
            if (decimal_step == 15) begin
                decimal_digits <= decimal_next;
                decimal_busy <= 0;
            end else begin
                decimal_step <= decimal_step + 1'b1;
            end
        end
    end

    // A small built-in 5x7 font; no external font file or software is needed.
    function [4:0] font_row;
        input [7:0] character;
        input [2:0] row;
        reg [34:0] glyph;
        begin
            case (character)
                "0": glyph = {5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
                "1": glyph = {5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "2": glyph = {5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
                "3": glyph = {5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
                "4": glyph = {5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
                "5": glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
                "6": glyph = {5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
                "7": glyph = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                "8": glyph = {5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
                "9": glyph = {5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
                "A": glyph = {5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "B": glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
                "C": glyph = {5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110};
                "D": glyph = {5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
                "E": glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
                "F": glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
                "G": glyph = {5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110};
                "H": glyph = {5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "I": glyph = {5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "K": glyph = {5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
                "L": glyph = {5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
                "M": glyph = {5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
                "N": glyph = {5'b10001,5'b11001,5'b11001,5'b10101,5'b10011,5'b10011,5'b10001};
                "O": glyph = {5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "P": glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
                "R": glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
                "S": glyph = {5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
                "T": glyph = {5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
                "U": glyph = {5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "V": glyph = {5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
                "W": glyph = {5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010};
                "X": glyph = {5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
                "Y": glyph = {5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
                "Z": glyph = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111};
                ".": glyph = {5'b00000,5'b00000,5'b00000,5'b00000,5'b00000,5'b00110,5'b00110};
                "/": glyph = {5'b00001,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b10000};
                default: glyph = 0;
            endcase
            font_row = glyph >> ((6-row)*5);
        end
    endfunction

    // Diagnostic counters are hexadecimal, avoiding run-time division and
    // leaving the existing decimal POINTS display unchanged.
    function [7:0] hex_character;
        input [3:0] value;
        begin
            if (value < 10) hex_character = {4'h3, value};
            else hex_character = 8'h41 + {4'b0, value} - 8'd10;
        end
    endfunction
    function [95:0] diagnostic_count_text;
        input [15:0] value;
        begin
            diagnostic_count_text = {hex_character(value[15:12]),
                                     hex_character(value[11:8]),
                                     hex_character(value[7:4]),
                                     hex_character(value[3:0]), " HEX    "};
        end
    endfunction

    wire in_plot = video_x >= 256 && video_x < 768 &&
                   video_y >= 44 && video_y < 556;
    wire signed [12:0] dx_full = $signed({1'b0, video_x}) - 13'sd512;
    wire signed [12:0] dy_full = $signed({1'b0, video_y}) - 13'sd300;
    wire signed [8:0] dx = dx_full[8:0];
    wire signed [8:0] dy = dy_full[8:0];
    wire [17:0] dx_squared = dx * dx;
    wire [17:0] dy_squared = dy * dy;
    wire [18:0] radius_squared = {1'b0, dx_squared} + {1'b0, dy_squared};
    wire on_ring = (radius_squared >= 2450 && radius_squared <= 2550) ||
                   (radius_squared >= 9900 && radius_squared <= 10100) ||
                   (radius_squared >= 22350 && radius_squared <= 22650) ||
                   (radius_squared >= 39800 && radius_squared <= 40200) ||
                   (radius_squared >= 62250 && radius_squared <= 62750);
    wire on_axis = video_x == 512 || video_y == 300;
    wire plot_border = in_plot && (video_x == 256 || video_x == 767 ||
                                  video_y == 44 || video_y == 555);

    reg [95:0] text;
    reg [11:0] text_x, text_y;
    reg text_enabled;
    reg [23:0] text_color;
    always @* begin
        text = "            ";
        text_x = 24;
        text_y = 40;
        text_enabled = 1'b0;
        text_color = 24'hb6cad4;
        if (video_x >= 24 && video_x < 168) begin
            text_enabled = 1'b1;
            if (video_y >= 40 && video_y < 54) begin
                text = "M10 RADAR   "; text_y = 40; text_color = 24'h56f0ba;
            end else if (video_y >= 72 && video_y < 86) begin
                text = "RX BYTES    "; text_y = 72;
            end else if (video_y >= 96 && video_y < 110) begin
                text = diagnostic_count_text(diagnostic_rx_bytes); text_y = 96;
            end else if (video_y >= 132 && video_y < 146) begin
                text = "POINTS      "; text_y = 132;
            end else if (video_y >= 164 && video_y < 178) begin
                text = {8'h30+{4'b0,decimal_digits[19:16]},
                        8'h30+{4'b0,decimal_digits[15:12]},
                        8'h30+{4'b0,decimal_digits[11:8]},
                        8'h30+{4'b0,decimal_digits[7:4]},
                        8'h30+{4'b0,decimal_digits[3:0]}, "       "};
                text_y = 164; text_color = 24'h56f0ba;
            end else if (video_y >= 224 && video_y < 238) begin
                text = "MAX RANGE   "; text_y = 224;
            end else if (video_y >= 256 && video_y < 270) begin
                text = zoom_range_text; text_y = 256;
            end else if (video_y >= 316 && video_y < 330) begin
                text = "SCALE       "; text_y = 316;
            end else if (video_y >= 348 && video_y < 362) begin
                text = zoom_scale_text;
                text_y = 348;
            end else if (video_y >= 384 && video_y < 398) begin
                text = "GOOD PACKETS"; text_y = 384;
            end else if (video_y >= 408 && video_y < 422) begin
                text = diagnostic_count_text(diagnostic_good_packets); text_y = 408;
            end else if (video_y >= 452 && video_y < 466) begin
                text = "S1 ZOOM IN  "; text_y = 452;
            end else if (video_y >= 484 && video_y < 498) begin
                text = "S2 ZOOM OUT "; text_y = 484;
            end else if (video_y >= 520 && video_y < 534) begin
                text = "LAST BYTE   "; text_y = 520;
            end else if (video_y >= 544 && video_y < 558) begin
                text = {hex_character(diagnostic_last_byte[7:4]),
                        hex_character(diagnostic_last_byte[3:0]), " HEX      "};
                text_y = 544;
            end else text_enabled = 1'b0;
        end else if (video_x >= 820 && video_x < 964) begin
            text_x = 820;
            text_enabled = 1'b1;
            if (video_y >= 104 && video_y < 118) begin
                text = "LINK        "; text_y = 104;
            end else if (video_y >= 136 && video_y < 150) begin
                text = status_data_active ? "LIVE        " : "WAIT        ";
                text_y = 136;
                text_color = status_data_active ? 24'h56f0ba : 24'h647887;
            end else if (video_y >= 200 && video_y < 214) begin
                text = "PACKET ERROR"; text_y = 200;
            end else if (video_y >= 232 && video_y < 246) begin
                text = "OK          "; text_y = 232;
                if (status_error) begin
                    case (diagnostic_last_error)
                        3'd1: text = "UART        ";
                        3'd2: text = "TAIL        ";
                        3'd3: text = "ANGLE       ";
                        3'd4: text = "TIMEOUT     ";
                        3'd5: text = "OVERFLOW    ";
                        default: text = "ERROR       ";
                    endcase
                end
                text_color = status_error ? 24'hff675d : 24'h56f0ba;
            end else if (video_y >= 296 && video_y < 310) begin
                text = "OVERFLOW    "; text_y = 296;
            end else if (video_y >= 328 && video_y < 342) begin
                text = status_overflow ? "ERROR       " : "OK          "; text_y = 328;
                text_color = status_overflow ? 24'hffb851 : 24'h56f0ba;
            end else if (video_y >= 400 && video_y < 414) begin
                text = "DISPLAY     "; text_y = 400;
            end else if (video_y >= 432 && video_y < 446) begin
                text = bitmap_ready ? "READY       " : "CLEAR       "; text_y = 432;
                text_color = bitmap_ready ? 24'h56f0ba : 24'h647887;
            end else if (video_y >= 480 && video_y < 494) begin
                text = "UART ERR    "; text_y = 480;
            end else if (video_y >= 504 && video_y < 518) begin
                text = diagnostic_count_text(diagnostic_uart_errors); text_y = 504;
            end else if (video_y >= 544 && video_y < 558) begin
                text = "BAD PACKETS "; text_y = 544;
            end else if (video_y >= 568 && video_y < 582) begin
                text = diagnostic_count_text(diagnostic_bad_packets); text_y = 568;
            end else text_enabled = 1'b0;
        end
    end
    wire [11:0] text_relative_x = video_x - text_x;
    wire [11:0] text_relative_y = video_y - text_y;
    wire [3:0] character_index = text_relative_x / 12;
    wire [3:0] character_column = (text_relative_x % 12) >> 1;
    wire [7:0] character = text >> ((11-character_index)*8);
    reg [7:0] character_q;
    reg [3:0] character_column_q;
    reg [2:0] character_row_q;
    reg text_enabled_q;
    reg [23:0] text_color_q;
    wire [4:0] glyph_line = font_row(character_q, character_row_q);
    wire on_text = text_enabled_q && character_column_q < 5 &&
                   glyph_line[4-character_column_q];

    reg [23:0] background_rgb;
    always @* begin
        background_rgb = 24'h080f18;
        if (in_plot) begin
            background_rgb = 24'h0b1720;
            if (on_ring) background_rgb = 24'h193d45;
            if (on_axis) background_rgb = 24'h245660;
            if (plot_border) background_rgb = 24'h34515d;
            if (video_x >= 510 && video_x <= 514 &&
                video_y >= 298 && video_y <= 302)
                background_rgb = 24'hc1dedc;
        end
    end

    reg [23:0] coordinate_background_q, background_q;
    reg bitmap_pixel_q;
    reg de_q, hs_q, vs_q, de_qq, hs_qq, vs_qq;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            character_q <= 0;
            character_column_q <= 0;
            character_row_q <= 0;
            text_enabled_q <= 0;
            text_color_q <= 0;
            coordinate_background_q <= 0;
            background_q <= 0;
            bitmap_pixel_q <= 0;
            rgb888 <= 0;
            de_q <= 0; hs_q <= 1; vs_q <= 1;
            de_qq <= 0; hs_qq <= 1; vs_qq <= 1;
            out_de <= 0; out_hs <= 1; out_vs <= 1;
        end else begin
            character_q <= character;
            character_column_q <= character_column;
            character_row_q <= text_relative_y[3:1];
            text_enabled_q <= text_enabled;
            text_color_q <= text_color;
            coordinate_background_q <= background_rgb;
            de_q <= video_de; hs_q <= video_hs; vs_q <= video_vs;
            background_q <= on_text ? text_color_q : coordinate_background_q;
            bitmap_pixel_q <= bitmap_pixel;
            de_qq <= de_q; hs_qq <= hs_q; vs_qq <= vs_q;
            rgb888 <= !de_qq ? 24'b0 : bitmap_pixel_q ? 24'h56f0ba : background_q;
            out_de <= de_qq; out_hs <= hs_qq; out_vs <= vs_qq;
        end
    end
endmodule
