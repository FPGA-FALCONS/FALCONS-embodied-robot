// Polar samples to screen coordinates. 0 degrees points upward; angles grow
// clockwise by default. Invalid / off-canvas samples are consumed silently.
// zoom_level 0/1/2/3 selects 10/20/40/80 mm per pixel. Each accepted point
// locks its zoom level, so a button change cannot alter an in-flight sample.
module polar_to_xy #(
    parameter integer MIN_DISTANCE_MM = 1,
    parameter integer CLOCKWISE = 1,
    parameter integer CENTER_X = 512,
    parameter integer CENTER_Y = 300
) (
    input              clk,
    input              reset_n,
    input              point_valid,
    output             point_ready,
    input      [15:0]  angle_cdeg,
    input      [15:0]  distance_mm,
    input       [1:0]  zoom_level,
    output reg         pixel_valid,
    input              pixel_ready,
    output reg [9:0]   pixel_x,
    output reg [9:0]   pixel_y
);
    localparam [2:0] S_IDLE = 0, S_LUT = 1, S_MULTIPLY = 2,
                     S_SCALE = 3, S_POSITION = 4, S_OUTPUT = 5;
    reg [2:0] state;
    reg [15:0] range_mm;
    reg [1:0] sample_zoom;
    reg [9:0] sin_index, cos_index;
    reg sin_negative, cos_negative;
    reg [15:0] sin_value, cos_value;
    reg [31:0] sin_product, cos_product;
    reg [15:0] x_offset, y_offset;
    reg [15:0] sin_rom [0:900];

    // Initialized synchronous dual-read ROM: suitable for Gowin block RAM.
    initial begin
        `include "sin_lut.vh"
    end
    always @(posedge clk) begin
        if (state == S_LUT) begin
            sin_value <= sin_rom[sin_index];
            cos_value <= sin_rom[cos_index];
        end
    end

    wire [15:0] normalized_angle = angle_cdeg >= 16'd36000
                                ? angle_cdeg - 16'd36000 : angle_cdeg;
    reg [15:0] quarter_angle;
    reg input_sin_negative, input_cos_negative;
    always @* begin
        input_sin_negative = 1'b0;
        input_cos_negative = 1'b0;
        if (normalized_angle < 16'd9000) begin
            quarter_angle = normalized_angle;
        end else if (normalized_angle < 16'd18000) begin
            quarter_angle = 16'd18000 - normalized_angle;
            input_cos_negative = 1'b1;
        end else if (normalized_angle < 16'd27000) begin
            quarter_angle = normalized_angle - 16'd18000;
            input_sin_negative = 1'b1;
            input_cos_negative = 1'b1;
        end else begin
            quarter_angle = 16'd36000 - normalized_angle;
            input_sin_negative = 1'b1;
        end
    end

    wire [9:0] input_sin_index = quarter_angle / 16'd10;
    wire [15:0] range_limit_mm = 16'd2500 << zoom_level;
    wire [31:0] round_bias = 32'd81920 << sample_zoom;
    wire [31:0] x_mm_rounded = (sin_product + round_bias) >> 14;
    wire [31:0] y_mm_rounded = (cos_product + round_bias) >> 14;
    wire signed [17:0] signed_x_offset = $signed({2'b00, x_offset});
    wire signed [17:0] signed_y_offset = $signed({2'b00, y_offset});
    wire signed [17:0] screen_x = sin_negative
        ? CENTER_X - signed_x_offset : CENTER_X + signed_x_offset;
    wire signed [17:0] screen_y = cos_negative
        ? CENTER_Y + signed_y_offset : CENTER_Y - signed_y_offset;

    assign point_ready = state == S_IDLE;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_IDLE;
            pixel_valid <= 1'b0;
            pixel_x <= 0;
            pixel_y <= 0;
            range_mm <= 0;
            sample_zoom <= 2;
            sin_index <= 0;
            cos_index <= 0;
            sin_negative <= 0;
            cos_negative <= 0;
            sin_product <= 0;
            cos_product <= 0;
            x_offset <= 0;
            y_offset <= 0;
        end else begin
            case (state)
                S_IDLE: if (point_valid) begin
                    if (distance_mm != 16'd0 && distance_mm != 16'hffff &&
                        distance_mm >= MIN_DISTANCE_MM &&
                        distance_mm <= range_limit_mm) begin
                        range_mm <= distance_mm;
                        sample_zoom <= zoom_level;
                        sin_index <= input_sin_index;
                        cos_index <= 10'd900 - input_sin_index;
                        sin_negative <= input_sin_negative ^ (CLOCKWISE == 0);
                        cos_negative <= input_cos_negative;
                        state <= S_LUT;
                    end
                end
                S_LUT: state <= S_MULTIPLY;
                S_MULTIPLY: begin
                    sin_product <= range_mm * sin_value;
                    cos_product <= range_mm * cos_value;
                    state <= S_SCALE;
                end
                S_SCALE: begin
                    // floor(floor(v / 10) / 2^zoom) equals
                    // floor(v / (10 * 2^zoom)) for these positive magnitudes.
                    // A constant divide and variable shift preserve rounding
                    // without synthesizing a run-time division circuit.
                    x_offset <= (x_mm_rounded / 32'd10) >> sample_zoom;
                    y_offset <= (y_mm_rounded / 32'd10) >> sample_zoom;
                    state <= S_POSITION;
                end
                S_POSITION: begin
                    if (screen_x >= CENTER_X - 256 && screen_x < CENTER_X + 256 &&
                        screen_y >= CENTER_Y - 256 && screen_y < CENTER_Y + 256) begin
                        pixel_x <= screen_x[9:0];
                        pixel_y <= screen_y[9:0];
                        pixel_valid <= 1'b1;
                        state <= S_OUTPUT;
                    end else begin
                        state <= S_IDLE;
                    end
                end
                S_OUTPUT: if (pixel_ready) begin
                    pixel_valid <= 1'b0;
                    state <= S_IDLE;
                end
                default: begin
                    state <= S_IDLE;
                    pixel_valid <= 1'b0;
                end
            endcase
        end
    end
endmodule
