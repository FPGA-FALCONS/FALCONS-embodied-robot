`timescale 1ns / 1ps

// ACG720 eight-digit display through its two cascaded 74HC595s.
// The shifted word is {DP,G,F,E,D,C,B,A,digit_select[7:0]}.
// Segments are active low, digit selects active high. Digit zero is rightmost.
// CH1 occupies the left four digits; CH2 occupies the right four digits.
// Every register is clocked by clk. sh_cp is an output, never an internal clock.
module rc_hc595_display #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SNAPSHOT_US = 1000,
    parameter integer DIGIT_US = 500,
    parameter integer SCLK_DIVIDER = 25
) (
    input wire clk,
    input wire rst_n,
    input wire [15:0] ch1_width_us,
    input wire [15:0] ch2_width_us,
    input wire ch1_valid,
    input wire ch2_valid,
    output wire ds,
    output wire sh_cp,
    output wire st_cp
);

    function integer counter_width;
        input integer maximum;
        integer remainder;
        begin
            remainder = maximum;
            counter_width = 0;
            while (remainder > 0) begin
                counter_width = counter_width + 1;
                remainder = remainder / 2;
            end
            if (counter_width < 1)
                counter_width = 1;
        end
    endfunction

    // Exact MHz clock required, positive intervals; legal RC widths fit 12 bits.
    localparam integer SNAPSHOT_CYCLES_I = (CLK_HZ / 1000000) * SNAPSHOT_US;
    localparam integer DIGIT_CYCLES_I = (CLK_HZ / 1000000) * DIGIT_US;
    localparam integer SNAPSHOT_BITS = counter_width(SNAPSHOT_CYCLES_I - 1);
    localparam integer DIGIT_BITS = counter_width(DIGIT_CYCLES_I - 1);
    localparam [SNAPSHOT_BITS-1:0] SNAPSHOT_LAST =
        SNAPSHOT_CYCLES_I[SNAPSHOT_BITS-1:0] - 1'b1;
    localparam [DIGIT_BITS-1:0] DIGIT_LAST =
        DIGIT_CYCLES_I[DIGIT_BITS-1:0] - 1'b1;

    reg [SNAPSHOT_BITS-1:0] snapshot_counter;
    reg [DIGIT_BITS-1:0] digit_counter;
    reg [11:0] ch1_binary;
    reg [11:0] ch2_binary;
    reg [15:0] ch1_work_bcd;
    reg [15:0] ch2_work_bcd;
    reg [15:0] ch1_bcd;
    reg [15:0] ch2_bcd;
    reg [3:0] bcd_bit;
    reg bcd_busy;
    reg work_ch1_valid;
    reg work_ch2_valid;
    reg snapshot_ch1_valid;
    reg snapshot_ch2_valid;

    function [15:0] add_three;
        input [15:0] value;
        integer nibble;
        reg [15:0] adjusted;
        begin
            adjusted = value;
            for (nibble = 0; nibble < 4; nibble = nibble + 1) begin
                if (value[nibble*4 +: 4] >= 4'd5)
                    adjusted[nibble*4 +: 4] = value[nibble*4 +: 4] + 4'd3;
            end
            add_three = adjusted;
        end
    endfunction

    // The discarded top bit cannot be set for the documented 0..4095 input.
    wire [15:0] unused_ch1_adjusted = add_three(ch1_work_bcd);
    wire [15:0] unused_ch2_adjusted = add_three(ch2_work_bcd);
    wire [15:0] ch1_bcd_next = {unused_ch1_adjusted[14:0], ch1_binary[11]};
    wire [15:0] ch2_bcd_next = {unused_ch2_adjusted[14:0], ch2_binary[11]};

    // Sequential double-dabble conversion: 12 clocks, then atomic BCD commit.
    // No combinational division by ten in the display data path.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_counter <= {SNAPSHOT_BITS{1'b0}};
            ch1_binary <= 12'd0;
            ch2_binary <= 12'd0;
            ch1_work_bcd <= 16'd0;
            ch2_work_bcd <= 16'd0;
            ch1_bcd <= 16'd0;
            ch2_bcd <= 16'd0;
            bcd_bit <= 4'd0;
            bcd_busy <= 1'b0;
            work_ch1_valid <= 1'b0;
            work_ch2_valid <= 1'b0;
            snapshot_ch1_valid <= 1'b0;
            snapshot_ch2_valid <= 1'b0;
        end else begin
            if (snapshot_counter == SNAPSHOT_LAST) begin
                snapshot_counter <= {SNAPSHOT_BITS{1'b0}};
                if (!bcd_busy) begin
                    ch1_binary <= ch1_width_us[11:0];
                    ch2_binary <= ch2_width_us[11:0];
                    ch1_work_bcd <= 16'd0;
                    ch2_work_bcd <= 16'd0;
                    work_ch1_valid <= ch1_valid && (ch1_width_us <= 16'd4095);
                    work_ch2_valid <= ch2_valid && (ch2_width_us <= 16'd4095);
                    bcd_bit <= 4'd0;
                    bcd_busy <= 1'b1;
                end
            end else begin
                snapshot_counter <= snapshot_counter + 1'b1;
            end

            if (bcd_busy) begin
                ch1_binary <= {ch1_binary[10:0], 1'b0};
                ch2_binary <= {ch2_binary[10:0], 1'b0};
                ch1_work_bcd <= ch1_bcd_next;
                ch2_work_bcd <= ch2_bcd_next;
                if (bcd_bit == 4'd11) begin
                    ch1_bcd <= ch1_bcd_next;
                    ch2_bcd <= ch2_bcd_next;
                    snapshot_ch1_valid <= work_ch1_valid;
                    snapshot_ch2_valid <= work_ch2_valid;
                    bcd_busy <= 1'b0;
                end else begin
                    bcd_bit <= bcd_bit + 1'b1;
                end
            end
        end
    end

    function [7:0] decimal_segment;
        input [3:0] digit;
        begin
            case (digit)
                4'd0: decimal_segment = 8'hc0;
                4'd1: decimal_segment = 8'hf9;
                4'd2: decimal_segment = 8'ha4;
                4'd3: decimal_segment = 8'hb0;
                4'd4: decimal_segment = 8'h99;
                4'd5: decimal_segment = 8'h92;
                4'd6: decimal_segment = 8'h82;
                4'd7: decimal_segment = 8'hf8;
                4'd8: decimal_segment = 8'h80;
                4'd9: decimal_segment = 8'h90;
                default: decimal_segment = 8'hbf;
            endcase
        end
    endfunction

    reg [2:0] digit_index;
    reg [3:0] selected_digit;
    reg selected_valid;
    reg [15:0] serial_word;
    reg serial_start;
    wire serial_busy;
    wire unused_serial_done;

    always @(*) begin
        selected_digit = 4'd0;
        selected_valid = 1'b0;
        case (digit_index)
            3'd0: selected_digit = ch2_bcd[3:0];
            3'd1: selected_digit = ch2_bcd[7:4];
            3'd2: selected_digit = ch2_bcd[11:8];
            3'd3: selected_digit = ch2_bcd[15:12];
            3'd4: selected_digit = ch1_bcd[3:0];
            3'd5: selected_digit = ch1_bcd[7:4];
            3'd6: selected_digit = ch1_bcd[11:8];
            3'd7: selected_digit = ch1_bcd[15:12];
            default: selected_digit = 4'd0;
        endcase
        if (digit_index[2])
            selected_valid = ch1_valid && snapshot_ch1_valid;
        else
            selected_valid = ch2_valid && snapshot_ch2_valid;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            digit_counter <= {DIGIT_BITS{1'b0}};
            digit_index <= 3'd0;
            serial_word <= 16'hff00;
            serial_start <= 1'b0;
        end else begin
            serial_start <= 1'b0;
            if (digit_counter == DIGIT_LAST) begin
                digit_counter <= {DIGIT_BITS{1'b0}};
                if (!serial_busy) begin
                    serial_word <= {
                        selected_valid ? decimal_segment(selected_digit) : 8'hbf,
                        8'b00000001 << digit_index
                    };
                    serial_start <= 1'b1;
                    digit_index <= digit_index + 1'b1;
                end
            end else begin
                digit_counter <= digit_counter + 1'b1;
            end
        end
    end

    rc_hc595_serial #(.SCLK_DIVIDER(SCLK_DIVIDER)) serial_driver (
        .clk(clk), .rst_n(rst_n), .start(serial_start), .word(serial_word),
        .ds(ds), .sh_cp(sh_cp), .st_cp(st_cp),
        .busy(serial_busy), .done(unused_serial_done)
    );
endmodule

// Private helper, colocated with the display; one start is accepted when idle.
/* verilator lint_off DECLFILENAME */
module rc_hc595_serial #(
    parameter integer SCLK_DIVIDER = 25
) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [15:0] word,
    output reg ds,
    output reg sh_cp,
    output reg st_cp,
    output reg busy,
    output reg done
);
    function integer counter_width;
        input integer maximum;
        integer remainder;
        begin
            remainder = maximum;
            counter_width = 0;
            while (remainder > 0) begin
                counter_width = counter_width + 1;
                remainder = remainder / 2;
            end
            if (counter_width < 1)
                counter_width = 1;
        end
    endfunction

    localparam integer DIV_BITS = counter_width(SCLK_DIVIDER - 1);
    localparam [DIV_BITS-1:0] DIV_LAST = SCLK_DIVIDER[DIV_BITS-1:0] - 1'b1;
    localparam [1:0] IDLE = 2'd0, SHIFT = 2'd1, HOLD = 2'd2, LATCH = 2'd3;
    reg [1:0] state;
    reg [DIV_BITS-1:0] divider_counter;
    reg [14:0] shift_word;
    reg [3:0] bit_index;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ds <= 1'b0;
            sh_cp <= 1'b0;
            st_cp <= 1'b0;
            busy <= 1'b0;
            done <= 1'b0;
            state <= IDLE;
            divider_counter <= {DIV_BITS{1'b0}};
            shift_word <= 15'd0;
            bit_index <= 4'd0;
        end else begin
            done <= 1'b0;
            if (state == IDLE) begin
                divider_counter <= {DIV_BITS{1'b0}};
                if (start) begin
                    shift_word <= word[14:0];
                    ds <= word[15];
                    sh_cp <= 1'b0;
                    st_cp <= 1'b0;
                    bit_index <= 4'd15;
                    busy <= 1'b1;
                    state <= SHIFT;
                end
            end else if (divider_counter == DIV_LAST) begin
                divider_counter <= {DIV_BITS{1'b0}};
                case (state)
                    SHIFT: begin
                        if (!sh_cp) begin
                            sh_cp <= 1'b1;
                        end else begin
                            sh_cp <= 1'b0;
                            if (bit_index == 4'd0) begin
                                state <= HOLD;
                            end else begin
                                shift_word <= {shift_word[13:0], 1'b0};
                                ds <= shift_word[14];
                                bit_index <= bit_index - 1'b1;
                            end
                        end
                    end
                    HOLD: begin
                        st_cp <= 1'b1;
                        state <= LATCH;
                    end
                    LATCH: begin
                        st_cp <= 1'b0;
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= IDLE;
                    end
                    default: begin
                        sh_cp <= 1'b0;
                        st_cp <= 1'b0;
                        busy <= 1'b0;
                        state <= IDLE;
                    end
                endcase
            end else begin
                divider_counter <= divider_counter + 1'b1;
            end
        end
    end
endmodule
/* verilator lint_on DECLFILENAME */
