// One complete frame uses one atomic glyph snapshot. An all-dash invalid
// indication overrides queued normal digits immediately, without waiting for
// the next snapshot. Only the external sh_cp pin is toggled; clk clocks RTL.
module tof_hc595_display #(
    parameter integer CLK_HZ=50000000,
    parameter integer DIGIT_US=500,
    parameter integer SCLK_DIVIDER=25
) (
    input wire clk,
    input wire rst_n,
    input wire [63:0] glyphs,
    output wire ds,
    output wire sh_cp,
    output wire st_cp
);
    function integer counter_width;
        input integer maximum;
        integer remainder;
        begin
            remainder=maximum; counter_width=0;
            while(remainder>0) begin
                counter_width=counter_width+1; remainder=remainder/2;
            end
            if(counter_width<1) counter_width=1;
        end
    endfunction
    localparam integer DIGIT_CYCLES=(CLK_HZ/1000000)*DIGIT_US;
    localparam integer DIGIT_BITS=counter_width(DIGIT_CYCLES-1);
    localparam [DIGIT_BITS-1:0] DIGIT_LAST=DIGIT_CYCLES[DIGIT_BITS-1:0]-1'b1;
    localparam [63:0] DASHES=64'hbfbfbfbfbfbfbfbf;
    reg [DIGIT_BITS-1:0] digit_counter;
    reg [2:0] digit_index;
    reg [63:0] frame_glyphs;
    reg frame_invalid;
    reg [15:0] serial_word;
    reg serial_start;
    wire serial_busy;
    wire unused_serial_done;
    wire [63:0] selected_frame=(digit_index==3'd0) ? glyphs : frame_glyphs;
    wire [7:0] selected_segment=((glyphs==DASHES) ||
        ((digit_index!=3'd0) && frame_invalid)) ? 8'hbf :
        selected_frame[digit_index*8 +: 8];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            digit_counter<={DIGIT_BITS{1'b0}};
            digit_index<=3'd0; frame_glyphs<=DASHES; frame_invalid<=1'b1;
            serial_word<=16'hff00; serial_start<=1'b0;
        end else begin
            serial_start<=1'b0;
            if(glyphs==DASHES) frame_invalid<=1'b1;
            if(digit_counter==DIGIT_LAST) begin
                digit_counter<={DIGIT_BITS{1'b0}};
                if(!serial_busy) begin
                    if(digit_index==3'd0) begin
                        frame_glyphs<=glyphs;
                        frame_invalid<=(glyphs==DASHES);
                    end
                    serial_word<={selected_segment,8'b1<<digit_index};
                    serial_start<=1'b1;
                    digit_index<=digit_index+1'b1;
                end
            end else digit_counter<=digit_counter+1'b1;
        end
    end
    tof_hc595_serial #(.SCLK_DIVIDER(SCLK_DIVIDER)) serial_driver (
        .clk(clk),.rst_n(rst_n),.start(serial_start),.word(serial_word),
        .ds(ds),.sh_cp(sh_cp),.st_cp(st_cp),.busy(serial_busy),
        .done(unused_serial_done)
    );
endmodule

// Private helper, colocated with the display; one start is accepted when idle.
/* verilator lint_off DECLFILENAME */
module tof_hc595_serial #(
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
