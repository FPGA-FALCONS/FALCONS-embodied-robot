// Active-low {DP,G,F,E,D,C,B,A}; byte zero is the rightmost digit.
// The seven-segment approximation of WARN is "uArn", STOP is "5toP".
module tof_display_formatter (
    input wire clk,
    input wire rst_n,
    input wire [15:0] distance_mm,
    input wire range_valid,
    input wire initialized,
    input wire [4:0] fault_code,
    output reg [63:0] glyphs,
    output wire warning,
    output wire stop
);
    localparam [63:0] DASHES = 64'hbfbfbfbfbfbfbfbf;
    wire usable = initialized && range_valid && (fault_code == 5'd0);
    assign warning = usable && (distance_mm >= 16'd100) && (distance_mm < 16'd200);
    assign stop = usable && (distance_mm < 16'd100);
    reg [15:0] binary_work;
    reg [19:0] bcd_work;
    reg [3:0] bit_index;
    reg converting;
    reg have_result;
    reg [31:0] header_work;
    reg overflow_work;
    reg [63:0] result;

    function [7:0] decimal_segment;
        input [3:0] digit;
        begin
            case (digit)
                0: decimal_segment=8'hc0;
                1: decimal_segment=8'hf9;
                2: decimal_segment=8'ha4;
                3: decimal_segment=8'hb0;
                4: decimal_segment=8'h99;
                5: decimal_segment=8'h92;
                6: decimal_segment=8'h82;
                7: decimal_segment=8'hf8;
                8: decimal_segment=8'h80;
                9: decimal_segment=8'h90;
                default: decimal_segment=8'hbf;
            endcase
        end
    endfunction
    function [19:0] add_three;
        input [19:0] value;
        integer nibble;
        begin
            add_three=value;
            for(nibble=0;nibble<5;nibble=nibble+1)
                if(value[nibble*4 +: 4]>=4'd5)
                    add_three[nibble*4 +: 4]=value[nibble*4 +: 4]+4'd3;
        end
    endfunction
    wire [19:0] unused_adjusted = add_three(bcd_work);
    wire [19:0] bcd_next = {unused_adjusted[18:0],binary_work[15]};

    // Sixteen iterations, with header and distance captured together.
    // Invalidity cancels conversion; no earlier session can reappear afterward.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            binary_work<=16'd0; bcd_work<=20'd0; bit_index<=4'd0;
            converting<=1'b0; have_result<=1'b0;
            header_work<=32'hffffffff; overflow_work<=1'b0; result<=DASHES;
        end else if(!usable) begin
            converting<=1'b0;
            have_result<=1'b0;
        end else if(!converting) begin
            binary_work<=distance_mm;
            bcd_work<=20'd0;
            bit_index<=4'd0;
            converting<=1'b1;
            overflow_work<=(distance_mm>16'd9999);
            if(distance_mm<16'd100) header_work<=32'h9287a38c;
            else if(distance_mm<16'd200) header_work<=32'he388afab;
            else header_work<=32'h92888e86;
        end else begin
            binary_work<={binary_work[14:0],1'b0};
            bcd_work<=bcd_next;
            if(bit_index==4'd15) begin
                converting<=1'b0;
                have_result<=1'b1;
                result<={header_work,overflow_work ? 32'hffff89f9 :
                    {decimal_segment(bcd_next[15:12]),decimal_segment(bcd_next[11:8]),
                     decimal_segment(bcd_next[7:4]),decimal_segment(bcd_next[3:0])}};
            end else bit_index<=bit_index+1'b1;
        end
    end

    always @(*) begin
        glyphs=DASHES;
        if((fault_code>=5'd1)&&(fault_code<=5'd9))
            glyphs={32'hff86afaf,24'hc0c0c0,decimal_segment(fault_code[3:0])};
        else if(!initialized)
            glyphs={32'hf9abf987,32'hbfbfbfbf};
        else if(usable && have_result)
            glyphs=result;
    end
endmodule
