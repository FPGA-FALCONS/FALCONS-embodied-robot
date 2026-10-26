// Independently measure IFCLK against the board system clock. Registered Gray
// count crosses two synchronizers, then balanced XOR reductions are registered
// before subtraction. USB diagnostics report this count, not an assumed USB
// bus speed or a firmware product string.
module inspection_fx2_clock #(
    parameter integer CLK_HZ=50_000_000
)(
    input wire clk,rst_n,fx2_clk,
    output reg [31:0] o_hz,
    output reg o_valid,o_alive
);
    localparam WB=CLK_HZ<2?1:$clog2(CLK_HZ);
    reg [31:0] count,gray;
    wire [31:0] next_count=count+1'b1;
    (* ASYNC_REG="TRUE" *) reg [31:0] gray_1,gray_2;
    reg [31:0] binary,last;
    reg [WB-1:0] window;
    localparam LOSS_CYCLES=CLK_HZ/100000<16?16:CLK_HZ/100000;
    localparam LB=$clog2(LOSS_CYCLES+1);
    reg [LB-1:0] quiet;
    reg [31:0] previous_gray;
    wire [31:0] decoded;
    genvar bit_index;
    generate for(bit_index=0;bit_index<32;bit_index=bit_index+1)begin:g_decode
        assign decoded[bit_index]=^(gray_2>>bit_index);
    end endgenerate
    always @(posedge fx2_clk or negedge rst_n)begin
        if(!rst_n)begin count<=0;gray<=0;end
        else begin count<=next_count;gray<=(next_count>>1)^next_count;end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin gray_1<=0;gray_2<=0;binary<=0;last<=0;window<=0;o_hz<=0;o_valid<=0;quiet<=0;previous_gray<=0;o_alive<=0;end
        else begin
            gray_1<=gray;gray_2<=gray_1;binary<=decoded;
            previous_gray<=gray_2;
            if(gray_2!=previous_gray)begin quiet<=0;o_alive<=1;end
            else if(quiet<LOSS_CYCLES)quiet<=quiet+1'b1;else o_alive<=0;
            if(window==CLK_HZ-1)begin window<=0;o_hz<=binary-last;last<=binary;o_valid<=1;end
            else window<=window+1'b1;
        end
    end
endmodule
