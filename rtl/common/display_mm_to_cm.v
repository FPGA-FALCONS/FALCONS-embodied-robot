// 显示专用毫米转厘米：有符号向零取整，连续输入，三拍流水。
// floor(x * 0x66666667 / 2^34) + (x < 0) 在 int32 全范围等于 x / 10。
module display_mm_to_cm (
    input  wire               clk,
    input  wire               reset_n,
    input  wire signed [31:0]  mm,
    output reg  signed [31:0]  cm
);
    reg signed [31:0] sample_mm;
    reg signed [63:0] product;
    wire signed [31:0] quotient = product >>> 34;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_mm <= 0;
            product   <= 0;
            cm        <= 0;
        end else begin
            sample_mm <= mm;
            product   <= sample_mm * 32'sd1717986919;
            cm        <= quotient + $signed({31'd0, product[63]});
        end
    end
endmodule
