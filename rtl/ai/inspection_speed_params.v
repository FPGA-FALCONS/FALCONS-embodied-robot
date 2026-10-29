// RK owns inspection cruise (physical mm/s); base navigation keeps its knobs.
// Snapshot operands, divide over 26 clocks, then register the controller inputs.
module inspection_speed_params (
    input wire clk,rst_n,i_owned,
    input wire [15:0] i_cruise_mm_s,
    input wire signed [31:0] i_base_cruise,i_d_slow,i_base_k_slow,
    output reg signed [31:0] o_cruise,o_k_slow
);
    reg owned_q,start;
    reg [15:0] cruise_q;
    reg [31:0] d_slow_q;
    reg [25:0] numerator,denominator;
    wire [25:0] quotient;
    wire valid;
    div_iter #(.WIDTH(26)) u_gain (
        .clk(clk),.rst_n(rst_n),.i_start(start),.i_a(numerator),.i_b(denominator),
        .o_q(quotient),.o_r(),.o_valid(valid),.o_busy());
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            owned_q<=0;start<=0;cruise_q<=0;d_slow_q<=0;
            numerator<=0;denominator<=0;o_cruise<=0;o_k_slow<=0;
        end else begin
            owned_q<=i_owned;start<=0;
            o_cruise<=i_owned ? $signed({16'd0,i_cruise_mm_s}) : i_base_cruise;
            if(!i_owned)o_k_slow<=i_base_k_slow;
            else begin
                if(!owned_q || cruise_q!=i_cruise_mm_s || d_slow_q!=i_d_slow)begin
                    cruise_q<=i_cruise_mm_s;d_slow_q<=i_d_slow;
                    numerator<={i_cruise_mm_s,10'd0};denominator<=i_d_slow[25:0];start<=1;
                end
                if(valid)o_k_slow<=$signed({6'd0,quotient});
            end
        end
    end
endmodule
