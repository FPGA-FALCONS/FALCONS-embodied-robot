// Localization epoch changes on events, not on the held IMU bias-valid level.
// Zero is reserved for an unbound host; simultaneous origin events coalesce.
module inspection_epoch_tracker(
    input wire clk,rst_n,i_origin_clear,i_calib_done,
    output reg [31:0] o_epoch
);
    reg clear_d,done_d;
    wire advance=(i_origin_clear && !clear_d) || (i_calib_done && !done_d);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin clear_d<=0;done_d<=0;o_epoch<=1;end
        else begin
            clear_d<=i_origin_clear;done_d<=i_calib_done;
            if(advance) o_epoch <= (o_epoch == 32'hffffffff) ? 32'd1 : o_epoch + 1'b1;
        end
    end
endmodule
