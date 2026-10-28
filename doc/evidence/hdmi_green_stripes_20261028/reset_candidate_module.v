// Assert without a running clock; release locally on the serializer clock.
// Keep the OSER10 reset load off the application pixel-domain reset tree.
module hdmi_tx_reset (
    input wire serial_clk,
    input wire reset_n,
    input wire pll_lock,
    output wire tx_reset_n
);
    wire async_reset_n=reset_n && pll_lock;
    (* ASYNC_REG="TRUE" *) reg [2:0] release_sync;
    always @(posedge serial_clk or negedge async_reset_n)begin
        if(!async_reset_n)release_sync<=0;
        else release_sync<={release_sync[1:0],1'b1};
    end
    assign tx_reset_n=release_sync[2];
endmodule
