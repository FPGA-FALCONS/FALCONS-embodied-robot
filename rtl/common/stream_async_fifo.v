// Ready/valid wrapper over the existing tested registered-read Gray FIFO.
// One held output plus one pending RAM read; no combinational cross-clock data.
module stream_async_fifo #(
    parameter integer WIDTH=16,DEPTH=1024
)(
    input wire wclk,wrst_n,i_valid,
    input wire [WIDTH-1:0] i_data,
    output wire o_ready,
    input wire rclk,rrst_n,
    output reg o_valid,
    output reg [WIDTH-1:0] o_data,
    input wire i_ready
);
    wire full,empty,read_valid;
    wire [WIDTH-1:0] read_data;
    reg pending;
    wire read_request=(!o_valid || i_ready) && !pending && !empty;
    assign o_ready=!full && wrst_n;
    acg_async_fifo #(.WIDTH(WIDTH),.DEPTH(DEPTH)) memory(
        .wr_clk(wclk),.wr_rst_n(wrst_n),.wr_en(i_valid && o_ready),.wr_data(i_data),.wr_full(full),.wr_free(),
        .rd_clk(rclk),.rd_rst_n(rrst_n),.rd_en(read_request),.rd_data(read_data),.rd_valid(read_valid),.rd_empty(empty));
    always @(posedge rclk or negedge rrst_n)begin
        if(!rrst_n)begin pending<=0;o_valid<=0;o_data<=0;end
        else begin
            if(o_valid && i_ready)o_valid<=0;
            if(read_request)pending<=1;
            if(read_valid)begin pending<=0;o_valid<=1;o_data<=read_data;end
        end
    end
endmodule
