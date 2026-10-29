// DDR-domain display reader. A frame-boundary ticket stops issuing reads,
// drains every accepted response, resets the pixel FIFO, then prefetches the
// selected front bank. Releasing the old bank requires this actual drain.
module inspection_frame_reader #(
    parameter integer BLOCKS=816*420/8,
    parameter [27:0] BANK0=28'h100000,BANK1=28'h180000
)(
    input wire clk,rst_n,i_enable,
    input wire i_pixel_tick,i_pixel_publish_ack,
    input wire i_publish_toggle,i_publish_bank,
    output reg o_publish_done,
    output wire o_read_valid,
    output wire [27:0] o_read_address,
    input wire i_read_ready,i_response_valid,
    input wire [127:0] i_response_data,
    input wire [31:0] i_fifo_free,
    output wire o_fifo_write,
    output wire [127:0] o_fifo_data,
    output reg o_fifo_reset_n,o_fault
);
    localparam WAIT_TICK=0,DRAIN=1,RESET_FIFO=2,RUN=3;
    reg [1:0] state;
    (* ASYNC_REG="TRUE" *) reg [1:0] tick_sync,ack_sync,enable_sync;
    reg last_tick,front_bank,front_valid,front_publication;
    reg [15:0] index;
    reg [8:0] outstanding;
    reg [4:0] reset_count;
    reg [2:0] drain_guard;
    wire tick=tick_sync[1]!=last_tick;
    wire accept=o_read_valid && i_read_ready;
    assign o_read_valid=state==RUN && !tick && front_valid && enable_sync[1] && !o_fault && index<BLOCKS &&
        i_fifo_free[9:0]>{1'b0,outstanding}+10'd4;
    assign o_read_address=(front_bank?BANK1:BANK0)+{9'd0,index,3'b000};
    assign o_fifo_write=state==RUN && !tick && !o_fault && i_response_valid && outstanding!=0;
    assign o_fifo_data=i_response_data;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin tick_sync<=0;ack_sync<=0;enable_sync<=0;end
        else begin tick_sync<={tick_sync[0],i_pixel_tick};ack_sync<={ack_sync[0],i_pixel_publish_ack};enable_sync<={enable_sync[0],i_enable};end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=WAIT_TICK;last_tick<=0;front_bank<=0;front_valid<=0;front_publication<=0;
            index<=0;outstanding<=0;reset_count<=0;drain_guard<=0;
            o_publish_done<=0;o_fifo_reset_n<=0;o_fault<=0;
        end else begin
            o_publish_done<=0;
            if(i_response_valid && outstanding==0)o_fault<=1;
            case({accept,i_response_valid && outstanding!=0})
                2'b10:outstanding<=outstanding+1'b1;
                2'b01:outstanding<=outstanding-1'b1;
                default:begin end
            endcase
            if(accept)index<=index+1'b1;
            if(tick)begin last_tick<=tick_sync[1];state<=DRAIN;drain_guard<=0;end
            else case(state)
                DRAIN:begin
                    if(drain_guard<3)drain_guard<=drain_guard+1'b1;
                    else if(outstanding==0 && !o_fault)begin
                        if(ack_sync[1]==i_publish_toggle && front_publication!=i_publish_toggle)begin
                            front_bank<=i_publish_bank;front_valid<=1;
                            front_publication<=i_publish_toggle;o_publish_done<=1;
                        end
                        o_fifo_reset_n<=0;reset_count<=0;index<=0;state<=RESET_FIFO;
                    end
                end
                RESET_FIFO:if(reset_count==15)begin o_fifo_reset_n<=1;state<=RUN;end
                    else reset_count<=reset_count+1'b1;
                default:begin end
            endcase
        end
    end
endmodule
