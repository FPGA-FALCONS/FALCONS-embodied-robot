// Pixel-side descriptor handoff and two-block lookahead. FIFO reset asserts
// asynchronously and releases through this clock. Failed prefetch hides the
// whole remaining content frame instead of shifting pixels across a scanline.
module inspection_frame_pixels #(
    parameter integer FIFO_DEPTH=128,
    parameter integer X0=208,Y0=120,WIDTH=816,HEIGHT=420
)(
    input wire clk,rst_n,
    input wire i_frame_sync,i_pixel_req,
    input wire [11:0] i_x,i_y,
    input wire i_show,
    input wire [31:0] i_session,i_task,i_epoch,i_context,
    input wire [1:0] i_view,
    input wire i_publish_toggle,
    input wire [175:0] i_publish_metadata,
    output reg o_tick,o_publish_ack,
    input wire i_fifo_reset_n,
    input wire i_fifo_empty,i_fifo_valid,
    input wire [127:0] i_fifo_data,
    output wire o_fifo_read,
    output wire [15:0] o_pixel,
    output wire o_pixel_valid,
    output reg o_underflow
);
    (* ASYNC_REG="TRUE" *) reg [1:0] publication_sync;
    (* ASYNC_REG="TRUE" *) reg [2:0] data_reset;
    reg frame_sync_q;
    reg [175:0] metadata;
    reg has_publication;
    reg [127:0] current_block,next_block;
    reg current_valid,next_valid,pending;
    reg [2:0] index;
    reg [161:0] binding_1,binding_2;
    reg show_1,show_2;
    wire boundary=i_frame_sync && !frame_sync_q;
    wire region=i_pixel_req && i_x>=X0 && i_x<X0+WIDTH && i_y>=Y0 && i_y<Y0+HEIGHT;
    // Metadata is session/task/epoch/context/sequence/view. Sequence occupies
    // 128..159 and is not part of the current live task binding.
    wire task_match=has_publication && metadata[127:0]==binding_2[127:0] && metadata[161:160]==binding_2[161:160];
    wire consume=region && current_valid && !o_underflow;
    wire load=(!current_valid || (consume && index==7)) && next_valid;
    assign o_fifo_read=data_reset[2] && !next_valid && !pending && !i_fifo_empty;
    assign o_pixel=current_block[15:0];
    assign o_pixel_valid=data_reset[2] && show_2 && task_match && region && current_valid && !o_underflow;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            publication_sync<=0;frame_sync_q<=0;metadata<=0;has_publication<=0;o_tick<=0;o_publish_ack<=0;
            binding_1<=0;binding_2<=0;show_1<=0;show_2<=0;
        end else begin
            publication_sync<={publication_sync[0],i_publish_toggle};frame_sync_q<=i_frame_sync;
            binding_1<={i_view,32'd0,i_context,i_epoch,i_task,i_session};binding_2<=binding_1;
            show_1<=i_show;show_2<=show_1;
            if(boundary)begin
                o_tick<=!o_tick;
                if(publication_sync[1]!=o_publish_ack)begin
                    metadata<=i_publish_metadata;has_publication<=1;o_publish_ack<=publication_sync[1];
                end
            end
        end
    end
    wire data_rst_n=rst_n && i_fifo_reset_n;
    always @(posedge clk or negedge data_rst_n)begin
        if(!data_rst_n)data_reset<=0;else data_reset<={data_reset[1:0],1'b1};
    end
    always @(posedge clk or negedge data_rst_n)begin
        if(!data_rst_n)begin
            current_block<=0;next_block<=0;current_valid<=0;next_valid<=0;pending<=0;index<=0;o_underflow<=0;
        end else if(data_reset[2])begin
            if(o_fifo_read)pending<=1;
            if(i_fifo_valid)begin next_block<=i_fifo_data;next_valid<=1;pending<=0;end
            if(load)begin
                current_block<=next_block;current_valid<=1;next_valid<=0;index<=0;
            end else if(consume)begin
                current_block<=current_block>>16;
                if(index==7)begin current_valid<=0;index<=0;end else index<=index+1'b1;
            end
            if(region && !current_valid)o_underflow<=1;
            if(boundary)begin current_valid<=0;next_valid<=0;o_underflow<=0;end
        end
    end
endmodule
