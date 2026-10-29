// Ordered sys->DDR tokens: 01 metadata (11 uint16 words), 00 pixels,
// 10 CRC-approved commit, 11 abort. A completion toggle prevents accepting
// another frame before the old display bank has really been released.
module inspection_frame_tokens(
    input wire clk,rst_n,
    input wire i_bank_ready,i_done_toggle,
    input wire i_begin,i_commit,i_abort,
    input wire [31:0] i_session,i_task,i_epoch,i_context,i_sequence,
    input wire [1:0] i_view,
    output wire o_frame_ready,
    input wire [15:0] i_pixel,
    input wire i_pixel_valid,
    output wire o_pixel_ready,
    output wire [17:0] o_token,
    output wire o_token_valid,
    input wire i_token_ready
);
    localparam IDLE=0,META=1,PIXELS=2,WAIT_DONE=3;
    reg [1:0] state;
    reg [175:0] metadata;
    reg [3:0] metadata_index;
    reg commit_pending,abort_pending,done_baseline;
    assign o_frame_ready=state==IDLE && i_bank_ready;
    assign o_token_valid=state==META || (state==PIXELS && (abort_pending || commit_pending || i_pixel_valid));
    assign o_pixel_ready=state==PIXELS && !abort_pending && !commit_pending && i_token_ready;
    assign o_token=state==META?{2'b01,metadata[15:0]}:
        abort_pending?{2'b11,16'd0}:commit_pending?{2'b10,16'd0}:{2'b00,i_pixel};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;metadata<=0;metadata_index<=0;commit_pending<=0;abort_pending<=0;done_baseline<=0;
        end else begin
            if(state!=IDLE)begin
                if(i_abort)abort_pending<=1;
                if(i_commit)commit_pending<=1;
            end
            case(state)
                IDLE:if(i_begin && i_bank_ready)begin
                    metadata<={14'd0,i_view,i_sequence,i_context,i_epoch,i_task,i_session};
                    metadata_index<=0;commit_pending<=0;abort_pending<=0;done_baseline<=i_done_toggle;state<=META;
                end
                META:if(i_token_ready)begin
                    metadata<=metadata>>16;
                    if(metadata_index==10)state<=PIXELS;else metadata_index<=metadata_index+1'b1;
                end
                PIXELS:if(o_token_valid && i_token_ready && (abort_pending || commit_pending))state<=WAIT_DONE;
                WAIT_DONE:if(i_done_toggle!=done_baseline)state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
