// DDR-domain back-bank writer. Only a complete ordered, CRC-approved frame
// requests publication. The previous front bank is not reused until the read
// engine acknowledges its actual response drain and frame-boundary handoff.
module inspection_frame_writer #(
    parameter integer PIXELS=816*420,
    parameter [27:0] BANK0=28'h100000,BANK1=28'h180000
)(
    input wire clk,rst_n,i_initialized,
    input wire [17:0] i_token,
    input wire i_token_valid,
    output wire o_token_ready,
    output wire o_bank_ready,
    output reg o_done_toggle,
    output reg o_write_valid,
    output wire [27:0] o_write_address,
    output reg [127:0] o_write_data,
    input wire i_write_ready,
    output reg o_publish_toggle,o_publish_bank,
    output reg [175:0] o_publish_metadata,
    input wire i_publish_done,
    output reg o_error
);
    localparam IDLE=0,META=1,DATA=2,WRITE=3,PUBLISH=4,DROP=5;
    localparam integer BLOCKS=PIXELS/8;
    reg [2:0] state,word_index;
    reg [3:0] metadata_index;
    reg [15:0] block_index;
    reg back_bank;
    reg [175:0] metadata;
    assign o_bank_ready=state==IDLE && i_initialized;
    assign o_token_ready=i_initialized && (state==IDLE || state==META || state==DATA || state==DROP);
    assign o_write_address=(back_bank?BANK1:BANK0)+{9'd0,block_index,3'b000};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;word_index<=0;metadata_index<=0;block_index<=0;back_bank<=1;
            metadata<=0;o_done_toggle<=0;o_write_valid<=0;o_write_data<=0;
            o_publish_toggle<=0;o_publish_bank<=0;o_publish_metadata<=0;o_error<=0;
        end else begin
            o_error<=0;
            if(state==WRITE)begin
                if(i_write_ready)begin o_write_valid<=0;block_index<=block_index+1'b1;word_index<=0;state<=DATA;end
            end else if(state==PUBLISH)begin
                if(i_publish_done)begin back_bank<=!back_bank;o_done_toggle<=!o_done_toggle;state<=IDLE;end
            end else if(i_token_valid && o_token_ready)begin
                if(i_token[17:16]==3)begin
                    o_done_toggle<=!o_done_toggle;state<=IDLE;word_index<=0;block_index<=0;
                end else case(state)
                    IDLE:begin
                        word_index<=0;block_index<=0;metadata_index<=1;
                        if(i_token[17:16]==1)begin metadata<={160'd0,i_token[15:0]};state<=META;end
                        else begin state<=DROP;o_error<=1;end
                    end
                    META:if(i_token[17:16]==1)begin
                        metadata[metadata_index*16+:16]<=i_token[15:0];
                        if(metadata_index==10)state<=DATA;else metadata_index<=metadata_index+1'b1;
                    end else begin state<=DROP;o_error<=1;end
                    DATA:if(i_token[17:16]==0 && block_index<BLOCKS)begin
                        o_write_data[word_index*16+:16]<=i_token[15:0];
                        if(word_index==7)begin o_write_valid<=1;state<=WRITE;end
                        else word_index<=word_index+1'b1;
                    end else if(i_token[17:16]==2)begin
                        if(block_index==BLOCKS && word_index==0)begin
                            o_publish_bank<=back_bank;o_publish_metadata<=metadata;
                            o_publish_toggle<=!o_publish_toggle;state<=PUBLISH;
                        end else begin o_error<=1;o_done_toggle<=!o_done_toggle;state<=IDLE;end
                    end else begin state<=DROP;o_error<=1;end
                    DROP:if(i_token[17:16]==2)begin o_done_toggle<=!o_done_toggle;state<=IDLE;end
                    default:state<=IDLE;
                endcase
            end
        end
    end
    initial if(PIXELS<8 || PIXELS%8 || PIXELS/8>65535)$fatal(1,"Complete 128-bit blocks required");
endmodule
