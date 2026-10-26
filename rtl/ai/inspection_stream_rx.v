// AIF1 16-bit little-endian containers. Two CRC byte steps per input word;
// payload delivery is a separate ready/valid stage. A failed display frame
// may dirty only the unpublished DDR bank. CONTROL/DIAG use a bounded RAM and
// become visible only after both CRCs pass. No data can address other UI pages.
module inspection_stream_rx #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer IMAGE_W=816,IMAGE_H=420
)(
    input wire clk,rst_n,
    input wire [15:0] i_data,
    input wire i_valid,
    output wire o_ready,
    input wire i_display_enabled,
    input wire [31:0] i_session,i_task,i_epoch,i_context,
    input wire [1:0] i_view,
    output reg o_frame_begin,o_frame_commit,o_frame_abort,
    output reg [31:0] o_sequence,
    input wire i_frame_ready,
    output wire [15:0] o_frame_data,
    output wire o_frame_valid,
    input wire i_frame_word_ready,
    output reg o_message_valid,
    input wire i_message_ready,
    output reg [7:0] o_message_kind,
    output reg [8:0] o_message_words,
    output reg [31:0] o_message_crc,
    output wire [31:0] o_message_session,o_message_task,o_message_epoch,o_message_context,
    output wire [15:0] o_message_view,
    input wire i_message_read,
    input wire [7:0] i_message_address,
    output reg [15:0] o_message_data,
    output reg o_error
);
    localparam SEEK=0,H_INPUT=1,H_LOW=2,H_HIGH=3,H_CHECK=4,
               P_INPUT=5,P_LOW=6,P_HIGH=7,P_SEND=8,FINISH=9,HELD=10;
    localparam integer MAX_IMAGE_BYTES=IMAGE_W*IMAGE_H*2;
    localparam integer GAP_BITS=CLK_HZ<4?1:$clog2(CLK_HZ/2+1);
    reg [3:0] state;
    reg [15:0] header[0:23],message_mem[0:255],word_q;
    reg [4:0] header_index;
    reg [31:0] crc,payload_crc;
    reg [19:0] remaining;
    reg [GAP_BITS-1:0] gap;
    reg [8:0] message_address;
    reg [7:0] kind;
    wire [31:0] length={header[15],header[14]};
    wire [31:0] session={header[5],header[4]},task_id={header[7],header[6]};
    wire [31:0] epoch={header[9],header[8]},context={header[11],header[10]};
    assign o_message_session=session;
    assign o_message_task=task_id;
    assign o_message_epoch=epoch;
    assign o_message_context=context;
    assign o_message_view=header[19];
    wire geometry=header[16]==IMAGE_W && header[17]==IMAGE_H && header[18]==IMAGE_W*2 && length==MAX_IMAGE_BYTES;
    wire display_context=i_display_enabled && session==i_session && task_id==i_task && epoch==i_epoch &&
                         context==i_context && header[19]=={14'd0,i_view};
    wire header_valid=header[2][7:0]==1 && header[3]==48 && (~crc)=={header[23],header[22]} &&
        length>=2 && !length[0] && header[19]<=2;
    wire frame_header=header[2][15:8]==1 && geometry && display_context && session!=0;
    wire message_geometry=header[16]==0 && header[17]==0 && header[18]==0;
    wire control_length=length<=408 && (length[8:0]%9'd12)==0;
    wire message_header=message_geometry && ((header[2][15:8]==4 && length<=512) ||
        (header[2][15:8]==2 && control_length && session!=0 && session==i_session && epoch==i_epoch));
    wire word_sink_ready=kind==1?i_frame_word_ready:1'b1;
    assign o_ready=state==SEEK || state==H_INPUT || state==P_INPUT;
    assign o_frame_valid=state==P_SEND && kind==1;
    assign o_frame_data=word_q;
    function [31:0] crc_byte;
        input [31:0] old;input [7:0] value;reg [31:0] c;integer n;
        begin c=old^{24'd0,value};for(n=0;n<8;n=n+1)c=(c>>1)^(c[0]?32'hedb88320:32'd0);crc_byte=c;end
    endfunction
    always @(posedge clk)if(i_message_read)o_message_data<=message_mem[i_message_address];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=SEEK;header_index<=0;word_q<=0;crc<=32'hffffffff;remaining<=0;payload_crc<=0;
            gap<=0;message_address<=0;kind<=0;o_sequence<=0;
            o_frame_begin<=0;o_frame_commit<=0;o_frame_abort<=0;o_message_valid<=0;o_message_kind<=0;
            o_message_words<=0;o_message_crc<=0;o_error<=0;
        end else begin
            o_frame_begin<=0;o_frame_commit<=0;o_frame_abort<=0;o_error<=0;
            if(state==SEEK || state==HELD || (i_valid && o_ready))gap<=0;
            else if(gap<CLK_HZ/2)gap<=gap+1'b1;
            case(state)
                SEEK:if(i_valid && i_data==16'h4941)begin
                    header[0]<=i_data;word_q<=i_data;header_index<=0;crc<=32'hffffffff;kind<=0;state<=H_LOW;
                end
                H_INPUT:if(i_valid)begin
                    if(header_index==1 && i_data!=16'h3146)begin
                        state<=SEEK;
                        if(i_data==16'h4941)begin
                            header[0]<=i_data;word_q<=i_data;header_index<=0;crc<=32'hffffffff;state<=H_LOW;
                        end
                    end else begin
                        header[header_index]<=i_data;word_q<=i_data;
                        if(header_index<22)state<=H_LOW;
                        else if(header_index==23)state<=H_CHECK;
                        else header_index<=header_index+1'b1;
                    end
                end
                H_LOW:begin crc<=crc_byte(crc,word_q[7:0]);state<=H_HIGH;end
                H_HIGH:begin crc<=crc_byte(crc,word_q[15:8]);header_index<=header_index+1'b1;state<=H_INPUT;end
                H_CHECK:begin
                    if(!header_valid || (!frame_header && !message_header))begin state<=SEEK;o_error<=1;end
                    else if(!frame_header || i_frame_ready)begin
                        kind<=header[2][15:8];remaining<=length[19:0];crc<=32'hffffffff;
                        payload_crc<={header[21],header[20]};message_address<=0;
                        o_sequence<={header[13],header[12]};
                        if(frame_header)o_frame_begin<=1;
                        state<=P_INPUT;
                    end
                end
                P_INPUT:if(i_valid)begin word_q<=i_data;remaining<=remaining-2;state<=P_LOW;end
                P_LOW:begin crc<=crc_byte(crc,word_q[7:0]);state<=P_HIGH;end
                P_HIGH:begin crc<=crc_byte(crc,word_q[15:8]);state<=P_SEND;end
                P_SEND:if(word_sink_ready)begin
                    if(kind!=1)begin message_mem[message_address[7:0]]<=word_q;message_address<=message_address+1'b1;end
                    if(remaining==0)state<=FINISH;else state<=P_INPUT;
                end
                FINISH:begin
                    state<=SEEK;
                    if((~crc)!=payload_crc)begin o_error<=1;if(kind==1)o_frame_abort<=1;end
                    else if(kind==1)begin
                        if(display_context)o_frame_commit<=1;else begin o_frame_abort<=1;o_error<=1;end
                    end else if(kind==4 || (session==i_session && epoch==i_epoch))begin
                        o_message_valid<=1;o_message_kind<=kind;o_message_words<=message_address;o_message_crc<=payload_crc;state<=HELD;
                    end else o_error<=1;
                end
                HELD:if(i_message_ready)begin o_message_valid<=0;state<=SEEK;end
                default:state<=SEEK;
            endcase
            if(gap>=CLK_HZ/2 && state!=SEEK && state!=HELD)begin
                state<=SEEK;o_error<=1;if(kind==1)o_frame_abort<=1;
            end
        end
    end
endmodule
