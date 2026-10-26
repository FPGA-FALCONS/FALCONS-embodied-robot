// AIF1 sender for bounded diagnostics and complete radar maps. Payload CRC is
// supplied by the snapshot producer. Header CRC advances one byte per clock;
// synchronous payload RAM requests and ready/valid output are separate stages.
module inspection_stream_tx(
    input wire clk,rst_n,i_start,
    input wire [7:0] i_kind,
    input wire [31:0] i_session,i_task,i_epoch,i_context,i_sequence,i_bytes,i_crc,
    input wire [15:0] i_width,i_height,i_stride,i_view,
    output wire o_busy,o_payload_read,
    output reg [11:0] o_payload_address,
    input wire [15:0] i_payload_data,
    output reg [15:0] o_data,
    output wire o_valid,o_last,
    input wire i_ready
);
    localparam IDLE=0,C_READ=1,C_LOW=2,C_HIGH=3,H_READ=4,H_SEND=5,P_READ=6,P_LATCH=7,P_SEND=8;
    reg [3:0] state;
    reg [15:0] header[0:23],crc_word;
    reg [4:0] index;
    reg [31:0] crc;
    reg [12:0] words;
    assign o_busy=state!=IDLE;
    assign o_payload_read=state==P_READ;
    assign o_valid=state==H_SEND || state==P_SEND;
    assign o_last=state==P_SEND && {1'b0,o_payload_address}+1'b1==words;
    function [31:0] crc_byte;
        input [31:0] old;input [7:0] value;reg [31:0] c;integer n;
        begin c=old^{24'd0,value};for(n=0;n<8;n=n+1)c=(c>>1)^(c[0]?32'hedb88320:32'd0);crc_byte=c;end
    endfunction
    wire [31:0] next_crc=crc_byte(crc,crc_word[15:8]);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;index<=0;crc_word<=0;crc<=32'hffffffff;words<=0;o_payload_address<=0;o_data<=0;end
        else case(state)
            IDLE:if(i_start && i_bytes>=2 && i_bytes<=8192 && !i_bytes[0])begin
                header[0]<=16'h4941;header[1]<=16'h3146;header[2]<={i_kind,8'd1};header[3]<=48;
                header[4]<=i_session[15:0];header[5]<=i_session[31:16];header[6]<=i_task[15:0];header[7]<=i_task[31:16];
                header[8]<=i_epoch[15:0];header[9]<=i_epoch[31:16];header[10]<=i_context[15:0];header[11]<=i_context[31:16];
                header[12]<=i_sequence[15:0];header[13]<=i_sequence[31:16];header[14]<=i_bytes[15:0];header[15]<=i_bytes[31:16];
                header[16]<=i_width;header[17]<=i_height;header[18]<=i_stride;header[19]<=i_view;
                header[20]<=i_crc[15:0];header[21]<=i_crc[31:16];
                crc<=32'hffffffff;index<=0;words<=i_bytes[13:1];o_payload_address<=0;state<=C_READ;
            end
            C_READ:begin crc_word<=header[index];state<=C_LOW;end
            C_LOW:begin crc<=crc_byte(crc,crc_word[7:0]);state<=C_HIGH;end
            C_HIGH:begin
                crc<=next_crc;
                if(index==21)begin header[22]<=~next_crc[15:0];header[23]<=~next_crc[31:16];index<=0;state<=H_READ;end
                else begin index<=index+1'b1;state<=C_READ;end
            end
            H_READ:begin o_data<=header[index];state<=H_SEND;end
            H_SEND:if(i_ready)begin
                if(index==23)state<=P_READ;else begin index<=index+1'b1;state<=H_READ;end
            end
            P_READ:state<=P_LATCH;
            P_LATCH:begin o_data<=i_payload_data;state<=P_SEND;end
            P_SEND:if(i_ready)begin
                if(o_last)state<=IDLE;
                else begin o_payload_address<=o_payload_address+1'b1;state<=P_READ;end
            end
            default:state<=IDLE;
        endcase
    end
endmodule
