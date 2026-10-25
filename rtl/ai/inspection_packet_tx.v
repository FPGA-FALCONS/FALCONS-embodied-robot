// Up to eight telemetry words, emitted as one indivisible transaction.
// Payload selection is registered before byte generation; both CRCs advance
// on accepted bytes, so arbitrary UART backpressure preserves checksums.
module inspection_packet_tx(
    input wire clk,rst_n,
    input wire i_start,
    input wire [7:0] i_kind,
    input wire [3:0] i_count,
    input wire [255:0] i_words,
    output wire o_busy,
    output reg [7:0] o_data,
    output wire o_valid,o_last,
    input wire i_ready
);
    localparam IDLE=0,PREP=1,SEND=2;
    reg [1:0] state;
    reg [255:0] words;
    reg [31:0] payload,message_crc;
    reg [15:0] crc;
    reg [7:0] kind,sequence;
    reg [3:0] count,index,byte_index;
    function [15:0] crc16_byte;
        input [15:0] old;input [7:0] value;integer k;reg [15:0] c;
        begin c=old^{value,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc16_byte=c;end
    endfunction
    function [31:0] crc32_byte;
        input [31:0] old;input [7:0] value;integer k;reg [31:0] c;
        begin c=old^{24'd0,value};for(k=0;k<8;k=k+1)c=(c>>1)^(c[0]?32'hedb88320:32'd0);crc32_byte=c;end
    endfunction
    wire final_frame=index==count+1;
    assign o_busy=state!=IDLE;
    assign o_valid=state==SEND;
    assign o_last=o_valid && final_frame && byte_index==11;
    always @* begin
        case(byte_index)
            0:o_data=8'h4d;1:o_data=8'h4f;2:o_data=8'h54;3:o_data=8'h31;
            4:o_data=sequence;5:o_data=index==0?8'h60:final_frame?8'h62:8'h61;
            6:o_data=payload[31:24];7:o_data=payload[23:16];8:o_data=payload[15:8];9:o_data=payload[7:0];
            10:o_data=crc[15:8];default:o_data=crc[7:0];
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin state<=IDLE;words<=0;payload<=0;message_crc<=32'hffffffff;crc<=16'hffff;
            kind<=0;sequence<=0;count<=0;index<=0;byte_index<=0;end
        else case(state)
            IDLE:if(i_start && i_count>0 && i_count<=8 && i_kind!=0)begin
                words<=i_words;kind<=i_kind;count<=i_count;index<=0;message_crc<=32'hffffffff;state<=PREP;
            end
            PREP:begin
                if(index==0)payload<={kind,4'd0,count,16'd0};
                else if(final_frame)payload<=~message_crc;
                else payload<=words[(index-1)*32+:32];
                crc<=16'hffff;byte_index<=0;state<=SEND;
            end
            SEND:if(i_ready)begin
                if(byte_index<10)crc<=crc16_byte(crc,o_data);
                if(byte_index>=6 && byte_index<=9 && !final_frame)message_crc<=crc32_byte(message_crc,o_data);
                if(byte_index==11)begin
                    if(final_frame)begin state<=IDLE;sequence<=sequence+1'b1;end
                    else begin index<=index+1'b1;state<=PREP;end
                end else byte_index<=byte_index+1'b1;
            end
            default:state<=IDLE;
        endcase
    end
endmodule
