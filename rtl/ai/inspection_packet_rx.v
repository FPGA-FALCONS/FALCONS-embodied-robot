// Bounded inspection transactions carried by reserved MOT1 envelopes 60..62.
// CRC16 is checked per byte; transaction CRC32 advances one BYTE per clock.
// At least 12 input bytes separate complete words, so four CRC steps fit even
// on a byte-per-clock stream. Message RAM has a synchronous read port.
module inspection_packet_rx #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer GAP_CYCLES=CLK_HZ/50
)(
    input wire clk,rst_n,
    input wire [7:0] i_data,
    input wire i_valid,
    output wire o_ready,
    output reg o_message_valid,
    input wire i_message_ready,
    output reg [7:0] o_kind,o_sequence,
    output reg [5:0] o_count,
    input wire i_word_read,
    input wire [4:0] i_word_index,
    output reg [31:0] o_word,
    output reg o_error
);
    reg [7:0] frame[0:10];
    reg [3:0] position;
    reg [15:0] frame_crc;
    reg [31:0] gap,message_crc,crc_word;
    reg [2:0] crc_bytes;
    reg staging;
    reg [5:0] received;
    reg [31:0] words[0:31];
    function [15:0] crc16_byte;
        input [15:0] old;input [7:0] value;reg [15:0] c;integer k;
        begin c=old^{value,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc16_byte=c;end
    endfunction
    function [31:0] crc32_byte;
        input [31:0] old;input [7:0] value;reg [31:0] c;integer k;
        begin c=old^{24'd0,value};for(k=0;k<8;k=k+1)c=(c>>1)^(c[0]?32'hedb88320:32'd0);crc32_byte=c;end
    endfunction
    function [7:0] magic;
        input [3:0] p;
        begin case(p) 0:magic=8'h4d;1:magic=8'h4f;2:magic=8'h54;default:magic=8'h31;endcase end
    endfunction
    wire [31:0] payload={frame[6],frame[7],frame[8],frame[9]};
    assign o_ready=!o_message_valid;
    always @(posedge clk) if(i_word_read) o_word<=words[i_word_index];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            position<=0;frame_crc<=16'hffff;gap<=0;message_crc<=32'hffffffff;
            crc_word<=0;crc_bytes<=0;staging<=0;received<=0;
            o_message_valid<=0;o_kind<=0;o_sequence<=0;o_count<=0;o_error<=0;
        end else begin
            o_error<=0;
            if(o_message_valid && i_message_ready)o_message_valid<=0;
            if(crc_bytes!=0) begin
                message_crc<=crc32_byte(message_crc,crc_word[31:24]);
                crc_word<=crc_word<<8;crc_bytes<=crc_bytes-1'b1;
            end
            if(gap<GAP_CYCLES)gap<=gap+1'b1;
            else if(position!=0 || staging) begin
                position<=0;frame_crc<=16'hffff;staging<=0;crc_bytes<=0;o_error<=1;
            end
            if(i_valid) begin
                gap<=0;
                if(position<4 && i_data!=magic(position)) begin
                    if(i_data==8'h4d)begin position<=1;frame[0]<=i_data;frame_crc<=crc16_byte(16'hffff,i_data);end
                    else begin position<=0;frame_crc<=16'hffff;end
                end else if(position==11) begin
                    position<=0;frame_crc<=16'hffff;
                    if(frame_crc!={frame[10],i_data}) begin staging<=0;o_error<=1;end
                    else if(frame[5]==8'h60) begin
                        if(o_message_valid || frame[6]==0 || frame[7]==0 || frame[7]>32 || frame[8]!=0 || frame[9]!=0) begin
                            staging<=0;o_error<=1;
                        end else begin
                            staging<=1;received<=0;o_kind<=frame[6];o_count<=frame[7][5:0];o_sequence<=frame[4];
                            message_crc<=32'hffffffff;crc_word<=payload;crc_bytes<=4;
                        end
                    end else if(frame[5]==8'h61) begin
                        if(!staging || frame[4]!=o_sequence || received>=o_count || crc_bytes!=0) begin staging<=0;o_error<=1;end
                        else begin
                            words[received[4:0]]<=payload;received<=received+1'b1;
                            crc_word<=payload;crc_bytes<=4;
                        end
                    end else if(frame[5]==8'h62) begin
                        staging<=0;
                        if(staging && frame[4]==o_sequence && received==o_count && crc_bytes==0 && (~message_crc)==payload)
                            o_message_valid<=1;
                        else o_error<=1;
                    end
                end else begin
                    frame[position]<=i_data;position<=position+1'b1;
                    if(position<10)frame_crc<=position==0?crc16_byte(16'hffff,i_data):crc16_byte(frame_crc,i_data);
                end
            end
        end
    end
endmodule
