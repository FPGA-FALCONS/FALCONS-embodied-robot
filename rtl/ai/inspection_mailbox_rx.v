// Replays one CRC-approved USB CONTROL mailbox into its own MOT1 validator.
// UART bytes never enter this parser. Only layer kinds 3/4/5 leave this port;
// HELLO, heartbeat, UI and motion authority remain exclusively on UART.
module inspection_mailbox_rx #(
    parameter integer CLK_HZ=50_000_000
)(
    input wire clk,rst_n,
    input wire i_valid,
    input wire [8:0] i_words,
    output reg o_release,
    output wire o_read,
    output wire [7:0] o_address,
    input wire [15:0] i_data,
    output wire o_valid,
    input wire i_ready,
    output wire [7:0] o_kind,o_sequence,
    output wire [5:0] o_count,
    input wire i_word_read,
    input wire [4:0] i_word_index,
    output wire [31:0] o_word,
    output reg o_error
);
    localparam IDLE=0,RESET=1,READ=2,TAKE=3,LOW=4,HIGH=5,CHECK=6,HOLD=7,RELEASE=8,WAIT_EMPTY=9;
    reg [3:0] state;
    reg [8:0] words,index;
    reg [15:0] word_data;
    reg [3:0] settle;
    reg parser_reset_n;
    wire parsed,parser_ready,parser_error;
    wire allowed=o_kind==3 || o_kind==4 || o_kind==5;
    assign o_valid=state==HOLD && parsed && allowed;
    assign o_read=state==READ;
    assign o_address=index[7:0];
    inspection_packet_rx #(.CLK_HZ(CLK_HZ)) parser(
        .clk(clk),.rst_n(rst_n && parser_reset_n),.i_data(state==LOW?word_data[7:0]:word_data[15:8]),
        .i_valid(state==LOW || state==HIGH),.o_ready(parser_ready),
        .o_message_valid(parsed),.i_message_ready(state==HOLD && i_ready),
        .o_kind(o_kind),.o_sequence(o_sequence),.o_count(o_count),
        .i_word_read(i_word_read),.i_word_index(i_word_index),.o_word(o_word),.o_error(parser_error));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;words<=0;index<=0;word_data<=0;settle<=0;parser_reset_n<=0;o_release<=0;o_error<=0;end
        else begin
            o_release<=0;o_error<=0;
            case(state)
                IDLE:if(i_valid)begin words<=i_words;index<=0;parser_reset_n<=0;state<=RESET;end
                RESET:begin
                    parser_reset_n<=1;
                    if(words<18 || words>204 || words%6!=0)begin o_error<=1;state<=RELEASE;end
                    else state<=READ;
                end
                READ:state<=TAKE;
                TAKE:begin word_data<=i_data;state<=LOW;end
                LOW:state<=HIGH;
                HIGH:if(index+1==words)begin settle<=0;state<=CHECK;end
                    else if(parsed)begin o_error<=1;state<=RELEASE;end // No multiple transactions per container.
                    else begin index<=index+1'b1;state<=READ;end
                CHECK:if(parsed)begin
                    if(allowed)begin o_release<=1;state<=HOLD;end
                    else begin o_error<=1;state<=RELEASE;end
                end else if(settle==7)begin o_error<=1;state<=RELEASE;end
                    else settle<=settle+1'b1;
                HOLD:if(i_ready)begin parser_reset_n<=0;state<=IDLE;end
                RELEASE:begin o_release<=1;parser_reset_n<=0;state<=WAIT_EMPTY;end
                WAIT_EMPTY:if(!i_valid)state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
