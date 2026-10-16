// RC-only diagnostic CSV. One atomic snapshot per line; UART backpressure is respected.
// RC,view,ok,mode,steer,throttle,ch4,ch2,ch5,ch7,ch6,unused,valid_mask,timeout_mask,events
// Each field is a sign and five decimal digits. Output is NOT the production CSV protocol.
`timescale 1ns/1ps
module rc_probe_report #(
    parameter integer CLK_HZ=50_000_000, BAUD=115200, REPORT_HZ=10
)(
    input wire clk,rst_n,i_view,i_signal_valid,
    input wire [1:0] i_mode,
    input wire signed [11:0] i_steer,i_throttle,
    input wire [95:0] i_chan_us,
    input wire [5:0] i_valid,i_timeout,
    input wire [3:0] i_events,
    output wire o_uart_tx
);
    localparam integer REPORT_TICKS=(CLK_HZ/REPORT_HZ>0)?CLK_HZ/REPORT_HZ:1;
    localparam IDLE=0,PREFIX=1,LOAD=2,BCD=3,SEND=4,SEP=5,CR=6,LF=7;
    reg [2:0] state;
    reg [31:0] tick_count;
    reg signed [31:0] snapshot[0:13];
    reg [3:0] field;
    reg [2:0] character;
    reg [1:0] prefix_index;
    reg [4:0] bits_left;
    reg [15:0] binary;
    reg [19:0] decimal_bcd,adjusted;
    reg negative;
    reg [7:0] tx_byte;
    wire tx_ready;
    wire tx_valid=state==PREFIX || state==SEND || state==SEP || state==CR || state==LF;
    integer d,n;
    always @(*) begin
        adjusted=decimal_bcd;
        for(d=0;d<5;d=d+1)
            if(decimal_bcd[d*4 +:4]>=5) adjusted[d*4 +:4]=decimal_bcd[d*4 +:4]+4'd3;
        tx_byte=8'h20;
        case(state)
            PREFIX: case(prefix_index) 0:tx_byte="R";1:tx_byte="C";default:tx_byte=",";endcase
            SEND: case(character)
                0:tx_byte=negative ? "-" : "+";
                1:tx_byte=8'h30+{4'd0,decimal_bcd[19:16]};
                2:tx_byte=8'h30+{4'd0,decimal_bcd[15:12]};
                3:tx_byte=8'h30+{4'd0,decimal_bcd[11:8]};
                4:tx_byte=8'h30+{4'd0,decimal_bcd[7:4]};
                default:tx_byte=8'h30+{4'd0,decimal_bcd[3:0]};
            endcase
            SEP:tx_byte=",";
            CR:tx_byte=8'h0d;
            LF:tx_byte=8'h0a;
            default:tx_byte=8'h20;
        endcase
    end
    uart_byte_tx #(.CLK_FREQ_HZ(CLK_HZ),.BAUD_RATE(BAUD)) u_tx(
        .i_clock(clk),.i_reset_n(rst_n),.i_tx_data(tx_byte),.i_tx_valid(tx_valid),
        .o_tx_ready(tx_ready),.o_uart_tx(o_uart_tx));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;tick_count<=0;field<=0;character<=0;prefix_index<=0;
            bits_left<=0;binary<=0;decimal_bcd<=0;negative<=0;
            for(n=0;n<14;n=n+1) snapshot[n]<=0;
        end else begin
            if(tick_count==REPORT_TICKS-1) tick_count<=0;else tick_count<=tick_count+1'b1;
            case(state)
                IDLE:if(tick_count==0) begin
                    snapshot[0]<={31'd0,i_view};snapshot[1]<={31'd0,i_signal_valid};
                    snapshot[2]<={30'd0,i_mode};snapshot[3]<={{20{i_steer[11]}},i_steer};
                    snapshot[4]<={{20{i_throttle[11]}},i_throttle};
                    for(n=0;n<6;n=n+1) snapshot[5+n]<={16'd0,i_chan_us[n*16 +:16]};
                    snapshot[11]<={26'd0,i_valid};snapshot[12]<={26'd0,i_timeout};
                    snapshot[13]<={28'd0,i_events};field<=0;prefix_index<=0;state<=PREFIX;
                end
                PREFIX:if(tx_ready) begin
                    if(prefix_index==2) state<=LOAD;else prefix_index<=prefix_index+1'b1;
                end
                LOAD:begin
                    negative<=snapshot[field]<0;
                    binary<=(snapshot[field]<0) ? -snapshot[field] : snapshot[field];
                    decimal_bcd<=0;bits_left<=16;state<=BCD;
                end
                BCD:begin
                    decimal_bcd<={adjusted[18:0],binary[15]};binary<={binary[14:0],1'b0};
                    bits_left<=bits_left-1'b1;
                    if(bits_left==1) begin character<=0;state<=SEND;end
                end
                SEND:if(tx_ready) begin
                    if(character==5) state<=(field==13)?CR:SEP;
                    else character<=character+1'b1;
                end
                SEP:if(tx_ready) begin field<=field+1'b1;state<=LOAD;end
                CR:if(tx_ready) state<=LF;
                LF:if(tx_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
