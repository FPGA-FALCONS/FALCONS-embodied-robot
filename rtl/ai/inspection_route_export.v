// Passive copy of the existing 64-waypoint published navigation prefix.
// Payload: uint16 count, then <=64 {uint8 x,uint8 y} cells. No backpressure.
// One back-bank write per clock; count precedes a serialized CRC pass.
module inspection_route_export(
    input wire clk,rst_n,i_enabled,i_valid,i_start,i_done,
    input wire [7:0] i_x,i_y,
    input wire [15:0] i_route_id,i_map_version,
    input wire [31:0] i_session,i_task,i_epoch,i_context,
    input wire [1:0] i_view,
    output reg o_valid,
    input wire i_release,
    output reg [31:0] o_session,o_task,o_epoch,o_context,o_sequence,o_crc,
    output reg [1:0] o_view,
    output reg [6:0] o_words,
    input wire i_read,
    input wire [6:0] i_address,
    output reg [15:0] o_data,
    output reg o_error
);
    localparam IDLE=0,CAPTURE=1,COUNT=2,READ=3,TAKE=4,BITS=5,PUBLISH=6;
    reg [2:0] state;
    reg [15:0] memory[0:255];
    reg front,back;
    reg [6:0] count,address;
    reg [4:0] bits;
    reg [15:0] word_data,crc_data,route_id,map_version;
    reg [31:0] crc,session,task_id,epoch,context;
    reg [1:0] view;
    wire binding=i_enabled && i_session==session && i_task==task_id && i_epoch==epoch && i_context==context && i_view==view && i_route_id==route_id;
    wire start=i_enabled && i_session!=0 && i_start && state==IDLE;
    wire append=i_valid && (start || (state==CAPTURE && binding && !i_start));
    wire [6:0] write_index=start?7'd1:count+1'b1;
    wire write_bank=start?!front:back;
    always @(posedge clk)begin
        if(i_read)o_data<=memory[{front,i_address}];
        if(append && (start || count<64))memory[{write_bank,write_index}]<={i_y,i_x};
        else if(state==COUNT)memory[{back,7'd0}]<={9'd0,count};
        if(state==READ)crc_data<=memory[{back,address}];
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;front<=0;back<=1;count<=0;address<=0;bits<=0;word_data<=0;
            route_id<=0;map_version<=0;crc<=32'hffffffff;session<=0;task_id<=0;epoch<=0;context<=0;view<=0;
            o_valid<=0;o_session<=0;o_task<=0;o_epoch<=0;o_context<=0;o_sequence<=0;o_crc<=0;o_view<=0;o_words<=0;o_error<=0;
        end else begin
            o_error<=0;
            if(i_release)o_valid<=0;
            if(state!=IDLE && !binding)state<=IDLE;
            else case(state)
                IDLE:if(start)begin
                    back<=!front;count<=i_valid?1:0;route_id<=i_route_id;map_version<=i_map_version;
                    session<=i_session;task_id<=i_task;epoch<=i_epoch;context<=i_context;view<=i_view;
                    state<=i_done?COUNT:CAPTURE;
                end
                CAPTURE:begin
                    if(i_start)begin state<=IDLE;o_error<=1;end
                    else if(i_valid && count==64)begin state<=IDLE;o_error<=1;end
                    else begin
                        if(i_valid)count<=count+1'b1;
                        if(i_done)state<=COUNT;
                    end
                end
                COUNT:begin address<=0;crc<=32'hffffffff;state<=READ;end
                READ:state<=TAKE;
                TAKE:begin word_data<=crc_data;bits<=0;state<=BITS;end
                BITS:begin
                    crc<=(crc>>1)^((crc[0]^word_data[0])?32'hedb88320:32'd0);word_data<=word_data>>1;
                    if(bits==15)begin
                        if(address==count)state<=PUBLISH;else begin address<=address+1'b1;state<=READ;end
                    end else bits<=bits+1'b1;
                end
                PUBLISH:if(!o_valid || i_release)begin
                    front<=back;o_valid<=1;o_session<=session;o_task<=task_id;o_epoch<=epoch;o_context<=context;
                    o_sequence<={route_id,map_version};o_crc<=~crc;o_view<=view;o_words<=count+1'b1;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
