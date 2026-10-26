// Decode synchronous packet RAM in two-clock reads. Small headers are latched;
// layer data is streamed directly from packet RAM into the serialized writer.
// No 32-word combinational unpack or CRC/geometry is on a control output path.
module inspection_dispatch #(
    parameter integer TIMEOUT_CYCLES=2_500_000
)(
    input wire clk,rst_n,
    input wire i_valid,
    input wire [7:0] i_kind,
    input wire [5:0] i_count,
    output wire o_ready,
    output wire o_read,
    output wire [4:0] o_read_index,
    input wire [31:0] i_word,
    input wire [31:0] i_current_session,i_current_epoch,i_current_task,i_current_command,
    input wire [15:0] i_current_layer,
    input wire i_upload_allowed,i_layer_ready,i_layer_busy,
    output reg o_hello,o_control,o_heartbeat,o_begin,o_commit,o_word_valid,
    output reg [31:0] o_session,o_task,o_epoch,o_command,
    output reg [31:0] o_flags,o_goal_x,o_goal_y,o_layer_speed,o_expected_crc,o_layer_word,
    output reg [12:0] o_word_index,
    output reg [15:0] o_layer_version,
    output reg o_clock_request,
    output reg [31:0] o_clock_nonce,
    output reg o_ui_close,
    output reg [31:0] o_ui_revision,
    output reg o_error
);
    localparam IDLE=0,READ=1,TAKE=2,DECODE=3,DATA_READ=4,DATA_TAKE=5,DATA_SEND=6,FINISH=7,COMMIT_WAIT=8;
    reg [3:0] state;
    reg [7:0] kind;
    reg [5:0] count,index,header_count;
    reg [31:0] header[0:7];
    reg [12:0] data_offset;
    reg [15:0] upload_version;
    reg [31:0] upload_session,upload_epoch;
    localparam integer TIMER_BITS=TIMEOUT_CYCLES<2?1:$clog2(TIMEOUT_CYCLES+1);
    localparam [TIMER_BITS-1:0] TIMER_LIMIT=TIMEOUT_CYCLES;
    reg [31:0] upload_task,upload_command;
    reg [TIMER_BITS-1:0] timer;
    reg upload_context;
    assign o_ready=state==FINISH;
    assign o_read=state==READ || state==DATA_READ;
    assign o_read_index=index[4:0];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;kind<=0;count<=0;index<=0;header_count<=0;data_offset<=0;
            upload_version<=0;upload_session<=0;upload_epoch<=0;upload_context<=0;
            upload_task<=0;upload_command<=0;timer<=0;
            o_clock_request<=0;o_clock_nonce<=0;
            o_ui_close<=0;o_ui_revision<=0;
            o_hello<=0;o_control<=0;o_heartbeat<=0;o_begin<=0;o_commit<=0;o_word_valid<=0;o_error<=0;
            o_session<=0;o_task<=0;o_epoch<=0;o_command<=0;o_flags<=0;o_goal_x<=0;o_goal_y<=0;
            o_layer_speed<=0;o_expected_crc<=0;o_layer_word<=0;o_word_index<=0;o_layer_version<=0;
        end else begin
            o_hello<=0;o_control<=0;o_heartbeat<=0;o_begin<=0;o_commit<=0;o_word_valid<=0;o_error<=0;
            o_clock_request<=0;o_ui_close<=0;
            if(state==IDLE || state==FINISH)timer<=0;
            else if(timer<TIMER_LIMIT)timer<=timer+1'b1;
            if(!i_upload_allowed || i_current_epoch!=upload_epoch || i_current_session!=upload_session)upload_context<=0;
            case(state)
                IDLE: if(i_valid)begin
                    kind<=i_kind;count<=i_count;index<=0;header_count<=i_kind==4?4:i_count;
                    if(i_count==0 || (i_kind==4 && i_count<=4) || (i_kind!=4 && i_count>8))begin state<=FINISH;o_error<=1;end
                    else state<=READ;
                end
                READ: state<=TAKE;
                TAKE: begin
                    header[index[2:0]]<=i_word;
                    if(index+1==header_count)state<=DECODE;
                    else begin index<=index+1'b1;state<=READ;end
                end
                DECODE: begin
                    o_session<=header[0];state<=FINISH;
                    if(kind==1 && count==1) o_hello<=1;
                    else if(kind==2 && count==8)begin
                        o_task<=header[1];o_epoch<=header[2];o_command<=header[3];o_flags<=header[4];
                        o_goal_x<=header[5];o_goal_y<=header[6];o_layer_speed<=header[7];o_control<=1;
                        if(header[5]>255 || header[6]>255 || header[7][31:24]!=0)begin
                            o_flags<={header[4][31:8],8'hff};o_error<=1;
                        end
                    end else if(kind==7 && count==4 && header[0]==i_current_session &&
                                header[1]==i_current_epoch && header[3]==1)begin
                        o_ui_close<=1;o_ui_revision<=header[2];
                    end else if(kind==8 && count==3 && header[0]==i_current_session && header[1]==i_current_epoch)begin
                        o_clock_request<=1;o_clock_nonce<=header[2];
                    end else if(kind==6 && count==4)begin
                        o_task<=header[1];o_epoch<=header[2];o_command<=header[3];o_heartbeat<=1;
                    end else if(kind==3 && count==6 && i_upload_allowed &&
                                header[0]==i_current_session && header[2]==i_current_epoch &&
                                header[1]>=i_current_task && header[3]>=i_current_command &&
                                header[4]>i_current_layer && header[4]<=65535)begin
                        o_task<=header[1];o_epoch<=header[2];o_command<=header[3];
                        o_layer_version<=header[4][15:0];o_expected_crc<=header[5];o_begin<=1;
                        upload_context<=1;upload_session<=header[0];upload_epoch<=header[2];upload_version<=header[4][15:0];
                        upload_task<=header[1];upload_command<=header[3];
                    end else if(kind==4 && count>4 && upload_context && i_upload_allowed &&
                                header[0]==upload_session && header[1]==upload_epoch && header[2]==upload_version &&
                                header[3]<4096 && header[3]+count-4<=4096)begin
                        data_offset<=header[3][12:0];index<=4;state<=DATA_READ;
                    end else if(kind==5 && count==5 && upload_context && i_upload_allowed &&
                                header[0]==upload_session && header[1]==upload_task && header[2]==upload_epoch &&
                                header[3]==upload_command && header[4]==upload_version) begin
                        o_task<=header[1];o_epoch<=header[2];o_command<=header[3];state<=COMMIT_WAIT;
                    end else o_error<=1;
                end
                DATA_READ:state<=DATA_TAKE;
                DATA_TAKE:begin o_layer_word<=i_word;o_word_index<=data_offset;state<=DATA_SEND;end
                DATA_SEND:if(!i_upload_allowed)begin state<=FINISH;o_error<=1;upload_context<=0;end
                else if(i_layer_ready)begin
                    o_word_valid<=1;data_offset<=data_offset+1'b1;
                    if(index+1==count)state<=FINISH;
                    else begin index<=index+1'b1;state<=DATA_READ;end
                end
                COMMIT_WAIT:if(!i_upload_allowed)begin state<=FINISH;o_error<=1;upload_context<=0;end
                else if(!i_layer_busy)begin o_commit<=1;upload_context<=0;state<=FINISH;end
                FINISH:state<=IDLE;
                default:state<=IDLE;
            endcase
            if(timer==TIMER_LIMIT && state!=IDLE && state!=FINISH) begin
                state<=FINISH;upload_context<=0;o_word_valid<=0;o_commit<=0;o_begin<=0;o_error<=1;
            end
        end
    end
endmodule
