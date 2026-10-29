// Passive copy of the existing display sweep; it never backpressures radar or
// navigation. Each complete raster is CRC-protected. Its sequence carries
// start/end map versions, because updates may continue during a sweep. This
// is a UI snapshot, never a collision authority or proof of an atomic map.
module inspection_map_export(
    input wire clk,rst_n,i_enabled,
    input wire i_valid,i_occupied,i_start,i_done,
    input wire [15:0] i_version,
    input wire [31:0] i_session,i_task,i_epoch,i_context,
    input wire [1:0] i_view,
    output reg o_valid,
    input wire i_release,
    output reg [31:0] o_session,o_task,o_epoch,o_context,o_sequence,o_crc,
    output reg [1:0] o_view,
    input wire i_read,
    input wire [11:0] i_address,
    output reg [15:0] o_data,
    output reg o_error
);
    reg [15:0] memory[0:8191];
    reg front_bank,capture_bank,capturing,pending;
    reg [15:0] index,packed_word,start_version,end_version;
    reg [31:0] crc,complete_crc,session,task_id,epoch,context;
    reg [1:0] view;
    wire context_valid=i_enabled && i_session==session && i_task==task_id && i_epoch==epoch && i_context==context && i_view==view;
    wire [31:0] next_crc=(crc>>1)^((crc[0]^i_occupied)?32'hedb88320:32'd0);
    wire [15:0] packed_next={i_occupied,packed_word[15:1]};
    // First received cell is bit zero after sixteen right-shift samples.
    always @(posedge clk)begin
        if(i_read)o_data<=memory[{front_bank,i_address}];
        if(capturing && context_valid && i_valid && !i_start && index[3:0]==15)
            memory[{capture_bank,index[15:4]}]<=packed_next;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            front_bank<=0;capture_bank<=1;capturing<=0;pending<=0;index<=0;packed_word<=0;
            start_version<=0;end_version<=0;crc<=32'hffffffff;complete_crc<=0;
            session<=0;task_id<=0;epoch<=0;context<=0;view<=0;o_valid<=0;o_error<=0;
            o_session<=0;o_task<=0;o_epoch<=0;o_context<=0;o_sequence<=0;o_crc<=0;o_view<=0;
        end else begin
            o_error<=0;
            if(i_release)o_valid<=0;
            if(pending && (!o_valid || i_release))begin
                pending<=0;
                if(context_valid)begin
                    o_valid<=1;front_bank<=capture_bank;o_session<=session;o_task<=task_id;
                    o_epoch<=epoch;o_context<=context;o_view<=view;o_sequence<={start_version,end_version};o_crc<=complete_crc;
                end
            end
            if(capturing && !context_valid)capturing<=0;
            if(i_enabled && i_session!=0 && i_valid && i_start && !pending)begin
                capture_bank<=!front_bank;capturing<=1;index<=1;
                packed_word<={i_occupied,15'd0};
                crc<=(32'hffffffff>>1)^((1'b1^i_occupied)?32'hedb88320:32'd0);
                start_version<=i_version;session<=i_session;task_id<=i_task;epoch<=i_epoch;context<=i_context;view<=i_view;
            end else if(capturing && context_valid && i_valid)begin
                packed_word<=packed_next;crc<=next_crc;index<=index+1'b1;
                if(i_done || index==16'hffff)begin
                    capturing<=0;
                    if(i_done && index==16'hffff)begin pending<=1;complete_crc<=~next_crc;end_version<=i_version;end
                    else o_error<=1;
                end
            end
        end
    end
endmodule
