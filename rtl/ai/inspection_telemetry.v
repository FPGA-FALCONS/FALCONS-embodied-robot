// Snapshot producer: status and pose are periodic, execution changes are
// retained while the serializer is busy. All wide bus selection is registered
// before packet generation. No generated clock or pixel-domain dependency.
module inspection_telemetry #(
    parameter integer REPORT_CYCLES=5_000_000,
    parameter integer INPUT_ENABLE=0
)(
    input wire clk,rst_n,i_enabled,i_caps_request,
    input wire [31:0] i_session,i_task,i_epoch,i_command,
    input wire [7:0] i_status,i_error,
    input wire [15:0] i_layer_version,
    input wire [31:0] i_flags,i_pose_sequence,i_pose_time_us,
    input wire signed [31:0] i_x_mm,i_y_mm,i_yaw_cdeg,
    input wire i_pose_valid,
    input wire [255:0] i_input_words,
    input wire [31:0] i_input_sequence,i_input_flags,
    input wire i_tx_busy,
    output reg o_start,
    output reg [7:0] o_kind,
    output reg [3:0] o_count,
    output reg [255:0] o_words
);
    localparam integer TIMER_BITS=REPORT_CYCLES<2?1:$clog2(REPORT_CYCLES+1);
    reg [TIMER_BITS-1:0] timer;
    reg [31:0] last_command,last_flags;
    reg [7:0] last_status,last_error;
    reg [15:0] last_layer;
    reg status_due,pose_due,caps_due;
    reg input_due;
    reg [31:0] last_input_sequence,last_input_flags;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            timer<=0;last_command<=0;last_flags<=0;last_status<=0;last_error<=0;last_layer<=0;
            status_due<=0;pose_due<=0;caps_due<=0;o_start<=0;o_kind<=0;o_count<=0;o_words<=0;
            input_due<=0;last_input_sequence<=0;last_input_flags<=0;
        end else begin
            o_start<=0;
            last_input_sequence<=i_input_sequence;last_input_flags<=i_input_flags;
            last_command<=i_command;last_status<=i_status;last_flags<=i_flags;last_error<=i_error;last_layer<=i_layer_version;
            if(timer>=REPORT_CYCLES-1)timer<=0;else timer<=timer+1'b1;
            if(i_enabled && !i_tx_busy && !o_start)begin
                if(caps_due)begin
                    o_start<=1;o_kind<=8'h80;o_count<=6;caps_due<=0;
                    o_words<={64'd0,i_flags,(INPUT_ENABLE!=0?32'h00000007:32'h00000003),{16'd0,i_layer_version},i_epoch,32'd1,i_session};
                end else if(status_due)begin
                    o_start<=1;o_kind<=8'h81;o_count<=8;status_due<=0;
                    o_words<={{24'd0,i_error},i_flags,{16'd0,i_layer_version},{24'd0,i_status},i_command,i_epoch,i_task,i_session};
                end else if(input_due && INPUT_ENABLE!=0)begin
                    o_start<=1;o_kind<=8'h83;o_count<=8;input_due<=0;o_words<=i_input_words;
                end else if(pose_due)begin
                    o_start<=1;o_kind<=8'h82;o_count<=7;pose_due<=0;
                    o_words<={32'd0,{31'd0,i_pose_valid},i_yaw_cdeg,i_y_mm,i_x_mm,i_pose_time_us,i_pose_sequence,i_epoch};
                end
            end
            if(i_caps_request)caps_due<=1;
            if(timer>=REPORT_CYCLES-1)begin status_due<=1;pose_due<=1;end
            if(INPUT_ENABLE!=0 && (timer>=REPORT_CYCLES-1 || i_input_sequence!=last_input_sequence ||
               i_input_flags!=last_input_flags))input_due<=1;
            if(i_command!=last_command || i_status!=last_status || i_flags!=last_flags ||
               i_error!=last_error || i_layer_version!=last_layer)status_due<=1;
        end
    end
endmodule
