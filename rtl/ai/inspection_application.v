// Application integration boundary: validated byte transactions, authority,
// real execution receipts and periodic pose telemetry share the existing UART.
// All ports are in clk; the optional high-speed adapter must cross clocks first.
module inspection_application #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer UI_ENABLE=0
)(
    input wire clk,rst_n,
    input wire [7:0] i_rx_data,
    input wire i_rx_valid,
    output wire o_rx_ready,
    input wire i_permission,i_manual,i_safety,i_link,i_local_pause,
    input wire [15:0] i_user_token,
    input wire i_ui_open,i_ui_master,i_ui_record,i_ui_rc_valid,i_ui_confirm,i_ui_back,
    input wire [1:0] i_ui_view,
    input wire signed [11:0] i_ui_steer,i_ui_throttle,
    input wire [15:0] i_vehicle_width_mm,
    input wire [7:0] i_forward_check_cells,
    output reg o_ui_close,
    input wire [31:0] i_epoch,
    input wire i_pose_tick,i_pose_valid,
    input wire signed [31:0] i_x_mm,i_y_mm,i_yaw_cdeg,
    input wire i_ctrl_done,i_command_idle,i_ctrl_fault,
    input wire [3:0] i_nav_state,
    output wire o_owned,o_goal_valid,o_motion_enable,o_force_stop,o_pending_motion,
    output wire [7:0] o_goal_x,o_goal_y,
    output wire o_layer_begin,o_layer_commit,o_layer_word_valid,
    output wire [31:0] o_layer_epoch,o_layer_crc,o_layer_word,
    output wire [15:0] o_layer_version,
    output wire [12:0] o_layer_word_index,
    input wire i_layer_ready,i_layer_valid,i_layer_committed,i_layer_error,i_layer_busy,
    input wire [15:0] i_layer_version,
    input wire [3:0] i_layer_error_code,
    output wire o_stopped,
    output wire [7:0] o_status,
    output wire [7:0] o_tx_data,
    output wire o_tx_valid,o_tx_last,
    input wire i_tx_ready
);
    wire message_valid,message_ready,word_read;
    wire [7:0] kind,sequence;
    wire [5:0] count;
    wire [4:0] word_index;
    wire [31:0] read_word;
    inspection_packet_rx #(.CLK_HZ(CLK_HZ)) receiver(
        .clk(clk),.rst_n(rst_n),.i_data(i_rx_data),.i_valid(i_rx_valid),.o_ready(o_rx_ready),
        .o_message_valid(message_valid),.i_message_ready(message_ready),.o_kind(kind),.o_sequence(sequence),.o_count(count),
        .i_word_read(word_read),.i_word_index(word_index),.o_word(read_word),.o_error());
    wire hello,control,heartbeat,upload_allowed,caps_request;
    wire [31:0] request_session,request_task,request_epoch,request_command;
    wire [31:0] flags,goal_x,goal_y,layer_speed;
    wire [31:0] session,task_id,command;
    wire [7:0] error_code;
    wire ui_close_request;
    wire [31:0] ui_revision;
    inspection_dispatch dispatch(
        .clk(clk),.rst_n(rst_n),.i_valid(message_valid),.i_kind(kind),.i_count(count),.o_ready(message_ready),
        .o_read(word_read),.o_read_index(word_index),.i_word(read_word),
        .i_current_session(session),.i_current_epoch(i_epoch),.i_current_task(task_id),.i_current_command(command),
        .i_current_layer(i_layer_version),.i_upload_allowed(upload_allowed),.i_layer_ready(i_layer_ready),.i_layer_busy(i_layer_busy),
        .o_hello(hello),.o_control(control),.o_heartbeat(heartbeat),.o_begin(o_layer_begin),.o_commit(o_layer_commit),
        .o_word_valid(o_layer_word_valid),.o_session(request_session),.o_task(request_task),.o_epoch(request_epoch),.o_command(request_command),
        .o_flags(flags),.o_goal_x(goal_x),.o_goal_y(goal_y),.o_layer_speed(layer_speed),
        .o_expected_crc(o_layer_crc),.o_layer_word(o_layer_word),.o_word_index(o_layer_word_index),.o_layer_version(o_layer_version),
        .o_ui_close(ui_close_request),.o_ui_revision(ui_revision),.o_error());
    assign o_layer_epoch=request_epoch;
    inspection_nav_control #(.CLK_HZ(CLK_HZ)) authority(
        .clk(clk),.rst_n(rst_n),.i_permission(i_permission),.i_manual(i_manual),.i_safety(i_safety),.i_link(i_link),
        .i_local_pause(i_local_pause),.i_epoch(i_epoch),.i_user_token(i_user_token),
        .i_hello(hello),.i_control(control),.i_heartbeat(heartbeat),.i_layer_begin(o_layer_begin),
        .i_session(request_session),.i_task(request_task),.i_command(request_command),.i_request_epoch(request_epoch),
        .i_op(flags[7:0]),.i_speed(layer_speed[23:16]),.i_commissioned(flags[8]),.i_internal_pause(flags[9]),
        .i_token(flags[31:16]),.i_requested_layer(layer_speed[15:0]),.i_goal_x(goal_x[7:0]),.i_goal_y(goal_y[7:0]),
        .i_ctrl_done(i_ctrl_done),.i_command_idle(i_command_idle),.i_ctrl_fault(i_ctrl_fault),.i_nav_state(i_nav_state),
        .i_layer_valid(i_layer_valid),.i_layer_committed(i_layer_committed),.i_layer_error(i_layer_error),
        .i_layer_version(i_layer_version),.i_layer_error_code(i_layer_error_code),
        .o_owned(o_owned),.o_goal_valid(o_goal_valid),.o_motion_enable(o_motion_enable),.o_goal_x(o_goal_x),.o_goal_y(o_goal_y),
        .o_force_stop(o_force_stop),.o_stopped(o_stopped),.o_upload_allowed(upload_allowed),
        .o_pending_motion(o_pending_motion),
        .o_session(session),.o_task(task_id),.o_command(command),.o_status(o_status),.o_error_code(error_code),.o_caps_request(caps_request));
    localparam integer US_DIV=CLK_HZ/1_000_000<1?1:CLK_HZ/1_000_000;
    localparam integer US_BITS=US_DIV<2?1:$clog2(US_DIV);
    reg [US_BITS-1:0] us_divider;
    reg [31:0] time_us,pose_sequence,pose_time_us;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin us_divider<=0;time_us<=0;pose_sequence<=0;pose_time_us<=0;end
        else begin
            if(us_divider>=US_DIV-1)begin us_divider<=0;time_us<=time_us+1'b1;end
            else us_divider<=us_divider+1'b1;
            if(i_pose_tick)begin pose_sequence<=pose_sequence+1'b1;pose_time_us<=time_us;end
        end
    end
    wire tx_busy,tx_start;
    wire [7:0] tx_kind;
    wire [3:0] tx_count;
    wire [255:0] tx_words;
    wire [31:0] status_flags={25'd0,i_pose_valid,o_owned,i_layer_valid,i_safety,i_permission,i_manual,o_stopped};
    reg [31:0] input_sequence,last_ui_revision,ui_session,ui_epoch,previous_input_flags;
    reg [15:0] confirms,backs;
    wire [31:0] input_flags={23'd0,i_permission,i_ui_rc_valid,i_ui_record,i_ui_view,1'b0,i_ui_master,i_manual,i_ui_open};
    wire [31:0] input_axes={{4{i_ui_throttle[11]}},i_ui_throttle,{4{i_ui_steer[11]}},i_ui_steer};
    wire [255:0] input_words={{8'd0,i_forward_check_cells,i_vehicle_width_mm},{16'd0,i_user_token},
        backs,confirms,input_axes,input_flags,input_sequence,i_epoch,session};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            input_sequence<=0;last_ui_revision<=0;ui_session<=0;ui_epoch<=0;previous_input_flags<=0;
            confirms<=0;backs<=0;o_ui_close<=0;
        end else begin
            o_ui_close<=0;ui_session<=session;ui_epoch<=i_epoch;previous_input_flags<=input_flags;
            if(UI_ENABLE!=0)begin
                if(input_flags!=previous_input_flags || (i_ui_open && !i_manual && i_ui_rc_valid && (i_ui_confirm || i_ui_back)))
                    input_sequence<=input_sequence+1'b1;
                if(i_ui_open && !i_manual && i_ui_rc_valid)begin
                    if(i_ui_confirm)confirms<=confirms+1'b1;
                    if(i_ui_back)backs<=backs+1'b1;
                end
                if(session!=ui_session || i_epoch!=ui_epoch)last_ui_revision<=0;
                else if(ui_close_request && ui_revision>last_ui_revision)begin
                    last_ui_revision<=ui_revision;
                    if(o_stopped || !o_owned)o_ui_close<=1;
                end
            end
        end
    end
    inspection_telemetry #(.REPORT_CYCLES(CLK_HZ/10),.INPUT_ENABLE(UI_ENABLE)) telemetry(
        .clk(clk),.rst_n(rst_n),.i_enabled(i_link && session!=0),.i_caps_request(caps_request),
        .i_session(session),.i_task(task_id),.i_epoch(i_epoch),.i_command(command),.i_status(o_status),.i_error(error_code),
        .i_layer_version(i_layer_version),.i_flags(status_flags),.i_pose_sequence(pose_sequence),.i_pose_time_us(pose_time_us),
        .i_x_mm(i_x_mm),.i_y_mm(i_y_mm),.i_yaw_cdeg(i_yaw_cdeg),.i_pose_valid(i_pose_valid),
        .i_input_words(input_words),.i_input_sequence(input_sequence),.i_input_flags(input_flags),
        .i_tx_busy(tx_busy),.o_start(tx_start),.o_kind(tx_kind),.o_count(tx_count),.o_words(tx_words));
    inspection_packet_tx transmitter(.clk(clk),.rst_n(rst_n),.i_start(tx_start),.i_kind(tx_kind),.i_count(tx_count),.i_words(tx_words),
        .o_busy(tx_busy),.o_data(o_tx_data),.o_valid(o_tx_valid),.o_last(o_tx_last),.i_ready(i_tx_ready));
endmodule
