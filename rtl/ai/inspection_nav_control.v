// Authority and receipt state for the existing navigation core. No geometry,
// projection or path search here. STOP -> actual done low/high -> RELEASE ->
// idle handshake -> RUN. Manual/safety/watchdog revoke without waiting for RK.
module inspection_nav_control #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer LEASE_CYCLES=CLK_HZ/2
)(
    input wire clk,rst_n,
    input wire i_permission,i_manual,i_safety,i_link,i_local_pause,
    input wire [31:0] i_epoch,
    input wire [15:0] i_user_token,
    input wire i_hello,i_control,i_heartbeat,i_layer_begin,
    input wire [31:0] i_session,i_task,i_command,i_request_epoch,
    input wire [7:0] i_op,i_speed,
    input wire i_commissioned,i_internal_pause,
    input wire [15:0] i_token,i_requested_layer,
    input wire [7:0] i_goal_x,i_goal_y,
    input wire i_ctrl_done,i_command_idle,i_ctrl_fault,
    input wire [3:0] i_nav_state,
    input wire i_layer_valid,i_layer_committed,i_layer_error,
    input wire [15:0] i_layer_version,
    input wire [3:0] i_layer_error_code,
    output reg o_owned,o_goal_valid,o_motion_enable,
    output reg [7:0] o_goal_x,o_goal_y,
    output wire o_force_stop,o_stopped,o_upload_allowed,o_pending_motion,
    output reg [31:0] o_session,o_task,o_command,
    output reg [7:0] o_status,o_error_code,
    output reg o_caps_request
);
    localparam IDLE=0,STOPPING=1,HOLD=2,RELEASE=3,RUN=4;
    reg [2:0] state;
    reg seen_low,pending_go,release_owner,armed,suspended;
    reg [15:0] consumed_token;
    localparam integer AGE_BITS=LEASE_CYCLES<2?1:$clog2(LEASE_CYCLES+1);
    localparam [AGE_BITS-1:0] AGE_LIMIT=LEASE_CYCLES;
    reg [AGE_BITS-1:0] age;
    reg [31:0] epoch_q;
    reg [7:0] stop_status;
    wire unsafe=i_manual || !i_safety || !i_link || age>=AGE_LIMIT;
    wire identity_ok=i_session==o_session && o_session!=0 && i_request_epoch==i_epoch;
    wire fresh_token=i_token!=0 && i_token==i_user_token && i_token!=consumed_token;
    assign o_force_stop=o_owned && state!=RUN && state!=RELEASE;
    assign o_pending_motion=pending_go || state==RELEASE;
    assign o_stopped=o_force_stop && state==HOLD && i_ctrl_done && !i_manual;
    assign o_upload_allowed=o_stopped && i_permission && i_safety && i_link;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;seen_low<=0;pending_go<=0;release_owner<=0;armed<=0;suspended<=0;
            consumed_token<=0;age<=AGE_LIMIT;epoch_q<=0;stop_status<=2;
            o_owned<=0;o_goal_valid<=0;o_motion_enable<=0;o_goal_x<=0;o_goal_y<=0;
            o_session<=0;o_task<=0;o_command<=0;o_status<=0;o_error_code<=0;o_caps_request<=0;
        end else begin
            o_caps_request<=0;epoch_q<=i_epoch;
            if(o_owned && age<AGE_LIMIT)age<=age+1'b1;
            if(!unsafe)suspended<=0;
            case(state)
                STOPPING: begin
                    if(!i_ctrl_done)seen_low<=1;
                    if(seen_low && i_ctrl_done && !i_manual) begin
                        if(pending_go && !unsafe && i_permission) begin state<=RELEASE;end
                        else begin state<=HOLD;o_status<=stop_status;end
                    end
                end
                HOLD: if(release_owner && i_ctrl_done && !i_manual) begin
                    state<=RELEASE;pending_go<=0;
                end
                RELEASE: if(i_command_idle && !i_ctrl_done) begin
                    if(pending_go && !unsafe && i_permission) begin
                        o_goal_valid<=1;o_motion_enable<=1;o_status<=1;state<=RUN;pending_go<=0;
                    end else begin
                        state<=IDLE;o_owned<=0;release_owner<=0;
                    end
                end
                RUN: begin
                    if(i_ctrl_fault) begin
                        o_goal_valid<=0;o_motion_enable<=0;state<=STOPPING;seen_low<=0;stop_status<=5;
                        armed<=0;o_error_code<=8'd16;
                    end else if(i_nav_state==3) begin
                        o_goal_valid<=0;o_motion_enable<=0;state<=STOPPING;seen_low<=0;stop_status<=3;
                    end else if(i_nav_state==4)o_status<=4;
                    else o_status<=1;
                end
                default: ;
            endcase
            if(i_hello && i_session!=0) begin
                o_caps_request<=1;age<=0;
                if(i_session!=o_session) begin
                    o_session<=i_session;o_task<=0;o_command<=0;armed<=0;consumed_token<=i_user_token;
                    o_goal_valid<=0;o_motion_enable<=0;pending_go<=0;
                    if(o_owned)begin state<=STOPPING;seen_low<=0;stop_status<=2;end
                end
            end
            if(i_heartbeat && identity_ok && i_task==o_task && i_command==o_command)age<=0;
            // Entering the inspection page claims zero output first. It never
            // authorizes movement; first GOTO still needs a physical confirm.
            if(i_permission && o_session!=0 && !o_owned) begin
                o_owned<=1;state<=STOPPING;seen_low<=0;stop_status<=2;
                o_goal_valid<=0;o_motion_enable<=0;release_owner<=0;pending_go<=0;
            end
            if(i_layer_begin && identity_ok && o_upload_allowed && i_task>=o_task && i_command>=o_command) begin
                o_task<=i_task;o_command<=i_command;o_status<=7;age<=0;
            end
            if(i_layer_committed && o_owned)begin o_status<=6;end
            if(i_layer_error && o_owned)begin o_status<=5;o_error_code<={4'h4,i_layer_error_code};armed<=0;end
            if(i_control && identity_ok && i_task>=o_task && i_command>o_command) begin
                age<=0;
                if(i_op==1 && i_permission && !unsafe && i_commissioned && i_speed==30 &&
                   i_layer_valid && i_requested_layer==i_layer_version && ((armed && i_task==o_task) || fresh_token)) begin
                    o_task<=i_task;o_command<=i_command;o_goal_x<=i_goal_x;o_goal_y<=i_goal_y;
                    o_goal_valid<=0;o_motion_enable<=0;pending_go<=1;release_owner<=0;
                    armed<=1;consumed_token<=i_token;o_error_code<=0;
                    if(o_stopped)state<=RELEASE;
                    else begin state<=STOPPING;seen_low<=0;stop_status<=2;end
                end else if(i_op==2 || i_op==3) begin
                    o_task<=i_task;o_command<=i_command;o_goal_valid<=0;o_motion_enable<=0;pending_go<=0;
                    release_owner<=i_op==3;stop_status<=2;
                    if(!i_internal_pause || i_op==3) begin armed<=0;consumed_token<=i_user_token;end
                    if(o_stopped)o_status<=2;
                    else begin state<=STOPPING;seen_low<=0;end
                end else begin
                    // Invalid GOTO has a receipt but never changes an active
                    // goal into a new one; revoke and hold for explicit retry.
                    o_task<=i_task;o_command<=i_command;o_goal_valid<=0;o_motion_enable<=0;
                    pending_go<=0;armed<=0;consumed_token<=i_user_token;o_error_code<=8'd17;stop_status<=5;
                    if(o_stopped)o_status<=5;else begin state<=STOPPING;seen_low<=0;end
                end
            end
            if(o_owned && (!i_permission || unsafe) && !suspended) begin
                suspended<=1;o_goal_valid<=0;o_motion_enable<=0;pending_go<=0;
                armed<=0;consumed_token<=i_user_token;release_owner<=!i_permission;
                stop_status<=2;o_error_code<=i_manual?8'd1:!i_permission?8'd2:8'd3;
                if(state==RUN || state==RELEASE)begin state<=STOPPING;seen_low<=0;end
            end
            if(o_owned && i_local_pause && (state==RUN || state==RELEASE || pending_go))begin
                o_goal_valid<=0;o_motion_enable<=0;pending_go<=0;armed<=0;consumed_token<=i_user_token;
                state<=STOPPING;seen_low<=0;stop_status<=2;
            end
            if(i_epoch!=epoch_q) begin
                o_goal_valid<=0;o_motion_enable<=0;armed<=0;pending_go<=0;
                consumed_token<=i_user_token;
                if(o_owned)begin state<=STOPPING;seen_low<=0;stop_status<=5;o_error_code<=8'd4;end
            end
        end
    end
endmodule
