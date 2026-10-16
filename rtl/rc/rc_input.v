// Application RC adapter. Channel indices refer to FPGA input slots, NOT receiver CH numbers.
// Board-specific HT-10A wiring/calibration lives in ht10a_input and falcons_top.
// Vendor rc_pwm_capture is reused unchanged. All commands/events require every input valid.
`timescale 1ns/1ps
module rc_input #(
    parameter integer CLK_FREQ_HZ=50_000_000, NUM_CH=6,
    parameter integer CENTER_US=1500, HALF_RANGE_US=500, DEADZONE_US=25,
    parameter integer STEER_CENTER_US=CENTER_US, THROTTLE_CENTER_US=CENTER_US,
    parameter integer STEER_HALF_RANGE_US=HALF_RANGE_US, THROTTLE_HALF_RANGE_US=HALF_RANGE_US,
    parameter integer STEER_DEADZONE_US=DEADZONE_US, THROTTLE_DEADZONE_US=DEADZONE_US,
    parameter integer MIN_PULSE_US=900, MAX_PULSE_US=2100, TIMEOUT_US=100_000,
    parameter integer SW_LO_US=1300, SW_HI_US=1700, SW_HYST_US=30, SW_CONFIRM_FRAMES=2,
    parameter integer STEER_REVERSE=0, THROTTLE_REVERSE=0,
    parameter integer MODE_REVERSE=0, SELECT_ACTIVE_LOW=0, CONFIRM_ACTIVE_LOW=0, BACK_ACTIVE_LOW=0,
    parameter integer MENU_STEP_TH=400, MENU_IDLE_TH=150, MENU_REVERSE=0,
    parameter integer CH_STEER=0, CH_THROTTLE=1, CH_CONFIRM=2, CH_BACK=3, CH_MODE=4, CH_SELECT=5
)(
    input wire clk, rst_n,
    input wire [NUM_CH-1:0] rc_pwm,
    output reg signed [11:0] o_steer, o_throttle,
    output reg [1:0] o_mode,
    output reg o_btn_select, o_btn_confirm, o_btn_back,
    output reg o_menu_up, o_menu_down,
    output wire o_signal_valid,
    output wire [NUM_CH*16-1:0] o_chan_us,
    output wire [NUM_CH-1:0] o_chan_timeout, o_chan_valid
);
    wire [15:0] us [0:NUM_CH-1];
    wire [NUM_CH-1:0] updated, valid, expired;
    wire all_good = rst_n && (&valid) && !(|expired);
    genvar g;
    generate for (g=0; g<NUM_CH; g=g+1) begin: g_cap
        rc_pwm_capture #(.CLK_HZ(CLK_FREQ_HZ), .MIN_PULSE_US(MIN_PULSE_US),
            .MAX_PULSE_US(MAX_PULSE_US), .TIMEOUT_US(TIMEOUT_US)) u_cap (
            .clk(clk), .rst_n(rst_n), .rc_pwm(rc_pwm[g]), .pulse_width_us(us[g]),
            .pulse_valid(valid[g]), .timed_out(expired[g]), .pulse_update(updated[g]));
        assign o_chan_us[g*16 +: 16] = us[g];
    end endgenerate
    assign o_chan_valid=valid;
    assign o_chan_timeout=expired;

    // Calibration is real: each stick has its own center, travel and deadzone.
    // Offset, constant-Q10 multiply and clipping occupy separate registers.
    localparam integer SS=(STEER_HALF_RANGE_US>STEER_DEADZONE_US) ? STEER_HALF_RANGE_US-STEER_DEADZONE_US : 1;
    localparam integer TS=(THROTTLE_HALF_RANGE_US>THROTTLE_DEADZONE_US) ? THROTTLE_HALF_RANGE_US-THROTTLE_DEADZONE_US : 1;
    localparam signed [31:0] SK=(1024000+SS-1)/SS, TK=(1024000+TS-1)/TS;
    function signed [31:0] offset_us;
        input [15:0] width;
        input integer center, deadzone, span;
        integer delta, mag;
        begin
            delta=$signed({1'b0,width})-center;
            mag=(delta<0) ? -delta : delta;
            if (mag<=deadzone) mag=0;
            else mag=mag-deadzone;
            if (mag>span) mag=span;
            offset_us=(delta<0) ? -mag : mag;
        end
    endfunction
    function signed [11:0] clip_cmd;
        input signed [31:0] product;
        reg signed [31:0] value;
        begin
            value=(product<0) ? -((-product) >>> 10) : (product >>> 10);
            if (value>1000) clip_cmd=12'sd1000;
            else if (value< -1000) clip_cmd=-12'sd1000;
            else clip_cmd=value[11:0];
        end
    endfunction
    reg signed [31:0] steer_delta, throttle_delta, steer_product, throttle_product;
    reg signed [11:0] throttle_unreversed;
    reg [3:0] command_ready;
    // Prevent a temporary zero from the arithmetic pipeline authorizing startup.
    assign o_signal_valid=all_good && command_ready[3];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            steer_delta<=0; throttle_delta<=0; steer_product<=0; throttle_product<=0;
            o_steer<=0; o_throttle<=0; throttle_unreversed<=0; command_ready<=0;
        end else if (!all_good) begin
            steer_delta<=0; throttle_delta<=0; steer_product<=0; throttle_product<=0;
            o_steer<=0; o_throttle<=0; throttle_unreversed<=0; command_ready<=0;
        end else begin
            command_ready<={command_ready[2:0],1'b1};
            steer_delta<=offset_us(us[CH_STEER],STEER_CENTER_US,STEER_DEADZONE_US,SS);
            throttle_delta<=offset_us(us[CH_THROTTLE],THROTTLE_CENTER_US,THROTTLE_DEADZONE_US,TS);
            steer_product<=steer_delta*SK;
            throttle_product<=throttle_delta*TK;
            o_steer<=command_ready[2] ? (STEER_REVERSE ? -clip_cmd(steer_product) : clip_cmd(steer_product)) : 12'sd0;
            o_throttle<=command_ready[2] ? (THROTTLE_REVERSE ? -clip_cmd(throttle_product) : clip_cmd(throttle_product)) : 12'sd0;
            throttle_unreversed<=command_ready[2] ? clip_cmd(throttle_product) : 12'sd0;
        end
    end

    // Schmitt thresholds + consecutive completed frames, not consecutive clocks.
    // Stable positions: 0=low, 1=middle/invalid, 2=high. MODE uses low vs others.
    function [1:0] classify;
        input [15:0] width;
        input [1:0] previous;
        begin
            if ((previous==0 && width<SW_LO_US+SW_HYST_US) || width<SW_LO_US-SW_HYST_US) classify=0;
            else if ((previous==2 && width>SW_HI_US-SW_HYST_US) || width>SW_HI_US+SW_HYST_US) classify=2;
            else classify=1;
        end
    endfunction
    reg [1:0] position [0:NUM_CH-1];
    reg [1:0] candidate [0:NUM_CH-1];
    reg [7:0] frames [0:NUM_CH-1];
    reg [NUM_CH-1:0] switch_ready;
    generate for (g=0; g<NUM_CH; g=g+1) begin: g_switch
        wire [1:0] next_position=classify(us[g],position[g]);
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                position[g]<=1; candidate[g]<=1; frames[g]<=0; switch_ready[g]<=0;
            end else if (!all_good) begin
                position[g]<=1; candidate[g]<=1; frames[g]<=0; switch_ready[g]<=0;
            end else if (updated[g]) begin
                if (next_position!=candidate[g]) begin
                    candidate[g]<=next_position;
                    frames[g]<=1;
                    if (SW_CONFIRM_FRAMES==1) begin position[g]<=next_position; switch_ready[g]<=1; end
                end else if (frames[g]<SW_CONFIRM_FRAMES) begin
                    frames[g]<=frames[g]+1'b1;
                    if (frames[g]>=SW_CONFIRM_FRAMES-1) begin position[g]<=next_position; switch_ready[g]<=1; end
                end
            end
        end
    end endgenerate
    wire select_down=SELECT_ACTIVE_LOW ? position[CH_SELECT]==0 : position[CH_SELECT]==2;
    wire confirm_down=CONFIRM_ACTIVE_LOW ? position[CH_CONFIRM]==0 : position[CH_CONFIRM]==2;
    wire back_down=BACK_ACTIVE_LOW ? position[CH_BACK]==0 : position[CH_BACK]==2;
    reg select_armed, confirm_armed, back_armed;
    reg menu_armed;
    wire signed [11:0] menu_value=MENU_REVERSE ? -throttle_unreversed : throttle_unreversed;
    wire mode_manual=MODE_REVERSE ? (position[CH_MODE]==2) : (position[CH_MODE]==0);
    wire cursor_ready=o_signal_valid && switch_ready[CH_MODE] && !mode_manual && o_mode==1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_mode<=1; o_btn_select<=0; o_btn_confirm<=0; o_btn_back<=0;
            o_menu_up<=0; o_menu_down<=0;
            select_armed<=0; confirm_armed<=0; back_armed<=0; menu_armed<=0;
        end else begin
            o_btn_select<=0; o_btn_confirm<=0; o_btn_back<=0;
            o_menu_up<=0; o_menu_down<=0;
            if (!o_signal_valid || !switch_ready[CH_MODE]) o_mode<=1;
            else if (MODE_REVERSE) o_mode<=(position[CH_MODE]==2) ? 0 : 1;
            else o_mode<=(position[CH_MODE]==0) ? 0 : 1;
            if (!cursor_ready) begin
                select_armed<=0; confirm_armed<=0; back_armed<=0; menu_armed<=0;
            end else begin
                // Require a verified release after reset/loss/mode entry; held buttons never create phantom clicks.
                if (switch_ready[CH_SELECT]) begin
                    if (!select_down) select_armed<=1;
                    else if (select_armed) begin o_btn_select<=1; select_armed<=0; end
                end
                if (switch_ready[CH_CONFIRM]) begin
                    if (!confirm_down) confirm_armed<=1;
                    else if (confirm_armed) begin o_btn_confirm<=1; confirm_armed<=0; end
                end
                if (switch_ready[CH_BACK]) begin
                    if (!back_down) back_armed<=1;
                    else if (back_armed) begin o_btn_back<=1; back_armed<=0; end
                end
                // A sampled center is required; jumping directly between endpoints is NOT a release.
                if (menu_value>=-MENU_IDLE_TH && menu_value<=MENU_IDLE_TH) menu_armed<=1;
                else if (menu_armed && menu_value>MENU_STEP_TH) begin o_menu_up<=1; menu_armed<=0; end
                else if (menu_armed && menu_value< -MENU_STEP_TH) begin o_menu_down<=1; menu_armed<=0; end
            end
        end
    end
endmodule
