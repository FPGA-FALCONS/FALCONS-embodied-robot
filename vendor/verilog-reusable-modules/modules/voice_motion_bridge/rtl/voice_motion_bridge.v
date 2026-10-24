`timescale 1ns/1ps
// MOT1 motion bridge: RK3588 -> FPGA bounded motion primitives.
//
// This module deliberately does NOT replace the high-level FAM1 mission
// protocol.  It only accepts short, bounded motion commands and drives the
// existing chassis_ctrl auto inputs.  RC/manual, estop, ToF and run_en remain
// higher-priority safety controls outside this module.
//
// Frame: MOT1 + seq + cmd + value(int16 BE) + speed + flags + CRC16
//   cmd 1/2: forward/backward, value = duration_ms
//   cmd 3/4: turn left/right, value = angle_deg
//   cmd 5:   spin, value = signed angle_deg (positive = clockwise)
//   cmd 6/7: patrol start/stop pulses (navigation layer handles them)
//   cmd 8:   heartbeat
//
// A command must be refreshed/acknowledged with heartbeats; no valid frame for
// WATCHDOG_MS stops the motion.  Turn/spin use i_yaw_deg (integer degrees,
// wrapped to [-180,180]) for closed-loop stopping.
module voice_motion_bridge #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer MAX_CENTER   = 1024,
    parameter integer TURN_DIFF    = 600,
    parameter integer WATCHDOG_MS  = 500,
    parameter integer TIMEOUT_MS   = 8000,
    parameter integer YAW_SIGN     = 1,
    parameter integer BYTE_TIMEOUT_MS = 20
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire [7:0]         i_rx_data,
    input  wire               i_rx_valid,

    input  wire               i_enable,
    input  wire               i_estop,
    input  wire               i_manual,
    input  wire signed [31:0] i_yaw_deg,
    input  wire               i_yaw_valid,
    input  wire               i_imu_sample_valid,
    input  wire               i_imu_healthy,
    input  wire               i_nav_release_ready,

    output wire               o_auto_valid,
    output wire signed [31:0] o_auto_center,
    output wire signed [31:0] o_auto_diff,
    output wire               o_active,
    output reg                o_owned,
    output reg  [7:0]         o_last_seq,
    output reg  [7:0]         o_last_cmd,
    output reg  [7:0]         o_last_flags,
    output wire               o_cmd_error,
    output reg                o_patrol_start,
    output reg                o_patrol_stop
);
    localparam [7:0] CMD_STOP = 8'd0, CMD_FORWARD = 8'd1, CMD_BACKWARD = 8'd2,
        CMD_LEFT = 8'd3, CMD_RIGHT = 8'd4, CMD_SPIN = 8'd5,
        CMD_PATROL_START = 8'd6, CMD_PATROL_STOP = 8'd7, CMD_HEARTBEAT = 8'd8;
    localparam [1:0] MODE_IDLE = 2'd0, MODE_DRIVE = 2'd1, MODE_TURN = 2'd2;

    // --------------------------------------------------------------
    // MOT1 parser
    // --------------------------------------------------------------
    reg [2:0] magic_idx;
    reg [2:0] data_idx;
    reg [7:0] fd0, fd1, fd2, fd3, fd4, fd5, fd6;
    reg [15:0] crc_acc;
    reg [7:0] cmd_code;
    reg signed [15:0] cmd_value;
    reg [7:0] cmd_speed;
    reg cmd_valid;
    reg [7:0] expect_byte;
    reg rx_error;
    localparam integer BYTE_TIMEOUT_CLKS = (CLK_HZ / 1000) * BYTE_TIMEOUT_MS;
    reg [31:0] byte_gap_clks;

    function [15:0] crc16_byte(input [15:0] crc, input [7:0] data);
        integer i;
        reg [15:0] c;
        begin
            c = crc ^ {data, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte = c;
        end
    endfunction

    always @* begin
        case (magic_idx)
            3'd0: expect_byte = 8'h4D; // M
            3'd1: expect_byte = 8'h4F; // O
            3'd2: expect_byte = 8'h54; // T
            default: expect_byte = 8'h31; // 1
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            magic_idx <= 3'd0;
            data_idx  <= 3'd0;
            fd0 <= 8'd0; fd1 <= 8'd0; fd2 <= 8'd0; fd3 <= 8'd0;
            fd4 <= 8'd0; fd5 <= 8'd0; fd6 <= 8'd0;
            crc_acc <= 16'hFFFF;
            cmd_valid <= 1'b0;
            cmd_code <= 8'd0; cmd_value <= 16'sd0;
            cmd_speed <= 8'd0;
            o_last_seq <= 8'd0; o_last_cmd <= 8'd0; o_last_flags <= 8'd0;
            rx_error <= 1'b0;
            byte_gap_clks <= 0;
        end else begin
            cmd_valid <= 1'b0;
            rx_error <= 1'b0;
            if (i_rx_valid) begin
                byte_gap_clks <= 0;
                if (magic_idx < 3'd4) begin
                    if (i_rx_data == expect_byte) begin
                        crc_acc <= (magic_idx == 3'd0) ?
                            crc16_byte(16'hFFFF, i_rx_data) :
                            crc16_byte(crc_acc, i_rx_data);
                        magic_idx <= magic_idx + 3'd1;
                    end else if (i_rx_data == 8'h4D) begin
                        magic_idx <= 3'd1;
                        crc_acc <= crc16_byte(16'hFFFF, 8'h4D);
                    end else begin
                        magic_idx <= 3'd0;
                    end
                end else begin
                    case (data_idx)
                        3'd0: fd0 <= i_rx_data;
                        3'd1: fd1 <= i_rx_data;
                        3'd2: fd2 <= i_rx_data;
                        3'd3: fd3 <= i_rx_data;
                        3'd4: fd4 <= i_rx_data;
                        3'd5: fd5 <= i_rx_data;
                        3'd6: fd6 <= i_rx_data;
                        default: ;
                    endcase
                    if (data_idx < 3'd6)
                        crc_acc <= crc16_byte(crc_acc, i_rx_data);
                    if (data_idx == 3'd7) begin
                        if (crc_acc == {fd6, i_rx_data}) begin
                            cmd_valid <= 1'b1;
                            cmd_code  <= fd1;
                            cmd_value <= $signed({fd2, fd3});
                            cmd_speed <= fd4;
                            o_last_seq <= fd0;
                            o_last_cmd <= fd1;
                            o_last_flags <= fd5;
                        end else begin
                            rx_error <= 1'b1;
                        end
                        magic_idx <= 3'd0;
                        data_idx  <= 3'd0;
                    end else begin
                        data_idx <= data_idx + 3'd1;
                    end
                end
            end else if (magic_idx != 0) begin
                if (byte_gap_clks >= BYTE_TIMEOUT_CLKS - 1) begin
                    magic_idx <= 0; data_idx <= 0; crc_acc <= 16'hFFFF;
                    byte_gap_clks <= 0; rx_error <= 1;
                end else byte_gap_clks <= byte_gap_clks + 1;
            end else byte_gap_clks <= 0;
        end
    end

    // Millisecond freshness is based on actual sensor sample pulses, not on a
    // permanently latched "yaw valid" flag. No command can forge freshness.
    localparam integer MS_DIV = (CLK_HZ / 1000 > 0) ? (CLK_HZ / 1000) : 1;
    localparam [31:0] IMU_FRESH_MS = 32'd200;
    reg [31:0] ms_div, wd_ms, remaining_ms, timeout_ms, imu_age_ms;
    reg imu_seen;
    reg signed [31:0] target_angle, accum, yaw_prev;
    reg signed [31:0] drive_center, turn_diff;
    reg desired_sign; // 1 = left/CCW, 0 = right/CW
    reg [1:0] mode;
    reg motion_error;
    wire ms_tick = (ms_div == MS_DIV - 1);
    wire safety_ok = i_enable && !i_estop && !i_manual;
    wire yaw_in_range = (i_yaw_deg >= -32'sd180) && (i_yaw_deg <= 32'sd180);
    wire sample_ok = i_imu_sample_valid && i_imu_healthy && i_yaw_valid && yaw_in_range;
    wire imu_fresh = i_imu_healthy && i_yaw_valid && yaw_in_range &&
                     (sample_ok || (imu_seen && imu_age_ms < IMU_FRESH_MS &&
                                   !(ms_tick && imu_age_ms == IMU_FRESH_MS-1)));
    wire signed [31:0] yaw_eff = (YAW_SIGN != 0) ? i_yaw_deg : -i_yaw_deg;
    wire signed [31:0] cmd_value32 = {{16{cmd_value[15]}}, cmd_value};
    wire signed [31:0] abs_angle = cmd_value32 < 0 ? -cmd_value32 : cmd_value32;
    wire signed [31:0] raw_speed = $signed({22'd0, cmd_speed, 2'b00});
    wire signed [31:0] speed_scaled = raw_speed > MAX_CENTER ? MAX_CENTER : raw_speed;
    wire signed [31:0] yaw_delta_raw = yaw_eff - yaw_prev;
    wire signed [31:0] yaw_delta = yaw_delta_raw > 32'sd180 ? yaw_delta_raw-32'sd360 :
                                 yaw_delta_raw < -32'sd180 ? yaw_delta_raw+32'sd360 : yaw_delta_raw;
    wire signed [31:0] next_accum = accum + yaw_delta;
    wire turn_cmd = cmd_code == CMD_LEFT || cmd_code == CMD_RIGHT || cmd_code == CMD_SPIN;
    wire drive_cmd = cmd_code == CMD_FORWARD || cmd_code == CMD_BACKWARD;
    wire semantic_ok = (cmd_code == CMD_STOP || cmd_code == CMD_PATROL_STOP ||
                        cmd_code == CMD_HEARTBEAT) ||
        (cmd_code == CMD_PATROL_START && i_nav_release_ready) ||
        (drive_cmd && cmd_value > 0 && cmd_value <= 16'sd5000 && cmd_speed != 0) ||
        (turn_cmd && imu_fresh && abs_angle >= 32'sd1 && abs_angle <= 32'sd360 &&
         (cmd_code == CMD_SPIN || cmd_value > 0) && cmd_speed != 0);
    wire accepted = cmd_valid && semantic_ok;
    wire new_action = accepted && cmd_code != CMD_HEARTBEAT;

    assign o_cmd_error = rx_error | motion_error;
    // Ownership persists after completion/fault/STOP: idle voice zero must not
    // silently reveal a stale valid navigation command behind the mux.
    assign o_auto_valid = safety_ok && o_owned;
    assign o_auto_center = safety_ok && mode == MODE_DRIVE ? drive_center : 32'sd0;
    assign o_auto_diff = safety_ok && mode == MODE_TURN ? turn_diff : 32'sd0;
    assign o_active = mode != MODE_IDLE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_div <= 0; wd_ms <= 0; remaining_ms <= 0; timeout_ms <= 0;
            imu_age_ms <= IMU_FRESH_MS; imu_seen <= 0;
            target_angle <= 0; accum <= 0; yaw_prev <= 0;
            drive_center <= 0; turn_diff <= 0; desired_sign <= 1;
            mode <= MODE_IDLE; o_owned <= 0; motion_error <= 0;
            o_patrol_start <= 0; o_patrol_stop <= 0;
        end else begin
            o_patrol_start <= 0; o_patrol_stop <= 0; motion_error <= 0;
            if (ms_tick) ms_div <= 0;
            else ms_div <= ms_div + 1;
            if (!i_imu_healthy || !i_yaw_valid || !yaw_in_range) begin
                imu_seen <= 0; imu_age_ms <= IMU_FRESH_MS;
            end else if (sample_ok) begin
                imu_seen <= 1; imu_age_ms <= 0;
            end else if (ms_tick && imu_age_ms < IMU_FRESH_MS)
                imu_age_ms <= imu_age_ms + 1;

            // Only commands admitted by semantics and safety refresh watchdog.
            if (accepted && safety_ok) wd_ms <= 0;
            else if (ms_tick && wd_ms < WATCHDOG_MS) wd_ms <= wd_ms + 1;

            if (!safety_ok) begin
                // Safety cancels the old action. Ownership stays held, so
                // recovering a safety input cannot restart it or stale nav.
                mode <= MODE_IDLE; drive_center <= 0; turn_diff <= 0;
                // STOP remains a persistent request even while manual/RC/
                // physical safety owns the chassis. It must prevent a stale
                // navigation command from reappearing after recovery.
                if (cmd_valid && (cmd_code == CMD_STOP || cmd_code == CMD_PATROL_STOP))
                    o_owned <= 1;
            end else if (new_action) begin
                // Explicit command replacement wins over old duration, yaw and
                // timeout updates even when the CRC commit coincides with tick.
                mode <= MODE_IDLE; drive_center <= 0; turn_diff <= 0;
                o_owned <= 1;
                case (cmd_code)
                    CMD_STOP: ;
                    CMD_FORWARD, CMD_BACKWARD: begin
                        mode <= MODE_DRIVE; remaining_ms <= cmd_value32;
                        drive_center <= cmd_code == CMD_FORWARD ? speed_scaled : -speed_scaled;
                    end
                    CMD_LEFT, CMD_RIGHT, CMD_SPIN: begin
                        mode <= MODE_TURN; accum <= 0; yaw_prev <= yaw_eff;
                        target_angle <= abs_angle; timeout_ms <= TIMEOUT_MS;
                        desired_sign <= cmd_code == CMD_LEFT ||
                            (cmd_code == CMD_SPIN && cmd_value < 0);
                        turn_diff <= (cmd_code == CMD_LEFT ||
                            (cmd_code == CMD_SPIN && cmd_value < 0)) ? TURN_DIFF : -TURN_DIFF;
                    end
                    CMD_PATROL_START: begin
                        o_patrol_start <= 1; o_owned <= 0;
                    end
                    CMD_PATROL_STOP: o_patrol_stop <= 1;
                    default: ;
                endcase
            end else if (cmd_valid && cmd_code == CMD_PATROL_START && !i_nav_release_ready) begin
                // A refused release is a stop/hold, never a handoff to nav.
                mode <= MODE_IDLE; drive_center <= 0; turn_diff <= 0;
                o_owned <= 1; motion_error <= 1;
            end else begin
                if (cmd_valid && !semantic_ok) motion_error <= 1;
                if ((mode == MODE_TURN && !imu_fresh) ||
                    (ms_tick && !accepted && mode != MODE_IDLE && wd_ms >= WATCHDOG_MS-1)) begin
                    mode <= MODE_IDLE; drive_center <= 0; turn_diff <= 0;
                    o_owned <= 1; motion_error <= 1;
                end else if (ms_tick && mode == MODE_DRIVE && remaining_ms <= 1) begin
                    mode <= MODE_IDLE; remaining_ms <= 0; drive_center <= 0;
                end else if (ms_tick && mode == MODE_TURN && timeout_ms <= 1) begin
                    mode <= MODE_IDLE; timeout_ms <= 0; turn_diff <= 0; motion_error <= 1;
                end else begin
                    if (ms_tick && mode == MODE_DRIVE) remaining_ms <= remaining_ms - 1;
                    if (ms_tick && mode == MODE_TURN) timeout_ms <= timeout_ms - 1;
                    // Sensor samples are discrete. Signed target comparisons
                    // preserve a small reverse perturbation instead of turning
                    // -1 into a giant unsigned "completed" angle.
                    if (mode == MODE_TURN && sample_ok) begin
                        accum <= next_accum; yaw_prev <= yaw_eff;
                        if (desired_sign ? next_accum >= target_angle : next_accum <= -target_angle) begin
                            mode <= MODE_IDLE; turn_diff <= 0;
                        end
                    end
                end
            end
        end
    end

    initial begin
        if (BYTE_TIMEOUT_MS < 1) $fatal(1, "byte timeout must be positive");
        if (CLK_HZ < 1000) $fatal(1, "voice_motion_bridge CLK_HZ too low");
        if (MAX_CENTER < 1 || TURN_DIFF < 1)
            $fatal(1, "voice_motion_bridge motion limits must be positive");
        if (WATCHDOG_MS < 50 || TIMEOUT_MS < 100)
            $fatal(1, "voice_motion_bridge watchdog/timeout too small");
    end
endmodule
