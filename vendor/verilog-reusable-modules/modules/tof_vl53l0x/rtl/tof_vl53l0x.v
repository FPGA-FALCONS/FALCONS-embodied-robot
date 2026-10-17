// VL53L0X single-shot distance controller. All logic uses clk.
// Reference initialization/tuning: Pololu VL53L0X.cpp (MIT); see LICENSE.
// No continuous-mode tearing: read status/high/low BEFORE acknowledging IRQ.
// rst_n may assert asynchronously; release is synchronized internally.
// retry is synchronous; its rising edge retries initialization from XSHUT.
// xshut_n, scl and sda drive LOW or high impedance only; external pull-ups required.
// Default profile retains tuning register timeouts (no runtime timing-budget API).
// fault_code: 0 none; 1 I2C NACK; 2 SCL timeout; 3 bus stuck;
// 4 wrong model ID; 5 SPAD-info poll timeout; 6 calibration timeout;
// 7 single-shot START timeout; 8 range-ready timeout; 9 invalid factory SPAD map;
// 10 stale sample (nonfatal, cleared by a new valid sample).
// Fatal errors remain latched until retry or reset. Raw status other than 11
// invalidates the sample without aborting subsequent acquisition.
module tof_vl53l0x #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 100000,
    parameter integer I2C_TIMEOUT_US = 2000,
    parameter integer BOOT_DELAY_US = 10000,
    parameter integer RESET_LOW_US = 1000,
    parameter integer POLL_INTERVAL_US = 1000,
    parameter integer INIT_TIMEOUT_US = 100000,
    parameter integer MEAS_TIMEOUT_US = 100000,
    parameter integer SAMPLE_PERIOD_US = 50000,
    parameter integer STALE_TIMEOUT_US = 200000
) (
    input wire clk, input wire rst_n, input wire retry,
    inout wire scl, inout wire sda, inout wire xshut_n,
    output reg [15:0] distance_mm,
    output reg range_valid, output reg sample_pulse,
    output reg initialized, output wire busy,
    output reg [3:0] raw_range_status,
    output reg [4:0] fault_code
);
    // ceil division avoids a zero prescaler for simulation clocks < 1 MHz.
    localparam integer US_DIV = (CLK_HZ + 999999) / 1000000;
    localparam [31:0] US_LAST = US_DIV - 1;
    localparam [31:0] RESET_LIMIT = RESET_LOW_US > 0 ? RESET_LOW_US - 1 : 0;
    localparam [31:0] BOOT_LIMIT = BOOT_DELAY_US > 0 ? BOOT_DELAY_US - 1 : 0;
    localparam [31:0] POLL_LIMIT = POLL_INTERVAL_US > 0 ? POLL_INTERVAL_US - 1 : 0;
    localparam [31:0] SAMPLE_LIMIT = SAMPLE_PERIOD_US;
    localparam [31:0] STALE_LIMIT = STALE_TIMEOUT_US;
    localparam [31:0] INIT_LIMIT = INIT_TIMEOUT_US;
    localparam [31:0] MEAS_LIMIT = MEAS_TIMEOUT_US;
    reg [1:0] reset_pipe;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) reset_pipe <= 2'b00;
        else reset_pipe <= {reset_pipe[0], 1'b1};
    wire reset_ok = reset_pipe[1];
    reg retry_d;
    wire retry_edge = retry && !retry_d;
    // The master also resets on retry so an interrupted transaction cannot
    // deliver an old response into a new initialization sequence.
    wire master_reset_n = reset_ok && !retry_edge;
    reg cmd_valid, cmd_read;
    reg [7:0] cmd_reg, cmd_wdata;
    wire cmd_ready, rsp_valid;
    wire [7:0] rsp_rdata;
    wire [2:0] rsp_error;
    tof_i2c_reg_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(I2C_HZ),
        .I2C_TIMEOUT_US(I2C_TIMEOUT_US), .DEVICE_ADDR(7'h29)) master (
        .clk(clk), .rst_n(master_reset_n), .cmd_valid(cmd_valid),
        .cmd_read(cmd_read), .cmd_reg(cmd_reg), .cmd_wdata(cmd_wdata),
        .cmd_ready(cmd_ready), .rsp_valid(rsp_valid), .rsp_rdata(rsp_rdata),
        .rsp_error(rsp_error), .scl(scl), .sda(sda));
    localparam [3:0] S_RESET=0, S_BOOT=1, S_FETCH=2, S_SEND=3,
        S_RESPONSE=4, S_RMW_SEND=5, S_RMW_RESPONSE=6, S_FILTER=7,
        S_POLL_DELAY=8, S_SAMPLE_WAIT=9, S_FAULT=10;
    localparam [4:0] OP_ID=5'd0, OP_OR=5'd1, OP_WRITE=5'd2, OP_STOPVAR=5'd3, OP_TIMER=5'd4, OP_POLL_SPAD=5'd5, OP_SPADINFO=5'd6, OP_AND=5'd7, OP_READMAP=5'd8, OP_FILTER=5'd9, OP_WRITEMAP=5'd10, OP_POLL_CAL=5'd11, OP_INITIALIZED=5'd12, OP_WAIT_SAMPLE=5'd13, OP_RESTORE=5'd14, OP_POLL_START=5'd15, OP_POLL_RANGE=5'd16, OP_STATUS=5'd17, OP_HIGH=5'd18, OP_LOW=5'd19, OP_PUBLISH=5'd20, OP_JUMP=5'd21;
    function [20:0] instruction;
        input [7:0] address;
        begin
            case(address)
                8'd0: instruction = {OP_ID, 8'hc0, 8'h00};
                8'd1: instruction = {OP_OR, 8'h89, 8'h01};
                8'd2: instruction = {OP_WRITE, 8'h88, 8'h00};
                8'd3: instruction = {OP_WRITE, 8'h80, 8'h01};
                8'd4: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd5: instruction = {OP_WRITE, 8'h00, 8'h00};
                8'd6: instruction = {OP_STOPVAR, 8'h91, 8'h00};
                8'd7: instruction = {OP_WRITE, 8'h00, 8'h01};
                8'd8: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd9: instruction = {OP_WRITE, 8'h80, 8'h00};
                8'd10: instruction = {OP_OR, 8'h60, 8'h12};
                8'd11: instruction = {OP_WRITE, 8'h44, 8'h00};
                8'd12: instruction = {OP_WRITE, 8'h45, 8'h20};
                8'd13: instruction = {OP_WRITE, 8'h01, 8'hff};
                8'd14: instruction = {OP_WRITE, 8'h80, 8'h01};
                8'd15: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd16: instruction = {OP_WRITE, 8'h00, 8'h00};
                8'd17: instruction = {OP_WRITE, 8'hff, 8'h06};
                8'd18: instruction = {OP_OR, 8'h83, 8'h04};
                8'd19: instruction = {OP_WRITE, 8'hff, 8'h07};
                8'd20: instruction = {OP_WRITE, 8'h81, 8'h01};
                8'd21: instruction = {OP_WRITE, 8'h80, 8'h01};
                8'd22: instruction = {OP_WRITE, 8'h94, 8'h6b};
                8'd23: instruction = {OP_WRITE, 8'h83, 8'h00};
                8'd24: instruction = {OP_TIMER, 8'h00, 8'h00};
                8'd25: instruction = {OP_POLL_SPAD, 8'h83, 8'h00};
                8'd26: instruction = {OP_WRITE, 8'h83, 8'h01};
                8'd27: instruction = {OP_SPADINFO, 8'h92, 8'h00};
                8'd28: instruction = {OP_WRITE, 8'h81, 8'h00};
                8'd29: instruction = {OP_WRITE, 8'hff, 8'h06};
                8'd30: instruction = {OP_AND, 8'h83, 8'hfb};
                8'd31: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd32: instruction = {OP_WRITE, 8'h00, 8'h01};
                8'd33: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd34: instruction = {OP_WRITE, 8'h80, 8'h00};
                8'd35: instruction = {OP_READMAP, 8'hb0, 8'h00};
                8'd36: instruction = {OP_READMAP, 8'hb1, 8'h01};
                8'd37: instruction = {OP_READMAP, 8'hb2, 8'h02};
                8'd38: instruction = {OP_READMAP, 8'hb3, 8'h03};
                8'd39: instruction = {OP_READMAP, 8'hb4, 8'h04};
                8'd40: instruction = {OP_READMAP, 8'hb5, 8'h05};
                8'd41: instruction = {OP_FILTER, 8'h00, 8'h00};
                8'd42: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd43: instruction = {OP_WRITE, 8'h4f, 8'h00};
                8'd44: instruction = {OP_WRITE, 8'h4e, 8'h2c};
                8'd45: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd46: instruction = {OP_WRITE, 8'hb6, 8'hb4};
                8'd47: instruction = {OP_WRITEMAP, 8'hb0, 8'h00};
                8'd48: instruction = {OP_WRITEMAP, 8'hb1, 8'h01};
                8'd49: instruction = {OP_WRITEMAP, 8'hb2, 8'h02};
                8'd50: instruction = {OP_WRITEMAP, 8'hb3, 8'h03};
                8'd51: instruction = {OP_WRITEMAP, 8'hb4, 8'h04};
                8'd52: instruction = {OP_WRITEMAP, 8'hb5, 8'h05};
                8'd53: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd54: instruction = {OP_WRITE, 8'h00, 8'h00};
                8'd55: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd56: instruction = {OP_WRITE, 8'h09, 8'h00};
                8'd57: instruction = {OP_WRITE, 8'h10, 8'h00};
                8'd58: instruction = {OP_WRITE, 8'h11, 8'h00};
                8'd59: instruction = {OP_WRITE, 8'h24, 8'h01};
                8'd60: instruction = {OP_WRITE, 8'h25, 8'hff};
                8'd61: instruction = {OP_WRITE, 8'h75, 8'h00};
                8'd62: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd63: instruction = {OP_WRITE, 8'h4e, 8'h2c};
                8'd64: instruction = {OP_WRITE, 8'h48, 8'h00};
                8'd65: instruction = {OP_WRITE, 8'h30, 8'h20};
                8'd66: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd67: instruction = {OP_WRITE, 8'h30, 8'h09};
                8'd68: instruction = {OP_WRITE, 8'h54, 8'h00};
                8'd69: instruction = {OP_WRITE, 8'h31, 8'h04};
                8'd70: instruction = {OP_WRITE, 8'h32, 8'h03};
                8'd71: instruction = {OP_WRITE, 8'h40, 8'h83};
                8'd72: instruction = {OP_WRITE, 8'h46, 8'h25};
                8'd73: instruction = {OP_WRITE, 8'h60, 8'h00};
                8'd74: instruction = {OP_WRITE, 8'h27, 8'h00};
                8'd75: instruction = {OP_WRITE, 8'h50, 8'h06};
                8'd76: instruction = {OP_WRITE, 8'h51, 8'h00};
                8'd77: instruction = {OP_WRITE, 8'h52, 8'h96};
                8'd78: instruction = {OP_WRITE, 8'h56, 8'h08};
                8'd79: instruction = {OP_WRITE, 8'h57, 8'h30};
                8'd80: instruction = {OP_WRITE, 8'h61, 8'h00};
                8'd81: instruction = {OP_WRITE, 8'h62, 8'h00};
                8'd82: instruction = {OP_WRITE, 8'h64, 8'h00};
                8'd83: instruction = {OP_WRITE, 8'h65, 8'h00};
                8'd84: instruction = {OP_WRITE, 8'h66, 8'ha0};
                8'd85: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd86: instruction = {OP_WRITE, 8'h22, 8'h32};
                8'd87: instruction = {OP_WRITE, 8'h47, 8'h14};
                8'd88: instruction = {OP_WRITE, 8'h49, 8'hff};
                8'd89: instruction = {OP_WRITE, 8'h4a, 8'h00};
                8'd90: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd91: instruction = {OP_WRITE, 8'h7a, 8'h0a};
                8'd92: instruction = {OP_WRITE, 8'h7b, 8'h00};
                8'd93: instruction = {OP_WRITE, 8'h78, 8'h21};
                8'd94: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd95: instruction = {OP_WRITE, 8'h23, 8'h34};
                8'd96: instruction = {OP_WRITE, 8'h42, 8'h00};
                8'd97: instruction = {OP_WRITE, 8'h44, 8'hff};
                8'd98: instruction = {OP_WRITE, 8'h45, 8'h26};
                8'd99: instruction = {OP_WRITE, 8'h46, 8'h05};
                8'd100: instruction = {OP_WRITE, 8'h40, 8'h40};
                8'd101: instruction = {OP_WRITE, 8'h0e, 8'h06};
                8'd102: instruction = {OP_WRITE, 8'h20, 8'h1a};
                8'd103: instruction = {OP_WRITE, 8'h43, 8'h40};
                8'd104: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd105: instruction = {OP_WRITE, 8'h34, 8'h03};
                8'd106: instruction = {OP_WRITE, 8'h35, 8'h44};
                8'd107: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd108: instruction = {OP_WRITE, 8'h31, 8'h04};
                8'd109: instruction = {OP_WRITE, 8'h4b, 8'h09};
                8'd110: instruction = {OP_WRITE, 8'h4c, 8'h05};
                8'd111: instruction = {OP_WRITE, 8'h4d, 8'h04};
                8'd112: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd113: instruction = {OP_WRITE, 8'h44, 8'h00};
                8'd114: instruction = {OP_WRITE, 8'h45, 8'h20};
                8'd115: instruction = {OP_WRITE, 8'h47, 8'h08};
                8'd116: instruction = {OP_WRITE, 8'h48, 8'h28};
                8'd117: instruction = {OP_WRITE, 8'h67, 8'h00};
                8'd118: instruction = {OP_WRITE, 8'h70, 8'h04};
                8'd119: instruction = {OP_WRITE, 8'h71, 8'h01};
                8'd120: instruction = {OP_WRITE, 8'h72, 8'hfe};
                8'd121: instruction = {OP_WRITE, 8'h76, 8'h00};
                8'd122: instruction = {OP_WRITE, 8'h77, 8'h00};
                8'd123: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd124: instruction = {OP_WRITE, 8'h0d, 8'h01};
                8'd125: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd126: instruction = {OP_WRITE, 8'h80, 8'h01};
                8'd127: instruction = {OP_WRITE, 8'h01, 8'hf8};
                8'd128: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd129: instruction = {OP_WRITE, 8'h8e, 8'h01};
                8'd130: instruction = {OP_WRITE, 8'h00, 8'h01};
                8'd131: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd132: instruction = {OP_WRITE, 8'h80, 8'h00};
                8'd133: instruction = {OP_WRITE, 8'h0a, 8'h04};
                8'd134: instruction = {OP_AND, 8'h84, 8'hef};
                8'd135: instruction = {OP_WRITE, 8'h0b, 8'h01};
                8'd136: instruction = {OP_WRITE, 8'h01, 8'he8};
                8'd137: instruction = {OP_WRITE, 8'h01, 8'h01};
                8'd138: instruction = {OP_WRITE, 8'h00, 8'h41};
                8'd139: instruction = {OP_TIMER, 8'h00, 8'h00};
                8'd140: instruction = {OP_POLL_CAL, 8'h13, 8'h00};
                8'd141: instruction = {OP_WRITE, 8'h0b, 8'h01};
                8'd142: instruction = {OP_WRITE, 8'h00, 8'h00};
                8'd143: instruction = {OP_WRITE, 8'h01, 8'h02};
                8'd144: instruction = {OP_WRITE, 8'h00, 8'h01};
                8'd145: instruction = {OP_TIMER, 8'h00, 8'h00};
                8'd146: instruction = {OP_POLL_CAL, 8'h13, 8'h00};
                8'd147: instruction = {OP_WRITE, 8'h0b, 8'h01};
                8'd148: instruction = {OP_WRITE, 8'h00, 8'h00};
                8'd149: instruction = {OP_WRITE, 8'h01, 8'he8};
                8'd150: instruction = {OP_INITIALIZED, 8'h00, 8'h00};
                8'd151: instruction = {OP_WAIT_SAMPLE, 8'h00, 8'h00};
                8'd152: instruction = {OP_WRITE, 8'h80, 8'h01};
                8'd153: instruction = {OP_WRITE, 8'hff, 8'h01};
                8'd154: instruction = {OP_WRITE, 8'h00, 8'h00};
                8'd155: instruction = {OP_RESTORE, 8'h91, 8'h00};
                8'd156: instruction = {OP_WRITE, 8'h00, 8'h01};
                8'd157: instruction = {OP_WRITE, 8'hff, 8'h00};
                8'd158: instruction = {OP_WRITE, 8'h80, 8'h00};
                8'd159: instruction = {OP_WRITE, 8'h00, 8'h01};
                8'd160: instruction = {OP_TIMER, 8'h00, 8'h00};
                8'd161: instruction = {OP_POLL_START, 8'h00, 8'h00};
                8'd162: instruction = {OP_POLL_RANGE, 8'h13, 8'h00};
                8'd163: instruction = {OP_STATUS, 8'h14, 8'h00};
                8'd164: instruction = {OP_HIGH, 8'h1e, 8'h00};
                8'd165: instruction = {OP_LOW, 8'h1f, 8'h00};
                8'd166: instruction = {OP_WRITE, 8'h0b, 8'h01};
                8'd167: instruction = {OP_PUBLISH, 8'h00, 8'h00};
                8'd168: instruction = {OP_JUMP, 8'h97, 8'h00};
                default: instruction = {OP_JUMP, 8'd0, 8'd0};
            endcase
        end
    endfunction
    reg [3:0] state;
    reg [7:0] pc;
    wire [20:0] inst = instruction(pc);
    wire [4:0] inst_op = inst[20:16];
    wire [7:0] inst_reg = inst[15:8];
    wire [7:0] inst_data = inst[7:0];
    reg [4:0] active_op;
    reg [7:0] active_data;
    reg [31:0] us_divider, delay_us, poll_elapsed_us;
    reg [31:0] sample_elapsed_us, stale_elapsed_us;
    wire us_tick = us_divider == US_LAST;
    reg [7:0] stop_variable;
    reg [47:0] spad_map;
    reg [6:0] spad_count, spads_enabled;
    reg spad_aperture;
    reg [5:0] spad_index;
    reg [15:0] pending_distance;
    reg [3:0] pending_status;
    // Hold the sensor shut down after a fatal failure. A retry always starts
    // with a fresh boot rather than continuing an unknown register-bank state.
    assign xshut_n = (state == S_RESET || state == S_FAULT) ? 1'b0 : 1'bz;
    assign busy = state != S_SAMPLE_WAIT && state != S_FAULT;
    task fatal;
        input [4:0] code;
        begin
            fault_code <= code;
            range_valid <= 1'b0;
            initialized <= 1'b0;
            cmd_valid <= 1'b0;
            state <= S_FAULT;
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            retry_d <= 1'b0;
            state <= S_RESET; pc <= 8'd0;
            cmd_valid <= 1'b0; cmd_read <= 1'b0;
            cmd_reg <= 8'd0; cmd_wdata <= 8'd0;
            active_op <= 5'd0; active_data <= 8'd0;
            us_divider <= 32'd0; delay_us <= 32'd0;
            poll_elapsed_us <= 32'd0; sample_elapsed_us <= 32'd0;
            stale_elapsed_us <= 32'd0;
            stop_variable <= 8'd0; spad_map <= 48'd0;
            spad_count <= 7'd0; spads_enabled <= 7'd0;
            spad_aperture <= 1'b0; spad_index <= 6'd0;
            pending_distance <= 16'd0; pending_status <= 4'd0;
            distance_mm <= 16'd0; range_valid <= 1'b0;
            sample_pulse <= 1'b0; initialized <= 1'b0;
            raw_range_status <= 4'd0; fault_code <= 5'd0;
        end else begin
            retry_d <= retry;
            sample_pulse <= 1'b0;
            if (!reset_ok || retry_edge) begin
                state <= S_RESET; pc <= 8'd0;
                cmd_valid <= 1'b0; cmd_read <= 1'b0;
                cmd_reg <= 8'd0; cmd_wdata <= 8'd0;
                us_divider <= 32'd0; delay_us <= 32'd0;
                poll_elapsed_us <= 32'd0; sample_elapsed_us <= 32'd0;
                stale_elapsed_us <= 32'd0;
                range_valid <= 1'b0; initialized <= 1'b0;
                distance_mm <= 16'd0; raw_range_status <= 4'd0;
                fault_code <= 5'd0;
                stop_variable <= 8'd0; spad_map <= 48'd0;
                spads_enabled <= 7'd0; spad_count <= 7'd0;
                spad_aperture <= 1'b0; spad_index <= 6'd0;
                pending_distance <= 16'd0; pending_status <= 4'd0;
            end else begin
                if (us_tick) begin
                    us_divider <= 32'd0;
                    if (poll_elapsed_us != 32'hffffffff)
                        poll_elapsed_us <= poll_elapsed_us + 32'd1;
                    if (sample_elapsed_us != 32'hffffffff)
                        sample_elapsed_us <= sample_elapsed_us + 32'd1;
                    if (stale_elapsed_us != 32'hffffffff)
                        stale_elapsed_us <= stale_elapsed_us + 32'd1;
                end else us_divider <= us_divider + 32'd1;
                if (range_valid && stale_elapsed_us >= STALE_LIMIT) begin
                    range_valid <= 1'b0;
                    fault_code <= 5'd10;
                end
                case(state)
                    S_RESET: if (us_tick) begin
                        if (delay_us >= RESET_LIMIT) begin
                            state <= S_BOOT; delay_us <= 32'd0;
                        end else delay_us <= delay_us + 32'd1;
                    end
                    S_BOOT: if (us_tick) begin
                        if (delay_us >= BOOT_LIMIT) begin
                            state <= S_FETCH; delay_us <= 32'd0;
                        end else delay_us <= delay_us + 32'd1;
                    end
                    S_FETCH: begin
                        active_op <= inst_op; active_data <= inst_data;
                        case(inst_op)
                            OP_TIMER: begin
                                poll_elapsed_us <= 32'd0;
                                pc <= pc + 8'd1;
                            end
                            OP_FILTER: begin
                                spads_enabled <= 7'd0; spad_index <= 6'd0;
                                state <= S_FILTER;
                            end
                            OP_INITIALIZED: begin
                                initialized <= 1'b1;
                                sample_elapsed_us <= SAMPLE_LIMIT;
                                pc <= pc + 8'd1;
                            end
                            OP_WAIT_SAMPLE: state <= S_SAMPLE_WAIT;
                            OP_PUBLISH: begin
                                raw_range_status <= pending_status;
                                distance_mm <= pending_distance;
                                range_valid <= pending_status == 4'd11;
                                sample_pulse <= 1'b1;
                                if (pending_status == 4'd11) begin
                                    stale_elapsed_us <= 32'd0;
                                    fault_code <= 5'd0;
                                end
                                pc <= pc + 8'd1;
                            end
                            OP_JUMP: pc <= inst_reg;
                            default: begin
                                cmd_reg <= inst_reg;
                                cmd_read <= inst_op != OP_WRITE &&
                                    inst_op != OP_WRITEMAP && inst_op != OP_RESTORE;
                                case(inst_op)
                                    OP_WRITEMAP: cmd_wdata <= spad_map[{inst_data[2:0],3'b000} +: 8];
                                    OP_RESTORE: cmd_wdata <= stop_variable;
                                    default: cmd_wdata <= inst_data;
                                endcase
                                cmd_valid <= 1'b1;
                                state <= S_SEND;
                            end
                        endcase
                    end
                    S_SEND: if (cmd_ready) begin
                        cmd_valid <= 1'b0;
                        state <= S_RESPONSE;
                    end
                    S_RESPONSE: if (rsp_valid) begin
                        if (rsp_error != 3'd0) fatal({2'b00,rsp_error});
                        else begin
                            state <= S_FETCH;
                            pc <= pc + 8'd1;
                            case(active_op)
                                OP_ID: if (rsp_rdata != 8'hee) fatal(5'd4);
                                OP_STOPVAR: stop_variable <= rsp_rdata;
                                OP_OR, OP_AND: begin
                                    cmd_read <= 1'b0;
                                    cmd_wdata <= active_op == OP_OR ?
                                        rsp_rdata | active_data : rsp_rdata & active_data;
                                    cmd_valid <= 1'b1;
                                    state <= S_RMW_SEND;
                                end
                                OP_SPADINFO: begin
                                    spad_count <= rsp_rdata[6:0];
                                    spad_aperture <= rsp_rdata[7];
                                    if (rsp_rdata[6:0] == 7'd0) fatal(5'd9);
                                end
                                OP_READMAP: spad_map[{active_data[2:0],3'b000} +: 8] <= rsp_rdata;
                                OP_STATUS: pending_status <= rsp_rdata[6:3];
                                OP_HIGH: pending_distance[15:8] <= rsp_rdata;
                                OP_LOW: pending_distance[7:0] <= rsp_rdata;
                                OP_POLL_SPAD, OP_POLL_CAL, OP_POLL_START, OP_POLL_RANGE: begin
                                    if ((active_op == OP_POLL_SPAD && rsp_rdata == 8'd0) ||
                                        (active_op == OP_POLL_START && rsp_rdata[0]) ||
                                        ((active_op == OP_POLL_CAL || active_op == OP_POLL_RANGE) &&
                                         rsp_rdata[2:0] == 3'd0)) begin
                                        if (poll_elapsed_us >=
                                            ((active_op == OP_POLL_SPAD || active_op == OP_POLL_CAL) ?
                                             INIT_LIMIT : MEAS_LIMIT)) begin
                                            case(active_op)
                                                OP_POLL_SPAD: fatal(5'd5);
                                                OP_POLL_CAL: fatal(5'd6);
                                                OP_POLL_START: fatal(5'd7);
                                                default: fatal(5'd8);
                                            endcase
                                        end else begin
                                            pc <= pc;
                                            delay_us <= 32'd0;
                                            state <= S_POLL_DELAY;
                                        end
                                    end
                                end
                                default: begin end
                            endcase
                        end
                    end
                    S_RMW_SEND: if (cmd_ready) begin
                        cmd_valid <= 1'b0;
                        state <= S_RMW_RESPONSE;
                    end
                    S_RMW_RESPONSE: if (rsp_valid) begin
                        if (rsp_error != 3'd0) fatal({2'b00,rsp_error});
                        else state <= S_FETCH;
                    end
                    S_FILTER: begin
                        if (spad_index == 6'd48) begin
                            if (spads_enabled != spad_count) fatal(5'd9);
                            else begin pc <= pc + 8'd1; state <= S_FETCH; end
                        end else begin
                            if ((spad_aperture && spad_index < 6'd12) ||
                                spads_enabled >= spad_count)
                                spad_map[spad_index] <= 1'b0;
                            else if (spad_map[spad_index])
                                spads_enabled <= spads_enabled + 7'd1;
                            spad_index <= spad_index + 6'd1;
                        end
                    end
                    S_POLL_DELAY: if (us_tick) begin
                        if (delay_us >= POLL_LIMIT) state <= S_FETCH;
                        else delay_us <= delay_us + 32'd1;
                    end
                    S_SAMPLE_WAIT: if (sample_elapsed_us >= SAMPLE_LIMIT) begin
                        sample_elapsed_us <= 32'd0;
                        pc <= pc + 8'd1;
                        state <= S_FETCH;
                    end
                    S_FAULT: begin cmd_valid <= 1'b0; end
                    default: fatal(5'd3);
                endcase
            end
        end
    end
endmodule
