//////////////////////////////////////////////////////////////////////////////////
// 模块：ai_vision_link —— RK3588S 视觉观察帧接收/校验（安全边界：只观察，不控车）
//
// 帧：AA 55 | LEN=14 | seq[2] flags class status conf x1[2] y1[2] x2[2] y2[2] | CRC16
// CRC16-CCITT-FALSE（poly 0x1021, init 0xFFFF），覆盖 LEN + payload。
//
// 安全规则（FPGA 独立复核，不信任主机）：
//   * 只有 flags[0]=1 且 status 为 TRACKING(0) 或 VERIFIED(2) 才可能 target_valid
//   * UNCERTAIN(3)/TIMEOUT(4)/VERIFY_PENDING(1) 永远不能携带可执行目标
//   * class 必须 0..3；confidence < MIN_CONF_Q8 不确认
//   * CRC/长度/状态非法：撤销目标，保留观测数据并给错误脉冲
//   * 连续 TIMEOUT_CYCLES 没有合法帧 → o_link_alive=0（下游应安全停车）
//
// 本模块不产生速度、不使能底盘、不替代 ToF 急停/堵转/遥控仲裁。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps

module ai_vision_link #(
    parameter integer CLK_FREQ_HZ   = 50_000_000,
    parameter integer BAUD_RATE     = 115_200,
    parameter integer TIMEOUT_CYCLES = 15_000_000, // 50 MHz 下 300 ms
    parameter integer MIN_CONF_Q8   = 77           // 约 0.302
)(
    input  wire        i_clock,
    input  wire        i_reset_n,
    input  wire        i_uart_rx,

    output reg  [15:0] o_seq,
    output reg  [7:0]  o_flags,
    output reg  [7:0]  o_class,
    output reg  [7:0]  o_status,
    output reg  [7:0]  o_confidence,
    output reg  [15:0] o_x1,
    output reg  [15:0] o_y1,
    output reg  [15:0] o_x2,
    output reg  [15:0] o_y2,

    output wire        o_target_valid,    // = target_valid_r && o_link_alive
    output reg         o_frame_valid,     // 单拍：CRC 正确的帧已更新
    output reg         o_crc_error,       // 单拍：CRC 错误
    output reg         o_frame_error,     // 单拍：长度/状态/非法目标声明
    output reg         o_policy_violation,// 单拍：主机把不可执行状态声明为 target_valid
    output reg         o_replay_error,    // 单拍：重复/倒退序号，目标立即撤销
    output reg         o_heartbeat,       // 单拍：flags[1]
    output reg         o_link_alive
);

    localparam [2:0] S_SOF1 = 3'd0, S_SOF2 = 3'd1, S_LEN = 3'd2,
                     S_PAY  = 3'd3, S_CRC_H = 3'd4, S_CRC_L = 3'd5;

    reg [2:0]  state;
    reg [3:0]  idx;
    reg [7:0]  payload [0:13];
    reg [7:0]  crc_hi;
    reg [15:0] crc_calc;
    reg [31:0] timeout_cnt;
    reg        seen_valid;
    reg        target_valid_r;
    reg [15:0] last_seq;
    reg        seq_seen;

    wire [7:0] rx_data;
    wire       rx_valid;

    uart_byte_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .RECOVERY_EN (1)
    ) u_uart (
        .i_clock   (i_clock),
        .i_reset_n (i_reset_n),
        .i_uart_rx (i_uart_rx),
        .o_rx_data (rx_data),
        .o_rx_valid(rx_valid),
        .i_rx_ready(1'b1),
        .o_rx_error()
    );

    function [15:0] crc16_byte;
        input [15:0] crc;
        input [7:0]  data;
        integer i;
        reg [15:0] c;
        begin
            c = crc ^ {data, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte = c;
        end
    endfunction

    wire [7:0] p_flags   = payload[2];
    wire [7:0] p_class   = payload[3];
    wire [7:0] p_status  = payload[4];
    wire [7:0] p_conf    = payload[5];
    wire [15:0] p_seq    = {payload[0], payload[1]};
    wire       link_expired = seen_valid && (timeout_cnt >= TIMEOUT_CYCLES);
    wire [15:0] seq_delta = p_seq - last_seq;
    wire       seq_new   = !seq_seen || link_expired || (seq_delta != 16'd0 && seq_delta < 16'h8000);
    wire       claim     = p_flags[0];
    wire       valid_cls = (p_class < 8'd4);
    wire       valid_sta = (p_status < 8'd5);
    wire       status_ok = (p_status == 8'd0) || (p_status == 8'd2);
    wire       frame_ok  = valid_sta && (!claim || valid_cls);
    wire       policy_bad = claim && !status_ok;

    // target_valid_r is cleared when the link expires; keeping this output
    // registered makes the first valid frame observable immediately.
    assign o_target_valid = target_valid_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state <= S_SOF1; idx <= 0; crc_hi <= 0; crc_calc <= 16'hFFFF;
            timeout_cnt <= 0; seen_valid <= 0; target_valid_r <= 0; last_seq <= 0; seq_seen <= 0;
            o_seq <= 0; o_flags <= 0; o_class <= 8'hFF; o_status <= 0; o_confidence <= 0;
            o_x1 <= 0; o_y1 <= 0; o_x2 <= 0; o_y2 <= 0;
            o_frame_valid <= 0; o_crc_error <= 0;
            o_frame_error <= 0; o_policy_violation <= 0; o_replay_error <= 0;
            o_heartbeat <= 0; o_link_alive <= 0;
        end else begin
            o_frame_valid <= 1'b0;
            o_crc_error <= 1'b0;
            o_frame_error <= 1'b0;
            o_policy_violation <= 1'b0;
            o_replay_error <= 1'b0;
            o_heartbeat <= 1'b0;

            // Expired link: raw target must be false even if no new frame arrives.
            // A valid frame later in this same cycle is allowed to re-arm the seq.
            if (link_expired) begin
                target_valid_r <= 1'b0;
                seq_seen       <= 1'b0;
            end

            if (rx_valid) begin
                case (state)
                    S_SOF1: if (rx_data == 8'hAA) state <= S_SOF2;
                    S_SOF2: begin
                        if (rx_data == 8'h55) state <= S_LEN;
                        else if (rx_data != 8'hAA) state <= S_SOF1;
                    end
                    S_LEN: begin
                        if (rx_data == 8'd14) begin
                            crc_calc <= crc16_byte(16'hFFFF, rx_data);
                            idx <= 0;
                            state <= S_PAY;
                        end else begin
                            o_frame_error <= 1'b1;
                            state <= S_SOF1;
                        end
                    end
                    S_PAY: begin
                        payload[idx] <= rx_data;
                        crc_calc <= crc16_byte(crc_calc, rx_data);
                        if (idx == 4'd13) state <= S_CRC_H;
                        else idx <= idx + 1'b1;
                    end
                    S_CRC_H: begin
                        crc_hi <= rx_data;
                        state <= S_CRC_L;
                    end
                    S_CRC_L: begin
                        state <= S_SOF1;
                        if ({crc_hi, rx_data} != crc_calc) begin
                            o_crc_error <= 1'b1;
                            target_valid_r <= 1'b0;
                        end else if (!frame_ok) begin
                            o_frame_error <= 1'b1;
                            target_valid_r <= 1'b0;
                        end else if (!seq_new) begin
                            // Replay/out-of-order frame must not extend the target TTL.
                            o_replay_error <= 1'b1;
                            target_valid_r <= 1'b0;
                        end else begin
                            o_seq          <= p_seq;
                            o_flags        <= p_flags;
                            o_class        <= valid_cls ? p_class : 8'hFF;
                            o_status       <= p_status;
                            o_confidence   <= p_conf;
                            o_x1           <= {payload[6],  payload[7]};
                            o_y1           <= {payload[8],  payload[9]};
                            o_x2           <= {payload[10], payload[11]};
                            o_y2           <= {payload[12], payload[13]};
                            target_valid_r <= claim && valid_cls && status_ok && (p_conf >= MIN_CONF_Q8);
                            o_policy_violation <= policy_bad;
                            o_heartbeat    <= p_flags[1];
                            o_frame_valid  <= 1'b1;
                            seen_valid     <= 1'b1;
                            last_seq       <= p_seq;
                            seq_seen       <= 1'b1;
                            timeout_cnt    <= 0;
                        end
                    end
                    default: state <= S_SOF1;
                endcase
            end

            if (o_frame_valid) begin
                timeout_cnt <= 0;
            end else if (seen_valid && timeout_cnt < TIMEOUT_CYCLES) begin
                timeout_cnt <= timeout_cnt + 1'b1;
            end
            o_link_alive <= seen_valid && (timeout_cnt < TIMEOUT_CYCLES);
        end
    end
endmodule
