//////////////////////////////////////////////////////////////////////////////////
// 模块：speed_report —— 速度闭环调试上报（CSV 文本，115200 8N1）
//
// 输出格式（每帧一行，8 个十进制有符号整数）：
//     tgt,act,duty,kp,ki,kd,flags,err\r\n
//
//   | 列 | 含义 | 单位 / 说明 |
//   | --- | --- | --- |
//   | 1 | tgt   | 目标转速，单位【计数/测速窗口】，**rpm = 本列 ÷ 10** |
//   | 2 | act   | 实际转速，同单位，**rpm = 本列 ÷ 10** |
//   | 3 | duty  | 当前输出的占空比（有符号；负号 = 反转方向），满值 2^PWM_BITS |
//   | 4 | kp    | Kp，Q8 定点（256 = 1.0） |
//   | 5 | ki    | Ki，Q8 |
//   | 6 | kd    | Kd，Q8 |
//   | 7 | flags | bit0 = 闭环使能；bit1 = 任一堵转；bit2 = 命令解析出错；bit3 = 任一输出饱和 |
//   | 8 | err   | e(k) = tgt − act，同列 1/2 的单位 |
//
// 例：3000,2870,612,26,3,0,1,130
//     → 目标 300 rpm，实际 287 rpm，占空比 612（≈30%），Kp=0.1016，Ki=0.0117，
//       Kd=0，闭环运行中，还有 13 rpm 的静差（如果一直有静差 → 把 Ki 调大）
//
// 说明：
//   * 依赖 ext/uart_tx_queue.v（内部再依赖 ext/sync_fifo.v 与 uart_byte_tx.v）
//   * 上报周期与测速窗口解耦：测速 100 Hz，上报默认 50 Hz（REPORT_DIV = 1_000_000）
//   * i_force 单周期脉冲可在空闲时立刻插播一行（串口发 'R' 时用）
//   * ch_pending 必须用组合逻辑（见 uart_report.v 里踩过的坑）
//
// 参数：
//   CLK_FREQ_HZ      系统时钟频率(Hz)   默认 50_000_000
//   BAUD_RATE        串口波特率         默认 115_200
//   REPORT_DIV       上报间隔时钟数     默认 1_000_000 → 20ms（50Hz）
//   FIFO_DEPTH_LOG2  发送 FIFO 地址位宽 默认 5 → 32 字节
//////////////////////////////////////////////////////////////////////////////////

module speed_report #(
    parameter integer CLK_FREQ_HZ     = 50_000_000,
    parameter integer BAUD_RATE       = 115_200,
    parameter integer REPORT_DIV      = 1_000_000,
    parameter integer FIFO_DEPTH_LOG2 = 5
)(
    input  wire        i_clock,
    input  wire        i_reset_n,

    input  wire signed [31:0] i_tgt,
    input  wire signed [31:0] i_act,
    input  wire signed [31:0] i_duty,
    input  wire signed [31:0] i_kp,
    input  wire signed [31:0] i_ki,
    input  wire signed [31:0] i_kd,
    input  wire signed [31:0] i_flags,
    input  wire signed [31:0] i_err,

    input  wire        i_force,        // 单周期：立即插播一行
    output wire        o_uart_tx
);

    localparam [2:0] S_IDLE  = 3'd0,
                     S_PREP  = 3'd1,
                     S_DIGIT = 3'd2,
                     S_SEP   = 3'd3,
                     S_END   = 3'd4;

    reg [2:0]  state;
    reg [2:0]  field;
    reg [31:0] cur_val;
    reg [2:0]  widx;
    reg [3:0]  dcount;
    reg        out_started;
    reg [1:0]  end_idx;

    reg [31:0] tick_cnt;
    wire       report_tick = (tick_cnt == 32'd0) && (state == S_IDLE);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)             tick_cnt <= 32'd0;
        else if (tick_cnt == 32'd0) tick_cnt <= REPORT_DIV;
        else                        tick_cnt <= tick_cnt - 32'd1;
    end

    //---------------------------------------------------------------
    // 字符发送通道（ch_pending 必须是组合逻辑，用寄存器版会晚一拍）
    //---------------------------------------------------------------
    reg  [7:0] ch_byte;
    reg        ch_start;
    reg        ch_req;
    reg        ch_busy;
    wire       ch_ready;
    wire       ch_pending = ch_start | ch_busy;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            ch_req  <= 1'b0;
            ch_busy <= 1'b0;
        end else begin
            if (ch_start) begin
                ch_req  <= 1'b1;
                ch_busy <= 1'b1;
            end else if (ch_busy && ch_ready) begin
                ch_req  <= 1'b0;
                ch_busy <= 1'b0;
            end
        end
    end

    uart_tx_queue #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .BAUD_RATE       (BAUD_RATE),
        .DATA_BITS       (8),
        .PARITY_EN       (0),
        .PARITY_TYPE     (0),
        .FIFO_DEPTH_LOG2 (FIFO_DEPTH_LOG2)
    ) u_tx_queue (
        .i_clock    (i_clock),
        .i_reset_n  (i_reset_n),
        .i_tx_data  (ch_byte),
        .i_tx_valid (ch_req),
        .o_tx_ready (ch_ready),
        .o_uart_tx  (o_uart_tx)
    );

    //---------------------------------------------------------------
    // 权重查表 10^6 .. 10^0
    //---------------------------------------------------------------
    reg [31:0] cur_weight;
    always @(*) begin
        case (widx)
            3'd0:    cur_weight = 32'd1000000;
            3'd1:    cur_weight = 32'd100000;
            3'd2:    cur_weight = 32'd10000;
            3'd3:    cur_weight = 32'd1000;
            3'd4:    cur_weight = 32'd100;
            3'd5:    cur_weight = 32'd10;
            default: cur_weight = 32'd1;
        endcase
    end

    reg signed [31:0] field_val;
    always @(*) begin
        case (field)
            3'd0:    field_val = i_tgt;
            3'd1:    field_val = i_act;
            3'd2:    field_val = i_duty;
            3'd3:    field_val = i_kp;
            3'd4:    field_val = i_ki;
            3'd5:    field_val = i_kd;
            3'd6:    field_val = i_flags;
            default: field_val = i_err;
        endcase
    end

    //---------------------------------------------------------------
    // 主状态机
    //---------------------------------------------------------------
    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state       <= S_IDLE;
            field       <= 3'd0;
            cur_val     <= 32'd0;
            widx        <= 3'd0;
            dcount      <= 4'd0;
            out_started <= 1'b0;
            end_idx     <= 2'd0;
            ch_byte     <= 8'h00;
            ch_start    <= 1'b0;
        end else begin
            ch_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (report_tick || i_force) begin
                        field <= 3'd0;
                        state <= S_PREP;
                    end
                end
                S_PREP: begin
                    if (!ch_pending) begin
                        if (field_val[31]) begin
                            cur_val  <= (~field_val) + 32'd1;
                            ch_byte  <= 8'h2D;              // '-'
                            ch_start <= 1'b1;
                        end else begin
                            cur_val  <= field_val;
                        end
                        widx        <= 3'd0;
                        dcount      <= 4'd0;
                        out_started <= 1'b0;
                        state       <= S_DIGIT;
                    end
                end
                S_DIGIT: begin
                    if (!ch_pending) begin
                        if (cur_val >= cur_weight) begin
                            cur_val <= cur_val - cur_weight;
                            dcount  <= dcount + 4'd1;
                        end else begin
                            if ((dcount != 4'd0) || out_started || (widx == 3'd6)) begin
                                ch_byte     <= {4'h3, dcount};
                                ch_start    <= 1'b1;
                                out_started <= 1'b1;
                            end
                            if (widx == 3'd6) begin
                                state <= S_SEP;
                            end else begin
                                widx   <= widx + 3'd1;
                                dcount <= 4'd0;
                            end
                        end
                    end
                end
                S_SEP: begin
                    if (!ch_pending) begin
                        if (field == 3'd7) begin
                            end_idx <= 2'd0;
                            state   <= S_END;
                        end else begin
                            ch_byte  <= 8'h2C;              // ','
                            ch_start <= 1'b1;
                            field    <= field + 3'd1;
                            state    <= S_PREP;
                        end
                    end
                end
                S_END: begin
                    if (!ch_pending) begin
                        case (end_idx)
                            2'd0: begin
                                ch_byte  <= 8'h0D;
                                ch_start <= 1'b1;
                                end_idx  <= 2'd1;
                            end
                            2'd1: begin
                                ch_byte  <= 8'h0A;
                                ch_start <= 1'b1;
                                end_idx  <= 2'd2;
                            end
                            default: state <= S_IDLE;
                        endcase
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
