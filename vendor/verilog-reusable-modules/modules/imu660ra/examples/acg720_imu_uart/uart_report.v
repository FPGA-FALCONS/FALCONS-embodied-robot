//////////////////////////////////////////////////////////////////////////////////
// 模块：uart_report —— 把 IMU 六轴数据按 CSV 文本经串口输出
//
// 输出格式（每帧一行，6 个十进制有符号整数）：
//     gx,gy,gz,ax,ay,az\r\n
//   gx/gy/gz 单位 0.01°/s，ax/ay/az 单位 mg
//   例如：0,0,-1,12,-5,7980
//   （上位机解析时角度除以 100 得 °/s，加速度除以 1000 得 g）
//
// 说明：
//   * 采样 1kHz，但 115200 下每秒最多约 280 行，故本模块按 REPORT_DIV
//     分频上报（默认 50Hz），与采样率解耦
//   * 依赖模块库的 uart/rtl/uart_tx_queue.v（其内部再依赖 sync_fifo.v 与 uart_byte_tx.v）
//   * 字符发送用 valid/ready 握手，队列满时自动等待，不丢字符
//   * 字符发送握手统一为：发起 ch_start → 等 ch_pending 清除，各状态共用
//
// 参数：
//   CLK_FREQ_HZ      系统时钟频率(Hz)          默认 50_000_000
//   BAUD_RATE        串口波特率                默认 115_200
//   REPORT_DIV       上报间隔时钟数            默认 1_000_000 → 50Hz
//   FIFO_DEPTH_LOG2  发送 FIFO 地址位宽        默认 5 → 32 字节
//
// 端口：
//   i_clock        输入  系统时钟
//   i_reset_n      输入  异步低电平复位
//   i_gyro_x/y/z   输入  角速度，0.01°/s（int32）
//   i_acc_x/y/z    输入  加速度，mg（int16）
//   o_uart_tx      输出  串口发送线
//////////////////////////////////////////////////////////////////////////////////

module uart_report #(
    parameter integer CLK_FREQ_HZ     = 50_000_000,
    parameter integer BAUD_RATE       = 115_200,
    parameter integer REPORT_DIV      = 1_000_000,
    parameter integer FIFO_DEPTH_LOG2 = 5
)(
    input  wire        i_clock,
    input  wire        i_reset_n,

    input  wire signed [31:0] i_gyro_x,
    input  wire signed [31:0] i_gyro_y,
    input  wire signed [31:0] i_gyro_z,
    input  wire signed [15:0] i_acc_x,
    input  wire signed [15:0] i_acc_y,
    input  wire signed [15:0] i_acc_z,

    output wire        o_uart_tx
);

    //---------------------------------------------------------------
    // 状态与数据
    //---------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0,
                     S_PREP  = 3'd1,
                     S_DIGIT = 3'd2,
                     S_SEP   = 3'd3,
                     S_END   = 3'd4;

    reg [2:0]  state;
    reg [2:0]  field;          // 当前字段 0..5
    reg [31:0] cur_val;        // 当前字段绝对值
    reg [2:0]  widx;           // 权重索引 0..6 → 10^6 .. 10^0
    reg [3:0]  dcount;         // 本位已减次数 = 该位数字
    reg        out_started;    // 已输出过数字（去前导零）
    reg [1:0]  end_idx;        // 行尾字符索引

    // 上报节拍
    reg [31:0] tick_cnt;
    wire       report_tick = (tick_cnt == 32'd0) && (state == S_IDLE);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n)             tick_cnt <= 32'd0;
        else if (tick_cnt == 32'd0) tick_cnt <= REPORT_DIV;
        else                        tick_cnt <= tick_cnt - 32'd1;
    end

    //---------------------------------------------------------------
    // 字符发送通道
    //   ch_start 单周期发起；ch_busy 表示正在等待 FIFO 接收。
    //   ch_pending 必须用组合逻辑（ch_start | ch_busy）：
    //   若用寄存器版本会晚一拍，状态机在发起字符后的下一拍会重复执行同一
    //   分支，把 ch_byte 覆盖、widx/end_idx 多推进一步（表现为数字少一位、
    //   逗号乱插、\r 丢失）。
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
    // 权重查表：10^6 .. 10^0
    //   |gyro| <= 200000（0.01°/s），|acc| <= 8000（mg），7 位足够
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

    // 当前字段数据（统一扩展到 32 位有符号）
    reg signed [31:0] field_val;
    always @(*) begin
        case (field)
            3'd0:    field_val = i_gyro_x;
            3'd1:    field_val = i_gyro_y;
            3'd2:    field_val = i_gyro_z;
            3'd3:    field_val = {{16{i_acc_x[15]}}, i_acc_x};
            3'd4:    field_val = {{16{i_acc_y[15]}}, i_acc_y};
            default: field_val = {{16{i_acc_z[15]}}, i_acc_z};
        endcase
    end

    //---------------------------------------------------------------
    // 主状态机：所有"发字符"操作都在 !ch_pending 时发起，发起后本状态
    // 自动等待；需要切换状态的在发起同拍直接切换，下一状态同样会等。
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
            ch_start <= 1'b0;                   // 单周期脉冲

            case (state)
                //-----------------------------------------------
                S_IDLE: begin
                    if (report_tick) begin
                        field <= 3'd0;
                        state <= S_PREP;
                    end
                end
                //-----------------------------------------------
                // 取字段值、处理符号
                S_PREP: begin
                    if (!ch_pending) begin
                        if (field_val[31]) begin
                            cur_val  <= (~field_val) + 32'd1;   // 取绝对值
                            ch_byte  <= 8'h2D;                  // '-'
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
                //-----------------------------------------------
                // 逐位求数字：反复减去当前权重
                S_DIGIT: begin
                    if (!ch_pending) begin
                        if (cur_val >= cur_weight) begin
                            cur_val <= cur_val - cur_weight;
                            dcount  <= dcount + 4'd1;
                        end else begin
                            // 本位数字 = dcount；首位为 0 时跳过（去前导零）
                            if ((dcount != 4'd0) || out_started || (widx == 3'd6)) begin
                                ch_byte     <= {4'h3, dcount};          // '0'~'9'
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
                //-----------------------------------------------
                // 字段间分隔：逗号；最后一个字段转行尾
                S_SEP: begin
                    if (!ch_pending) begin
                        if (field == 3'd5) begin
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
                //-----------------------------------------------
                // 行尾 \r\n
                S_END: begin
                    if (!ch_pending) begin
                        case (end_idx)
                            2'd0: begin
                                ch_byte  <= 8'h0D;          // '\r'
                                ch_start <= 1'b1;
                                end_idx  <= 2'd1;
                            end
                            2'd1: begin
                                ch_byte  <= 8'h0A;          // '\n'
                                ch_start <= 1'b1;
                                end_idx  <= 2'd2;
                            end
                            default: state <= S_IDLE;
                        endcase
                    end
                end
                //-----------------------------------------------
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
