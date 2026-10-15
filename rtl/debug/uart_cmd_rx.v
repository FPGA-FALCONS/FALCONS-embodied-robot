//////////////////////////////////////////////////////////////////////////////////
// 模块：uart_cmd_rx —— 串口命令解析（ASCII 文本，115200 8N1）
//
// 【为什么要有它】调参必须在【不重新综合】的前提下完成：综合 + 布局布线 + 下载
//   一次要一两分钟，靠改 parameter 调 PID 参数是完全不可接受的。本模块把所有
//   可调量做成运行时可写的寄存器，用串口随时改、随时看效果。
//
// 【命令集】（大小写均可；数值可带负号；行尾 \r 或 \n 均可，两者都当结束符）
//   T<rpm>     四路目标转速全部设为 rpm         例：T300
//   1<rpm>     第 1 路(A) 目标转速               例：1300
//   2<rpm>     第 2 路(B)                       3<rpm> 第 3 路(C)   4<rpm> 第 4 路(D)
//   P<n>       Kp（Q8 定点整数）                例：P26     → 0.1016
//   I<n>       Ki（Q8）                        例：I3      → 0.0117
//   D<n>       Kd（Q8）                        例：D0      → 0
//   E<0|1>     闭环使能：1 = 闭环，0 = 开环（用 U 的占空比）
//   U<duty>    开环占空比（0 ~ 2^PWM_BITS−1）   例：U512    → 25%
//   Z          位置计数清零（立即）
//   X          清除堵转锁存（立即）
//   R          立即回读一行（不等下一个上报周期）
//
// 【Q8 换算表】数值 = 实际系数 × 256，所以要设 0.05 就发 P13：
//   0.005→1   0.01→3    0.02→5    0.05→13   0.1→26
//   0.2→51    0.5→128   1.0→256   2.0→512   4.0→1024
//   （想更细就取整到最近的整数，256 档分辨率对调试足够）
//
// 【单位】串口用 rpm（人看得懂），本模块内部转成"计数/测速窗口"：
//   本车标定 1 rpm = 10 计数/10ms，故内部目标 = rpm × 10（移位相加实现，零乘法器）
//   ⚠ 若换编码器/减速比/窗口长度，这个 10 倍关系要跟着改（参数 TGT_SCALE_LG）
//
// 【设计要点】
//   * 每个输出都是"单周期 valid + 数值保持"，顶层用 valid 锁存，不会重复触发
//   * 数字累积带饱和，乱输字符不会回绕成小值（错值比不响应更难查）
//   * 无法识别的字节只丢一个字符并置 o_cmd_error，不会卡死解析器
//
// 端口见下方声明；o_cmd_error 为单周期脉冲，可直接接 LED 便于发现"命令没打对"
//////////////////////////////////////////////////////////////////////////////////

module uart_cmd_rx #(
    parameter integer PWM_BITS    = 11,
    parameter integer DUTY_MAX    = (1 << PWM_BITS) - 1,
    parameter integer TGT_SCALE_LG = 3,      // rpm → 内部单位：×10 = ×8 + ×2
    parameter integer MAG_LIMIT   = 2_000_000
)(
    input  wire        i_clock,
    input  wire        i_reset_n,

    input  wire [7:0]  i_rx_data,
    input  wire        i_rx_valid,

    // ---- 目标转速（单位：计数/窗口）----
    output reg         o_tgt_valid,
    output reg  [3:0]  o_tgt_sel,          // 每位对应一路
    output reg  signed [31:0] o_tgt_value,

    // ---- PID 参数（Q8）----
    output reg         o_kp_valid,
    output reg  signed [15:0] o_kp_value,
    output reg         o_ki_valid,
    output reg  signed [15:0] o_ki_value,
    output reg         o_kd_valid,
    output reg  signed [15:0] o_kd_value,

    // ---- 模式 ----
    output reg         o_mode_valid,
    output reg         o_closed_loop,      // 1 = 闭环
    output reg         o_duty_valid,
    output reg  [PWM_BITS-1:0] o_duty_value,

    // ---- 立即动作脉冲 ----
    output reg         o_count_clear,
    output reg         o_stall_clear,
    output reg         o_report_req,
    output reg         o_cmd_error
);

    localparam [1:0] S_WAIT = 2'd0,
                     S_NUM  = 2'd1;

    reg [1:0]  state;
    reg [7:0]  cmd;
    reg [31:0] acc;
    reg        neg;
    reg        has_digit;

    //---------------------------------------------------------------
    // 提交时的数值换算（组合）
    //---------------------------------------------------------------
    wire signed [31:0] mag = (acc > MAG_LIMIT[31:0]) ? MAG_LIMIT[31:0] : acc[31:0];
    wire signed [31:0] sval = neg ? -mag : mag;

    // rpm → 计数/窗口：×10 = (×8 + ×2)
    wire signed [31:0] tgt_counts =
        (sval <<< TGT_SCALE_LG) + (sval <<< (TGT_SCALE_LG - 2));

    // 参数限幅到 int16
    wire signed [15:0] kval =
        (sval >  32'sd32767) ?  16'sd32767 :
        (sval < -32'sd32768) ? -16'sd32768 :
                                sval[15:0];

    // 开环占空比限幅
    wire [PWM_BITS-1:0] dval =
        (sval < 0)                  ? {PWM_BITS{1'b0}} :
        (sval > DUTY_MAX)           ? DUTY_MAX[PWM_BITS-1:0] :
                                      sval[PWM_BITS-1:0];

    // 单路目标掩码：'1'..'4' → bit0..bit3
    wire [3:0] sel_one = 4'd1 << (cmd - 8'h31);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state         <= S_WAIT;
            cmd           <= 8'h00;
            acc           <= 32'd0;
            neg           <= 1'b0;
            has_digit     <= 1'b0;
            o_tgt_valid   <= 1'b0;
            o_tgt_sel     <= 4'b0000;
            o_tgt_value   <= 32'sd0;
            o_kp_valid    <= 1'b0;
            o_kp_value    <= 16'sd0;
            o_ki_valid    <= 1'b0;
            o_ki_value    <= 16'sd0;
            o_kd_valid    <= 1'b0;
            o_kd_value    <= 16'sd0;
            o_mode_valid  <= 1'b0;
            o_closed_loop <= 1'b0;
            o_duty_valid  <= 1'b0;
            o_duty_value  <= {PWM_BITS{1'b0}};
            o_count_clear <= 1'b0;
            o_stall_clear <= 1'b0;
            o_report_req  <= 1'b0;
            o_cmd_error   <= 1'b0;
        end else begin
            // 默认：所有脉冲回到 0（数值保持，供顶层锁存）
            o_tgt_valid   <= 1'b0;
            o_kp_valid    <= 1'b0;
            o_ki_valid    <= 1'b0;
            o_kd_valid    <= 1'b0;
            o_mode_valid  <= 1'b0;
            o_duty_valid  <= 1'b0;
            o_count_clear <= 1'b0;
            o_stall_clear <= 1'b0;
            o_report_req  <= 1'b0;
            o_cmd_error   <= 1'b0;

            if (i_rx_valid) begin
                case (state)
                    //------------------------------
                    // 等首字符
                    //------------------------------
                    S_WAIT: begin
                        case (i_rx_data)
                            8'h0D, 8'h0A, 8'h20: ;               // 空行/空格：忽略
                            8'h5A, 8'h7A: o_count_clear <= 1'b1; // 'Z' / 'z'
                            8'h58, 8'h78: o_stall_clear <= 1'b1; // 'X' / 'x'
                            8'h52, 8'h72: o_report_req  <= 1'b1; // 'R' / 'r'
                            default: begin
                                cmd       <= i_rx_data;
                                acc       <= 32'd0;
                                neg       <= 1'b0;
                                has_digit <= 1'b0;
                                state     <= S_NUM;
                            end
                        endcase
                    end
                    //------------------------------
                    // 收集数字
                    //------------------------------
                    S_NUM: begin
                        if (i_rx_data >= 8'h30 && i_rx_data <= 8'h39) begin
                            // 数字：累积（带饱和，防回绕）
                            if (acc <= MAG_LIMIT[31:0])
                                acc <= acc * 32'd10 + {28'd0, i_rx_data[3:0]};
                            has_digit <= 1'b1;
                        end else if (i_rx_data == 8'h2D && !has_digit) begin
                            neg <= 1'b1;                          // 负号
                        end else if (i_rx_data == 8'h0D || i_rx_data == 8'h0A ||
                                     i_rx_data == 8'h20) begin
                            //--------------- 提交 ----------------
                            if (has_digit) begin
                                case (cmd)
                                    8'h54, 8'h74: begin       // 'T' 全部目标
                                        o_tgt_valid <= 1'b1;
                                        o_tgt_sel   <= 4'b1111;
                                        o_tgt_value <= tgt_counts;
                                    end
                                    8'h31, 8'h32, 8'h33, 8'h34: begin  // '1'..'4' 单路目标
                                        o_tgt_valid <= 1'b1;
                                        o_tgt_sel   <= sel_one;
                                        o_tgt_value <= tgt_counts;
                                    end
                                    8'h50, 8'h70: begin       // 'P' Kp
                                        o_kp_valid <= 1'b1;
                                        o_kp_value <= kval;
                                    end
                                    8'h49, 8'h69: begin       // 'I' Ki
                                        o_ki_valid <= 1'b1;
                                        o_ki_value <= kval;
                                    end
                                    8'h44, 8'h64: begin       // 'D' Kd
                                        o_kd_valid <= 1'b1;
                                        o_kd_value <= kval;
                                    end
                                    8'h45, 8'h65: begin       // 'E' 闭环使能
                                        o_mode_valid  <= 1'b1;
                                        o_closed_loop <= (sval != 32'sd0);
                                    end
                                    8'h55, 8'h75: begin       // 'U' 开环占空比
                                        o_duty_valid <= 1'b1;
                                        o_duty_value <= dval;
                                    end
                                    default: o_cmd_error <= 1'b1;
                                endcase
                            end else begin
                                o_cmd_error <= 1'b1;              // 只有命令字母没有数字
                            end
                            state <= S_WAIT;
                        end else begin
                            o_cmd_error <= 1'b1;                  // 非法字符：丢弃该命令
                            state       <= S_WAIT;
                        end
                    end
                endcase
            end
        end
    end

endmodule
