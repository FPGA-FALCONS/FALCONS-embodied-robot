//////////////////////////////////////////////////////////////////////////////////
// 模块：spi_master_mode0 —— 通用 SPI 主机（模式0：CPOL=0, CPHA=0，MSB first）
//
// 特点：
//   * 字节级接口，支持任意长度连续传输（单字节读写 / burst 读写皆可）
//   * 数据发送不握手：上层根据 o_byte_idx 组合给出当前字节内容
//   * 收发同时进行，每个字节接收完成输出一个 o_rx_valid 脉冲
//   * CS 由本模块控制，一次传输期间保持低电平
//   * 单时钟域，SCK 由寄存器翻转产生（SCK 是输出给外部器件的信号，不是内部时钟）
//
// 时序（模式0）：
//   CS 拉低 → S_LOAD 等 1 拍（让上层数据源，如同步 ROM，稳定）
//   → S_SETUP 锁存本字节并保持 MOSI=bit7 达 SCK_HALF_DIV 拍
//   → 每个 bit 两相位：SCK=0 相位（本期开头更新 MOSI）
//                     SCK=1 相位（本期开头采样 MISO）
//   → 8 个 bit 后字节完成 → 下一字节 / 结束拉高 CS
//
//   关键：MOSI 只在 SCK=0 的相位更新，绝不在 SCK 上升沿附近变化，
//         保证从机在上升沿能采到稳定值。
//
// 参数：
//   SCK_HALF_DIV   SCK 半周期 = SCK_HALF_DIV 个系统时钟
//                  SCK 频率 = CLK_FREQ / (2 * SCK_HALF_DIV)
//                  50MHz 下：3 → 8.33MHz，5 → 5MHz，25 → 1MHz（调试用）
//
// 端口：
//   i_clock       输入  系统时钟
//   i_reset_n     输入  异步低电平复位
//   i_start       输入  单周期脉冲，启动一次传输（仅在 o_busy=0 时有效）
//   i_byte_num    输入  本次传输的总字节数（1~65535），传输期间须保持不变
//   o_byte_idx    输出  当前字节索引 0..i_byte_num-1（组合），上层据此给出 i_tx_byte
//   i_tx_byte     输入  当前字节要发送的数据（组合）
//   o_rx_byte     输出  接收到的字节
//   o_rx_idx      输出  接收字节对应的索引（与 o_rx_valid 同拍有效）
//   o_rx_valid    输出  单周期脉冲：o_rx_byte 有效
//   o_busy        输出  传输进行中
//   o_done        输出  单周期脉冲：一次传输完成
//   o_sck         输出  SPI 时钟
//   o_mosi        输出  主机输出
//   i_miso        输入  主机输入
//   o_cs_n        输出  片选，低有效，空闲为高
//////////////////////////////////////////////////////////////////////////////////

module spi_master_mode0 #(
    parameter integer SCK_HALF_DIV = 3          // SCK 半周期时钟数
)(
    input  wire        i_clock,
    input  wire        i_reset_n,

    // 控制
    input  wire        i_start,
    input  wire [15:0] i_byte_num,
    output wire [15:0] o_byte_idx,
    input  wire [7:0]  i_tx_byte,
    output wire [7:0]  o_rx_byte,
    output wire [15:0] o_rx_idx,
    output wire        o_rx_valid,
    output wire        o_busy,
    output wire        o_done,

    // 物理接口
    output reg         o_sck,
    output reg         o_mosi,
    input  wire        i_miso,
    output reg         o_cs_n
);

    localparam [2:0] S_IDLE  = 3'd0,
                     S_LOAD  = 3'd1,
                     S_SETUP = 3'd2,
                     S_BIT   = 3'd3,
                     S_BEND  = 3'd4,
                     S_DONE  = 3'd5;

    reg [2:0]  state;
    reg [15:0] byte_cnt;        // 当前字节索引
    reg [15:0] byte_num_r;      // 本次传输字节数（锁存）
    reg [3:0]  bit_cnt;         // 本字节已采样 bit 数
    reg        phase;           // 0 = SCK 低相位，1 = SCK 高相位
    reg [15:0] half_cnt;        // 半周期计数器
    reg [7:0]  tx_shift;        // 发送移位寄存器
    reg [7:0]  rx_shift;        // 接收移位寄存器
    reg        setup_load;      // S_SETUP 第一拍需要装载

    reg [7:0]  rx_byte_r;
    reg [15:0] rx_idx_r;
    reg        rx_valid_r;
    reg        done_r;

    assign o_byte_idx = byte_cnt;
    assign o_rx_byte  = rx_byte_r;
    assign o_rx_idx   = rx_idx_r;
    assign o_rx_valid = rx_valid_r;
    assign o_busy     = (state != S_IDLE);
    assign o_done     = done_r;

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state      <= S_IDLE;
            byte_cnt   <= 16'd0;
            byte_num_r <= 16'd0;
            bit_cnt    <= 4'd0;
            phase      <= 1'b0;
            half_cnt   <= 16'd0;
            tx_shift   <= 8'd0;
            rx_shift   <= 8'd0;
            setup_load <= 1'b0;
            rx_byte_r  <= 8'd0;
            rx_idx_r   <= 16'd0;
            rx_valid_r <= 1'b0;
            done_r     <= 1'b0;
            o_sck      <= 1'b0;
            o_mosi     <= 1'b0;
            o_cs_n     <= 1'b1;                 // 空闲时片选为高
        end else begin
            // 默认单周期脉冲
            rx_valid_r <= 1'b0;
            done_r     <= 1'b0;

            case (state)
                //------------------------------------------------------------
                S_IDLE: begin
                    o_sck  <= 1'b0;
                    o_cs_n <= 1'b1;
                    if (i_start) begin
                        byte_num_r <= i_byte_num;
                        byte_cnt   <= 16'd0;
                        o_cs_n     <= 1'b0;             // 片选拉低
                        state      <= S_LOAD;
                    end
                end

                //------------------------------------------------------------
                // 字节预取等待（1 拍）：保证上层数据源（如同步 ROM 输出打拍）
                // 已经给出本字节的正确值
                S_LOAD: begin
                    half_cnt   <= SCK_HALF_DIV - 1;
                    setup_load <= 1'b1;
                    state      <= S_SETUP;
                end

                //------------------------------------------------------------
                // 锁存本字节 + 建立时间（首字节同时完成 CS 建立）
                S_SETUP: begin
                    if (setup_load) begin
                        tx_shift   <= i_tx_byte;        // o_byte_idx 已是当前值
                        o_mosi     <= i_tx_byte[7];     // MSB 先出
                        bit_cnt    <= 4'd0;
                        phase      <= 1'b0;
                        setup_load <= 1'b0;
                    end
                    if (half_cnt == 16'd0) begin
                        half_cnt <= SCK_HALF_DIV - 1;
                        state    <= S_BIT;
                    end else begin
                        half_cnt <= half_cnt - 16'd1;
                    end
                end

                //------------------------------------------------------------
                // 位循环：每个 bit 由 SCK 低相位 + SCK 高相位组成
                S_BIT: begin
                    if (half_cnt == 16'd0) begin
                        half_cnt <= SCK_HALF_DIV - 1;
                        if (phase == 1'b0) begin
                            // 低相位结束 → 上升沿：从机采样 MOSI，主机采样 MISO
                            // （MOSI 在上一相位就已稳定，此处不改变它）
                            o_sck    <= 1'b1;
                            rx_shift <= {rx_shift[6:0], i_miso};
                            bit_cnt  <= bit_cnt + 4'd1;
                            phase    <= 1'b1;
                        end else begin
                            // 高相位结束 → 下降沿：拉低 SCK，并在低电平期间更新 MOSI
                            o_sck  <= 1'b0;
                            phase  <= 1'b0;
                            if (bit_cnt == 4'd8) begin
                                state <= S_BEND;
                            end else begin
                                tx_shift <= {tx_shift[6:0], 1'b0};
                                o_mosi   <= tx_shift[6];
                            end
                        end
                    end else begin
                        half_cnt <= half_cnt - 16'd1;
                    end
                end

                //------------------------------------------------------------
                S_BEND: begin                       // 字节结束
                    rx_byte_r  <= rx_shift;
                    rx_idx_r   <= byte_cnt;         // 锁存旧值：本字节的索引
                    rx_valid_r <= 1'b1;
                    if (byte_cnt >= byte_num_r - 16'd1) begin
                        state <= S_DONE;            // 全部字节完成
                    end else begin
                        byte_cnt <= byte_cnt + 16'd1;
                        state    <= S_LOAD;         // 下一字节（先预取）
                    end
                end

                //------------------------------------------------------------
                S_DONE: begin
                    o_sck  <= 1'b0;
                    o_cs_n <= 1'b1;                 // 拉高片选，结束传输
                    done_r <= 1'b1;
                    state  <= S_IDLE;
                end

                //------------------------------------------------------------
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
