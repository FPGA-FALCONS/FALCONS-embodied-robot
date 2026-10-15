//////////////////////////////////////////////////////////////////////////////////
// 模块：uart_byte_tx —— 可配置的单字节UART发送器
// 帧：1位起始位 + DATA_BITS位数据（低位先发）+ 可选校验位 + 1位停止位
//
// 参数：
//   CLK_FREQ_HZ  系统时钟频率(Hz)       默认50_000_000
//   BAUD_RATE    UART波特率             默认9_600
//   DATA_BITS    有效数据位数           5～8，默认8
//   PARITY_EN    校验使能               0=无校验，1=有校验
//   PARITY_TYPE  校验类型               0=偶校验，1=奇校验
//
// 端口：
//   i_clock       输入  系统时钟
//   i_reset_n     输入  异步低电平复位
//   i_tx_data     输入  待发送数据；DATA_BITS<8时忽略高位
//   i_tx_valid    输入  数据有效，未完成握手前应保持为1
//   o_tx_ready    输出  可接收数据；valid与ready同时为1即完成交接
//   o_uart_tx     输出  UART串行发送线，空闲时为高电平
//
// 最小示例（115200 8N1）：
//   uart_byte_tx #(.BAUD_RATE(115_200)) u_tx (
//       .i_clock(clk), .i_reset_n(reset_n),
//       .i_tx_data(send_data), .i_tx_valid(send_valid),
//       .o_tx_ready(send_ready), .o_uart_tx(uart_tx)
//   );
//   // send_request到来时提交数据；握手前send_valid保持为1
//   always @(posedge clk or negedge reset_n) begin
//       if (!reset_n)
//           send_valid <= 1'b0;
//       else begin
//           if (send_valid && send_ready)
//               send_valid <= 1'b0;              // 本次握手完成
//           if (send_request && (!send_valid || send_ready)) begin
//               send_data  <= 8'h41;             // 准备发送字符'A'
//               send_valid <= 1'b1;
//           end
//       end
//   end
//////////////////////////////////////////////////////////////////////////////////
module uart_byte_tx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 9_600,
    parameter integer DATA_BITS   = 8,
    parameter integer PARITY_EN   = 0,
    parameter integer PARITY_TYPE = 0
)(
    input  wire       i_clock,
    input  wire       i_reset_n,
    input  wire [7:0] i_tx_data,
    input  wire       i_tx_valid,
    output wire       o_tx_ready,
    output reg        o_uart_tx
);

    // 四舍五入计算每个UART数据位对应的系统时钟数，减小波特率误差
    localparam integer CLKS_PER_BIT =
        (CLK_FREQ_HZ + BAUD_RATE / 2) / BAUD_RATE;

    localparam [2:0] IDLE   = 3'd0;
    localparam [2:0] START  = 3'd1;
    localparam [2:0] DATA   = 3'd2;
    localparam [2:0] PARITY = 3'd3;
    localparam [2:0] STOP   = 3'd4;

    reg [2:0]  state;
    reg [31:0] baud_cnt;
    reg [2:0]  bit_index;
    reg [7:0]  data_reg;
    reg        parity_bit;

    assign o_tx_ready = (state == IDLE);

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            state      <= IDLE;
            baud_cnt   <= 32'd0;
            bit_index  <= 3'd0;
            data_reg   <= 8'd0;
            parity_bit <= 1'b0;
            o_uart_tx   <= 1'b1;
        end else begin
            case (state)
                IDLE: begin
                    o_uart_tx <= 1'b1;
                    // 当前处于空闲状态（ready=1），valid=1时接收并保存数据
                    if (i_tx_valid) begin
                        data_reg  <= i_tx_data;
                        bit_index <= 3'd0;
                        baud_cnt  <= CLKS_PER_BIT - 1;
                        o_uart_tx <= 1'b0;
                        state     <= START;

                        // 有效数据位的归约异或结果用于偶校验，取反后用于奇校验
                        if (PARITY_TYPE == 0)
                            parity_bit <= ^i_tx_data[DATA_BITS-1:0];
                        else
                            parity_bit <= ~^i_tx_data[DATA_BITS-1:0];
                    end
                end

                START: begin
                    if (baud_cnt == 0) begin
                        baud_cnt <= CLKS_PER_BIT - 1;
                        bit_index <= 3'd0;
                        o_uart_tx <= data_reg[0];
                        state <= DATA;
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                DATA: begin
                    if (baud_cnt == 0) begin
                        baud_cnt <= CLKS_PER_BIT - 1;
                        if (bit_index == DATA_BITS - 1) begin
                            if (PARITY_EN != 0) begin
                                o_uart_tx <= parity_bit;
                                state <= PARITY;
                            end else begin
                                o_uart_tx <= 1'b1;
                                state <= STOP;
                            end
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            o_uart_tx <= data_reg[bit_index + 1'b1];
                        end
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                PARITY: begin
                    if (baud_cnt == 0) begin
                        baud_cnt <= CLKS_PER_BIT - 1;
                        o_uart_tx <= 1'b1;
                        state <= STOP;
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                STOP: begin
                    if (baud_cnt == 0) begin
                        o_uart_tx <= 1'b1;
                        state <= IDLE;
                    end else
                        baud_cnt <= baud_cnt - 1'b1;
                end

                default: begin
                    o_uart_tx <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
