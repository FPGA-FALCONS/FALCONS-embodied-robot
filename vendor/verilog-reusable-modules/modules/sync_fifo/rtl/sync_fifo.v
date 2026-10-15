//////////////////////////////////////////////////////////////////////////////////
// 模块：sync_fifo —— 单时钟同步先进先出缓冲区
// 写入条件：i_write_valid && o_write_ready
// 读出条件：o_read_valid && i_read_ready
//
// 参数：
//   DATA_WIDTH  每个数据的位宽
//   ADDR_WIDTH  地址位宽；FIFO深度=2^ADDR_WIDTH，例如4表示16项
//
// 端口：
//   i_clock          输入  读写共用时钟
//   i_reset_n        输入  异步低电平复位
//   i_write_data     输入  准备写入的数据
//   i_write_valid    输入  写入数据有效
//   o_write_ready    输出  FIFO还有空间，可以写入
//   o_read_data      输出  当前队首数据
//   o_read_valid     输出  队首数据有效，FIFO非空
//   i_read_ready     输入  下游准备取走队首数据
//
// 最小示例（16字节FIFO）：
//   sync_fifo #(.DATA_WIDTH(8), .ADDR_WIDTH(4)) u_fifo (
//       .i_clock(clk), .i_reset_n(reset_n),
//       .i_write_data(wr_data), .i_write_valid(wr_valid),
//       .o_write_ready(wr_ready),
//       .o_read_data(rd_data), .o_read_valid(rd_valid),
//       .i_read_ready(rd_ready)
//   );
//   // 写入一次：wr_valid必须保持到wr_valid && wr_ready成立
//   always @(posedge clk or negedge reset_n) begin
//       if (!reset_n)
//           wr_valid <= 1'b0;
//       else if (wr_valid && wr_ready)
//           wr_valid <= 1'b0;                    // 写握手完成
//       else if (write_request) begin
//           wr_data  <= new_data;
//           wr_valid <= 1'b1;
//       end
//   end
//   // 读取一次：需要数据且FIFO非空时，产生一次读握手
//   always @(posedge clk or negedge reset_n) begin
//       if (!reset_n)
//           rd_ready <= 1'b0;
//       else begin
//           rd_ready <= read_request;
//           if (rd_valid && rd_ready)
//               saved_data <= rd_data;           // 读握手完成
//       end
//   end
//////////////////////////////////////////////////////////////////////////////////
module sync_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 4
)(
    input  wire                  i_clock,
    input  wire                  i_reset_n,

    input  wire [DATA_WIDTH-1:0] i_write_data,
    input  wire                  i_write_valid,
    output wire                  o_write_ready,

    output wire [DATA_WIDTH-1:0] o_read_data,
    output wire                  o_read_valid,
    input  wire                  i_read_ready
);

    localparam integer DEPTH = (1 << ADDR_WIDTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] write_ptr;
    reg [ADDR_WIDTH-1:0] read_ptr;
    reg [ADDR_WIDTH:0]   count;

    wire fifo_empty = (count == 0);
    wire fifo_full  = (count == DEPTH);
    wire do_read    = o_read_valid && i_read_ready;
    wire do_write   = i_write_valid && o_write_ready;

    // FIFO满时，如果同一个时钟周期也会读出一个数据，仍允许同时写入
    assign o_write_ready = !fifo_full || do_read;
    assign o_read_valid = !fifo_empty;
    assign o_read_data = mem[read_ptr];

    always @(posedge i_clock or negedge i_reset_n) begin
        if (!i_reset_n) begin
            write_ptr <= {ADDR_WIDTH{1'b0}};
            read_ptr  <= {ADDR_WIDTH{1'b0}};
            count     <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            if (do_write) begin
                mem[write_ptr] <= i_write_data;
                write_ptr <= write_ptr + 1'b1;
            end

            if (do_read)
                read_ptr <= read_ptr + 1'b1;

            case ({do_write, do_read})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
