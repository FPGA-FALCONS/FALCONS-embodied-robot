/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : i2c_control
// Description   : I2C控制器顶层文件
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module i2c_control #(
  // 输入时钟和目标 SCL 频率，单位均为 Hz
  parameter SYS_CLOCK = 50_000_000,
  parameter SCL_CLOCK = 400_000
)
(
  input            Clk,          // 系统时钟
  input            Rst_n,        // 异步低有效复位

  input            wrreg_req,    // 写寄存器请求，在空闲状态采样
  input            rdreg_req,    // 读寄存器请求，在空闲状态采样
  input     [15:0] addr,         // 目标寄存器地址
  input            addr_mode,    // 1：16 位地址；0：使用 addr[7:0]
  input     [7:0]  wrdata,       // 待写入的寄存器数据
  output reg[7:0]  rddata,       // 最近一次读事务返回的数据
  input     [7:0]  device_id,    // 7 位从机地址左移一位，bit 0 必须为 0
  output reg       RW_Done,      // 事务及 dly_cnt_max 延迟完成，单周期脉冲

  output reg       ack,          // 0：全部收到 ACK；1：至少一次 NACK

  input     [31:0] dly_cnt_max,  // 事务后延迟，单位为 Clk 周期
  output           i2c_sclk,     // SCCB/I2C 串行时钟
  inout            i2c_sdat      // 开漏串行数据，需要外部上拉
);

  reg [5:0]Cmd;
  reg [7:0]Tx_DATA;
  wire [7:0]Rx_DATA;
  wire Trans_Done;
  wire ack_o;
  reg Go;
  wire [15:0] reg_addr;

  assign reg_addr = addr_mode?addr:{addr[7:0],addr[15:8]};

  localparam 
    WR   = 6'b000001,   //写请求
    STA  = 6'b000010,   //起始位请求
    RD   = 6'b000100,   //读请求
    STO  = 6'b001000,   //停止位请求
    ACK  = 6'b010000,   //应答位请求
    NACK = 6'b100000;   //无应答请求

  i2c_bit_shift #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
  )i2c_bit_shift(
    .Clk       (Clk        ),
    .Rst_n     (Rst_n      ),
    .Cmd       (Cmd        ),
    .Go        (Go         ),
    .Rx_DATA   (Rx_DATA    ),
    .Tx_DATA   (Tx_DATA    ),
    .Trans_Done(Trans_Done ),
    .ack_o     (ack_o      ),
    .i2c_sclk  (i2c_sclk   ),
    .i2c_sdat  (i2c_sdat   )
  );

  reg [7:0]state;
  reg [7:0]cnt;
  reg [31:0]dly_cnt;

  localparam
    IDLE         = 8'b0000_0001,   //空闲状态
    WR_REG       = 8'b0000_0010,   //写寄存器状态
    WAIT_WR_DONE = 8'b0000_0100,   //等待写寄存器完成状态
    WR_REG_DONE  = 8'b0000_1000,   //写寄存器完成状态
    RD_REG       = 8'b0001_0000,   //读寄存器状态
    WAIT_RD_DONE = 8'b0010_0000,   //等待读寄存器完成状态
    RD_REG_DONE  = 8'b0100_0000,   //读寄存器完成状态
    WAIT_DLY     = 8'b1000_0000;   //等带读写完成后延迟完成
  always@(posedge Clk or negedge Rst_n)
  if(!Rst_n)begin
    Cmd <= 6'd0;
    Tx_DATA <= 8'd0;
    Go <= 1'b0;
    rddata <= 0;
    state <= IDLE;
    ack <= 0;
    dly_cnt <= 0;
    cnt <= 0;
    RW_Done <= 1'b0;
  end
  else begin
    case(state)
      IDLE:
        begin
          cnt <= 0;
          dly_cnt <= 0;
          ack <= 0;
          RW_Done <= 1'b0;
          if(wrreg_req)
            state <= WR_REG;
          else if(rdreg_req)
            state <= RD_REG;
          else
            state <= IDLE;
        end

      WR_REG:
        begin
          state <= WAIT_WR_DONE;
          case(cnt)
            0://write_byte(WR | STA, device_id);
            begin Cmd <= WR | STA; Tx_DATA <= device_id; Go <= 1'b1; end

            1://write_byte(WR, reg_addr[15:8]);
            begin Cmd <= WR; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end

            2://write_byte(WR, reg_addr[7:0]);
            begin Cmd <= WR; Tx_DATA <= reg_addr[7:0]; Go <= 1'b1; end

            3://write_byte(WR | STO, wrdata);
            begin Cmd <= WR | STO; Tx_DATA <= wrdata; Go <= 1'b1; end

            default:;
          endcase
        end

      WAIT_WR_DONE:
        begin
          Go <= 1'b0; 
          if(Trans_Done)begin
            ack <= ack | ack_o;
            case(cnt)
              0: begin cnt <= 1; state <= WR_REG;end
              1: 
                begin 
                  state <= WR_REG;
                  if(addr_mode)
                    cnt <= 2; 
                  else
                    cnt <= 3;
                end

              2: begin
                  cnt <= 3;
                  state <= WR_REG;
                end
              3:state <= WR_REG_DONE;
              default:state <= IDLE;
            endcase
          end
        end

      WR_REG_DONE:
        begin
//          RW_Done <= 1'b1;
          state <= WAIT_DLY;
        end

      RD_REG:
        begin
          state <= WAIT_RD_DONE;
          case(cnt)
            0://write_byte(WR | STA, device_id);
            begin Cmd <= WR | STA; Tx_DATA <= device_id; Go <= 1'b1; end

            1: begin
              if(addr_mode)
                //write_byte(WR, reg_addr[15:8]);
                begin Cmd <= WR; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end
              else 
                //write_byte(WR | STO, reg_addr[15:8]);
                begin Cmd <= WR | STO; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end
            end

            2://write_byte(WR | STO, reg_addr[7:0]);
            begin Cmd <= WR | STO; Tx_DATA <= reg_addr[7:0]; Go <= 1'b1; end

            3://write_byte(WR | STA, device_id | 8'd1);
            begin Cmd <= WR | STA; Tx_DATA <= device_id | 8'd1; Go <= 1'b1; end

            4://read_byte(RD | NACK | STO);
            begin Cmd <= RD | NACK | STO; Go <= 1'b1; end

            default:;
          endcase
        end

      WAIT_RD_DONE:
        begin
          Go <= 1'b0; 
          if(Trans_Done)begin
            if(cnt <= 3)
              ack <= ack | ack_o;
            case(cnt)
              0: begin cnt <= 1; state <= RD_REG;end
              1: 
                begin 
                  state <= RD_REG;
                  if(addr_mode)
                    cnt <= 2; 
                  else
                    cnt <= 3;
                end
              2: begin
                  cnt <= 3;
                  state <= RD_REG;
                end
              3:begin
                  cnt <= 4;
                  state <= RD_REG;
                end
              4:state <= RD_REG_DONE;
              default:state <= IDLE;
            endcase
          end
        end

      RD_REG_DONE:
        begin
//          RW_Done <= 1'b1;
          rddata <= Rx_DATA;
          state <= WAIT_DLY;
        end
      default:state <= IDLE;
      WAIT_DLY:
        begin
          if(dly_cnt <= dly_cnt_max) begin
            dly_cnt <= dly_cnt + 1'b1;
            state <= WAIT_DLY;
          end
         else begin
           dly_cnt <= 0;
           RW_Done <= 1'b1;
           state <= IDLE;
         end
       end
    endcase
  end

//考虑部分国产编译器暂不支持task，去掉该种写法
//  task read_byte;
//    input [5:0]Ctrl_Cmd;
//    begin
//      Cmd <= Ctrl_Cmd;
//      Go <= 1'b1; 
//    end
//  endtask
//
//  task write_byte;
//    input [5:0]Ctrl_Cmd;
//    input [7:0]Wr_Byte_Data;
//    begin
//      Cmd <= Ctrl_Cmd;
//      Tx_DATA <= Wr_Byte_Data;
//      Go <= 1'b1; 
//    end
//  endtask

endmodule
