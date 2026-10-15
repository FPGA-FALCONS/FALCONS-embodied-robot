`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// bmi270_model —— BMI270 SPI 从机行为模型（仅用于仿真）
//
// 模拟内容：
//   * Mode0 SPI 从机：上升沿采样 MOSI，下降沿更新 MISO
//   * 地址字节 bit7=1 表示读（BMI270 约定），bit6:0 为寄存器地址
//   * 读事务：地址 + dummy + N 个数据字节（寄存器地址自动递增）
//   * 写事务：地址（bit7=0）+ N 个数据字节（自动递增）
//   * 寄存器文件 256 字节，可被 tb 通过层次引用直接读写/预置
//   * 特殊行为：
//       - 0x00 CHIP_ID 预置为 0x24
//       - 0x59 INIT_CTRL 写入 0x01 时，把 0x21 INTERNAL_STATUS 置为 0x01
//       - 记录以 0x5E(INIT_DATA) 为起始地址的 burst 写入字节数（burst_cnt）
//
// tb 可用的观测信号（层次引用）：
//   u_bmi270.regfile[addr]   寄存器内容
//   u_bmi270.burst_cnt        0x5E 起始的 burst 写入字节计数
//
// 端口：
//   sck / mosi / cs_n   输入   SPI 信号（由主机驱动）
//   miso                输出   从机数据输出
//////////////////////////////////////////////////////////////////////////////////

module bmi270_model (
    input  wire sck,
    input  wire mosi,
    output wire miso,
    input  wire cs_n
);

    reg [7:0] regfile  [0:255];
    reg [7:0] cfg_ram  [0:8191]; // INIT_DATA(0x5E) 的写入目标：模拟芯片内部 8KB 配置 RAM
                                 // （不能写进 256 字节寄存器空间，否则会覆盖数据寄存器）
    reg [7:0] rx_shift;          // MOSI 移位寄存器
    reg [7:0] tx_shift;          // MISO 移位寄存器
    reg [2:0] bit_cnt;
    reg [13:0] byte_cnt;         // 本次 CS 周期内已完成的字节数（需支持 8KB burst）
    reg [7:0] cur_addr;          // 本事务的起始寄存器地址
    reg       is_read;           // 1 = 读事务

    // 0x5E 起始的 burst 写入字节计数（每完成一个字节累加）
    reg [15:0] burst_cnt;

    integer k;

    assign miso = tx_shift[7];

    //------------------------------------------------------------------
    // 寄存器初值
    //------------------------------------------------------------------
    initial begin
        for (k = 0; k < 256; k = k + 1) regfile[k] = 8'h00;

        regfile[8'h00] = 8'h24;                     // CHIP_ID

        // 加速度：x=0, y=0, z=0x1000(4096) → 期望换算为 1000 mg
        regfile[8'h0C] = 8'h00; regfile[8'h0D] = 8'h00;
        regfile[8'h0E] = 8'h00; regfile[8'h0F] = 8'h00;
        regfile[8'h10] = 8'h00; regfile[8'h11] = 8'h10;

        // 陀螺：x = 0x0050(80) → 80×25000/4096 ≈ 488 (0.01°/s) = 4.88°/s
        //   用这个值模拟"真实的零偏"，而不是大角速度：
        //   零偏是静止时也该有的固有偏移，标定就是要把它测出来
        regfile[8'h12] = 8'h50; regfile[8'h13] = 8'h00;
        regfile[8'h14] = 8'h00; regfile[8'h15] = 8'h00;
        regfile[8'h16] = 8'h00; regfile[8'h17] = 8'h00;

        rx_shift  = 8'h00;
        tx_shift  = 8'h00;
        bit_cnt   = 3'd0;
        byte_cnt  = 14'd0;
        cur_addr  = 8'h00;
        is_read   = 1'b0;
        burst_cnt = 16'd0;
    end

    //------------------------------------------------------------------
    // CS 下降沿：开始新事务
    //------------------------------------------------------------------
    always @(negedge cs_n) begin
        bit_cnt  <= 3'd0;
        byte_cnt <= 14'd0;
        tx_shift <= 8'h00;
    end

    //------------------------------------------------------------------
    // SCK 上升沿：采样 MOSI
    //------------------------------------------------------------------
    always @(posedge sck) begin
        if (!cs_n) begin
            rx_shift <= {rx_shift[6:0], mosi};
        end
    end

    //------------------------------------------------------------------
    // SCK 下降沿：位计数、字节处理、MISO 更新
    //   每字节 8 个上升沿后被识别；字节边界在 bit_cnt 回绕为 0 的下降沿
    //------------------------------------------------------------------
    always @(negedge sck) begin
        if (!cs_n) begin
            if (bit_cnt == 3'd7) begin
                bit_cnt <= 3'd0;

                if (byte_cnt == 14'd0) begin
                    // ---- 地址字节完成 ----
                    is_read  <= rx_shift[7];
                    cur_addr <= rx_shift[6:0];
                    tx_shift <= 8'h00;              // 地址字节期间 MISO 无意义
                end else if (is_read) begin
                    // ---- 读事务：byte_cnt=1 是 dummy，其后输出数据 ----
                    // 本拍装载的是"下一个字节"要输出的内容
                    tx_shift <= regfile[cur_addr + byte_cnt[7:0] - 8'd1];
                end else if (cur_addr == 8'h5E) begin
                    // ---- INIT_DATA burst 写：目标是内部 8KB 配置 RAM ----
                    // 关键：不能写进 regfile，否则 8KB 数据回绕会覆盖数据寄存器
                    cfg_ram[byte_cnt[12:0] - 13'd1] <= rx_shift;
                    burst_cnt <= burst_cnt + 16'd1;
                end else begin
                    // ---- 普通寄存器写 ----
                    regfile[cur_addr + byte_cnt[7:0] - 8'd1] <= rx_shift;

                    // INIT_CTRL(0x59) 写 0x01 → 配置完成，INTERNAL_STATUS 置 1
                    if ((cur_addr == 8'h59) && (rx_shift == 8'h01)) begin
                        regfile[8'h21] <= 8'h01;
                    end
                end

                byte_cnt <= byte_cnt + 14'd1;
            end else begin
                bit_cnt  <= bit_cnt + 3'd1;
                tx_shift <= {tx_shift[6:0], 1'b0};   // MISO 移位
            end
        end
    end

endmodule
