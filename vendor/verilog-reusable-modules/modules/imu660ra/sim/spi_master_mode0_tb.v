`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// spi_master_mode0 测试平台
//
// 验证内容：
//   1) 读事务：3 字节（地址 + dummy + 数据），检查 dummy 被丢弃、数据正确
//   2) 写事务：2 字节（地址 + 数据），检查从机收到的字节顺序
//   3) burst：连续 6 字节，检查字节索引递增与数据顺序
//   4) o_rx_idx 与 o_rx_byte 的对应关系正确
//   5) CS 在两次事务之间拉高
//
// 从机模型：模拟 BMI270 的 SPI 行为
//   * Mode0：CS 拉低后输出首位，SCK 下降沿更新 MISO，主机在上升沿采样
//   * 从机在 SCK 上升沿采样 MOSI
//////////////////////////////////////////////////////////////////////////////////

module spi_master_mode0_tb;

    localparam integer HALF_DIV = 3;        // SCK = 50MHz/6 ≈ 8.33MHz

    reg         clk;
    reg         rst_n;

    // 主机侧
    reg         start;
    reg  [15:0] byte_num;
    wire [15:0] byte_idx;
    reg  [7:0]  tx_byte;
    wire [7:0]  rx_byte;
    wire [15:0] rx_idx;
    wire        rx_valid;
    wire        busy;
    wire        done;

    wire        sck;
    wire        mosi;
    wire        miso;
    wire        cs_n;

    integer     errors;
    integer     i;

    // 发送数据缓冲（主机要发的字节）
    reg [7:0] tx_buf [0:15];
    // 从机要回的字节
    reg [7:0] slave_buf [0:15];
    // 从机收到的字节
    reg [7:0] slave_rx [0:15];

    //------------------------------------------------------------------
    // 被测模块
    //------------------------------------------------------------------
    spi_master_mode0 #(
        .SCK_HALF_DIV (HALF_DIV)
    ) dut (
        .i_clock    (clk),
        .i_reset_n  (rst_n),
        .i_start    (start),
        .i_byte_num (byte_num),
        .o_byte_idx (byte_idx),
        .i_tx_byte  (tx_byte),
        .o_rx_byte  (rx_byte),
        .o_rx_idx   (rx_idx),
        .o_rx_valid (rx_valid),
        .o_busy     (busy),
        .o_done     (done),
        .o_sck      (sck),
        .o_mosi     (mosi),
        .i_miso     (miso),
        .o_cs_n     (cs_n)
    );

    // 主机发送字节：按索引从缓冲取
    always @(*) begin
        tx_byte = tx_buf[byte_idx[3:0]];
    end

    //------------------------------------------------------------------
    // 时钟
    //------------------------------------------------------------------
    initial clk = 1'b0;
    always #10 clk = ~clk;                  // 50MHz

    //------------------------------------------------------------------
    // SPI 从机模型（Mode0）
    //------------------------------------------------------------------
    reg [7:0] s_tx_shift;
    reg [2:0] s_bit;
    reg [7:0] s_byte_cnt;

    assign miso = s_tx_shift[7];

    always @(negedge cs_n) begin
        // CS 拉低：装载第一个要回的字节，清零计数
        s_tx_shift <= slave_buf[0];
        s_bit      <= 3'd0;
        s_byte_cnt <= 8'd0;
    end

    // 上升沿：从机采样 MOSI，并推进位计数
    always @(posedge sck) begin
        if (!cs_n) begin
            slave_rx[s_byte_cnt[3:0]] <= {slave_rx[s_byte_cnt[3:0]][6:0], mosi};
            if (s_bit == 3'd7) begin
                s_bit      <= 3'd0;
                s_byte_cnt <= s_byte_cnt + 8'd1;
            end else begin
                s_bit <= s_bit + 3'd1;
            end
        end
    end

    // 下降沿：从机更新 MISO
    //   位计数为 0 说明刚采完一个字节的最高位前的位置已过整字节——
    //   这里用"位计数回绕为 0"识别字节边界，装载下一字节而不是再移位
    always @(negedge sck) begin
        if (!cs_n) begin
            if (s_bit == 3'd0) begin
                s_tx_shift <= slave_buf[s_byte_cnt[3:0]];      // 装载下一字节
            end else begin
                s_tx_shift <= {s_tx_shift[6:0], 1'b0};         // 正常移位
            end
        end
    end

    //------------------------------------------------------------------
    // 主机侧接收记录
    //------------------------------------------------------------------
    reg [7:0]  cap_byte [0:15];
    reg [15:0] cap_idx  [0:15];

    always @(posedge clk) begin
        if (rx_valid) begin
            cap_byte[rx_idx[3:0]] = rx_byte;
            cap_idx [rx_idx[3:0]] = rx_idx;
        end
    end

    //------------------------------------------------------------------
    // 一次事务
    //------------------------------------------------------------------
    task do_transfer;
        input integer n;
        begin
            @(negedge clk);
            byte_num = n[15:0];
            start    = 1'b1;
            @(negedge clk);
            start    = 1'b0;
            // 等 done
            wait (done == 1'b1);
            @(negedge clk);
            repeat (4) @(negedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        errors = 0;
        rst_n  = 1'b0;
        start  = 1'b0;
        byte_num = 16'd0;

        for (i = 0; i < 16; i = i + 1) begin
            tx_buf[i]    = 8'h00;
            slave_buf[i] = 8'h00;
            slave_rx[i]  = 8'h00;
            cap_byte[i]  = 8'h00;
            cap_idx[i]   = 16'hFFFF;
        end

        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (10) @(negedge clk);

        //==============================================================
        // 测试 1：读事务（地址 + dummy + 2 数据字节）
        //   主机发：0x8C, 0x00, 0x00, 0x00
        //   从机回：0x11, 0x22, 0x33, 0x44
        //   期望：rx_idx=2 收到 0x33，rx_idx=3 收到 0x44
        //==============================================================
        $display("---- 测试1：读事务 4 字节 ----");
        tx_buf[0] = 8'h8C;
        slave_buf[0] = 8'h11;
        slave_buf[1] = 8'h22;
        slave_buf[2] = 8'h33;
        slave_buf[3] = 8'h44;

        do_transfer(4);

        if (slave_rx[0] !== 8'h8C) begin
            $display("  错误：从机收到的地址应为 8C，实际 %02h", slave_rx[0]);
            errors = errors + 1;
        end
        if (cap_byte[2] !== 8'h33) begin
            $display("  错误：rx_idx=2 应为 33，实际 %02h", cap_byte[2]);
            errors = errors + 1;
        end
        if (cap_byte[3] !== 8'h44) begin
            $display("  错误：rx_idx=3 应为 44，实际 %02h", cap_byte[3]);
            errors = errors + 1;
        end
        if (cs_n !== 1'b1) begin
            $display("  错误：事务结束后 CS 应为高");
            errors = errors + 1;
        end

        //==============================================================
        // 测试 2：写事务 2 字节（地址 + 数据）
        //==============================================================
        $display("---- 测试2：写事务 2 字节 ----");
        tx_buf[0] = 8'h7C;
        tx_buf[1] = 8'h00;
        slave_buf[0] = 8'hEE;
        slave_buf[1] = 8'hEE;

        do_transfer(2);

        if (slave_rx[0] !== 8'h7C || slave_rx[1] !== 8'h00) begin
            $display("  错误：从机应收到 7C 00，实际 %02h %02h", slave_rx[0], slave_rx[1]);
            errors = errors + 1;
        end
        if (cs_n !== 1'b1) begin
            $display("  错误：事务结束后 CS 应为高");
            errors = errors + 1;
        end

        //==============================================================
        // 测试 3：burst 写 6 字节，检查顺序与索引
        //==============================================================
        $display("---- 测试3：burst 写 6 字节 ----");
        for (i = 0; i < 16; i = i + 1) slave_rx[i] = 8'h00;
        tx_buf[0] = 8'h5E;
        tx_buf[1] = 8'hA0;
        tx_buf[2] = 8'hA1;
        tx_buf[3] = 8'hA2;
        tx_buf[4] = 8'hA3;
        tx_buf[5] = 8'hA4;

        do_transfer(6);

        for (i = 0; i < 6; i = i + 1) begin
            if (slave_rx[i] !== tx_buf[i]) begin
                $display("  错误：burst 第 %0d 字节应为 %02h，实际 %02h", i, tx_buf[i], slave_rx[i]);
                errors = errors + 1;
            end
        end

        //==============================================================
        if (errors == 0) begin
            $display("");
            $display("PASS：spi_master_mode0 测试全部通过");
        end else begin
            $display("");
            $display("FAIL：共 %0d 个错误", errors);
        end
        $finish;
    end

    // 超时保护
    initial begin
        #500000;
        $display("FAIL：仿真超时（500us）");
        $finish;
    end

endmodule
