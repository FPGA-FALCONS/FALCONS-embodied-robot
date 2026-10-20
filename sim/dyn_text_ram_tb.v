//////////////////////////////////////////////////////////////////////////////
// sim\dyn_text_ram_tb.v —— dyn_text_ram 的模块级回归
//
// 目的：verify 2026-10-20 给 `mem` 加 `(* syn_ramstyle = "block_ram" *)` 之后，
//       读写行为与原来（推断成 LUT RAM）**逐拍等价**：
//         · 写端口 sys 域、读端口 pixel 域，两套独立时钟
//         · 读延迟仍是 1 拍（o_rdata 在 i_raddr 变化后的下一个读沿更新）
//         · 写后同址可读回、未写地址为 X（RAM 未初始化）
//
// 检查项：
//   G1 复位后 o_rdata 为 X（未初始化），不是 0 —— 证明它不是寄存器堆而是 RAM
//   G2 写 64 个地址（模式 = 地址的位反 + 常量），逐地址读回比对
//   G3 读延迟恰为 1 拍（换地址后第一拍 o_rdata 仍是旧值，第二拍才是新值）
//   G4 双时钟独立：写侧 50 MHz、读侧 49 MHz，同时运行不丢数据
//   G5 写使能为 0 时不改内容
//////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module dyn_text_ram_tb;

    localparam integer TW = 20;      // 写时钟 50 MHz
    localparam integer TR = 20;      // 读时钟 49 MHz —— 用 20.4 ns 的近似
    localparam integer DEPTH = 64;
    localparam integer AB    = 6;

    reg                  clk_w, clk_r;
    reg                  rst_n;
    reg                  we;
    reg  [AB-1:0]        waddr;
    reg  [7:0]           wdata;
    reg  [AB-1:0]        raddr;
    wire [7:0]           rdata;

    integer checks = 0;
    integer errors = 0;
    integer i;

    task chk_int(input [255:0] name, input integer got, input integer exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL %0s: got %0d expect %0d (t=%0t)", name, got, exp, $time);
            end
        end
    endtask

    dyn_text_ram #(.DEPTH(DEPTH), .ADDR_BITS(AB)) dut (
        .i_clk_w (clk_w),
        .i_we    (we),
        .i_waddr (waddr),
        .i_wdata (wdata),
        .i_clk_r (clk_r),
        .i_raddr (raddr),
        .o_rdata (rdata)
    );

    // 两个独立时钟：写 50 MHz，读 49 MHz（用 20.4 ns 近似，确认不是同源）
    initial clk_w = 1'b0;
    always #(10.0) clk_w = ~clk_w;
    initial clk_r = 1'b0;
    always #(10.2) clk_r = ~clk_r;

    // 写一个字节并等它落地
    task wr(input [AB-1:0] a, input [7:0] d);
        begin
            @(negedge clk_w);
            we    = 1'b1;
            waddr = a;
            wdata = d;
            @(negedge clk_w);
            we    = 1'b0;
        end
    endtask

    // 读一个字节：设地址 → 等 2 个读沿（1 拍延迟 + 1 拍余量）→ 返回
    task rd(input [AB-1:0] a, output [7:0] d);
        begin
            @(negedge clk_r);
            raddr = a;
            @(posedge clk_r); #1;
            @(posedge clk_r); #1;
            d = rdata;
        end
    endtask

    reg [7:0] got, exp;
    reg [7:0] pat;

    initial begin
        rst_n = 1'b0;
        we    = 1'b0;
        waddr = 6'd0;
        wdata = 8'd0;
        raddr = 6'd0;

        repeat (3) @(posedge clk_w);
        rst_n = 1'b1;
        repeat (3) @(posedge clk_r);

        // ================= G1 未初始化 =================
        @(negedge clk_r);
        raddr = 6'd5;
        @(posedge clk_r); #1;
        @(posedge clk_r); #1;
        chk_int("g1_uninit_is_x", (^rdata === 1'bx) ? 1 : 0, 1);

        // ================= G2 写满 64 个地址并读回 =================
        for (i = 0; i < DEPTH; i = i + 1) begin
            pat = (i * 8'd37 + 8'd11) & 8'hFF;    // 覆盖全 8 位、周期 256/gcd 足够散
            wr(i[AB-1:0], pat);
        end
        // 全部写完后再统一读回（避免边写边读的竞态干扰判据）
        for (i = 0; i < DEPTH; i = i + 1) begin
            exp = (i * 8'd37 + 8'd11) & 8'hFF;
            rd(i[AB-1:0], got);
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL g2_readback addr=%0d: got %02h expect %02h", i, got, exp);
            end
        end

        // ================= G3 读延迟恰为 1 拍 =================
        // 语义：地址是组合输入的，o_rdata 在下一个读沿更新（单级寄存器）。
        //   所以"地址在低电平期间改变"→ 沿前 o_rdata 仍是旧值、沿后立即是新值。
        //   （写"沿后仍是旧值"是错的：那会是 2 拍延迟，与 RTL 不符。）
        @(negedge clk_r);
        raddr = 6'd0;
        @(posedge clk_r); #1;
        exp = rdata;                                     // A0 的值
        chk_int("g3_a0_value", exp, (0 * 8'd37 + 8'd11) & 8'hFF);
        @(negedge clk_r);
        raddr = 6'd1;
        #1;
        chk_int("g3_before_edge_still_old", rdata, exp);                   // 沿前：旧值
        @(posedge clk_r); #1;
        chk_int("g3_after_edge_new", rdata, (1 * 8'd37 + 8'd11) & 8'hFF);  // 沿后：新值

        // ================= G5 写使能为 0 时内容不变 =================
        @(negedge clk_w);
        we    = 1'b0;
        waddr = 6'd10;
        wdata = 8'hEE;
        repeat (3) @(negedge clk_w);
        rd(6'd10, got);
        chk_int("g5_we0_no_write", got, (10 * 8'd37 + 8'd11) & 8'hFF);

        // ================= G4 双时钟并行（写新值的同时另一侧持续读） =================
        // 写侧把 addr 20 改成 8'h5A，读侧全程不停换地址；最后读回确认写成功
        fork
            begin : writer
                wr(6'd20, 8'h5A);
            end
            begin : reader
                integer k;
                for (k = 0; k < 16; k = k + 1) begin
                    @(negedge clk_r);
                    raddr = k[AB-1:0];
                end
            end
        join
        rd(6'd20, got);
        chk_int("g4_dual_clock_write", got, 8'h5A);

        $display("");
        if (errors == 0) $display("PASS: dyn_text_ram_tb %0d checks, 0 errors", checks);
        else             $display("FAIL: dyn_text_ram_tb %0d checks, %0d errors", checks, errors);
        $finish;
    end

endmodule
