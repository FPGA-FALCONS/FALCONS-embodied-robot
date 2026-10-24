//////////////////////////////////////////////////////////////////////////////
// sim\disp_driver_tb.v —— vendor disp_driver 的模块级回归
//
// 目的：verify `Disp_DE` 的窗口判定与像素透传语义。
//       历史：2026-10-20 曾给 `Disp_DE` 加 `(* syn_keep = 1 *)`，试图阻止综合器
//       把 `:107` 的比较表达式折叠进 `:113` 的三路 mux，以缩短
//       `hcount_r → 比较器 → 24 位 mux → Disp_*` 的长链。
//       综合复核后确认该做法属负优化 —— 同域违例 7 条 → 19 条、最差
//       −1.231 → −2.339，逻辑级数 28~30；原因是 `syn_keep` 只保寄存器不被
//       优化掉，**不阻止 logic replication**，比较器被复制到 24 个消费点。
//       该属性已按用户指令回退，本 tb 作为 `disp_driver` 的行为基线保留。
//
// 检查项：
//   G1 复位后 hcount_r / vcount_r / Disp_DE / Disp_HS / Disp_VS 全 0，Disp_PCLK 跟随 ClkDisp
//   G2 行扫描：hcount_r 从 0 递增，到 H_TOTAL-1 回绕到 0
//   G3 场扫描：每行结束 vcount_r 加 1
//   G4 Disp_DE 只在 hcount∈[hdat_begin, hdat_end) 且 vcount∈[vdat_begin, vdat_end)
//      的下一拍为 1，边界外为 0
//   G5 像素透传：Disp_DE=1 的下一拍 RGB == 该拍的 Data；Disp_DE=0 的下一拍 RGB == 0
//   G6 H_Addr / V_Addr 在有效区等于 hcount-hdat_begin-1 / vcount-vdat_begin
//
// ★ 时序要点：`Disp_DE`（:107）与 `{Disp_*} <= (Disp_DE)?Data:0`（:113）是两个同时钟的
//   always 块，:113 用的是**沿前**的 Disp_DE 与 Data。所以"透传"必须跨拍比对。
// ★ tb 写法要点：所有采样都在 `@(posedge ClkDisp); #1;` 之后做。
//   直接用 `wait(hcount_r == X)` 会在非阻塞赋值完成前触发，读到的是旧值。
//////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

`include "disp_parameter_cfg.vh"

module disp_driver_tb;

    localparam integer CLK_PERIOD = 20;      // pixel 域约 20.4 ns，仿真取 20 ns

    localparam integer HDAT_BEGIN = `HDMI_1024X600_H_SYNC + `HDMI_1024X600_H_BACK_PORCH
                                  + `HDMI_1024X600_H_LEFT_BORDER - 1;
    localparam integer HDAT_END   = `HDMI_1024X600_H_TOTAL - `HDMI_1024X600_H_RIGHT_BORDER
                                  - `HDMI_1024X600_H_FRONT_PORCH - 1;
    localparam integer VDAT_BEGIN = `HDMI_1024X600_V_SYNC + `HDMI_1024X600_V_BACK_PORCH
                                  + `HDMI_1024X600_V_TOP_BORDER - 1;
    localparam integer VDAT_END   = `HDMI_1024X600_V_TOTAL - `HDMI_1024X600_V_BOTTOM_BORDER
                                  - `HDMI_1024X600_V_FRONT_PORCH - 1;

    reg         ClkDisp;
    reg         Rst_n;
    reg  [23:0] Data;
    wire        DataReq;
    wire [11:0] H_Addr, V_Addr;
    wire        Disp_HS, Disp_VS;
    wire [7:0]  Disp_Red, Disp_Green, Disp_Blue;
    wire        Frame_Begin;
    wire        Disp_DE;
    wire        Disp_PCLK;

    integer checks = 0;
    integer errors = 0;

    task chk_int(input [255:0] name, input integer got, input integer exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL %0s: got %0d expect %0d (t=%0t)", name, got, exp, $time);
            end
        end
    endtask

    // 统一节拍：等一个时钟沿，再跳开非阻塞赋值生效的瞬间
    task step;
        begin
            @(posedge ClkDisp);
            #1;
        end
    endtask

    // 等到 hcount_r 等于指定值（沿后采样）
    task wait_h(input integer h);
        begin
            while (dut.hcount_r !== h[11:0]) step;
        end
    endtask

    disp_driver dut (
        .ClkDisp     (ClkDisp),
        .Rst_n       (Rst_n),
        .Data        (Data),
        .DataReq     (DataReq),
        .H_Addr      (H_Addr),
        .V_Addr      (V_Addr),
        .Disp_HS     (Disp_HS),
        .Disp_VS     (Disp_VS),
        .Disp_Red    (Disp_Red),
        .Disp_Green  (Disp_Green),
        .Disp_Blue   (Disp_Blue),
        .Frame_Begin (Frame_Begin),
        .Disp_DE     (Disp_DE),
        .Disp_PCLK   (Disp_PCLK)
    );

    always #(CLK_PERIOD/2) ClkDisp = ~ClkDisp;

    // Data 每拍 +1：Disp_* 只要等于某个历史 Data 值，就证明是透传而非巧合
    always @(posedge ClkDisp) Data <= Data + 24'd1;

    reg        de_d1;
    reg [23:0] data_d1;

    integer i;
    integer saw_de_high, saw_de_low;

    initial begin
        ClkDisp = 1'b0;
        Rst_n   = 1'b0;
        Data    = 24'h000001;
        de_d1   = 1'b0;
        data_d1 = 24'd0;
        saw_de_high = 0;
        saw_de_low  = 0;

        repeat (4) @(posedge ClkDisp);
        Rst_n = 1'b1;
        #1;                   // 复位刚释放、还没过第一个沿：计数器仍在复位值 0

        // ================= G1 复位后初值 =================
        chk_int("g1_reset_hcount", dut.hcount_r, 0);
        chk_int("g1_reset_vcount", dut.vcount_r, 0);
        chk_int("g1_reset_de",     Disp_DE,      0);
        chk_int("g1_reset_hs",     Disp_HS,      0);
        chk_int("g1_reset_vs",     Disp_VS,      0);
        chk_int("g1_pclk_follow",  Disp_PCLK,    ClkDisp);

        // ================= G2/G3 扫描计数器 =================
        step;
        chk_int("g2_hcount_1", dut.hcount_r, 1);
        repeat (10) step;
        chk_int("g2_hcount_11", dut.hcount_r, 11);

        wait_h(`HDMI_1024X600_H_TOTAL - 1);
        chk_int("g2_hcount_last", dut.hcount_r, `HDMI_1024X600_H_TOTAL - 1);
        step;
        chk_int("g2_hcount_wrap", dut.hcount_r, 0);
        chk_int("g3_vcount_after_1line", dut.vcount_r, 1);

        // ================= G4 边界（先把场计数压到有效行内）=================
        force dut.vcount_r = VDAT_BEGIN;

        wait_h(HDAT_BEGIN - 1);
        step;
        chk_int("g4_de_before_begin", Disp_DE, 0);
        wait_h(HDAT_BEGIN);
        step;
        chk_int("g4_de_at_begin", Disp_DE, 1);
        wait_h(HDAT_END - 1);
        step;
        chk_int("g4_de_at_end_m1", Disp_DE, 1);
        wait_h(HDAT_END);
        step;
        chk_int("g4_de_at_end", Disp_DE, 0);

        // ================= G6 地址映射 =================
        wait_h(HDAT_BEGIN + 5);
        step;
        chk_int("g6_haddr", H_Addr, dut.hcount_r - HDAT_BEGIN - 1);
        chk_int("g6_vaddr", V_Addr, dut.vcount_r - VDAT_BEGIN);

        // ================= G5 逐拍透传（跑满 1 个有效行）=================
        wait_h(0);
        step;
        for (i = 0; i < `HDMI_1024X600_H_TOTAL; i = i + 1) begin
            de_d1   <= Disp_DE;
            data_d1 <= Data;
            step;
            checks=checks+2;
            if(H_Addr !== (Disp_DE ? (dut.hcount_r-HDAT_BEGIN-1) : 12'd0))begin
                errors=errors+1;$display("FAIL: registered H address changed active or blanking alignment");
            end
            if(V_Addr !== (Disp_DE ? (dut.vcount_r-VDAT_BEGIN) : 12'd0))begin
                errors=errors+1;$display("FAIL: registered V address changed active or blanking alignment");
            end
            if (de_d1 === 1'b1) begin
                saw_de_high = saw_de_high + 1;
                checks = checks + 1;
                if ({Disp_Red, Disp_Green, Disp_Blue} !== data_d1) begin
                    errors = errors + 1;
                    $display("  FAIL g5_pixel_pass: got %06h expect %06h (t=%0t)",
                             {Disp_Red, Disp_Green, Disp_Blue}, data_d1, $time);
                end
            end else begin
                saw_de_low = saw_de_low + 1;
                checks = checks + 1;
                if ({Disp_Red, Disp_Green, Disp_Blue} !== 24'd0) begin
                    errors = errors + 1;
                    $display("  FAIL g5_pixel_blank: got %06h expect 0 (t=%0t)",
                             {Disp_Red, Disp_Green, Disp_Blue}, $time);
                end
            end
        end

        release dut.vcount_r;

        // 有效像素数应等于 HDAT_END - HDAT_BEGIN
        chk_int("g5_de_count", saw_de_high, HDAT_END - HDAT_BEGIN);
        chk_int("g5_blank_count", saw_de_low, `HDMI_1024X600_H_TOTAL - (HDAT_END - HDAT_BEGIN));

        $display("");
        $display("  统计：有效像素 %0d 拍、消隐 %0d 拍（H_TOTAL=%0d）",
                 saw_de_high, saw_de_low, `HDMI_1024X600_H_TOTAL);
        $display("  窗口：hdat [%0d, %0d)、vdat [%0d, %0d)", HDAT_BEGIN, HDAT_END, VDAT_BEGIN, VDAT_END);

        if (errors == 0) $display("PASS: disp_driver_tb %0d checks, 0 errors", checks);
        else             $display("FAIL: disp_driver_tb %0d checks, %0d errors", checks, errors);
        $finish;
    end

endmodule
