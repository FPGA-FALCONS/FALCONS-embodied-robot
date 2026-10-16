//////////////////////////////////////////////////////////////////////////////////
// ui_render_tb —— 中文界面渲染测试（逐像素叠加，无帧缓存）
//
// 做法：模拟 HDMI 像素扫描（pixel_clk + pixel_x/y + video_active/pixel_req），
// 只在关心的行区间内逐像素采样 o_pixel_rgb888，按颜色与区域分类计数。
//
// 【覆盖】
//   1. 菜单文字真的点亮了（未选中项用 E5E7EB）
//   2. 选中项高亮带存在（1D4ED8），位置随 i_page 移动
//   3. 状态栏文字点亮（白色 FFFFFF）
//   4. 数据页（i_page=2）：中文标签（白）+ **动态数值**（亮绿 7CFC98，来自字符 RAM）
//   5. 数据页不该出现占位提示（占位只在页 0/1/3）
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module ui_render_tb;

    reg         pixel_clk = 1'b0;
    reg         rst_n = 1'b0;
    reg  [11:0] px = 12'd0, py = 12'd0;
    reg         preq = 1'b0, vact = 1'b0;

    reg  [1:0]  page = 2'd0;
    reg  [1:0]  uimode = 2'd0;
    reg         run_i = 1'b0, estop_i = 1'b0, sigok_i = 1'b1;
    reg         armhint_i = 1'b0;           // A1：启动门禁提示（"请回中位"）
    reg  [1:0]  menusel = 2'd0;             // 菜单高亮项（两段式：摇杆改它、确认才换页）

    wire [23:0] rgb;
    wire [5:0]  dyn_raddr;
    reg  [7:0]  dyn_rdata = 8'd24;          // 字库索引 24 = '8'（合法字符，能点亮）

    integer errors = 0;
    integer cnt_menu_text, cnt_sel_text, cnt_hl, cnt_status, cnt_label, cnt_num, cnt_ph, cnt_yaw;
    integer px_manual, px_hint, px_estop, px_nosig;   // 状态栏文案切换对比

    integer x, y;

    ui_render dut (
        .i_pixel_clk    (pixel_clk),
        .i_reset_n      (rst_n),
        .i_pixel_x      (px),
        .i_pixel_y      (py),
        .i_pixel_req    (preq),
        .i_video_active (vact),
        .o_pixel_rgb888 (rgb),
        .i_page         (page),
        .i_ui_mode      (uimode),
        .i_run          (run_i),
        .i_estop        (estop_i),
        .i_signal_ok    (sigok_i),
        .i_arm_hint     (armhint_i),
        .i_menu_sel     (menusel),
        .o_dyn_raddr    (dyn_raddr),
        .i_dyn_rdata    (dyn_rdata)
    );

    always #10 pixel_clk = ~pixel_clk;      // 周期 20 ns（≈49 MHz 水平）

    // ------------------------------------------------------------------
    // 扫描若干行：逐像素设置坐标并采样本拍的输出
    //   注意屏幕坐标偏移：模块"提前 2 拍"取数，输出对应 x−2 的像素，
    //   分类窗口留了 ≥16 px 余量，不影响判定。
    // ------------------------------------------------------------------
    task scan;
        input integer y0;
        input integer y1;
        begin
            for (y = y0; y <= y1; y = y + 1) begin
                for (x = 0; x < 1060; x = x + 1) begin
                    px = x[11:0];
                    py = y[11:0];
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);

                    // ---- 按区域 + 颜色分类 ----
                    // 菜单文字（未选中）：x 16..112, y 48..64
                    if ((x >= 16) && (x < 208) && (y >= 96) && (y < 112) && (rgb == 24'hE5E7EB))
                        cnt_menu_text = cnt_menu_text + 1;
                    // 选中项文字（亮黄）：x 16..208, y 48..64
                    if ((x >= 16) && (x < 208) && (y >= 48) && (y < 64) && (rgb == 24'hFFE066))
                        cnt_sel_text = cnt_sel_text + 1;
                    if ((x < 208) && (y >= 40) && (y < 72) && (rgb == 24'h1D4ED8))
                        cnt_hl = cnt_hl + 1;
                    // 状态栏白字：x 16..112, y 560..576
                    if ((x >= 16) && (x < 112) && (y >= 560) && (y < 576) && (rgb == 24'hFFFFFF))
                        cnt_status = cnt_status + 1;
                    // 数据页标签（白）：x 208..304, y 120..136
                    if ((x >= 208) && (x < 304) && (y >= 120) && (y < 136) && (rgb == 24'hFFFFFF))
                        cnt_label = cnt_label + 1;
                    // 航向角行的动态数值（亮绿）：x 320..384, y 80..96
                    if ((x >= 320) && (x < 384) && (y >= 80) && (y < 96) && (rgb == 24'h7CFC98))
                        cnt_yaw = cnt_yaw + 1;
                    // 数据页数值（亮绿）：x 320..384, y 120..136
                    if ((x >= 320) && (x < 384) && (y >= 120) && (y < 136) && (rgb == 24'h7CFC98))
                        cnt_num = cnt_num + 1;
                    // 占位提示（白）：页 2 时此处应为 0
                    if ((x >= 208) && (x < 400) && (y >= 256) && (y < 272) && (rgb == 24'hFFFFFF))
                        cnt_ph = cnt_ph + 1;
                end
            end
            vact = 1'b0;
            preq = 1'b0;
        end
    endtask

    task check_pos;
        input [8*36-1:0] name;
        input integer    v;
        begin
            if (v <= 0) begin
                errors = errors + 1;
                $display("  FAIL  %0s: %0d (expect > 0)", name, v);
            end else begin
                $display("  pass  %0s = %0d px", name, v);
            end
        end
    endtask

    task check_zero;
        input [8*36-1:0] name;
        input integer    v;
        begin
            if (v != 0) begin
                errors = errors + 1;
                $display("  FAIL  %0s: %0d (expect 0)", name, v);
            end else begin
                $display("  pass  %0s = 0", name);
            end
        end
    endtask

    // ------------------------------------------------------------------
    initial begin
        $display("======== ui_render 界面渲染测试 ========");
        // Explicit stimulus initializes reset-independent combinational status on both simulators.
        #1; uimode = 2'd1; #1; uimode = 2'd0;
        cnt_menu_text = 0; cnt_sel_text = 0; cnt_hl = 0; cnt_status = 0;
        cnt_label = 0; cnt_num = 0; cnt_ph = 0;

        px = 12'd0; py = 12'd0; vact = 1'b0; preq = 1'b0;
        repeat (6) @(posedge pixel_clk);
        rst_n = 1'b1;
        repeat (4) @(posedge pixel_clk);

        // ---------------- 页 0：菜单 + 状态栏 ----------------
        $display("-- 1/2/3. 页0：菜单文字、选中高亮、状态栏 --");
        page = 2'd0; sigok_i = 1'b1; uimode = 2'd0; run_i = 1'b0; estop_i = 1'b0;
        repeat (4) @(posedge pixel_clk);
        scan(40, 130);                      // 覆盖菜单第 1 项（选中）与第 2 项（未选中）
        scan(550, 590);                     // 状态栏
        check_pos ("sel_text_px",   cnt_sel_text);
        check_pos ("menu_text_px",  cnt_menu_text);
        check_pos ("menu_hl_px",    cnt_hl);
        check_pos ("status_text_px", cnt_status);

        // ---------------- 数据页（页 2）----------------
        $display("-- 4. 页2：中文标签 + 动态数值（来自字符 RAM 的亮绿字符）--");
        cnt_label = 0; cnt_num = 0; cnt_ph = 0;
        page = 2'd2;
        repeat (4) @(posedge pixel_clk);
        cnt_yaw = 0;
        scan(80, 96);                       // 航向角行（IMU 链路接入后为实测偏航角）
        check_pos ("yaw_number_px", cnt_yaw);
        scan(120, 136);
        check_pos ("data_label_px", cnt_label);
        check_pos ("data_number_px", cnt_num);

        $display("-- 5. 页2 不应出现占位提示 --");
        scan(256, 272);
        check_zero("placeholder_on_page2", cnt_ph);

        // ---------------- 页 1 的占位提示应存在 ----------------
        $display("-- 6. 页1（雷达）应显示占位提示 --");
        cnt_ph = 0;
        page = 2'd1;
        repeat (4) @(posedge pixel_clk);
        scan(256, 272);
        check_pos ("placeholder_on_page1", cnt_ph);

        // ---------------- A1 启动门禁提示（请回中位）----------------
        //   判据用【状态栏白字像素数】对比（不比较具体汉字，只比较字数）：
        //     ST_MANUAL   "手动"   = 2 字
        //     ST_ESTOP    "急停"   = 2 字
        //     ST_NOSIGNAL "无信号" = 3 字
        //     ST_ARM_DENY "请回中位" = 4 字
        //   所以 hint 像素数 > manual 像素数 ⇒ 文案确实切到了更长的门禁提示；
        //   estop / nosignal 时像素数 < hint ⇒ 优先级顺序正确（门禁提示不抢占它们）。
        $display("-- 7. A1 启动门禁：arm_hint=1 显示 请回中位；急停/无信号优先级更高 --");
        page = 2'd0; sigok_i = 1'b1; estop_i = 1'b0; armhint_i = 1'b0; uimode = 2'd0;
        cnt_status = 0;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_manual = cnt_status;

        cnt_status = 0; armhint_i = 1'b1;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_hint = cnt_status;

        if (px_hint > px_manual) begin
            $display("  pass  arm_hint_status: hint=%0d > manual=%0d px", px_hint, px_manual);
        end else begin
            errors = errors + 1;
            $display("  FAIL  arm_hint_status: hint=%0d <= manual=%0d px (text not switched)", px_hint, px_manual);
        end

        cnt_status = 0; estop_i = 1'b1;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_estop = cnt_status;
        if (px_estop < px_hint) begin
            $display("  pass  estop_over_hint: estop=%0d < hint=%0d px", px_estop, px_hint);
        end else begin
            errors = errors + 1;
            $display("  FAIL  estop_over_hint: estop=%0d >= hint=%0d px (priority wrong)", px_estop, px_hint);
        end

        cnt_status = 0; estop_i = 1'b0; sigok_i = 1'b0;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_nosig = cnt_status;
        if (px_nosig < px_hint) begin
            $display("  pass  nosignal_over_hint: nosig=%0d < hint=%0d px", px_nosig, px_hint);
        end else begin
            errors = errors + 1;
            $display("  FAIL  nosignal_over_hint: nosig=%0d >= hint=%0d px (priority wrong)", px_nosig, px_hint);
        end

        $display("======== 结果 ========");
        if (errors == 0) $display("PASS: ui_render all checks passed");
        else             $display("FAIL: %0d checks failed", errors);
        $finish;
    end

    initial begin
        #10_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule
