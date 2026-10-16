//////////////////////////////////////////////////////////////////////////////////
// ui_render_tb —— 中文界面渲染测试（逐像素叠加，无帧缓存）
//
// 做法：模拟 HDMI 像素扫描（pixel_clk + pixel_x/y + video_active/pixel_req），
// 只在关心的行区间内逐像素采样 o_pixel_rgb888，按颜色与区域分类计数。
//
// 【覆盖】（2026-10-16 更新）
//   1. 菜单文字真的点亮了（未选中项用 E5E7EB）
//   2. ★ 高亮带位置随 **i_menu_sel**（摇杆选中的项）移动，而不是随 i_page
//      —— 两段式：摇杆只改高亮，按确认才换页；两者是两个独立状态
//   3. 状态栏文字点亮（白色 FFFFFF），且**光标档文案**正确（"光标档"而非"请选择模式"）
//   4. 数据页（i_page=2）：中文标签（白）+ **动态数值**（亮绿 7CFC98，来自字符 RAM）
//   5. 数据页不该出现占位提示（占位只在页 0/1/3）
//   6. ★ 急停 与"无信号"同时成立时，状态栏必须显示"急停"（优先级验证，见第 7c 段）
//
// 【本测试台覆盖不到的部分（如实记录）】
//   "摇杆拨动只改 menu_sel、确认键才把 menu_sel 赋给 page"这条**状态机**逻辑在
//   `rtl\top\falcons_top.v` 里，本台只测 ui_render 的渲染，不测顶层 —— 顶层级
//   集成测试目前仍缺（见 doc\设计落实待办.md 的台账）。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module ui_render_tb;

    reg         pixel_clk = 1'b0;
    reg         rst_n = 1'b0;
    reg  [11:0] px = 12'd0, py = 12'd0;
    reg         preq = 1'b0, vact = 1'b0;

    reg  [1:0]  page = 2'd0;
    reg  [1:0]  uimode = 2'd0;
    // ★ 2026-10-16：`run_i` 已删 —— ui_render 的 i_run 端口与其在顶层的
    //   run_s/run_ps1/run_ps2 跨域链一并删除（模块内从不使用该信号）。
    reg         estop_i = 1'b0, sigok_i = 1'b1;
    reg         armhint_i = 1'b0;           // A1：启动门禁提示（"请回中位"）
    reg  [1:0]  menusel = 2'd0;             // 菜单高亮项（两段式：摇杆改它、确认才换页）

    wire [23:0] rgb;
    wire [5:0]  dyn_raddr;
    reg  [7:0]  dyn_rdata = 8'd24;          // 字库索引 24 = '8'（合法字符，能点亮）

    integer errors = 0;
    integer cnt_menu_text, cnt_sel_text, cnt_hl, cnt_status, cnt_label, cnt_num, cnt_ph, cnt_yaw;
    integer cnt_hl_win;                               // 单次 scan_hl 的高亮像素数
    integer px_manual, px_hint, px_estop, px_nosig;   // 状态栏文案切换对比
    integer px_estop_nosig;                           // ★ 急停 + 无信号 同时成立时的对比
    integer px_cursor;                                // 光标档文案对比

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
                    // 菜单文字（未选中项）：x 16..208, y 96..112（第 2 项的 16 px 文字行）
                    if ((x >= 16) && (x < 208) && (y >= 96) && (y < 112) && (rgb == 24'hE5E7EB))
                        cnt_menu_text = cnt_menu_text + 1;
                    // 选中项文字（亮黄）：x 16..208, y 48..64（第 1 项被高亮时的文字行）
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

    // ------------------------------------------------------------------
    // 只统计【菜单条高亮带】像素：x < 208（菜单条宽）且 y ∈ [y0, y1]
    //   高亮带几何（ui_render）：y ∈ [MENU_Y0 + sel*MENU_STEP - 8, +24)
    //     MENU_Y0 = 48 / MENU_STEP = 48  →  sel=0:[40,72) 1:[88,120) 2:[136,168) 3:[184,216)
    // ------------------------------------------------------------------
    task scan_hl;
        input integer y0;
        input integer y1;
        begin
            cnt_hl_win = 0;
            for (y = y0; y <= y1; y = y + 1) begin
                for (x = 0; x < 208; x = x + 1) begin
                    px = x[11:0];
                    py = y[11:0];
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    if (rgb == 24'h1D4ED8) cnt_hl_win = cnt_hl_win + 1;
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
        // Real stimulus initializes reset-independent combinational status in both engines.
        #1; uimode = 2'd1; #1; uimode = 2'd0;
        cnt_menu_text = 0; cnt_sel_text = 0; cnt_hl = 0; cnt_status = 0;
        cnt_label = 0; cnt_num = 0; cnt_ph = 0;

        px = 12'd0; py = 12'd0; vact = 1'b0; preq = 1'b0;
        repeat (6) @(posedge pixel_clk);
        rst_n = 1'b1;
        repeat (4) @(posedge pixel_clk);

        // ---------------- 页 0：菜单 + 状态栏 ----------------
        $display("-- 1/2/3. 页0：菜单文字、选中高亮、状态栏 --");
        page = 2'd0; sigok_i = 1'b1; uimode = 2'd0; estop_i = 1'b0;
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

        // ---------------- 7c. ★ 急停 与 无信号 同时成立 → 必须显示"急停" ----------------
        //   状态栏译码是 `if (i_estop) … else if (!i_signal_ok) …`（见 ui_render.v 的
        //   status_text），即**急停优先**。原测试台只单独测过这两档，
        //   **没测过两者同时置位** —— 而"接管档下丢信号 + 按下急停"正是现场最容易
        //   同时出现的组合，若优先级写反就会给出错误的处置提示。
        //   判据（不比较具体汉字，只比较字数，同第 7 段）：
        //     ST_ESTOP    "急停"   = 2 字
        //     ST_NOSIGNAL "无信号" = 3 字
        //   所以像素数必须 ≈ 急停那一档、且 < 无信号那一档。
        $display("-- 7c. estop=1 且 signal_ok=0 同时成立 → 应是 急停（2 字），不是 无信号（3 字）--");
        estop_i = 1'b1; sigok_i = 1'b0;
        cnt_status = 0;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_estop_nosig = cnt_status;
        if ((px_estop_nosig == px_estop) && (px_estop_nosig < px_nosig)) begin
            $display("  pass  estop_beats_nosignal: both=%0d, estop=%0d, nosig=%0d px",
                     px_estop_nosig, px_estop, px_nosig);
        end else begin
            errors = errors + 1;
            $display("  FAIL  estop_beats_nosignal: both=%0d, estop=%0d, nosig=%0d px (priority wrong)",
                     px_estop_nosig, px_estop, px_nosig);
        end
        estop_i = 1'b0; sigok_i = 1'b1;

        // ---------------- 8. ★ 高亮带跟随 i_menu_sel（两段式的"高亮"一半）----------------
        //   需求方 2026-10-16 明确："摇杆拨动后要按确认才切页" → 高亮项（i_menu_sel）
        //   与已确认页（i_page）是两个独立状态。原测试台把 menusel 恒钉在 0，
        //   只证明了"第 0 项有一条高亮带"，这条需求等于没测。
        $display("-- 8. 高亮带随 i_menu_sel 移动，且不残留到别的项 --");
        page       = 2'd0;
        uimode     = 2'd0;
        estop_i    = 1'b0;
        sigok_i    = 1'b1;
        armhint_i  = 1'b0;
        begin : hl_scan
            integer mi, other;
            for (mi = 0; mi < 4; mi = mi + 1) begin
                menusel = mi[1:0];
                repeat (4) @(posedge pixel_clk);

                // (a) 高亮应出现在本项的带区
                scan_hl(48 + mi * 48 - 8, 48 + mi * 48 + 23);
                if (cnt_hl_win > 0) begin
                    $display("  pass  menu_hl_sel%0d = %0d px", mi, cnt_hl_win);
                end else begin
                    errors = errors + 1;
                    $display("  FAIL  menu_hl_sel%0d = 0 px (highlight did not follow i_menu_sel)", mi);
                end

                // (b) 另一项的带区不应有高亮（防"高亮带随 page 移动/残留"）
                other = (mi == 0) ? 1 : 0;
                scan_hl(48 + other * 48 - 8, 48 + other * 48 + 23);
                if (cnt_hl_win == 0) begin
                    $display("  pass  menu_hl_sel%0d_other_empty", mi);
                end else begin
                    errors = errors + 1;
                    $display("  FAIL  menu_hl_sel%0d other=%0d px (stale highlight)", mi, cnt_hl_win);
                end
            end
        end
        menusel = 2'd0;
        repeat (4) @(posedge pixel_clk);

        // ---------------- 8b. 光标档状态栏文案 ----------------
        //   模式只有两档：0 = 手动接管 → "手动"(2 字) / 1 = 屏幕光标 → "光标档"(3 字)。
        //   原实现把光标档落进 else 显示"请选择模式"(5 字) —— 语义错（用户已经在光标档），
        //   且 `== 2'd2`（自动档）是永不成立的死分支。用像素数比例锁住"是 3 字不是 5 字"。
        $display("-- 8b. 光标档状态栏显示 光标档（3 字），不是 请选择模式（5 字）--");
        estop_i = 1'b0; sigok_i = 1'b1; armhint_i = 1'b0;
        uimode = 2'd0; cnt_status = 0;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_manual = cnt_status;

        uimode = 2'd1; cnt_status = 0;
        repeat (4) @(posedge pixel_clk);
        scan(550, 590);
        px_cursor = cnt_status;

        if ((px_cursor > 0) && (px_cursor < px_manual * 2)) begin
            $display("  pass  cursor_status: cursor=%0d, manual=%0d px (≈3 字 vs 2 字)", px_cursor, px_manual);
        end else begin
            errors = errors + 1;
            $display("  FAIL  cursor_status: cursor=%0d, manual=%0d px (文案不对：可能是 请选择模式)", px_cursor, px_manual);
        end

        $display("======== 结果 ========");
        if (errors == 0) $display("PASS: ui_render all checks passed");
        else             $display("FAIL: %0d checks failed", errors);
        $finish;
    end

    initial begin
        // 看门狗上限：本台是"逐像素扫描"，新增的高亮扫描段（i_menu_sel 0..3）与
        // 光标档文案段使总扫描量上升到 ~35 万拍 ≈ 7 ms，原 10 ms 上限会被撞到
        // （表现为 "FAIL: timeout"，但此前各段其实都 pass）→ 放宽到 40 ms。
        #40_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule
