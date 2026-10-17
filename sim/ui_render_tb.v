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
//   7. ★ 段边界（2026-10-17 新增）：把"文字段覆盖哪几个字组"的判定与点阵数据对齐，
//      段起点左侧不得多出像素（原缺陷：每个 label 左边一串竖排白点，来自
//      ui_text_rom[0] 首字「摄」的残列），段末尾最右两列不得丢笔画。
//      判据用【字库/字符 ROM 本体】做黄金参考逐像素比对，偏移量自动探测并打印。
//      覆盖两类数据源：静态文案 ROM（菜单/标签）与动态字符 RAM（数值）。
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
    integer cnt_car;                                  // 车图包围盒内的白色像素数
    integer cnt_car_spill;                            // ★ 车图包围盒【外】的白色像素数（应为 0）
    integer cnt_hl_win;                               // 单次 scan_hl 的高亮像素数
    integer px_manual, px_hint, px_estop, px_nosig;   // 状态栏文案切换对比
    integer px_estop_nosig;                           // ★ 急停 + 无信号 同时成立时的对比
    integer px_cursor;                                // 光标档文案对比

    integer x, y;

    // ------------------------------------------------------------------
    // 布局常量（与 ui_render.v 的 parameter 对应）
    //   ★ 2026-10-17 起字号分两套：
    //       大字 24x24 → 左侧菜单条（4 项 + 状态栏档位），组宽 24 px
    //       小字 16x16 → 右侧内容区（数据页标签/数值、占位提示），组宽 16 px
    // ------------------------------------------------------------------
    // 大字（左侧）
    localparam integer LG_CELL      = 24;
    localparam integer LG_MENU_Y0   = 72;      // 第一项文字 y（MENU_Y0）
    localparam integer LG_MENU_STEP = 72;      // 项间距（MENU_STEP）
    localparam integer LG_STATUS_Y  = 552;     // 状态栏文字 y（STATUS_Y）
    localparam integer LG_XW_SLOT   = 1;       // 菜单/状态栏文字起始大字组（x=24）
    localparam integer MENU_W_PX    = 208;     // 菜单条宽度（不变）
    // 小字（右侧）
    localparam integer PAGE_XW_SLOT = 13;      // 内容区文字起始小字组（x=208，占位提示用）
    // 数据页（2026-10-17 车图改版）各元素列位置，与 ui_render 的 PX_* 一一对应
    localparam integer PX_YAW_SLOT  = 35;      // 车头偏航角数值 x=560
    localparam integer PX_WL_SLOT   = 26;      // 左侧轮速 x=416
    localparam integer PX_WR_SLOT   = 44;      // 右侧轮速 x=704
    localparam integer PX_LBL_SLOT  = 50;      // 右侧列表标签 x=800
    localparam integer PX_VAL_SLOT  = 55;      // 右侧列表数值 x=880

    // ★ 文案地址一律来自生成的头文件，**不再硬编码**：
    //   本次把「车身运行数据显示」缩短为「运行数据显示」（8→6 字）后，其后所有文案的
    //   起始地址整体前移 2。硬编码 51 之类会静默指向错误的字符流，断言会假红或假绿。
    `include "ui_text_index.vh"

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
                    // ★ 2026-10-17：左侧菜单/档位改为 24 px 大字（行距 72、起点 x=24），
                    //   窗口按下表更新；右侧内容区仍是 16 px 小字，窗口不变。
                    // 菜单文字（未选中项）：第 2 项文字行 y = 72+72 .. +24
                    if ((x >= 24) && (x < MENU_W_PX) &&
                        (y >= (LG_MENU_Y0 + LG_MENU_STEP)) && (y < (LG_MENU_Y0 + LG_MENU_STEP + LG_CELL)) &&
                        (rgb == 24'hE5E7EB))
                        cnt_menu_text = cnt_menu_text + 1;
                    // 选中项文字（亮黄）：第 1 项文字行 y = 72 .. 96
                    if ((x >= 24) && (x < MENU_W_PX) &&
                        (y >= LG_MENU_Y0) && (y < (LG_MENU_Y0 + LG_CELL)) &&
                        (rgb == 24'hFFE066))
                        cnt_sel_text = cnt_sel_text + 1;
                    // 高亮带（第 0 项）：y = 72-12 .. 72+36
                    if ((x < MENU_W_PX) &&
                        (y >= (LG_MENU_Y0 - 12)) && (y < (LG_MENU_Y0 + 36)) &&
                        (rgb == 24'h1D4ED8))
                        cnt_hl = cnt_hl + 1;
                    // 状态栏白字：x 24..144, y 552..576（24 px 字）
                    if ((x >= 24) && (x < 144) &&
                        (y >= LG_STATUS_Y) && (y < (LG_STATUS_Y + LG_CELL)) &&
                        (rgb == 24'hFFFFFF))
                        cnt_status = cnt_status + 1;
                    // ★ 2026-10-17 数据页改版：车图居中 + 轮旁转速 + 右侧列表
                    // 右侧列表标签（白）：x 800..864, y 208..224（「中心速度」4 字）
                    if ((x >= 800) && (x < 864) && (y >= 208) && (y < 224) && (rgb == 24'hFFFFFF))
                        cnt_label = cnt_label + 1;
                    // 车头偏航角数值（亮绿）：x 560..624, y 160..176
                    if ((x >= 560) && (x < 624) && (y >= 160) && (y < 176) && (rgb == 24'h7CFC98))
                        cnt_yaw = cnt_yaw + 1;
                    // 右侧列表数值（亮绿）：x 880..944, y 208..224
                    if ((x >= 880) && (x < 944) && (y >= 208) && (y < 224) && (rgb == 24'h7CFC98))
                        cnt_num = cnt_num + 1;
                    // 小车俯视简笔图（白色）：包围盒 x 492..692, y 185..415
                    if ((x >= 492) && (x <= 692) && (y >= 185) && (y <= 415) && (rgb == 24'hFFFFFF))
                        cnt_car = cnt_car + 1;
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
    // 只统计【菜单条高亮带】像素：x < MENU_W 且 y ∈ [y0, y1]
    //   高亮带几何（ui_render，2026-10-17 随 24 px 大字放大）：
    //     y ∈ [MENU_Y0 + sel*MENU_STEP - 12, +36)
    //     MENU_Y0 = 72 / MENU_STEP = 72  →  sel=0:[60,108) 1:[132,180) 2:[204,252) 3:[276,324)
    // ------------------------------------------------------------------
    task scan_hl;
        input integer y0;
        input integer y1;
        begin
            cnt_hl_win = 0;
            for (y = y0; y <= y1; y = y + 1) begin
                for (x = 0; x < MENU_W_PX; x = x + 1) begin
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

    // ==================================================================
    // ★ 数据页小车俯视简笔图：黄金参考 + 越界检查（2026-10-17 新增）
    //
    //   期望值**从 SVG 原始坐标独立重算**，不复用 ui_render 里的屏幕坐标常数 ——
    //   否则 RTL 常数写错时，TB 用同一套错常数也会"一致通过"，断言就白写了。
    //
    //   SVG 事实（doc/小车俯视简笔.svg，画布 800x600）：
    //     车体 = 45° 斜切凸六边形 (360,170)(330,200)(330,400)(470,400)(470,200)(440,170)，
    //            stroke-width=3、fill=none → 屏幕上表现为 3 px 描边
    //     四轮 = 25x48 实心矩形，x=300 / 474.81，y=215 / 335
    //   屏幕 = SVG + (CAR_DX, CAR_DY) = SVG + (192, 15)
    // ==================================================================
    localparam integer CAR_DX = 192;
    localparam integer CAR_DY = 15;

    function car_expect;
        input integer xs;          // 屏幕 x
        input integer ys;          // 屏幕 y
        integer sx, sy;
        reg out_, in_;
        begin
            sx = xs - CAR_DX;
            sy = ys - CAR_DY;
            // 车体外轮廓
            out_ = (sx >= 330) && (sx <= 470) && (sy >= 170) && (sy <= 400) &&
                   ((sy >= 200) ||
                    ((sx >= (330 + (200 - sy))) && (sx <= (470 - (200 - sy)))));
            // 车体内轮廓（外轮廓向内收 3 px）
            in_  = (sx >= 333) && (sx <= 467) && (sy >= 173) && (sy <= 397) &&
                   ((sy >= 203) ||
                    ((sx >= (333 + (203 - sy))) && (sx <= (467 - (203 - sy)))));
            car_expect = (out_ && !in_) ||
                         ((sx >= 300) && (sx <= 324) && (sy >= 215) && (sy <= 262)) ||
                         ((sx >= 475) && (sx <= 499) && (sy >= 215) && (sy <= 262)) ||
                         ((sx >= 300) && (sx <= 324) && (sy >= 335) && (sy <= 382)) ||
                         ((sx >= 475) && (sx <= 499) && (sy >= 335) && (sy <= 382));
        end
    endfunction

    // 车图扫描：包围盒内逐像素对黄金参考；同时统计包围盒【外】的白色像素（越界）
    integer err_car;
    task scan_car;
        integer yy, xx;
        reg actual_on, expect_on;
        begin
            err_car = 0;
            cnt_car = 0;
            cnt_car_spill = 0;
            for (yy = 150; yy <= 430; yy = yy + 1) begin
                for (xx = 400; xx <= 780; xx = xx + 1) begin
                    px = xx[11:0];
                    py = yy[11:0];
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;
                    actual_on = (rgb === 24'hFFFFFF);
                    if ((xx >= 492) && (xx <= 692) && (yy >= 185) && (yy <= 415)) begin
                        expect_on = car_expect(xx, yy);
                        if (actual_on) cnt_car = cnt_car + 1;
                        if (actual_on !== expect_on) err_car = err_car + 1;
                    end else if (actual_on) begin
                        cnt_car_spill = cnt_car_spill + 1;
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;
        end
    endtask

    // ------------------------------------------------------------------
    // ★ 段边界黄金参考比对（2026-10-17 修正"label 左侧竖排白点"的回归）
    //
    //   缺陷形态（原实现）：门控 in_text 取的是【当前拍】的 sel，而点阵数据来自
    //   "2 拍前请求、请求 pixel_x+2 所在字组"的那一条通路，两者不同拍：
    //     → 段起点【左侧】被误开，画的是 ui_text_rom[0] 首字「摄」的残列
    //       —— 所以每个 label 左边都是同一组竖排白点；
    //     → 段末尾最右两列被误关（笔画缺失）。
    //
    //   判据：不重复实现渲染逻辑，而是直接查【字库/字符 ROM 本体】做黄金参考：
    //       期望字符索引 = dut.u_text_rom_sm.mem[txt_addr + (slot - xw)]
    //       期望字模行   = dut.u_font_rom.mem[char_index*16 + (y & 15)]
    //       期望点亮     = 该行 bit[15 - (x & 15)]
    //   逐像素比对整段，并把"段起点左侧 2 列必须无字"单列成显式计数。
    // ------------------------------------------------------------------
    // 打印某扫描行的文字点亮序列（ASCII 0/1），用于比对图案是否整体错位
    task dump_row;
        input integer yy;
        input [23:0]  text_rgb;
        integer xx;
        reg [8*96-1:0] s;
        begin
            s = {8*96{1'b0}};
            for (xx = 8; xx < 104; xx = xx + 1) begin
                px = xx[11:0];
                py = yy[11:0];
                vact = 1'b1;
                preq = 1'b1;
                @(posedge pixel_clk);
                #1;
                s = {s[8*96-9:0], ((rgb === text_rgb) ? 8'h31 : 8'h30)};
            end
            vact = 1'b0;
            preq = 1'b0;
            $display("  ROW y=%0d x=8..103: %0s", yy, s);
        end
    endtask

    integer err_left;      // 段起点左侧 2 列被误点亮（本次缺陷的直接指标）
    integer err_glyph;     // 段内与字模不符
    integer err_other;     // 其它段外位置被点亮
    integer probe_off;     // 自动探测到的"采样输出 ↔ 像素坐标"偏移
    integer err_off [0:10];

    // 段内期望：像素 xo 是否应点亮（查字库本体）
    function glyph_expect_rom;
        input integer xo;
        input integer ybase_yy;
        input integer xw;
        input integer len;
        input integer txt_addr;
        integer slot, ci;
        reg [15:0] frow;
        begin
            slot = xo >> 4;
            if ((slot >= xw) && (slot < xw + len)) begin
                ci   = dut.u_text_rom_sm.mem[txt_addr + (slot - xw)];
                frow = dut.u_font_rom.mem[ci * 16 + (ybase_yy & 15)];
                glyph_expect_rom = frow[15 - (xo & 15)];
            end else begin
                glyph_expect_rom = 1'b0;
            end
        end
    endfunction

    // 大字段内期望：像素 xo 是否应点亮（查 24x24 字库本体）
    //   段基址都取 24 的倍数（72/144/216/288/552）→ 字模行号 = 绝对 y % 24
    function glyph_expect_lg;
        input integer xo;
        input integer y_abs;
        input integer xw;
        input integer len;
        input integer txt_addr;
        integer slot, col, ci, row;
        reg [23:0] frow;
        begin
            slot = xo / LG_CELL;
            col  = xo % LG_CELL;
            if ((slot >= xw) && (slot < xw + len)) begin
                ci   = dut.u_text_rom_lg.mem[txt_addr + (slot - xw)];
                row  = y_abs % LG_CELL;
                frow = dut.u_font_rom_lg.mem[ci * LG_CELL + row];
                glyph_expect_lg = frow[LG_CELL - 1 - col];
            end else begin
                glyph_expect_lg = 1'b0;
            end
        end
    endfunction

    // ------------------------------------------------------------------
    // ★ 段边界黄金参考比对（2026-10-17 修正"label 左侧竖排白点"的回归）
    //
    //   缺陷形态（原实现）：门控 in_text 比点阵数据早 2 像素
    //     → 段起点【左侧 2 像素】被点亮，画的是 ui_text_rom[0]
    //       （= 第一条文案首字「摄」）的最右两列 —— 所以每个 label 左边
    //       都是同一组竖排白点；
    //     → 同时段末尾最右 2 列被漏掉（笔画缺失）。
    //
    //   判据：不重复实现渲染逻辑，直接查【字库/字符 ROM 本体】当黄金参考：
    //       期望字符索引 = dut.u_text_rom_sm.mem[txt_addr + (slot - xw)]
    //       期望字模行   = dut.u_font_rom.mem[char_index*16 + (y & 15)]
    //       期望点亮     = 该行 bit[15 - (x & 15)]
    //
    //   ★ 偏移自动探测：TB 采样的 rgb 与屏幕上"第几个像素"之间存在固定流水线
    //     延迟，本台不硬编码这个数（否则一旦改流水线深度，断言会集体假红），
    //     而是对 −5..+5 的偏移各统计一次不符数，取最小者作为真实偏移，
    //     再在该偏移下单独统计"段左 2 列多字"。探测结果会打印出来备查。
    // ------------------------------------------------------------------
    task check_seg_rom;
        input integer ybase;
        input integer xw;
        input integer len;
        input integer txt_addr;
        input [23:0]  text_rgb;
        input [8*36-1:0] name;
        integer yy, xx, xa, xo, oi, obest, obesterr;
        reg expect_on, actual_on;
        begin
            for (oi = 0; oi <= 10; oi = oi + 1) err_off[oi] = 0;
            for (yy = 0; yy < 16; yy = yy + 1) begin
                for (xx = ((xw > 2) ? ((xw - 2) * 16) : 2);
                     xx <= (xw + len) * 16 + 10; xx = xx + 1) begin
                    px = xx[11:0];
                    py = (ybase + yy);
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;    // 沿后 1 ns 再采样：读的是本拍的寄存器值（与硬件 pixel_req 采样点一致），
                           // 否则读到沿前值，会把组合/寄存信号的区别误当成像素错位

                    xa = xx - 2;
                    actual_on = (rgb === text_rgb);
                    // ★ 偏移探测【只用段中部】像素（离两端各 ≥4 像素）：
                    //   段边界缺陷（多点/少列）若也参与选择，会把最优偏移带偏，
                    //   从而"吸收"掉缺陷 —— 这正是自动探测最容易掩盖问题的地方。
                    for (oi = 0; oi <= 10; oi = oi + 1) begin
                        xo = xa + (oi - 5);
                        if ((xo >= xw * 16 + 4) && (xo < (xw + len) * 16 - 4)) begin
                            expect_on = glyph_expect_rom(xo, ybase + yy, xw, len, txt_addr);
                            if (actual_on !== expect_on) err_off[oi] = err_off[oi] + 1;
                        end
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;

            obest = 5; obesterr = err_off[5];
            for (oi = 0; oi <= 10; oi = oi + 1) begin
                if (err_off[oi] < obesterr) begin
                    obesterr = err_off[oi];
                    obest    = oi;
                end
            end
            err_glyph  = obesterr;
            probe_off  = obest - 5;

            // 在最优偏移下单独统计"段起点左侧 2 列被误点亮"
            err_left  = 0;
            err_other = 0;
            for (yy = 0; yy < 16; yy = yy + 1) begin
                for (xx = ((xw > 2) ? ((xw - 2) * 16) : 2);
                     xx <= (xw + len) * 16 + 10; xx = xx + 1) begin
                    px = xx[11:0];
                    py = (ybase + yy);
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;    // 沿后 1 ns 再采样：读的是本拍的寄存器值（与硬件 pixel_req 采样点一致），
                           // 否则读到沿前值，会把组合/寄存信号的区别误当成像素错位
                    actual_on = (rgb === text_rgb);
                    xo = (xx - 2) + probe_off;
                    if (xo >= 0) begin
                        expect_on = glyph_expect_rom(xo, ybase + yy, xw, len, txt_addr);
                        if (actual_on && !expect_on) begin
                            if ((xo >= xw * 16 - 2) && (xo < xw * 16)) err_left  = err_left + 1;
                            else                                            err_other = err_other + 1;
                        end
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;

            if ((err_left == 0) && (err_glyph == 0) && (err_other == 0)) begin
                $display("  pass  %0s: 偏移=%0d 段内不符=0 段左2列多字=0", name, probe_off);
            end else begin
                errors = errors + 1;
                $display("  FAIL  %0s: 偏移=%0d 段内不符=%0d 段左2列多字=%0d 其它段外多字=%0d",
                         name, probe_off, err_glyph, err_left, err_other);
            end
        end
    endtask

    // 动态数值段（src=ram）：字符索引直接取 i_dyn_rdata（本台恒为 dyn_rdata）
    task check_seg_ram;
        input integer ybase;
        input integer xw;
        input integer len;
        input [7:0]   charidx;
        input [23:0]  text_rgb;
        input [8*36-1:0] name;
        integer yy, xx, xa, xo, oi, obest, obesterr;
        reg expect_on, actual_on;
        reg [15:0] frow;
        begin
            for (oi = 0; oi <= 10; oi = oi + 1) err_off[oi] = 0;
            for (yy = 0; yy < 16; yy = yy + 1) begin
                for (xx = ((xw > 2) ? ((xw - 2) * 16) : 2);
                     xx <= (xw + len) * 16 + 10; xx = xx + 1) begin
                    px = xx[11:0];
                    py = (ybase + yy);
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;    // 沿后 1 ns 再采样：读的是本拍的寄存器值（与硬件 pixel_req 采样点一致），
                           // 否则读到沿前值，会把组合/寄存信号的区别误当成像素错位

                    xa = xx - 2;
                    actual_on = (rgb === text_rgb);
                    frow = dut.u_font_rom.mem[charidx * 16 + ((ybase + yy) & 15)];
                    // 同上：偏移探测只用段中部像素
                    for (oi = 0; oi <= 10; oi = oi + 1) begin
                        xo = xa + (oi - 5);
                        if ((xo >= xw * 16 + 4) && (xo < (xw + len) * 16 - 4)) begin
                            expect_on = frow[15 - (xo & 15)];
                            if (actual_on !== expect_on) err_off[oi] = err_off[oi] + 1;
                        end
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;

            obest = 5; obesterr = err_off[5];
            for (oi = 0; oi <= 10; oi = oi + 1) begin
                if (err_off[oi] < obesterr) begin
                    obesterr = err_off[oi];
                    obest    = oi;
                end
            end
            err_glyph = obesterr;
            probe_off = obest - 5;

            err_left  = 0;
            err_other = 0;
            for (yy = 0; yy < 16; yy = yy + 1) begin
                for (xx = ((xw > 2) ? ((xw - 2) * 16) : 2);
                     xx <= (xw + len) * 16 + 10; xx = xx + 1) begin
                    px = xx[11:0];
                    py = (ybase + yy);
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;    // 沿后 1 ns 再采样：读的是本拍的寄存器值（与硬件 pixel_req 采样点一致），
                           // 否则读到沿前值，会把组合/寄存信号的区别误当成像素错位
                    actual_on = (rgb === text_rgb);
                    xo = (xx - 2) + probe_off;
                    if (xo >= 0) begin
                        // ★ 必须按当前行重新取字模行：frow 若沿用第一遍循环结束时的值，
                        //   期望图案会整体错行（表现为"段内实际点亮全被判成段外多字"）
                        frow = dut.u_font_rom.mem[charidx * 16 + ((ybase + yy) & 15)];
                        if (((xo >> 4) >= xw) && ((xo >> 4) < xw + len))
                            expect_on = frow[15 - (xo & 15)];
                        else
                            expect_on = 1'b0;
                        if (actual_on && !expect_on) begin
                            if ((xo >= xw * 16 - 2) && (xo < xw * 16)) err_left  = err_left + 1;
                            else                                            err_other = err_other + 1;
                        end
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;

            if ((err_left == 0) && (err_glyph == 0) && (err_other == 0)) begin
                $display("  pass  %0s: 偏移=%0d 段内不符=0 段左2列多字=0", name, probe_off);
            end else begin
                errors = errors + 1;
                $display("  FAIL  %0s: 偏移=%0d 段内不符=%0d 段左2列多字=%0d 其它段外多字=%0d",
                         name, probe_off, err_glyph, err_left, err_other);
            end
        end
    endtask

    // ------------------------------------------------------------------
    // ★ 大字段边界黄金参考（2026-10-17 字号放大 1.5 倍后新增）
    //   判据与 check_seg_rom 一致，只是换成 24 px 组 / 24x24 字库：
    //     期望字符索引 = dut.u_text_rom_lg.mem[txt_addr + (slot - xw)]
    //     期望字模行   = dut.u_font_rom_lg.mem[ci*24 + (y % 24)]
    //     期望点亮     = 该行 bit[23 - (x % 24)]
    //   为什么单写一份：Verilog-2001 的 task 里没法按参数去挑不同的层次实例名
    //   （u_font_rom / u_font_rom_lg），硬套只会在内部堆两处 case。
    // ------------------------------------------------------------------
    task check_seg_lg;
        input integer ybase;
        input integer xw;
        input integer len;
        input integer txt_addr;
        input [23:0]  text_rgb;
        input [8*36-1:0] name;
        integer yy, xx, xa, xo, oi, obest, obesterr;
        reg expect_on, actual_on;
        begin
            for (oi = 0; oi <= 10; oi = oi + 1) err_off[oi] = 0;
            // ---- 第一遍：只用段中部像素探测采样偏移 ----
            for (yy = 0; yy < LG_CELL; yy = yy + 1) begin
                for (xx = ((xw > 2) ? ((xw - 2) * LG_CELL) : 2);
                     xx <= (xw + len) * LG_CELL + 10; xx = xx + 1) begin
                    px = xx[11:0];
                    py = (ybase + yy);
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;    // 沿后 1 ns 再采样：读本拍寄存器值（与硬件 pixel_req 采样点一致）
                    xa = xx - 2;
                    actual_on = (rgb === text_rgb);
                    for (oi = 0; oi <= 10; oi = oi + 1) begin
                        xo = xa + (oi - 5);
                        if ((xo >= xw * LG_CELL + 4) && (xo < (xw + len) * LG_CELL - 4)) begin
                            expect_on = glyph_expect_lg(xo, ybase + yy, xw, len, txt_addr);
                            if (actual_on !== expect_on) err_off[oi] = err_off[oi] + 1;
                        end
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;

            obest = 5; obesterr = err_off[5];
            for (oi = 0; oi <= 10; oi = oi + 1) begin
                if (err_off[oi] < obesterr) begin
                    obesterr = err_off[oi];
                    obest    = oi;
                end
            end
            err_glyph = obesterr;
            probe_off = obest - 5;

            // ---- 第二遍：在最优偏移下统计"段起点左侧多字"与其它段外多字 ----
            err_left  = 0;
            err_other = 0;
            for (yy = 0; yy < LG_CELL; yy = yy + 1) begin
                for (xx = ((xw > 2) ? ((xw - 2) * LG_CELL) : 2);
                     xx <= (xw + len) * LG_CELL + 10; xx = xx + 1) begin
                    px = xx[11:0];
                    py = (ybase + yy);
                    vact = 1'b1;
                    preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;
                    actual_on = (rgb === text_rgb);
                    xo = (xx - 2) + probe_off;
                    if (xo >= 0) begin
                        expect_on = glyph_expect_lg(xo, ybase + yy, xw, len, txt_addr);
                        if (actual_on && !expect_on) begin
                            if ((xo >= xw * LG_CELL - 2) && (xo < xw * LG_CELL)) err_left  = err_left + 1;
                            else                                               err_other = err_other + 1;
                        end
                    end
                end
            end
            vact = 1'b0;
            preq = 1'b0;

            if ((err_left == 0) && (err_glyph == 0) && (err_other == 0)) begin
                $display("  pass  %0s: 偏移=%0d 段内不符=0 段左2列多字=0", name, probe_off);
            end else begin
                errors = errors + 1;
                $display("  FAIL  %0s: 偏移=%0d 段内不符=%0d 段左2列多字=%0d 其它段外多字=%0d",
                         name, probe_off, err_glyph, err_left, err_other);
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
        scan(56, 190);                      // 覆盖菜单第 1 项（选中）与第 2 项（未选中），24px 大字
        scan(536, 596);                     // 状态栏（24px 大字，文字 552..576）
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
        scan(160, 176);                     // 车头偏航角行（IMU 链路接入后为实测偏航角）
        check_pos ("yaw_number_px", cnt_yaw);
        scan(208, 224);                     // 右侧列表「中心速度」行（标签 + 数值）
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
        scan(536, 596);
        px_manual = cnt_status;

        cnt_status = 0; armhint_i = 1'b1;
        repeat (4) @(posedge pixel_clk);
        scan(536, 596);
        px_hint = cnt_status;

        if (px_hint > px_manual) begin
            $display("  pass  arm_hint_status: hint=%0d > manual=%0d px", px_hint, px_manual);
        end else begin
            errors = errors + 1;
            $display("  FAIL  arm_hint_status: hint=%0d <= manual=%0d px (text not switched)", px_hint, px_manual);
        end

        cnt_status = 0; estop_i = 1'b1;
        repeat (4) @(posedge pixel_clk);
        scan(536, 596);
        px_estop = cnt_status;
        if (px_estop < px_hint) begin
            $display("  pass  estop_over_hint: estop=%0d < hint=%0d px", px_estop, px_hint);
        end else begin
            errors = errors + 1;
            $display("  FAIL  estop_over_hint: estop=%0d >= hint=%0d px (priority wrong)", px_estop, px_hint);
        end

        cnt_status = 0; estop_i = 1'b0; sigok_i = 1'b0;
        repeat (4) @(posedge pixel_clk);
        scan(536, 596);
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
        scan(536, 596);
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
                scan_hl(LG_MENU_Y0 + mi * LG_MENU_STEP - 12, LG_MENU_Y0 + mi * LG_MENU_STEP + 35);
                if (cnt_hl_win > 0) begin
                    $display("  pass  menu_hl_sel%0d = %0d px", mi, cnt_hl_win);
                end else begin
                    errors = errors + 1;
                    $display("  FAIL  menu_hl_sel%0d = 0 px (highlight did not follow i_menu_sel)", mi);
                end

                // (b) 另一项的带区不应有高亮（防"高亮带随 page 移动/残留"）
                other = (mi == 0) ? 1 : 0;
                scan_hl(LG_MENU_Y0 + other * LG_MENU_STEP - 12, LG_MENU_Y0 + other * LG_MENU_STEP + 35);
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
        scan(536, 596);
        px_manual = cnt_status;

        uimode = 2'd1; cnt_status = 0;
        repeat (4) @(posedge pixel_clk);
        scan(536, 596);
        px_cursor = cnt_status;

        if ((px_cursor > 0) && (px_cursor < px_manual * 2)) begin
            $display("  pass  cursor_status: cursor=%0d, manual=%0d px (≈3 字 vs 2 字)", px_cursor, px_manual);
        end else begin
            errors = errors + 1;
            $display("  FAIL  cursor_status: cursor=%0d, manual=%0d px (文案不对：可能是 请选择模式)", px_cursor, px_manual);
        end

        // ---------------- 9. ★ 段边界回归：菜单文字段（src = 静态 ROM）----------------
        //   缺陷：段起点左侧 2 像素被点亮（画 ui_text_rom[0] 的残列）、段末尾最右 2 列丢失。
        //   这里用字库黄金参考逐像素比对整段（24 px 大字，x=24..143，5 个字），并单列
        //   "起点左侧 2 列必须无字"。被测项必须【不被高亮】，否则文字色是 FFE066，
        //   所以把高亮钉在第 4 项（menusel=3）。
        //   ★ 2026-10-17：菜单已改为 24x24 大字（24 px 组），断言改用 check_seg_lg。
        $display("-- 9. 菜单段边界（24px 大字）：整段对字库黄金参考；起点左侧 2 列必须无字 --");
        page = 2'd0; uimode = 2'd0; estop_i = 1'b0; sigok_i = 1'b1; armhint_i = 1'b0;
        menusel = 2'd3;
        repeat (4) @(posedge pixel_clk);
        dump_row(LG_MENU_Y0, 24'hE5E7EB);      // 第 0 行点亮序列（诊断图案是否整体错位）
        dump_row(LG_MENU_Y0 + 4, 24'hE5E7EB);
        check_seg_lg(LG_MENU_Y0, LG_XW_SLOT,
                     UI_TXT_MENU_CAMERA_LEN, UI_TXT_MENU_CAMERA_ADDR,
                     24'hE5E7EB, "menu_item0_boundary");

        // ---------------- 10. ★ 段边界回归：数据页右侧列表 + 车头偏航角 ----------------
        //   2026-10-17 车图改版后：列表标签 x=800（组 50）、列表数值 x=880（组 55）、
        //   车头偏航角 x=560（组 35）。三者都是 16 px 小字，判据同前。
        $display("-- 10. 数据页边界：右侧列表标签 + 列表数值 + 车头偏航角 黄金参考 --");
        page = 2'd2;
        repeat (4) @(posedge pixel_clk);
        check_seg_rom(208, PX_LBL_SLOT,
                      UI_TXT_LBL_VEL_LEN, UI_TXT_LBL_VEL_ADDR,
                      24'hFFFFFF, "data_label_boundary");
        check_seg_ram(208, PX_VAL_SLOT, 4, dyn_rdata, 24'h7CFC98, "data_number_boundary");
        check_seg_ram(160, PX_YAW_SLOT, 4, dyn_rdata, 24'h7CFC98, "data_yaw_boundary");

        // ---------------- 11. ★ 数据页车图：黄金参考逐像素 + 不得越界 ----------------
        //   需求方明确"不要太大导致挤占其他数据的空间"，所以除了形状对不对，
        //   还必须断言"包围盒之外一个白色像素都没有"。
        $display("-- 11. 车图：逐像素对 SVG 黄金参考；包围盒外不得有白色像素 --");
        scan_car;
        if (err_car == 0) begin
            $display("  pass  car_shape: %0d px 与 SVG 黄金参考完全一致", cnt_car);
        end else begin
            errors = errors + 1;
            $display("  FAIL  car_shape: %0d 像素与 SVG 黄金参考不符", err_car);
        end
        if (cnt_car > 500) begin
            $display("  pass  car_present: %0d px", cnt_car);
        end else begin
            errors = errors + 1;
            $display("  FAIL  car_present: 只有 %0d px（车图没画出来）", cnt_car);
        end
        if (cnt_car_spill == 0) begin
            $display("  pass  car_bounds: 包围盒外白色像素 = 0");
        end else begin
            errors = errors + 1;
            $display("  FAIL  car_bounds: 包围盒外有 %0d 个白色像素（越界，会挤占数据区）",
                     cnt_car_spill);
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
