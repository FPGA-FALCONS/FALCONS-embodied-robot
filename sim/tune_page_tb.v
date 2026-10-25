`timescale 1ns/1ps
// ============================================================================
// tune_page_tb.v —— 参数整定页（ui_render 的 PG_TUNE 分支）渲染测试
//
//  只验证「画出来了、位置对、颜色对、不串页」，不校验字形（字形由
//  ui_render_tb 的字库黄金参考负责）。做法与 ui_render_tb 同构：
//  模拟像素扫描，在指定窗口内按颜色计数。
//
//  页面几何（rtl\ui\ui_render.v:126-134）：
//    TUNE_Y0=72 TUNE_STEP=36 TUNE_TDY=6 TUNE_VDY=10
//    标签 x=240（24px 大字）  数值 x=384（16px 数字）  单位 x=480
//    高亮条 x≥224，y∈[72+36·sel, +36)
//
//  覆盖：
//    G1 菜单第 6 项「参数整定」的存在与选中/未选中配色
//    G2 参数页标题
//    G3 11 行标签逐行有白字
//    G4 11 行数值逐行有绿字
//    G5 单位只在第 0..4 行出现（5..10 无量纲）
//    G6 高亮条随 i_tune_sel 移动（0 / 5 / 10），且互不重叠
//    G7 参数页不画「页面开发中」占位提示
//    G8 非参数页时高亮条不出现（i_page 门控生效）
// ============================================================================
module tune_page_tb;

    reg pixel_clk = 1'b0;
    always #10 pixel_clk = ~pixel_clk;      // 50 MHz

    reg         rst_n = 1'b0;
    reg  [11:0] px = 12'd0, py = 12'd0;
    reg         preq = 1'b0, vact = 1'b0;

    reg  [2:0]  page    = 3'd5;
    reg  [1:0]  uimode  = 2'd0;
    reg         estop_i = 1'b0, sigok_i = 1'b1, armhint_i = 1'b0;
    reg  [2:0]  menusel = 3'd5;
    reg         navrun_i = 1'b0, navpz_i = 1'b0, menudim_i = 1'b0;
    reg  [2:0]  auxsel_i = 3'd0;
    reg  [9:0]  carpx_i  = 10'd412, carpy_i = 10'd300;
    reg  [9:0]  curpx_i  = 10'd412, curpy_i = 10'd300;
    reg  [159:0] ptpx_i  = 160'd0,  ptpy_i  = 160'd0;
    reg  [15:0]  ptv_i   = 16'd0;
    reg          imufail_i = 1'b0;
    reg  [3:0]   tsel_i = 4'd0;
    // 参数页「编辑态」（= falcons_top 的 tune_active）。默认 1：本 tb 早先的用例
    //   都假定"已在参数页且可编辑"；G9 专门测未进编辑态那一态。
    reg          tact_i = 1'b1;

    wire [23:0] rgb;
    wire [6:0]  dyn_raddr;
    reg  [7:0]  dyn_rdata = 8'd24;          // 字库索引 24 = '8'（合法字形，能点亮）

    // 与实际扫描器一致：像素坐标在沿后推进，再由下一沿消费。
    reg [11:0] scan_px = 0, scan_py = 0;
    always @(posedge pixel_clk) begin
        scan_px <= px;
        scan_py <= py;
    end

    ui_render dut (
        .i_map_pixel(1'b0), .i_route_pixel(1'b0), .i_obstacle_state(3'd0),
        .i_pixel_clk    (pixel_clk),
        .i_reset_n      (rst_n),
        .i_pixel_x      (scan_px),
        .i_pixel_y      (scan_py),
        .i_pixel_req    (preq),
        .i_video_active (vact),
        .o_pixel_rgb888 (rgb),
        .i_page         (page),
        .i_ui_mode      (uimode),
        .i_estop        (estop_i),
        .i_signal_ok    (sigok_i),
        .i_arm_hint     (armhint_i),
        .i_menu_sel     (menusel),
        .i_nav_run      (navrun_i),
        .i_nav_paused   (navpz_i),
        .i_menu_dim     (menudim_i),
        .i_aux_sel      (auxsel_i),
        .i_car_px       (carpx_i),
        .i_car_py       (carpy_i),
        .i_cur_px       (curpx_i),
        .i_cur_py       (curpy_i),
        .i_pt_px        (ptpx_i),
        .i_pt_py        (ptpy_i),
        .i_pt_v         (ptv_i),
        .o_dyn_raddr    (dyn_raddr),
        .i_dyn_rdata    (dyn_rdata),
        .i_imu_fail     (imufail_i),
        .i_tune_sel     (tsel_i),
        .i_tune_active  (tact_i)
    );

    // 布局常量（与 ui_render.v 一致）
    localparam integer TUNE_Y0   = 72;
    localparam integer TUNE_STEP = 36;
    localparam integer TUNE_TDY  = 6;
    localparam integer TUNE_VDY  = 10;
    localparam integer LBL_X0    = 240;
    localparam integer LBL_X1    = 359;
    localparam integer VAL_X0    = 384;
    localparam integer VAL_X1    = 463;
    localparam integer UNI_X0    = 480;
    localparam integer UNI_X1    = 543;
    localparam integer HL_X0     = 224;
    localparam integer HL_X1     = 655;   // = TUNE_HL_X1-1（ui_render.v）

    localparam [23:0] C_WHITE  = 24'hFFFFFF;
    localparam [23:0] C_GREEN  = 24'h7CFC98;
    localparam [23:0] C_BLUE   = 24'h1D4ED8;
    localparam [23:0] C_YELLOW = 24'hFFE066;
    localparam [23:0] C_GRAY   = 24'hE5E7EB;
    // ★ 参数页菜单条变暗后的三色（2026-10-20 需求方要求：进页后菜单栏变暗）
    localparam [23:0] C_YELLOW_DIM = 24'h8A7A2E;
    localparam [23:0] C_GRAY_DIM   = 24'h5A6170;
    localparam [23:0] C_MENUBG_DIM = 24'h0B1019;
    localparam [23:0] C_MENUBG = 24'h101828;      // 正常亮度菜单底

    integer checks = 0, errors = 0;
    integer wcnt;           // scan_color 的结果
    integer x, y;
    integer i;
    integer row_lbl [0:13];
    integer row_val [0:13];

    task scan_color;
        input integer x0;
        input integer x1;
        input integer y0;
        input integer y1;
        input [23:0]  col;
        begin
            wcnt = 0;
            for (y = y0; y <= y1; y = y + 1) begin
                // 局部跳读先走消隐，再从窗口前四个像素开始预读。
                px = 12'd1100; py = y[11:0];
                vact = 1'b0; preq = 1'b0;
                repeat (5) @(posedge pixel_clk);
                #1;
                for (x = x0 - 4; x <= x1; x = x + 1) begin
                    px = x[11:0]; py = y[11:0];
                    vact = 1'b1; preq = 1'b1;
                    @(posedge pixel_clk);
                    #1;
                    if (x >= x0 && rgb == col) wcnt = wcnt + 1;
                end
            end
            vact = 1'b0; preq = 1'b0;
        end
    endtask

    task chk_int;
        input [127:0] name;
        input integer got;
        input integer exp;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL %0s: got %0d, expect %0d", name, got, exp);
            end else begin
                $display("  PASS: %0s", name);
            end
        end
    endtask

    task chk_pos;
        input [127:0] name;
        input integer got;
        begin
            checks = checks + 1;
            if (got <= 0) begin
                errors = errors + 1;
                $display("  FAIL %0s: %0d (expect > 0)", name, got);
            end else begin
                $display("  PASS: %0s (%0d px)", name, got);
            end
        end
    endtask

    task chk_zero;
        input [127:0] name;
        input integer got;
        begin
            checks = checks + 1;
            if (got != 0) begin
                errors = errors + 1;
                $display("  FAIL %0s: %0d (expect 0)", name, got);
            end else begin
                $display("  PASS: %0s", name);
            end
        end
    endtask

    initial begin
        $display("=== tune_page_tb ===");
        rst_n = 1'b0;
        repeat (10) @(posedge pixel_clk);
        rst_n = 1'b1;
        repeat (10) @(posedge pixel_clk);

        // ------------------------------------------------------------------
        // G1 菜单第 6 项「参数整定」
        //   项间距 72，第 i 项文字 y = 48 + 72i → 第 6 项（i=5）y = 408
        // ------------------------------------------------------------------
        page = 3'd0; menusel = 3'd5;
        repeat (5) @(posedge pixel_clk);
        scan_color(24, 207, 408, 431, C_YELLOW);
        chk_pos("g1_menu_item6_selected_yellow", wcnt);
        scan_color(24, 207, 408, 431, C_GRAY);
        chk_zero("g1_menu_item6_not_gray_when_sel", wcnt);

        menusel = 3'd0;
        repeat (5) @(posedge pixel_clk);
        scan_color(24, 207, 408, 431, C_GRAY);
        chk_pos("g1_menu_item6_gray_when_unsel", wcnt);
        scan_color(24, 207, 408, 431, C_YELLOW);
        chk_zero("g1_menu_item6_not_yellow_when_unsel", wcnt);

        // ------------------------------------------------------------------
        // G1b 参数页：左侧菜单条**整体变暗**（不进页时是 FFE066/E5E7EB/101828）
        //   变暗后：选中项 8A7A2E、未选中 5A6170、底色 0B1019
        // ------------------------------------------------------------------
        page = 3'd5; menusel = 3'd5;
        repeat (5) @(posedge pixel_clk);
        scan_color(24, 207, 408, 431, C_YELLOW_DIM);
        chk_pos("g1b_tune_menu_dim_sel", wcnt);
        scan_color(24, 207, 408, 431, C_YELLOW);
        chk_zero("g1b_tune_menu_not_bright_sel", wcnt);
        scan_color(24, 207, 48, 71, C_GRAY_DIM);
        chk_pos("g1b_tune_menu_dim_unsel", wcnt);
        scan_color(24, 207, 48, 71, C_GRAY);
        chk_zero("g1b_tune_menu_not_bright_unsel", wcnt);
        scan_color(24, 200, 496, 532, C_MENUBG_DIM);
        chk_pos("g1b_tune_menu_dim_bg", wcnt);

        // ------------------------------------------------------------------
        // G2 参数页标题「参数整定」：4 字 × 24 = 96 px，x 240..335，y 24..47
        // ------------------------------------------------------------------
        page = 3'd5; menusel = 3'd5; tsel_i = 4'd0;
        repeat (5) @(posedge pixel_clk);
        scan_color(LBL_X0, LBL_X0 + 95, 24, 47, C_WHITE);
        chk_pos("g2_tune_title", wcnt);

        // ------------------------------------------------------------------
        // G3 12 行标签逐行有白字
        // ------------------------------------------------------------------
        for (i = 0; i < 14; i = i + 1) begin
            scan_color(LBL_X0, LBL_X1,
                       TUNE_Y0 + i * TUNE_STEP + TUNE_TDY,
                       TUNE_Y0 + i * TUNE_STEP + TUNE_TDY + 23,
                       C_WHITE);
            row_lbl[i] = wcnt;
        end
        chk_pos("g3_label_row0",  row_lbl[0]);
        chk_pos("g3_label_row1",  row_lbl[1]);
        chk_pos("g3_label_row2",  row_lbl[2]);
        chk_pos("g3_label_row3",  row_lbl[3]);
        chk_pos("g3_label_row4",  row_lbl[4]);
        chk_pos("g3_label_row5",  row_lbl[5]);
        chk_pos("g3_label_row6",  row_lbl[6]);
        chk_pos("g3_label_row7",  row_lbl[7]);
        chk_pos("g3_label_row8",  row_lbl[8]);
        chk_pos("g3_label_row9",  row_lbl[9]);
        chk_pos("g3_label_row10", row_lbl[10]);
        chk_pos("g3_label_row11", row_lbl[11]);
        chk_pos("g3_label_row12", row_lbl[12]);
        chk_pos("g3_label_row13", row_lbl[13]);

        // ------------------------------------------------------------------
        // G4 12 行数值逐行有绿字（5 字符 × 16 = 80 px）
        //   ★ 2026-10-20 改：原先只扫 VDY..VDY+15（16 行），而 font_num 是
        //     16 宽 × 24 高 —— 正好漏掉了「下部被截断」这个上板 bug。
        //     这里扫满 24 行，截断会立刻失败。
        // ------------------------------------------------------------------
        for (i = 0; i < 14; i = i + 1) begin
            scan_color(VAL_X0, VAL_X1,
                       TUNE_Y0 + i * TUNE_STEP + TUNE_VDY,
                       TUNE_Y0 + i * TUNE_STEP + TUNE_VDY + 23,
                       C_GREEN);
            row_val[i] = wcnt;
        end
        chk_pos("g4_value_row0",  row_val[0]);
        chk_pos("g4_value_row1",  row_val[1]);
        chk_pos("g4_value_row2",  row_val[2]);
        chk_pos("g4_value_row3",  row_val[3]);
        chk_pos("g4_value_row4",  row_val[4]);
        chk_pos("g4_value_row5",  row_val[5]);
        chk_pos("g4_value_row6",  row_val[6]);
        chk_pos("g4_value_row7",  row_val[7]);
        chk_pos("g4_value_row8",  row_val[8]);
        chk_pos("g4_value_row9",  row_val[9]);
        chk_pos("g4_value_row10", row_val[10]);
        chk_pos("g4_value_row11", row_val[11]);
        chk_pos("g4_value_row12", row_val[12]);
        chk_pos("g4_value_row13", row_val[13]);

        // ------------------------------------------------------------------
        // G5 单位：第 0..4 行有，第 5..10 行无
        //   ★ ROM 源（静态文案）在 :1028 被固定着白色 24'hFFFFFF，
        //     这是既有视觉规则（数据页的 "/°" "/rpm" 后缀同为白色），不是缺陷。
        //     故单位段判 C_WHITE；数值段（RAM 源）才是 C_GREEN。
        // ------------------------------------------------------------------
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 0 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 0 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_pos("g5_unit_row0_mms", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 1 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 1 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_pos("g5_unit_row1_mm", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 2 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 2 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_pos("g5_unit_row2_deg", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 3 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 3 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_pos("g5_unit_row3_deg", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 4 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 4 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_pos("g5_unit_row4_degs", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 5 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 5 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_zero("g5_unit_row5_none", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 10 * TUNE_STEP + TUNE_VDY,
                   TUNE_Y0 + 10 * TUNE_STEP + TUNE_VDY + 15, C_WHITE);
        chk_zero("g5_unit_row10_none", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 12*TUNE_STEP + 6,
                   TUNE_Y0 + 12*TUNE_STEP + 29, C_WHITE);
        chk_pos("front_cells_unit", wcnt);
        scan_color(UNI_X0, UNI_X1, TUNE_Y0 + 13*TUNE_STEP + 6,
                   TUNE_Y0 + 13*TUNE_STEP + 29, C_WHITE);
        chk_pos("width_mm_unit", wcnt);
        tsel_i = 13;
        repeat (5) @(posedge pixel_clk);
        scan_color(HL_X0, HL_X1, 540, 575, C_BLUE);
        chk_pos("width_last_row_highlight", wcnt);
        // ------------------------------------------------------------------
        // G6 高亮条随 i_tune_sel 移动
        // ------------------------------------------------------------------
        tsel_i = 4'd0;
        repeat (5) @(posedge pixel_clk);
        scan_color(HL_X0, HL_X1, 72, 107, C_BLUE);
        chk_pos("g6_hl_sel0", wcnt);
        scan_color(HL_X0, HL_X1, 252, 287, C_BLUE);
        chk_zero("g6_hl_sel0_other_row_empty", wcnt);

        tsel_i = 4'd5;
        repeat (5) @(posedge pixel_clk);
        scan_color(HL_X0, HL_X1, 252, 287, C_BLUE);
        chk_pos("g6_hl_sel5", wcnt);
        scan_color(HL_X0, HL_X1, 72, 107, C_BLUE);
        chk_zero("g6_hl_sel5_row0_empty", wcnt);

        tsel_i = 4'd10;
        repeat (5) @(posedge pixel_clk);
        scan_color(HL_X0, HL_X1, 432, 467, C_BLUE);
        chk_pos("g6_hl_sel10", wcnt);
        scan_color(HL_X0, HL_X1, 252, 287, C_BLUE);
        chk_zero("g6_hl_sel10_row5_empty", wcnt);

        // 高亮条左界 224（不应侵入左侧菜单条 x<208）
        tsel_i = 4'd0;
        repeat (5) @(posedge pixel_clk);
        scan_color(0, 207, 72, 107, C_BLUE);
        // 左侧菜单条在该 y 区间本来就有"第 1 项"的高亮带（menusel=5 时没有），
        // 这里 menusel=5 → 菜单条高亮在 y 420..468，与 72..107 不重叠
        chk_zero("g6_hl_left_edge_clear", wcnt);

        // 高亮条右界 656：656..1023 内不应有蓝条（单位段最右 543，留 113 px 余量）
        scan_color(656, 1023, 72, 107, C_BLUE);
        chk_zero("g6_hl_right_edge_clear", wcnt);

        // ------------------------------------------------------------------
        // G7 参数页不画「页面开发中」占位提示
        //   占位提示几何：x 216..335，y 264..287（24px 大字，白色）。
        //   参数页行 5 的标签起点 x=240 会侵入该窗口，所以只取
        //   x 216..239（标签左侧留白）判定 —— 有白像素就是占位提示。
        // ------------------------------------------------------------------
        scan_color(216, 239, 264, 287, C_WHITE);
        chk_zero("g7_no_placeholder_text", wcnt);

        // ------------------------------------------------------------------
        // G8 非参数页时高亮条不出现（i_page 门控）
        // ------------------------------------------------------------------
        page = 3'd0;
        repeat (5) @(posedge pixel_clk);
        scan_color(HL_X0, HL_X1, 252, 287, C_BLUE);
        chk_zero("g8_hl_hidden_on_data_page", wcnt);
        scan_color(HL_X0, HL_X1, 72, 107, C_BLUE);
        chk_zero("g8_hl_hidden_row0_data_page", wcnt);

        // 参数页标题也不应在数据页出现（y 24..47 x 240..335 白色）
        scan_color(LBL_X0, LBL_X0 + 95, 24, 47, C_WHITE);
        chk_zero("g8_title_hidden_on_data_page", wcnt);

        // ------------------------------------------------------------------
        // G9 参数页但**未进编辑态**（= 按返回退出编辑后那一态，与导航页退出 nav_ui 同构）
        //   page 仍停在参数页 ⇒ 标题与 12 行内容照常显示；
        //   但菜单栏恢复正常亮度、选中行高亮条消失。
        //   ★ 若把 i_tune_active 从这两处门控里去掉，本组第 2/4 条会立刻失败。
        // ------------------------------------------------------------------
        page = 3'd5; menusel = 3'd5; tsel_i = 4'd0; tact_i = 1'b0;
        repeat (5) @(posedge pixel_clk);
        scan_color(24, 207, 408, 431, C_YELLOW);
        chk_pos("g9_idle_menu_bright_sel", wcnt);
        scan_color(24, 207, 408, 431, C_YELLOW_DIM);
        chk_zero("g9_idle_menu_not_dim_sel", wcnt);
        scan_color(24, 200, 496, 532, C_MENUBG);
        chk_pos("g9_idle_menu_bg_bright", wcnt);
        scan_color(HL_X0, HL_X1, 72, 107, C_BLUE);
        chk_zero("g9_idle_no_hl_band", wcnt);
        scan_color(LBL_X0, LBL_X0 + 95, 24, 47, C_WHITE);
        chk_pos("g9_idle_title_still_shown", wcnt);
        tact_i = 1'b1;

        // ------------------------------------------------------------------
        $display("");
        if (errors == 0)
            $display("PASS: tune_page_tb %0d checks, 0 errors", checks);
        else
            $display("FAIL: tune_page_tb %0d checks, %0d errors", checks, errors);
        $finish;
    end

    initial begin
        #50000000;          // 50 ms 上限
        $display("TIMEOUT: tune_page_tb");
        $finish;
    end

endmodule
