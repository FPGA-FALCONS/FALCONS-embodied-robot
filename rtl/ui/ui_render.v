//////////////////////////////////////////////////////////////////////////////////
// 模块：ui_render —— 1024x600 中文界面渲染（菜单 + 四页 + 状态栏 + 实时数值）
//
// 【工作方式】逐像素叠加，不用帧缓存
//   界面由「点阵字符」拼成，布局强制一条规则：
//     ★ 所有文本起点 x（像素）与 y 都是【字组边长】的倍数
//   于是每个字符恰好落在一个"字组"内：
//     字组号 slot = x / CELL；格内位 = CELL-1 - (x % CELL)（最高位 = 最左像素）
//     点阵行 row = y - 段基址（由段描述符直接给出，避免除法）
//
// ★★ 全界面统一 24x24（2026-10-17 晚，需求方："菜单/档位字太小" → "数据页内容也太小"）★★
//   两轮要求合并后，整个界面都用 24x24 大字（左侧菜单条 + 右侧数据页 + 占位提示），
//   字库见 tools/gen_font_rom.py 生成的 font_rom_lg.v。
//   16x16 的小字通路**保留但不再有任何段**（译码全 0，综合器会把它优化掉），
//   这样将来若要把某块内容改回小字，只要往 sm_seg 里加段即可，不必重写通路。
//
// 【提前 2 拍取数】字符索引（ROM/RAM）与点阵行（font_rom）各延迟 1 拍，共 2 拍。
//   做法：始终请求「x+2 所在的字组」——数据 2 拍后有效时 x 已推进到 x+2，仍在同一字组内。
//   ★ 坑（已踩过，见工作记录 §30.5）：**位选 bit_sel 必须用当前 x（组合）**，
//     只有"段门控"随数据打拍。位选若也打拍，整屏文字会整体平移。
//
// 【文本段（seg）】同一条扫描行可能有多段文字，所以译码结果是若干段描述符，
//   再按当前 slot 选出命中的那一段。
//   大字段 40 位：{valid, src, rom_addr[11:0], ram_addr[5:0], len[5:0], xw[5:0], row[4:0]}
//     src = 0 → 静态文案 ROM；src = 1 → 动态字符 RAM（num_text 生成的实时数值）
//   小字段 32 位：{valid, src, rom_addr[11:0], ram_addr[5:0], len[5:0], xw[5:0]}（保留不用）
//
// 【四页】0 摄像头 / 1 雷达 / 2 运行数据 / 3 路径点导航
//   页 0/1/3 显示占位提示；页 2 显示车图 + 实时数值。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module ui_render #(
    parameter integer SCREEN_W  = 1024,
    parameter integer SCREEN_H  = 600,
    // ---- 左侧菜单条 ----
    parameter integer MENU_W    = 208,      // 左侧菜单条宽度（★ 需求方要求不变）
    parameter integer LG_CELL   = 24,       // 字模边长（全界面统一 24）
    parameter integer LG_XW     = 1,        // 菜单文字起始字组（x = 24）
    parameter integer MENU_Y0   = 72,       // 第一项文字 y 起点（须为 LG_CELL 的倍数）
    parameter integer MENU_STEP = 72,       // 项间距（保持文字:行距 = 1:3）
    parameter integer STATUS_Y  = 552,      // 状态栏文字 y 起点（须为 LG_CELL 的倍数）
    // ---- 右侧内容区 ----
    parameter integer PAGE_Y    = 264,      // 占位提示 y 起点（LG_CELL 的倍数）
    parameter integer PAGEXW    = 9         // 内容区文字起始字组（x = 216）
)(
    // ---- HDMI 像素域 ----
    input  wire        i_pixel_clk,
    input  wire        i_reset_n,
    input  wire [11:0] i_pixel_x,
    input  wire [11:0] i_pixel_y,
    input  wire        i_pixel_req,
    input  wire        i_video_active,
    output wire [23:0] o_pixel_rgb888,

    // ---- 界面状态（sys 域送来，已在顶层两级同步）----
    input  wire [1:0]  i_page,          // 0=摄像头 1=雷达 2=数据 3=导航
    input  wire [1:0]  i_ui_mode,       // 0=手动接管 1=屏幕光标
    input  wire        i_estop,         // 1 = 急停
    input  wire        i_signal_ok,     // 1 = 遥控信号有效
    input  wire        i_arm_hint,      // 1 = 想启动但油门不在中位
    input  wire [1:0]  i_menu_sel,      // 菜单【高亮选中项】

    // ---- 动态数值字符 RAM 读端口（pixel 域）----
    output wire [5:0]  o_dyn_raddr,
    input  wire [7:0]  i_dyn_rdata
);

    // UI 文案索引（localparam；由 tools/gen_font_rom.py 生成，与本文件同目录）
    `include "ui_text_index.vh"

    localparam integer SM_CELL = 16;        // 小字点阵边长（保留通路，当前无段）

    // ------------------------------------------------------------------
    // 文案取用（function：纯组合，综合稳）  {addr[11:0], len[5:0]}
    // ------------------------------------------------------------------
    function [17:0] menu_text;
        input [1:0] idx;
        begin
            case (idx)
                2'd0:    menu_text = {UI_TXT_MENU_CAMERA_ADDR,  UI_TXT_MENU_CAMERA_LEN};
                2'd1:    menu_text = {UI_TXT_MENU_RADAR_ADDR,   UI_TXT_MENU_RADAR_LEN};
                2'd2:    menu_text = {UI_TXT_MENU_VEHICLE_ADDR, UI_TXT_MENU_VEHICLE_LEN};
                default: menu_text = {UI_TXT_MENU_NAV_ADDR,     UI_TXT_MENU_NAV_LEN};
            endcase
        end
    endfunction

    // 数据页右侧列表的 4 个标签
    //   0 角速度 / 1 中心速度 / 2 航位x / 3 航位y
    function [17:0] label_text;
        input [1:0] idx;
        begin
            case (idx)
                2'd0:    label_text = {UI_TXT_LBL_GYRO_ADDR,   UI_TXT_LBL_GYRO_LEN};
                2'd1:    label_text = {UI_TXT_LBL_VEL_ADDR,    UI_TXT_LBL_VEL_LEN};
                2'd2:    label_text = {UI_TXT_LBL_POSE_X_ADDR, UI_TXT_LBL_POSE_X_LEN};
                default: label_text = {UI_TXT_LBL_POSE_Y_ADDR, UI_TXT_LBL_POSE_Y_LEN};
            endcase
        end
    endfunction

    reg [17:0] status_text;
    always @(*) begin
        if (i_estop)                status_text = {UI_TXT_ST_ESTOP_ADDR,       UI_TXT_ST_ESTOP_LEN};
        else if (!i_signal_ok)      status_text = {UI_TXT_ST_NOSIGNAL_ADDR,    UI_TXT_ST_NOSIGNAL_LEN};
        else if (i_arm_hint)        status_text = {UI_TXT_ST_ARM_DENY_ADDR,    UI_TXT_ST_ARM_DENY_LEN};
        else if (i_ui_mode == 2'd0) status_text = {UI_TXT_ST_MANUAL_ADDR,      UI_TXT_ST_MANUAL_LEN};
        else                        status_text = {UI_TXT_ST_CURSOR_ADDR,      UI_TXT_ST_CURSOR_LEN};
    end

    reg [17:0] placeholder_text;
    always @(*) begin
        case (i_page)
            2'd0:    placeholder_text = {UI_TXT_PLACEHOLDER_CAM_ADDR,   UI_TXT_PLACEHOLDER_CAM_LEN};
            2'd1:    placeholder_text = {UI_TXT_PLACEHOLDER_RADAR_ADDR, UI_TXT_PLACEHOLDER_RADAR_LEN};
            2'd3:    placeholder_text = {UI_TXT_PLACEHOLDER_NAV_ADDR,   UI_TXT_PLACEHOLDER_NAV_LEN};
            default: placeholder_text = 18'd0;
        endcase
    end

    // ------------------------------------------------------------------
    // 段描述符打包 —— 大字（24 px 组，全界面通用）
    //   位段（从左到右拼，务必与下方 lg_sel[...] 的取位一致）：
    //     [39]valid | [38]src | [37:26]rom_addr | [25:20]ram_addr
    //     | [19:14]len | [13:8]xw | [7:3]row | [2:0]保留
    //   ★ row 直接由译码给出（y − 该项基址），省掉 y % 24 的除法；
    //     必须用【完整位宽】相减后再取值，写成 y[4:0] − 基址 会因截断算错。
    // ------------------------------------------------------------------
    //   lg 通路只做静态文案：段 30 位，无 ram 字段（数值走 sm 通路）
    function [29:0] seg_lg_rom;
        input [17:0] txt;      // {addr, len}
        input [5:0]  xw;
        input [4:0]  row;
        begin
            seg_lg_rom = {1'b1, txt[17:6], txt[5:0], xw, row};
        end
    endfunction

    //   sm 通路只做动态数值：段 24 位，无 rom 字段（省掉一套字符索引 ROM）
    function [23:0] seg_sm_ram;
        input [5:0] base;      // 动态 RAM 起始地址（字段号 × 4）
        input [5:0] len;       // 字符数
        input [5:0] xw;
        input [4:0] row;
        begin
            seg_sm_ram = {1'b1, base, len, xw, row};
        end
    endfunction

    // ---------------- 数据页各元素的列位置（字组号，24 px/组）----------------
    //   画面总宽 1024，菜单条占 0..207，内容区 208..1023。
    //   车图左移后：轮速贴两侧轮子，4 行列表靠右。
    //     x = 256（组 16） 左侧轮速      4 字符 = 64 px，贴左轮外侧 17 px
    //     x = 336..616     车图（280×322，SVG 的 1.4 倍）
    //     x = 648（组 27） 右侧轮速
    //     x = 792（组 33） 列表标签      4 字符 = 96 px（与右侧轮速拉开 48 px）
    //     x = 912（组 38） 列表数值
    //     x = 432（组 18） 车头偏航角（96 px，中心 480 ≈ 车体中心 476）
    //   ★ 数值走【16 px 组】的紧凑数字通路（font_num，16x24），标签走【24 px 组】的大字通路。
    //     这样数字间距从 ~11 px 收到 ~3 px，而字高仍是 24，与中文标签等高。
    localparam [5:0] PX_LBL = 6'd33;                    // 标签列（24 组）：x=792
    //   ★ 四个轮速要"贴住轮子"（2026-10-17 需求方："转速数字与轮子的间距靠近一点"）：
    //     轮子外沿 左 336 / 右 624（车图 336..624，缩放宽 288 使两端都 16 对齐），
    //     数字宽 4×16 = 64：
    //       左起 256 → 数字右边 319，离左轮外沿(352) 32 px
    //       右起 672 → 数字左边 672，离右轮外沿(641) 31 px   ← 两侧基本一致
    //     （2026-10-18：车图右移 16 px 配合文本框；间距由 16 加到 ~32，呼吸感更足。）
    localparam [5:0] PX_YAW_N = 6'd29,                  // 偏航角（16 组）：x=464，中心 496
                     PX_WL_N  = 6'd16,                  // 左轮速（16 组）：x=256
                     PX_WR_N  = 6'd42,                  // 右轮速（16 组）：x=672
                     PX_VAL_N = 6'd57;                  // 列表数值（16 组）：x=912

    // ---------------- 大字段数组：菜单 4 项 + 状态栏 + 数据页 + 占位 ----------------
    //   同一条扫描行最多 4 段（数据页 y=240 那行：左轮速 / 右前轮速 / 角速度标签 / 角速度数值）
    reg [29:0] lg_seg [0:5];
    integer    li, mi;
    always @(*) begin
        for (li = 0; li < 6; li = li + 1) lg_seg[li] = 30'd0;

        // ---- 左侧菜单 4 项 ----
        for (mi = 0; mi < 4; mi = mi + 1) begin
            if ((i_pixel_y >= (MENU_Y0 + mi * MENU_STEP)) &&
                (i_pixel_y <  (MENU_Y0 + mi * MENU_STEP + LG_CELL))) begin
                lg_seg[0] = seg_lg_rom(menu_text(mi[1:0]), LG_XW[5:0],
                                       (i_pixel_y - (MENU_Y0 + mi * MENU_STEP)));
            end
        end
        // ---- 状态栏档位文字 ----
        if ((i_pixel_y >= STATUS_Y) && (i_pixel_y < STATUS_Y + LG_CELL)) begin
            lg_seg[0] = seg_lg_rom(status_text, LG_XW[5:0], (i_pixel_y - STATUS_Y));
        end

        // ---- 数据页（页 2）：车图居中 + 轮旁转速 + 车头偏航角 + 右侧列表 ----
        //   依据需求原文 §29："菜单右侧画面中心为一辆小车俯视示意图，小车前端实时显示
        //   车辆朝向（航向角），四个轮子旁实时显示每个轮子转速，右侧单独竖向排列显示
        //   角速度、中心速度、航位等信息。"
        //   ★ 字段号（num_text，每字段 4 字符，基址 = 字段号 × 4）：
        //     0 航向角 / 1=A右后 / 2=B左后 / 3=C右前 / 4=D左前 / 5 角速度 / 6 中心速度 / 7,8 航位 x,y
        //   ★ 轮位映射（需求方 2026-10-17 确认真源）：A=右后 B=左后 C=右前 D=左前
        //   ★ 数据页的中文标签在这里（24 px 组）；数值在下面的 sm_seg（16 px 组）
        //   ★★ 必须用【独立槽位】lg_seg[1..4]，不能挤进 lg_seg[0]：
        //      菜单第 4 项「路径点导航」在 y=288..311，而「中心速度」行**正好也是** y=288..311。
        //      两者若共用 lg_seg[0]，后者会把菜单项覆盖掉 —— 表现为"选中数据页后第 4 项消失"。
        //      分开后同一扫描行两段并存，靠 slot 范围区分（菜单 xw=1，标签 xw=33）。
        if (i_page == 2'd2) begin
            if ((i_pixel_y >= 240) && (i_pixel_y < 264))
                lg_seg[1] = seg_lg_rom(label_text(2'd0), PX_LBL, (i_pixel_y - 12'd240));
            if ((i_pixel_y >= 288) && (i_pixel_y < 312))
                lg_seg[2] = seg_lg_rom(label_text(2'd1), PX_LBL, (i_pixel_y - 12'd288));
            if ((i_pixel_y >= 336) && (i_pixel_y < 360))
                lg_seg[3] = seg_lg_rom(label_text(2'd2), PX_LBL, (i_pixel_y - 12'd336));
            if ((i_pixel_y >= 384) && (i_pixel_y < 408))
                lg_seg[4] = seg_lg_rom(label_text(2'd3), PX_LBL, (i_pixel_y - 12'd384));
        end

        // ---- 占位提示（页 0/1/3）----
        if ((i_page != 2'd2) && (i_pixel_y >= PAGE_Y) && (i_pixel_y < PAGE_Y + LG_CELL) &&
            (placeholder_text[5:0] != 6'd0)) begin
            lg_seg[0] = seg_lg_rom(placeholder_text, PAGEXW[5:0], (i_pixel_y - PAGE_Y));
        end
    end

    // ==================================================================
    // 提前 2 拍取数：请求 (x+2) 所在的字组
    // ==================================================================
    wire [11:0] x_look = i_pixel_x + 12'd2;

    // ---- 24 px/组 不是 2 的幂：一次常数除法 + 一次乘法回算格内列号 ----
    //   ★ 用【无符号 12 位常量】参与运算：直接写 LG_CELL（integer，有符号 32 位）会引入
    //     有符号/无符号混算；而 `slot * LG_CELL[5:0]` 更危险 —— 两个 6 位相乘截成 6 位，
    //     24*8=192 直接溢出成错值。
    localparam [11:0] LG_CELL_N = LG_CELL;
    wire [5:0]  lg_slot     = x_look / LG_CELL_N;
    wire [11:0] lg_col_look = x_look - lg_slot * LG_CELL_N;   // 对应 x_look 的格内列号

    // ★ 位选必须用【当前 pixel_x】的列号：col(x) = (x_look − 2) mod 24，
    //   用条件减法避免再引一个除法器（x_look 落在组内前 2 列时，x 属于上一组的末尾）。
    wire [11:0] lg_col      = (lg_col_look >= 12'd2) ? (lg_col_look - 12'd2)
                                                    : (lg_col_look + 12'd22);

    // ---- 选段：同一条扫描行可能有多段，取命中当前 slot 的那一段 ----
    reg [29:0] lg_sel;
    always @(*) begin
        lg_sel = 30'd0;
        for (li = 0; li < 6; li = li + 1) begin
            if (lg_seg[li][29] &&
                (lg_slot >= lg_seg[li][10:5]) &&
                (lg_slot <  (lg_seg[li][10:5] + lg_seg[li][16:11]))) begin
                lg_sel = lg_seg[li];
            end
        end
    end

    wire        lg_in_text = lg_sel[29];
    wire [5:0]  lg_off     = lg_slot - lg_sel[10:5];
    wire [11:0] lg_rom_rd  = lg_sel[28:17] + lg_off;

    // 一级：字符索引（静态 ROM / 动态 RAM，各 1 拍）
    //   ★ 地址 0 不是空白：ui_text_rom[0] 存的是第一条文案的首字（"摄"）。段外若读地址 0
    //     会画出它的残列 → 所以在【选择之后】补 0（字库 index 0 = ASCII 空格，字模全 0）。
    wire [7:0] lg_rom_char;
    ui_text_rom u_text_rom_lg (
        .i_clk        (i_pixel_clk),
        .i_addr       (lg_in_text ? lg_rom_rd : 12'd0),
        .o_char_index (lg_rom_char)
    );

    // lg 通路只有静态文案，不需要 RAM 源（也省掉一级源选择）
    reg lg_in_text_d1;
    reg [4:0] lg_row_d1;
    always @(posedge i_pixel_clk) begin
        lg_in_text_d1 <= lg_in_text;
        lg_row_d1     <= lg_sel[4:0];
    end
    wire [7:0] lg_char_index = lg_in_text_d1 ? lg_rom_char : 8'd0;

    // 二级：点阵行（1 拍）；地址 = 字符索引 × 24 + 行号；24 = 16 + 8 → 两个移位相加
    wire [12:0] lg_base      = {1'b0, lg_char_index, 4'd0} + {2'b0, lg_char_index, 3'd0};
    wire [12:0] lg_font_addr = lg_base + {8'd0, lg_row_d1};
    wire [23:0] lg_row_bits;
    font_rom_lg u_font_rom_lg (
        .i_clk      (i_pixel_clk),
        .i_addr     (lg_font_addr),
        .o_row_bits (lg_row_bits)
    );

    // ★ 位选保持【组合】用当前列号（见文件头的坑说明）
    wire [4:0] lg_bit_sel = 5'd23 - lg_col[4:0];
    wire       lg_glyph   = lg_in_text_d1 && lg_row_bits[lg_bit_sel];

    // ==================================================================
    // 小字通路（16x16）—— 保留结构但当前【无任何段】（数据页已迁到 24x24）
    //   译码全 0 → 综合器会把大部分逻辑优化掉；将来要恢复小字内容，
    //   只要往 sm_seg 里加段即可，通路本身不用重写。
    // ==================================================================
    //   用途：数据页的**实时数值**（数字/负号/空格）。字模 16 宽 x 24 高（font_num），
    //   组宽 16 px → 4 个字符 64 px（24 宽时是 96 px），数字间距从 ~11 px 收到 ~3 px，
    //   而字高仍 24 与中文标签等高。
    //   ★ 段格式与 lg 通路**完全一致**（40 位，带 row），所以只是换了一套字模与组宽。
    wire [5:0] sm_slot = x_look[9:4];

    reg [23:0] sm_seg [0:5];
    integer    si;
    always @(*) begin
        for (si = 0; si < 6; si = si + 1) sm_seg[si] = 24'd0;

        // ---- 数据页数值（页 2）----
        //   字段号：0 航向角 / 1=A右后 / 2=B左后 / 3=C右前 / 4=D左前
        //           5 角速度 / 6 中心速度 / 7 航位x / 8 航位y
        //   轮位映射（需求方确认真源）：A=右后 B=左后 C=右前 D=左前
        if (i_page == 2'd2) begin
            // 车头偏航角（字段 0），y=120
            if ((i_pixel_y >= 120) && (i_pixel_y < 144))
                sm_seg[0] = seg_sm_ram(6'd0, 6'd4, PX_YAW_N, (i_pixel_y - 12'd120));
            // ★ 2026-10-18：轮速下移，使其**垂直居中于车轮**（原来比轮心偏上 8 px）：
            //     前轮 y 225..294 → 中心 259.5，数字 24 高 → 基址 248（248..271）
            //   数字的 y 不需要 24 对齐（row = y − 基址，基址任意），只有 x 需要 16 对齐。
            //   前轮速（D / C）
            if ((i_pixel_y >= 248) && (i_pixel_y < 272)) begin
                sm_seg[0] = seg_sm_ram(6'd16, 6'd4, PX_WL_N, (i_pixel_y - 12'd248));
                sm_seg[1] = seg_sm_ram(6'd12, 6'd4, PX_WR_N, (i_pixel_y - 12'd248));
            end
            // 右侧列表第 1 行：角速度数值（字段 5），仍在 y=240..263
            if ((i_pixel_y >= 240) && (i_pixel_y < 264)) begin
                sm_seg[2] = seg_sm_ram(6'd20, 6'd4, PX_VAL_N, (i_pixel_y - 12'd240));
            end
            // y=288：中心速度（字段 6）
            if ((i_pixel_y >= 288) && (i_pixel_y < 312))
                sm_seg[0] = seg_sm_ram(6'd24, 6'd4, PX_VAL_N, (i_pixel_y - 12'd288));
            // y=336：航位x（字段 7）
            if ((i_pixel_y >= 336) && (i_pixel_y < 360))
                sm_seg[0] = seg_sm_ram(6'd28, 6'd4, PX_VAL_N, (i_pixel_y - 12'd336));
            // y=384：航位y（字段 8）
            if ((i_pixel_y >= 384) && (i_pixel_y < 408))
                sm_seg[0] = seg_sm_ram(6'd32, 6'd4, PX_VAL_N, (i_pixel_y - 12'd384));
            // 后轮 y 397..467 → 中心 432，数字基址 420（420..443，中心 431.5）
            if ((i_pixel_y >= 420) && (i_pixel_y < 444)) begin
                sm_seg[0] = seg_sm_ram(6'd8, 6'd4, PX_WL_N, (i_pixel_y - 12'd420));
                sm_seg[1] = seg_sm_ram(6'd4, 6'd4, PX_WR_N, (i_pixel_y - 12'd420));
            end
        end
    end

    reg [23:0] sm_sel;
    always @(*) begin
        sm_sel = 24'd0;
        for (si = 0; si < 6; si = si + 1) begin
            if (sm_seg[si][23] &&
                (sm_slot >= sm_seg[si][10:5]) &&
                (sm_slot <  (sm_seg[si][10:5] + sm_seg[si][16:11]))) begin
                sm_sel = sm_seg[si];
            end
        end
    end

    wire        sm_in_text = sm_sel[23];
    wire [5:0]  sm_off     = sm_slot - sm_sel[10:5];
    wire [5:0]  sm_ram_rd  = sm_sel[22:17] + sm_off;

    // 动态数值 RAM 的读口（数值不需要字符索引 ROM，直接读 RAM 里的字符索引）
    assign o_dyn_raddr = sm_ram_rd;
    wire [7:0] sm_ram_char = i_dyn_rdata;

    reg sm_in_text_d1;
    reg [4:0] sm_row_d1;
    always @(posedge i_pixel_clk) begin
        sm_in_text_d1 <= sm_in_text;
        sm_row_d1     <= sm_sel[4:0];
    end
    wire [7:0] sm_char_index = sm_in_text_d1 ? sm_ram_char : 8'd0;

    // 字模地址 = 字符索引 × 24 + 行号（与 font_rom_lg 同一套寻址，24 = 16 + 8）
    wire [12:0] sm_base      = {1'b0, sm_char_index, 4'd0} + {2'b0, sm_char_index, 3'd0};
    wire [12:0] sm_font_addr = sm_base + {8'd0, sm_row_d1};
    wire [15:0] sm_row_bits;
    font_num u_font_num (
        .i_clk      (i_pixel_clk),
        .i_addr     (sm_font_addr),
        .o_row_bits (sm_row_bits)
    );

    wire [3:0]  sm_bit_sel = 4'd15 - i_pixel_x[3:0];
    wire        sm_glyph   = sm_in_text_d1 && sm_row_bits[sm_bit_sel];

    // ------------------------------------------------------------------
    // 数据页：小车俯视简笔图（实时几何生成，不占 BSRAM）
    //   图形源自 `doc/小车俯视简笔.svg`，放大到 **1.44 倍**并把整体左移，
    //   屏幕坐标 = SVG × 1.44 + (−80, −85)，占 x 352..640 / y 160..491（288×331）。
    //   （2026-10-18：为给轮速文本框腾地方整幅右移 16 px；右移量取 16 的倍数，
    //    使车体中心 496 也能被偏航角数字的 16 px 字组整除对齐。）
    //   原图事实（画布 800x600）：
    //     车体 = 45° 斜切凸六边形 (360,170)(330,200)(330,400)(470,400)(470,200)(440,170)，
    //            stroke-width=3、fill=none
    //     四轮 = 25x48 实心矩形，x=300 / 474.81，y=215 / 335
    //   缩放后的关键量（都是算好的常数，直接写进来；取整一律用四舍五入）：
    //     车体外框 x 395..597、y 160..491；斜边仍是 45°，y<203 时左右各收 (203−y)
    //     车体内框（描边 3px×1.44≈4px）x 399..593、y 164..487；y<207 时各收 (207−y)
    //     四轮：左轮 x 352..388 / 右轮 x 604..640；前轮 y 225..294 / 后轮 y 397..467
    //     车体水平中心 = (395+597)/2 = 496（16 的倍数 → 偏航角可精确同心）
    //   ★ 为什么用几何而不是位图 ROM：图形只有矩形与 45° 斜边，判定是几次常数比较；
    //     位图则要一块 280x322 的 ROM + 地址运算 + 额外的流水线延迟，反而更贵。
    //   ★ 边界一律写成【屏幕绝对坐标】常数，不做 x−336 这类减法 ——
    //     无符号数在 x<336 时会回绕成大正数，图形会跑到屏幕左边去。
    // ------------------------------------------------------------------
    //   缩放 1.44：屏幕 = SVG × 1.44 + (−96, −85)，占 x 336..624 / y 160..491（288×331）
    //   ★ 左界取 16 的倍数（352）：这样车体中心 = 352+144 = 496 也是 16 的倍数，
    //     车头偏航角（16 px 组、宽 64）才能同时做到"16 对齐"和"与车体同心"。
    //     取 344 时中心为 488（488%16=8），偏航角数字无论放 448 还是 464 都会差 8 px
    //     —— 这就是"图案顶部的角度数据没有同步右移"的根因。
    wire car_out = (i_pixel_x >= 12'd395) && (i_pixel_x <= 12'd597) &&
                   (i_pixel_y >= 12'd160) && (i_pixel_y <= 12'd491) &&
                   ((i_pixel_y >= 12'd203) ||
                    ((i_pixel_x >= (12'd598 - i_pixel_y)) &&
                     (i_pixel_x <= (12'd394 + i_pixel_y))));
    wire car_in  = (i_pixel_x >= 12'd399) && (i_pixel_x <= 12'd593) &&
                   (i_pixel_y >= 12'd164) && (i_pixel_y <= 12'd487) &&
                   ((i_pixel_y >= 12'd207) ||
                    ((i_pixel_x >= (12'd606 - i_pixel_y)) &&
                     (i_pixel_x <= (12'd386 + i_pixel_y))));
    wire car_frame = car_out && !car_in;          // 描边 ≈4 px = 外轮廓 − 内轮廓

    wire wh_lf = (i_pixel_x >= 12'd352) && (i_pixel_x <= 12'd388) &&
                 (i_pixel_y >= 12'd225) && (i_pixel_y <= 12'd294);
    wire wh_rf = (i_pixel_x >= 12'd604) && (i_pixel_x <= 12'd640) &&
                 (i_pixel_y >= 12'd225) && (i_pixel_y <= 12'd294);
    wire wh_lr = (i_pixel_x >= 12'd352) && (i_pixel_x <= 12'd388) &&
                 (i_pixel_y >= 12'd397) && (i_pixel_y <= 12'd467);
    wire wh_rr = (i_pixel_x >= 12'd604) && (i_pixel_x <= 12'd640) &&
                 (i_pixel_y >= 12'd397) && (i_pixel_y <= 12'd467);

    wire car_pix = (i_page == 2'd2) &&
                   (car_frame || wh_lf || wh_rf || wh_lr || wh_rr);

    // ------------------------------------------------------------------
    // 四个轮速的文本框（2026-10-18 需求方："用与背景不同底色的文本框突出显示"）
    //   数字 4 字符 × 16 px = 64 px，四周各留 8 px 内边距 → 框 80×36
    //     左框 x 248..327 / 右框 x 664..743；前框 y 240..279 / 后框 y 412..451
    //   ★ 框不与车轮重合：左框右界 327 < 左轮左沿 352；右框左界 664 > 右轮右沿 640
    //     右框 664..743 与右侧列表标签(x=792) 之间还有 49 px 余量
    //   ★ 内边距 8 px（不是 2~3 px 那种贴边效果）
    // ------------------------------------------------------------------
    //   随数字下移：前框 y 240..279（数字 248..271 ± 8）、后框 y 412..451（数字 420..443 ± 8）
    wire box_lf = (i_pixel_x >= 12'd248) && (i_pixel_x <= 12'd327) &&
                  (i_pixel_y >= 12'd240) && (i_pixel_y <= 12'd279);
    wire box_rf = (i_pixel_x >= 12'd664) && (i_pixel_x <= 12'd743) &&
                  (i_pixel_y >= 12'd240) && (i_pixel_y <= 12'd279);
    wire box_lr = (i_pixel_x >= 12'd248) && (i_pixel_x <= 12'd327) &&
                  (i_pixel_y >= 12'd412) && (i_pixel_y <= 12'd451);
    wire box_rr = (i_pixel_x >= 12'd664) && (i_pixel_x <= 12'd743) &&
                  (i_pixel_y >= 12'd412) && (i_pixel_y <= 12'd451);
    wire wheel_box = (i_page == 2'd2) && (box_lf || box_rf || box_lr || box_rr);

    // ------------------------------------------------------------------
    // 图层与颜色
    // ------------------------------------------------------------------
    wire in_menu_bar = (i_pixel_x < MENU_W);
    // ★ 高亮跟随【摇杆选中的项 i_menu_sel】，内容区显示【已按确认进入的页 i_page】
    //   两段式交互：摇杆只移动高亮，按确认才换页。
    wire menu_sel_band = in_menu_bar &&
                         (i_pixel_y >= (MENU_Y0 + i_menu_sel * MENU_STEP - 12)) &&
                         (i_pixel_y <  (MENU_Y0 + i_menu_sel * MENU_STEP + 36));
    //   ★ 状态栏底色**只覆盖菜单条宽度**（需求方 2026-10-18：原来横跨整页，与左侧菜单条不一致）。
    //     文字本来就在 x=24 起、最长 6 字（144 px），全部落在 208 px 内，所以只改底色范围即可。
    wire in_status_bar = (i_pixel_x < MENU_W) &&
                         (i_pixel_y >= (STATUS_Y - 12)) && (i_pixel_y < (STATUS_Y + 36));

    reg [23:0] rgb;
    always @(*) begin
        // 1) 底色（★ 状态栏优先：它横跨全宽，左段也落在菜单条的 x 范围内）
        if (in_status_bar)      rgb = 24'h202020;
        else if (in_menu_bar)   rgb = 24'h101828;
        else                    rgb = 24'h000000;

        // 1b) 轮速文本框底色（盖在内容区黑底之上；文字与车图仍在其上）
        //     24'h2E3440 = 深版 22262E 与浅版 3A4252 的逐通道中点（需求方选定）。
        //       R: (34+58)/2=46   G: (38+66)/2=52   B: (46+82)/2=64
        //     比纯黑明显可辨，又不至于亮到抢走数字的注意力；与菜单条 101828 也能区分。
        if (wheel_box)          rgb = 24'h2E3440;

        // 2) 菜单选中高亮
        if (menu_sel_band)      rgb = 24'h1D4ED8;

        // 3) 大字（菜单 / 状态栏 / 中文标签 / 占位提示）
        if (lg_glyph) begin
            if (in_status_bar)                      rgb = 24'hFFFFFF;   // 状态栏：白
            else if (in_menu_bar && menu_sel_band)  rgb = 24'hFFE066;   // 选中项：亮黄
            else if (in_menu_bar)                   rgb = 24'hE5E7EB;   // 未选中菜单：浅灰
            else                                    rgb = 24'hFFFFFF;   // 标签/占位：白
        end

        // 4) 紧凑数字（数据页实时数值，16x24）
        if (sm_glyph)                               rgb = 24'h7CFC98;   // 数值：亮绿

        // 5) 数据页小车俯视简笔图（白色 = 与标签文字同色；放最后 = 最上层）
        if (car_pix)                                rgb = 24'hFFFFFF;
    end

    // pixel_req 为高的那一拍，hdmi_1024x600 会采样本信号 → 必须组合输出
    assign o_pixel_rgb888 = i_video_active ? rgb : 24'h000000;

endmodule
