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
// ★★ 两套字号（2026-10-17 新增，需求方要求"左侧菜单与档位文字放大 1.5 倍"）★★
//   16x16 点阵无法无损放大，所以字库生成两套（见 tools/gen_font_rom.py）：
//     小字 16x16 → 右侧内容区（数据页标签/数值、占位提示）
//     大字 24x24 → 左侧菜单条（4 个菜单项 + 状态栏档位文字），MENU_W 宽度不变
//   两条通路**完全并行**（各自一套 text_rom + font_rom + 流水线），因为同一扫描行
//   左侧要画大字、右侧要画小字。两套字库共用同一份字符索引（ui_text_rom.v 只有一份，
//   所以 ui_text_index.vh 的 ADDR/LEN 与字号无关）。
//
// 【提前 2 拍取数】
//   字符索引（ROM/RAM）与点阵行（font_rom）各延迟 1 拍，共 2 拍。做法：始终请求
//   「x+2 所在的字组」——数据 2 拍后有效时 x 已推进到 x+2，仍落在同一字组内。
//   无需行缓冲、无撕裂。
//   ★ 坑（已踩过，见工作记录 §30.5）：**位选 bit_sel 必须用当前 x（组合）**，
//     只有"段门控"随数据打拍。位选若也打拍，整屏文字会整体平移。
//
// 【文本段（seg）】同一条扫描行可能有多段文字，所以译码结果不是"一行一项"，
//   而是若干段描述符；再按当前 slot 选出命中的那一段。
//   小字段 32 位：{valid, src, rom_addr[11:0], ram_addr[5:0], len[5:0], xw[5:0]}
//     src = 0 → 静态文案 ROM；src = 1 → 动态字符 RAM（num_text 生成的实时数值）
//   大字段 32 位：{valid, 2'b00, rom_addr[11:0], len[5:0], xw[5:0], row[4:0]}
//     大字无动态来源，且把字模行号直接放进描述符（省掉 y%24 的除法）
//
// 【四页】0 摄像头 / 1 雷达 / 2 车身运行数据 / 3 路径点导航
//   页 0/1/3 尚未接入真实数据源 → 占位提示；页 2 显示标签 + 实时数值。
//   数值用亮绿色，与白色标签区分。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module ui_render #(
    parameter integer SCREEN_W  = 1024,
    parameter integer SCREEN_H  = 600,
    // ---- 左侧菜单条（大字 24x24）----
    parameter integer MENU_W    = 208,      // 左侧菜单条宽度（★ 需求方要求不变）
    parameter integer LG_CELL   = 24,       // 大字点阵边长（1.5 x 16）
    parameter integer LG_XW     = 1,        // 菜单文字起始大字组（x = 24）
    parameter integer MENU_Y0   = 72,       // 第一项文字 y 起点（须为 LG_CELL 的倍数）
    parameter integer MENU_STEP = 72,       // 项间距（原 48 x 1.5，保持文字:行距 = 1:3）
    parameter integer STATUS_Y  = 552,      // 状态栏文字 y 起点（须为 LG_CELL 的倍数）
    // ---- 右侧内容区（小字 16x16，布局未改动）----
    parameter integer PAGE_Y    = 256,      // 占位提示 y 起点
    parameter integer PAGEXW    = 13        // 内容区文字起始小字组（x=208）
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
    input  wire [1:0]  i_page,          // 0=摄像头 1=雷达 2=车身数据 3=路径点导航
    input  wire [1:0]  i_ui_mode,       // 0=手动接管 1=屏幕光标（两档，"自动"档已删）
    input  wire        i_estop,         // 1 = 急停
    input  wire        i_signal_ok,     // 1 = 遥控信号有效
    input  wire        i_arm_hint,      // 1 = 想启动但油门不在中位（提示"请回中位"）
    input  wire [1:0]  i_menu_sel,      // 菜单【高亮选中项】（摇杆拨动改它，与 i_page 分开）

    // ---- 动态数值字符 RAM 读端口（pixel 域）----
    output wire [5:0]  o_dyn_raddr,
    input  wire [7:0]  i_dyn_rdata
);

    // UI 文案索引（localparam；由 tools/gen_font_rom.py 生成，与本文件同目录）
    `include "ui_text_index.vh"

    localparam integer SM_CELL = 16;        // 小字点阵边长

    // ------------------------------------------------------------------
    // 文案取用（function：纯组合，综合稳）
    //   {addr[11:0], len[5:0]}
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

    // 数据页的 4 个列表标签（2026-10-17 改版：车图 + 轮旁转速 + 右侧列表）
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
        // A1：启动被门禁拒绝时给出明确原因，否则用户只会看到"按 S1 没反应"
        else if (i_arm_hint)        status_text = {UI_TXT_ST_ARM_DENY_ADDR,    UI_TXT_ST_ARM_DENY_LEN};
        else if (i_ui_mode == 2'd0) status_text = {UI_TXT_ST_MANUAL_ADDR,      UI_TXT_ST_MANUAL_LEN};
        // ★ 2026-10-16：模式只有两档。原先光标档落到 else 显示"请选择模式"（语义错，
        //   用户已经在光标档了），且 `== 2'd2`（自动档）是永不成立的死分支 → 一并改掉。
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
    // 段描述符打包 —— 小字（16 px 组，右侧内容区）
    //   [31] valid | [30] src | [29:18] rom_addr | [17:12] ram_addr
    //   | [11:6] len | [5:0] xw
    // ------------------------------------------------------------------
    function [31:0] seg_rom;
        input [17:0] txt;      // {addr, len}
        input [5:0]  xw;
        begin
            seg_rom = {1'b1, 1'b0, txt[17:6], 6'd0, txt[5:0], xw};
        end
    endfunction

    function [31:0] seg_ram;
        input [5:0] base;      // 动态 RAM 起始地址
        input [5:0] len;
        input [5:0] xw;
        begin
            seg_ram = {1'b1, 1'b1, 12'd0, base, len, xw};
        end
    endfunction

    // 数据页各元素的列位置（字组号，16 px/组）—— 2026-10-17 车图改版后的布局
    //   左侧轮速 x=416（组 26）/ 右侧轮速 x=704（组 44）
    //   车头偏航角 x=560（组 35，4 字符宽 64 px，中心 592 = 车身水平中心）
    //   右侧列表：标签 x=800（组 50）/ 数值 x=880（组 55）
    localparam [5:0] PX_YAW = 6'd35,
                     PX_WL  = 6'd26,
                     PX_WR  = 6'd44,
                     PX_LBL = 6'd50,
                     PX_VAL = 6'd55;

    // 段描述符打包 —— 大字（24 px 组，左侧菜单条）
    //   ★ 位段（从左到右拼，务必与下方 lg_sel[...] 的取位一致）：
    //     [31]valid | [30:29]保留 | [28:17]rom_addr | [16:11]len | [10:5]xw | [4:0]row
    //   ★ row 直接由译码给出（y − 该项基址），省掉 y % 24 的除法
    //     —— 注意必须用【完整位宽】相减后再取值，写成 y[4:0] − 基址 会因 5 位截断算错
    function [31:0] seg_lg;
        input [17:0] txt;      // {addr, len}
        input [5:0]  xw;
        input [4:0]  row;
        begin
            seg_lg = {1'b1, 2'b00, txt[17:6], txt[5:0], xw, row};
        end
    endfunction

    // ---------------- 大字段：菜单 4 项 + 状态栏（只有静态 ROM 来源）----------------
    reg [31:0] lg_seg;
    integer    mi;
    always @(*) begin
        lg_seg = 32'd0;
        for (mi = 0; mi < 4; mi = mi + 1) begin
            if ((i_pixel_y >= (MENU_Y0 + mi * MENU_STEP)) &&
                (i_pixel_y <  (MENU_Y0 + mi * MENU_STEP + LG_CELL))) begin
                lg_seg = seg_lg(menu_text(mi[1:0]), LG_XW[5:0],
                                (i_pixel_y - (MENU_Y0 + mi * MENU_STEP)));
            end
        end
        if ((i_pixel_y >= STATUS_Y) && (i_pixel_y < STATUS_Y + LG_CELL)) begin
            lg_seg = seg_lg(status_text, LG_XW[5:0], (i_pixel_y - STATUS_Y));
        end
    end

    // ---------------- 小字段：数据页（页 2）+ 占位提示（页 0/1/3）----------------
    reg [31:0] seg [0:5];
    integer    si;
    always @(*) begin
        for (si = 0; si < 6; si = si + 1) seg[si] = 32'd0;   // 默认全部无效

        // ---------------- 数据页（页 2）：车图居中 + 轮旁转速 + 右侧列表 ----------------
        //   依据需求原文 §29："菜单右侧画面中心为一辆小车俯视示意图，小车前端（即画面上方）
        //   实时显示车辆朝向（航向角），四个轮子旁实时显示每个轮子转速，右侧单独竖向排列
        //   显示角速度、中心速度、航位等信息。"
        //   ★ 布局约束（也是 TB 断言的判据）：
        //     - 车图占 x 492..692 / y 185..415（200x230，1:1 照 doc/小车俯视简笔.svg）
        //     - 车图水平中心 592 与车头偏航角数值中心对齐；图之外不与任何文字重叠
        //     - 轮速贴轮子外侧（左 x=416..479，右 x=704..767），右侧列表从 x=800 起
        //   ★ 字段号即 num_text 的字段（每字段 4 字符，基址 = 字段号 × 4）：
        //     0 航向角 / 1~4 四轮转速 A,B,C,D / 5 角速度 / 6 中心速度 / 7,8 航位 x,y
        if (i_page == 2'd2) begin
            // 车头：偏航角（字段 0）
            if ((i_pixel_y >= 160) && (i_pixel_y < 176)) begin
                seg[0] = seg_ram(6'd0, 6'd4, PX_YAW);
            end
            // 右侧列表 第 1 行：角速度（字段 5）
            if ((i_pixel_y >= 176) && (i_pixel_y < 192)) begin
                seg[0] = seg_rom(label_text(2'd0), PX_LBL);
                seg[1] = seg_ram(6'd20, 6'd4, PX_VAL);
            end
            // 右侧列表 第 2 行：中心速度（字段 6）
            if ((i_pixel_y >= 208) && (i_pixel_y < 224)) begin
                seg[0] = seg_rom(label_text(2'd1), PX_LBL);
                seg[1] = seg_ram(6'd24, 6'd4, PX_VAL);
            end
            // ★ 这一行同时有 4 段：左前轮速 / 右前轮速 / 航位x 标签 / 航位x 数值
            // ★ 2026-10-17 需求方指出映射错位，修正后：
            //     左前 = D = 字段 4（基址 16）／右前 = C = 字段 3（基址 12）
            //   ★ 通道 ↔ 轮位真相（需求方 2026-10-17 确认）：
            //     A = 右后 / B = 左后 / C = 右前 / D = 左前
            //     以前按"A 左前、B 右前、C 左后、D 右后"排，正好是错的。
            if ((i_pixel_y >= 240) && (i_pixel_y < 256)) begin
                seg[0] = seg_ram(6'd16, 6'd4, PX_WL);    // 左前轮速 = D
                seg[1] = seg_ram(6'd12, 6'd4, PX_WR);    // 右前轮速 = C
                seg[2] = seg_rom(label_text(2'd2), PX_LBL);
                seg[3] = seg_ram(6'd28, 6'd4, PX_VAL);
            end
            // 右侧列表 第 4 行：航位y（字段 8）
            if ((i_pixel_y >= 272) && (i_pixel_y < 288)) begin
                seg[0] = seg_rom(label_text(2'd3), PX_LBL);
                seg[1] = seg_ram(6'd32, 6'd4, PX_VAL);
            end
            // 后轮转速
            // ★ 2026-10-17 需求方指出映射错位，修正后：
            //     左后 = B = 字段 2（基址 8）／右后 = A = 字段 1（基址 4）
            if ((i_pixel_y >= 368) && (i_pixel_y < 384)) begin
                seg[0] = seg_ram(6'd8,  6'd4, PX_WL);    // 左后轮速 = B
                seg[1] = seg_ram(6'd4,  6'd4, PX_WR);    // 右后轮速 = A
            end
        end

        // ---------------- 占位提示（页 0/1/3）----------------
        if ((i_page != 2'd2) && (i_pixel_y >= PAGE_Y) && (i_pixel_y < PAGE_Y + 16) &&
            (placeholder_text[5:0] != 6'd0)) begin
            seg[0] = seg_rom(placeholder_text, PAGEXW[5:0]);
        end
    end

    // ==================================================================
    // 小字通路（16x16，右侧内容区）—— 提前 2 拍取数
    // ==================================================================
    wire [11:0] x_look   = i_pixel_x + 12'd2;
    wire [5:0]  sm_slot  = x_look[9:4];         // 16 px/组 → 直接取位
    wire [3:0]  sm_row   = i_pixel_y[3:0];

    reg [31:0] sm_sel;
    always @(*) begin
        sm_sel = 32'd0;
        for (si = 0; si < 6; si = si + 1) begin
            if (seg[si][31] &&
                (sm_slot >= seg[si][5:0]) &&
                (sm_slot <  (seg[si][5:0] + seg[si][11:6]))) begin
                sm_sel = seg[si];
            end
        end
    end

    wire        sm_in_text = sm_sel[31];
    wire [5:0]  sm_off     = sm_slot - sm_sel[5:0];
    wire [11:0] sm_rom_rd  = sm_sel[29:18] + {6'd0, sm_off};
    wire [5:0]  sm_ram_rd  = sm_sel[17:12] + sm_off;

    // 一级：字符索引（静态 ROM / 动态 RAM，各 1 拍）
    //   ★ 地址 0 不是空白：ui_text_rom[0] 存的是**第一条文案的首字**（当前是"摄"）。
    //     所以"段外"不能靠读地址 0 来得到空白 —— 那样会在段边界画出首字的残列。
    //     这里仍喂合法地址 0（ROM 读端口要求地址有效），但在【选择之后】补 0：
    //     字库 index 0 = ASCII 空格（字模全 0），这才是真正的空白。
    wire [7:0] sm_rom_char;
    ui_text_rom u_text_rom_sm (
        .i_clk        (i_pixel_clk),
        .i_addr       (sm_in_text ? sm_rom_rd : 12'd0),
        .o_char_index (sm_rom_char)
    );

    assign o_dyn_raddr = sm_ram_rd;
    wire [7:0] sm_ram_char = i_dyn_rdata;

    // 源选择也打一拍，与 ROM/RAM 的 1 拍延迟对齐
    reg sm_src_d;
    always @(posedge i_pixel_clk) sm_src_d <= sm_sel[30];
    // in_text 打一拍：与 rom_char / ram_char 对齐在同一拍
    reg sm_in_text_d1;
    always @(posedge i_pixel_clk) sm_in_text_d1 <= sm_in_text;
    wire [7:0] sm_char_index = sm_in_text_d1
                             ? (sm_src_d ? sm_ram_char : sm_rom_char)
                             : 8'd0;

    // 二级：点阵行（1 拍）；地址 = 字符索引 × 16 + 行号
    wire [11:0] sm_font_addr = {sm_char_index, sm_row};
    wire [15:0] sm_row_bits;
    font_rom u_font_rom (
        .i_clk      (i_pixel_clk),
        .i_addr     (sm_font_addr),
        .o_row_bits (sm_row_bits)
    );

    // ★ 位选保持【组合】用当前 pixel_x（见文件头的坑说明）
    wire [3:0]  sm_bit_sel = 4'd15 - i_pixel_x[3:0];
    wire        sm_glyph   = sm_in_text_d1 && sm_row_bits[sm_bit_sel];

    // ==================================================================
    // 大字通路（24x24，左侧菜单条）—— 同样的提前 2 拍结构
    // ==================================================================
    // 24 px/组 不是 2 的幂，需要一次常数除法；只在左侧区域有意义。
    //   ★ 用【无符号 12 位参数】参与运算：直接写 LG_CELL（integer，有符号 32 位）
    //     会引入有符号/无符号混算；而 `lg_slot * LG_CELL[5:0]` 更危险 ——
    //     两个 6 位相乘会被截成 6 位，24*8=192 直接溢出成错值。
    localparam [11:0] LG_CELL_N = LG_CELL;
    wire [5:0]  lg_slot     = x_look / LG_CELL_N;
    wire [11:0] lg_col_look = x_look - lg_slot * LG_CELL_N;   // 格内列号（对应 x_look）

    // ★ 位选必须与小字通路同一口径：用【当前 pixel_x】的列号，而不是 x_look 的。
    //   col(x) = (x_look − 2) mod 24，这里用条件减法避免再引一个除法器：
    //     x_look 落在组内前 2 列时，x 属于上一组的末尾（24−2 .. 24−1）。
    wire [11:0] lg_col      = (lg_col_look >= 12'd2) ? (lg_col_look - 12'd2)
                                                    : (lg_col_look + 12'd22);

    reg [31:0] lg_sel;
    always @(*) begin
        lg_sel = 32'd0;
        if (lg_seg[31] &&
            (lg_slot >= lg_seg[10:5]) &&
            (lg_slot <  (lg_seg[10:5] + lg_seg[16:11]))) begin
            lg_sel = lg_seg;
        end
    end

    wire        lg_in_text = lg_sel[31];
    wire [5:0]  lg_off     = lg_slot - lg_sel[10:5];
    wire [11:0] lg_rom_rd  = lg_sel[28:17] + {6'd0, lg_off};

    wire [7:0] lg_rom_char;
    ui_text_rom u_text_rom_lg (               // 与右侧共用同一份字符索引数据
        .i_clk        (i_pixel_clk),
        .i_addr       (lg_in_text ? lg_rom_rd : 12'd0),
        .o_char_index (lg_rom_char)
    );

    reg lg_in_text_d1;
    reg [4:0] lg_row_d1;
    always @(posedge i_pixel_clk) begin
        lg_in_text_d1 <= lg_in_text;
        lg_row_d1     <= lg_sel[4:0];
    end
    wire [7:0] lg_char_index = lg_in_text_d1 ? lg_rom_char : 8'd0;

    // 地址 = 字符索引 × 24 + 行号；24 = 16 + 8 → 两个移位相加，不用乘法器
    wire [12:0] lg_base = {1'b0, lg_char_index, 4'd0} + {2'b0, lg_char_index, 3'd0};
    wire [12:0] lg_font_addr = lg_base + {8'd0, lg_row_d1};
    wire [23:0] lg_row_bits;
    font_rom_lg u_font_rom_lg (
        .i_clk      (i_pixel_clk),
        .i_addr     (lg_font_addr),
        .o_row_bits (lg_row_bits)
    );

    wire [4:0]  lg_bit_sel = 5'd23 - lg_col[4:0];
    wire        lg_glyph   = lg_in_text_d1 && (i_pixel_x < MENU_W) && lg_row_bits[lg_bit_sel];

    // ------------------------------------------------------------------
    // 数据页：小车俯视简笔图（实时几何生成，不占 BSRAM）
    //   图形 1:1 照 `doc/小车俯视简笔.svg`：
    //     车体 = 45° 斜切凸六边形 (360,170)(330,200)(330,400)(470,400)(470,200)(440,170)，描边 3 px
    //     四轮 = 25x48 实心矩形，x=300/475，y=215/335
    //   屏幕 = SVG + (192, 15) → 整幅占 x 492..692 / y 185..415（200x230）
    //   ★ 为什么用几何而不是位图 ROM：图形只有矩形与 45° 斜边，判定是几次常数比较；
    //     位图则要一块 200x230 的 ROM + 地址运算 + 额外的流水线延迟，反而更贵、更难对齐。
    //   ★ 边界一律写成【屏幕绝对坐标】常数，不做 x-192 这类减法 ——
    //     无符号数在 x<192 时会回绕成大正数，图形会跑到屏幕左边去。
    // ------------------------------------------------------------------
    // 车体外轮廓（凸六边形）：矩形 [522,662]x[185,415]，y<215 处左右各收 (215-y)
    wire car_out = (i_pixel_x >= 12'd522) && (i_pixel_x <= 12'd662) &&
                   (i_pixel_y >= 12'd185) && (i_pixel_y <= 12'd415) &&
                   ((i_pixel_y >= 12'd215) ||
                    ((i_pixel_x >= (12'd737 - i_pixel_y)) &&
                     (i_pixel_x <= (12'd447 + i_pixel_y))));
    // 车体内轮廓（外轮廓向内收 3 px）：[525,659]x[188,412]，y<218 处左右各收 (218-y)
    wire car_in  = (i_pixel_x >= 12'd525) && (i_pixel_x <= 12'd659) &&
                   (i_pixel_y >= 12'd188) && (i_pixel_y <= 12'd412) &&
                   ((i_pixel_y >= 12'd218) ||
                    ((i_pixel_x >= (12'd743 - i_pixel_y)) &&
                     (i_pixel_x <= (12'd441 + i_pixel_y))));
    wire car_frame = car_out && !car_in;          // 描边 3 px = 外轮廓 − 内轮廓

    // 四轮（实心矩形 25x48）
    wire wh_lf = (i_pixel_x >= 12'd492) && (i_pixel_x <= 12'd516) &&
                 (i_pixel_y >= 12'd230) && (i_pixel_y <= 12'd277);
    wire wh_rf = (i_pixel_x >= 12'd667) && (i_pixel_x <= 12'd691) &&
                 (i_pixel_y >= 12'd230) && (i_pixel_y <= 12'd277);
    wire wh_lr = (i_pixel_x >= 12'd492) && (i_pixel_x <= 12'd516) &&
                 (i_pixel_y >= 12'd350) && (i_pixel_y <= 12'd397);
    wire wh_rr = (i_pixel_x >= 12'd667) && (i_pixel_x <= 12'd691) &&
                 (i_pixel_y >= 12'd350) && (i_pixel_y <= 12'd397);

    wire car_pix = (i_page == 2'd2) &&
                   (car_frame || wh_lf || wh_rf || wh_lr || wh_rr);

    // ------------------------------------------------------------------
    // 图层与颜色
    // ------------------------------------------------------------------
    wire in_menu_bar = (i_pixel_x < MENU_W);
    // ★ 2026-10-16：高亮跟随【摇杆选中的项 i_menu_sel】，而内容区显示【已按确认进入的页 i_page】
    //   —— 两段式交互（需求方明确）：摇杆只移动高亮，按确认才换页。
    //   ★ 2026-10-17：高亮带高度随字号放大（原 ±8/24 按 16 px 字，现 ±12/36 按 24 px 字）
    wire menu_sel_band = in_menu_bar &&
                         (i_pixel_y >= (MENU_Y0 + i_menu_sel * MENU_STEP - 12)) &&
                         (i_pixel_y <  (MENU_Y0 + i_menu_sel * MENU_STEP + 36));
    wire in_status_bar = (i_pixel_y >= (STATUS_Y - 12)) && (i_pixel_y < (STATUS_Y + 36));

    reg [23:0] rgb;
    always @(*) begin
        // 1) 底色（★ 状态栏优先：它横跨全宽，左段也落在菜单条的 x 范围内；
        //    否则状态栏左半边是菜单色、右半边是状态栏色，视觉割裂）
        if (in_status_bar)      rgb = 24'h202020;
        else if (in_menu_bar)   rgb = 24'h101828;
        else                    rgb = 24'h000000;

        // 2) 菜单选中高亮
        if (menu_sel_band)      rgb = 24'h1D4ED8;

        // 3) 大字（左侧：菜单项 + 档位文字）
        if (lg_glyph) begin
            if (in_status_bar)                     rgb = 24'hFFFFFF;   // 状态栏：白
            else if (in_menu_bar && menu_sel_band) rgb = 24'hFFE066;   // 选中项：亮黄
            else                                   rgb = 24'hE5E7EB;   // 未选中菜单：浅灰
        end

        // 4) 小字（右侧：数据页标签/数值、占位提示）
        if (sm_glyph) begin
            if (sm_src_d)                          rgb = 24'h7CFC98;   // 动态数值：亮绿
            else                                   rgb = 24'hFFFFFF;   // 标签/占位：白
        end

        // 5) 数据页小车俯视简笔图（白色 = 与标签文字同色；放最后 = 最上层）
        if (car_pix)                               rgb = 24'hFFFFFF;
    end

    // pixel_req 为高的那一拍，hdmi_1024x600 会采样本信号 → 必须组合输出
    assign o_pixel_rgb888 = i_video_active ? rgb : 24'h000000;

endmodule
