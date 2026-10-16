//////////////////////////////////////////////////////////////////////////////////
// 模块：ui_render —— 1024x600 中文界面渲染（菜单 + 四页 + 状态栏 + 实时数值）
//
// 【工作方式】逐像素叠加，不用帧缓存
//   界面由「16x16 点阵字符」拼成，布局强制一条规则：
//     ★ 所有文本起点 x（像素）与 y 都是 16 的倍数
//   于是每个字符恰好落在一个 16 px 宽的"字组"(x>>4) 内：
//     字组号 slot = x >> 4；格内位 = 15 - (x & 15)（bit15 = 最左像素）
//     点阵行 row = y & 15
//
// 【提前 2 拍取数】
//   字符索引（ROM/RAM）与点阵行（font_rom）各延迟 1 拍，共 2 拍。做法：始终请求
//   「x+2 所在的字组」——数据 2 拍后有效时 x 已推进到 x+2，仍落在同一字组内
//   （x+2 跨 16 边界时请求的正好是新字组）。无需行缓冲、无撕裂。
//
// 【文本段（seg）】同一条扫描行可能有多段文字（标签 + 若干数值），所以译码结果
//   不是"一行一项"，而是最多 6 个段描述符；再按当前 slot 选出命中的那一段。
//   每段 32 位：{valid, src, rom_addr[11:0], ram_addr[5:0], len[5:0], xw[5:0]}
//     src = 0 → 静态文案 ROM（菜单/标签/状态词/占位提示）
//     src = 1 → 动态字符 RAM（num_text 生成的实时数值，4 字符/段）
//
// 【四页】0 摄像头 / 1 雷达 / 2 车身运行数据 / 3 路径点导航
//   页 0/1/3 尚未接入真实数据源 → 占位提示；页 2 显示标签 + 实时数值。
//   数值用亮绿色，与白色标签区分。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module ui_render #(
    parameter integer SCREEN_W  = 1024,
    parameter integer SCREEN_H  = 600,
    parameter integer MENU_W    = 208,      // 左侧菜单条宽度（16 的倍数）
    parameter integer MENU_Y0   = 48,       // 第一项文字 y 起点
    parameter integer MENU_STEP = 48,       // 项间距
    parameter integer MENU_XW   = 1,        // 菜单文字起始字组（x=16）
    parameter integer STATUS_Y  = 560,      // 状态栏文字 y 起点
    parameter integer PAGE_Y    = 256,      // 占位提示 y 起点
    parameter integer PAGEXW    = 13        // 内容区文字起始字组（x=208）
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
    input  wire        i_run,           // 1 = 底盘正在输出
                                        //   ★ 当前【未参与渲染】：状态栏没有"运行中"这一档，
                                        //     顶层仍在连它。保留端口是为将来加"运行中"指示，
                                        //     若确认不需要可连同顶层与 tb 一起删（见待办台账）
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

    // 数据页的 5 行标签（按需求原文 §29 的信息项排）
    //   0 航向角 / 1 车轮转速 / 2 角速度 / 3 中心速度 / 4 航位
    function [17:0] label_text;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    label_text = {UI_TXT_LBL_YAW_ADDR,   UI_TXT_LBL_YAW_LEN};
                3'd1:    label_text = {UI_TXT_LBL_WHEEL_ADDR, UI_TXT_LBL_WHEEL_LEN};
                3'd2:    label_text = {UI_TXT_LBL_GYRO_ADDR,  UI_TXT_LBL_GYRO_LEN};
                3'd3:    label_text = {UI_TXT_LBL_VEL_ADDR,   UI_TXT_LBL_VEL_LEN};
                default: label_text = {UI_TXT_LBL_POSE_ADDR,  UI_TXT_LBL_POSE_LEN};
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
    // 段描述符打包
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

    // 数据页数值段列位置（每段 4 字符 = 4 字组；字组 = 16 px）
    localparam [5:0] VX_L  = 6'd20,     // x = 320（单值行）
                     VX_R  = 6'd26,     // x = 416
                     VX_E0 = 6'd20,     // x = 320  ┐
                     VX_E1 = 6'd25,     // x = 400  │ 一行放 4 个轮速，段间留 16 px 间隙
                     VX_E2 = 6'd30,     // x = 480  │ （4 字符 = 64 px，步进 80 px）
                     VX_E3 = 6'd35;     // x = 560  ┘

    reg [31:0] seg [0:5];
    integer    si, mi;

    always @(*) begin
        for (si = 0; si < 6; si = si + 1) seg[si] = 32'd0;   // 默认全部无效

        // ---------------- 左侧菜单 4 项 ----------------
        for (mi = 0; mi < 4; mi = mi + 1) begin
            if ((i_pixel_y >= (MENU_Y0 + mi * MENU_STEP)) &&
                (i_pixel_y <  (MENU_Y0 + mi * MENU_STEP + 16))) begin
                seg[0] = seg_rom(menu_text(mi[1:0]), MENU_XW[5:0]);
            end
        end

        // ---------------- 状态栏 ----------------
        if ((i_pixel_y >= STATUS_Y) && (i_pixel_y < STATUS_Y + 16)) begin
            seg[0] = seg_rom(status_text, MENU_XW[5:0]);
        end

        // ---------------- 数据页（页 2）----------------
        //   字段映射（num_text，每字段 4 字符 = 4 字组，基址 = 字段号 × 4）：
        //     0 航向角 / 1~4 四轮转速 A,B,C,D / 5 角速度 / 6 中心速度 / 7,8 航位 x,y
        //   ★ 需求原文 §29 的完整布局是"中心小车俯视图 + 车轮旁标转速 + 右侧竖排"，
        //     本轮先把【数据项与标签】对齐（下面这 5 行），车体示意图的绘制是下一步。
        //     车轮旁显示的以转速（rpm，±400）为准 —— 不是编码器累计计数。
        if (i_page == 2'd2) begin
            // 行0：航向角（整数度）—— 字段 0
            if ((i_pixel_y >= 80) && (i_pixel_y < 96)) begin
                seg[0] = seg_rom(label_text(3'd0), PAGEXW[5:0]);
                seg[1] = seg_ram(6'd0, 6'd4, VX_L);
            end
            // 行1：四个车轮的转速 A/B/C/D —— 字段 1~4（§29"四个轮子旁显示每个轮子转速"）
            if ((i_pixel_y >= 120) && (i_pixel_y < 136)) begin
                seg[0] = seg_rom(label_text(3'd1), PAGEXW[5:0]);
                seg[1] = seg_ram(6'd4,  6'd4, VX_E0);
                seg[2] = seg_ram(6'd8,  6'd4, VX_E1);
                seg[3] = seg_ram(6'd12, 6'd4, VX_E2);
                seg[4] = seg_ram(6'd16, 6'd4, VX_E3);
            end
            // 行2：角速度 —— 字段 5（单位：整数 °/s）
            if ((i_pixel_y >= 160) && (i_pixel_y < 176)) begin
                seg[0] = seg_rom(label_text(3'd2), PAGEXW[5:0]);
                seg[1] = seg_ram(6'd20, 6'd4, VX_L);
            end
            // 行3：中心速度 —— 字段 6（rpm）
            if ((i_pixel_y >= 200) && (i_pixel_y < 216)) begin
                seg[0] = seg_rom(label_text(3'd3), PAGEXW[5:0]);
                seg[1] = seg_ram(6'd24, 6'd4, VX_L);
            end
            // 行4：航位 x / y —— 字段 7、8（odom 未实现 → 暂恒 0）
            if ((i_pixel_y >= 240) && (i_pixel_y < 256)) begin
                seg[0] = seg_rom(label_text(3'd4), PAGEXW[5:0]);
                seg[1] = seg_ram(6'd28, 6'd4, VX_L);
                seg[2] = seg_ram(6'd32, 6'd4, VX_R);
            end
        end

        // ---------------- 占位提示（页 0/1/3）----------------
        if ((i_page != 2'd2) && (i_pixel_y >= PAGE_Y) && (i_pixel_y < PAGE_Y + 16) &&
            (placeholder_text[5:0] != 6'd0)) begin
            seg[0] = seg_rom(placeholder_text, PAGEXW[5:0]);
        end
    end

    // ------------------------------------------------------------------
    // 提前 2 拍取数：请求 (x+2) 所在的字组
    // ------------------------------------------------------------------
    wire [11:0] x_look   = i_pixel_x + 12'd2;
    wire [5:0]  slot_req = x_look[9:4];
    wire [3:0]  row_now  = i_pixel_y[3:0];

    // 选出命中当前 slot 的段（段之间不重叠，后写覆盖无妨）
    reg [31:0] sel;
    always @(*) begin
        sel = 32'd0;
        for (si = 0; si < 6; si = si + 1) begin
            if (seg[si][31] &&
                (slot_req >= seg[si][5:0]) &&
                (slot_req <  (seg[si][5:0] + seg[si][11:6]))) begin
                sel = seg[si];
            end
        end
    end

    wire        in_text = sel[31];
    wire [5:0]  off     = slot_req - sel[5:0];
    wire [11:0] rom_rd  = sel[29:18] + {6'd0, off};
    wire [5:0]  ram_rd  = sel[17:12] + off;

    // 一级：字符索引（静态 ROM / 动态 RAM，各 1 拍）
    wire [7:0] rom_char;
    ui_text_rom u_text_rom (
        .i_clk        (i_pixel_clk),
        .i_addr       (in_text ? rom_rd : 12'd0),
        .o_char_index (rom_char)
    );

    assign o_dyn_raddr = ram_rd;
    wire [7:0] ram_char = i_dyn_rdata;

    // 源选择也打一拍，与 ROM/RAM 的 1 拍延迟对齐
    reg src_d;
    always @(posedge i_pixel_clk) src_d <= sel[30];
    wire [7:0] char_index = src_d ? ram_char : rom_char;

    // 二级：点阵行（1 拍）；地址 = 字符索引 × 16 + 行号
    wire [11:0] font_rd_addr = {char_index, row_now};
    wire [15:0] row_bits;
    font_rom u_font_rom (
        .i_clk      (i_pixel_clk),
        .i_addr     (font_rd_addr),
        .o_row_bits (row_bits)
    );

    // 数据 2 拍后有效，此刻 i_pixel_x 仍在同一字组内 → 直接取位
    wire [3:0] bit_sel  = 4'd15 - i_pixel_x[3:0];
    wire       glyph_on = row_bits[bit_sel];

    // ------------------------------------------------------------------
    // 图层与颜色
    // ------------------------------------------------------------------
    wire in_menu_bar = (i_pixel_x < MENU_W);
    // ★ 2026-10-16：高亮跟随【摇杆选中的项 i_menu_sel】，而内容区显示【已按确认进入的页 i_page】
    //   —— 两段式交互（需求方明确）：摇杆只移动高亮，按确认才换页。
    wire menu_sel_band = in_menu_bar &&
                         (i_pixel_y >= (MENU_Y0 + i_menu_sel * MENU_STEP - 8)) &&
                         (i_pixel_y <  (MENU_Y0 + i_menu_sel * MENU_STEP + 24));
    wire in_status_bar = (i_pixel_y >= (STATUS_Y - 8)) && (i_pixel_y < (STATUS_Y + 24));

    reg [23:0] rgb;
    always @(*) begin
        // 1) 底色（★ 状态栏优先：它横跨全宽，左段也落在菜单条的 x 范围内；
        //    否则状态栏左半边是菜单色、右半边是状态栏色，视觉割裂）
        if (in_status_bar)      rgb = 24'h202020;
        else if (in_menu_bar)   rgb = 24'h101828;
        else                    rgb = 24'h000000;

        // 2) 菜单选中高亮
        if (menu_sel_band)      rgb = 24'h1D4ED8;

        // 3) 文字
        if (in_text && glyph_on) begin
            if (in_status_bar)                     rgb = 24'hFFFFFF;   // 状态栏：白
            else if (in_menu_bar && menu_sel_band) rgb = 24'hFFE066;   // 选中项：亮黄
            else if (in_menu_bar)                  rgb = 24'hE5E7EB;   // 未选中菜单：浅灰
            else if (sel[30])                      rgb = 24'h7CFC98;   // 动态数值：亮绿
            else                                   rgb = 24'hFFFFFF;   // 标签：白
        end
    end

    // pixel_req 为高的那一拍，hdmi_1024x600 会采样本信号 → 必须组合输出
    assign o_pixel_rgb888 = i_video_active ? rgb : 24'h000000;

endmodule
