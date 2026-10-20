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
    input  wire [2:0]  i_page,          // 0=数据 1=摄像头 2=雷达 3=避障导航 4=非避障导航
    input  wire [1:0]  i_ui_mode,       // 0=手动接管 1=屏幕光标
    input  wire        i_estop,         // 1 = 急停
    input  wire        i_signal_ok,     // 1 = 遥控信号有效
    input  wire        i_arm_hint,      // 1 = 想启动但油门不在中位
    input  wire [2:0]  i_menu_sel,      // 菜单【高亮选中项】

    // ---- 非避障导航页（2026-10-19 新增，全部来自 nav_free_ctrl + 两级同步）----
    input  wire         i_nav_run,      // 1 = 导航中（状态栏变绿并显示「导航中」）
    input  wire         i_nav_paused,   // 1 = 导航暂停（状态栏浅橙 994D00 并显示「导航暂停」）
    input  wire         i_menu_dim,     // 1 = 导航页交互态（菜单栏变暗、摇杆不再切菜单）
    input  wire [2:0]   i_aux_sel,      // 右下辅助菜单高亮项 0..4
    input  wire [9:0]   i_car_px,       // 车绿点像素坐标（来自 odom 航位）
    input  wire [9:0]   i_car_py,
    input  wire [9:0]   i_cur_px,       // 十字准心像素坐标
    input  wire [9:0]   i_cur_py,
    input  wire [159:0] i_pt_px,        // 已记录点：16 个 × 10 位（扁平）
    input  wire [159:0] i_pt_py,
    input  wire [15:0]  i_pt_v,         // 该点有效

    // ---- 动态数值字符 RAM 读端口（pixel 域）----
    output wire [5:0]  o_dyn_raddr,
    input  wire [7:0]  i_dyn_rdata,

    // ---- IMU 状态提示（2026-10-19 需求方指示）----
    //   1 = IMU 初始化失败（vendor imu_top 进了 S_ERROR 的死胡同）→
    //   数据页「航向角」标签正上方用**红字**显示「imu初始化错误」。
    //   此刻航向角/角速度会恒为 0，没有这行提示根本分不清"车没转"和"IMU 没起来"。
    input  wire         i_imu_fail,
    input  wire [3:0]   i_tune_sel,     // 参数整定页当前选中行（0..11）
    // 2026-10-20（上板二次反馈）：参数页处于「编辑态」才变暗左侧菜单栏、才画行高亮条。
    //   退出编辑态后 page 仍停在参数页（与导航页退出 nav_ui 同构），
    //   此时菜单栏恢复正常亮度、高亮条消失，菜单键恢复可用。
    input  wire         i_tune_active
);

    // UI 文案索引（localparam；由 tools/gen_font_rom.py 生成，与本文件同目录）
    `include "ui_text_index.vh"

    localparam integer SM_CELL = 16;        // 小字点阵边长（保留通路，当前无段）

    // ------------------------------------------------------------------
    // 页面编码（★ 必须与 rtl\top\falcons_top.v 的 PAGE_* 保持一致）
    //
    //   ★ 2026-10-19（需求方指示）：一级菜单由 4 项扩为 5 项，顺序改为需求原文的
    //     章节顺序 —— 运行数据显示 / 摄像头画面 / 雷达建图展示 / 避障导航 / 非避障导航。
    //     原第 4 项「路径点导航」在需求原文里拆成「避障导航」+「非避障导航」两页。
    //   ★ 连带影响：数据页编码由 **2 变成 0**（新顺序的第 1 项），
    //     所以 `falcons_top.PAGE_DEFAULT` 也由 2'd2 改成 3'd0；**摄像头页由 0 变成 1**
    //     （`cam_overlay` 的 `show_cam` 由顶层按 `page == 1` 给出，不再是比较 0）。
    //     改序时必须**两处同改**：这里的 PG_* 与顶层的 PAGE_*。
    // ------------------------------------------------------------------
    localparam [2:0] PG_DATA     = 3'd0;    // 运行数据显示（上电默认页）
    localparam [2:0] PG_CAMERA   = 3'd1;    // 摄像头画面
    localparam [2:0] PG_RADAR    = 3'd2;    // 雷达建图展示
    localparam [2:0] PG_NAV_OBS  = 3'd3;    // 避障导航
    localparam [2:0] PG_NAV_FREE = 3'd4;    // 非避障导航
    localparam [2:0] PG_TUNE     = 3'd5;    // 参数整定（2026-10-20 新增，一期：存 RAM）

    // ---- 非避障导航页：右下角辅助菜单几何（真源；改这里即可整体平移）----
    localparam integer NAV_AUX_X    = 840;  // 文字左界（24 的倍数 ⇒ 字组号 35）
    localparam integer NAV_AUX_XW   = NAV_AUX_X / LG_CELL;
    localparam integer NAV_AUX_Y0   = 420;  // 第 1 项框顶
    localparam integer NAV_AUX_STEP = 36;   // 项高
    localparam integer NAV_AUX_TDY  = 6;    // 文字相对框顶偏移（(36−24)/2）

    // ---- 参数整定页几何（2026-10-20 新增，一期：11 行参数，存 RAM）----
    //   内容区 x 208..1023。三列：中文标签（24px）x=240 / 数值（16px，5 格）x=384 /
    //   单位（16px）x=480。行距 36，11 行占 y=72..467，最底 461 < 状态栏 540。
    //   标题「参数整定」x=240 / y=24；第 6 个菜单项最底 455 < 状态栏 552，菜单不用改版式。
    //   ★ 标签只有 4~5 个字（最长「转向角速度」「减速距离」），5×24=120 px，
    //     240+120=360 < 384（数值左界），不重叠。
    localparam integer TUNE_Y0      = 72;   // 第 1 行顶
    localparam integer TUNE_STEP    = 36;   // 行高
    localparam integer TUNE_TDY     = 6;    // 24px 标签相对行顶偏移 (36−24)/2
    // ★ 2026-10-20 上板修：数值点阵是 **16 宽 × 24 高**（`font_num` 是 24 行），
    //   不是 16×16。原来 VDY 按 (36−16)/2=10 算、窗口又只有 SM_CELL=16 行，
    //   于是只画出字形的第 0..15 行 —— 上板现象是"右侧数值与单位下部被截掉"。
    //   改法：VDY = (36−24)/2 = 6（与标签同一基线），窗口高度用 TUNE_VH = 24。
    localparam integer TUNE_VDY     = 6;    // 24px 数字相对行顶偏移 (36−24)/2
    localparam integer TUNE_VH      = 24;   // 数值点阵行数（font_num: 16×24）
    localparam integer TUNE_LBL_X   = 240;  // 标签左界（24 的倍数 ⇒ 字组 10）
    localparam integer TUNE_LBL_XW  = TUNE_LBL_X / LG_CELL;
    localparam integer TUNE_VAL_XW  = 24;   // 数值左界 24×16 = 384
    localparam integer TUNE_UNIT_XW = 30;   // 单位左界 30×16 = 480
    localparam integer TUNE_ROWS    = 12;   // 参数行数（与 tune_ctrl 的 12 项一一对应）

    // ------------------------------------------------------------------
    // 文案取用（function：纯组合，综合稳）  {addr[11:0], len[5:0]}
    // ------------------------------------------------------------------
    function [17:0] menu_text;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    menu_text = {UI_TXT_MENU_VEHICLE_ADDR,  UI_TXT_MENU_VEHICLE_LEN};
                3'd1:    menu_text = {UI_TXT_MENU_CAMERA_ADDR,   UI_TXT_MENU_CAMERA_LEN};
                3'd2:    menu_text = {UI_TXT_MENU_RADAR_ADDR,    UI_TXT_MENU_RADAR_LEN};
                3'd3:    menu_text = {UI_TXT_MENU_NAV_OBS_ADDR,  UI_TXT_MENU_NAV_OBS_LEN};
                3'd4:    menu_text = {UI_TXT_MENU_NAV_FREE_ADDR, UI_TXT_MENU_NAV_FREE_LEN};
                3'd5:    menu_text = {UI_TXT_MENU_TUNE_ADDR,     UI_TXT_MENU_TUNE_LEN};
                default: menu_text = {UI_TXT_MENU_NAV_FREE_ADDR, UI_TXT_MENU_NAV_FREE_LEN};
            endcase
        end
    endfunction

    // ---- 参数整定页：11 行参数的中文标签（24px）----
    //   ★ 顺序与 rtl\ui\tune_ctrl.v 的 o_sel 0..10、以及 num_text 字段号 0..10
    //     三者必须完全一致：标签取 idx，数值取字段 idx，改一处就要三处同改。
    function [17:0] tune_label_text;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:    tune_label_text = {UI_TXT_TUNE_L0_ADDR,  UI_TXT_TUNE_L0_LEN};
                4'd1:    tune_label_text = {UI_TXT_TUNE_L1_ADDR,  UI_TXT_TUNE_L1_LEN};
                4'd2:    tune_label_text = {UI_TXT_TUNE_L2_ADDR,  UI_TXT_TUNE_L2_LEN};
                4'd3:    tune_label_text = {UI_TXT_TUNE_L3_ADDR,  UI_TXT_TUNE_L3_LEN};
                4'd4:    tune_label_text = {UI_TXT_TUNE_L4_ADDR,  UI_TXT_TUNE_L4_LEN};
                4'd5:    tune_label_text = {UI_TXT_TUNE_L5_ADDR,  UI_TXT_TUNE_L5_LEN};
                4'd6:    tune_label_text = {UI_TXT_TUNE_L6_ADDR,  UI_TXT_TUNE_L6_LEN};
                4'd7:    tune_label_text = {UI_TXT_TUNE_L7_ADDR,  UI_TXT_TUNE_L7_LEN};
                // 2026-10-20（需求方）：积分限幅插在「航向 Kd」下、「速度环 Kp」上 ⇒
                // 行号 8 = 积分限幅（字库常量 L11），行号 9/10/11 = 速度环三项（常量 L8/L9/L10）。
                // 常量名里的数字是**文案编号**，与行号不再一一对应，改行序时只动这里。
                4'd8:    tune_label_text = {UI_TXT_TUNE_L11_ADDR, UI_TXT_TUNE_L11_LEN};
                4'd9:    tune_label_text = {UI_TXT_TUNE_L8_ADDR,  UI_TXT_TUNE_L8_LEN};
                4'd10:   tune_label_text = {UI_TXT_TUNE_L9_ADDR,  UI_TXT_TUNE_L9_LEN};
                4'd11:   tune_label_text = {UI_TXT_TUNE_L10_ADDR, UI_TXT_TUNE_L10_LEN};
                default: tune_label_text = {UI_TXT_TUNE_L0_ADDR,  UI_TXT_TUNE_L0_LEN};
            endcase
        end
    endfunction


    // ---- 参数整定页：数值单位（16px，跟在小数右侧）----
    //   返回 {addr, len}；len = 0 表示该行不画单位（Kp/Ki/Kd 是无量纲）。
    //   ★ 度号 '°' 与 '/' 字库里本来就有（LBL_DEG:/°），小写 's' 是本页新增。
    function [17:0] tune_unit_text;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:       tune_unit_text = {UI_TXT_TUNE_UMMS_ADDR,  UI_TXT_TUNE_UMMS_LEN};
                4'd1:       tune_unit_text = {UI_TXT_TUNE_UMM_ADDR,   UI_TXT_TUNE_UMM_LEN};
                4'd2, 4'd3: tune_unit_text = {UI_TXT_TUNE_UDEG_ADDR,  UI_TXT_TUNE_UDEG_LEN};
                4'd4:       tune_unit_text = {UI_TXT_TUNE_UDEGS_ADDR, UI_TXT_TUNE_UDEGS_LEN};
                default:    tune_unit_text = 18'd0;
            endcase
        end
    endfunction

    // ---- 非避障导航页：右下角辅助菜单（5 项）文案 ----
    //   需求原文顺序：确认路径点 / 撤销路径点 / 间断寻点 / 非间断寻点 / 清除所有路径点
    //   ★ 2026-10-19 需求方指示：辅助菜单两页都**不再有「返回」项**，返回改由独立的
    //     返回按键触发（该按键尚未接入，nav_free_ctrl 的 i_back 预留）。
    function [17:0] nav_aux_text;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    nav_aux_text = {UI_TXT_AUX_SET_POINT_ADDR,  UI_TXT_AUX_SET_POINT_LEN};
                3'd1:    nav_aux_text = {UI_TXT_AUX_UNDO_POINT_ADDR, UI_TXT_AUX_UNDO_POINT_LEN};
                3'd2:    nav_aux_text = {UI_TXT_AUX_POINT_GAP_ADDR,  UI_TXT_AUX_POINT_GAP_LEN};
                3'd3:    nav_aux_text = {UI_TXT_AUX_POINT_CONT_ADDR, UI_TXT_AUX_POINT_CONT_LEN};
                default: nav_aux_text = {UI_TXT_AUX_CLEAR_ALL_ADDR,  UI_TXT_AUX_CLEAR_ALL_LEN};
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

    // ★ 2026-10-18 两处带单位标签改为【中文 24px + 符号 16px】混排（第二批）
    //   ① 「航向角」+「/°」   ② 「车轮转速」+「/rpm」
    //   中文用 `UI_TXT_LBL_YAW` / `UI_TXT_LBL_WHEEL`（lg 通路），
    //   后缀用 `UI_TXT_LBL_DEG` / `UI_TXT_LBL_RPM`（sm 通路，ROM 源），
    //   在下面 lg_seg / sm_seg 两处分别打包。改文案只需改 tools\font_chars.txt。
    reg [17:0] status_text;
    always @(*) begin
        if (i_estop)                status_text = {UI_TXT_ST_ESTOP_ADDR,       UI_TXT_ST_ESTOP_LEN};
        else if (!i_signal_ok)      status_text = {UI_TXT_ST_NOSIGNAL_ADDR,    UI_TXT_ST_NOSIGNAL_LEN};
        else if (i_arm_hint)        status_text = {UI_TXT_ST_ARM_DENY_ADDR,    UI_TXT_ST_ARM_DENY_LEN};
        // ★ 2026-10-19：非避障导航页按下「间断/非间断寻点」后进入运行态，状态栏显示「导航中」
        else if (i_nav_paused)      status_text = {UI_TXT_ST_PAUSED_ADDR,      UI_TXT_ST_PAUSED_LEN};
        else if (i_nav_run)         status_text = {UI_TXT_ST_RUNNING_ADDR,     UI_TXT_ST_RUNNING_LEN};
        else if (i_ui_mode == 2'd0) status_text = {UI_TXT_ST_MANUAL_ADDR,      UI_TXT_ST_MANUAL_LEN};
        else                        status_text = {UI_TXT_ST_CURSOR_ADDR,      UI_TXT_ST_CURSOR_LEN};
    end

    reg [17:0] placeholder_text;
    always @(*) begin
        case (i_page)
            PG_CAMERA:   placeholder_text = {UI_TXT_PLACEHOLDER_CAM_ADDR,   UI_TXT_PLACEHOLDER_CAM_LEN};
            PG_RADAR:    placeholder_text = {UI_TXT_PLACEHOLDER_RADAR_ADDR, UI_TXT_PLACEHOLDER_RADAR_LEN};
            PG_NAV_OBS:  placeholder_text = {UI_TXT_PLACEHOLDER_NAV_ADDR,   UI_TXT_PLACEHOLDER_NAV_LEN};

            default:     placeholder_text = 18'd0;
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

    //   ★ 2026-10-18：sm 段由 24 位扩到 **37 位**，新增静态文案 ROM 源。
    //     动机：lg 通路一个字符固定占 24 px 格，ASCII（/ ° r p m）笔画只有 7~12 px，
    //     居中后左右各留 6~7 px 空白（实测 '/' 空隙 14 px），看起来"散"。
    //     改由 16 px 的 sm 通路渲染后，'/'、'p'、'm' 的空隙降到 6/6/4 px，
    //     与数字的 '0'（宽 10、空隙 6）完全一致。
    //
    //     位段（**低 17 位与 lg 段位置完全相同**）：
    //       [36]valid | [35]src | [34:23]rom_addr(12) | [22:17]ram_addr(6)
    //       | [16:11]len | [10:5]xw | [4:0]row
    //     ★ 因为 [22:17]/[16:11]/[10:5]/[4:0] 一个都没动，原有位索引只有
    //       valid 需要从 [23] 上移到 [36]。src = 0 → 动态数值 RAM，1 → 静态文案 ROM。
    function [36:0] seg_sm;
        input              src;
        input [11:0]       rom_addr;
        input [5:0]        ram_addr;
        input [5:0]        len;
        input [5:0]        xw;
        input [4:0]        row;
        begin
            seg_sm = {1'b1, src, rom_addr, ram_addr, len, xw, row};
        end
    endfunction

    // 动态数值（RAM 源）
    function [36:0] seg_sm_ram;
        input [5:0] base;      // 动态 RAM 起始地址（字段号 × 5）
        input [5:0] len;       // 字符数
        input [5:0] xw;
        input [4:0] row;
        begin
            seg_sm_ram = seg_sm(1'b0, 12'd0, base, len, xw, row);
        end
    endfunction

    // 静态文案（ROM 源，16 px 格）—— 用于「/°」「/rpm」这类符号后缀
    function [36:0] seg_sm_rom;
        input [11:0] addr;     // ui_text_rom 地址
        input [5:0]  len;
        input [5:0]  xw;
        input [4:0]  row;
        begin
            seg_sm_rom = seg_sm(1'b1, addr, 6'd0, len, xw, row);
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
                     // ★ 2026-10-18：列表数值由 4 字符扩到 5 字符，**向左**拓展一格：
                     //   右边缘仍 975（原来是 912+64=976），左边界 912 → 896（56 × 16）。
                     //   轮速与标签列都不动（需求方明确："轮速不要动，标签文字也不动"）。
                     //   ★ 为什么不会撞标签：中心速度 ≤ 9.99 m/s → ×100 ≤ 999 = 3 位数字，
                     //     在 5 格里右对齐后仍从第 3 格（x=928）起，与「中心速度」标签右界 887
                     //     之间还有 41 px；负值时负号落在第 2 格（x=912），间隙 25 px，
                     //     与改动前（912 起）**逐像素相同**。
                     PX_VAL_N = 6'd56;                  // 列表数值（16 组）：x=896

    // ---------------- 大字段数组：菜单 4 项 + 状态栏 + 数据页 + 占位 ----------------
    //   同一条扫描行最多 4 段（数据页 y=240 那行：左轮速 / 右前轮速 / 角速度标签 / 角速度数值）
    reg [29:0] lg_seg [0:5];
    integer    li, mi, ai, ti;
    always @(*) begin
        for (li = 0; li < 6; li = li + 1) lg_seg[li] = 30'd0;

        // ---- 左侧菜单 6 项（2026-10-19 由 4 项扩为 5 项；2026-10-20 增第 6 项「参数整定」）----
        //   各项起始 y：72 / 144 / 216 / 288 / 360 / 432，每项 24 高、项间距 72。
        //   第 4 项 y=288..311 与数据页「中心速度」标签**同行** —— 用独立槽位区分（见下）。
        //   第 5 项 y=360..383 与数据页「航位x」(336..359)、「航位y」(384..407) 均不相交。
        //   第 6 项 y=432..455 —— 只与参数整定页的标签行（最底 461）相邻，两者互斥（页不同）。
        //   6 项最底 455 < 状态栏框顶 540，布局不用改（MENU_STEP 保持 72）。
        for (mi = 0; mi < 6; mi = mi + 1) begin
            if ((i_pixel_y >= (MENU_Y0 + mi * MENU_STEP)) &&
                (i_pixel_y <  (MENU_Y0 + mi * MENU_STEP + LG_CELL))) begin
                lg_seg[0] = seg_lg_rom(menu_text(mi[2:0]), LG_XW[5:0],
                                       (i_pixel_y - (MENU_Y0 + mi * MENU_STEP)));
            end
        end
        // ---- 非避障导航页：右下角辅助菜单（5 项 / 24px 字 / 项高 36 / 文字 x = 840）----
        //   窗口几何（与下面 nav_aux_bg / nav_aux_hi 必须一致）：
        //     背景 x 832..1023、y 414..599；项框 y = 420 + i*36（高 36）；
        //     文字 x = 840（35 组 × 24）、y = 项框顶 + 6（(36−24)/2 垂直居中）。
        //   ★ 最长项「清除所有路径点」7 字 × 24 = 168 px，840+168 = 1008 ≤ 1023 ✓
        if (i_page == PG_NAV_FREE) begin
            for (ai = 0; ai < 5; ai = ai + 1) begin
                if ((i_pixel_y >= (NAV_AUX_Y0 + ai * NAV_AUX_STEP + NAV_AUX_TDY)) &&
                    (i_pixel_y <  (NAV_AUX_Y0 + ai * NAV_AUX_STEP + NAV_AUX_TDY + LG_CELL))) begin
                    lg_seg[1] = seg_lg_rom(nav_aux_text(ai[2:0]), NAV_AUX_XW,
                                           (i_pixel_y - (NAV_AUX_Y0 + ai * NAV_AUX_STEP + NAV_AUX_TDY)));
                end
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
        if (i_page == PG_DATA) begin
            // ★ 2026-10-19：IMU 初始化失败提示（**红字**），画在「航向角」标签正上方。
            //   需求方："如果陀螺仪初始化失败，在运行数据显示页面，'航向角'字段上方
            //   用红色字显示 'imu初始化失败'"（后确认「初始化错误」亦可）。
            //   「imu初始化错误」8 字 × 24 = 192 px，居中于标签中心 491.5 ⇒ 左界 395.5；
            //   x 须为 24 的倍数 → 取 **384**（384..575，中心 479.5，偏左 12 px）。
            //   y = 48..71：紧贴标签（标签在 72..95），且不与任何已有段冲突
            //   （辅助菜单窗口 y ≥ 414、车轮转速标签 y = 504、状态栏 y = 552）。
            //   ★ 颜色不在这里定，见文件末尾"3) 大字"——那里按 `lg_imu_fail_d1` 染红。
            if (i_imu_fail && (i_pixel_y >= 48) && (i_pixel_y < 72))
                lg_seg[5] = seg_lg_rom({UI_TXT_IMU_FAIL_ADDR, UI_TXT_IMU_FAIL_LEN},
                                       6'd16, (i_pixel_y - 12'd48));

            if ((i_pixel_y >= 240) && (i_pixel_y < 264))
                lg_seg[1] = seg_lg_rom(label_text(2'd0), PX_LBL, (i_pixel_y - 12'd240));
            if ((i_pixel_y >= 288) && (i_pixel_y < 312))
                lg_seg[2] = seg_lg_rom(label_text(2'd1), PX_LBL, (i_pixel_y - 12'd288));
            if ((i_pixel_y >= 336) && (i_pixel_y < 360))
                lg_seg[3] = seg_lg_rom(label_text(2'd2), PX_LBL, (i_pixel_y - 12'd336));
            if ((i_pixel_y >= 384) && (i_pixel_y < 408))
                lg_seg[4] = seg_lg_rom(label_text(2'd3), PX_LBL, (i_pixel_y - 12'd384));

            // ★★ 2026-10-18（第二批）：两处标签改为【中文 24px 格 + 符号 16px 格】混排 ★★
            //   中文走 lg 通路（24 px 格），后缀走 sm 通路（16 px 格）——
            //   ASCII 笔画只有 7~12 px，挤在 24 px 格里左右各留 6~7 px 空白，
            //   换成 16 px 格后 '/'、'p'、'm' 的空隙从 14/14/12 降到 6/6/4，
            //   与数字 '0'（宽 10、空隙 6）一致。
            //
            //   ★★ 2026-10-18（第五批）：航向角**不再带单位**，三个字自身居中 ★★
            //   需求方："算了，航向角后面不带单位，让这三个字居中。"
            //   「航向角」3 字 × 24 = 72 px，居中于车体中心 496 ⇒ 左界 460；
            //   x 须为 24 的倍数 → 取 **456**（占 456..527，中心 **491.5**，偏左 4.5 px）。
            //   取 480 会到 480..551（中心 515.5，偏右 19.5 px），故 456 最优。
            //   ★ 数字的底色框 456..535（中心 495.5）与标签左界同为 456，视觉上左缘对齐。
            if ((i_pixel_y >= 72) && (i_pixel_y < 96))
                lg_seg[5] = seg_lg_rom({UI_TXT_LBL_YAW_ADDR, UI_TXT_LBL_YAW_LEN},
                                       6'd19, (i_pixel_y - 12'd72));

            //   ② y = 504..527，车图底边 491 下方留 13 px
            if ((i_pixel_y >= 504) && (i_pixel_y < 528))
                lg_seg[5] = seg_lg_rom({UI_TXT_LBL_WHEEL_ADDR, UI_TXT_LBL_WHEEL_LEN},
                                       6'd17, (i_pixel_y - 12'd504));
        end

        // ---- 参数整定页：11 行中文标签 + 标题（2026-10-20 新增）----
        //   ★ 用 lg_seg[1]（菜单占 lg_seg[0]），与数据页标签、导航辅助菜单同一槽位，
        //     靠 i_page 互斥 —— 同一时刻只有一个页面在写。
        if (i_page == PG_TUNE) begin
            if ((i_pixel_y >= 12'd24) && (i_pixel_y < (12'd24 + LG_CELL))) begin
                lg_seg[2] = seg_lg_rom({UI_TXT_MENU_TUNE_ADDR, UI_TXT_MENU_TUNE_LEN},
                                       TUNE_LBL_XW[5:0], (i_pixel_y - 12'd24));
            end
            for (ti = 0; ti < TUNE_ROWS; ti = ti + 1) begin
                if ((i_pixel_y >= (TUNE_Y0 + ti * TUNE_STEP + TUNE_TDY)) &&
                    (i_pixel_y <  (TUNE_Y0 + ti * TUNE_STEP + TUNE_TDY + LG_CELL))) begin
                    lg_seg[1] = seg_lg_rom(tune_label_text(ti[3:0]), TUNE_LBL_XW[5:0],
                                           (i_pixel_y - (TUNE_Y0 + ti * TUNE_STEP + TUNE_TDY)));
                end
            end
        end

        // ---- 占位提示（摄像头 / 雷达 / 避障导航三页；数据页与参数页有自己的内容）----
        if ((i_page != PG_DATA) && (i_page != PG_TUNE) &&
            (i_pixel_y >= PAGE_Y) && (i_pixel_y < PAGE_Y + LG_CELL) &&
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
    //   ★ 2026-10-19：顺带标出"当前命中的段是 IMU 失败提示"（它要染红）。
    //     段描述符 30 位已经用满（valid 1 + addr 12 + len 6 + slot 6 + row 5），
    //     没有空闲位放颜色标记，所以这里用 ROM 地址反查 —— 追加一条独立文案
    //     就有唯一地址，比挤位宽干净。
    reg [29:0] lg_sel;
    reg        lg_imu_fail;
    always @(*) begin
        lg_sel      = 30'd0;
        lg_imu_fail = 1'b0;
        for (li = 0; li < 6; li = li + 1) begin
            if (lg_seg[li][29] &&
                (lg_slot >= lg_seg[li][10:5]) &&
                (lg_slot <  (lg_seg[li][10:5] + lg_seg[li][16:11]))) begin
                lg_sel      = lg_seg[li];
                lg_imu_fail = (lg_seg[li][28:17] == UI_TXT_IMU_FAIL_ADDR);
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
    reg lg_imu_fail_d1;
    always @(posedge i_pixel_clk) begin
        lg_in_text_d1  <= lg_in_text;
        lg_row_d1      <= lg_sel[4:0];
        lg_imu_fail_d1 <= lg_imu_fail;   // 与 lg_in_text_d1 同拍，供末尾"大字"染红
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

    reg [36:0] sm_seg [0:5];
    integer    si;
    reg [17:0] tu;          // 参数整定页：单位 {addr,len} 的组合暂存
    always @(*) begin
        for (si = 0; si < 6; si = si + 1) sm_seg[si] = 37'd0;

        // ---- 非避障导航页：右上角准心坐标（x 在上、y 在下，竖排）----
        //   两行 y = 24 与 56，最右墨迹到 x=1007（离右边缘 16 px，不贴边）。
        //   字段 9 = 准心 x、字段 10 = 准心 y（num_text 的 NUM_FIELDS 已由 9 扩到 11），
        //   单位 0.01 m、5 字符、悬挂小数点 —— 与数据页的数值完全同一套格式化。
        //   ★ 与数据页的 sm 段条件互斥（i_page 不同），所以谁写在前都不影响。
        //   ★ 2026-10-19 需求方给了样例「x:  2.60m」并要求"右侧不要紧贴、留出合适间距"：
        //       前 "x:" 2 字符（组 54，x=864） + 数值 5 字符（组 57，x=912）
        //       + 后 "m" 1 字符（组 62，x=992） → **右界 1007**，离开右边缘 16 px。
        //     ★ 上板后从 55/60（右界 975）挪到 57/62：`x:` 占 2 个字符格（32 px），
        //       初版数值起在组 55 会压住它的第 2 格 ⇒ 冒号被数值首格的符号位空格盖掉，
        //       现象是「冒号不显示」。同理 `m` 必须跟着挪，否则数值末格会压住它。
        //     数值是 5 字符（符号 + 4 位）⇒ 「x:」与数字之间出现两个空格，
        //     与样例 "x:  2.60m" 的间距一致。
        if (i_page == PG_NAV_FREE) begin
            if ((i_pixel_y >= 12'd24) && (i_pixel_y < 12'd48)) begin
                sm_seg[0] = seg_sm_rom(UI_TXT_NAV_COORD_X_ADDR, UI_TXT_NAV_COORD_X_LEN,
                                       6'd54, (i_pixel_y - 12'd24));
                sm_seg[1] = seg_sm_ram(6'd45, 6'd5, 6'd57, (i_pixel_y - 12'd24));
                sm_seg[2] = seg_sm_rom(UI_TXT_NAV_COORD_M_ADDR, UI_TXT_NAV_COORD_M_LEN,
                                       6'd62, (i_pixel_y - 12'd24));
            end
            if ((i_pixel_y >= 12'd56) && (i_pixel_y < 12'd80)) begin
                sm_seg[3] = seg_sm_rom(UI_TXT_NAV_COORD_Y_ADDR, UI_TXT_NAV_COORD_Y_LEN,
                                       6'd54, (i_pixel_y - 12'd56));
                sm_seg[4] = seg_sm_ram(6'd50, 6'd5, 6'd57, (i_pixel_y - 12'd56));
                sm_seg[5] = seg_sm_rom(UI_TXT_NAV_COORD_M_ADDR, UI_TXT_NAV_COORD_M_LEN,
                                       6'd62, (i_pixel_y - 12'd56));
            end
        end

        // ---- 数据页数值（页 2）----
        //   字段号：0 航向角 / 1=A右后 / 2=B左后 / 3=C右前 / 4=D左前
        //           5 角速度 / 6 中心速度 / 7 航位x / 8 航位y
        //   轮位映射（需求方确认真源）：A=右后 B=左后 C=右前 D=左前
        //   ★ 2026-10-18：num_text 的字段步长 4 → 5（"符号 + 千百十个"）。
        //     段描述符里的 base 是【读取起始地址】，两类字段取法不同：
        //       4 字符（航向角 / 四轮转速）→ base = 字段号×5 + 1（跳过符号格）
        //       5 字符（角速度 / 中心速度 / 航位）→ base = 字段号×5
        //     地址表：f0=0 f1=5 f2=10 f3=15 f4=20 f5=25 f6=30 f7=35 f8=40（共 45 < 64）
        if (i_page == PG_DATA) begin
            // 车头偏航角（字段 0），y=112 → 读 1..4
            //   ★ 2026-10-18（第四批）：120 → **112**（需求方"数字稍微上移一点"）。
            //     数字的 y 不要求 24 对齐（row = y − 段起点，任意值），
            //     ui_render_tb 的期望也改用"段内相对行"而非 `y % 24`，所以非 24 倍数没问题。
            if ((i_pixel_y >= 112) && (i_pixel_y < 136))
                sm_seg[0] = seg_sm_ram(6'd1, 6'd4, PX_YAW_N, (i_pixel_y - 12'd112));
            // ★ 2026-10-18：轮速下移，使其**垂直居中于车轮**（原来比轮心偏上 8 px）：
            //     前轮 y 225..294 → 中心 259.5，数字 24 高 → 基址 248（248..271）
            //   数字的 y 不需要 24 对齐（row = y − 基址，基址任意），只有 x 需要 16 对齐。
            //   ★★ 2026-10-18 车体掉头 180°（设计变更）：通道↔轮位映射变为
            //      **A=左前 B=右前 C=左后 D=右后**（原 A=右后 B=左后 C=右前 D=左前）。
            //      所以"前轮两个位置"现在读 A/B（字段 1/2）、"后轮两个位置"读 C/D（字段 3/4），
            //      恰好是原来四处的对称互换（21↔6、16↔11）。
            //   前轮速：左=A(字段1→读6) / 右=B(字段2→读11)
            if ((i_pixel_y >= 248) && (i_pixel_y < 272)) begin
                sm_seg[0] = seg_sm_ram(6'd6,  6'd4, PX_WL_N, (i_pixel_y - 12'd248));
                sm_seg[1] = seg_sm_ram(6'd11, 6'd4, PX_WR_N, (i_pixel_y - 12'd248));
            end
            // 右侧列表第 1 行：角速度数值（字段 5），y=240..263，5 字符
            if ((i_pixel_y >= 240) && (i_pixel_y < 264)) begin
                sm_seg[2] = seg_sm_ram(6'd25, 6'd5, PX_VAL_N, (i_pixel_y - 12'd240));
            end
            // y=288：中心速度（字段 6），5 字符
            if ((i_pixel_y >= 288) && (i_pixel_y < 312))
                sm_seg[0] = seg_sm_ram(6'd30, 6'd5, PX_VAL_N, (i_pixel_y - 12'd288));
            // y=336：航位x（字段 7），5 字符
            if ((i_pixel_y >= 336) && (i_pixel_y < 360))
                sm_seg[0] = seg_sm_ram(6'd35, 6'd5, PX_VAL_N, (i_pixel_y - 12'd336));
            // y=384：航位y（字段 8），5 字符
            if ((i_pixel_y >= 384) && (i_pixel_y < 408))
                sm_seg[0] = seg_sm_ram(6'd40, 6'd5, PX_VAL_N, (i_pixel_y - 12'd384));
            // 后轮 y 397..467 → 中心 432，数字基址 420（420..443，中心 431.5）
            //   ★ 掉头后 左=C(字段3→读16) / 右=D(字段4→读21)
            if ((i_pixel_y >= 420) && (i_pixel_y < 444)) begin
                sm_seg[0] = seg_sm_ram(6'd16, 6'd4, PX_WL_N, (i_pixel_y - 12'd420));
                sm_seg[1] = seg_sm_ram(6'd21, 6'd4, PX_WR_N, (i_pixel_y - 12'd420));
            end

            // ★ 2026-10-18：只剩「车轮转速/rpm」用 16px 后缀（航向角已按需求方要求去掉单位）。
            //   中文与后缀的衔接：中文 408..503（右界 503），后缀从 **x=512（xw=32）** 起，留 8 px 缝。
            //   为什么能这么做：sm 段的 xw 是 16 的倍数，与 lg 的 24 网格相互独立，
            //   所以后缀位置可以单独对齐，不必"紧接"中文的 24 格边界。
            if ((i_pixel_y >= 504) && (i_pixel_y < 528))
                sm_seg[4] = seg_sm_rom(UI_TXT_LBL_RPM_ADDR, UI_TXT_LBL_RPM_LEN,
                                       6'd32, (i_pixel_y - 12'd504));
        end

        // ---- 参数整定页：11 行数值（16px，读 dyn_text_ram）+ 单位（静态 ROM）----
        //   ★ 数值复用 num_text 的字段 0..10（字段号 = 行号，基址 = 行号 × 5）：
        //     顶层在 page==PAGE_TUNE 时把 num_text 的 i_v0..i_v10 换成 tune_ctrl 的
        //     显示值，所以这里完全沿用数据页那套「数字 → 字符串 → RAM」通路，
        //     只换地址与坐标。小数点由 field_dec 统一按字段号决定，不用改。
        //   ★ 单位只对 s>0 的行画（Kp/Ki/Kd 无量纲），xw 从 30 起 ⇒ x=480。
        if (i_page == PG_TUNE) begin
            for (ti = 0; ti < TUNE_ROWS; ti = ti + 1) begin
                if ((i_pixel_y >= (TUNE_Y0 + ti * TUNE_STEP + TUNE_VDY)) &&
                    (i_pixel_y <  (TUNE_Y0 + ti * TUNE_STEP + TUNE_VDY + TUNE_VH))) begin
                    sm_seg[0] = seg_sm_ram(ti * 5, 6'd5, TUNE_VAL_XW[5:0],
                                           (i_pixel_y - (TUNE_Y0 + ti * TUNE_STEP + TUNE_VDY)));
                    // ★ 不能对 function 调用直接做 bit/part-select（Verilog-2001 非法），
                    //   先落到 tu 再取 {addr,len}。
                    tu = tune_unit_text(ti[3:0]);
                    if (tu[5:0] != 6'd0) begin
                        sm_seg[1] = seg_sm_rom(tu[17:6], tu[5:0],
                                               TUNE_UNIT_XW[5:0],
                                               (i_pixel_y - (TUNE_Y0 + ti * TUNE_STEP + TUNE_VDY)));
                    end
                end
            end
        end
    end

    reg [36:0] sm_sel;
    always @(*) begin
        sm_sel = 37'd0;
        for (si = 0; si < 6; si = si + 1) begin
            if (sm_seg[si][36] &&
                (sm_slot >= sm_seg[si][10:5]) &&
                (sm_slot <  (sm_seg[si][10:5] + sm_seg[si][16:11]))) begin
                sm_sel = sm_seg[si];
            end
        end
    end

    wire        sm_in_text = sm_sel[36];
    wire        sm_src     = sm_sel[35];
    wire [5:0]  sm_off     = sm_slot - sm_sel[10:5];
    wire [5:0]  sm_ram_rd  = sm_sel[22:17] + sm_off;
    wire [11:0] sm_rom_rd  = sm_sel[34:23] + sm_off;

    // 动态数值 RAM 的读口
    assign o_dyn_raddr = sm_ram_rd;
    wire [7:0] sm_ram_char = i_dyn_rdata;

    // ★ 2026-10-18：sm 通路的第二套字符源（静态文案 ROM），供「/°」「/rpm」16px 后缀用。
    //   这是第 2 个 ui_text_rom 实例（约 256×8 BSRAM）。lg 通路那一份不能共用 ——
    //   两条通路在同一拍可能要取不同地址。
    wire [7:0] sm_rom_char;
    ui_text_rom u_text_rom_sm (
        .i_clk        (i_pixel_clk),
        .i_addr       (sm_src ? sm_rom_rd : 12'd0),
        .o_char_index (sm_rom_char)
    );

    reg sm_in_text_d1;
    reg sm_src_d1;           // ★ 段源选择打 1 拍，与 sm_in_text_d1 同相
    reg [4:0] sm_row_d1;
    // ★ 2026-10-18：多锁两路，供小数点叠加层用（与 sm_in_text_d1 同相）
    reg [5:0] sm_xw_d1;      // 段起始字组号（字段左边界 / 16）
    reg [5:0] sm_fbase_d1;   // 段字符 RAM 基址（反查字段号 → 小数位数 dec）
    always @(posedge i_pixel_clk) begin
        sm_in_text_d1 <= sm_in_text;
        sm_src_d1     <= sm_src;
        sm_row_d1     <= sm_sel[4:0];
        sm_xw_d1      <= sm_sel[10:5];
        sm_fbase_d1   <= sm_sel[22:17];
    end
    //   src = 0 → 动态数值 RAM；src = 1 → 静态文案 ROM
    //   ★ ROM 段的 [22:17] 恒为 0，于是小数点判定的 field_dec(0) = 0（不画点），
    //     天然正确，不需要额外判断；但**别**把数值段误写成 ROM 段。
    wire [7:0] sm_char_index = sm_in_text_d1
                             ? (sm_src_d1 ? sm_rom_char : sm_ram_char)
                             : 8'd0;

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
    // 小数点叠加层（2026-10-18 新增）
    //   需求：小数点**不占字符位**，画在两个数字中间的底部。
    //
    //   为什么必须单独一层、不能靠字库：
    //     font_num 是固定 16 px 格、位选用 sm_bit_sel = 15 - i_pixel_x[3:0]，
    //     字符永远居中于 16 px 格 —— 字库里任何字形都逃不出这个网格，
    //     所以 '.' 必占一整格（这也是早先 '.' 字形被裁成 2x3 小方块、看起来像 '·' 的原因）。
    //
    //   为什么位置可以是常量：数值右对齐 + 定点小数位固定，
    //     所以小数点相对字段左边界只由「小数位数 dec」决定。
    //     个位占最后一格（pos4，offset 64），dec 位小数占最后 dec 格 ⇒ 小数点落在
    //     pos(4−dec) 与 pos(5−dec) 的交界，即 **offset = (5 − dec) × 16**：
    //       dec = 1 → 64（个位左边）
    //       dec = 2 → 48
    //       dec = 3 → 32
    //     例：中心速度 1.50 → 数字 "  150" 右对齐（pos2=1 pos3=5 pos4=0），
    //         点在 x = 896+48 = 944，正好落在 "1" 与 "5" 之间 → 显示 1.50
    //     ★ 这里差一格会让 9.99 显示成 ".999"（2026-10-18 预览图抓出的 bug，
    //       原写法 (FIELD_W−1−dec)×16 就是差一格）。
    //
    //   垂直：数字 '0' 的点阵占行 3..19（见 font_num.v），行 20..23 全空，
    //     点画在 **20..21** 正好落在字底下方 1 px，是小数点的自然位置。
    //   横向：点占 **2 px**（跨两格交界各 1 px）。'0' 的笔画占格内 x=3..12、
    //     相邻两格交界处合计 6 px 空白，2 px 的点既居中又不会碰到数字；
    //     宽度取 2 = 与数字笔画同粗（'0' 的竖笔就是 2 px），视觉上是一个标点而不是一个方块。
    //     ★ 2026-10-18 调整：初版是 3x3，需求方看预览图后要求"稍微小一点"。
    // ------------------------------------------------------------------
    function [1:0] field_dec;      // 返回该字段的小数位数（0 = 不画点）
        input [5:0] base_addr;
        begin
            // ★ 2026-10-20（综合报错后补）：参数整定页复用数据页的字段 0..11，
            //   但 12 项参数**全是整数**。若仍按 base_addr 查表，字段 6/7/8
            //   （base 30/35/40）会被画上小数点 —— 航向 Kd=6 显示成 "  0.06"。
            //   故参数页一律不画点；数据页行为完全不变。
            if (i_page == PG_TUNE) begin
                field_dec = 2'd0;
            end else begin
            case (base_addr)
                //   base 是 ui_render 里的**读取起始地址**：
                //     f6 中心速度 = 6×5 = 30     → 0.01 m/s
                //     f7 航位 x   = 7×5 = 35     → 0.01 m
                //     f8 航位 y   = 8×5 = 40     → 0.01 m
                //   角速度 f5 = 25 是整数 °/s，不需要小数点。
                6'd30:        field_dec = 2'd2;
                6'd35, 6'd40: field_dec = 2'd2;
                // ★ 2026-10-19：非避障导航页右上角的准心 x / y（字段 9 / 10，基址 45 / 50）。
                //   漏登这一条的直接现象是「数字有、小数点不画」—— 与配色/字库无关。
                6'd45, 6'd50: field_dec = 2'd2;
                default:      field_dec = 2'd0;
            endcase
            end
        end
    endfunction

    wire [1:0]  sm_dec     = field_dec(sm_fbase_d1);
    //   ★ 位宽：{6'd0, sm_dec, 4'd0} 恰好 12 位 = sm_dec × 16（写 {8'd0,...} 会成 14 位被截断）
    //   ★ 位置 = (FIELD_W − dec) × 16 = 80 − dec×16：dec=2→48, dec=3→32（不是 (4−dec)×16）
    wire [11:0] sm_dp_x    = 12'd80 - {6'd0, sm_dec, 4'd0};
    wire [11:0] sm_fld_x   = {sm_xw_d1, 4'b0000};             // 字段左边界像素
    wire [11:0] sm_rel     = (i_pixel_x >= sm_fld_x) ? (i_pixel_x - sm_fld_x) : 12'd0;
    //   sm_rel ∈ {dp_x−1, dp_x} 时画点（2 px 宽，跨交界各 1 px）
    wire        sm_dot_col = (i_pixel_x >= sm_fld_x) &&
                             (sm_rel + 12'd1 >= sm_dp_x) &&
                             (sm_rel         <= sm_dp_x);
    wire        sm_dot     = sm_in_text_d1 && (sm_dec != 2'd0) &&
                             (sm_row_d1 >= 5'd20) && (sm_row_d1 <= 5'd21) &&
                             sm_dot_col;

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

    wire car_pix = (i_page == PG_DATA) &&
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
    wire wheel_box = (i_page == PG_DATA) && (box_lf || box_rf || box_lr || box_rr);

    // ★★ 2026-10-18（第四批）：只给【航向角数字】加底色块，标签不加 ★★
    //   需求方："航向角/°不要加背景框，只要航向角数字加。"
    //   数字 4 字符 = 64 px（x 464..527）、24 高（y 112..135），四周 8 px 内边距：
    //     x 456..535（宽 **80**，中心 **495.5** ≈ 车体中心 496）
    //     y 104..143（高 **40**，与轮速框同尺寸 80×40）
    //   颜色沿用轮速文本框的 24'h2E3440。
    wire yaw_box = (i_page == PG_DATA) &&
                   (i_pixel_x >= 12'd456) && (i_pixel_x <= 12'd535) &&
                   (i_pixel_y >= 12'd104) && (i_pixel_y <= 12'd143);

    // ==================================================================
    // 非避障导航页几何（2026-10-19）
    //
    //   屏幕几何真源（与 rtl\ui\nav_free_ctrl.v 头部注释一致，**改一处必须两处同改**）：
    //     内容区 x 208..1023 × y 0..599；原点 **(412, 300)**；
    //     纵向 ±5 m 铺满上下各 300 px ⇒ 60 px/m；横向同比例
    //     ⇒ x 可见 −3400 … +10183 mm、y 可见 −4983 … +5000 mm。
    //
    //   ★ 全部是「像素坐标比较」，不用字库、不用乘法：点与绿点都是圆，用离散圆判据。
    //     不用 dx²+dy² —— 16 个点 × 2 个平方器会把组合路径拉长，而 pixel 域只有 1 拍。
    // ==================================================================
    localparam [11:0] NAV_ORG_X = 12'd412;      // 原点像素 x = 208 + 816/4
    localparam [11:0] NAV_ORG_Y = 12'd300;      // 原点像素 y = 600/2

    // ★ 2026-10-19：避障页与非避障页**共用**同一套状态机与菜单变暗效果
    wire nav_pg_any = (i_page == PG_NAV_FREE) || (i_page == PG_NAV_OBS);
    wire nav_ui     = nav_pg_any && i_menu_dim;   // 导航页且处于交互态
    wire nav_pg     = (i_page == PG_NAV_FREE);    // 只有非避障页有坐标系/准心/点

    // ★ 2026-10-20：参数整定页也要把左侧菜单栏**变暗** —— 进页后摇杆/按键都归
    //   参数编辑，不再切菜单，用亮度提示和导航交互态一致。
    wire tune_ui    = (i_page == PG_TUNE) && i_tune_active;

    // ---- 坐标轴：只画轴、不画网格；都延伸到画面边缘 ----
    //   x 轴 y=300，从菜单右缘 208 到 1023；y 轴 x=412，全高 0..599。
    wire nav_axis_x = nav_pg && (i_pixel_y == NAV_ORG_Y) && (i_pixel_x >= MENU_W);
    wire nav_axis_y = nav_pg && (i_pixel_x == NAV_ORG_X);

    // ---- 已记录点：直径 **6 px**（2026-10-19 需求方上调）圆 = 29 px 离散圆 ----
    //   |dy|=0→|dx|≤3；|dy|=1→≤2；|dy|=2→≤2；|dy|=3→=0
    wire [15:0] nav_pt_hit;
    genvar gi;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_navpt
            wire signed [12:0] ndx = $signed({1'b0, i_pixel_x}) - $signed({1'b0, i_pt_px[gi*10 +: 10]});
            wire signed [12:0] ndy = $signed({1'b0, i_pixel_y}) - $signed({1'b0, i_pt_py[gi*10 +: 10]});
            wire [12:0] adx = ndx[12] ? (13'd0 - ndx) : ndx;
            wire [12:0] ady = ndy[12] ? (13'd0 - ndy) : ndy;
            assign nav_pt_hit[gi] = i_pt_v[gi] &&
                   (((ady == 13'd0) && (adx <= 13'd3)) ||
                    ((ady == 13'd1) && (adx <= 13'd2)) ||
                    ((ady == 13'd2) && (adx <= 13'd2)) ||
                    ((ady == 13'd3) && (adx == 13'd0)));
        end
    endgenerate
    wire nav_point = nav_pg && (|nav_pt_hit);

    // ---- 车绿点：直径 **12 px**（2026-10-19 需求方上调）圆 = 113 px 离散圆 ----
    //   |dy|=0→|dx|≤6；1..3→≤5；4→≤4；5→≤3；6→=0
    //   ★ 仍用比较式而不是 dx²+dy²：pixel 域只有 1 拍，乘法会把这条路径拉长。
    wire signed [12:0] cdx = $signed({1'b0, i_pixel_x}) - $signed({1'b0, i_car_px});
    wire signed [12:0] cdy = $signed({1'b0, i_pixel_y}) - $signed({1'b0, i_car_py});
    wire [12:0] cadx = cdx[12] ? (13'd0 - cdx) : cdx;
    wire [12:0] cady = cdy[12] ? (13'd0 - cdy) : cdy;
    wire nav_car = nav_pg &&
                   (((cady == 13'd0) && (cadx <= 13'd6)) ||
                    ((cady == 13'd1) && (cadx <= 13'd5)) ||
                    ((cady == 13'd2) && (cadx <= 13'd5)) ||
                    ((cady == 13'd3) && (cadx <= 13'd5)) ||
                    ((cady == 13'd4) && (cadx <= 13'd4)) ||
                    ((cady == 13'd5) && (cadx <= 13'd3)) ||
                    ((cady == 13'd6) && (cadx == 13'd0)));

    // ---- 十字准心：1 px 粗、臂长 **10 px**（总宽 21 px）----
    //   ★ 2026-10-19：绿点从 6 px 增到 12 px 后，原臂长 8（总 17）只比绿点宽 2.5 px，
    //     准心落在绿点上时几乎看不见 —— 加到 10 后每侧伸出 4 px，两者都清晰。
    wire signed [12:0] kdx = $signed({1'b0, i_pixel_x}) - $signed({1'b0, i_cur_px});
    wire signed [12:0] kdy = $signed({1'b0, i_pixel_y}) - $signed({1'b0, i_cur_py});
    wire [12:0] kadx = kdx[12] ? (13'd0 - kdx) : kdx;
    wire [12:0] kady = kdy[12] ? (13'd0 - kdy) : kdy;
    wire nav_cross = nav_pg &&
                     (((kady == 13'd0) && (kadx <= 13'd10)) ||
                      ((kadx == 13'd0) && (kady <= 13'd10)));

    // ---- 右下角辅助菜单窗口（背景 832..1023 × 414..599；项框 420 + sel*36，高 36）----
    wire [11:0] nav_aux_top = NAV_AUX_Y0 + i_aux_sel * NAV_AUX_STEP;
    wire nav_aux_bg = nav_pg &&
        (i_pixel_x >= 12'd832) && (i_pixel_x <= 12'd1023) &&
        (i_pixel_y >= 12'd414) && (i_pixel_y <= 12'd599);
    wire nav_aux_hi = nav_pg &&
        (i_pixel_x >= 12'd832) && (i_pixel_x <= 12'd1023) &&
        (i_pixel_y >= nav_aux_top) && (i_pixel_y < (nav_aux_top + NAV_AUX_STEP));

    // ---- 参数整定页：选中行高亮条（x 224..1023，行高 36）----
    //   ★ 起点取 224 而不是 208：左侧菜单条占 x<208，留 16 px 缝，视觉上像"行选中"。
    //   ★ i_tune_sel 来自 tune_ctrl 的 o_sel；非参数页时该信号无意义，用 i_page 门控。
    //   ★ 2026-10-20：右边界收在 656（= 41 字组）。参数页内容最右是单位段
    //     TUNE_UNIT_XW=30 起 4 字 = x 480..543，整行铺到 1023 会留下 480 px
    //     纯蓝空条，视觉上把行高亮读成了"标题栏"。656 给单位右侧留 113 px 余量。
    localparam [11:0] TUNE_HL_X1 = 12'd656;
    wire tune_hi_band = (i_page == PG_TUNE) && i_tune_active &&
                        (i_pixel_x >= 12'd224) &&
                        (i_pixel_x <  TUNE_HL_X1) &&
                        (i_pixel_y >= (TUNE_Y0 + i_tune_sel * TUNE_STEP)) &&
                        (i_pixel_y <  (TUNE_Y0 + i_tune_sel * TUNE_STEP + TUNE_STEP));

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
        // 1) 底色（★ 状态栏优先：它的 x 范围落在菜单条内）
        //   ★ 2026-10-18：状态栏底色随【模式】变化（需求方要求）：
        //       手动接管档（i_ui_mode=0）→ 深红 24'h660000（提示"车会动"；初版 8B0000 偏亮，已加深）
        //       屏幕光标档（i_ui_mode=1）→ 保持原有 24'h202020
        //     判据用 i_ui_mode 而不是状态栏文案：急停/无信号/请回中位时状态栏显示的是
        //     别的词，但"当前处于哪个模式"没变，底色应始终反映模式。
        //   ★ 2026-10-19：非避障导航页进入寻点运行态时，状态栏底色**变绿**
        //     （需求方指示；24'h006600 与手动档 24'h660000 同强度、只换通道）。
        //   ★ 同一页里菜单栏**变暗**：摇杆此时控制准心、不再切菜单，用亮度提示。
        //   三态底色（2026-10-19）：导航暂停 994D00（浅橙）> 导航中 006600（绿）
        //                          > 手动 660000（深红）> 光标档 202020
        if (in_status_bar)      rgb = (nav_ui && i_nav_paused) ? 24'h994D00
                                    : (nav_ui && i_nav_run)    ? 24'h006600
                                    : ((i_ui_mode == 2'd0) ? 24'h660000 : 24'h202020);
        else if (in_menu_bar)   rgb = (nav_ui || tune_ui) ? 24'h0B1019 : 24'h101828;
        else                    rgb = 24'h000000;

        // 1b) 轮速文本框底色（盖在内容区黑底之上；文字与车图仍在其上）
        //     24'h2E3440 = 深版 22262E 与浅版 3A4252 的逐通道中点（需求方选定）。
        //       R: (34+58)/2=46   G: (38+66)/2=52   B: (46+82)/2=64
        //     比纯黑明显可辨，又不至于亮到抢走数字的注意力；与菜单条 101828 也能区分。
        if (wheel_box || yaw_box) rgb = 24'h2E3440;

        // 2) 菜单选中高亮
        if (menu_sel_band)      rgb = 24'h1D4ED8;
        if (tune_hi_band)       rgb = 24'h1D4ED8;

        // 2b) 非避障导航页：辅助菜单窗口底 / 选中项底 / 坐标轴 / 已记录点 / 车绿点
        //     ★ 全部排在「大字 / 数字」之前 —— 文字压在它们上面才可读；
        //       只有十字准心放到最后（见 6b），保证准心永远可见。
        if (nav_aux_bg)                rgb = 24'h101828;
        if (nav_aux_hi)                rgb = 24'h1D4ED8;
        if (nav_axis_x || nav_axis_y)  rgb = 24'h5A6470;   // 轴：中性灰蓝，不与白点混淆
        if (nav_point)                 rgb = 24'hFFFFFF;   // 已记录点
        if (nav_car)                   rgb = 24'h00C000;   // 车绿点（暗一档：纯绿 00FF00 上叠白十字对比不足）

        // 3) 大字（菜单 / 状态栏 / 中文标签 / 占位提示）
        if (lg_glyph) begin
            if (lg_imu_fail_d1)                     rgb = 24'hFF4040;   // IMU 初始化失败：红
            else if (in_status_bar)                 rgb = 24'hFFFFFF;   // 状态栏：白
            else if (in_menu_bar && menu_sel_band)  rgb = (nav_ui || tune_ui) ? 24'h8A7A2E : 24'hFFE066;   // 选中项
            else if (in_menu_bar)                   rgb = (nav_ui || tune_ui) ? 24'h5A6170 : 24'hE5E7EB;   // 未选中
            else                                    rgb = 24'hFFFFFF;   // 标签/占位：白
        end

        // 4) 紧凑数字（数据页实时数值，16x24）+ 悬挂小数点（同色，2026-10-18）
        //   ★ 2026-10-18：sm 段有两种源 —— RAM 源是**实时数值**（亮绿），
        //     ROM 源是**标签的符号后缀**（如「/°」「/rpm」，应与中文同为白色）。
        //     若不加这个区分，后缀会被画成绿色，与同一行的中文标签不同色。
        if (sm_glyph || sm_dot)                     rgb = sm_src_d1 ? 24'hFFFFFF : 24'h7CFC98;

        // 6b) 非避障导航页：十字准心（最上层）
        if (nav_cross)                              rgb = 24'hFFFFFF;

        // 5) 数据页小车俯视简笔图（白色 = 与标签文字同色；放最后 = 最上层）
        if (car_pix)                                rgb = 24'hFFFFFF;
    end

    // pixel_req 为高的那一拍，hdmi_1024x600 会采样本信号 → 必须组合输出
    assign o_pixel_rgb888 = i_video_active ? rgb : 24'h000000;

endmodule
