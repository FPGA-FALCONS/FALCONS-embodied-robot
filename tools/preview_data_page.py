#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""preview_data_page.py —— 按 ui_render.v 的真实布局，离屏渲染「运行数据页」预览图

为什么需要这个脚本：
  * 数据页的数值字段在 2026-10-18 由 4 字符扩到 5 字符（符号 + 4 位数字），
    并新增了**悬挂小数点**（不占字符位，画在两格交界的底部）。
  * 这类改动最怕"数字撞到中文标签 / 轮速框"——只有把像素画出来才能一眼确认，
    靠读代码算间隙容易漏掉某个前导空格或负号位置。
  * 点阵数据**直接读 rtl/common/font_num.v 与 font_rom_lg.v**（不是重画字体），
    所以预览与上板效果共用同一份字库，能真实反映字宽、笔画位置与空白。

本脚本是只读的：不生成/不修改任何 RTL，只输出 PNG。

用法：
    python tools/preview_data_page.py
    python tools/preview_data_page.py --out doc/img/data_page_fullscale.png
    python tools/preview_data_page.py --preset typical          # 典型行驶值
    python tools/preview_data_page.py --center 999 --pose-x 9999 --gyro -200

布局常量的出处（改布局时请同步这里，否则预览会失真）：
    rtl/ui/ui_render.v   PX_LBL=33  PX_YAW_N=29  PX_WL_N=16  PX_WR_N=42  PX_VAL_N=56
                         轮速框 248..327 / 664..743，前 240..279，后 412..451
                         车图与四轮：见该文件 L412..431
    rtl/ui/num_text.v    FIELD_W=5，4 字符字段从 base+1 读，5 字符字段从 base+0 读
"""

import argparse
import os
import re
import sys

import numpy as np
from PIL import Image

SCREEN_W, SCREEN_H = 1024, 600

# ---------------------------------------------------------------------------
# 颜色（与 ui_render.v 的 rgb 赋值一一对应）
# ---------------------------------------------------------------------------
C_BG        = (0x00, 0x00, 0x00)   # 内容区黑底
C_MENU_BG   = (0x10, 0x18, 0x28)   # 左侧菜单条
C_MENU_SEL  = (0x1D, 0x4E, 0xD8)   # 菜单高亮
C_STATUS_BG = (0x66, 0x00, 0x00)   # 状态栏（手动模式 = 深红）
C_TEXT      = (0xFF, 0xFF, 0xFF)   # 菜单未选中 / 标签 / 车图
C_TEXT_SEL  = (0xFF, 0xE0, 0x66)   # 菜单选中项（亮黄）
C_NUM       = (0x7C, 0xFC, 0x98)   # 实时数值（亮绿）
C_BOX       = (0x2E, 0x34, 0x40)   # 轮速文本框底色
# ★ 2026-10-19：IMU 初始化失败提示（= ui_render.v 的 24'hFF4040）
#   几何：8 字 × 24 = 192 px，x 384..575，y 48..71 —— 「航向角」标签（456..527, y 72..95）正上方
C_IMU_FAIL  = (0xFF, 0x40, 0x40)
IMU_FAIL_X, IMU_FAIL_Y = 384, 48

# ---------------------------------------------------------------------------
# 布局常量（ui_render.v）
# ---------------------------------------------------------------------------
MENU_W      = 208
LG_CELL     = 24
LG_XW       = 1          # 菜单文字 x = 24
MENU_Y0     = 72
MENU_STEP   = 72
STATUS_Y    = 552
PAGE_Y      = 264        # 占位提示 y（= ui_render.v 的 PAGE_Y）
PAGEXW      = 9          # 占位提示起始字组（x = 9 * 24 = 216）
PX_LBL      = 33         # 标签列 x = 792
PX_YAW_N    = 29         # 偏航角 x = 464
PX_WL_N     = 16         # 左轮速 x = 256
PX_WR_N     = 42         # 右轮速 x = 672
PX_VAL_N    = 56         # 列表数值 x = 896（5 字符，右边缘 975）

# 数值 16 px 格 / 标签 24 px 格
NUM_W, NUM_H = 16, 24
LG_W,  LG_H  = 24, 24


# ---------------------------------------------------------------------------
# 字库解析：从生成的 Verilog ROM 里取点阵（文本里内嵌，不需要 $readmemh）
# ---------------------------------------------------------------------------
RE_MEM = re.compile(
    r"mem\[\s*(\d+)\]\s*=\s*(\d+)'h([0-9A-Fa-f]+)\s*;\s*//\s*U\+([0-9A-Fa-f]{4})")


def parse_font(path, width, height):
    """返回 {码点: [每行位图 int, ...]}，以及 {字库索引: 码点}。

    ROM 地址约定：addr = char_index * height + row（与 RTL 一致）。
    同一字符的每一行注释里都带 U+XXXX，所以从任一行都能拿到码点。
    """
    glyphs = {}
    index_of = {}
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = RE_MEM.search(line)
            if not m:
                continue
            addr = int(m.group(1))
            bits = int(m.group(3), 16)
            code = int(m.group(4), 16)
            ci, row = divmod(addr, height)
            index_of[ci] = code
            glyphs.setdefault(code, [0] * height)[row] = bits
    return glyphs, index_of


RE_TXT_MEM = re.compile(r"mem\[\s*(\d+)\]\s*=\s*8'd(\d+)\s*;")
RE_TXT_PAR = re.compile(r"localparam\s+\[11:0\]\s+UI_TXT_(\w+)_ADDR\s*=\s*12'd(\d+)\s*;")
RE_TXT_LEN = re.compile(r"localparam\s+\[5:0\]\s+UI_TXT_(\w+)_LEN\s*=\s*6'd(\d+)\s*;")


def parse_text_rom(rom_path, index_path):
    """返回 ({名: (addr, len)}, [字符索引序列])"""
    stream = {}
    with open(rom_path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = RE_TXT_MEM.search(line)
            if m:
                stream[int(m.group(1))] = int(m.group(2))
    addrs, lens = {}, {}
    with open(index_path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = RE_TXT_PAR.search(line)
            if m:
                addrs[m.group(1)] = int(m.group(2))
            m = RE_TXT_LEN.search(line)
            if m:
                lens[m.group(1)] = int(m.group(2))
    items = {k: (addrs[k], lens[k]) for k in addrs if k in lens}
    return items, stream


# ---------------------------------------------------------------------------
# 画布
# ---------------------------------------------------------------------------
class Canvas:
    def __init__(self):
        self.img = np.zeros((SCREEN_H, SCREEN_W, 3), dtype=np.uint8)
        self.img[:, :] = C_BG

    def rect(self, x0, y0, x1, y1, color):
        x0 = max(0, x0); y0 = max(0, y0)
        x1 = min(SCREEN_W - 1, x1); y1 = min(SCREEN_H - 1, y1)
        if x0 <= x1 and y0 <= y1:
            self.img[y0:y1 + 1, x0:x1 + 1] = color

    def glyph(self, rows, x, y, width, color):
        """把一张点阵贴到 (x, y)；bit(width-1) = 最左像素"""
        for r, bits in enumerate(rows):
            yy = y + r
            if not (0 <= yy < SCREEN_H) or not bits:
                continue
            for c in range(width):
                if bits & (1 << (width - 1 - c)):
                    xx = x + c
                    if 0 <= xx < SCREEN_W:
                        self.img[yy, xx] = color

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        Image.fromarray(self.img, "RGB").save(path)


# ---------------------------------------------------------------------------
# num_text 的格式化复现（必须与 RTL 完全一致，否则预览会骗人）
# ---------------------------------------------------------------------------
def fmt_field(v, min_digits=1):
    """复现 num_text.v 的 5 格格式化：'符号 + 千 + 百 + 十 + 个'，符号跟随最高位左移一格。

    与 RTL 的 S_W0..S_W4 + started_*/npos/min_dig 一一对应。

    ★ 前导零抑制按"高位是否已开始"判断，**不是**按"该位是否为 0"——
      否则 400 会变成 " 4 0"、-200 会变成 "-2 0"（2026-10-18 预览图抓出的 bug）。
    ★ min_digits = 最少显示的数字位数（= num_text 的 min_dig）：
      带小数的字段（中心速度 / 航位，dec=2）必须 ≥ 3，否则 0.00 会显示成 "   .0"。
    """
    v = max(-9999, min(9999, v))
    sign = v < 0
    m = abs(v)
    d_th, m = divmod(m, 1000)
    d_h, m = divmod(m, 100)
    d_t, m = divmod(m, 10)
    d_o = m

    started_h = (d_th != 0) or (min_digits >= 4)
    started_t = started_h or (d_h != 0) or (min_digits >= 3)
    started_o = started_t or (d_t != 0) or (min_digits >= 2)
    npos = 0 if started_h else (1 if started_t else (2 if started_o else 3))

    pos0 = '-' if (sign and npos == 0) else ' '
    pos1 = str(d_th) if started_h else ('-' if (sign and npos == 1) else ' ')
    pos2 = str(d_h) if started_t else ('-' if (sign and npos == 2) else ' ')
    pos3 = str(d_t) if started_o else ('-' if (sign and npos == 3) else ' ')
    pos4 = str(d_o)
    return pos0 + pos1 + pos2 + pos3 + pos4


def draw_num(cv, glyphs, text5, x, y, nchars):
    """画一个数值字段。

    nchars=4 → 取 text5[1:5]（跳过符号格，与 ui_render 的 base=字段号×5+1 对应）
    nchars=5 → 取 text5[0:5]
    """
    s = text5[1:5] if nchars == 4 else text5[0:5]
    for i, ch in enumerate(s):
        code = ord(ch)
        if code in glyphs:
            cv.glyph(glyphs[code], x + i * NUM_W, y, NUM_W, C_NUM)


def draw_dot(cv, base_x, y0, dec):
    """悬挂小数点：与 ui_render 的 sm_dot 等价。

    横向：dp_x = (FIELD_W − dec) × 16（个位在 offset 64，dec 位小数占最后 dec 格）
          dec=1→64, dec=2→48, dec=3→32；点占 **2 px**（dp_x−1 .. dp_x，跨两格交界各 1 px）
    纵向：**row 20..21**（数字 '0' 占行 3..19）；2 px 宽 = 与数字笔画同粗
    ★ 初版是 3x3，需求方看预览后要求"稍微小一点"。
    """
    if dec == 0:
        return
    dp_x = (5 - dec) * NUM_W
    for dy in range(20, 22):
        for dx in (-1, 0):
            xx = base_x + dp_x + dx
            yy = y0 + dy
            if 0 <= xx < SCREEN_W and 0 <= yy < SCREEN_H:
                cv.img[yy, xx] = C_NUM


# ---------------------------------------------------------------------------
# 静态部件
# ---------------------------------------------------------------------------
def draw_menu(cv, lg, text_items, stream, index_of, sel=0,
              status_bg=None, menu_dim=False, status_name="ST_MANUAL"):
    """左侧菜单条 + 状态栏。

    menu_dim=True：非避障导航页专用 —— 摇杆此时控制准心、不再切菜单，
                   菜单底色与文字整体变暗（与 ui_render.v 的 nav_pg 分支一致）。
    status_bg   ：状态栏底色；None = 手动档深红。
    """
    bg      = (0x0B, 0x10, 0x19) if menu_dim else C_MENU_BG
    fg      = (0x5A, 0x61, 0x70) if menu_dim else C_TEXT
    fg_sel  = (0x8A, 0x7A, 0x2E) if menu_dim else C_TEXT_SEL
    cv.rect(0, 0, MENU_W - 1, SCREEN_H - 1, bg)

    # 高亮项（sel=0 = 运行数据显示：2026-10-19 起它是菜单第 1 项，也是上电默认页）
    cv.rect(0, MENU_Y0 + sel * MENU_STEP - 12,
            MENU_W - 1, MENU_Y0 + sel * MENU_STEP + 35, C_MENU_SEL)

    # ★ 2026-10-19：菜单 4 项 → 5 项，顺序必须与 rtl/ui/ui_render.v 的 menu_text() 一致。
    #   原第 4 项「路径点导航」在需求原文里拆成「避障导航」+「非避障导航」。
    names = ["MENU_VEHICLE", "MENU_CAMERA", "MENU_RADAR", "MENU_NAV_OBS", "MENU_NAV_FREE"]
    for i, nm in enumerate(names):
        if nm not in text_items:
            continue
        addr, length = text_items[nm]
        y = MENU_Y0 + i * MENU_STEP
        for k in range(length):
            ci = stream.get(addr + k)
            if ci is None:
                continue
            code = index_of.get(ci)
            if code is None or code not in lg:
                continue
            cv.glyph(lg[code], LG_XW * LG_CELL + k * LG_W, y, LG_W,
                     fg_sel if i == sel else fg)

    # 状态栏（手动模式）
    cv.rect(0, STATUS_Y - 12, MENU_W - 1, STATUS_Y + 35,
            C_STATUS_BG if status_bg is None else status_bg)
    if status_name in text_items:
        addr, length = text_items[status_name]
        for k in range(length):
            ci = stream.get(addr + k)
            if ci is None:
                continue
            code = index_of.get(ci)
            if code is None or code not in lg:
                continue
            cv.glyph(lg[code], LG_XW * LG_CELL + k * LG_W, STATUS_Y, LG_W, C_TEXT)


def draw_text_label(cv, glyphs, index_of, text_items, stream, name, x, y, cell,
                    color=None):
    """按指定格宽画一条静态标签（混排用）。

    cell=24 → 中文走 font_rom_lg；cell=16 → 符号/字母走 font_num。
    两者共用同一份字符索引（ui_text_rom），所以只需换字库与格宽。
    """
    if name not in text_items:
        print(f"  [警告] 字表里没有 {name}，跳过")
        return
    addr, length = text_items[name]
    for k in range(length):
        ci = stream.get(addr + k)
        if ci is None:
            continue
        code = index_of.get(ci)
        if code is None or code not in glyphs:
            continue
        cv.glyph(glyphs[code], x + k * cell, y, cell, C_TEXT if color is None else color)


def draw_placeholder(cv, lg, index_of, text_items, stream, name):
    """占位页：内容区黑底 + 占位提示（位置与 ui_render.v 的 PAGE_Y / PAGEXW 一致）"""
    draw_text_label(cv, lg, index_of, text_items, stream, name,
                    PAGEXW * LG_CELL, PAGE_Y, LG_CELL)


# =============================================================================
# 非避障导航页（2026-10-19）
#   几何真源 = rtl/ui/nav_free_ctrl.v 头部注释 + ui_render.v 的 NAV_* 常量：
#     原点 (412, 300)、0.06 px/mm、轴延伸到画面边缘、辅助菜单 840 / 420 / 36。
# =============================================================================
NAV_ORG_X, NAV_ORG_Y = 412, 300
NAV_PPM   = 0.06                      # px per mm
C_AXIS    = (0x5A, 0x64, 0x70)
C_GREEN   = (0x00, 0xC0, 0x00)   # 车绿点：暗一档，白十字叠上去才看得清
C_RUN_BG  = (0x00, 0x66, 0x00)        # 导航中状态栏（= ui_render 的 24'h006600）
C_PAUSE_BG = (0x99, 0x4D, 0x00)       # 导航暂停状态栏（= ui_render 的 24'h994D00）
NAV_AUX_X, NAV_AUX_Y0, NAV_AUX_STEP, NAV_AUX_TDY = 840, 420, 36, 6
NAV_AUX_NAMES = ["AUX_SET_POINT", "AUX_UNDO_POINT",
                 "AUX_POINT_GAP", "AUX_POINT_CONT", "AUX_CLEAR_ALL"]


def nav_px(xmm):
    return NAV_ORG_X + int(round(xmm * NAV_PPM))


def nav_py(ymm):
    return NAV_ORG_Y - int(round(ymm * NAV_PPM))


def _put(cv, x, y, color):
    if 0 <= x < SCREEN_W and 0 <= y < SCREEN_H:
        cv.img[y, x] = color


def _disk(cv, cx, cy, r, color):
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                _put(cv, cx + dx, cy + dy, color)


def _cross(cv, cx, cy, arm, color):
    for d in range(-arm, arm + 1):
        _put(cv, cx + d, cy, color)
        _put(cv, cx, cy + d, color)


def draw_nav_page(cv, lg, index_of, text_items, stream, num_g, num_idx,
                  cur=(0, 0), car=(0, 0), pts=(), aux=0):
    """坐标系 + 车绿点 + 十字准心 + 已记录点 + 右下辅助菜单 + 右上角坐标。"""
    # 坐标轴（只画轴，不画网格；延伸到画面边缘）
    for x in range(MENU_W, SCREEN_W):
        _put(cv, x, NAV_ORG_Y, C_AXIS)
    for y in range(0, SCREEN_H):
        _put(cv, NAV_ORG_X, y, C_AXIS)

    # 已记录点（直径 6 px 圆，2026-10-19 需求方上调：半径 3）
    for (px_, py_) in pts:
        _disk(cv, nav_px(px_), nav_py(py_), 3, C_TEXT)

    # 车绿点（直径 12 px 圆，2026-10-19 需求方上调：半径 6）
    _disk(cv, nav_px(car[0]), nav_py(car[1]), 6, C_GREEN)

    # 十字准心（臂长 10 px，总宽 21 px —— 绿点 12 px 后每侧伸出 4 px）
    _cross(cv, nav_px(cur[0]), nav_py(cur[1]), 10, C_TEXT)

    # 右下角辅助菜单（背景 832..1023 × 414..599；项框 420+sel*36）
    cv.rect(832, 414, 1023, 599, C_MENU_BG)
    top = NAV_AUX_Y0 + aux * NAV_AUX_STEP
    cv.rect(832, top, 1023, top + NAV_AUX_STEP - 1, C_MENU_SEL)
    for i, nm in enumerate(NAV_AUX_NAMES):
        draw_text_label(cv, lg, index_of, text_items, stream, nm,
                        NAV_AUX_X, NAV_AUX_Y0 + i * NAV_AUX_STEP + NAV_AUX_TDY, 24,
                        C_TEXT_SEL if i == aux else C_TEXT)

    # 右上角准心坐标：前缀 + 5 字符数值 + "m"，右界 975
    #   2026-10-19 需求方样例「x:  2.60m」并要求"右侧不要紧贴、留出合适间距"：
    #   前缀 2 字符（x=864，组 54）+ 数值 5 字符（x=880，组 55）+ "m" 1 字符（x=960，组 60）
    #   → 右界 975，离右边缘 48 px，且与「运行数据显示」页数值列右界对齐。
    #   冒号后两个空格 = 1 个分隔 + 数值的 1 个符号位。
    for (_pfx, _val, _y) in (("NAV_COORD_X", cur[0], 24), ("NAV_COORD_Y", cur[1], 56)):
        draw_text_label(cv, num_g, num_idx, text_items, stream, _pfx, 864, _y, 16, C_TEXT)
        draw_num(cv, num_g, fmt_field(int(round(_val / 10.0)), 3), 912, _y, 5)
        draw_dot(cv, 912, _y, 2)
        draw_text_label(cv, num_g, num_idx, text_items, stream, "NAV_COORD_M", 992, _y, 16, C_TEXT)


def draw_car(cv):
    """小车俯视简笔图（几何判定与 ui_render.v L412..431 完全一致）"""
    for y in range(160, 492):
        for x in range(395, 598):
            car_out = (y >= 203) or (x >= 598 - y and x <= 394 + y)
            car_in = (399 <= x <= 593 and 164 <= y <= 487 and
                      ((y >= 207) or (x >= 606 - y and x <= 386 + y)))
            if car_out and not car_in:
                cv.img[y, x] = C_TEXT

    for (x0, x1, y0, y1) in ((352, 388, 225, 294), (604, 640, 225, 294),
                             (352, 388, 397, 467), (604, 640, 397, 467)):
        cv.rect(x0, y0, x1, y1, C_TEXT)


def draw_wheel_boxes(cv):
    """四个轮速文本框（数字 4 字符 64 px + 四周 8 px 内边距 = 80×36）"""
    for (x0, y0) in ((248, 240), (664, 240), (248, 412), (664, 412)):
        cv.rect(x0, y0, x0 + 79, y0 + 39, C_BOX)


def draw_yaw_box(cv):
    """航向角「标签 + 数字」的底色块（与轮速框同色 2E3440）。

    只包【数字】(464..527, y112..135)，四周 8 px 内边距：
      x 456..535（宽 80，中心 495.5 ≈ 车体中心 496）、y 104..143
    """
    cv.rect(456, 104, 535, 143, C_BOX)


# ---------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    rtl_common = os.path.join(root, "rtl", "common")

    ap = argparse.ArgumentParser(description="离屏渲染运行数据页预览图")
    ap.add_argument("--out", default=os.path.join(root, "doc", "img", "data_page_fullscale.png"))
    ap.add_argument("--preset", choices=["fullscale", "typical", "zero"], default="fullscale")
    ap.add_argument("--yaw",    type=int, default=None, help="航向角（度）")
    ap.add_argument("--w0",     type=int, default=None, help="A 右后轮速（rpm）")
    ap.add_argument("--w1",     type=int, default=None, help="B 左后轮速（rpm）")
    ap.add_argument("--w2",     type=int, default=None, help="C 右前轮速（rpm）")
    ap.add_argument("--w3",     type=int, default=None, help="D 左前轮速（rpm）")
    ap.add_argument("--gyro",   type=int, default=None, help="角速度（°/s，整数）")
    ap.add_argument("--center", type=int, default=None, help="中心速度（0.01 m/s）")
    ap.add_argument("--pose-x", type=int, default=None, help="航位 x（0.01 m）")
    ap.add_argument("--pose-y", type=int, default=None, help="航位 y（0.01 m）")
    ap.add_argument("--menu-sel", type=int, default=0,
                    help="左侧菜单高亮项 0..4（0=运行数据显示 … 4=非避障导航）")
    ap.add_argument("--page", type=int, default=0, choices=[0, 1, 2, 3, 4],
                    help="要预览的页面 0=运行数据显示(默认) 1=摄像头 2=雷达 3=避障导航 4=非避障导航")
    # ---- page 4（非避障导航页）专用 ----
    ap.add_argument("--cur-x", type=int, default=0, help="准心 x（mm）")
    ap.add_argument("--cur-y", type=int, default=0, help="准心 y（mm）")
    ap.add_argument("--car-x", type=int, default=0, help="车航位 x（mm）")
    ap.add_argument("--car-y", type=int, default=0, help="车航位 y（mm）")
    ap.add_argument("--aux",   type=int, default=0, choices=[0, 1, 2, 3, 4],
                    help="辅助菜单高亮 0确认路径点 1撤销 2间断寻点 3非间断 4清除")
    ap.add_argument("--run",   type=int, default=0, choices=[0, 1],
                    help="1 = 寻点任务运行中（状态栏变绿 + 显示导航中）")
    ap.add_argument("--nav-state", type=int, default=0, choices=[0, 1, 2],
                    help="导航页状态 0=SETUP 1=导航中(绿) 2=导航暂停(橙)")
    ap.add_argument("--pts",   type=str, default="",
                    help="已记录点，形如 \"800,500;1600,-400;2600,1200\"（mm）")
    ap.add_argument("--imu-fail", type=int, default=0, choices=[0, 1],
                    help="1 = 模拟 IMU 初始化失败（数据页「航向角」上方红色大字提示）")
    args = ap.parse_args()

    num_g, num_idx = parse_font(os.path.join(rtl_common, "font_num.v"), NUM_W, NUM_H)
    lg_g,  lg_idx  = parse_font(os.path.join(rtl_common, "font_rom_lg.v"), LG_W, LG_H)
    text_items, stream = parse_text_rom(
        os.path.join(rtl_common, "ui_text_rom.v"),
        os.path.join(root, "rtl", "ui", "ui_text_index.vh"))

    # ---- 各字段取值 ----
    if args.preset == "fullscale":
        # 每个字段都取显示上限，专门用来验证"最宽情形"不会撞标签/撞框
        vals = dict(yaw=180, w=(400, -400, 400, -400),
                    gyro=-200, center=999, pose_x=9999, pose_y=-9999)
    elif args.preset == "zero":
        # 全 0：验证"0 也要显示成 0.00"（小数点左边的 0 必须补出来）
        vals = dict(yaw=0, w=(0, 0, 0, 0),
                    gyro=0, center=0, pose_x=0, pose_y=0)
    else:
        # 典型行驶：前进 300 rpm、轻微转向、1.52 m/s、航位 12.34 / -5.67 m
        vals = dict(yaw=15, w=(302, 298, 300, 299),
                    gyro=12, center=152, pose_x=1234, pose_y=-567)

    if args.yaw    is not None: vals["yaw"]    = args.yaw
    if args.gyro   is not None: vals["gyro"]   = args.gyro
    if args.center is not None: vals["center"] = args.center
    if args.pose_x is not None: vals["pose_x"] = args.pose_x
    if args.pose_y is not None: vals["pose_y"] = args.pose_y
    if args.w0     is not None: vals["w"] = (args.w0, vals["w"][1], vals["w"][2], vals["w"][3])
    if args.w1     is not None: vals["w"] = (vals["w"][0], args.w1, vals["w"][2], vals["w"][3])
    if args.w2     is not None: vals["w"] = (vals["w"][0], vals["w"][1], args.w2, vals["w"][3])
    if args.w3     is not None: vals["w"] = (vals["w"][0], vals["w"][1], vals["w"][2], args.w3)

    cv = Canvas()

    # ---- 非避障导航页（page 4）：坐标系 + 车绿点 + 准心 + 已记录点 + 辅助菜单 ----
    _ns = args.nav_state
    _status_name = {0: "ST_MANUAL", 1: "ST_RUNNING", 2: "ST_PAUSED"}[_ns]
    _status_bg   = {0: None, 1: C_RUN_BG, 2: C_PAUSE_BG}[_ns]

    if args.page == 4:
        draw_menu(cv, lg_g, text_items, stream, lg_idx, 4,
                  status_bg=_status_bg, menu_dim=True, status_name=_status_name)
        pts = []
        for seg in args.pts.split(";"):
            seg = seg.strip()
            if not seg:
                continue
            a, b = seg.split(",")
            pts.append((int(a), int(b)))
        draw_nav_page(cv, lg_g, lg_idx, text_items, stream, num_g, num_idx,
                      cur=(args.cur_x, args.cur_y), car=(args.car_x, args.car_y),
                      pts=pts, aux=args.aux)
        cv.save(args.out)
        print("wrote", args.out, "- page 4（非避障导航页）aux=%d run=%d pts=%d"
              % (args.aux, args.run, len(pts)))
        return 0

    draw_menu(cv, lg_g, text_items, stream, lg_idx, args.menu_sel)

    # ★ 2026-10-19：支持逐页预览。page 0 = 运行数据显示（默认页，有车图与全部数值）；
    #   1..4 是占位页，内容区只有一条占位提示（与 ui_render.v 的 placeholder_text 一致）。
    #   ★ page 1（摄像头）在实物上会被 cam_overlay 的摄像头画面**整块覆盖**（x >= 208），
    #     这里画的是 ui_render 的输出，用于确认"叠加前"的内容对不对。
    if args.page != 0:
        _ph = {1: "PLACEHOLDER_CAM", 2: "PLACEHOLDER_RADAR",
               3: "PLACEHOLDER_NAV", 4: "PLACEHOLDER_NAV"}[args.page]
        draw_placeholder(cv, lg_g, lg_idx, text_items, stream, _ph)
        cv.save(args.out)
        print("wrote", args.out, "- page %d（占位页）" % args.page)
        return 0

    draw_car(cv)
    draw_wheel_boxes(cv)
    draw_yaw_box(cv)

    # ---- ★ 2026-10-18 两处带单位标签：中文 24px + 后缀 16px 混排 ----
    #   ① 「航向角」x=432（24 组 18）+「/°」x=512（16 组 32），y=88（距数字 16 px）
    #   ② 「车轮转速」x=408（24 组 17）+「/rpm」x=512（16 组 32），y=504
    #   两条的中文右界都是 503、后缀都从 512 起，故天然左对齐于同一竖线。
    # ---- ★ 2026-10-19 IMU 初始化失败提示：红色大字，压在「航向角」标签正上方 ----
    #   几何与 ui_render.v 完全一致：8 字 × 24 = 192 px，左界 384、y = 48..71
    if args.imu_fail:
        draw_text_label(cv, lg_g, lg_idx, text_items, stream, "IMU_FAIL",
                        IMU_FAIL_X, IMU_FAIL_Y, 24, color=C_IMU_FAIL)
    draw_text_label(cv, lg_g,  lg_idx,  text_items, stream, "LBL_YAW",   456,  72, 24)
    draw_text_label(cv, lg_g,  lg_idx,  text_items, stream, "LBL_WHEEL", 408, 504, 24)
    draw_text_label(cv, num_g, num_idx, text_items, stream, "LBL_RPM",   512, 504, 16)

    # ---- 车头偏航角（4 字符，x=464, y=120）----
    draw_num(cv, num_g, fmt_field(vals["yaw"], 1), PX_YAW_N * NUM_W, 112, 4)

    # ---- 四轮转速（4 字符，框内居中：x 不变，y = 框顶 + 8）----
    #   RTL 里前轮 y=248（框 240..279），后轮 y=420（框 412..451）→ 都是"框顶 + 8"
    #   ★ 2026-10-18 车体掉头 180° 后映射：A=左前 B=右前 C=左后 D=右后
    #     （掉头前是 A=右后 B=左后 C=右前 D=左前；180° 旋转把每个通道送到对角象限）
    w = vals["w"]
    draw_num(cv, num_g, fmt_field(w[0], 1), PX_WL_N * NUM_W, 248, 4)   # 左前 = A
    draw_num(cv, num_g, fmt_field(w[1], 1), PX_WR_N * NUM_W, 248, 4)   # 右前 = B
    draw_num(cv, num_g, fmt_field(w[2], 1), PX_WL_N * NUM_W, 420, 4)   # 左后 = C
    draw_num(cv, num_g, fmt_field(w[3], 1), PX_WR_N * NUM_W, 420, 4)   # 右后 = D

    # ---- 右侧列表：标签（24x24, x=792） + 数值（16x24, x=896） ----
    rows = [("LBL_GYRO",   "gyro",   240, 0),
            ("LBL_VEL",    "center", 288, 2),
            ("LBL_POSE_X", "pose_x", 336, 2),
            ("LBL_POSE_Y", "pose_y", 384, 2)]
    for nm, key, y, dec in rows:
        if nm in text_items:
            addr, length = text_items[nm]
            for k in range(length):
                ci = stream.get(addr + k)
                if ci is None:
                    continue
                code = lg_idx.get(ci)
                if code is None or code not in lg_g:
                    continue
                cv.glyph(lg_g[code], PX_LBL * LG_CELL + k * LG_W, y, LG_W, C_TEXT)
        # min_digits = dec + 1：带小数的字段整数部分至少 1 位（0 要显示成 0.00）
        draw_num(cv, num_g, fmt_field(vals[key], (dec + 1) if dec else 1), PX_VAL_N * NUM_W, y, 5)
        draw_dot(cv, PX_VAL_N * NUM_W, y, dec)

    cv.save(args.out)
    print("wrote", args.out)
    print("  航向角 %d | 轮速 %s | 角速度 %d | 中心 %d (%.2f m/s) | 航位 %d / %d (%.2f / %.2f m)"
          % (vals["yaw"], w, vals["gyro"], vals["center"], vals["center"] / 100.0,
             vals["pose_x"], vals["pose_y"], vals["pose_x"] / 100.0, vals["pose_y"] / 100.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
