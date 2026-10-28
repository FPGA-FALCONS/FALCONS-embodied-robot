#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""preview_tune_page.py —— 按 ui_render.v 的真实布局，离屏渲染「参数整定页」预览图

为什么需要这个脚本：
  * 参数整定页是 15 行「标签 + 5 格数值 + 单位」的三列排布，行高只有 36 px，
    数值右边缘 464 与单位起点 480 之间只剩 16 px 缝。这类"贴边布局"靠读代码
    算间隙极易漏掉一个前导空格 —— 只有把像素画出来才能一眼确认。
  * 点阵数据**直接读 rtl/common/font_num.v / font_rom_lg.v / ui_text_rom.v**，
    与上板共用同一份字库，能真实反映字宽、笔画位置与空白。

本脚本是只读的：不生成/不修改任何 RTL，只输出 PNG。

用法：
    python tools/preview_tune_page.py
    python tools/preview_tune_page.py --out doc/img/tune_page_sel4.png --sel 4
    python tools/preview_tune_page.py --preset min        # 全部取区间下限

布局常量的出处（改布局时请同步这里，否则预览会失真）：
    rtl/ui/ui_render.v   TUNE_Y0=72  TUNE_STEP=32  TUNE_TDY=4  TUNE_VDY    = 4
                         TUNE_LBL_XW=10(x=240)  TUNE_VAL_XW=24(x=384)
                         TUNE_UNIT_XW=30(x=480)  TUNE_ROWS=15
                         高亮条 x>=224，y = TUNE_Y0+sel*TUNE_STEP .. +TUNE_STEP
    rtl/ui/tune_ctrl.v   15 项默认值/区间/步长/显示换算（见下面 PRESETS）
"""

import argparse
import os
import re
import sys

import numpy as np
from PIL import Image

SCREEN_W, SCREEN_H = 1024, 600
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ---------------------------------------------------------------------------
# 颜色（与 ui_render.v 的 rgb 赋值一一对应）
# ---------------------------------------------------------------------------
C_BG        = (0x00, 0x00, 0x00)   # 内容区黑底
C_MENU_BG   = (0x10, 0x18, 0x28)   # 左侧菜单条
C_MENU_SEL  = (0x1D, 0x4E, 0xD8)   # 菜单高亮 / 参数行高亮条
C_STATUS_BG = (0x20, 0x20, 0x20)   # 状态栏底色（光标档 20/20/20；手动档 66/00/00、导航中 00/66/00、暂停 99/4D/00）
C_TEXT      = (0xFF, 0xFF, 0xFF)   # 菜单未选中 / 标签 / 单位
C_TEXT_SEL  = (0xFF, 0xE0, 0x66)   # 菜单选中项（亮黄）
C_MENU_TEXT = (0xE5, 0xE7, 0xEB)   # 菜单未选中（近白，ui_render.v 的 24'hE5E7EB）
# ★ 参数整定页整条菜单栏变暗（2026-10-20）：ui_render.v 的 tune_ui
C_MENU_BG_DIM   = (0x0B, 0x10, 0x19)
C_MENU_TEXT_DIM = (0x5A, 0x61, 0x70)
C_TEXT_SEL_DIM  = (0x8A, 0x7A, 0x2E)
C_NUM       = (0x7C, 0xFC, 0x98)   # 实时数值（亮绿）

# ---------------------------------------------------------------------------
# 布局常量（ui_render.v）
# ---------------------------------------------------------------------------
MENU_W      = 208
LG_CELL     = 24
LG_XW       = 1          # 菜单文字 x = 24
MENU_Y0     = 48
MENU_STEP   = 72
MENU_ITEMS  = 7          # 2026-10-20 由 5 项扩到 6 项（末项「参数整定」）
STATUS_Y    = 552

TUNE_Y0     = 72
TUNE_STEP   = 32
TUNE_TDY    = 4          # 24 px 标签相对行顶的偏移
TUNE_VDY    = 4          # 24 px 数值相对行顶的偏移 (36-24)/2
TUNE_LBL_XW = 10         # 标签 x = 240
TUNE_VAL_XW = 24         # 数值 x = 384
TUNE_UNIT_XW= 30         # 单位 x = 480
TUNE_ROWS   = 15
TUNE_HL_X0  = 224        # 高亮条左边界（不侵入 208 px 菜单条）
TUNE_HL_X1  = 655        # 高亮条右边界（= ui_render.v TUNE_HL_X1-1）

NUM_W, NUM_H = 16, 24
LG_W,  LG_H  = 24, 24


# ---------------------------------------------------------------------------
# 字库解析：从生成的 Verilog ROM 里取点阵
# ---------------------------------------------------------------------------
RE_MEM = re.compile(
    r"mem\[\s*(\d+)\]\s*=\s*(\d+)'h([0-9A-Fa-f]+)\s*;\s*//\s*U\+([0-9A-Fa-f]{4})")


def parse_font(path, width, height):
    """返回 {码点: [每行位图 int, ...]}，以及 {字库索引: 码点}。"""
    glyphs, index_of = {}, {}
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
    """复现 num_text.v 的 5 格格式化：'符号 + 千 + 百 + 十 + 个'。"""
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


# ---------------------------------------------------------------------------
# 参数表（与 rtl/ui/tune_ctrl.v 的 p_def / d_def 默认值一一对应）
# ---------------------------------------------------------------------------
# ★ 2026-10-20：积分限幅插到行 8（航向 Kd 下、速度环 Kp 上），
#   速度环三项后移到行 9/10/11。字库常量名里的数字是**文案编号**，
#   与行号不再一一对应 —— 必须与 rtl\ui\ui_render.v 的 tune_label_text 完全一致。
LBL_NAMES = [f"TUNE_L{i}" for i in range(8)] + ["TUNE_L11", "TUNE_L8", "TUNE_L9", "TUNE_L10", "TUNE_FORWARD", "TUNE_WIDTH", "TUNE_WINDOW"]
UNIT_NAMES = ["TUNE_UMMS", "TUNE_UMM", "TUNE_UDEG", "TUNE_UDEG", "TUNE_UDEGS",
              "", "", "", "", "", "", "", "TUNE_CELLS", "TUNE_UMM", "TUNE_CELLS"]

# (参数默认值, 显示值)
PRESET_DEFAULT = [300, 100, 800, 3500, 126, 40, 3, 6, 120, 24, 36, 0, 4, 180, 5]
# 显示换算：p2/p3 是 0.01° 单位（÷100），p4 是 counts 单位（×0.238)
PRESET_MIN = [50, 50, 100, 1000, 30, 0, 0, 0, 0, 0, 0, 0, 1, 50, 2]
PRESET_MAX = [600, 300, 5000, 9000, 612, 200, 50, 50, 612, 255, 255, 255, 16, 500, 64]


def to_disp(idx, raw):
    """复现 tune_ctrl.v 的显示换算。

    p2/p3（对准小角/大角）输入单位 0.01°，显示为整数度：(v * 328) >> 15 ≈ v/100
    p4（转向角速度）输入单位 counts/10ms，显示为 °/s：(v * 7813 + 16384) >> 15
    其余原样。
    """
    if idx in (2, 3):
        return (raw * 328) >> 15
    if idx == 4:
        return (raw * 7813 + 16384) >> 15
    return raw


# ---------------------------------------------------------------------------
# 主绘制
# ---------------------------------------------------------------------------
def draw_page(cv, lg_glyphs, num_glyphs, text_items, text_stream,
              sel, presets, menu_sel, active=True):
    # --- 左侧菜单条 ---
    # ★ 参数页整条菜单栏变暗（ui_render.v 的 tune_ui，2026-10-20 需求方要求）
    #   active=False = 已退出编辑态那一态：菜单栏恢复正常亮度、不画行高亮条
    #   （page 仍停在参数页，与导航页退出 nav_ui 同构）
    cv.rect(0, 0, MENU_W - 1, SCREEN_H - 1,
            C_MENU_BG_DIM if active else C_MENU_BG)
    # 菜单选中项底色：ui_render.v 的 menu_sel_band
    #   x 0..207，y = MENU_Y0 + menu_sel*MENU_STEP - 12 .. +36（共 48 px）
    y_b = MENU_Y0 + menu_sel * MENU_STEP - 12
    cv.rect(0, y_b, MENU_W - 1, y_b + 47, C_MENU_SEL)
    # 状态栏底色：只覆盖菜单条宽度（x 0..207），y = STATUS_Y-12 .. +36
    cv.rect(0, STATUS_Y - 12, MENU_W - 1, STATUS_Y + 36 - 1, C_STATUS_BG)
    # 状态栏文案：光标档 → 「光标档」（ui_render.v :238，x=24 y=552 起，24px 白）
    if "ST_CURSOR" in text_items:
        _a, _l = text_items["ST_CURSOR"]
        for _k in range(_l):
            _code = INDEX_TO_CODE_LG.get(text_stream.get(_a + _k, 0))
            if _code is None or _code not in lg_glyphs:
                continue
            cv.glyph(lg_glyphs[_code], LG_XW * LG_CELL + _k * LG_CELL, STATUS_Y, LG_W, C_TEXT)
    menu_names = ["MENU_VEHICLE", "MENU_CAMERA", "MENU_RADAR",
                  "MENU_NAV_OBS", "MENU_NAV_FREE", "MENU_TUNE", "MENU_AI"]
    for i, nm in enumerate(menu_names):
        if nm not in text_items:
            continue
        addr, ln = text_items[nm]
        y = MENU_Y0 + i * MENU_STEP
        if active:
            color = C_TEXT_SEL_DIM if i == menu_sel else C_MENU_TEXT_DIM
        else:
            color = C_TEXT_SEL if i == menu_sel else C_MENU_TEXT
        for k in range(ln):
            ch = text_stream.get(addr + k, 0)
            # ui_text_rom 存的是"字库索引"，不是码点，需反查。
            code = INDEX_TO_CODE_LG.get(ch)
            if code is None or code not in lg_glyphs:
                continue
            cv.glyph(lg_glyphs[code], LG_XW * LG_CELL + k * LG_CELL, y, LG_W, color)

    # --- 参数页高亮条（在文字之下；仅编辑态画）---
    if active:
        cv.rect(TUNE_HL_X0, TUNE_Y0 + sel * TUNE_STEP,
                TUNE_HL_X1, TUNE_Y0 + sel * TUNE_STEP + TUNE_STEP - 1, C_MENU_SEL)

    # --- 标题「参数整定」y = 24 ---
    addr, ln = text_items["MENU_TUNE"]
    for k in range(ln):
        code = INDEX_TO_CODE_LG.get(text_stream.get(addr + k, 0))
        if code is None or code not in lg_glyphs:
            continue
        cv.glyph(lg_glyphs[code], TUNE_LBL_XW * LG_CELL + k * LG_CELL, 24, LG_W, C_TEXT)

    # --- 15 行：标签 / 数值 / 单位 ---
    for i in range(TUNE_ROWS):
        y_lbl = TUNE_Y0 + i * TUNE_STEP + TUNE_TDY
        y_val = TUNE_Y0 + i * TUNE_STEP + TUNE_VDY

        nm = LBL_NAMES[i]
        if nm in text_items:
            addr, ln = text_items[nm]
            for k in range(ln):
                code = INDEX_TO_CODE_LG.get(text_stream.get(addr + k, 0))
                if code is None or code not in lg_glyphs:
                    continue
                cv.glyph(lg_glyphs[code], TUNE_LBL_XW * LG_CELL + k * LG_CELL,
                         y_lbl, LG_W, C_TEXT)

        txt5 = fmt_field(to_disp(i, presets[i]))
        for k, ch in enumerate(txt5):
            if ch == ' ':
                continue
            cv.glyph(num_glyphs[ord(ch)], TUNE_VAL_XW * NUM_W + k * NUM_W,
                     y_val, NUM_W, C_NUM)

        unm = UNIT_NAMES[i]
        if unm and unm in text_items:
            addr, ln = text_items[unm]
            for k in range(ln):
                code = INDEX_TO_CODE_LG.get(text_stream.get(addr + k, 0))
                if code is None or code not in num_glyphs:
                    continue
                cv.glyph(num_glyphs[code], TUNE_UNIT_XW * NUM_W + k * NUM_W,
                         y_val, NUM_W, C_TEXT)




INDEX_TO_CODE_LG = {}
INDEX_TO_CODE_NUM = {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "doc", "img", "tune_page_default.png"))
    ap.add_argument("--sel", type=int, default=0, help="高亮行 0..14")
    ap.add_argument("--menu-sel", type=int, default=5, help="菜单选中项 0..5（默认 5=参数整定）")
    ap.add_argument("--preset", choices=["default", "min", "max"], default="default")
    ap.add_argument("--idle", action="store_true", help="画已退出编辑态那一态：菜单栏不暗、无行高亮条")
    args = ap.parse_args()

    lg_glyphs, idx_lg = parse_font(os.path.join(ROOT, "rtl", "common", "font_rom_lg.v"),
                                   LG_W, LG_H)
    num_glyphs, idx_num = parse_font(os.path.join(ROOT, "rtl", "common", "font_num.v"),
                                     NUM_W, NUM_H)
    INDEX_TO_CODE_LG.clear()
    INDEX_TO_CODE_LG.update(idx_lg)
    INDEX_TO_CODE_NUM.clear()
    INDEX_TO_CODE_NUM.update(idx_num)

    text_items, text_stream = parse_text_rom(
        os.path.join(ROOT, "rtl", "common", "ui_text_rom.v"),
        os.path.join(ROOT, "rtl", "ui", "ui_text_index.vh"))

    presets = {"default": PRESET_DEFAULT, "min": PRESET_MIN, "max": PRESET_MAX}[args.preset]

    cv = Canvas()
    draw_page(cv, lg_glyphs, num_glyphs, text_items, text_stream,
              args.sel, presets, args.menu_sel, active=(not args.idle))
    cv.save(args.out)
    print(f"wrote {args.out}  ({SCREEN_W}x{SCREEN_H})  sel={args.sel} menu_sel={args.menu_sel} preset={args.preset}" + ("  idle" if args.idle else ""))


if __name__ == "__main__":
    main()
