#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_font_rom.py —— 从字表生成中文 16x16 点阵字库与 UI 字符串 ROM（Verilog 源）

为什么这么做：
  * 界面要中文（1024x600 屏，菜单/标签/状态词），FPGA 里必须有点阵字库；
  * 数据**内嵌**进 Verilog（不用 $readmemh），避免综合工作目录、外部文件缺失、
    CR 混入等问题（class\\car_fpga 的 BMI270 配置 ROM 就是这么做并踩过坑的）。
  * 生成物可复现：改文案 → 重跑本脚本即可。

用法：
    python tools/gen_font_rom.py
    python tools/gen_font_rom.py --chars tools/font_chars.txt --out rtl/common --font C:/Windows/Fonts/simhei.ttf

产物（UTF-8 无 BOM）：
    rtl/common/font_rom.v        16x16 点阵字库 ROM（深度 = 字符数 x 16，宽度 16）
    rtl/common/ui_text_rom.v     字符串 ROM：按顺序存放每个文案的字符索引
    rtl/common/ui_text_index.vh  每个文案的 {起始地址, 长度} localparam + 总字符数

地址约定：
    font_rom 地址 = char_index * 16 + row   (row 0 = 最上面一行)
    每行 16 bit，bit15 = 最左像素（1 = 点亮）
"""

import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

CELL = 16            # 点阵边长
ADDR_BITS = 12       # 地址位宽（4096 项，够 256 字 x16 行 或 4096 个文本字符）
ASCII_MIN, ASCII_MAX = 0x20, 0x7E
# ★ 2026-10-16：单条文案的长度上限。
#   UI_TXT_<名>_LEN 生成的是 `localparam [5:0]`（6 位），ui_render 的段描述符里
#   长度字段也是 6 位 —— 超过 63 会被**静默截断**（65 → 1），屏幕上只显示开头一两个字，
#   而本脚本 / vlog / 仿真都不会报错。所以生成前显式校验、越界直接报错退出。
MAX_TEXT_LEN = 63


def comment_text(text):
    """把文案安全地内嵌进【生成的 Verilog 注释】里。

    为什么要处理：
      * 反斜杠与双引号先转义（`\\`、`\\"`）—— 注释里虽然不参与语法，但本工程生成物
        是"人读的源文件"，原文里的引号若原样写进去，读者会分不清哪是注释哪是语法；
        将来若有人把注释改成字符串字面量（$display 等），也不会因为引号而语法错。
      * CR / LF / TAB 一律换成空格 —— 一条文案若含换行，注释会被断成两行，
        多出来那行不是注释，直接变成语法错误（注释里内嵌原文最危险的就是这一条）。
    """
    return (text.replace("\\", "\\\\")
                .replace('"', '\\"')
                .replace("\r", " ")
                .replace("\n", " ")
                .replace("\t", " "))


def parse_chars_file(path):
    """返回 [(name, text), ...]"""
    items = []
    seen = set()
    with open(path, "r", encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if ":" not in line:
                print(f"  [警告] 第 {lineno} 行缺少 ':'，已跳过：{line}")
                continue
            name, text = line.split(":", 1)
            name = name.strip().upper()
            text = text.strip()
            if not name or not text:
                print(f"  [警告] 第 {lineno} 行名或文案为空，已跳过")
                continue
            if name in seen:
                raise SystemExit(f"[错误] 常量名重复：{name}（第 {lineno} 行）")
            seen.add(name)
            items.append((name, text))
    if not items:
        raise SystemExit("[错误] 字表里没有任何文案")
    return items


def build_charset(items):
    """ASCII 可打印字符 + 所有文案里出现的字符，去重排序"""
    chars = set(chr(c) for c in range(ASCII_MIN, ASCII_MAX + 1))
    for _, text in items:
        chars.update(text)
    # ASCII 在前（按码点），其余按码点排序，便于人工核对
    return sorted(chars, key=lambda ch: (ord(ch) > ASCII_MAX, ord(ch)))


def render_glyph(ch, font):
    """渲染单字到 16x16 点阵，返回 16 个整数（每行 16 bit，bit15 = 最左）"""
    canvas = CELL * 2
    img = Image.new("L", (canvas, canvas), 0)
    draw = ImageDraw.Draw(img)
    draw.text((canvas // 2, canvas // 2), ch, fill=255, font=font, anchor="mm")

    bbox = img.getbbox()
    if bbox is None:
        return [0] * CELL      # 空白字符（空格）

    glyph = img.crop(bbox)
    w, h = glyph.size
    if w > CELL or h > CELL:
        # 缩小到能放下（保持宽高比）
        scale = min(CELL / w, CELL / h)
        glyph = glyph.resize((max(1, int(w * scale)), max(1, int(h * scale))), Image.LANCZOS)
        w, h = glyph.size

    cell = Image.new("L", (CELL, CELL), 0)
    cell.paste(glyph, ((CELL - w) // 2, (CELL - h) // 2))

    rows = []
    px = cell.load()
    for y in range(CELL):
        bits = 0
        for x in range(CELL):
            if px[x, y] >= 100:
                bits |= 1 << (CELL - 1 - x)
        rows.append(bits)
    return rows


def write_font_rom(path, charset, glyphs):
    depth = len(charset) * CELL
    if depth > (1 << ADDR_BITS):
        raise SystemExit(f"[错误] 字库深度 {depth} 超过 2^{ADDR_BITS}，请加大 ADDR_BITS")
    lines = []
    a = lines.append
    a("// font_rom.v —— 16x16 中文点阵字库（由 tools/gen_font_rom.py 自动生成，请勿手改）")
    a("//")
    a(f"// 字符数：{len(charset)}（ASCII 可打印字符 + UI 文案用字）")
    a(f"// 深度  ：{depth} = 字符数 x 16 行；宽度 16 bit，bit15 = 最左像素")
    a("// 地址  ：char_index * 16 + row（row 0 = 最上面一行）")
    a("//")
    a("// 字表来源：tools/font_chars.txt；字体：simhei（黑体）")
    a("// 字体数据内嵌，不使用 $readmemh —— 避免综合工作目录与外部文件的依赖。")
    a("")
    a("`timescale 1ns/1ps")
    a("")
    a("module font_rom (")
    a("    input  wire             i_clk,")
    a(f"    input  wire [{ADDR_BITS-1}:0] i_addr,")
    a("    output reg  [15:0]      o_row_bits")
    a(");")
    a("")
    a(f"    localparam integer ROM_DEPTH = {depth};")
    a(f"    localparam integer CHAR_COUNT = {len(charset)};")
    a("")
    a("    reg [15:0] mem [0:ROM_DEPTH-1];")
    a("")
    a("    initial begin")
    for idx, ch in enumerate(charset):
        code = ord(ch)
        for row in range(CELL):
            a(f"        mem[{idx * CELL + row:5d}] = 16'h{glyphs[ch][row]:04X};  // U+{code:04X} '{comment_text(ch)}' 行{row}")
    a("    end")
    a("")
    a("    always @(posedge i_clk) begin")
    a("        o_row_bits <= mem[i_addr];")
    a("    end")
    a("")
    a("endmodule")
    write_text(path, "\n".join(lines) + "\n")


def write_ui_text_rom(path, index_path, charset, items):
    index_of = {ch: i for i, ch in enumerate(charset)}
    stream = []
    table = []
    for name, text in items:
        start = len(stream)
        stream.extend(index_of[ch] for ch in text)
        table.append((name, text, start, len(text)))
    depth = max(1, len(stream))
    if depth > (1 << ADDR_BITS):
        raise SystemExit(f"[错误] 文本字符总数 {depth} 超过 2^{ADDR_BITS}，请加大 ADDR_BITS")

    # ---- ★ 长度校验（必须在生成 UI_TXT_*_LEN 之前）----
    #   UI_TXT_<名>_LEN 是 `localparam [5:0]`，ui_render 段描述符的长度字段同样是 6 位：
    #   文案长度 > 63 会被静默截断（65 → 1），屏幕只显示开头一两个字，
    #   而本脚本、vlog、仿真都不会报错 —— 属于"改了文案、看不出问题"的一类坑。
    too_long = [(name, text, length) for name, text, start, length in table
                if length > MAX_TEXT_LEN]
    if too_long:
        detail = "；".join(
            "{}（{} 字符: {}）".format(n, length, comment_text(t))
            for n, t, length in too_long
        )
        raise SystemExit(
            "[错误] 有 {} 条文案超过 {} 个字符，UI_TXT_*_LEN 是 6 位装不下：\n        {}\n"
            "        处理办法：把长文案拆成两条，或加大 LEN 的位宽"
            "（本脚本的 MAX_TEXT_LEN 与 ui_render.v 的段格式 [11:6] 必须同步改）。".format(
                len(too_long), MAX_TEXT_LEN, detail)
        )

    lines = []
    a = lines.append
    a("// ui_text_rom.v —— UI 字符串 ROM（由 tools/gen_font_rom.py 自动生成，请勿手改）")
    a("//")
    a("// 内容：把 tools/font_chars.txt 里每个文案拆成字符，按顺序存放其【字库索引】")
    a("//       配合 ui_text_index.vh 给出的 ADDR/LEN，即可逐字取点阵渲染")
    a(f"// 深度：{depth}（所有文案的字符总数），宽度 8 bit")
    a("")
    a("`timescale 1ns/1ps")
    a("")
    a("module ui_text_rom (")
    a("    input  wire             i_clk,")
    a(f"    input  wire [{ADDR_BITS-1}:0] i_addr,")
    a("    output reg  [7:0]       o_char_index")
    a(");")
    a("")
    a(f"    localparam integer ROM_DEPTH = {depth};")
    a("")
    a("    reg [7:0] mem [0:ROM_DEPTH-1];")
    a("")
    a("    initial begin")
    for addr, cidx in enumerate(stream, 0):
        a(f"        mem[{addr:5d}] = 8'd{cidx};")
    if not stream:
        a("        mem[0] = 8'd0;")
    a("    end")
    a("")
    a("    always @(posedge i_clk) begin")
    a("        o_char_index <= mem[i_addr];")
    a("    end")
    a("")
    a("endmodule")
    write_text(path, "\n".join(lines) + "\n")

    # ---- 索引头文件 ----
    h = []
    b = h.append
    b("// ui_text_index.vh —— UI 文案的地址/长度索引（由 tools/gen_font_rom.py 自动生成，请勿手改）")
    b("//")
    b("// 用法：text_renderer 用 UI_TXT_<名>_ADDR / UI_TXT_<名>_LEN 从 ui_text_rom 取字符索引，")
    b("//       再用 char_index*16+row 从 font_rom 取 16bit 行点阵。")
    b("")
    b(f"localparam integer UI_FONT_CHAR_COUNT = {len(charset)};")
    b(f"localparam integer UI_TEXT_TOTAL_CHARS = {depth};")
    b("")
    b("// {名, 文案, 起始地址, 字符数}")
    for name, text, start, length in table:
        # 注意：绝不能在标识符里插入对齐空格（Verilog 标识符不允许空格）
        #       文案内嵌进注释前先转义（引号/反斜杠/换行，见 comment_text）
        b(f"localparam [11:0] UI_TXT_{name}_ADDR = 12'd{start};  // {comment_text(text)}")
        b(f"localparam [5:0] UI_TXT_{name}_LEN  = 6'd{length};")   # 长度用 6 位：与 ADDR 拼起来正好 18 位
    b("")
    write_text(index_path, "\n".join(h) + "\n")
    return table


def write_text(path, text):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)


def main():
    ap = argparse.ArgumentParser(description="生成中文点阵字库与 UI 字符串 ROM")
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    ap.add_argument("--chars", default=os.path.join(here, "font_chars.txt"))
    ap.add_argument("--font", default=r"C:\Windows\Fonts\simhei.ttf")
    ap.add_argument("--out", default=os.path.join(root, "rtl", "common"))
    ap.add_argument("--size", type=int, default=16, help="渲染字号（默认 16，点阵仍是 16x16）")
    ap.add_argument("--index-out", default=os.path.join(root, "rtl", "ui"),
                    help="ui_text_index.vh 的输出目录（默认 rtl/ui，与 ui_render.v 同目录）")
    args = ap.parse_args()

    if not os.path.isfile(args.chars):
        raise SystemExit(f"[错误] 找不到字表：{args.chars}")
    if not os.path.isfile(args.font):
        raise SystemExit(f"[错误] 找不到字体：{args.font}（用 --font 指定）")

    items = parse_chars_file(args.chars)
    charset = build_charset(items)
    font = ImageFont.truetype(args.font, args.size)
    glyphs = {ch: render_glyph(ch, font) for ch in charset}

    write_font_rom(os.path.join(args.out, "font_rom.v"), charset, glyphs)
    index_path = os.path.join(args.index_out, "ui_text_index.vh")
    table = write_ui_text_rom(os.path.join(args.out, "ui_text_rom.v"), index_path, charset, items)

    print(f"字表文案 : {len(items)} 条")
    print(f"字符集   : {len(charset)} 个（含 ASCII 可打印字符 {ASCII_MAX - ASCII_MIN + 1} 个）")
    print(f"字库     : {len(charset) * CELL} x 16 bit  ->  rtl\\common\\font_rom.v")
    print(f"文本 ROM : {sum(t[3] for t in table)} 字符           ->  rtl\\common\\ui_text_rom.v")
    print(f"索引     : {len(table)} 条 localparam          ->  {os.path.relpath(index_path, root)}")
    empty = [ch for ch in charset if not any(glyphs[ch])]
    if empty:
        print(f"[提示] 渲染为空白（通常是空格）：{''.join(repr(c) for c in empty)}")


if __name__ == "__main__":
    sys.exit(main())
