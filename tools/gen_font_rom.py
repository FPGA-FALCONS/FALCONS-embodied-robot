#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_font_rom.py —— 从字表生成中文点阵字库（大小两套）与 UI 字符串 ROM（Verilog 源）

为什么这么做：
  * 界面要中文（1024x600 屏，菜单/标签/状态词），FPGA 里必须有点阵字库；
  * 数据**内嵌**进 Verilog（不用 $readmemh），避免综合工作目录、外部文件缺失、
    CR 混入等问题（class\\car_fpga 的 BMI270 配置 ROM 就是这么做并踩过坑的）。
  * 生成物可复现：改文案 → 重跑本脚本即可。

★ 两套字模（2026-10-17 新增）：
  需求方要求"左侧菜单文字与档位文字放大为原来 1.5 倍左右"，但 16x16 点阵无法无损放大，
  所以同一份字符集渲染两遍：
      font_rom.v     16x16  → 用于右侧内容区（数据页标签/数值、占位提示）
      font_rom_lg.v  24x24  → 用于左侧菜单条（4 个菜单项 + 状态栏档位文字）
  两者**共用同一个字符索引**（char_index 的含义一致），所以 ui_text_rom.v 与
  ui_text_index.vh 只有一份，改字号不影响文案索引。

用法：
    python tools/gen_font_rom.py
    python tools/gen_font_rom.py --chars tools/font_chars.txt --out rtl/common --font C:/Windows/Fonts/simhei.ttf

产物（UTF-8 无 BOM）：
    rtl/common/font_rom.v        16x16 点阵字库 ROM（深度 = 字符数 x 16，宽度 16）
    rtl/common/font_rom_lg.v     24x24 点阵字库 ROM（深度 = 字符数 x 24，宽度 24）
    rtl/common/ui_text_rom.v     字符串 ROM：按顺序存放每个文案的字符索引
    rtl/common/ui_text_index.vh  每个文案的 {起始地址, 长度} localparam + 总字符数

地址约定：
    font_rom    地址 = char_index * 16 + row   (row 0 = 最上面一行)，每行 16 bit，bit15 = 最左
    font_rom_lg 地址 = char_index * 24 + row   (row 0 = 最上面一行)，每行 24 bit，bit23 = 最左
"""

import argparse
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

CELL = 16            # 小字点阵边长（右侧内容区）
CELL_LG = 24         # 大字点阵边长（左侧菜单条，约 1.5 倍）
ADDR_BITS = 12       # 小字库/文本 ROM 地址位宽（4096 项）
ASCII_MIN, ASCII_MAX = 0x20, 0x7E
# ★ 2026-10-16：单条文案的长度上限。
#   UI_TXT_<名>_LEN 生成的是 `localparam [5:0]`（6 位），ui_render 的段描述符里
#   长度字段也是 6 位 —— 超过 63 会被**静默截断**（65 → 1），屏幕上只显示开头一两个字，
#   而本脚本 / vlog / 仿真都不会报错。所以生成前显式校验、越界直接报错退出。
MAX_TEXT_LEN = 63


def comment_text(text):
    """把文案安全地内嵌进【生成的 Verilog 注释】里。

    为什么要处理：
      * 反斜杠与双引号先转义（`\\\\`、`\\\\"`）—— 注释里虽然不参与语法，但本工程生成物
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


def render_glyph(ch, font, cell, cell_h=None):
    """渲染单字到 cell x cell_h 点阵，返回 cell_h 个整数（每行 cell bit，最高位 = 最左）

    cell_h 省略时按正方形处理（16x16 / 24x24）。
    ★ 非正方形用于"紧凑数字字库"：16 宽 x 24 高。数字本来窄，横向压窄不会明显变形，
      而纵向保持 24 与中文标签等高 —— 比"把数字拉宽撑满 24 格"自然得多。
    """
    if cell_h is None:
        cell_h = cell

    canvas_w, canvas_h = cell * 2, cell_h * 2
    img = Image.new("L", (canvas_w, canvas_h), 0)
    draw = ImageDraw.Draw(img)
    draw.text((canvas_w // 2, canvas_h // 2), ch, fill=255, font=font, anchor="mm")

    bbox = img.getbbox()
    if bbox is None:
        return [0] * cell_h      # 空白字符（空格）

    glyph = img.crop(bbox)
    w, h = glyph.size
    if w > cell or h > cell_h:
        # 缩小到能放下（保持宽高比）
        scale = min(cell / w, cell_h / h)
        glyph = glyph.resize((max(1, int(w * scale)), max(1, int(h * scale))), Image.LANCZOS)
        w, h = glyph.size

    out = Image.new("L", (cell, cell_h), 0)
    out.paste(glyph, ((cell - w) // 2, (cell_h - h) // 2))

    rows = []
    px = out.load()
    for y in range(cell_h):
        bits = 0
        for x in range(cell):
            if px[x, y] >= 100:
                bits |= 1 << (cell - 1 - x)
        rows.append(bits)
    return rows


def addr_bits_for(depth):
    """按深度算所需地址位宽（深度必须是 2 的幂以上）"""
    bits = max(1, int(math.ceil(math.log2(max(2, depth)))))
    return bits


def write_font_rom(path, charset, glyphs, cell, module_name, title, usage_note):
    """写一个点阵字库 ROM（cell x cell，行宽 cell bit）"""
    depth = len(charset) * cell
    bits = addr_bits_for(depth)
    hexw = (cell + 3) // 4

    lines = []
    a = lines.append
    a(f"// {os.path.basename(path)} —— {title}（由 tools/gen_font_rom.py 自动生成，请勿手改）")
    a("//")
    a(f"// 字符数：{len(charset)}（ASCII 可打印字符 + UI 文案用字）")
    a(f"// 深度  ：{depth} = 字符数 x {cell} 行；宽度 {cell} bit，bit{cell-1} = 最左像素")
    a(f"// 地址  ：char_index * {cell} + row（row 0 = 最上面一行）")
    a(f"// 位宽  ：地址需 {bits} 位（深度 {depth}）")
    a("//")
    a("// 字表来源：tools/font_chars.txt；字体：simhei（黑体）")
    a(f"// {usage_note}")
    a("// 字体数据内嵌，不使用 $readmemh —— 避免综合工作目录与外部文件的依赖。")
    a("")
    a("`timescale 1ns/1ps")
    a("")
    a(f"module {module_name} (")
    a("    input  wire             i_clk,")
    a(f"    input  wire [{bits-1}:0] i_addr,")
    a(f"    output reg  [{cell-1}:0] o_row_bits")
    a(");")
    a("")
    a(f"    localparam integer ROM_DEPTH = {depth};")
    a(f"    localparam integer CHAR_COUNT = {len(charset)};")
    a("")
    a(f"    reg [{cell-1}:0] mem [0:ROM_DEPTH-1];")
    a("")
    a("    initial begin")
    for idx, ch in enumerate(charset):
        code = ord(ch)
        for row in range(cell):
            a(f"        mem[{idx * cell + row:5d}] = {cell}'h{glyphs[ch][row]:0{hexw}X};"
              f"  // U+{code:04X} '{comment_text(ch)}' 行{row}")
    a("    end")
    a("")
    a("    always @(posedge i_clk) begin")
    a("        o_row_bits <= mem[i_addr];")
    a("    end")
    a("")
    a("endmodule")
    write_text(path, "\n".join(lines) + "\n")
    return depth, bits


def write_font_rom_wh(path, charset, glyphs, w, h, module_name, title, usage_note):
    """写非正方形点阵字库（宽 w、高 h，行宽 w bit、每字符 h 行）。

    与 write_font_rom 的唯一区别：行数与行宽分开。
    地址约定保持 ci * h + row —— 与 font_rom_lg（ci*24+row）一致，便于 RTL 复用同一套寻址。
    """
    depth = len(charset) * h
    bits = addr_bits_for(depth)
    hexw = (w + 3) // 4

    lines = []
    a = lines.append
    a(f"// {os.path.basename(path)} —— {title}（由 tools/gen_font_rom.py 自动生成，请勿手改）")
    a("//")
    a(f"// 字符数：{len(charset)}（与 ui_text_rom 的字符索引一致）")
    a(f"// 深度  ：{depth} = 字符数 x {h} 行；宽度 {w} bit，bit{w-1} = 最左像素")
    a(f"// 地址  ：char_index * {h} + row（row 0 = 最上面一行）")
    a(f"// 位宽  ：地址需 {bits} 位（深度 {depth}）")
    a("//")
    a("// 字表来源：tools/font_chars.txt；字体：simhei（黑体）")
    a(f"// {usage_note}")
    a("")
    a("`timescale 1ns/1ps")
    a("")
    a(f"module {module_name} (")
    a("    input  wire             i_clk,")
    a(f"    input  wire [{bits-1}:0] i_addr,")
    a(f"    output reg  [{w-1}:0] o_row_bits")
    a(");")
    a("")
    a(f"    localparam integer ROM_DEPTH = {depth};")
    a(f"    localparam integer CHAR_COUNT = {len(charset)};")
    a("")
    a(f"    reg [{w-1}:0] mem [0:ROM_DEPTH-1];")
    a("")
    a("    initial begin")
    for idx, ch in enumerate(charset):
        code = ord(ch)
        for row in range(h):
            a(f"        mem[{idx * h + row:5d}] = {w}'h{glyphs[ch][row]:0{hexw}X};"
              f"  // U+{code:04X} '{comment_text(ch)}' 行{row}")
    a("    end")
    a("")
    a("    always @(posedge i_clk) begin")
    a("        o_row_bits <= mem[i_addr];")
    a("    end")
    a("")
    a("endmodule")
    write_text(path, "\n".join(lines) + "\n")
    return depth, bits


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
    a("//       ★ 大小两套字库（font_rom / font_rom_lg）共用这里的字符索引")
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
    b("//       再用 char_index*16+row 从 font_rom（16x16）取 16bit 行点阵，")
    b("//       或 char_index*24+row 从 font_rom_lg（24x24）取 24bit 行点阵。")
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
    ap = argparse.ArgumentParser(description="生成中文点阵字库（大小两套）与 UI 字符串 ROM")
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    ap.add_argument("--chars", default=os.path.join(here, "font_chars.txt"))
    ap.add_argument("--font", default=r"C:\Windows\Fonts\simhei.ttf")
    ap.add_argument("--out", default=os.path.join(root, "rtl", "common"))
    ap.add_argument("--size", type=int, default=16, help="小字渲染字号（默认 16，点阵 16x16）")
    ap.add_argument("--cell", type=int, default=CELL, help="小字点阵边长（默认 16）")
    ap.add_argument("--lg-size", type=int, default=CELL_LG,
                    help="大字渲染字号（默认 24，点阵 24x24）")
    ap.add_argument("--lg-cell", type=int, default=CELL_LG, help="大字点阵边长（默认 24）")
    ap.add_argument("--num-w", type=int, default=16,
                    help="紧凑数字字库的宽度（默认 16；高度沿用 --lg-cell=24）")
    ap.add_argument("--index-out", default=os.path.join(root, "rtl", "ui"),
                    help="ui_text_index.vh 的输出目录（默认 rtl/ui，与 ui_render.v 同目录）")
    args = ap.parse_args()

    if not os.path.isfile(args.chars):
        raise SystemExit(f"[错误] 找不到字表：{args.chars}")
    if not os.path.isfile(args.font):
        raise SystemExit(f"[错误] 找不到字体：{args.font}（用 --font 指定）")

    items = parse_chars_file(args.chars)
    charset = build_charset(items)

    # ---- 小字库（16x16）----
    font_sm = ImageFont.truetype(args.font, args.size)
    glyphs_sm = {ch: render_glyph(ch, font_sm, args.cell) for ch in charset}
    depth_sm, bits_sm = write_font_rom(
        os.path.join(args.out, "font_rom.v"), charset, glyphs_sm, args.cell,
        "font_rom", f"{args.cell}x{args.cell} 中文点阵字库（右侧内容区用小字）",
        "用途：右侧内容区（数据页标签/数值、占位提示）。")

    # ---- 大字库（24x24，约 1.5 倍）----
    font_lg = ImageFont.truetype(args.font, args.lg_size)
    glyphs_lg = {ch: render_glyph(ch, font_lg, args.lg_cell) for ch in charset}
    depth_lg, bits_lg = write_font_rom(
        os.path.join(args.out, "font_rom_lg.v"), charset, glyphs_lg, args.lg_cell,
        "font_rom_lg", f"{args.lg_cell}x{args.lg_cell} 中文点阵字库（左侧菜单条用大字）",
        "用途：左侧菜单条（4 个菜单项 + 状态栏档位文字），约为小字的 1.5 倍。")

    # ---- 紧凑数字字库（16 宽 x 24 高）----
    #   需求方 2026-10-17："数字之间的间距可以紧密一点"。
    #   24x24 里数字笔画只占 ~13 px，格宽 24 → 相邻数字空 ~11 px；改成 16 宽后空 ~3 px，
    #   4 个字符 64 px（原 96 px），而高度仍 24，与中文标签等高。
    #   ★ 用同一份字符集与同样的地址约定（ci × 24 + row），只是行宽变 16 bit，
    #     所以与 ui_text_rom 的字符索引完全兼容。
    glyphs_num = {ch: render_glyph(ch, font_lg, args.num_w, args.lg_cell) for ch in charset}
    depth_num, bits_num = write_font_rom_wh(
        os.path.join(args.out, "font_num.v"), charset, glyphs_num,
        args.num_w, args.lg_cell, "font_num",
        f"{args.num_w}x{args.lg_cell} 紧凑数字点阵字库（数值字段专用）",
        f"用途：数据页的实时数值（数字/负号/空格）。宽 {args.num_w}、高 {args.lg_cell}："
        "与大字库同高、更窄，用来收紧数字间距。")
    index_path = os.path.join(args.index_out, "ui_text_index.vh")
    table = write_ui_text_rom(os.path.join(args.out, "ui_text_rom.v"), index_path, charset, items)

    print(f"字表文案 : {len(items)} 条")
    print(f"字符集   : {len(charset)} 个（含 ASCII 可打印字符 {ASCII_MAX - ASCII_MIN + 1} 个）")
    print(f"小字库   : {depth_sm} x {args.cell} bit（地址 {bits_sm} 位）->  rtl\\common\\font_rom.v")
    print(f"大字库   : {depth_lg} x {args.lg_cell} bit（地址 {bits_lg} 位）->  rtl\\common\\font_rom_lg.v")
    print(f"数字库   : {depth_num} x {args.num_w} bit（地址 {bits_num} 位）->  rtl\\common\\font_num.v")
    print(f"文本 ROM : {sum(t[3] for t in table)} 字符           ->  rtl\\common\\ui_text_rom.v")
    print(f"索引     : {len(table)} 条 localparam          ->  {os.path.relpath(index_path, root)}")
    for name, text, start, length in table[:4]:
        print(f"   {name:14s} addr={start:3d} len={length}  {text}")
    empty = [ch for ch in charset if not any(glyphs_sm[ch])]
    if empty:
        print(f"[提示] 渲染为空白（通常是空格）：{''.join(repr(c) for c in empty)}")


if __name__ == "__main__":
    sys.exit(main())
