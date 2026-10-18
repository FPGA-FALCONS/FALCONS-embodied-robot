#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_sin_lut.py —— 生成航位推算用的 sin/cos 查找表（Verilog 头文件）

为什么要 LUT 而不是 CORDIC：
  * odom 每 10 ms 只算一次（100 Hz），吞吐要求极低，但**要求确定性**、易验证；
  * CORDIC 要迭代 12~16 次、状态机复杂度高，出错难查；
  * 256 点表 + 线性插值在本车的转弯半径下误差 < 0.1%，足够。
  * 表用 `initial` 内嵌进 Verilog（与字库同一套做法，避免 $readmemh 的外部文件依赖）。

精度与格式：
  * 256 点覆盖 0..360°，步长 360/256 = **1.40625°**
  * 值为 **Q15 有符号**（±32767 表示 ±1.0）
  * 索引 i 对应角度 = i × 1.40625°，即 i = yaw_deg × 256 / 360

用法：
    python tools/gen_sin_lut.py
    python tools/gen_sin_lut.py --out rtl/odom/sin_lut.vh --points 256
"""

import argparse
import math
import os
import sys


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    ap = argparse.ArgumentParser(description="生成 sin/cos 查找表（Verilog 头文件）")
    ap.add_argument("--out", default=os.path.join(root, "rtl", "odom", "sin_lut.vh"))
    ap.add_argument("--points", type=int, default=256, help="表长（必须为 2 的幂）")
    ap.add_argument("--q", type=int, default=15, help="定点位数（Q15 → ±32767）")
    args = ap.parse_args()

    n = args.points
    if n & (n - 1):
        raise SystemExit("[错误] --points 必须是 2 的幂")
    scale = (1 << args.q) - 1
    idx_bits = int(math.log2(n))

    lines = []
    a = lines.append
    a("// sin_lut.vh —— 航位推算用 sin/cos 查找表（由 tools/gen_sin_lut.py 自动生成，请勿手改）")
    a("//")
    a(f"// 表长：{n} 点，覆盖 0..360°，步长 {360.0 / n:.5f}°")
    a(f"// 格式：Q{args.q} 有符号（±{scale} 表示 ±1.0）")
    a(f"// 索引：i = yaw_deg × {n} / 360  （即 i = yaw_001deg × {n} / 36000）")
    a("//")
    a("// ★ cos 不单独存表：cos(θ) = sin(θ + 90°)，索引加 "
      f"{n // 4}（= 90° / 步长）即可，省一半 ROM。")
    a("")
    a(f"localparam integer SIN_LUT_N      = {n};")
    a(f"localparam integer SIN_LUT_IDX_B  = {idx_bits};")
    a(f"localparam integer SIN_LUT_QUARTER = {n // 4};")
    a(f"localparam integer SIN_LUT_Q      = {args.q};")
    a("")
    a("// 由 odom.v 例化：reg signed [15:0] sin_lut [0:SIN_LUT_N-1];")
    a("// 用 generate/initial 初始化（内嵌，无需 $readmemh）")
    a("`define SIN_LUT_INIT \\")
    for i in range(n):
        v = int(round(math.sin(2.0 * math.pi * i / n) * scale))
        if v > scale:
            v = scale
        if v < -scale - 1:
            v = -scale - 1
        tail = " \\" if i != n - 1 else ""
        a(f"    sin_lut[{i:5d}] = 16'sd{v:6d};{tail}")
    a("")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"wrote {args.out}")
    print(f"  {n} 点  Q{args.q}  步长 {360.0 / n:.5f}°  索引 {idx_bits} 位")
    print(f"  BSRAM/ROM 估算：{n} × 16 bit = {n * 2} 字节")
    return 0


if __name__ == "__main__":
    sys.exit(main())
