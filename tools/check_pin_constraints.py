#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按《ACG720_引脚接线手册_RevI.pdf》校验本工程的引脚约束（constr/falcons.cst）。

用途
----
约束文件里的引脚一旦与硬件手册不符，上板就是整排错位、且症状往往像"模块坏了"。
本脚本从手册 PDF 原文解析出「连接器针号 -> 主板网络(GPIO) -> J1正式Pin」的权威映射，
再与 .cst 里实际生效的 IO_LOC 逐针比对，把差异直接指出来。

校验内容
--------
1. 电机 21 根信号（4 路 x (IN1/IN2/PWM) + STBY + 4 路 x (A/B) 编码器）与手册 J4/J5 表逐针一致
2. IMU 4 根 SPI 信号（J2 接口）与手册 J2 表逐针一致
3. ToF 4 根 I2C 线（SDA/SCL/INT/SHUT）与手册 J7 表逐针一致
4. 遥控 6 路（CH1~CH6）与手册 J6 表逐针一致
5. constr/falcons.cst 内部引脚无重复分配
6. .cst 注释里写的 "J1.xx" 是否与手册的 J1正式Pin 相符（防注释与代码脱节）

> 2026-10-15 扩展：原本只校验电机 + IMU（25 根）。正是这次查手册 J6/J7 才发现
> **顶层把 J7 的 SDA/SCL 误当成"ToF 回波 + RK3588 串口"** —— 所以把 J6/J7 也纳入校验。

用法
----
    python tools/check_pin_constraints.py

依赖
----
    pdftotext（TeXLive / poppler 自带）；若不在 PATH，回退到 PyMuPDF（pip install pymupdf）

退出码：0 = 全部通过；1 = 存在不符

---
本文件迁自 `class\\car_fpga\\tools\\check_pin_constraints.py`（2026-10-15），针对本工程做了三处适配：
  ① 手册位置：工程根 -> `doc\\`（本工程把硬件资料收在 doc 下）
  ② 约束文件：car_fpga 是 motor_test.cst + imu_demo.cst 两个，本工程合并为 `constr\\falcons.cst` 一个
  ③ 端口名：本工程 TB6612 的待机脚叫 `o_stby`（car_fpga 叫 `o_motor_stby`）
其余解析与比对逻辑保持原样（锚仍是"跨页唯一"的那几个，见 parse_manual 的注释）。
"""

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]          # FALCONS-embodied-robot/
MANUAL = ROOT / "doc" / "ACG720_引脚接线手册_RevI.pdf"
CST = ROOT / "constr" / "falcons.cst"

# 手册里连接器"功能名" -> cst 顶层端口名。
# 注：本工程轮位语义 = A 左前 / B 右前 / C 左后 / D 右后（car_fpga 的注释写作 左前/左后/右前/右后，
#     只是命名习惯不同；这里只按 J4/J5 表格里的 A/B/C/D 索引对应，不影响逐针比对）。
IMU_PORTS = {
    "SCL/SPC": "o_imu_sck",
    "SDA/SDI": "o_imu_mosi",
    "SA0/SDO": "i_imu_miso",
    "CS": "o_imu_cs_n",
}
WHEEL_IDX = {"A": 0, "B": 1, "C": 2, "D": 3}

# ToF（J7 排针，I2C）：手册功能名 -> 顶层端口名（VIN/GND 是电源地，不校验）
TOF_PORTS = {
    "SDA": "io_tof_sda",
    "SCL": "o_tof_scl",
    "INT": "i_tof_int",
    "SHUT": "o_tof_shut",
}


def rc_port(sig: str):
    """遥控（J6）：手册里的 CH1~CH6 -> 顶层 i_rc_pwm[0..5]。"""
    m = re.fullmatch(r"CH([1-6])", sig)
    return f"i_rc_pwm[{int(m.group(1)) - 1}]" if m else None


# J6 在手册里是**左右两栏交错排版**（左栏"针号 CHn 网络"、右栏"针号 CHn 网络 J1Pin"），
# 一行里可能同时出现两组数据，所以不能用通用的四列 ROW_RE —— 用 finditer 全都捞出来。
J6_ROW_RE = re.compile(r"(\d+)\s+CH(\d+)\s+([A-Z]+\d+)(?:\s+(\d+))?")


def parse_j6(lines, start_re, end_re) -> dict:
    rows, inside = {}, False
    for line in lines:
        if not inside:
            if re.search(start_re, line):
                inside = True
            continue
        if re.search(end_re, line):
            break
        for m in J6_ROW_RE.finditer(line):
            ch = int(m.group(2))
            net, j1 = m.group(3), (m.group(4) or "")
            if ch > 6:
                continue
            # 同一 CH 可能在两栏各出现一次：优先保留带 J1Pin 的那条
            if ch not in rows or (not rows[ch]["j1"] and j1):
                rows[ch] = {"sig": f"CH{ch}", "net": net, "j1": j1}
    return rows


def motor_port(sig: str):
    """把手册里的电机信号名映射到本工程顶层的端口名。"""
    m = re.fullmatch(r"([ABCD])IN([12])", sig)
    if m:
        return f"o_motor_in{m.group(2)}[{WHEEL_IDX[m.group(1)]}]"
    m = re.fullmatch(r"PWM([ABCD])", sig)
    if m:
        return f"o_motor_pwm[{WHEEL_IDX[m.group(1)]}]"
    m = re.fullmatch(r"E([1234])([AB])", sig)
    if m:
        return f"i_enc_{m.group(2).lower()}[{int(m.group(1)) - 1}]"
    if sig == "STBY":
        return "o_stby"          # ★ 本工程端口名（car_fpga 是 o_motor_stby）
    return None


def manual_lines() -> list:
    """把手册 PDF 提取成文本行（-layout 保留表格列对齐）。"""
    out = Path(tempfile.gettempdir()) / "pin_manual_check.txt"
    pdftotext = shutil.which("pdftotext")
    if pdftotext:
        subprocess.run(
            [pdftotext, "-layout", "-enc", "UTF-8", str(MANUAL), str(out)],
            check=True, capture_output=True,
        )
        return out.read_text(encoding="utf-8").splitlines()

    try:                                   # 回退：PyMuPDF
        import pymupdf
    except ImportError:
        sys.exit("找不到 pdftotext，也没装 pymupdf，无法提取手册 PDF。")
    doc = pymupdf.open(str(MANUAL))
    return "\n".join(p.get_text(sort=True) for p in doc).splitlines()


ROW_RE = re.compile(
    r"^\s*(\d+)\s+(\S+)\s+([A-Z]+\d+|GND|5V|3V3|NC)\s+(\S+)\s+.*$"
)


def parse_table(lines, start_re, end_re) -> dict:
    """解析手册里"针号 功能 网络 J1Pin 说明"式表格。

    start/end 必须用**跨页唯一**的锚：手册封面与每页页眉都印有 "J4 电机驱动 A/B"
    之类的字样，用宽松锚会在封面就误触发、并在页眉处提前结束（踩过的坑）。
    """
    rows, inside = {}, False
    for line in lines:
        if not inside:
            if re.search(start_re, line):
                inside = True
            continue
        if re.search(end_re, line):
            break
        m = ROW_RE.match(line)
        if m:
            rows[int(m.group(1))] = {"sig": m.group(2), "net": m.group(3), "j1": m.group(4)}
    return rows


def parse_manual() -> dict:
    lines = manual_lines()
    # "Pin1悬空，保留原位置" 只出现在 J4/J5 的表格标题行，可作唯一锚
    j4 = parse_table(lines, r"^J4\s+电机驱动 A/B.*Pin1悬空", r"^J5\s+电机驱动 C/D.*Pin1悬空")
    j5 = parse_table(lines, r"^J5\s+电机驱动 C/D.*Pin1悬空", r"^Rev I")
    # J2 用 "1×6 公排针" 区分于封面上的 "J2 IMU" 示意图
    j2 = parse_table(lines, r"^J2\s+IMU\s+1×6 公排针", r"^J3\s+雷达")
    # J7（ToF）在手册里的位置：J3 之后、J4 之前
    j7 = parse_table(lines, r"^J7\s+ToF", r"^J4\s+电机驱动 A/B")
    # J6（遥控器信号）在 J5 之后、J9 之前
    j6 = parse_j6(lines, r"^J6\s+遥控器信号", r"^J9\s+遥控器电源")
    return {"J4": j4, "J5": j5, "J2": j2, "J7": j7, "J6": j6}


IO_LOC_RE = re.compile(r'IO_LOC\s+"([^"]+)"\s+([A-Z]+\d+)\s*;')
IO_LOC_J1_RE = re.compile(
    r'IO_LOC\s+"([^"]+)"\s+([A-Z]+\d+)\s*;\s*//[^\r\n]*?J1\.(\d+)'
)


def parse_cst(path: Path):
    text = path.read_text(encoding="utf-8")
    pins = {m.group(1): m.group(2) for m in IO_LOC_RE.finditer(text)}
    claims = [(m.group(1), m.group(2), int(m.group(3))) for m in IO_LOC_J1_RE.finditer(text)]
    return pins, claims


def main() -> int:
    if not MANUAL.exists():
        sys.exit(f"手册不存在：{MANUAL}（应从 class\\car_fpga 拷入 doc\\）")
    if not CST.exists():
        sys.exit(f"约束文件不存在：{CST}")

    man = parse_manual()
    print("=== 手册解析 ===")
    for name in ("J2", "J4", "J5", "J6", "J7"):
        print(f"  {name}: {len(man[name])} 针")

    failures = []

    # ---- 期望映射 ----
    expect = {}
    for pin, row in man["J4"].items():
        port = motor_port(row["sig"])
        if port:
            expect[port] = (row["net"], row["j1"])
    for pin, row in man["J5"].items():
        port = motor_port(row["sig"])
        if port:
            expect[port] = (row["net"], row["j1"])
    imu_expect = {
        IMU_PORTS[row["sig"]]: (row["net"], row["j1"])
        for row in man["J2"].values() if row["sig"] in IMU_PORTS
    }
    tof_expect = {
        TOF_PORTS[row["sig"]]: (row["net"], row["j1"])
        for row in man["J7"].values() if row["sig"] in TOF_PORTS
    }
    for name, expected_count in (("J2", 6), ("J4", 12), ("J5", 12), ("J6", 6), ("J7", 6)):
        if len(man[name]) != expected_count:
            failures.append(f"{name}: parsed {len(man[name])} pins, expected {expected_count}")
    rc_expect = {}
    for row in man["J6"].values():
        port = rc_port(row["sig"])
        if port:
            rc_expect[port] = (row["net"], row["j1"])

    pins, claims = parse_cst(CST)

    # ---- 电机（21 根）----
    print(f"\n=== 电机约束 {CST.name} ===")
    if len(expect) != 21:
        failures.append(f"手册解析出的电机信号数为 {len(expect)}，应为 21 —— 检查解析锚")
    for port, (net, _) in sorted(expect.items()):
        actual = pins.get(port)
        if actual != net:
            failures.append(f"motor {port}: 手册={net} cst={actual}")
    print(f"  比对 {len(expect)} 根信号 -> " +
          ("全部一致" if not any(f.startswith('motor ') for f in failures) else "存在不符"))

    # ---- IMU（4 根）----
    print(f"\n=== IMU 约束 {CST.name} ===")
    for port, (net, _) in sorted(imu_expect.items()):
        actual = pins.get(port)
        if actual != net:
            failures.append(f"imu {port}: 手册={net} cst={actual}")
    print(f"  比对 {len(imu_expect)} 根信号 -> " +
          ("全部一致" if not any(f.startswith('imu ') for f in failures) else "存在不符"))

    # ---- ToF（4 根，J7 的 I2C 四线）----
    print(f"\n=== ToF 约束 {CST.name}（J7 排针）===")
    for port, (net, _) in sorted(tof_expect.items()):
        actual = pins.get(port)
        if actual != net:
            failures.append(f"tof {port}: 手册={net} cst={actual}")
    print(f"  比对 {len(tof_expect)} 根信号 -> " +
          ("全部一致" if not any(f.startswith('tof ') for f in failures) else "存在不符"))

    # ---- 遥控（6 根，J6 整排）----
    print(f"\n=== 遥控约束 {CST.name}（J6 整排）===")
    for port, (net, _) in sorted(rc_expect.items()):
        actual = pins.get(port)
        if actual != net:
            failures.append(f"rc {port}: 手册={net} cst={actual}")
    print(f"  比对 {len(rc_expect)} 根信号 -> " +
          ("全部一致" if not any(f.startswith('rc ') for f in failures) else "存在不符"))

    # ---- 引脚唯一性（本工程只有一个 cst，查它的全部引脚）----
    print("\n=== 引脚唯一性 ===")
    seen = {}
    for port, pin in pins.items():
        seen.setdefault(pin, []).append(port)
    dups = {p: v for p, v in seen.items() if len(v) > 1}
    if dups:
        for pin, ports in dups.items():
            failures.append(f"{CST.name} 引脚 {pin} 重复分配: {', '.join(ports)}")
        print(f"  {CST.name}: 存在重复")
    else:
        print(f"  {CST.name}: {len(pins)} 个引脚无重复")

    # ---- 注释里的 J1.xx 是否与手册一致 ----
    print("\n=== 注释 J1正式Pin 核对 ===")
    net2j1 = {}
    for name in ("J4", "J5", "J2", "J6", "J7"):
        for row in man[name].values():
            if row["j1"]:                     # J6 左栏没有 J1Pin，跳过空值
                net2j1[row["net"]] = row["j1"]
    bad = 0
    for port, pin, claim in claims:
        truth = net2j1.get(pin)
        if truth and "/" not in truth and int(truth) != claim:
            failures.append(f"{port}: 注释 J1.{claim}，但 {pin} 实为 J1.{truth}")
            bad += 1
    print(f"  {CST.name}: {len(claims)} 条注释, {bad} 条不符")

    print()
    if failures:
        print("=== 结果：不符 ===")
        for f in failures:
            print(f"  X {f}")
        return 1
    print("=== 结果：全部通过（与手册逐针一致） ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
