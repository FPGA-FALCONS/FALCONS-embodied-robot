#!/usr/bin/env python3
"""生成清图取证场景，运行原 RTL 的 UART 仿真，汇总并绘图。"""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
INVALID = 65535
WALLS = [(-5000, -3500, 5000, -3500), (5000, -3500, 5000, 3500),
         (5000, 3500, -5000, 3500), (-5000, 3500, -5000, -3500),
         (1000, -1800, 1000, 1800)]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def range_mm(pose, angle):
    theta = math.radians(pose[2] / 100 - angle / 100)
    ray = math.cos(theta), math.sin(theta)
    nearest = 10000.0
    for x1, y1, x2, y2 in WALLS:
        edge, delta = (x2 - x1, y2 - y1), (x1 - pose[0], y1 - pose[1])
        den = cross(ray, edge)
        if abs(den) < 1e-12:
            continue
        t, u = cross(delta, edge) / den, cross(delta, ray) / den
        if 0 <= t < nearest and 0 <= u <= 1:
            nearest = t
    return round(nearest) if nearest < 9999.5 else 0


def fixtures(out, baseline=False):
    lines, snapshots = [], []

    def packet(angle, values, pose=(0, 0, 0)):
        assert len(values) <= 42
        lines.append("1 " + " ".join(map(str, (*pose, angle, *values,
                                              *([INVALID] * (42 - len(values)))))))

    def boundary():
        packet(34500, [])
        packet(0, [])
        # 换圈标记由 polar 携带至下一条有效射线，填充槽不会直接启动清理。
        packet(9000, [1500])

    def snapshot(label, group, pose=(0, 0, 0)):
        i = len(snapshots)
        snapshots.append(dict(id=i, label=label, group=group, pose=pose))
        lines.append(f"2 {i}")

    def check(x, y, value):
        lines.append(f"3 {x} {y} {value}")

    def scan(true_pose, measured_pose=None, dropout=False):
        for angle in range(0, 36000, 1500):
            values = []
            # 实际采集文件每包主要有 40/41 个非填充槽，取 40。
            for i in range(40):
                a = (angle + i * 1500 // 40) % 36000
                d = range_mm(true_pose, a)
                # 可重复的稀疏无回波假设，不冒充真实采集时的故障分布。
                if dropout and ((angle // 1500 * 40 + i) % 5 == 0):
                    d = 0
                values.append(d)
            packet(angle, values, measured_pose or true_pose)

    lines.append("0")
    packet(0, [1000])
    snapshot("初次发现障碍", "unit")
    check(148, 128, 1)
    # 后续多圈的射线在 +y 轴，与历史 +x 障碍不相交。
    for _ in range(3):
        packet(9000, [1500])
        boundary()
    snapshot("未被经过：跨圈仍保留", "unit")
    check(148, 128, 1)
    # 同一圈内先 hit 再 free，跨圈也不能清除该圈 hit。
    packet(0, [1000])
    packet(0, [2000])
    boundary()
    check(148, 128, 1)
    # 下一圈只有零距离射线（按现行规则推至 10 m）。
    packet(0, [0])
    snapshot("0 距离经过，尚未结圈", "unit")
    check(148, 128, 1)
    boundary()
    snapshot("下一圈清理：旧障碍消失", "unit")
    check(148, 128, 0)
    # 相同现象不要求零距离：更远的正常命中射线也会清掉前面的旧格。
    lines.append("0")
    packet(0, [1000])
    boundary()
    packet(0, [2000])
    boundary()
    check(148, 128, 0)
    check(168, 128, 1)

    # 同一组实际墙面：精确定位、逐渐远离、原地转弯。
    lines.append("0")
    for label, pose in [("近处：准确定位", (0, -300, 0)),
                        ("远离：准确定位", (-3000, -300, 0)),
                        ("转弯 90°：准确定位", (-3000, -300, 9000))]:
        scan(pose)
        scan(pose)
        snapshot(label, "ideal", pose)
    # 错位的地图仍来自同一组真实墙，不移动物体。
    true_pose = (-3000, -300, 9000)
    measured = (-2920, -340, 9100)
    scan(true_pose, measured)
    scan(true_pose, measured)
    snapshot("同一场地：定位偏 80/-40 mm、朝向偏 1°", "ideal", measured)

    # 单独复现实物未动、没有定位误差，只有部分无回波。
    lines.append("0")
    for label, pose, missing in [("近处：正常命中", (0, -300, 0), False),
                                 ("远离：20% 无回波", (-3000, -300, 0), True),
                                 ("转弯：20% 无回波", (-3000, -300, 9000), True)]:
        scan(pose, dropout=missing)
        scan(pose, dropout=missing)
        snapshot(label, "dropout", pose)

    # 两束射线从同一格的不同高度经过：一束被近物挡住，一束看见更远处。
    lines.append("0")
    packet(0, [1000], (25, 25, 0))
    boundary()
    snapshot("记录后方旧障碍", "occlusion", (25, 25, 0))
    packet(0, [500], (25, 25, 0))
    packet(0, [2000], (25, 49, 0))
    boundary()
    snapshot("近处遮挡与远处空闲意见冲突", "occlusion", (25, 49, 0))
    check(148, 128, 0 if baseline else 1)
    check(138, 128, 1)
    check(168, 128, 1)
    check(158, 128, 0)
    packet(0, [2000], (25, 49, 0))
    boundary()
    snapshot("近物移走：后方明确可通行", "occlusion", (25, 49, 0))
    check(148, 128, 0)
    check(138, 128, 0)
    check(168, 128, 1)
    packet(0, [0], (25, 49, 0))
    boundary()
    snapshot("全部移走：无回波清除远端", "occlusion", (25, 49, 0))
    check(168, 128, 0)

    (out / "input.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (out / "snapshots.json").write_text(json.dumps(snapshots, ensure_ascii=False, indent=2),
                                         encoding="utf-8")
    return snapshots


def run_sim(out, modelsim, rtl_root, continuous=False):
    lib = out / "work_retention"
    vendor = rtl_root / "vendor/verilog-reusable-modules/modules"
    sources = [vendor / "uart/rtl/uart_byte_rx.v",
               vendor / "m10_radar/rtl/m10_packet_parser.v",
               vendor / "m10_radar/rtl/m10_radar.v"]
    sources += [vendor / "auto_navigation/rtl" / (name + ".v")
                for name in ("nav_polar", "nav_scanner", "nav_scan_accumulator", "nav_map")]
    sources.append(rtl_root / "sim/map_retention_tb.v")

    def run(name, args):
        result = subprocess.run([str(modelsim / (name + ".exe")),
                                 *[str(a).replace("\\", "/") for a in args]],
                                cwd=out, capture_output=True, text=True, errors="replace")
        (out / (name + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode or "** Error" in result.stdout or "FAIL:" in result.stdout:
            raise RuntimeError((result.stdout + result.stderr)[-4000:])
        return result.stdout

    if not lib.exists():
        run("vlib", [lib])
    run("vlog", ["-work", lib, *sources])
    args = ["-c", "-lib", lib, "map_retention_tb", "+INPUT=input.txt", "+OUT=."]
    if continuous:
        args.append("+CONTINUOUS")
    output = run("vsim", [*args, "-do", "run -all; quit -f", "-l", "transcript.log"])
    if "PASS: map_retention_tb" not in output:
        raise RuntimeError("simulation did not finish")
    print("\n".join(line for line in output.splitlines()
                    if any(key in line for key in ("METRICS:", "CHECKS:", "PASS:", "SNAPSHOT:"))),
          flush=True)


def summarize(out, snapshots):
    maps = {}
    for entry in snapshots:
        maps[entry["id"]] = {tuple(map(int, line.split()))
                              for line in (out / f'map_{entry["id"]}.txt').read_text().splitlines()}
    events = list(csv.DictReader((out / "clear_events.csv").open()))
    stats = []
    for entry in snapshots:
        cells = maps[entry["id"]]
        item = dict(entry, occupied=len(cells))
        # 固定几何 ROI：内部墙 x=1000 mm，对应 x 格148，边界误差可落147。
        wall = {(x, y) for x, y in cells if 147 <= x <= 148 and 92 <= y <= 164}
        item["interior_wall_rows"] = len({y for x, y in wall})
        prior = next((p for p in reversed(stats) if p["group"] == entry["group"]), None)
        if prior:
            before = maps[prior["id"]]
            item.update(lost_from_prior=len(before - cells), new_from_prior=len(cells - before))
        stats.append(item)
    summary = dict(snapshots=stats, clear_count=len(events),
                   clear_distance_sources=dict(Counter("zero" if e["distance"] == "0" else "nonzero"
                                                       for e in events)),
                   protected_clears=sum(int(e["hit_epoch"]) >= int(e["circle"]) - 1 for e in events),
                   free_epoch_mismatches=sum(int(e["free_epoch"]) != int(e["circle"]) - 1
                                             for e in events))
    assert summary["protected_clears"] == summary["free_epoch_mismatches"] == 0
    assert maps[5] == maps[6], "accurate 90-degree turn must retain the same map"
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    draw(out, stats, maps)


def draw(out, stats, maps):
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
    small = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 16)
    panels = [stats[i] for i in (4, 5, 7, 9)]
    panel_w, panel_h = 550, 460
    image = Image.new("RGB", (panel_w * 2, panel_h * 2 + 85), "white")
    draw = ImageDraw.Draw(image)
    draw.text((20, 10), "真实 UART + 现有建图 RTL：远离、转弯后的地图留存", fill="black", font=font)
    draw.text((20, 44), "灰：真实墙  黑：当前 raw 障碍  红：上一图有、本图已清除的格  蓝：定位位置", fill="black", font=small)
    for i, e in enumerate(panels):
        left, top = (i % 2) * panel_w, (i // 2) * panel_h + 85
        draw.text((left + 12, top + 5), e["label"], fill="black", font=small)
        draw.text((left + 12, top + 31),
                  f'raw={e["occupied"]}  内墙覆盖行={e["interior_wall_rows"]}/73  '
                  f'较上图清除={e.get("lost_from_prior", 0)}', fill="black", font=small)
        scale = 0.038

        def xy(x, y):
            return left + 275 + x * scale, top + 255 - y * scale

        for wall in WALLS:
            draw.line((*xy(*wall[:2]), *xy(*wall[2:])), fill="#cccccc", width=4)
        cells = maps[e["id"]]
        prior = next((s for s in reversed(stats[:e["id"]]) if s["group"] == e["group"]), None)
        before = maps[prior["id"]] if prior else set()
        for x, y in before - cells:
            a, b = xy(x * 50 - 6375, y * 50 - 6375)
            draw.rectangle((a-1, b-1, a+1, b+1), fill="#d33b31")
        for x, y in cells:
            a, b = xy(x * 50 - 6375, y * 50 - 6375)
            draw.rectangle((a-1, b-1, a+1, b+1), fill="#202124")
        x, y, heading = e["pose"]
        a, b = xy(x, y)
        draw.ellipse((a-4, b-4, a+4, b+4), fill="#1866d4")
        r = math.radians(heading / 100)
        draw.line((a, b, a + 20*math.cos(r), b-20*math.sin(r)), fill="#1866d4", width=3)
    image.save(out / "map_retention.png")


def compare(out, baseline):
    from PIL import Image, ImageDraw, ImageFont
    before = json.loads((baseline / "summary.json").read_text(encoding="utf-8"))
    after = json.loads((out / "summary.json").read_text(encoding="utf-8"))

    def cells(directory, index):
        return {tuple(map(int, line.split()))
                for line in (directory / f"map_{index}.txt").read_text().splitlines()}

    # 比较使用同样的报文及采样位姿；仅遮挡冲突的预期值不同。
    packet_lines = lambda p: [s for s in (p / "input.txt").read_text().splitlines() if s.startswith("1 ")]
    assert packet_lines(out) == packet_lines(baseline)
    assert after["snapshots"][9]["lost_from_prior"] < before["snapshots"][9]["lost_from_prior"]
    assert (148, 128) not in cells(baseline, 12) and (148, 128) in cells(out, 12)
    assert (148, 128) not in cells(out, 13) and (138, 128) not in cells(out, 13)
    assert (168, 128) not in cells(out, 14)
    result = dict(before_clears=before["clear_count"], after_clears=after["clear_count"],
                  dropout_before=before["snapshots"][9], dropout_after=after["snapshots"][9],
                  shadow_retained=True, removed_obstacles_cleared=True, identical_packets=True)
    (out / "comparison.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
    small = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 17)
    image = Image.new("RGB", (1100, 520), "white")
    draw = ImageDraw.Draw(image)
    draw.text((20, 12), "同样的室内测量：远离后 20% 无回波，定位准确", fill="black", font=font)
    draw.text((20, 45), "黑：当前 raw 障碍　红：近处曾记录、远离后已清除的格　灰：真实墙", fill="black", font=small)
    for i, (directory, summary, title) in enumerate(((baseline, before, "修改前"), (out, after, "遮挡保护后"))):
        x0 = 550 * i
        row = summary["snapshots"][9]
        draw.text((x0 + 20, 82), f'{title}：清除旧格 {row["lost_from_prior"]}，内墙覆盖 {row["interior_wall_rows"]}/73 行',
                  fill="black", font=small)
        xy = lambda x, y: (x0 + 275 + x * .038, 320 - y * .038)
        for wall in WALLS:
            draw.line((*xy(*wall[:2]), *xy(*wall[2:])), fill="#cccccc", width=4)
        current = cells(directory, 9)
        for color, points in (("#d33b31", cells(directory, 8) - current), ("#202124", current)):
            for x, y in points:
                a, b = xy(x * 50 - 6375, y * 50 - 6375)
                draw.rectangle((a-1, b-1, a+1, b+1), fill=color)
    image.save(out / "shadow_retention_comparison.png")

    image = Image.new("RGB", (1100, 480), "white")
    draw = ImageDraw.Draw(image)
    draw.text((20, 12), "真实 UART 冲突及移走用例：每个圆点来自 RTL raw 地图", fill="black", font=font)
    draw.text((20, 45), "实心：障碍仍存在于地图　空心：该格无障碍；遮挡本身不生成新障碍", fill="black", font=small)
    labels = ["初始只记录后方旧点", "近物遮挡，另一束穿过", "近物移走，后方明确空闲", "远端也移走，收到无回波"]
    for column, (directory, title) in enumerate(((baseline, "修改前"), (out, "遮挡保护后"))):
        left = column * 550
        draw.text((left + 20, 85), title, fill="black", font=font)
        for j, name in enumerate(("近物", "后方旧点 G", "远端")):
            draw.text((left + 220 + j * 105, 118), name, fill="black", font=small)
        for row, index in enumerate((11, 12, 13, 14)):
            y = 175 + row * 85
            draw.text((left + 10, y - 10), labels[row], fill="black", font=small)
            current = cells(directory, index)
            for j, x in enumerate((138, 148, 168)):
                a = left + 245 + j * 105
                draw.ellipse((a-9, y-9, a+9, y+9),
                             fill="#2369b0" if (x, 128) in current else "white",
                             outline="#2369b0", width=2)
    image.save(out / "shadow_clearance_comparison.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "logs/map_retention_20261026")
    parser.add_argument("--modelsim", type=Path, default=Path("D:/apps/modelsim/win64"))
    parser.add_argument("--summarize-only", action="store_true")
    parser.add_argument("--rtl-root", type=Path, default=ROOT,
                        help="使用保存的修改前源码做对照；默认使用当前生产 RTL")
    parser.add_argument("--baseline", action="store_true", help="仅设置修改前遮挡用例的预期结果")
    parser.add_argument("--continuous", action="store_true", help="连续发送 UART 报文，不插入包间结算等待")
    parser.add_argument("--compare", type=Path, help="与修改前产物目录比较，并生成留存/移走对照图")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if args.summarize_only:
        snapshots = json.loads((out / "snapshots.json").read_text(encoding="utf-8"))
    else:
        snapshots = fixtures(out, args.baseline)
        print(f"Fixtures ready: {out}", flush=True)
        run_sim(out, args.modelsim, args.rtl_root.resolve(), args.continuous)
    summarize(out, snapshots)
    if args.compare:
        compare(out, args.compare.resolve())


if __name__ == "__main__":
    main()
