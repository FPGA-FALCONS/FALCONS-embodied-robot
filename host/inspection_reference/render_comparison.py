"""Draw actual synthetic benchmark paths; requires Pillow only for rendering."""
from pathlib import Path
import json
import math
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent


def main():
    report = json.loads((HERE / "evidence/benchmark.json").read_text(encoding="utf-8"))
    font_path = "C:/Windows/Fonts/msyh.ttc"
    fonts = {s: ImageFont.truetype(font_path, s) for s in (18, 20, 22, 24, 28, 38)}
    image = Image.new("RGB", (1740, 820), "#eef3f8")
    draw = ImageDraw.Draw(image)
    draw.text((32, 22), "从八方向斜线到任意角度：离线规划实测", font=fonts[38], fill="#152a43")
    pct = report["theta_vs_astar8"]["mean_per_map_length_reduction_percent"]
    draw.text((34, 82), f"固定合成地图：26张 / 23张共同可达；相对软件八方向A*平均路长减少 {pct:.2f}%", font=fonts[22], fill="#52647c")
    draw.text((34, 118), "不是正式RTL路径/时延对照；任意角度尚未接入FPGA执行。灰格为已膨胀障碍。", font=fonts[20], fill="#80532a")
    scenarios = {r["name"]: r for r in report["scenarios"]}
    panels = [scenarios["open_oblique"], scenarios["wall_detour"], report["return_replanning"]]
    names = ["1  不再局限水平/45°", "2  任意角度也逐段避障", "3  旧路受阻，另找路回起点"]
    for i, (row, heading) in enumerate(zip(panels, names)):
        left = 22 + i * 574
        draw.rounded_rectangle((left, 175, left + 550, 732), radius=15, fill="white", outline="#cdd8e5", width=2)
        draw.text((left + 16, 191), heading, font=fonts[24], fill="#1d3753")
        grid = row["map"]
        scale = min(480 / grid["width"], 340 / grid["height"])
        x0 = left + (550 - grid["width"] * scale) / 2
        y0 = 252 + (340 - grid["height"] * scale) / 2

        def xy(p):
            return x0 + (p[0] + .5) * scale, y0 + (grid["height"] - p[1] - .5) * scale

        draw.rectangle((x0, y0, x0 + grid["width"] * scale, y0 + grid["height"] * scale), fill="#f0f4f8", outline="#b8c9da")
        for x in range(grid["width"] + 1):
            draw.line((x0 + x * scale, y0, x0 + x * scale, y0 + grid["height"] * scale), fill="#dde6ef")
        for y in range(grid["height"] + 1):
            draw.line((x0, y0 + y * scale, x0 + grid["width"] * scale, y0 + y * scale), fill="#dde6ef")
        for cell in grid["blocked"]:
            cx, cy = xy(cell)
            draw.rectangle((cx - scale / 2, cy - scale / 2, cx + scale / 2, cy + scale / 2), fill="#687b8f" if i < 2 else "#d34c63")
        if i < 2:
            first = row["plans"]["astar8_euclidean"]
            second = row["plans"]["theta"]
            start, goal = row["start"], row["goal"]
            draw.line([xy(p) for p in first["path"]], fill="#397bd0", width=5)
            draw.line([xy(p) for p in second["path"]], fill="#e58a27", width=4)
            for p in second["path"]:
                cx, cy = xy(p)
                draw.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), fill="#b66212")
            draw.text((left + 18, 615), f"八方向A*：{first['length_m']:.3f}m / {first['turns']}次转折", font=fonts[22], fill="#397bd0")
            draw.text((left + 18, 654), f"Theta*：{second['length_m']:.3f}m / {second['turns']}次转折", font=fonts[22], fill="#b96a15")
        else:
            start, goal = row["current"], row["home"]
            a, b = xy(start), xy(goal)
            for part in range(0, 30, 2):
                p = (a[0] + (b[0] - a[0]) * part / 30, a[1] + (b[1] - a[1]) * part / 30)
                q = (a[0] + (b[0] - a[0]) * (part + 1) / 30, a[1] + (b[1] - a[1]) * (part + 1) / 30)
                draw.line((p, q), fill="#a2aebc", width=3)
            draw.line([xy(p) for p in row["return"]["path"]], fill="#e58a27", width=4)
            draw.text((left + 18, 615), "灰虚线：原直返路径已被阻断", font=fonts[22], fill="#687b8f")
            draw.text((left + 18, 654), "橙线：按新障碍地图重规划返航", font=fonts[22], fill="#b96a15")
        for p, color in ((start, "#1ca07a"), (goal, "#273b59")):
            cx, cy = xy(p)
            draw.ellipse((cx - 6, cy - 6, cx + 6, cy + 6), fill=color)
        draw.text((left + 18, 699), "线段触碰障碍角也拒绝；每格50mm", font=fonts[18], fill="#52647c")
    draw.text((32, 762), "已完成软件原型与碰撞检查；未包含相机误差、车体动力学、连续曲线跟踪或硬件时延验证。", font=fonts[22], fill="#52647c")
    target = HERE / "evidence/算法对照.png"
    image.save(target)
    print(target)


if __name__ == "__main__":
    main()
