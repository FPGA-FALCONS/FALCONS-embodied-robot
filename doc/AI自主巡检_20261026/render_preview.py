"""Render a synthetic three-view design illustration, not a hardware screenshot."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent


def main():
    normal = Path("C:/Windows/Fonts/msyh.ttc")
    bold = Path("C:/Windows/Fonts/msyhbd.ttc")
    if not normal.is_file() or not bold.is_file():
        raise SystemExit("Provide Microsoft YaHei font paths in this offline drawing script")
    fonts = {s: ImageFont.truetype(str(normal), s) for s in (18, 20, 22, 24, 26, 28)}
    title = ImageFont.truetype(str(bold), 40)
    im = Image.new("RGB", (1800, 850), "#eff3f8")
    d = ImageDraw.Draw(im)
    d.text((40, 25), "AI 自主巡检：同一任务，三个视图", font=title, fill="#14243b")
    d.text((42, 85), "设计示意 · 全部画面、物体与坐标均为合成示例 · 不是上板截图", font=fonts[22], fill="#52647c")
    headers = ["摄像头", "雷达地图", "巡检记录"]
    for index, name in enumerate(headers):
        left = 32 + index * 590
        d.rounded_rectangle((left, 140, left + 556, 725), radius=18, fill="#ffffff", outline="#ccd5e1", width=2)
        d.text((left + 20, 158), "AI 模块 / 自主巡检", font=fonts[26], fill="#17273f")
        for j, tab in enumerate(headers):
            x = left + 16 + j * 174
            selected = j == index
            d.rounded_rectangle((x, 207, x + 164, 249), radius=7,
                                fill="#2563eb" if selected else "#e8edf5")
            d.text((x + 22, 214), tab, font=fonts[20], fill="white" if selected else "#52647c")
        d.text((left + 20, 670), "select 切视图   确认/返回：暂停", font=fonts[20], fill="#52647c")
        d.text((left + 20, 700), "任务保持运行，感知与地图持续更新", font=fonts[18], fill="#52647c")

    # Camera mock. A drawn object and box are explicitly synthetic.
    x = 48
    d.rectangle((x, 270, x + 523, 640), fill="#203247")
    d.polygon([(x, 640), (x + 170, 360), (x + 350, 360), (x + 523, 640)], fill="#465a6e")
    d.line((x + 40, 640, x + 190, 365), fill="#a2b4c4", width=3)
    d.line((x + 485, 640, x + 330, 365), fill="#a2b4c4", width=3)
    d.rounded_rectangle((x + 225, 456, x + 250, 505), radius=5, fill="#f4a340")
    d.polygon([(x + 235, 505), (x + 243, 505), (x + 263, 564), (x + 257, 567)], fill="#b6c0c8")
    d.rectangle((x + 203, 438, x + 283, 585), outline="#42e7a1", width=3)
    d.rectangle((x + 169, 405, x + 340, 437), fill="#102d26")
    d.text((x + 177, 408), "#02 螺丝刀", font=fonts[22], fill="#42e7a1")
    d.text((x + 15, 286), "同帧框 · 位置已记录", font=fonts[22], fill="#ffffff")
    d.text((x + 15, 604), "相机图像示意，不代表检测效果", font=fonts[20], fill="#e5edf8")

    # Map mock: red objects, outbound and independently replanned return route.
    x = 638
    d.rectangle((x, 270, x + 523, 640), fill="#edf2f7")
    for gx in range(x + 10, x + 524, 26):
        d.line((gx, 270, gx, 640), fill="#dce4ef")
    for gy in range(280, 640, 26):
        d.line((x, gy, x + 523, gy), fill="#dce4ef")
    for rect in [(x + 15, 350, x + 110, 380), (x + 355, 430, x + 500, 460),
                 (x + 20, 580, x + 100, 612)]:
        d.rectangle(rect, fill="#617083")
    route = [(x + 90, 540), (x + 180, 540), (x + 180, 475), (x + 275, 475),
             (x + 275, 350), (x + 400, 350)]
    d.line(route, fill="#2689d8", width=6)
    d.line([(x + 400, 350), (x + 515, 350), (x + 515, 570), (x + 110, 570),
            (x + 90, 540)], fill="#a36bd2", width=3)
    for n, (ox, oy) in enumerate([(x + 220, 535), (x + 320, 410)], 1):
        d.ellipse((ox - 25, oy - 25, ox + 25, oy + 25), fill="#f8d3d8", outline="#ec7787", width=2)
        d.ellipse((ox - 6, oy - 6, ox + 6, oy + 6), fill="#db354f")
        d.text((ox + 12, oy - 38), f"#{n:02}", font=fonts[20], fill="#b72e45")
    d.ellipse((x + 80, 530, x + 100, 550), fill="#18a575")
    d.text((x + 28, 548), "起点", font=fonts[18], fill="#168761")
    d.rectangle((x + 390, 340, x + 410, 360), fill="#dfab20")
    d.text((x + 405, 300), "终点", font=fonts[18], fill="#95720c")
    d.polygon([(x + 500, 570), (x + 519, 559), (x + 519, 581)], fill="#14243b")
    d.text((x + 12, 283), "红点：物体   淡红：障碍包络", font=fonts[20], fill="#52647c")
    d.text((x + 12, 606), "蓝：去程    紫：自主返航", font=fonts[20], fill="#52647c")

    # Record view. The values are presentation fixtures, not measured positions.
    x = 1228
    d.text((x + 5, 280), "阶段：返程中    已记录：2", font=fonts[24], fill="#1f3853")
    d.text((x + 5, 324), "坐标：导航起点坐标系 / m", font=fonts[20], fill="#52647c")
    rows = [("#01 胶带", "x = 1.20   y = -0.35", "已记录 · 已绕过"),
            ("#02 螺丝刀", "x = 2.15   y = +0.40", "已记录 · 已绕过")]
    for i, (label, coord, state) in enumerate(rows):
        yy = 372 + i * 126
        d.rounded_rectangle((x + 2, yy, x + 515, yy + 110), radius=9, fill="#f1f5fa")
        d.text((x + 17, yy + 9), label, font=fonts[24], fill="#192e48")
        d.text((x + 17, yy + 43), coord, font=fonts[22], fill="#364d68")
        d.text((x + 17, yy + 77), state, font=fonts[20], fill="#188663")
    d.text((x + 7, 627), "示意数值；实测误差需另行标定", font=fonts[18], fill="#9b5f26")
    d.text((40, 765), "流程：地图选终点 → 沿途识别定位 → 融合障碍并绕行 → 重新规划回起点 → 保留坐标清单", font=fonts[24], fill="#243e5b")
    d.text((40, 808), "已支持八方向斜线；任意角度路径为离线候选。未知位置/感知失效/无路：停车并提示。", font=fonts[20], fill="#52647c")
    target = HERE / "三视图示意.png"
    im.save(target)
    print(target)


if __name__ == "__main__":
    main()
