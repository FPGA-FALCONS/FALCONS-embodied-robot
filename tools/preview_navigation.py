#!/usr/bin/env python3
"""用 ModelSim 渲染实际导航 RTL，生成 PNG 与中文状态总览。"""
import argparse
from pathlib import Path
import subprocess
import zipfile

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
STATES = [
    ("00_radar_map", "雷达页：实际障碍地图"),
    ("01_navigation_idle", "避障页：尚未进入选点"),
    ("02_empty_selection", "选点：空列表"),
    ("03_destinations", "选点：已记录两个目的地"),
    ("04_undo_selected", "选点：选择撤销"),
    ("05_clear_selected", "选点：选择清空"),
    ("06_ready_start", "选点：准备开始导航"),
    ("07_running", "运行：含规划、转向及避障过程"),
    ("08_paused", "暂停：保留地图与目的地"),
    ("09_arrived", "已到达：最后一个目的地"),
    ("10_no_path", "无路：不跳过目的地"),
    ("11_emergency_stop", "整车急停：状态栏显示急停"),
    ("12_manual_takeover", "手动接管：自动任务保持暂停"),
    ("13_no_signal", "遥控失联：状态栏显示无信号"),
    ("14_control_fault", "控制故障：确认重试当前目的地"),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--modelsim", type=Path, default=Path("D:/apps/modelsim/win64"))
    parser.add_argument("--out", type=Path, default=ROOT / "logs/nav_ui_preview")
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    library = output / "preview_work"
    sources = [
        "rtl/common/font_rom.v", "rtl/common/font_rom_lg.v", "rtl/common/font_num.v",
        "rtl/common/ui_text_rom.v", "rtl/ui/nav_map_view.v", "rtl/ui/ui_render.v",
        "sim/nav_ui_preview_tb.v",
    ]

    def run(arguments, cwd=ROOT):
        result = subprocess.run([str(a).replace("\\", "/") for a in arguments],
                                cwd=cwd, capture_output=True, text=True, errors="replace")
        with (output / "render.log").open("a", encoding="utf-8") as log:
            log.write(result.stdout + result.stderr)
        if result.returncode or "** Error" in result.stdout or "FAIL:" in result.stdout:
            raise RuntimeError((result.stdout + result.stderr)[-4000:])
        return result.stdout

    (output / "render.log").write_text("", encoding="utf-8")
    if not library.exists():
        run([args.modelsim / "vlib.exe", library])
    run([args.modelsim / "vlog.exe", "-work", library, "+incdir+" + str(ROOT / "rtl/ui"),
         *[ROOT / p for p in sources]])
    run([args.modelsim / "vmap.exe", "-c"], cwd=output)
    ini = output / "modelsim.ini"
    run([args.modelsim / "vmap.exe", "-modelsimini", ini, "preview_work", library])
    result = run([args.modelsim / "vsim.exe", "-c", "-modelsimini", ini,
                  "preview_work.nav_ui_preview_tb", "+OUT=" + str(output),
                  "-do", "run -all; quit -f"])
    if "PASS: nav_ui_preview_tb 15 frames" not in result:
        raise RuntimeError("Preview did not finish; see render.log")
    title_font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 22)
    caption_font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 18)
    sheet = Image.new("RGB", (1328, ((len(STATES) + 1) // 2) * 432 + 106), "#151b26")
    draw = ImageDraw.Draw(sheet)
    draw.text((24, 16), "自动导航 UI · 当前 RTL 渲染预览", font=title_font, fill="#edf3ff")
    draw.text((24, 53), "示例地图与状态输入；原图 1024×600。橙色：原始障碍  蓝色：路线  绿色：车辆  白色：目的地 / 准心",
              font=caption_font, fill="#acb9cc")
    for i, (name, title) in enumerate(STATES):
        with Image.open(output / f"{name}.ppm") as image:
            image.save(output / f"{name}.png")
            thumb = image.resize((640, 375), Image.Resampling.NEAREST)
        left = 16 + (i % 2) * 656
        top = 106 + (i // 2) * 432
        draw.text((left + 4, top), f"{i + 1:02d}  {title}", font=caption_font, fill="#edf3ff")
        sheet.paste(thumb, (left, top + 32))
    sheet.save(output / "overview.png")
    for group_name, indices in [("selection", range(8)), ("states", range(8, len(STATES)))]:
        tiles = list(indices)
        group = Image.new("RGB", (1328, ((len(tiles) + 1) // 2) * 432 + 60), "#151b26")
        group_draw = ImageDraw.Draw(group)
        group_draw.text((24, 14), "当前 RTL 渲染 · " + ("建图与选点" if group_name == "selection" else "运行状态"),
                        font=title_font, fill="#edf3ff")
        for j, i in enumerate(tiles):
            name, title = STATES[i]
            left = 16 + (j % 2) * 656
            top = 60 + (j // 2) * 432
            group_draw.text((left + 4, top), f"{i + 1:02d}  {title}", font=caption_font, fill="#edf3ff")
            with Image.open(output / f"{name}.png") as image:
                group.paste(image.resize((640, 375), Image.Resampling.NEAREST), (left, top + 32))
        group.save(output / f"{group_name}_overview.png")
    links = "\n".join(f"- [{title}]({name}.png)" for name, title in STATES)
    (output / "README.md").write_text("# 导航界面预览\n\n实际 ui_render + nav_map_view RTL 渲染；示例地图与状态快照，非实车截图或运行轨迹。\n\n" + links + "\n", encoding="utf-8")
    with zipfile.ZipFile(output / "navigation_previews.zip", "w", zipfile.ZIP_DEFLATED) as bundle:
        for file in [output / f"{name}.png" for name, _ in STATES] + [
                output / "overview.png", output / "selection_overview.png",
                output / "states_overview.png", output / "README.md"]:
            bundle.write(file, file.name)
    print(f"Rendered {len(STATES)} frames: {output / 'overview.png'}", flush=True)


if __name__ == "__main__":
    main()
