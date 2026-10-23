"""Deterministic synthetic handoff replay; standard library only, no devices."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path

from bridge import BridgeConfig, VisionBridge, parse_json_line
from display_adapter import display_packet
from serial_export import encode_observation

HERE = Path(__file__).resolve().parent


def run_demo(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    source = HERE / "examples" / "schema2_sample.jsonl"
    rows = [parse_json_line(line) for line in source.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows or any(r.get("source_kind") != "synthetic_test_fixture" for r in rows):
        raise ValueError("demo requires explicitly synthetic fixtures")
    bridge = VisionBridge(BridgeConfig(require_source_metadata=True))
    events, packets = [], []
    now = 0.0
    for seq, row in enumerate(rows):
        now = seq / 10
        # Synthetic capture/receive times; these do NOT prove live freshness.
        event = bridge.ingest(row, now, source_seq=seq, captured_at_s=now)
        event["replay_only"] = True
        events.append(event)
        packets.append(display_packet(
            event, frame_id=row["frame_id"], image_size=(640, 480), now_s=now,
            semantic_text="示例文字：识别结果用于显示，尚未授权导航。",
            semantic_frame_id=row["frame_id"]))
    final = bridge.poll(now + 0.301)
    final["replay_only"] = True
    events.append(final)
    packets.append(display_packet(final, frame_id=rows[-1]["frame_id"],
                                  image_size=(640, 480), now_s=now + 0.301))
    for name, records in (("observations.jsonl", events), ("display.jsonl", packets)):
        (output / name).write_text("".join(json.dumps(r, ensure_ascii=False, allow_nan=False) + "\n"
                                          for r in records), encoding="utf-8")
    frames = b"".join(encode_observation(e, i) for i, e in enumerate(events))
    (output / "frames.hex").write_text("".join(f"{b:02x}\n" for b in frames), encoding="ascii")
    expectations = []
    for i in range(len(events)):
        f = frames[i * 19:(i + 1) * 19]
        expectations.append(f"{int.from_bytes(f[3:5], 'big')} {int(bool(f[5] & 1))} {f[6]} {f[7]} {f[8]}\n")
    (output / "expectations.txt").write_text("".join(expectations), encoding="ascii")
    cards = []
    for p in packets:
        boxes = []
        for b in p["boxes"]:
            x1, y1, x2, y2 = b["bbox_xyxy"]
            label = html.escape(f'{b["label"]} {b["confidence"]:.2f}')
            boxes.append(f'<rect x="{x1}" y="{y1}" width="{x2-x1}" height="{y2-y1}" fill="none" stroke="#22c55e" stroke-width="3"/>'
                         f'<text x="{x1}" y="{max(18, y1-5)}" fill="#22c55e">{label}</text>')
        cards.append('<article><h2>' + html.escape(p["frame_id"]) + '</h2>'
                     '<svg viewBox="0 0 640 480" role="img" aria-label="合成画面叠框示例">'
                     '<rect width="640" height="480" fill="#18212f"/>'
                     '<text x="220" y="260" fill="white">合成背景，非摄像头画面</text>'
                     + "".join(boxes) + '</svg><p>' + html.escape(p["status_text"])
                     + ' / ' + html.escape(p["reason"]) + '</p><p>'
                     + html.escape(p["semantic_text"]) + '</p></article>')
    (output / "preview.html").write_text(
        '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>AI 联调接口演示</title>'
        '<style>body{font:16px sans-serif;background:#f1f5f9;padding:24px}main{display:flex;flex-wrap:wrap;gap:16px}'
        'article{background:white;padding:16px;width:360px}svg{width:100%}h2{font-size:18px}</style>'
        '<h1>视觉观察 → 框与文字 → UART 文件</h1><p>固定合成样例；文字非 Qwen 实测输出；未连接摄像头、串口或电机。</p><main>'
        + "".join(cards) + '</main></html>', encoding="utf-8")
    files = ("observations.jsonl", "display.jsonl", "frames.hex", "expectations.txt", "preview.html")
    summary = {"status": "PASS", "synthetic_fixture": True, "hardware_tested": False,
               "camera_opened": False, "serial_bytes_sent": 0, "motion_authorized": False,
               "observations": len(events), "valid_targets": sum(e["target_valid"] for e in events),
               "wire_frames": len(events), "fixture_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
               "files_sha256": {f: hashlib.sha256((output / f).read_bytes()).hexdigest() for f in files}}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE / "build" / "team_demo")
    args = parser.parse_args()
    print(json.dumps(run_demo(args.output), ensure_ascii=False, indent=2))
