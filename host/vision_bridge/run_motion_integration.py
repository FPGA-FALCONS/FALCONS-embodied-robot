"""Replay real schema-v2 predictions through bridge -> visual-servo -> RTL chassis.

This is a simulation-only integration:
  predictions.jsonl -> VisionBridge -> MotionPlanner -> motion vectors
  -> vision_motion_sink -> real chassis_ctrl auto ports -> Icarus checks.

It does not open a serial port, modify falcons_top, enable the real vehicle, or
claim that the automatic channel is production-ready.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

from PIL import Image

from bridge import VisionBridge, parse_json_line
from motion import CLASSES, MotionConfig, propose, propose_zero

CLASS_IDS = {name: i for i, name in enumerate(CLASSES)}
HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def run(args, cwd, timeout=120):
    return subprocess.run([str(x) for x in args], cwd=cwd, capture_output=True,
                          text=True, encoding="utf-8", errors="replace", timeout=timeout)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--predictions", type=Path, required=True)
    ap.add_argument("--output", type=Path, default=HERE / "evidence" / "motion_link")
    ap.add_argument("--allow-motion", action="store_true",
                    help="emit simulation auto_valid=1; without it proposals are observation-only")
    args = ap.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)

    raw = args.predictions.read_bytes()
    rows = [line for line in raw.decode("utf-8-sig").splitlines() if line.strip()]
    bridge = VisionBridge()
    now = 0.0
    events = []
    vectors = []
    previous = 0.0
    accepted = 0
    motion_valid = 0
    stop_reasons = {}
    for line in rows:
        row = {}
        try:
            row = parse_json_line(line)
        except (ValueError, TypeError):
            event = bridge.reject("invalid_json", now)
        else:
            event = bridge.ingest(row, now)
        image_path = Path(row.get("image", "")) if isinstance(row, dict) else Path("")
        try:
            with Image.open(image_path) as im:
                size = im.size
        except Exception:
            size = (1280, 720)
        proposal = propose(event, size, MotionConfig(), allow_motion=args.allow_motion)
        gap_ms = round((now - previous) * 1000)
        previous = now
        if event.get("target_valid"):
            accepted += 1
        if proposal["fpga_auto_valid"]:
            motion_valid += 1
        stop_reasons[proposal["reason"]] = stop_reasons.get(proposal["reason"], 0) + 1
        rec = {"monotonic_s": round(now, 6), "event": event, "proposal": proposal,
               "image_size": list(size), "image": str(image_path)}
        events.append(rec)
        class_id = CLASS_IDS.get(proposal.get("target_class"), 0) if proposal["fpga_auto_valid"] else 0
        vectors.append(
            f"{gap_ms} 1 {1 if proposal['fpga_auto_valid'] else 0} {class_id} "
            f"{proposal['fpga_auto_center']} {proposal['fpga_auto_diff']} "
            f"{max(0, min(1000, int(event.get('ttl_remaining_ms') or 0)))}\n"
        )
        now += 0.100
    # EOF stop event.
    now += 0.301
    event = bridge.poll(now)
    proposal = propose_zero("eof_stop")
    events.append({"monotonic_s": round(now, 6), "event": event, "proposal": proposal,
                   "image_size": [0, 0], "image": ""})
    vectors.append("0 1 0 0 0 0 0\n")

    (out / "motion_events.jsonl").write_text(
        "".join(json.dumps(e, ensure_ascii=False) + "\n" for e in events), encoding="utf-8")
    (out / "motion_vectors.txt").write_text("".join(vectors), encoding="ascii")
    (out / "input_predictions.jsonl").write_bytes(raw)

    report = {
        "status": "PENDING",
        "hardware_tested": False,
        "serial_bytes_sent": 0,
        "bitstream_downloaded": False,
        "production_top_modified": False,
        "allow_motion_simulated": bool(args.allow_motion),
        "predictions": len(rows),
        "bridge_accepted": accepted,
        "motion_valid_frames": motion_valid,
        "stop_reasons": stop_reasons,
        "input_sha256": digest(raw),
        "vectors_sha256": digest((out / "motion_vectors.txt").read_bytes()),
    }

    if args.allow_motion:
        compiler, simulator = shutil.which("iverilog"), shutil.which("vvp")
        if not compiler or not simulator:
            report["error"] = "iverilog/vvp not found"
        else:
            sources = [
                REPO / "rtl/motor/chassis_ctrl.v",
                REPO / "rtl/motor/speed_pid.v",
                HERE / "sim/vision_motion_sink.v",
                HERE / "sim/vision_motion_tb.v",
            ]
            binary = out / "vision_motion_tb.vvp"
            comp = run([compiler, "-g2012", "-Wall", "-o", binary, *sources], HERE)
            (out / "compile.txt").write_text(comp.stdout + comp.stderr, encoding="utf-8")
            report["compile_exit"] = comp.returncode
            if comp.returncode:
                report["error"] = "compile failed"
            else:
                sim = run([simulator, str(binary), "+VECTORS=" + str(out / "motion_vectors.txt")], HERE)
                (out / "simulation.txt").write_text(sim.stdout + sim.stderr, encoding="utf-8")
                report["simulation_exit"] = sim.returncode
                report["rtl_pass"] = "PASS vision motion proposal" in sim.stdout
                if sim.returncode or not report["rtl_pass"]:
                    report["error"] = "simulation failed"
                else:
                    report["status"] = "PASS"
    else:
        report["status"] = "OBSERVATION_ONLY"

    (out / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                      encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] in {"PASS", "OBSERVATION_ONLY"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
