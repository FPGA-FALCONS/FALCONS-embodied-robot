"""Replay actual schema-v2 predictions into a standalone RTL status mailbox.

This program invokes git/iverilog/vvp locally. It neither opens a serial port
nor downloads a bitstream. The production auto channel stays disconnected.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

from bridge import CLASSES, VisionBridge, parse_json_line

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SOURCE_FILES = ("rtl/top/falcons_top.v", "rtl/rc/rc_drive_guard.v", "rtl/motor/chassis_ctrl.v",
                "rtl/motor/speed_pid.v", "rtl/debug/uart_cmd_rx.v")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def command(args, cwd, timeout=60):
    return subprocess.run([str(x) for x in args], cwd=cwd, capture_output=True,
                          text=True, encoding="utf-8", errors="replace", timeout=timeout)


def pin_sources(ref, out):
    pinned = command(["git", "rev-parse", "--verify", ref + "^{commit}"], REPO)
    if pinned.returncode:
        raise RuntimeError(pinned.stderr)
    commit = pinned.stdout.strip()
    source_dir = out / "build" / "pinned_sources"
    source_dir.mkdir(parents=True, exist_ok=True)
    hashes, paths = {}, {}
    for name in SOURCE_FILES:
        result = subprocess.run(["git", "show", f"{commit}:{name}"], cwd=REPO, capture_output=True)
        if result.returncode:
            raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
        path = source_dir / Path(name).name
        path.write_bytes(result.stdout)
        hashes[name] = digest(result.stdout)
        paths[name] = path
    top = paths[SOURCE_FILES[0]].read_text(encoding="utf-8")
    # Refuse to silently apply the old contract after teammates wire a receiver.
    required = (r"\.i_auto_valid\s*\(1'b0\)", r"\.i_auto_center\s*\(32'sd0\)",
                r"\.i_auto_diff\s*\(32'sd0\)", r"\.o_tgt_valid\s*\(\s*\)",
                r"\.o_mode_valid\s*\(\s*\)", r"\.o_duty_valid\s*\(\s*\)")
    if not all(re.search(p, top) for p in required):
        raise RuntimeError("Production interface changed: review adapter and test harness before running")
    return {"source_ref": ref, "source_commit": commit, "sha256": hashes,
            "top_tof_estop_tied_zero": bool(re.search(r"wire\s+tof_estop\s*=\s*1'b0", top))}, paths


def create_vectors(input_path, out):
    raw = input_path.read_bytes()
    (out / "input_predictions.jsonl").write_bytes(raw)
    lines = raw.decode("utf-8-sig").splitlines()
    bridge, timeline, originals, accepted, now = VisionBridge(), [], [], [], 0.0

    def append(event, kind, emit=True):
        timeline.append({"kind": kind, "emit_mailbox_event": emit, "event": event})

    for line in lines:
        if not line.strip():
            continue
        try:
            row = parse_json_line(line)
        except (ValueError, TypeError):
            event = bridge.reject("invalid_json", now)
        else:
            originals.append(row)
            event = bridge.ingest(row, now)
            if event["target_valid"]:
                accepted.append(row)
        append(event, "model")
        now += 0.100
    if not originals:
        raise ValueError("No schema-v2 predictions were supplied")
    now += 0.301
    append(bridge.poll(now), "timer", False)
    if not accepted:
        raise ValueError("Need at least one accepted model result for derived fault injection")
    seed = deepcopy(accepted[-1])

    def fault(suffix, *, qwen_conflict=False, status=None, source_seq=None, age_s=None):
        nonlocal now
        now += 0.100
        row = deepcopy(seed)
        row["frame_id"] = "derived_fault_" + suffix
        if qwen_conflict:
            row["qwen"] = {"class": CLASSES[(CLASSES.index(row["fusion"]["target"]) + 1) % 4],
                           "elapsed_ms": 1, "error": None}
        if status:
            row["fusion"].update(status=status, action="reobserve", target_valid=False, target=None, reason=suffix)
        event = bridge.ingest(row, now, source_seq=source_seq,
                              captured_at_s=now - age_s if age_s is not None else None)
        append(event, "derived_fault")
        return row

    fault("accepted_seed")
    fault("qwen_conflict", qwen_conflict=True)
    fault("VERIFY_PENDING", status="VERIFY_PENDING")
    fault("TIMEOUT", status="TIMEOUT")
    row = fault("duplicate_seed")
    now += 0.100
    append(bridge.ingest(row, now), "derived_fault")
    fault("ordered_seed", source_seq=10, age_s=0.1)
    fault("late_reply", source_seq=9, age_s=0.1)
    fault("old_capture", source_seq=11, age_s=1.1)
    fault("short_lifetime", source_seq=12, age_s=0.95)
    now += 0.051
    append(bridge.poll(now), "timer", False)
    now += 0.100
    append(bridge.reject("invalid_json", now), "derived_fault")
    (out / "bridge_timeline.jsonl").write_text("".join(json.dumps(t, ensure_ascii=False, allow_nan=False) + "\n"
                                                       for t in timeline), encoding="utf-8")
    vector_lines, previous = [], 0.0
    for item in timeline:
        e = item["event"]
        current = e["received_monotonic_s"]
        gap_ms = round((current - previous) * 1000)
        previous = current
        v = int(e["target_valid"])
        c = e["class_id"] if v else 0
        ttl = max(0, math.floor(e["ttl_remaining_ms"] + 1e-6)) if v else 0
        vector_lines.append(f"{gap_ms} {int(item['emit_mailbox_event'])} {v} {c} {ttl} {v} {c}\n")
    vectors = out / "replay_vectors.txt"
    vectors.write_text("".join(vector_lines), encoding="ascii")
    return {"input": str(input_path.resolve()), "input_sha256": digest(raw),
            "model_records": sum(t["kind"] == "model" for t in timeline),
            "model_accepted_observations": sum(t["kind"] == "model" and t["event"]["target_valid"] for t in timeline),
            "derived_fault_records": sum(t["kind"] == "derived_fault" for t in timeline),
            "timer_records": sum(t["kind"] == "timer" for t in timeline),
            "total_replay_records": len(timeline), "vector_sha256": digest(vectors.read_bytes())}, vectors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=HERE / "evidence")
    parser.add_argument("--source-ref", default="origin/main")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    report = {"hardware_tested": False, "serial_bytes_sent": 0, "bitstream_downloaded": False,
              "motion_enabled_by_vision": False, "timing_mode": "accelerated logical milliseconds",
              "status": "FAIL"}
    try:
        source, pinned_paths = pin_sources(args.source_ref, out)
        report.update(source)
        replay, vectors = create_vectors(args.predictions, out)
        report.update(replay)
        unit = command([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"], HERE)
        (out / "python_tests.txt").write_text(unit.stdout + unit.stderr, encoding="utf-8")
        report["python_test_exit"] = unit.returncode
        unit_count = re.search(r"Ran (\d+) tests", unit.stdout + unit.stderr)
        report["python_tests"] = int(unit_count[1]) if unit_count else None
        if unit.returncode:
            raise RuntimeError("Python bridge tests failed")
        compiler, simulator = shutil.which("iverilog"), shutil.which("vvp")
        if not compiler or not simulator:
            raise RuntimeError("Icarus iverilog and vvp are required for RTL integration")
        version = command([compiler, "-V"], HERE)
        (out / "tool_versions.txt").write_text(version.stdout + version.stderr, encoding="utf-8")
        report["iverilog_executable"] = compiler
        report["vvp_executable"] = simulator
        binary = out / "build" / "vision_module_tb.vvp"
        sources = [HERE / "sim" / "vision_status_sink.v", HERE / "sim" / "vision_module_tb.v"]
        sources += [pinned_paths[name] for name in SOURCE_FILES if name != "rtl/top/falcons_top.v"]
        compile_command = [compiler, "-g2012", "-Wall", "-s", "vision_module_tb", "-o", binary, *sources]
        sim_command = [simulator, binary, "+VECTORS=" + str(vectors)]
        (out / "commands.json").write_text(json.dumps({"compile": list(map(str, compile_command)),
                                                       "simulate": list(map(str, sim_command))}, indent=2) + "\n", encoding="utf-8")
        compile_result = command(compile_command, HERE)
        (out / "iverilog_compile.txt").write_text(compile_result.stdout + compile_result.stderr, encoding="utf-8")
        report["compile_exit"] = compile_result.returncode
        report["compile_warnings"] = len(re.findall("warning:", compile_result.stdout + compile_result.stderr, re.I))
        if compile_result.returncode:
            raise RuntimeError("Icarus compilation failed")
        sim = command(sim_command, HERE)
        (out / "rtl_simulation.txt").write_text(sim.stdout + sim.stderr, encoding="utf-8")
        report["rtl_exit"] = sim.returncode
        counts = re.search(r"REPLAY_RECORDS=(\d+) CHECKS=(\d+) FAILURES=(\d+)", sim.stdout)
        if counts:
            report.update({"rtl_replay_records": int(counts[1]), "rtl_checks": int(counts[2]), "rtl_failures": int(counts[3])})
        if sim.returncode or not counts or int(counts[3]) != 0 or "PASS vision observation" not in sim.stdout:
            raise RuntimeError("RTL integration simulation failed")
        report["status"] = "PASS"
    except Exception as error:
        report["error"] = str(error)
    # Source identity for this host-side implementation, not just the old RTL.
    report["bridge_source_sha256"] = {str(p.relative_to(HERE)): digest(p.read_bytes())
                                      for p in (*HERE.glob("*.py"), *HERE.glob("sim/*.v"), *HERE.glob("tests/*.py"))}
    (out / "integration_summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
