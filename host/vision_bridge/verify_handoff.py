"""Portable clean-checkout verification; no Git history, models or devices needed."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

from team_demo import run_demo

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def run(command, output, *, cwd=REPO):
    result = subprocess.run([str(x) for x in command], cwd=cwd, capture_output=True,
                            text=True, encoding="utf-8", errors="replace", timeout=180)
    content = result.stdout + result.stderr
    output.write_text(content, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"verification exited {result.returncode}: {output.name}\n{content[-2500:]}")
    return content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE / "build" / "verification")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {"status": "FAIL", "hardware_tested": False, "serial_bytes_sent": 0,
              "python_version": sys.version.split()[0], "rtl": []}
    try:
        python_log = run([sys.executable, "-m", "unittest", "discover", "-s", HERE / "tests", "-v"], output / "python_tests.txt")
        report["python_tests"] = "PASS"
        count = re.search(r"Ran (\d+) tests?", python_log)
        if not count:
            raise RuntimeError("test runner did not report a test count")
        report["python_test_count"] = int(count[1])
        demo = run_demo(output / "demo")
        report["demo"] = demo
        compiler, simulator = shutil.which("iverilog"), shutil.which("vvp")
        if not compiler or not simulator:
            raise RuntimeError("Icarus iverilog and vvp must be on PATH; RTL checks were not run")
        run([compiler, "-V"], output / "iverilog_version.txt")
        run([simulator, "-V"], output / "vvp_version.txt")
        common = [REPO / "rtl/ai/ai_vision_link.v",
                  REPO / "vendor/verilog-reusable-modules/modules/uart/rtl/uart_byte_rx.v"]
        tests = [("ai_vision_link_tb", REPO / "sim/ai_vision_link_tb.v", []),
                 ("vision_serial_bridge_tb", HERE / "sim/vision_serial_bridge_tb.v",
                  ["+HEX=demo/frames.hex", "+EXPECT=demo/expectations.txt",
                   "+FRAMES=" + str(demo["wire_frames"])])]
        for name, source, extra in tests:
            binary = output / (name + ".vvp")
            run([compiler, "-g2012", "-Wall", "-s", name, "-o", binary, source, *common],
                output / (name + "_compile.txt"))
            # Relative ASCII paths avoid native Windows Icarus mangling CJK
            # filenames in Verilog $fopen/$readmemh plusargs.
            log = run([simulator, binary.name, *extra], output / (name + "_simulation.txt"), cwd=output)
            if "PASS" not in log or re.search(r"(?im)^\s*(?:FAIL\b|ERROR\b|FATAL\b)", log):
                raise RuntimeError(f"missing PASS or error text from {name}")
            entry = {"name": name, "status": "PASS"}
            checks = re.search(r"CHECKS=(\d+) FAILURES=(\d+)", log)
            if checks:
                entry.update(checks=int(checks[1]), failures=int(checks[2]))
            report["rtl"].append(entry)
        files = [*HERE.glob("*.py"), *HERE.glob("tests/*.py"),
                 *HERE.glob("sim/*.v"), HERE / "examples/schema2_sample.jsonl",
                 REPO / "sim/ai_vision_link_tb.v", *common]
        report["source_sha256"] = {p.relative_to(REPO).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(files)}
        report["status"] = "PASS"
    except Exception as exc:
        report["error"] = str(exc)
    (output / "verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("status", "hardware_tested", "serial_bytes_sent", "rtl")}, ensure_ascii=False))
    if report["status"] != "PASS":
        print(report.get("error", "verification failed"), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
