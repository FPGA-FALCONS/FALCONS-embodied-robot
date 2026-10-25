"""One offline entry point: tests, fixed-map comparison, reproducible manifest."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def main():
    output = HERE / "evidence"
    output.mkdir(exist_ok=True)
    report = {"status": "FAIL", "scope": "offline_planning_reference_only",
              "hardware_tested": False, "motion_commands_sent": 0,
              "production_rtl_changed": False, "python_version": sys.version.split()[0]}
    try:
        tests = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", str(HERE / "tests"), "-v"],
                               cwd=REPO, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120)
        test_log = tests.stdout + tests.stderr
        (output / "python_tests.txt").write_bytes(test_log.encode("utf-8"))
        if tests.returncode:
            raise RuntimeError("planning tests failed; see python_tests.txt")
        count = re.search(r"Ran (\d+) tests?", test_log)
        if not count:
            raise RuntimeError("test count missing")
        report["tests_run"] = int(count[1])
        comparison = subprocess.run([sys.executable, str(HERE / "benchmark.py")], cwd=REPO,
                                    capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120)
        if comparison.returncode:
            raise RuntimeError("map comparison failed: " + comparison.stderr[-1000:])
        benchmark = json.loads((output / "benchmark.json").read_text(encoding="utf-8"))
        if benchmark["status"] != "PASS" or benchmark["motion_authorized"] is not False:
            raise RuntimeError("benchmark failed or unexpectedly authorized movement")
        sample = subprocess.run([sys.executable, str(HERE / "route_cli.py"),
                                 str(HERE / "examples/costmap.json"), "--return-home",
                                 "--output", str(output / "example_return.json")], cwd=REPO,
                                capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=30)
        if sample.returncode:
            raise RuntimeError("snapshot example failed: " + sample.stderr[-1000:])
        files = [*HERE.glob("*.py"), *HERE.glob("tests/*.py"), *HERE.glob("examples/*.json"),
                 HERE / "README.md", output / "benchmark.json", output / "example_return.json"]
        report.update(status="PASS", maps=benchmark["maps"], common_solved=benchmark["common_solved"],
                      theta_vs_astar8=benchmark["theta_vs_astar8"],
                      files_sha256={p.relative_to(REPO).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in sorted(files)})
    except Exception as exc:
        report["error"] = str(exc)
    (output / "verification.json").write_bytes((json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(json.dumps({k: report[k] for k in ("status", "hardware_tested", "motion_commands_sent")}, ensure_ascii=False))
    if "error" in report:
        print(report["error"], file=sys.stderr)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
