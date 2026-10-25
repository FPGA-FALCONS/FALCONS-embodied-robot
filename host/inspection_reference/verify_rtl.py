"""Icarus simulation against independent exact segment/closed-AABB geometry."""
from pathlib import Path
import hashlib
import json
import random
import re
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE / "tests"))
from test_planning import hits_closed_square


def oracle(a, b):
    return sorted((x, y) for x in range(min(a[0], b[0]), max(a[0], b[0])+1)
                  for y in range(min(a[1], b[1]), max(a[1], b[1])+1)
                  if hits_closed_square(a, b, (x, y)))


def vectors():
    rng = random.Random(261026)
    pairs = [((0, 0), (255, 255)), ((255, 255), (0, 0)),
             ((0, 255), (255, 0)), ((255, 0), (0, 255)),
             ((0, 0), (255, 1)), ((255, 0), (0, 1)),
             ((0, 0), (1, 255)), ((0, 255), (1, 0)),
             ((0, 128), (255, 128)), ((128, 255), (128, 0)),
             ((255, 255), (255, 255))]
    pairs += [(tuple(rng.randrange(12) for _ in range(2)),
               tuple(rng.randrange(12) for _ in range(2))) for _ in range(400)]
    cases = [(a, b, 0, (-1, -1), 2) for a, b in pairs]
    # Every touched cell independently blocks a diagonal and arbitrary slope,
    # including start, goal and corner side cells.
    for a, b in [((1, 1), (7, 7)), ((8, 2), (1, 6)), ((4, 9), (4, 1))]:
        cases += [(a, b, 1, c, 2) for c in oracle(a, b)]
    cases += [((1, 1), (7, 5), code, (-1, -1), state)
              for code in (2, 4) for state in (1, 2, 3)]
    cases += [(a, b, 3, (-1, -1), 2) for a, b in (
        ((256, 0), (1, 1)), ((0, 0), (65535, 1)),
        ((0, 256), (1, 1)), ((0, 0), (1, 256)))]
    lines = []
    for a, b, code, blocked, abort_state in cases:
        cells = [] if code == 3 else oracle(a, b)
        lines.append(" ".join(map(str, (*a, *b, len(cells), code, *blocked, abort_state))))
        lines.extend(str(y*256+x) for x, y in cells)
    return "\n".join(lines)+"\n", len(cases)


def main():
    output = HERE / "evidence"
    output.mkdir(exist_ok=True)
    rtl = REPO / "rtl/inspection/nav_supercover_check.v"
    tb = HERE / "rtl_tb/tb_supercover.sv"
    report = {"status": "FAIL", "scope": "standalone_rtl_simulation",
              "hardware_tested": False, "production_integrated": False,
              "synthesis_timing_verified": False, "seed": 261026}
    logs = []
    try:
        compiler, runner = shutil.which("iverilog"), shutil.which("vvp")
        if not compiler or not runner:
            raise RuntimeError("Icarus iverilog/vvp required; no packages are installed automatically")
        data, count = vectors()
        report.update(cases=count, vectors_sha256=hashlib.sha256(data.encode()).hexdigest())
        with tempfile.TemporaryDirectory(prefix="falcons_supercover_") as folder:
            root = Path(folder)
            (root/"vectors.txt").write_text(data, encoding="ascii")
            (root/rtl.name).write_bytes(rtl.read_bytes())
            (root/tb.name).write_bytes(tb.read_bytes())
            for cmd in ([compiler, "-g2012", "-Wall", "-s", "tb_supercover", "-o", "sim.vvp", rtl.name, tb.name],
                        [runner, "sim.vvp"]):
                completed = subprocess.run(cmd, cwd=root, capture_output=True, text=True,
                                           encoding="utf-8", errors="replace", timeout=120)
                logs.append(completed.stdout+completed.stderr)
                if completed.returncode:
                    raise RuntimeError("RTL compile/simulation failed; see rtl_tests.txt")
            match = re.search(r"PASS cases=(\d+) accepted_reads=(\d+)", logs[-1])
            if not match or int(match[1]) != count:
                raise RuntimeError("simulation completion count missing")
            report["accepted_reads"] = int(match[2])
        report.update(status="PASS", files_sha256={p.relative_to(REPO).as_posix():
                      hashlib.sha256(p.read_bytes()).hexdigest() for p in (rtl, tb, Path(__file__), HERE/"tests/test_planning.py")})
    except Exception as exc:
        report["error"] = str(exc)
    (output/"rtl_tests.txt").write_bytes("\n".join(logs).encode("utf-8"))
    (output/"rtl_verification.json").write_bytes((json.dumps(report, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
