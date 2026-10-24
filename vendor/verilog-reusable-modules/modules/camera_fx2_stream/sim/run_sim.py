"""Run the standalone camera/FX2 RTL checks using installed Icarus Verilog.

No FPGA or USB hardware is opened. Outputs include actual sampled FD bytes,
an expected complete-frame manifest, compiler output, and simulation output.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=Path(__file__).parent / "results")
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    rtl = base.parent / "rtl" / "camera_fx2_stream.v"
    iverilog = shutil.which("iverilog") or "C:/iverilog/bin/iverilog.exe"
    vvp = shutil.which("vvp") or "C:/iverilog/bin/vvp.exe"
    scenarios = [("small", 8, 6, 2), ("multi_packet", 32, 16, 1), ("exact_packet", 244, 1, 1)]
    results = []
    for name, width, height, ds in scenarios:
        out = (args.out / name).resolve()
        out.mkdir(parents=True, exist_ok=True)
        executable = out / "simulation.vvp"
        compile_command = [iverilog, "-g2012", "-Wall", "-s", "tb_camera_fx2_stream",
                           f"-Ptb_camera_fx2_stream.SOURCE_WIDTH={width}",
                           f"-Ptb_camera_fx2_stream.SOURCE_HEIGHT={height}",
                           f"-Ptb_camera_fx2_stream.DOWNSAMPLE={ds}",
                           "-o", str(executable), str(rtl), str(base / "tb_camera_fx2_stream.sv")]
        compiled = subprocess.run(compile_command, capture_output=True, text=True, timeout=60)
        (out / "compile.log").write_text(compiled.stdout + compiled.stderr, encoding="utf-8")
        (out / "commands.json").write_text(json.dumps({"compile": compile_command,
            "run": [vvp, str(executable), "+OUT_DIR=."]}, indent=2), encoding="utf-8")
        if compiled.returncode:
            raise RuntimeError(f"{name}: compile failed: {compiled.stdout}{compiled.stderr}")
        # Icarus Windows $fopen mishandles non-ASCII command-line strings.
        # A relative output path preserves evidence under any parent directory.
        ran = subprocess.run([vvp, str(executable), "+OUT_DIR=."], cwd=out,
                             capture_output=True, text=True, timeout=60)
        (out / "simulation.log").write_text(ran.stdout + ran.stderr, encoding="utf-8")
        print(name + ": " + ran.stdout.strip())
        if ran.returncode or "PASS completed=3" not in ran.stdout:
            raise RuntimeError(f"{name}: simulation failed: {ran.stdout}{ran.stderr}")
        raw = (out / "wire.bin").read_bytes()
        hex_bytes = bytes.fromhex((out / "wire.hex").read_text())
        if raw != hex_bytes:
            raise AssertionError(f"{name}: binary/hex evidence differ")
        frames = []
        for line in (out / "expected_frames.txt").read_text().splitlines():
            seq, tag, sw, sh, factor, ow, oh = map(int, line.split())
            frames.append(dict(sequence=seq, tag=tag, source_width=sw, source_height=sh,
                               downsample=factor, width=ow, height=oh))
        (out / "expected_frames.json").write_text(json.dumps(frames, indent=2), encoding="utf-8")
        results.append(dict(scenario=name, status="PASS", wire_bytes=len(raw), expected_frames=frames,
                            log=ran.stdout.strip()))
    (args.out.resolve() / "summary.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
