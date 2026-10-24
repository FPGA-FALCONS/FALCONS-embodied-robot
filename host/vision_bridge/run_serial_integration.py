"""Safe host bridge -> 19-byte frames -> pinned candidate UART RTL receiver."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from run_integration import HERE, REPO, command, digest
from serial_export import export_records

CANDIDATE_FILES = ("rtl/ai/ai_vision_link.v", "vendor/verilog-reusable-modules/modules/uart/rtl/uart_byte_rx.v")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--parser-ref", default="6972cebb813f4b598e24b3fd2d353f76937bc8a4")
    parser.add_argument("--output", type=Path, default=HERE / "evidence" / "candidate_uart")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    report = {"status": "FAIL", "hardware_tested": False, "serial_bytes_sent": 0,
              "candidate_feature_branch_only": True, "production_top_modified": False,
              "consumer_requires_target_valid_and_link_alive": True,
              "candidate_parser_sequence_replay_protection": False}
    try:
        ref = command(["git", "rev-parse", "--verify", args.parser_ref + "^{commit}"], REPO)
        if ref.returncode:
            raise RuntimeError(ref.stderr)
        commit = ref.stdout.strip()
        report["candidate_parser_commit"] = commit
        build = out / "build"
        build.mkdir(exist_ok=True)
        paths, hashes = [], {}
        for filename in CANDIDATE_FILES:
            result = subprocess.run(["git", "show", f"{commit}:{filename}"], cwd=REPO, capture_output=True)
            if result.returncode:
                raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
            path = build / Path(filename).name
            path.write_bytes(result.stdout)
            paths.append(path)
            hashes[filename] = digest(result.stdout)
        report["candidate_source_sha256"] = hashes
        raw = args.predictions.read_bytes()
        (out / "input_predictions.jsonl").write_bytes(raw)
        report["input_sha256"] = digest(raw)
        pairs = export_records(raw.decode("utf-8-sig").splitlines())
        stream = b"".join(frame for event, frame in pairs)
        hex_path, expect = out / "safe_frames.hex", out / "expectations.txt"
        hex_path.write_text("".join(f"{b:02x}\n" for b in stream), encoding="ascii")
        expect.write_text("".join(f"{int.from_bytes(f[3:5], 'big')} {int(bool(f[5]&1))} {f[6]} {f[7]} {f[8]}\n"
                                   for event, f in pairs), encoding="ascii")
        (out / "safe_bridge_events.jsonl").write_text("".join(json.dumps(e, ensure_ascii=False) + "\n"
                                                             for e, f in pairs), encoding="utf-8")
        report.update({"model_observation_records": len(pairs)-1, "encoded_frames_with_eof_revoke": len(pairs),
                       "encoded_valid_targets": sum(bool(f[5]&1) for e, f in pairs),
                       "wire_bytes_simulated": len(stream), "safe_frames_sha256": digest(stream)})
        compiler, simulator = shutil.which("iverilog"), shutil.which("vvp")
        if not compiler or not simulator:
            raise RuntimeError("Icarus iverilog and vvp are required")
        binary = build / "vision_serial_bridge_tb.vvp"
        compile_command = [compiler, "-g2012", "-Wall", "-s", "vision_serial_bridge_tb", "-o", binary,
                           HERE / "sim" / "vision_serial_bridge_tb.v", *paths]
        run_command = [simulator, binary, "+HEX="+str(hex_path), "+EXPECT="+str(expect), "+FRAMES="+str(len(pairs))]
        (out / "commands.json").write_text(json.dumps({"compile": list(map(str, compile_command)),
                                                       "simulate": list(map(str, run_command))}, indent=2), encoding="utf-8")
        compiled = command(compile_command, HERE)
        (out / "compile.txt").write_text(compiled.stdout+compiled.stderr, encoding="utf-8")
        report["compile_exit"] = compiled.returncode
        if compiled.returncode:
            raise RuntimeError("candidate UART integration compilation failed")
        simulation = command(run_command, HERE, timeout=180)
        (out / "simulation.txt").write_text(simulation.stdout+simulation.stderr, encoding="utf-8")
        report["simulation_exit"] = simulation.returncode
        match = re.search(r"SERIAL_MODEL_RECORDS=(\d+) SERIAL_TARGETS=(\d+) CHECKS=(\d+) FAILURES=(\d+)", simulation.stdout)
        if match:
            report.update({"rtl_frames": int(match[1]), "rtl_targets": int(match[2]),
                           "rtl_checks": int(match[3]), "rtl_failures": int(match[4])})
        stale = re.search(r"RAW_TARGET_AFTER_TIMEOUT=(\d+) EFFECTIVE_TARGET_AFTER_TIMEOUT=(\d+)", simulation.stdout)
        if stale:
            report.update({"candidate_raw_target_after_timeout": int(stale[1]), "effective_target_after_timeout": int(stale[2])})
        if simulation.returncode or not match or int(match[4]) or "PASS safe bridge" not in simulation.stdout:
            raise RuntimeError("candidate UART integration simulation failed")
        report["status"] = "PASS"
    except Exception as error:
        report["error"] = str(error)
    report["bridge_source_sha256"] = {str(p.relative_to(HERE)): digest(p.read_bytes())
                                      for p in (*HERE.glob("*.py"), *HERE.glob("sim/*.v"), *HERE.glob("tests/*.py"))}
    (out / "serial_integration_summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
