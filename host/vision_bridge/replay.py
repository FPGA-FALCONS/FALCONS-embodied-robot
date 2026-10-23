"""Offline schema-v2 JSONL adapter. Requires only Python 3.10+ stdlib."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from bridge import BridgeConfig, replay_lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--period-ms", type=float, default=100.0,
                        help="synthetic replay interval; not a capture timestamp")
    parser.add_argument("--ttl-ms", type=float, default=300.0)
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error("input and output must differ")
    config = BridgeConfig(target_ttl_s=args.ttl_ms / 1000)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    counts = {"records": 0, "accepted_observations": 0, "invalid_observations": 0}
    with args.input.open(encoding="utf-8-sig") as source, args.output.open("w", encoding="utf-8") as target:
        for event in replay_lines(source, args.period_ms / 1000, config):
            target.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
            counts["records"] += 1
            counts["accepted_observations" if event["target_valid"] else "invalid_observations"] += 1
    counts.update({"output": str(args.output.resolve()), "serial_bytes_sent": 0, "hardware_tested": False})
    print(json.dumps(counts, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
