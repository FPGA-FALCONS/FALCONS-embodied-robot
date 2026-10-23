"""Safe bridge observations -> candidate ai_vision_link 19-byte frames.

Offline export only. No serial port is opened. The matching standalone receiver
is rtl/ai/ai_vision_link.v; it is not wired into the production falcons_top.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from bridge import CLASSES, VALID_PAIRS, replay_lines

STATUS = {"TRACKING": 0, "VERIFY_PENDING": 1, "VERIFIED": 2, "UNCERTAIN": 3, "TIMEOUT": 4}
MIN_CONF_Q8 = 77


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_observation(event: dict, sequence: int) -> bytes:
    """Encode only normalized bridge fields; quantization can only revoke.

    Sequence is an unbounded local counter, wrapped to the wire's uint16.
    It is distinct from producer source_seq. Do not invent transport freshness
    from this counter when replaying an old file.
    """
    if type(sequence) is not int or sequence < 0:
        raise ValueError("sequence must be a nonnegative integer")
    status = event.get("source_status")
    status_id = STATUS.get(status, STATUS["UNCERTAIN"]) if isinstance(status, str) else STATUS["UNCERTAIN"]
    action = event.get("source_action")
    valid = (event.get("target_valid") is True and isinstance(status, str) and isinstance(action, str)
             and (status, action) in VALID_PAIRS)
    cls, conf, box = event.get("class_id"), event.get("confidence"), event.get("bbox_xyxy")
    valid = valid and type(cls) is int and 0 <= cls < len(CLASSES) and event.get("target_class") == CLASSES[cls]
    valid = valid and type(conf) in (float, int) and math.isfinite(conf) and 0 <= conf <= 1
    valid = valid and isinstance(box, (list, tuple)) and len(box) == 4
    valid = valid and all(type(v) in (int, float) and math.isfinite(v) for v in box)
    valid = valid and box[2] > box[0] >= 0 and box[3] > box[1] >= 0
    conf_q8, quantized = 0, [0] * 4
    if valid:
        conf_q8 = round(conf * 255)
        quantized = [max(0, min(65535, round(value))) for value in box]
        ttl = event.get("ttl_remaining_ms")
        valid = (conf_q8 >= MIN_CONF_Q8 and quantized[2] > quantized[0]
                 and quantized[3] > quantized[1] and type(ttl) in (int, float)
                 and math.isfinite(ttl) and ttl > 0)
    if not valid:
        cls, conf_q8, quantized = 255, 0, [0] * 4
    # Invalid observations still carry an explicit revoke + heartbeat.
    flags = 1 if valid else 2
    payload = ((sequence & 0xFFFF).to_bytes(2, "big") + bytes([flags, cls, status_id, conf_q8])
               + b"".join(v.to_bytes(2, "big") for v in quantized))
    content = bytes([14]) + payload
    return b"\xAA\x55" + content + crc16_ccitt(content).to_bytes(2, "big")


def export_records(lines, *, include_eof_revoke=True):
    events = list(replay_lines(lines))
    if not include_eof_revoke:
        events = events[:-1]
    return [(event, encode_observation(event, i)) for i, event in enumerate(events)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--hex-out", type=Path, required=True)
    parser.add_argument("--bin-out", type=Path)
    args = parser.parse_args()
    paths = [args.input.resolve(), args.hex_out.resolve()]
    if args.bin_out:
        paths.append(args.bin_out.resolve())
    if len(set(paths)) != len(paths):
        parser.error("input/output paths must be distinct")
    with args.input.open(encoding="utf-8-sig") as stream:
        pairs = export_records(stream)
    data = b"".join(frame for event, frame in pairs)
    args.hex_out.parent.mkdir(parents=True, exist_ok=True)
    args.hex_out.write_text("".join(f"{b:02x}\n" for b in data), encoding="ascii")
    if args.bin_out:
        args.bin_out.parent.mkdir(parents=True, exist_ok=True)
        args.bin_out.write_bytes(data)
    print(json.dumps({"frames_including_eof_revoke": len(pairs), "bytes": len(data),
                      "wire_target_valid": sum(bool(frame[5] & 1) for event, frame in pairs),
                      "serial_bytes_sent": 0, "hardware_tested": False}, indent=2))


if __name__ == "__main__":
    main()
