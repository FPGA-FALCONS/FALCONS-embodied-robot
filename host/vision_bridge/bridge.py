"""Schema-v2 perception adapter. No serial transport and no motor commands.

The vehicle has navigation, but this bridge has no production motion transport.
This module deliberately preserves that boundary. A target is an observation,
never a velocity or a permission to enable the chassis.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from typing import Any

CLASSES = ("tape", "screwdriver", "ad_calcium_milk", "nutri_express")
VALID_PAIRS = {("VERIFIED", "target_confirmed"), ("TRACKING", "accept")}
STATES = {"TRACKING", "VERIFY_PENDING", "VERIFIED", "UNCERTAIN", "TIMEOUT"}


def _number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def parse_json_line(line: str) -> dict:
    """Reject ambiguous duplicate keys and non-object records."""
    row = json.loads(line, object_pairs_hook=_unique_object)
    if not isinstance(row, dict):
        raise ValueError("record must be a JSON object")
    return row


@dataclass(frozen=True)
class BridgeConfig:
    target_ttl_s: float = 0.300
    max_source_age_s: float = 1.000
    min_confidence: float = 0.300
    duplicate_window: int = 256
    require_source_metadata: bool = False

    def __post_init__(self):
        if not all(_number(v) for v in (self.target_ttl_s, self.max_source_age_s, self.min_confidence)):
            raise ValueError("configuration must be finite numeric values")
        if min(self.target_ttl_s, self.max_source_age_s) <= 0 or not 0 <= self.min_confidence <= 1:
            raise ValueError("invalid timeout or confidence")
        if type(self.duplicate_window) is not int or self.duplicate_window < 1:
            raise ValueError("duplicate_window must be a positive integer")
        if type(self.require_source_metadata) is not bool:
            raise ValueError("require_source_metadata must be bool")


class VisionBridge:
    """One ordered producer per instance; caller supplies a monotonic clock.

    ingest(..., source_seq=N, captured_at_s=T) enables ordering and source-age
    checks for an in-process producer. T must share the caller's monotonic
    clock domain. JSONL POC outputs currently contain neither value: replay
    therefore reports source_freshness_checked=False. Receive-time TTL alone
    cannot detect an old JSONL file or timestamp skew across computers.
    """

    def __init__(self, config: BridgeConfig | None = None):
        self.config = config or BridgeConfig()
        self._target = None
        self._confidence = self._bbox = None
        self._deadline = None
        self._last_now = None
        self._last_source_seq = None
        self._seen = deque()
        self._seen_set = set()
        self._event_seq = 0
        self._reason = "no_observation"
        self._frame_id = None
        self._status = None
        self._action = None
        self._source_checked = False
        self._sequence_checked = False

    def _invalidate(self, reason):
        self._target, self._deadline, self._reason = None, None, reason
        self._confidence = self._bbox = None

    def _clock(self, now_s):
        if not _number(now_s) or now_s < 0:
            raise ValueError("now_s must be a finite nonnegative monotonic time")
        if self._last_now is not None and now_s < self._last_now:
            self._invalidate("clock_regression")
            return False
        self._last_now = now_s
        return True

    def _remember(self, frame_id):
        if len(self._seen) == self.config.duplicate_window:
            self._seen_set.remove(self._seen.popleft())
        self._seen.append(frame_id)
        self._seen_set.add(frame_id)

    def _snapshot(self, now_s, was_valid=False):
        remaining = max(0.0, self._deadline - now_s) if self._deadline is not None else 0.0
        valid = self._target is not None and remaining > 0
        return {
            "schema_version": 1,
            "type": "vision_observation",
            "bridge_seq": self._event_seq,
            "received_monotonic_s": now_s,
            "frame_id": self._frame_id,
            "source_status": self._status,
            "source_action": self._action,
            "source_freshness_checked": self._source_checked,
            "source_sequence_checked": self._sequence_checked,
            "target_valid": valid,
            "target_class": self._target if valid else None,
            "class_id": CLASSES.index(self._target) if valid else None,
            "confidence": self._confidence if valid else None,
            "bbox_xyxy": list(self._bbox) if valid else None,
            "ttl_remaining_ms": round(remaining * 1000, 6),
            "reason": self._reason,
            "revoked": bool(was_valid and not valid),
            # Bridge policy, not a claim about current production top-level wiring.
            "fpga_auto_valid": False,
            "fpga_auto_center": 0,
            "fpga_auto_diff": 0,
            "motion_authorized": False,
            "fpga_command_hex": "",
            "integration_mode": "observation_only_no_transport",
        }

    def poll(self, now_s: float) -> dict:
        """Timer tick: clear on expiry even when the producer is silent."""
        self._event_seq += 1
        was_valid = self._target is not None
        if self._clock(now_s) and self._deadline is not None and now_s >= self._deadline:
            self._invalidate("target_timeout")
        return self._snapshot(now_s, was_valid)

    def reject(self, reason: str, now_s: float) -> dict:
        """Malformed transport records immediately revoke the last target."""
        self._event_seq += 1
        was_valid = self._target is not None
        if self._clock(now_s):
            self._invalidate(reason)
        self._source_checked = self._sequence_checked = False
        return self._snapshot(now_s, was_valid)

    def ingest(self, row: dict, now_s: float, *, source_seq: int | None = None,
               captured_at_s: float | None = None) -> dict:
        self._event_seq += 1
        was_valid = self._target is not None
        self._source_checked = self._sequence_checked = False
        self._frame_id = self._status = self._action = None

        def fail(reason):
            self._invalidate(reason)
            return self._snapshot(now_s, was_valid)

        if not self._clock(now_s):
            return self._snapshot(now_s, was_valid)
        if not isinstance(row, dict) or type(row.get("schema_version")) is not int or row["schema_version"] != 2:
            return fail("unsupported_schema")
        self._frame_id = row.get("frame_id")
        if not isinstance(self._frame_id, str) or not self._frame_id.strip():
            return fail("invalid_frame_id")
        if self.config.require_source_metadata and (source_seq is None or captured_at_s is None):
            return fail("missing_source_metadata")
        if source_seq is not None:
            if type(source_seq) is not int or source_seq < 0:
                return fail("invalid_source_seq")
            self._sequence_checked = True
            if self._last_source_seq is not None and source_seq <= self._last_source_seq:
                return fail("out_of_order_or_duplicate")
            self._last_source_seq = source_seq
        elif self._frame_id in self._seen_set:
            # POC file names must be distinct. A video producer should supply seq.
            return fail("duplicate_frame_id")
        if self._frame_id not in self._seen_set:
            self._remember(self._frame_id)
        if captured_at_s is not None:
            self._source_checked = True
            if not _number(captured_at_s) or captured_at_s < 0 or captured_at_s > now_s:
                return fail("invalid_source_time")
            if now_s - captured_at_s >= self.config.max_source_age_s:
                return fail("stale_source")

        fusion = row.get("fusion")
        if not isinstance(fusion, dict):
            return fail("missing_fusion")
        self._status, self._action = fusion.get("status"), fusion.get("action")
        if not isinstance(self._status, str) or self._status not in STATES or not isinstance(self._action, str):
            return fail("invalid_status")
        if type(fusion.get("target_valid")) is not bool:
            return fail("invalid_target_valid")
        if not fusion["target_valid"]:
            return fail("upstream_" + str(fusion.get("reason", self._status)).lower())
        if (self._status, self._action) not in VALID_PAIRS:
            return fail("invalid_status_action")
        target = fusion.get("target")
        if not isinstance(target, str) or target not in CLASSES:
            return fail("unknown_target")
        yolo = row.get("yolo")
        if not isinstance(yolo, dict) or yolo.get("class") != target:
            return fail("yolo_target_conflict")
        confidence, box = yolo.get("confidence"), yolo.get("bbox_xyxy")
        if not _number(confidence) or not self.config.min_confidence <= confidence <= 1:
            return fail("invalid_or_low_confidence")
        if (not isinstance(box, (list, tuple)) or len(box) != 4 or not all(_number(v) for v in box)
                or not (box[2] > box[0] >= 0 and box[3] > box[1] >= 0)):
            return fail("invalid_bbox")
        qwen = row.get("qwen")
        if self._status == "VERIFIED" and (row.get("vlm_called") is not True or not isinstance(qwen, dict)):
            return fail("missing_verification")
        if qwen is not None:
            if not isinstance(qwen, dict) or qwen.get("class") != target or qwen.get("error"):
                return fail("qwen_conflict_or_error")
            if not _number(qwen.get("elapsed_ms")) or qwen["elapsed_ms"] < 0:
                return fail("invalid_qwen_latency")
        self._target = target
        self._confidence, self._bbox = confidence, tuple(box)
        self._deadline = now_s + self.config.target_ttl_s
        if captured_at_s is not None:
            self._deadline = min(self._deadline, captured_at_s + self.config.max_source_age_s)
        self._reason = "accepted_observation"
        return self._snapshot(now_s, was_valid)


def replay_lines(lines, frame_period_s=0.100, config=None):
    """Deterministic offline replay; synthetic time, never hardware freshness."""
    if not _number(frame_period_s) or frame_period_s <= 0:
        raise ValueError("frame_period_s must be positive")
    bridge = VisionBridge(config)
    now = 0.0
    for line in lines:
        if not line.strip():
            continue
        try:
            row = parse_json_line(line)
        except (ValueError, TypeError):
            event = bridge.reject("invalid_json", now)
        else:
            event = bridge.ingest(row, now)
        event["replay_only"] = True
        yield event
        now += frame_period_s
    final = bridge.poll(now + bridge.config.target_ttl_s)
    final["replay_only"] = True
    yield final
