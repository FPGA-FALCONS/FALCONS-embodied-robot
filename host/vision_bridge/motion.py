"""Visual-servo motion proposal for the AI -> chassis simulation link.

This is deliberately *not* a production controller. It turns a validated
schema-v1 bridge observation into a slow visual-servo proposal:

    bbox center error -> differential command (turn toward target)
    bbox area         -> center command (approach slowly, stop when close)

`allow_motion=False` is the default: the proposal is computed for
visualization/tests, but `fpga_auto_valid` stays false. Even when enabled, the
output is a simulation vector only; no serial port is opened. The current
production top-level has navigation; this experimental planner is not wired in.

Sign convention matches chassis_ctrl:
    left  = center - diff
    right = center + diff
A target left of center gives a positive diff (right wheel faster) so the
robot turns left.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

CLASSES = ("tape", "screwdriver", "ad_calcium_milk", "nutri_express")


def _finite(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class MotionConfig:
    max_center: int = 512          # internal delta units (~100 mm/s)
    max_diff: int = 256            # internal delta units
    deadband: float = 0.08         # normalized x error at which turn stops
    turn_gain: float = 1.2         # diff units per normalized x error
    target_area: float = 0.10      # desired bbox area ratio
    min_area: float = 0.01         # below this, target is too far to approach
    stop_area: float = 0.30        # above this, stop to avoid pushing into it

    def __post_init__(self) -> None:
        if not (0 < self.deadband < 1 and 0 < self.turn_gain):
            raise ValueError("invalid deadband/turn gain")
        if not (0 <= self.min_area < self.target_area < self.stop_area <= 1):
            raise ValueError("require min_area < target_area < stop_area")
        if self.max_center <= 0 or self.max_diff <= 0:
            raise ValueError("max commands must be positive")


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def propose(
    event: dict,
    image_size: tuple[int, int],
    config: MotionConfig | None = None,
    *,
    allow_motion: bool = False,
    allowed_classes: Sequence[str] = CLASSES,
) -> dict:
    """Return a motion proposal for a bridge observation.

    The bridge event must contain `target_valid`, `target_class`, `bbox_xyxy`,
    and `ttl_remaining_ms`. `image_size` is (width, height) of the source frame.
    """
    cfg = config or MotionConfig()
    width, height = image_size
    if width <= 0 or height <= 0:
        raise ValueError("image_size must be positive")
    inside = bool(event.get("target_valid"))
    target = event.get("target_class")
    bbox = event.get("bbox_xyxy")
    reason = "ok"
    x_err = area = 0.0
    center = diff = 0

    if not inside or target not in allowed_classes:
        reason = "no_valid_target" if not inside else "class_not_allowed"
    elif not isinstance(bbox, (list, tuple)) or len(bbox) != 4 or not all(_finite(v) for v in bbox):
        reason = "invalid_bbox"
    else:
        x1, y1, x2, y2 = (float(v) for v in bbox)
        bw, bh = max(0.0, x2 - x1), max(0.0, y2 - y1)
        if bw < 2 or bh < 2:
            reason = "degenerate_bbox"
        else:
            x_err = _clamp(((x1 + x2) * 0.5 / width - 0.5) * 2.0, -1.0, 1.0)
            area = (bw * bh) / (width * height)
            if abs(x_err) > cfg.deadband:
                diff = int(round(_clamp(-cfg.turn_gain * x_err * cfg.max_diff,
                                         -cfg.max_diff, cfg.max_diff)))
            if area >= cfg.stop_area:
                center = 0
                reason = "target_too_close"
            elif area <= cfg.min_area:
                center = 0
                reason = "target_too_far"
            else:
                span = cfg.target_area - cfg.min_area
                center = int(round(cfg.max_center * _clamp(
                    (cfg.target_area - area) / span, 0.0, 1.0)))
                if center == 0:
                    reason = "at_approach_distance"

    # A zero command is a valid stop proposal; it just cannot be executed
    # unless motion is explicitly enabled in the simulation.
    auto_valid = bool(allow_motion and inside and target in allowed_classes and reason in {
        "ok", "at_approach_distance", "target_too_close", "target_too_far"
    })
    if not allow_motion:
        reason = "observation_only"
    return {
        "schema_version": 1,
        "type": "motion_proposal",
        "source_frame_id": event.get("frame_id"),
        "target_valid": inside,
        "target_class": target if inside else None,
        "bbox_area_ratio": round(area, 6),
        "x_error_norm": round(x_err, 6),
        "proposed_center": center,
        "proposed_diff": diff,
        "fpga_auto_valid": auto_valid,
        "fpga_auto_center": center if auto_valid else 0,
        "fpga_auto_diff": diff if auto_valid else 0,
        "motion_authorized": False,
        "simulation_only": True,
        "reason": reason,
        "ttl_remaining_ms": event.get("ttl_remaining_ms"),
        "left_target": (center - diff) if auto_valid else 0,
        "right_target": (center + diff) if auto_valid else 0,
    }


def propose_zero(reason: str = "stop") -> dict:
    """Explicit stop proposal, used by the integration loop on EOF/timeout."""
    return {
        "schema_version": 1,
        "type": "motion_proposal",
        "source_frame_id": None,
        "target_valid": False,
        "target_class": None,
        "bbox_area_ratio": 0.0,
        "x_error_norm": 0.0,
        "proposed_center": 0,
        "proposed_diff": 0,
        "fpga_auto_valid": False,
        "fpga_auto_center": 0,
        "fpga_auto_diff": 0,
        "motion_authorized": False,
        "simulation_only": True,
        "reason": reason,
        "ttl_remaining_ms": 0,
        "left_target": 0,
        "right_target": 0,
    }
