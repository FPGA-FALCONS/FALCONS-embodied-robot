"""Framework-independent display packets. No camera, model or motor I/O."""
from __future__ import annotations

import math
from bridge import CLASSES

LABELS = dict(zip(CLASSES, ("胶带", "螺丝刀", "AD钙奶", "营养快线")))
STATUS_TEXT = {
    "TRACKING": "检测中（未语义复核）", "VERIFY_PENDING": "等待复核",
    "VERIFIED": "已复核", "UNCERTAIN": "识别不确定", "TIMEOUT": "复核超时",
}


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def display_packet(event, *, frame_id, image_size, now_s,
                   semantic_text=None, semantic_frame_id=None):
    """Join a bridge event to the exact displayed frame, in original pixels.

    now_s and event.received_monotonic_s must share one monotonic clock.
    image_size is (width, height), AFTER undoing model letterbox/resize.
    The caller must preserve the camera/stream identity in unique frame_id.
    Late/mismatched/expired observations produce no overlay. Semantic text is
    untrusted display-only text, never a task or motion authorization.
    """
    if (not isinstance(frame_id, str) or not frame_id.strip()
            or not isinstance(image_size, (list, tuple)) or len(image_size) != 2
            or any(type(v) is not int or v <= 0 for v in image_size)
            or not finite(now_s) or now_s < 0):
        raise ValueError("invalid frame identity, image size or monotonic time")
    if not isinstance(event, dict):
        raise ValueError("event must be a normalized bridge object")
    width, height = image_size
    same_frame = event.get("frame_id") == frame_id
    received, ttl = event.get("received_monotonic_s"), event.get("ttl_remaining_ms")
    fresh = (finite(received) and finite(ttl) and ttl > 0
             and 0 <= now_s - received < ttl / 1000)
    target = event.get("target_class")
    box, confidence = event.get("bbox_xyxy"), event.get("confidence")
    valid_box = (isinstance(box, (list, tuple)) and len(box) == 4
                 and all(finite(v) for v in box)
                 and 0 <= box[0] < box[2] <= width
                 and 0 <= box[1] < box[3] <= height)
    accepted = (event.get("target_valid") is True and same_frame and fresh
                and isinstance(target, str) and target in LABELS
                and event.get("source_status") in ("TRACKING", "VERIFIED")
                and type(event.get("class_id")) is int
                and event["class_id"] == CLASSES.index(target)
                and finite(confidence) and 0 <= confidence <= 1 and valid_box)
    reason = ("frame_mismatch" if not same_frame else "expired_or_invalid_time" if not fresh
              else "accepted" if accepted else "no_valid_target")
    # Renderer must use text APIs (e.g. Qt setPlainText), never execute HTML
    # or commands from a language-model response.
    text = (semantic_text[:500] if same_frame and fresh and semantic_frame_id == frame_id
            and isinstance(semantic_text, str) else "")
    status = event.get("source_status")
    return {
        "schema_version": 1, "type": "vision_display", "frame_id": frame_id,
        "image_size": [width, height], "coordinate_space": "original_image_pixels",
        "status_text": STATUS_TEXT.get(status, "无有效目标") if isinstance(status, str) else "无有效目标",
        "overlay_valid": bool(accepted), "reason": reason,
        "boxes": [{"bbox_xyxy": list(box), "class": target,
                   "label": LABELS[target], "confidence": confidence}] if accepted else [],
        "semantic_text": text, "semantic_text_is_command": False,
        "motion_authorized": False,
    }
