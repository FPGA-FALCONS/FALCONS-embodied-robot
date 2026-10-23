from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import BridgeConfig, CLASSES, VisionBridge, parse_json_line, replay_lines


def observation(frame="f000", target="nutri_express", status="VERIFIED", action="target_confirmed"):
    return {"schema_version": 2, "frame_id": frame,
            "yolo": {"class": target, "confidence": 0.92, "bbox_xyxy": [20, 30, 120, 230]},
            "qwen": {"class": target, "elapsed_ms": 220.0, "error": None},
            "vlm_called": True,
            "fusion": {"status": status, "action": action, "target": target,
                       "target_valid": True, "reason": "agreement"}}


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = VisionBridge()

    def assert_safe(self, event):
        self.assertFalse(event["target_valid"], event)
        self.assertIsNone(event["target_class"])
        self.assertIsNone(event["class_id"])
        self.assertFalse(event["motion_authorized"])
        self.assertFalse(event["fpga_auto_valid"])
        self.assertEqual(event["fpga_auto_center"], 0)
        self.assertEqual(event["fpga_auto_diff"], 0)
        self.assertEqual(event["fpga_command_hex"], "")

    def test_reset_has_no_target(self):
        self.assert_safe(self.bridge.poll(0))

    def test_verified_all_four_class_ids(self):
        for i, name in enumerate(CLASSES):
            event = self.bridge.ingest(observation(f"f{i}", name), i)
            self.assertTrue(event["target_valid"])
            self.assertEqual(event["class_id"], i)
            self.assertFalse(event["motion_authorized"])

    def test_reviewed_tracking_accept(self):
        row = observation(status="TRACKING", action="accept")
        row.update(qwen=None, vlm_called=False)
        row["fusion"]["reason"] = "recent_review_stable_track"
        self.assertTrue(self.bridge.ingest(row, 0)["target_valid"])

    def test_false_target_flag_revokes_in_every_state(self):
        for state in ("TRACKING", "VERIFY_PENDING", "VERIFIED", "UNCERTAIN", "TIMEOUT"):
            with self.subTest(state=state):
                bridge = VisionBridge()
                bridge.ingest(observation(), 0)
                row = observation("next", status=state)
                row["fusion"].update(target_valid=False, target=None, candidate="nutri_express", reason=state)
                event = bridge.ingest(row, 0.01)
                self.assert_safe(event)
                self.assertTrue(event["revoked"])

    def test_invalid_status_action_cannot_accept(self):
        for state, action in (("TIMEOUT", "accept"), ("UNCERTAIN", "target_confirmed"),
                              ("VERIFY_PENDING", "target_confirmed"), ("VERIFIED", "continue_tracking")):
            with self.subTest(state=state, action=action):
                self.assert_safe(VisionBridge().ingest(observation(status=state, action=action), 0))

    def test_qwen_conflict_revokes(self):
        self.bridge.ingest(observation(), 0)
        row = observation("f001")
        row["qwen"]["class"] = "ad_calcium_milk"
        event = self.bridge.ingest(row, 0.01)
        self.assert_safe(event)
        self.assertEqual(event["reason"], "qwen_conflict_or_error")
        self.assertTrue(event["revoked"])

    def test_timeout_exact_boundary(self):
        self.bridge.ingest(observation(), 10.0)
        self.assertTrue(self.bridge.poll(10.299)["target_valid"])
        event = self.bridge.poll(10.3)
        self.assert_safe(event)
        self.assertTrue(event["revoked"])
        self.assertEqual(event["reason"], "target_timeout")

    def test_poll_does_not_refresh_ttl(self):
        self.bridge.ingest(observation(), 0)
        self.bridge.poll(0.1)
        self.bridge.poll(0.2)
        self.assert_safe(self.bridge.poll(0.31))

    def test_unknown_schema_revokes(self):
        for version in (1, 3, True, "2", None):
            bridge = VisionBridge()
            bridge.ingest(observation(), 0)
            row = observation("new")
            row["schema_version"] = version
            self.assert_safe(bridge.ingest(row, 0.01))

    def test_non_boolean_flag_rejected(self):
        for value in (1, "true", [], None):
            row = observation()
            row["fusion"]["target_valid"] = value
            self.assert_safe(VisionBridge().ingest(row, 0))

    def test_invalid_labels_and_boxes_rejected(self):
        for target in ("unknown", "motor_forward", None, ["tape"]):
            row = observation()
            row["fusion"]["target"] = target
            self.assert_safe(VisionBridge().ingest(row, 0))
        for box in ([0, 0, 0, 20], [-1, 0, 10, 20], [0, 20, 10, 10],
                    [0, 0, float("nan"), 20], [0, 0, True, 20], [1, 2], None):
            row = observation()
            row["yolo"]["bbox_xyxy"] = box
            self.assert_safe(VisionBridge().ingest(row, 0))

    def test_invalid_or_low_confidence_rejected(self):
        for confidence in (-1, 0.29, 1.1, float("nan"), float("inf"), True, "0.9", None):
            row = observation()
            row["yolo"]["confidence"] = confidence
            self.assert_safe(VisionBridge().ingest(row, 0))

    def test_verified_requires_verifier(self):
        for patch in ({"vlm_called": False}, {"qwen": None}, {"qwen": {"class": "nutri_express"}},
                      {"qwen": {"class": "nutri_express", "error": "timeout", "elapsed_ms": 1}},
                      {"qwen": {"class": "nutri_express", "elapsed_ms": -1}}):
            row = observation()
            row.update(patch)
            self.assert_safe(VisionBridge().ingest(row, 0))

    def test_yolo_target_must_match(self):
        row = observation()
        row["yolo"]["class"] = "tape"
        self.assert_safe(self.bridge.ingest(row, 0))

    def test_duplicate_record_revokes_and_does_not_extend(self):
        row = observation()
        self.bridge.ingest(row, 0)
        event = self.bridge.ingest(row, 0.25)
        self.assert_safe(event)
        self.assertEqual(event["reason"], "duplicate_frame_id")
        self.assert_safe(self.bridge.poll(0.31))

    def test_source_sequence_and_old_replies(self):
        self.bridge.ingest(observation(), 2, source_seq=10, captured_at_s=1.9)
        event = self.bridge.ingest(observation("f001"), 2.1, source_seq=9, captured_at_s=2.0)
        self.assert_safe(event)
        self.assertEqual(event["reason"], "out_of_order_or_duplicate")
        self.assertTrue(self.bridge.ingest(observation("f002"), 2.2, source_seq=11, captured_at_s=2.1)["target_valid"])

    def test_video_can_reuse_frame_name_if_sequence_increases(self):
        row = observation()
        self.bridge.ingest(row, 0, source_seq=1)
        self.assertTrue(self.bridge.ingest(row, 0.1, source_seq=2)["target_valid"])

    def test_stale_source_rejected(self):
        event = self.bridge.ingest(observation(), 2, source_seq=1, captured_at_s=1)
        self.assert_safe(event)
        self.assertEqual(event["reason"], "stale_source")

    def test_future_or_invalid_source_clock_rejected(self):
        for value in (3, -1, float("nan"), True):
            self.assert_safe(VisionBridge().ingest(observation(), 2, captured_at_s=value))

    def test_source_age_bounds_target_deadline(self):
        self.assertTrue(self.bridge.ingest(observation(), 2, captured_at_s=1.1)["target_valid"])
        self.assert_safe(self.bridge.poll(2.101))

    def test_require_source_metadata_mode(self):
        bridge = VisionBridge(BridgeConfig(require_source_metadata=True))
        self.assert_safe(bridge.ingest(observation(), 0))
        self.assertTrue(bridge.ingest(observation("new"), 0.1, source_seq=1, captured_at_s=0)["target_valid"])

    def test_replay_does_not_claim_capture_freshness(self):
        event = self.bridge.ingest(observation(), 0)
        self.assertFalse(event["source_freshness_checked"])
        self.assertFalse(event["source_sequence_checked"])

    def test_clock_regression_revokes(self):
        self.bridge.ingest(observation(), 2)
        event = self.bridge.poll(1)
        self.assert_safe(event)
        self.assertEqual(event["reason"], "clock_regression")

    def test_invalid_clock_rejected_by_api(self):
        for now in (float("nan"), float("inf"), -1, True):
            with self.assertRaises(ValueError):
                VisionBridge().poll(now)

    def test_malformed_json_revokes_and_eof_expires(self):
        lines = [json.dumps(observation()), "{not json}", json.dumps(observation("new"))]
        events = list(replay_lines(lines))
        self.assertTrue(events[0]["target_valid"])
        self.assert_safe(events[1])
        self.assertTrue(events[1]["revoked"])
        self.assertTrue(events[2]["target_valid"])
        self.assert_safe(events[3])
        self.assertTrue(all(e["replay_only"] for e in events))

    def test_duplicate_json_keys_rejected(self):
        with self.assertRaises(ValueError):
            parse_json_line('{"schema_version":2,"fusion":{"target_valid":false,"target_valid":true}}')
        with self.assertRaises(ValueError):
            parse_json_line("[]")

    def test_configuration_validation(self):
        for kwargs in ({"target_ttl_s": 0}, {"max_source_age_s": -1}, {"min_confidence": 2},
                       {"target_ttl_s": float("nan")}, {"duplicate_window": 0},
                       {"require_source_metadata": 1}):
            with self.assertRaises(ValueError):
                BridgeConfig(**kwargs)

    def test_non_mapping_nested_fields_fail_closed(self):
        for path, values in (("fusion", (None, [], "VERIFIED")), ("yolo", (None, [], "tape")),
                             ("qwen", ([], "nutri_express"))):
            for value in values:
                row = observation()
                row[path] = value
                self.assert_safe(VisionBridge().ingest(row, 0))


if __name__ == "__main__":
    unittest.main()
