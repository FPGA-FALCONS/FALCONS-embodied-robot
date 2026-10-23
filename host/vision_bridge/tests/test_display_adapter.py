from pathlib import Path
import copy
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import VisionBridge
from display_adapter import display_packet
from test_bridge import observation
from team_demo import run_demo
import tempfile
import json


class DisplayTests(unittest.TestCase):
    def setUp(self):
        self.event = VisionBridge().ingest(observation("camera0:1"), 10.0)

    def packet(self, event=None, **kwargs):
        args = dict(frame_id="camera0:1", image_size=(640, 480), now_s=10.0)
        args.update(kwargs)
        return display_packet(self.event if event is None else event, **args)

    def test_valid_same_frame_original_pixels(self):
        p = self.packet()
        self.assertEqual(p["boxes"][0]["bbox_xyxy"], [20, 30, 120, 230])
        self.assertEqual(p["boxes"][0]["label"], "营养快线")
        self.assertFalse(p["motion_authorized"])

    def test_cross_camera_or_frame_never_draws(self):
        for frame in ("camera1:1", "camera0:2"):
            self.assertEqual(self.packet(frame_id=frame)["boxes"], [])

    def test_deadline_and_clock_regression(self):
        for now in (9.9, 10.3, 15):
            self.assertEqual(self.packet(now_s=now)["boxes"], [])

    def test_bad_geometry_and_confidence(self):
        for key, value in (("bbox_xyxy", [0, 0, 641, 100]),
                           ("bbox_xyxy", [0, 0, float("nan"), 100]),
                           ("confidence", float("inf")), ("class_id", True),
                           ("source_status", "UNCERTAIN"), ("target_valid", 1)):
            event = copy.deepcopy(self.event)
            event[key] = value
            self.assertEqual(self.packet(event)["boxes"], [], (key, value))

    def test_text_requires_same_frame_and_freshness(self):
        p = self.packet(semantic_text="看到营养快线", semantic_frame_id="camera0:1")
        self.assertEqual(p["semantic_text"], "看到营养快线")
        self.assertFalse(p["semantic_text_is_command"])
        for extra in (dict(semantic_frame_id="camera0:0"), dict(now_s=11)):
            args = dict(semantic_text="旧文字", semantic_frame_id="camera0:1")
            args.update(extra)
            self.assertEqual(self.packet(**args)["semantic_text"], "")

    def test_invalid_shape_fails(self):
        with self.assertRaises(ValueError):
            self.packet(image_size=(640, 0))

    def test_demo_is_deterministic_and_revokes(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            x, y = run_demo(a), run_demo(b)
            self.assertEqual(x, y)
            self.assertEqual(x["valid_targets"], 4)
            self.assertEqual(x["wire_frames"], 8)
            self.assertEqual(x["serial_bytes_sent"], 0)
            events = [json.loads(s) for s in (Path(a) / "observations.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertFalse(events[-1]["target_valid"])


if __name__ == "__main__":
    unittest.main()
