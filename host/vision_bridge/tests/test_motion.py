"""Unit tests for the visual-servo motion proposal (no hardware)."""
from __future__ import annotations

import unittest

from motion import MotionConfig, propose, propose_zero


def event(cx=0.5, cy=0.5, bw=0.20, bh=0.20, valid=True, cls="ad_calcium_milk"):
    x1, y1 = (cx - bw / 2) * 1280, (cy - bh / 2) * 720
    x2, y2 = (cx + bw / 2) * 1280, (cy + bh / 2) * 720
    return {
        "frame_id": "f",
        "target_valid": valid,
        "target_class": cls if valid else None,
        "bbox_xyxy": [x1, y1, x2, y2] if valid else None,
        "ttl_remaining_ms": 300,
    }


class MotionTests(unittest.TestCase):
    def test_default_is_observation_only(self):
        p = propose(event(), (1280, 720))
        self.assertFalse(p["fpga_auto_valid"])
        self.assertEqual(p["fpga_auto_center"], 0)
        self.assertEqual(p["reason"], "observation_only")

    def test_centered_far_approaches(self):
        e = event(bw=0.03, bh=0.03)  # area ~0.0009 < min_area
        p = propose(e, (1280, 720), allow_motion=True)
        # Too far: stop rather than command blind forward.
        self.assertFalse(p["fpga_auto_valid"] and p["fpga_auto_center"] > 0)

    def test_centered_midrange_forward(self):
        p = propose(event(bw=0.10, bh=0.10), (1280, 720), allow_motion=True)
        self.assertTrue(p["fpga_auto_valid"])
        self.assertGreater(p["fpga_auto_center"], 0)
        self.assertEqual(p["fpga_auto_diff"], 0)

    def test_left_target_turns_left(self):
        p = propose(event(cx=0.25), (1280, 720), allow_motion=True)
        self.assertTrue(p["fpga_auto_valid"])
        self.assertGreater(p["fpga_auto_diff"], 0)
        self.assertLess(p["left_target"], p["right_target"])

    def test_right_target_turns_right(self):
        p = propose(event(cx=0.75), (1280, 720), allow_motion=True)
        self.assertTrue(p["fpga_auto_valid"])
        self.assertLess(p["fpga_auto_diff"], 0)
        self.assertGreater(p["left_target"], p["right_target"])

    def test_close_target_stops(self):
        p = propose(event(bw=0.70, bh=0.70), (1280, 720), allow_motion=True)
        self.assertTrue(p["fpga_auto_valid"])
        self.assertEqual(p["fpga_auto_center"], 0)
        self.assertEqual(p["reason"], "target_too_close")

    def test_invalid_target_stops(self):
        p = propose(event(valid=False), (1280, 720), allow_motion=True)
        self.assertFalse(p["fpga_auto_valid"])
        self.assertEqual(p["left_target"], 0)
        self.assertEqual(p["right_target"], 0)

    def test_unknown_class_stops(self):
        p = propose(event(cls="chair_leg"), (1280, 720), allow_motion=True)
        self.assertFalse(p["fpga_auto_valid"])
        self.assertEqual(p["reason"], "class_not_allowed")

    def test_zero_proposal(self):
        p = propose_zero("eof")
        self.assertFalse(p["fpga_auto_valid"])
        self.assertEqual(p["reason"], "eof")

    def test_bad_config(self):
        with self.assertRaises(ValueError):
            MotionConfig(min_area=0.2, target_area=0.1)


if __name__ == "__main__":
    unittest.main()
