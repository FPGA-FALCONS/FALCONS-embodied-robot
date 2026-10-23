from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import VisionBridge
from serial_export import crc16_ccitt, encode_observation, export_records
from test_bridge import observation


class SerialExportTests(unittest.TestCase):
    def event(self):
        return VisionBridge().ingest(observation(), 0)

    def test_standard_crc_check_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_packet_layout_and_crc(self):
        frame = encode_observation(self.event(), 0x1234)
        self.assertEqual(len(frame), 19)
        self.assertEqual(frame[:9], bytes.fromhex("aa550e1234010302eb"))
        self.assertEqual(frame[9:17], bytes.fromhex("0014001e007800e6"))
        self.assertEqual(int.from_bytes(frame[17:], "big"), crc16_ccitt(frame[2:17]))

    def test_sequence_wrap_is_explicit(self):
        self.assertEqual(encode_observation(self.event(), 65536)[3:5], b"\x00\x00")
        for seq in (-1, True, "2"):
            with self.assertRaises(ValueError):
                encode_observation(self.event(), seq)

    def test_bridge_rejects_tracking_qwen_conflict_before_export(self):
        row = observation(status="TRACKING", action="accept")
        row["qwen"]["class"] = "ad_calcium_milk"
        event = VisionBridge().ingest(row, 0)
        frame = encode_observation(event, 0)
        self.assertEqual(frame[5] & 1, 0)
        self.assertEqual(frame[6], 255)

    def test_nonfinite_or_degenerate_quantized_boxes_revoke(self):
        for box in ([0, 0, float("inf"), 20], [0.01, 0.01, 0.02, 0.02],
                    [65536, 0, 65537, 20], [0, 0, float("nan"), 20]):
            event = self.event()
            event["bbox_xyxy"] = box
            frame = encode_observation(event, 0)
            self.assertEqual(frame[5], 2)
            self.assertEqual(frame[6], 255)

    def test_confidence_quantization_can_only_revoke(self):
        event = self.event()
        event["confidence"] = 0.300
        self.assertEqual(encode_observation(event, 0)[5] & 1, 0)
        event["confidence"] = 77 / 255
        self.assertEqual(encode_observation(event, 0)[8], 77)

    def test_expired_event_is_not_serialized_as_valid(self):
        event = self.event()
        event["ttl_remaining_ms"] = 0
        self.assertEqual(encode_observation(event, 0)[5] & 1, 0)

    def test_export_eof_contains_explicit_revoke(self):
        pairs = export_records([json.dumps(observation())])
        self.assertEqual(len(pairs), 2)
        self.assertEqual(pairs[0][1][5] & 1, 1)
        self.assertEqual(pairs[1][1][5] & 1, 0)
        self.assertTrue(pairs[1][0]["replay_only"])


if __name__ == "__main__":
    unittest.main()
