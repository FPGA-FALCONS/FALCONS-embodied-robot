import unittest

from mot1 import (BACKWARD, FORWARD, SPIN, STOP, TURN_LEFT, TURN_RIGHT,
                  Frame, decode_frames, from_intent, heartbeat)
from voice_nav import parse_motion


class VoiceNavTests(unittest.TestCase):
    def test_directed_motion_defaults(self):
        self.assertEqual(parse_motion('前进')['cmd'], 'forward')
        self.assertEqual(parse_motion('后退')['cmd'], 'backward')
        self.assertEqual(parse_motion('左转')['direction'], 'left')
        self.assertEqual(parse_motion('右转')['direction'], 'right')

    def test_spin_both_directions(self):
        self.assertEqual(parse_motion('顺时针转一圈')['angle_deg'], 360.0)
        self.assertEqual(parse_motion('原地顺时针转一圈')['angle_deg'], 360.0)
        self.assertEqual(parse_motion('逆时针转一圈')['angle_deg'], -360.0)
        self.assertEqual(parse_motion('逆时针转半圈')['angle_deg'], -180.0)

    def test_numbers_and_patrol(self):
        self.assertEqual(parse_motion('左转90度')['angle_deg'], 90.0)
        self.assertEqual(parse_motion('向左转九十度')['angle_deg'], 90.0)
        self.assertEqual(parse_motion('前进2秒')['duration_s'], 2.0)
        self.assertEqual(parse_motion('后退1米')['distance_m'], 1.0)
        self.assertEqual(parse_motion('开启巡航')['cmd'], 'patrol_start')
        self.assertEqual(parse_motion('关闭巡航')['cmd'], 'patrol_stop')

    def test_stop_and_non_motion(self):
        self.assertEqual(parse_motion('停')['cmd'], 'stop')
        self.assertEqual(parse_motion('别动')['cmd'], 'stop')
        self.assertIsNone(parse_motion('找螺丝刀'))
        self.assertIsNone(parse_motion('帮我看看桌上有什么'))

    def test_out_of_range_rejected(self):
        with self.assertRaises(ValueError):
            parse_motion('左转999度')
        with self.assertRaises(ValueError):
            parse_motion('前进9米')

    def test_mot1_round_trip_and_crc(self):
        wire = Frame(7, FORWARD, 800, 40).encode()
        frames, rest, errors = decode_frames(wire)
        self.assertEqual(errors, 0)
        self.assertEqual(rest, b'')
        self.assertEqual(frames[0], Frame(7, FORWARD, 800, 40))

    def test_mot1_bad_crc_and_resync(self):
        bad = bytearray(Frame(1, STOP, 0, 0).encode())
        bad[-1] ^= 0xff
        good = Frame(2, BACKWARD, 500, 40).encode()
        frames, _, errors = decode_frames(bytes(bad) + good)
        self.assertEqual(errors, 1)
        self.assertEqual(frames[0].cmd, BACKWARD)

    def test_intent_to_frame(self):
        self.assertEqual(from_intent({'cmd': 'forward', 'duration_s': 0.8}).cmd, FORWARD)
        self.assertEqual(from_intent({'cmd': 'backward', 'duration_s': 0.8}).cmd, BACKWARD)
        self.assertEqual(from_intent({'cmd': 'turn', 'direction': 'left',
                                      'angle_deg': 90}).cmd, TURN_LEFT)
        self.assertEqual(from_intent({'cmd': 'turn', 'direction': 'right',
                                      'angle_deg': 90}).cmd, TURN_RIGHT)
        self.assertEqual(from_intent({'cmd': 'spin', 'angle_deg': -360}).cmd, SPIN)
        self.assertEqual(from_intent({'cmd': 'stop'}).cmd, STOP)
        self.assertEqual(heartbeat().cmd, 8)


if __name__ == '__main__':
    unittest.main()
