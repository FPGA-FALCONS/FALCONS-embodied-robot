from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from ai_coordinator.frames import FrameReader, encode_frame, encode_frame_v2


class FimgV2Tests(unittest.TestCase):
    def test_v2_round_trip_chunked(self):
        payload = bytes((i * 11) & 0xff for i in range(4 * 3 * 2))
        wire = encode_frame_v2(payload, 4, 3, 7)
        reader = FrameReader()
        frames = []
        for offset in range(0, len(wire), 7):
            frames.extend(reader.feed(wire[offset:offset + 7]))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]['data'], payload)
        self.assertEqual(frames[0]['sequence'], 7)
        self.assertEqual(frames[0]['fimg_version'], 2)
        self.assertTrue(frames[0]['crc_ok'])

    def test_bad_v2_trailer_is_discarded(self):
        payload = bytes(4 * 3 * 2)
        wire = bytearray(encode_frame_v2(payload, 4, 3, 0))
        wire[-1] ^= 0xff
        reader = FrameReader()
        self.assertEqual(reader.feed(bytes(wire)), [])
        self.assertEqual(reader.crc_errors, 1)
        self.assertEqual(len(reader.feed(encode_frame_v2(payload, 4, 3, 1))), 1)

    def test_bad_v2_header_resynchronizes(self):
        payload = bytes(4 * 3 * 2)
        bad = bytearray(encode_frame_v2(payload, 4, 3, 0))
        bad[20] ^= 0xff
        reader = FrameReader()
        frames = reader.feed(bytes(bad) + encode_frame_v2(payload, 4, 3, 1))
        self.assertEqual([frame['sequence'] for frame in frames], [1])
        self.assertGreaterEqual(reader.header_errors, 1)

    def test_v1_and_v2_mix_with_sequence_gap(self):
        payload = bytes(4 * 3 * 2)
        reader = FrameReader()
        frames = reader.feed(encode_frame(payload, 4, 3, 0) +
                             encode_frame_v2(payload, 4, 3, 3))
        self.assertEqual([frame['sequence'] for frame in frames], [0, 3])
        self.assertEqual([frame['fimg_version'] for frame in frames], [1, 2])


if __name__ == '__main__':
    unittest.main()
