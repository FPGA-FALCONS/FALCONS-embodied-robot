import unittest
from ai_coordinator.inspection import wire
from ai_coordinator.inspection.grid import build


class InspectionWireTests(unittest.TestCase):
    def test_fragmented_interleaved_transaction_and_crc_recovery(self):
        data = wire.encode(wire.CONTROL, [1, 2, 3, 4, 5, 0xffffffff, 7, 30], 77)
        reader = wire.Reader(); received = []
        for byte in data: received += reader.feed(bytes([byte]), now=1)
        self.assertEqual(received, [{'kind': wire.CONTROL, 'sequence': 77,
                                    'words': [1, 2, 3, 4, 5, 0xffffffff, 7, 30]}])
        corrupted = bytearray(data); corrupted[20] ^= 1
        self.assertEqual(reader.feed(corrupted, now=1.1), [])
        self.assertTrue(reader.errors)
        self.assertEqual(len(reader.feed(data, now=1.2)), 1)

    def test_missing_word_or_transaction_timeout_never_commits(self):
        data = wire.encode(wire.HELLO, [1, 2], 3)
        for broken in (data[:12]+data[24:], data[:24]+data[36:]):
            self.assertEqual(wire.Reader().feed(broken, now=1), [])
        reader = wire.Reader(); reader.feed(data[:12], now=1)
        self.assertEqual(reader.feed(data[12:], now=3), [])

    def test_layer_chunks_reconstruct_exact_grid_and_little_endian_bits(self):
        layer = build([{'confirmed': True, 'xy_m': [0, 0], 'radius_m': .05, 'xy_error_m': .03}], .2)
        messages = list(wire.layer_messages(12, 9, 1, layer))
        assembled = bytearray()
        for kind, words in messages[1:-1]:
            self.assertEqual(kind, wire.LAYER_DATA)
            self.assertEqual(words[3]*4, len(assembled))
            self.assertLessEqual(len(words), 32)
            assembled.extend(b''.join(w.to_bytes(4, 'little') for w in words[4:]))
            decoded = wire.Reader().feed(wire.encode(kind, words), now=1)
            self.assertEqual(decoded[0]['words'], words)
        self.assertEqual(assembled, layer['raw']+layer['cost'])

    def test_length_and_word_types_are_bounded(self):
        for words in ([], [1]*33, [-1], [True], [2**32]):
            with self.assertRaises(ValueError): wire.encode(1, words)
        reader = wire.Reader(); reader.feed(b'x'*100000)
        self.assertLessEqual(len(reader.buffer), 3)

    def test_control_preserves_token_and_full_16_bit_cruise_mm_s(self):
        command = dict(kind='goto', session_id=1, task_id=2, epoch=3, command_id=4,
                       goal_mm=[1000, 0], layer_version=7, cruise_speed_mm_s=300)
        words = wire.control(command, user_token=5, commissioned=True)
        self.assertEqual(words[4], (5 << 16) | 0x101)
        self.assertEqual(words[5:7], [148, 128])
        self.assertEqual(words[7], (300 << 16) | 7)
        self.assertEqual(wire.control(command)[4] & 0x100, 0)
        command['cruise_speed_mm_s']=600
        self.assertEqual(wire.control(command)[7]>>16,600)
        command['kind']='pause'
        self.assertEqual(wire.control(command)[7]>>16,0)

    def test_status_stopped_bit_is_not_fabricated_from_host_window(self):
        event = wire.status([1, 2, 3, 4, 3, 7, 0, 0])
        self.assertEqual(event['kind'], 'arrived'); self.assertFalse(event['stopped'])
        self.assertTrue(wire.status([1, 2, 3, 4, 2, 7, 1, 0])['stopped'])

    def test_physical_snapshot_signed_axes_counters_and_reserved_bits(self):
        words = [5, 1, 9, 0x195, (800 << 16) | (-500 & 65535), (2 << 16) | 7, 10, (4 << 16) | 180]
        event = wire.inputs(words)
        self.assertEqual((event['steer'], event['throttle'], event['view']), (-500, 800, 1))
        self.assertEqual((event['confirms'], event['backs'], event['user_token']), (7, 2, 10))
        self.assertTrue(event['open'] and event['permission'])
        for flags in (0x198, 0x1b5, 0x400):
            with self.assertRaises(ValueError): wire.inputs([*words[:3], flags, *words[4:]])


if __name__ == '__main__': unittest.main()
