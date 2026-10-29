import unittest
from ai_coordinator.inspection.stream import Container, Reader, DISPLAY, CONTROL, MAP, DIAGNOSTIC, MAX_PAYLOAD,diagnostic_reply,valid_diagnostic


class StreamTests(unittest.TestCase):
    def test_diagnostic_requires_actual_30mhz_and_fpga_signature(self):
        challenge=Container(DIAGNOSTIC,0,0,0,0,42,payload=b'challenge!')
        self.assertFalse(valid_diagnostic(challenge,challenge))
        self.assertFalse(valid_diagnostic(challenge,diagnostic_reply(challenge,48_000_000)))
        self.assertFalse(valid_diagnostic(challenge,diagnostic_reply(challenge,30_100_000)))
        self.assertFalse(valid_diagnostic(challenge,diagnostic_reply(challenge,0)))
        self.assertTrue(valid_diagnostic(challenge,diagnostic_reply(challenge,30_000_017)))
    def test_full_display_fragmentation_and_metadata(self):
        frame=Container(DISPLAY,1,2,3,7,42,816,420,1632,1,bytes(n & 255 for n in range(MAX_PAYLOAD)))
        wire=frame.encode(); reader=Reader(); result=[]
        for n in range(0,len(wire),113):result+=reader.feed(wire[n:n+113],now=1)
        self.assertEqual(result,[frame]);self.assertFalse(reader.buffer)

    def test_crc_header_body_resync_timeout_and_memory_bound(self):
        frame=Container(DIAGNOSTIC,1,2,3,7,42,payload=bytes(range(32)))
        wire=frame.encode()
        for index in (12,60):
            broken=bytearray(wire);broken[index]^=1
            reader=Reader();self.assertEqual(reader.feed(bytes(broken)+wire,now=1),[frame]);self.assertGreater(reader.errors,0)
        reader=Reader();reader.feed(wire[:30],now=1)
        self.assertEqual(reader.feed(wire,now=3),[frame]);self.assertGreater(reader.errors,0)
        reader=Reader();reader.feed(b'x'*2_000_000,now=1);self.assertLessEqual(len(reader.buffer),3)

    def test_map_is_complete_and_not_a_camera_frame(self):
        raw=Container(MAP,5,0,2,0,9,256,256,32,0,b'\x00'*8192)
        self.assertEqual(Reader().feed(raw.encode(),now=1),[raw])
        with self.assertRaises(ValueError):Container(DISPLAY,1,1,1,1,1,256,256,32,0,raw.payload).encode()

    def test_control_bounds_and_no_odd_word(self):
        for payload in (b'x', b'x'*514):
            with self.assertRaises(ValueError):Container(DIAGNOSTIC,1,1,1,1,1,payload=payload).encode()
        with self.assertRaises(ValueError):Container(CONTROL,1,1,1,1,1,payload=b'x'*14).encode()
        with self.assertRaises(ValueError):Container(DIAGNOSTIC,True,1,1,1,1,payload=b'xx').encode()


if __name__=='__main__':unittest.main()


class RouteContainerTests(unittest.TestCase):
    def test_complete_prefix_and_explicit_clear_are_distinct_from_maps(self):
        from ai_coordinator.inspection.stream import ROUTE,route_points
        frame=Container(ROUTE,1,2,3,4,5,256,256,2,1,bytes.fromhex('020080808a80'))
        restored=Reader().feed(frame.encode())[0]
        self.assertEqual(len(route_points(restored)),2)
        self.assertEqual(route_points(Container(ROUTE,1,2,3,4,5,256,256,2,1,b'\0\0')),[])
        with self.assertRaises(ValueError):route_points(Container(ROUTE,1,2,3,4,5,256,256,2,1,b'\x03\0'))
