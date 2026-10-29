import io
import unittest
from ai_coordinator.native_v4l2_camera import HEADER,MJPG,read_frame

class NativeCaptureTests(unittest.TestCase):
    def frame(self,flags=0x2000,**changes):
        fields=dict(magic=b'FVC1',header=48,width=1280,height=720,sequence=123,flags=flags,
                    length=8,fourcc=MJPG,timestamp=123456789,received=123460000)
        fields.update(changes)
        return HEADER.pack(*fields.values())+b'\xff\xd8test\xff\xd9'
    def test_driver_timestamp_is_retained_without_becoming_verified_exposure_time(self):
        payload,meta=read_frame(io.BytesIO(self.frame()),(1280,720))
        self.assertEqual(payload,b'\xff\xd8test\xff\xd9')
        self.assertEqual(meta['timestamp_clock'],'monotonic')
        self.assertEqual(meta['timestamp_source'],'end_of_frame')
        self.assertEqual(meta['source_sequence'],123)
        self.assertEqual(meta['driver_timestamp_s'],123.456789)
        self.assertNotIn('source_timestamp_verified',meta)
    def test_source_of_timestamp_flags_are_not_invented(self):
        for flags,clock,source in [(0,'unknown','end_of_frame'),(0x12000,'monotonic','start_of_exposure'),
                                   (0x22000,'monotonic','unknown'),(0x4000,'unknown','end_of_frame')]:
            _,m=read_frame(io.BytesIO(self.frame(flags)),(1280,720))
            self.assertEqual((m['timestamp_clock'],m['timestamp_source']),(clock,source))
    def test_mode_corruption_or_oversized_payload_is_rejected_before_read(self):
        for change in [dict(width=640),dict(height=480),dict(fourcc=0),dict(length=33554433),dict(magic=b'bad!')]:
            with self.subTest(change=change),self.assertRaises(ValueError):
                read_frame(io.BytesIO(self.frame(**change)),(1280,720))
    def test_truncation_is_not_published_as_a_frame(self):
        for size in (0,20,49,55):
            with self.subTest(size=size),self.assertRaises(EOFError):
                read_frame(io.BytesIO(self.frame()[:size]),(1280,720))

if __name__=='__main__':unittest.main()
