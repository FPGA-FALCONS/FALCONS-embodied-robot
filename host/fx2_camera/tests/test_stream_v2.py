from pathlib import Path
import json
import random
import struct
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.stream_frames import (StreamingFrameReader,encode_record,encode_stream_frame,
                                          START,DATA,END,ABORT,HEADER)
from fx2_camera.receive import main,benchmark_usb


def records(seq=1,width=4,height=3):
    raw=b''.join(v.to_bytes(2,'little') for v in range(width*height))
    rows=[encode_record(DATA,seq,y,width,height,raw[y*width*2:(y+1)*width*2]) for y in range(height)]
    return raw,encode_record(START,seq,0,width,height),rows,encode_record(END,seq,height,width,height)


class FIM2Tests(unittest.TestCase):
    def test_random_chunks_fullframe_and_independent_header_layout(self):
        raw,start,rows,end=records();reader=StreamingFrameReader((4,3));wire=start+b''.join(rows)+end
        self.assertEqual(HEADER.size,24)
        self.assertEqual(struct.unpack('<4sBBHIIHHI',start[:24]),(b'FIM2',2,1,0,1,0,4,3,0))
        rng=random.Random(3588);result=[]
        while wire:
            size=rng.randrange(1,47);result+=reader.feed(wire[:size]);wire=wire[size:]
        self.assertEqual(len(result),1);self.assertEqual(result[0]['data'],raw)
        self.assertEqual(result[0]['wire_protocol'],'fim2');self.assertFalse(result[0]['source_timestamp_verified'])
        self.assertEqual(reader.records,5)

    def test_no_publish_before_end(self):
        raw,start,rows,end=records();reader=StreamingFrameReader()
        self.assertEqual(reader.feed(start+b''.join(rows)),[])
        self.assertEqual(reader.feed(end)[0]['data'],raw)

    def test_missing_reordered_or_duplicate_row_never_published(self):
        raw,start,rows,end=records()
        for order in ([0,2],[1,0,2],[0,0,1,2],[0,1]):
            with self.subTest(order=order):
                reader=StreamingFrameReader()
                self.assertEqual(reader.feed(start+b''.join(rows[i] for i in order)+end),[])
                self.assertGreater(reader.errors,0)

    def test_abort_drops_partial_then_next_start_recovers(self):
        raw,start,rows,end=records();reader=StreamingFrameReader()
        bad=start+rows[0]+encode_record(ABORT,1,1,4,3)
        self.assertEqual(reader.feed(bad+rows[1]+rows[2]+end),[])
        result=reader.feed(encode_stream_frame(raw,4,3,2))
        self.assertEqual([f['sequence'] for f in result],[2]);self.assertEqual(reader.aborted_frames,1)

    def test_crc_covers_metadata_and_data_and_next_start_recovers(self):
        raw,start,rows,end=records()
        for index in (8,24,len(rows[0])-1):
            with self.subTest(index=index):
                corrupted=bytearray(rows[0]);corrupted[index]^=1
                reader=StreamingFrameReader();wire=start+corrupted+b''.join(rows[1:])+end
                self.assertEqual(reader.feed(wire),[])
                self.assertEqual(reader.feed(encode_stream_frame(raw,4,3,2))[0]['data'],raw)
                self.assertGreater(reader.errors,0)

    def test_new_start_and_truncated_record_recover(self):
        raw,start,rows,end=records()
        reader=StreamingFrameReader()
        self.assertEqual(reader.feed(start+rows[0][:29]),[])
        result=reader.feed(encode_stream_frame(raw,4,3,2))
        self.assertEqual([f['sequence'] for f in result],[2])

    def test_shape_change_and_wrong_frame_refused(self):
        raw,start,rows,end=records()
        for other in (encode_record(DATA,2,0,4,3,b'\0'*8),encode_record(DATA,1,0,2,3,b'\0'*4)):
            reader=StreamingFrameReader();self.assertEqual(reader.feed(start+other+b''.join(rows)+end),[])

    def test_expected_size_header_length_and_garbage_bounded(self):
        reader=StreamingFrameReader((1024,600))
        raw,start,rows,end=records()
        wire=HEADER.pack(b'FIM2',2,DATA,0,1,0,1024,600,0xffffffff)+b'garbage'
        self.assertEqual(reader.feed(wire+start+b''.join(rows)+end+b'x'*100000),[])
        self.assertLessEqual(len(reader.buffer),3)

    def test_uint32_wrap_and_replay_rejected(self):
        reader=StreamingFrameReader()
        for seq in (0xfffffffe,0xffffffff,0,1):
            self.assertEqual(len(reader.feed(encode_stream_frame(b'\0\0',1,1,seq))),1)
        self.assertEqual(reader.feed(encode_stream_frame(b'\0\0',1,1,1)),[])

    def test_assembly_timeout_keeps_rows_from_becoming_frame(self):
        raw,start,rows,end=records();reader=StreamingFrameReader(max_frame_age_s=.5)
        self.assertEqual(reader.feed(start+rows[0],now_s=0),[])
        self.assertEqual(reader.feed(b''.join(rows[1:])+end,now_s=.6),[])
        self.assertEqual(reader.aborted_frames,1)
        self.assertEqual(len(reader.feed(encode_stream_frame(raw,4,3,2),now_s=.7)),1)

    def test_full_resolution_frame_30fps_wire_budget(self):
        raw=b'\0\xf8'*(1024*600);wire=encode_stream_frame(raw,1024,600,0)
        self.assertEqual(len(wire),1245656)
        reader=StreamingFrameReader((1024,600));frames=[]
        for offset in range(0,len(wire),65536):frames+=reader.feed(wire[offset:offset+65536])
        self.assertEqual(len(frames),1);self.assertEqual(frames[0]['data'],raw)
        self.assertLess(len(reader.buffer),24)

    def test_encode_refuses_bad_payload_control_and_dimensions(self):
        for call in (lambda:encode_record(DATA,1,0,4,3,b'\0'),
                     lambda:encode_record(START,1,1,4,3),lambda:encode_record(END,1,0,4,3),
                     lambda:encode_record(ABORT,1,4,4,3),lambda:encode_record(START,-1,0,4,3),
                     lambda:encode_stream_frame(b'',4,3,1)):
            with self.assertRaises(ValueError):call()

    def test_offline_cli_benchmark_no_images_and_no_hardware_claim(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);wire=root/'wire.bin';out=root/'session'
            wire.write_bytes(encode_stream_frame(b'\0\xf8'*12,4,3,1))
            self.assertEqual(main(['--input',str(wire),'--protocol','fim2','--benchmark','--out',str(out)]),0)
            summary=json.loads((out/'summary.json').read_text())
            self.assertEqual(summary['frames'],1);self.assertFalse(summary['hardware_tested'])
            self.assertTrue(summary['benchmark_no_image_io']);self.assertFalse(list(out.glob('*.raw')))

    def test_usb_benchmark_counts_complete_frames_and_gaps(self):
        class FakeCamera:
            bytes_received=100;usb_calls=2;selection={};parser=StreamingFrameReader()
            def read_frames(self):return [{'sequence':s,'width':1024,'height':600} for s in (2,4)]
        result=benchmark_usb(FakeCamera(),2,1)
        self.assertEqual(result['frames'],2);self.assertEqual(result['sequence_gaps'],1)
        self.assertEqual(result['image_sizes'],[(1024,600)])


if __name__=='__main__':unittest.main()
