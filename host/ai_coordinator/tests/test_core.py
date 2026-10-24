import copy
from pathlib import Path
import struct
import sys
import unittest
import zlib

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.core import Mission,Policy
from ai_coordinator.schema import loads,packet,PacketReader,task,detection
from ai_coordinator.frames import encode_frame,FrameReader


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.core=Mission(mode='simulation');self.core.submit({'i':'search','t':'tape','f':'screwdriver','r':True},10)
        self.start=self.core.drain()[0]

    def status(self,kind,action=None,**extra):
        return {**(action or self.start),'kind':kind,**extra}

    def started(self):self.assertTrue(self.core.on_status(self.status('search_started'),10.1))

    def frame(self,seq,captured,cls='tape',**extra):
        det=[] if cls is None else [{'class':cls,'confidence':.9,'bbox':[1,2,10,20]}]
        return {'task_id':self.core.task_id,'goal_id':self.core.goal_id,'sequence':seq,'captured_at_s':captured,
            'width':640,'height':480,'detections':det,'replay':True,**extra}

    def test_found_requires_two_frames_and_fpga_ack(self):
        self.started();self.core.on_frame(self.frame(1,10.2),10.3)
        self.assertEqual(self.core.state,'searching');self.assertEqual(self.core.drain(),[])
        self.core.on_frame(self.frame(2,10.4),10.5);found=self.core.drain()[0]
        self.assertEqual(self.core.state,'waiting_found_ack')
        self.core.on_status(self.status('target_accepted',found),10.6);home=self.core.drain()[0]
        self.assertEqual(home['kind'],'return_home');self.assertEqual(self.core.state,'returning')
        self.core.on_status(self.status('home_arrived',home),10.7)
        self.assertEqual(self.core.state,'completed');self.assertFalse(self.core.snapshot()['physical_task_completed'])

    def test_primary_success_skips_fallback(self):
        self.core.submit({'i':'search','t':'tape','f':'screwdriver'},10);self.start=self.core.drain()[-1]
        self.started()
        for i in range(2):self.core.on_frame(self.frame(i,10.2+i*.1),10.3+i*.1)
        found=self.core.drain()[0];self.core.on_status(self.status('target_accepted',found),10.6)
        self.assertEqual(self.core.state,'completed');self.assertEqual(self.core.branch,'primary');self.assertEqual(self.core.drain(),[])

    def test_empty_frame_never_means_exhausted(self):
        self.started()
        for i in range(4):self.core.on_frame(self.frame(i,10.2+i*.1,None),10.3+i*.1)
        self.assertEqual(self.core.state,'searching');self.assertEqual(self.core.branch,'primary')

    def test_exhaustion_ack_can_select_fallback(self):
        self.started();self.core.on_frame(self.frame(0,10.2,None),10.3)
        self.core.on_status(self.status('region_exhausted'),10.4)
        self.assertEqual(self.core.target,'screwdriver');self.assertEqual(self.core.branch,'fallback')
        self.assertEqual(self.core.drain()[0]['kind'],'start_search')

    def test_exhaustion_without_vision_fails(self):
        self.started();self.core.on_status(self.status('region_exhausted'),10.2)
        self.assertEqual(self.core.state,'failed');self.assertEqual(self.core.reason,'search_finished_without_fresh_vision')

    def test_timeout_never_selects_fallback(self):
        self.started();self.core.tick(100)
        self.assertEqual(self.core.state,'failed');self.assertEqual(self.core.branch,'primary')

    def test_stop_and_late_ack_cannot_resurrect(self):
        self.core.stop(10.2);self.assertFalse(self.core.on_status(self.status('search_started'),10.3))
        self.assertEqual(self.core.state,'stopped')

    def test_superseded_task_ack_rejected(self):
        old=self.start;self.core.submit({'i':'search','t':'nutri_express'},10.2)
        self.assertFalse(self.core.on_status(self.status('search_started',old),10.3))
        self.assertEqual(self.core.target,'nutri_express')

    def test_wrong_action_and_task_rejected(self):
        for changes in ({'action_id':'other'},{'goal_id':'other'},{'session_id':'other'},{'task_id':'other'},{'v':True}):
            self.assertFalse(self.core.on_status(self.status('search_started',**changes),10.1))

    def test_old_future_reordered_and_duplicate_frames_rejected(self):
        self.started();self.assertTrue(self.core.on_frame(self.frame(2,10.3,None),10.4))
        for event,now in [(self.frame(2,10.4),10.5),(self.frame(1,10.5),10.6),(self.frame(3,9),10.5),
                          (self.frame(3,20),10.5),(self.frame(3,10.4),12),(self.frame(3,10.2),10.5)]:
            self.assertFalse(self.core.on_frame(event,now))

    def test_wrong_target_and_invalid_box_do_not_confirm(self):
        self.started()
        self.core.on_frame(self.frame(1,10.2,'screwdriver'),10.3)
        event=self.frame(2,10.4);event['detections'][0]['bbox']=[0,0,1000,20]
        self.core.on_frame(event,10.5);self.assertEqual(self.core.streak,0)

    def test_confirm_and_unknown_do_not_cancel_current_task(self):
        for kind in ('confirm','unknown'):
            response=self.core.submit({'i':kind},10.2);self.assertTrue(response['active_task_unchanged'])
            self.assertEqual(self.core.state,'waiting_start')

    def test_live_rejects_file_replay_and_unverified_source_time(self):
        self.core.mode='live';self.started()
        self.assertFalse(self.core.on_frame(self.frame(1,10.2),10.3))
        self.assertFalse(self.core.on_frame(self.frame(1,10.2,replay=False),10.3))
        self.assertTrue(self.core.on_frame(self.frame(1,10.2,replay=False,source_timestamp_verified=True),10.3))

    def test_return_requires_matching_arrival(self):
        self.started()
        for i in range(2):self.core.on_frame(self.frame(i,10.2+i*.1),10.3+i*.1)
        found=self.core.drain()[0];self.core.on_status(self.status('target_accepted',found),10.6);home=self.core.drain()[0]
        self.assertFalse(self.core.on_status(self.status('home_arrived',home,action_id='old'),10.7))
        self.core.tick(100);self.assertEqual(self.core.state,'failed')


class ContractTests(unittest.TestCase):
    def test_json_duplicate_and_nonfinite_rejected(self):
        for text in ('{"v":1,"v":2}','{"v":NaN}','{"v":Infinity}','{"v":1e999}'):
            with self.assertRaises(ValueError):loads(text)

    def test_task_whitelist_and_types(self):
        for spec in ({'i':'search','t':'table'},{'i':'search','t':'tape','r':1},{'i':'stop','t':'tape'},
                     {'i':'search','t':'tape','f':'tape'},{'i':'search','t':'tape','speed':100}):
            with self.assertRaises(ValueError):task(spec)

    def test_mission_packets_chunking_crc_and_recovery(self):
        reader=PacketReader();obj={'v':1,'kind':'hello','capabilities':['search_v1']};data=packet(obj)
        for b in data[:-1]:self.assertEqual(reader.feed(bytes([b])),[])
        self.assertEqual(reader.feed(data[-1:]),[obj])
        bad=bytearray(data);bad[-1]^=1
        self.assertEqual(reader.feed(b'noise'+bad+data),[obj]);self.assertGreater(reader.errors,0)

    def test_mission_oversize_header_recovers(self):
        reader=PacketReader();self.assertEqual(reader.feed(b'FAM1\xff\xff'+packet({'ok':1})),[{'ok':1}])
        self.assertLessEqual(len(reader.buffer),4096)

    def test_frame_fragmentation_crc_sequence(self):
        reader=FrameReader();data=encode_frame(b'\x00\xf8'*4,2,2,1)
        self.assertEqual(reader.feed(data[:12]),[]);frames=reader.feed(data[12:]);self.assertEqual(len(frames),1)
        self.assertEqual(frames[0]['width'],2);self.assertFalse(frames[0]['source_timestamp_verified'])
        self.assertEqual(reader.feed(data),[])
        bad=bytearray(encode_frame(b'\x00\xf8'*4,2,2,2));bad[-1]^=1
        self.assertEqual(reader.feed(bad),[]);self.assertEqual(len(reader.feed(encode_frame(b'\0\0'*4,2,2,3))),1)

    def test_frame_invalid_length_bounded(self):
        reader=FrameReader();data=bytearray(encode_frame(b'\0\0',1,1,1));data[16:20]=struct.pack('<I',0xffffffff)
        self.assertEqual(reader.feed(data),[]);self.assertLess(len(reader.buffer),24)


if __name__=='__main__':unittest.main()
