"""FIMG producer -> real decoder/preprocessing -> deterministic vision stub -> mission.

No actual SDK/model or USB/vehicle hardware is used. Detection tensors only
verify the interface, not model accuracy.
"""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.frames import FrameReader,encode_frame,encode_frame_v2
from ai_coordinator.worker import Vision
from ai_coordinator.core import Mission


class FakeONNX:
    def __init__(self):self.calls=0
    def run(self,outputs,inputs):
        image=inputs['image'];self.calls+=1
        if image.shape!=(1,3,640,640):raise AssertionError('Bad letterbox tensor')
        # A red source pixel remains RGB red; padding is the frozen v17 114.
        np.testing.assert_allclose(image[0,:,132,0],[1,0,0])
        np.testing.assert_allclose(image[0,:,0,0],[114/255]*3)
        result=np.zeros((1,8,1),np.float32)
        result[0,:4,0]=[320,232,80,40]  # source box y=80..120 after py132.
        result[0,5,0]=.95  # screwdriver
        return [result]


def vision():
    value=Vision.__new__(Vision);value.provider='onnx_cpu';value.size=640;value.contract='original'
    value.input=SimpleNamespace(name='image');value.rec=FakeONNX();return value


class CameraIngressTests(unittest.TestCase):
    def test_fimg2_geometry_rgb_and_mission_require_execution_ack(self):
        raw=b'\0\xf8'*(640*375);reader=FrameReader(expected_size=(640,375),versions=(2,))
        detector=vision();core=Mission(mode='simulation')
        core.submit({'i':'search','t':'screwdriver'},10);start=core.drain()[0]
        self.assertTrue(core.on_status({**start,'kind':'search_started'},10.01))
        with tempfile.TemporaryDirectory() as tmp:
            for seq in (0,1):
                wire=encode_frame_v2(raw,640,375,seq);frames=[]
                for offset in range(0,len(wire),65536):frames+=reader.feed(wire[offset:offset+65536])
                self.assertEqual(len(frames),1);frame=frames[0]
                path=Path(tmp)/f'{seq}.raw';path.write_bytes(frame.pop('data'))
                result=detector.run({'image':str(path),**frame})
                self.assertEqual(result['width'],640);self.assertEqual(result['height'],375)
                self.assertEqual(result['detections'][0]['bbox'],[280,80,360,120])
                tag={'task_id':core.task_id,'goal_id':core.goal_id,'sequence':seq,
                     'captured_at_s':10.1+seq*.1,'replay':True,'source_timestamp_verified':False,**result}
                self.assertTrue(core.on_frame(tag,10.15+seq*.1))
        self.assertEqual(detector.rec.calls,2)
        self.assertEqual(core.state,'waiting_found_ack')
        found=core.drain()[0]
        self.assertFalse(core.snapshot()['physical_task_completed'])
        self.assertTrue(core.on_status({**found,'kind':'target_accepted'},10.4))
        self.assertEqual(core.state,'completed');self.assertFalse(core.snapshot()['physical_task_completed'])

    def test_bad_crc_and_wrong_shape_never_reach_vision(self):
        raw=b'\0\xf8'*(640*375);reader=FrameReader(expected_size=(640,375),versions=(2,))
        detector=vision()
        corrupt=bytearray(encode_frame_v2(raw,640,375,0));corrupt[-1]^=1
        wrong=encode_frame_v2(b'\0\0'*4,2,2,1)
        self.assertEqual(reader.feed(corrupt),[]);self.assertEqual(reader.feed(wrong),[])
        self.assertEqual(detector.rec.calls,0)
        self.assertEqual(len(reader.feed(encode_frame_v2(raw,640,375,2))),1)

    def test_explicit_v2_refuses_legacy_and_recovers(self):
        raw=b'\0\0'*4;reader=FrameReader(expected_size=[2,2],versions=(2,))
        self.assertEqual(reader.feed(encode_frame(raw,2,2,0)),[])
        self.assertEqual(len(reader.feed(encode_frame_v2(raw,2,2,1))),1)
        self.assertGreater(reader.header_errors,0)

    def test_config_rejects_invalid_contract(self):
        for shape in ([True,2],[2],[-1,2],[8192,8192],'640x375'):
            with self.assertRaises(ValueError):FrameReader(expected_size=shape)
        for versions in ([],[True],[3],'2'):
            with self.assertRaises(ValueError):FrameReader(versions=versions)

    def test_transport_crc_never_proves_live_capture_time(self):
        core=Mission(mode='live');core.submit({'i':'search','t':'screwdriver'},10)
        start=core.drain()[0];core.on_status({**start,'kind':'search_started'},10.01)
        frame=FrameReader(versions=(2,)).feed(encode_frame_v2(b'\0\0'*4,2,2,0))[0]
        event={'task_id':core.task_id,'goal_id':core.goal_id,'sequence':frame['sequence'],
               'captured_at_s':10.1,'width':2,'height':2,'replay':False,
               'source_timestamp_verified':frame['source_timestamp_verified'],
               'detections':[{'class':'screwdriver','confidence':.99,'bbox':[0,0,1,1]}]}
        self.assertFalse(core.on_frame(event,10.2));self.assertEqual(core.drain(),[])


if __name__=='__main__':unittest.main()
