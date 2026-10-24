"""Fake USB driver boundary; real image codec/preprocessing. No camera/model opens."""
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.camera_sources import create_camera
from ai_coordinator.v4l2_camera import Camera,select_device,validate_config
from ai_coordinator.tests.test_camera_ingress import vision


class FakeCapture:
    def __init__(self,width=4,height=3,override=None,opened=True):
        self.props={};self.queue=queue.Queue();self.released=False;self.opened=opened
        self.override=override or {};self.width=width;self.height=height
    def isOpened(self):return self.opened
    def set(self,key,value):self.props[key]=value;return True
    def get(self,key):return self.override.get(key,self.props[key])
    def read(self):
        try:return self.queue.get(timeout=.15)
        except queue.Empty:return False,None
    def release(self):self.released=True
    def put(self,frame=None):
        if frame is None:frame=np.zeros((self.height,self.width,3),np.uint8);frame[:,:,2]=255
        self.queue.put((True,frame))


class UVCDeviceSelectionTests(unittest.TestCase):
    def test_default_index_or_missing_stable_identity_refused(self):
        for path in (None,0,'/dev/video0','/dev/v4l/by-id/../by-id/x','http://camera'):
            with self.subTest(path=path),self.assertRaises(ValueError):select_device(path)

    @unittest.skipIf(sys.platform=='win32','Linux sysfs symlink selection')
    def test_usb_uvc_ancestry_and_character_device_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);dev=root/'dev';sysroot=root/'sys';dev.mkdir()
            node=dev/'video2';node.touch();link=dev/'v4l/by-id/usb-test-video-index0'
            link.parent.mkdir(parents=True);link.symlink_to(node)
            usb=sysroot/'devices/usb1/1-1';interface=usb/'1-1:1.0';interface.mkdir(parents=True)
            (usb/'idVendor').write_text('abcd');(usb/'idProduct').write_text('1234')
            (interface/'driver').symlink_to(sysroot/'bus/usb/drivers/uvcvideo')
            entry=sysroot/'class/video4linux/video2';entry.mkdir(parents=True)
            (entry/'device').symlink_to(interface)
            with self.assertRaises(ValueError):select_device(str(link),dev,sysroot)
            with patch.object(Path,'is_char_device',return_value=True):
                selected=select_device(str(link),dev,sysroot)
                self.assertEqual((selected['vid'],selected['pid']),(0xabcd,0x1234))
                (interface/'driver').unlink()
                with self.assertRaises(ValueError):select_device(str(link),dev,sysroot)


class UVCCameraTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.cfg={'device':'/dev/v4l/by-id/usb-test','expected_size':[4,3],
                  'fps':30,'fourcc':'MJPG','warmup_frames':0,'storage':self.tmp.name}
        self.identity={'device':self.cfg['device'],'node':'/dev/video2','vid':123,'pid':456}

    def camera(self,cap):
        with patch('ai_coordinator.v4l2_camera.select_device',return_value=self.identity),patch.object(cv2,'VideoCapture',return_value=cap):
            camera=Camera(self.cfg)
        self.addCleanup(camera.close);return camera

    def await_sequence(self,camera,seq):
        with camera.condition:
            self.assertTrue(camera.condition.wait_for(lambda:camera.error or camera.sequence>=seq,timeout=2))
            self.assertIsNone(camera.error)

    def test_invalid_contract_never_opens_device(self):
        for change in ({'expected_size':None},{'expected_size':[True,3]}, {'expected_size':[8192,8192]},
                       {'fps':float('nan')},{'fps':True},{'fourcc':'AUTO'},{'warmup_frames':-1}):
            with self.subTest(change=change),patch.object(cv2,'VideoCapture') as opened:
                with self.assertRaises(ValueError):validate_config({**self.cfg,**change})
                opened.assert_not_called()
        with self.assertRaises(ValueError):create_camera({'backend':'auto'})

    def test_default_fx2_backend_kept_and_uvc_is_explicit(self):
        with patch('ai_coordinator.usb_camera.Camera') as fx2:
            create_camera({});fx2.assert_called_once_with({})
        with patch('ai_coordinator.v4l2_camera.Camera') as uvc:
            cfg={'backend':'v4l2'};create_camera(cfg);uvc.assert_called_once_with(cfg)

    def test_negotiated_fallback_or_unopened_device_is_released(self):
        for key,value in ((cv2.CAP_PROP_FRAME_WIDTH,320),(cv2.CAP_PROP_FPS,15),
                          (cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'YUYV'))):
            cap=FakeCapture(override={key:value})
            with self.subTest(key=key),self.assertRaises(RuntimeError):self.camera(cap)
            self.assertTrue(cap.released)
        cap=FakeCapture(opened=False)
        with self.assertRaises(RuntimeError):self.camera(cap)
        self.assertTrue(cap.released)

    def test_latest_slot_drops_backlog_and_never_claims_sensor_time(self):
        cap=FakeCapture();camera=self.camera(cap)
        for _ in range(20):cap.put()
        self.await_sequence(camera,20);frame=camera.run({})
        self.assertEqual(frame['sequence'],20);self.assertEqual(frame['sequence_origin'],'host_frame_counter')
        self.assertFalse(frame['source_timestamp_verified']);self.assertFalse(frame['replay'])
        self.assertEqual(frame['timestamp_origin'],'host_receipt_only')
        np.testing.assert_array_equal(cv2.imread(frame['image'])[0,0],[0,0,255])
        self.assertEqual(len(list(Path(self.tmp.name).glob('*.png'))),1)

    def test_uvc_png_flows_through_actual_vision_preprocessing(self):
        self.cfg['expected_size']=[640,375];cap=FakeCapture(640,375);camera=self.camera(cap)
        cap.put();self.await_sequence(camera,1);frame=camera.run({});detector=vision()
        result=detector.run(frame)
        self.assertEqual(result['detections'][0]['bbox'],[280,80,360,120])
        self.assertEqual(result['width'],640);self.assertEqual(result['height'],375)

    def test_wrong_pixel_shape_and_disconnect_fail_without_image(self):
        for pixels in (np.zeros((2,2,3),np.uint8),np.zeros((3,4,3),np.float32),None):
            cap=FakeCapture();camera=self.camera(cap)
            cap.queue.put((pixels is not None,pixels))
            with camera.condition:self.assertTrue(camera.condition.wait_for(lambda:camera.error,timeout=2))
            with self.assertRaises(RuntimeError):camera.run({})
            self.assertEqual(list(Path(self.tmp.name).glob('*.png')),[])
            camera.close();self.assertTrue(cap.released)

    def test_stalled_frame_refused_and_close_releases_owned_capture(self):
        cap=FakeCapture();camera=self.camera(cap);cap.put();self.await_sequence(camera,1)
        with camera.condition:
            seq,_,image=camera.latest;camera.latest=(seq,time.monotonic()-2,image)
        with self.assertRaisesRegex(RuntimeError,'stalled'):camera.run({})
        camera.close();self.assertTrue(cap.released)

    def test_warmup_frames_are_not_delivered(self):
        self.cfg['warmup_frames']=2;cap=FakeCapture();camera=self.camera(cap)
        for _ in range(3):cap.put()
        self.await_sequence(camera,1);self.assertEqual(camera.run({})['sequence'],1)

    def test_no_first_frame_and_no_new_frame_cannot_wait_forever(self):
        cap=FakeCapture();camera=self.camera(cap)
        # Exercise timeout paths without a real blocking driver or a 10s wait.
        with camera.condition:
            camera.started_at=time.monotonic()-11
            camera.condition.wait_for=lambda *args,**kwargs:False
        with self.assertRaisesRegex(RuntimeError,'first frame'):camera.run({})
        with camera.condition:
            camera.latest=(1,time.monotonic()-2,np.zeros((3,4,3),np.uint8));camera.delivered=1
        with self.assertRaisesRegex(RuntimeError,'stalled'):camera.run({})


if __name__=='__main__':unittest.main()
