import io
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from ai_coordinator.native_v4l2_camera import Camera
from ai_coordinator.v4l2_camera import configured_focus,Camera as OpenCVCamera


class FocusConfigurationTests(unittest.TestCase):
    def test_valid_legacy_and_manual_values(self):
        self.assertIsNone(configured_focus({}))
        for value in (0,260,1023):
            self.assertEqual(configured_focus({'focus':value,'autofocus':False}),value)

    def test_invalid_or_contradictory_focus_is_rejected(self):
        for cfg in ({'focus':True},{'focus':260.0},{'focus':-1},{'focus':'260'},
                    {'focus':2147483648},{'focus':260,'autofocus':True},{'autofocus':False}):
            with self.subTest(cfg=cfg),self.assertRaises(ValueError):configured_focus(cfg)

    def test_opencv_fallback_cannot_silently_ignore_requested_focus(self):
        with patch('ai_coordinator.device_roles.camera_config',side_effect=lambda cfg:cfg):
            with self.assertRaisesRegex(ValueError,'v4l2_native'):OpenCVCamera({'focus':260})

    def test_native_child_receives_focus_and_every_frame_reports_its_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            helper=Path(tmp)/'helper';helper.write_text('fixture')
            cfg={'device':'/dev/v4l/by-id/camera','native_helper':str(helper),'expected_size':[4,3],
                 'fps':30,'fourcc':'MJPG','warmup_frames':0,'storage':tmp,'focus':260,'autofocus':False}
            proc=type('Process',(),{'stdout':io.BytesIO(),'poll':lambda self:0})()
            with patch('ai_coordinator.device_roles.camera_config',side_effect=lambda x:x), \
                 patch('ai_coordinator.native_v4l2_camera.select_device',return_value={'device':cfg['device']}), \
                 patch('ai_coordinator.native_v4l2_camera.subprocess.Popen',return_value=proc) as start, \
                 patch('ai_coordinator.native_v4l2_camera.threading.Thread.start'):
                cam=Camera(cfg)
            try:
                self.assertEqual(start.call_args.args[0][-1],'260')
                self.assertEqual(cam.negotiated['focus'],260);self.assertIs(cam.negotiated['autofocus'],False)
                cam.latest=(1,b'\xff\xd8test\xff\xd9',dict(received_at_s=time.monotonic()))
                frame=cam.run({})
                self.assertEqual(frame['focus_value'],260);self.assertTrue(frame['focus_locked'])
                self.assertEqual(frame['focus_verification'],'v4l2_control_readback_each_frame')
                cam.error='native focus mismatch'
                with self.assertRaisesRegex(RuntimeError,'focus mismatch'):cam.run({})
            finally:
                cam.thread=None;cam.close()


@unittest.skipUnless(sys.platform.startswith('linux') and shutil.which('cc'),'Existing Linux C compiler required')
class NativeFocusDriverTests(unittest.TestCase):
    def test_production_c_driver_boundary_and_legacy_binary_build(self):
        root=Path(__file__).resolve().parents[2]/'usb_camera_v4l2/native'
        with tempfile.TemporaryDirectory() as tmp:
            for source,name in [('capture.c','capture'),('focus_selftest.c','focus_test')]:
                subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',str(root/source),'-o',str(Path(tmp)/name)],check=True,capture_output=True)
            result=subprocess.run([str(Path(tmp)/'focus_test')],check=True,capture_output=True,text=True)
            self.assertIn('PASS: 22 focus checks, 0 errors',result.stdout)
            missing=str(Path(tmp)/'absent-camera')
            for tail in ([],['0'],['260']):
                legacy=subprocess.run([str(Path(tmp)/'capture'),missing,'1280','720','30','1',*tail],capture_output=True)
                self.assertEqual(legacy.returncode,1) # parsed normally; only absent device fails
            for focus in ('-1','not-a-number','2147483648'):
                bad=subprocess.run([str(Path(tmp)/'capture'),'/unused','1280','720','30','1',focus],capture_output=True)
                self.assertEqual(bad.returncode,2)
