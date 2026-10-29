"""Commissioning boundaries; no real camera/model/service opens."""
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from ai_coordinator import uvc_prepare as uvc
from ai_coordinator.tests.test_v4l2_camera import FakeCapture


def request():
    return {'device': '/dev/v4l/by-id/usb-camera-video-index0', 'width': 4,
            'height': 3, 'fps': 30, 'fourcc': 'MJPG', 'frames': 3, 'warmup_frames': 0}


class UVCPreparationTests(unittest.TestCase):
    def test_metadata_capability_does_not_inherit_device_wide_video_bit(self):
        data = bytearray(104)
        data[:8] = b'uvcvideo'
        struct.pack_into('=III', data, 80, 0, 0x84a00001, 0x04a00000)
        decoded = uvc.decode_capability(data)
        self.assertFalse(decoded['video_capture'])
        self.assertTrue(decoded['metadata_capture'])
        struct.pack_into('=I', data, 88, 0x04200001)
        self.assertTrue(uvc.decode_capability(data)['video_capture'])

    def test_discrete_modes_do_not_leak_size_across_format_or_stepwise(self):
        text = """[0]: 'MJPG' (Motion-JPEG)
 Size: Discrete 1280x720
 Interval: Discrete 0.033s (30.000 fps)
 Interval: Discrete 0.067s (15.000 fps)
 Size: Stepwise 32x32 - 4096x4096
 Interval: Discrete 0.033s (30.000 fps)
[1]: 'YUYV'
 Interval: Discrete 0.033s (30.000 fps)
 Size: Discrete 640x480
 Interval: Discrete 0.033s (30.000 fps)
"""
        modes = uvc.discrete_modes(text)
        self.assertEqual([(x['fourcc'], x['width'], x['fps']) for x in modes],
                         [('MJPG', 1280, 30), ('MJPG', 1280, 15), ('YUYV', 640, 30)])
        self.assertFalse(uvc.advertised({'modes': modes}, {**request(), 'width': 1920, 'height': 1080}))

    def test_invalid_request_does_not_create_session_or_open_worker(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'capture'
            for change in ({'device': '/dev/video0'}, {'device': '/dev/v4l/by-id/../by-id/a'},
                           {'fps': float('nan')}, {'frames': 1}, {'width': True},
                           {'fourcc': 'AUTO'}, {'warmup_frames': -1}):
                with self.subTest(change=change), patch.object(subprocess, 'run') as run:
                    with self.assertRaises(ValueError):
                        uvc.capture_session({**request(), **change}, output, 10)
                    run.assert_not_called()
                    self.assertFalse(output.exists())

    def test_silent_negotiation_fallback_refused(self):
        cfg = request()
        for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, 320), (cv2.CAP_PROP_FPS, 15),
                            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))):
            cap = FakeCapture(override={prop: value})
            for key, val in ((cv2.CAP_PROP_FRAME_WIDTH, 4), (cv2.CAP_PROP_FRAME_HEIGHT, 3),
                             (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))):
                cap.set(key, val)
            with self.subTest(prop=prop), self.assertRaises(RuntimeError):
                uvc.verify_mode(cap, cv2, cfg)

    def test_capture_saves_bounded_evidence_and_never_claims_metric_time_or_stereo(self):
        cap = FakeCapture()
        for _ in range(3):
            cap.put()
        with tempfile.TemporaryDirectory() as tmp, patch.object(uvc.time, 'monotonic', side_effect=(1, 1.1, 1.2)):
            summary = uvc.collect(cap, cv2, request(), Path(tmp))
            self.assertEqual(summary['frames_read'], 3)
            self.assertAlmostEqual(summary['measured_host_fps'], 10)
            self.assertEqual(len(summary['samples']), 3)
            self.assertFalse(summary['source_timestamp_verified'])
            self.assertFalse(summary['stereo_sync_verified'])
            self.assertFalse(summary['metric_calibration_verified'])

    def test_bad_pixels_or_disconnect_fail(self):
        for frame in (None, np.zeros((2, 2, 3), np.uint8), np.zeros((3, 4, 3), np.float32)):
            cap = FakeCapture()
            cap.queue.put((frame is not None, frame))
            with tempfile.TemporaryDirectory() as tmp, self.assertRaises(RuntimeError):
                uvc.collect(cap, cv2, request(), Path(tmp))

    def test_parent_timeout_records_failure_and_never_creates_active_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'session'
            with patch.object(subprocess, 'run', side_effect=subprocess.TimeoutExpired('capture', 1)):
                result = uvc.capture_session(request(), output, 1)
            self.assertFalse(result['success'])
            self.assertEqual(result['reason'], 'capture_timeout')
            self.assertFalse(result['active_config_changed'])
            self.assertFalse(result['motion_commands_sent'])
            self.assertFalse((output / 'camera.candidate.json').exists())
            self.assertEqual(json.loads((output / 'result.json').read_text(encoding='utf-8')), result)
            with self.assertRaises(FileExistsError):
                uvc.capture_session(request(), output, 1)

    def test_metadata_node_is_rejected_before_opencv_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'request.json'
            uvc.write_json(path, request())
            with patch.object(uvc, 'inspect_device', return_value={'capture_candidate': False}), patch.object(cv2, 'VideoCapture') as opened:
                with self.assertRaisesRegex(ValueError, 'Metadata'):
                    uvc.capture_worker(path)
                opened.assert_not_called()

    def test_fixed_rate_control_restores_original_on_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(uvc, 'select_device', return_value={'node': '/dev/video20'}), \
                 patch.object(uvc, 'query_capability', return_value={'video_capture': True}), \
                 patch.object(uvc, 'dynamic_frame_rate', side_effect=(1, 0, 0, 1, 1)) as control, \
                 patch.object(subprocess, 'run', side_effect=subprocess.TimeoutExpired('capture', 1)):
                result = uvc.capture_session(request(), Path(tmp) / 'session', 1, True)
            self.assertFalse(result['success'])
            self.assertTrue(result['uvc_control_restored'])
            self.assertEqual(control.call_args_list[-2].args, (request()['device'], 1))

    def test_fixed_rate_does_not_change_metadata_node(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(uvc, 'select_device', return_value={'node': '/dev/video21'}), \
                 patch.object(uvc, 'query_capability', return_value={'video_capture': False}), \
                 patch.object(uvc, 'dynamic_frame_rate') as control, \
                 patch.object(subprocess, 'run') as opened:
                result = uvc.capture_session(request(), Path(tmp) / 'session', 1, True)
            self.assertFalse(result['success'])
            control.assert_not_called()
            opened.assert_not_called()

    def test_fixed_rate_restore_failure_is_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(uvc, 'select_device', return_value={'node': '/dev/video20'}), \
                 patch.object(uvc, 'query_capability', return_value={'video_capture': True}), \
                 patch.object(uvc, 'dynamic_frame_rate', side_effect=(1, 0, 0, RuntimeError('unplugged'))), \
                 patch.object(subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, '', '')):
                result = uvc.capture_session(request(), Path(tmp) / 'session', 1, True)
            self.assertFalse(result['success'])
            self.assertFalse(result['uvc_control_restored'])


if __name__ == '__main__':
    unittest.main()
