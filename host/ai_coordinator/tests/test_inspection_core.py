"""Metric geometry, actual receipts and manual priority; no hardware opens."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

import numpy as np

from ai_coordinator.device_roles import choose, camera_config
from ai_coordinator.inspection.calibration import Calibration
from ai_coordinator.inspection.grid import build, bit, cell, PLANE_BYTES
from ai_coordinator.inspection.objects import Objects
from ai_coordinator.inspection.mission import InspectionMission, REQUIRED
from ai_coordinator.inspection.render import render, rgb565


def profile():
    return {'camera_serial': 'camera-1', 'image_size': [1280, 720],
            **{k: True for k in ('intrinsics_measured', 'extrinsics_measured', 'ground_plane_verified', 'timing_verified')},
            'K': [[600, 0, 640], [0, 600, 360], [0, 0, 1]], 'distortion': [0]*5,
            'T_base_camera': [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, .5], [0, 0, 0, 1]],
            'max_range_m': 3, 'xy_error_m': .05}


def pose(x=0, y=0, epoch=1, yaw=0):
    return {'x_m': x, 'y_m': y, 'yaw_rad': yaw, 'epoch': epoch}


def detection(x=1, y=0, category='tape'):
    return {'class': category, 'xy_m': [x, y], 'radius_m': .05, 'xy_error_m': .03}


class DeviceAndGeometryTests(unittest.TestCase):
    def test_identity_survives_socket_and_video_number_changes(self):
        selector = {'vid': 3034, 'pid': 22614, 'serial': 'camera-1'}
        for node in ('/dev/video20', '/dev/video4'):
            inventory = [{**selector, 'role': 'camera', 'device': '/dev/v4l/by-id/usb-cam-index0',
                          'node': node, 'aliases': ['/dev/v4l/by-path/new-socket']}]
            resolved = camera_config({'selector': selector}, inventory)
            self.assertEqual(resolved['device'], '/dev/v4l/by-id/usb-cam-index0')

    def test_ambiguous_same_model_never_selects_first_camera(self):
        selector = {'vid': 1, 'pid': 2}
        inventory = [{**selector, 'role': 'camera', 'serial': s} for s in ('a', 'b')]
        with self.assertRaises(ValueError): choose(selector, inventory, 'camera')
        self.assertEqual(choose({**selector, 'serial': 'b'}, inventory, 'camera')['serial'], 'b')
        with self.assertRaises(ValueError): choose({'vid': True, 'pid': 2}, inventory, 'camera')

    def test_catalogue_lens_parameters_do_not_enable_metric_geometry(self):
        example = json.loads((Path(__file__).parents[1]/'inspection/inspection.example.json').read_text(encoding='utf-8'))
        self.assertEqual(example['lens_description']['focal_length_mm'], 6)
        self.assertTrue(Calibration(example['calibration']).problems((1280, 720), '200901010001'))

    def locate(self, calibration=None, **change):
        args = dict(anchor={'u': 640, 'v': 660, 'ground_verified': True}, image_size=(1280, 720),
                    camera_serial='camera-1', pose=pose(2, 1, yaw=math.pi/2), epoch=1,
                    captured_at=1., pose_at=1., now=1.1, source_timestamp_verified=True)
        return Calibration(profile() if calibration is None else calibration).locate(**{**args, **change})

    def test_measured_ground_projection_and_world_rotation(self):
        point = self.locate()
        np.testing.assert_allclose(point['xy_m'], [2, 2], atol=1e-8)
        self.assertEqual(point['xy_error_m'], .05)

    def test_unmeasured_misaligned_or_horizon_observations_rejected(self):
        for change in ({'source_timestamp_verified': False}, {'pose_at': 1.1}, {'now': 2},
                       {'epoch': 2}, {'camera_serial': 'other'}, {'image_size': (640, 480)},
                       {'anchor': {'u': 640, 'v': 360, 'ground_verified': True}},
                       {'anchor': {'u': 640, 'v': 660, 'ground_verified': False}}):
            with self.subTest(change=change), self.assertRaises(ValueError): self.locate(**change)
        for key, value in (('K', None), ('distortion', None), ('T_base_camera', None), ('timing_verified', False)):
            with self.subTest(key=key), self.assertRaises(ValueError): self.locate({**profile(), key: value})


class ObjectAndGridTests(unittest.TestCase):
    def test_same_class_objects_remain_distinct_and_frame_skips_are_allowed(self):
        objects = Objects()
        for index, sequence in enumerate((10, 20, 30)):
            confirmed = objects.observe([detection(1), detection(2)], sequence, 1, index*.1)
        self.assertEqual(confirmed, [1, 2])
        objects.observe([], 40, 1, .4)
        self.assertEqual(len([x for x in objects.items if x['confirmed']]), 2)
        with self.assertRaises(ValueError): objects.observe([], 40, 1, .5)
        with self.assertRaises(ValueError): objects.observe([], 41, 2, .5)

    def test_missed_observation_resets_pending_streak_without_erasing_confirmed(self):
        objects = Objects(confirmations=2)
        objects.observe([detection()], 1, 1, 0)
        objects.observe([], 2, 1, .1)
        self.assertEqual(objects.observe([detection()], 3, 1, .2), [])
        self.assertEqual(objects.observe([detection()], 4, 1, .3), [1])
        for i in range(5, 105):
            objects.observe([detection(1+.02*(i % 2))], i, 1, i*.1)
        self.assertLess(objects.items[0]['radius_m'], .09)

    def test_raw_and_inflated_cost_are_independent_and_never_wrap(self):
        item = {**detection(0, 0), 'confirmed': True}
        layer = build([item], .2)
        self.assertEqual(len(layer['raw']), PLANE_BYTES)
        self.assertTrue(bit(layer['raw'], *cell((0, 0))))
        self.assertFalse(bit(layer['raw'], *cell((.2, 0))))
        self.assertTrue(bit(layer['cost'], *cell((.2, 0))))
        self.assertFalse(any(a & ~b for a, b in zip(layer['raw'], layer['cost'])))
        border = build([{**item, 'xy_m': [-6.39, -6.39]}], .2)
        self.assertFalse(bit(border['raw'], 255, 255))
        self.assertEqual(cell((-6.4, -6.4)), (0, 0))
        with self.assertRaises(ValueError): cell((6.4, 0))

    def test_return_corridor_prevents_diagonal_shortcut_without_faking_physical_obstacle(self):
        layer = build([], .1, corridor=[[0, 0], [0, 2], [2, 2]], corridor_half_width_m=.4)
        self.assertFalse(bit(layer['cost'], *cell((0, 1))))
        self.assertTrue(bit(layer['cost'], *cell((1, 1))))
        self.assertFalse(any(layer['raw']))


class InspectionMissionTests(unittest.TestCase):
    def setUp(self):
        self.mission = InspectionMission(session_id=123)
        self.ready = {key: True for key in REQUIRED}
        self.mission.set_pose(pose(), 1)
        self.mission.select_goal([1, 0])

    def start(self):
        self.assertTrue(self.mission.start(self.ready, 1))
        command = self.mission.drain()[-1]
        self.assertEqual(command['speed_code'], 30)
        self.ack('accepted', 1.01)
        return command

    def ack(self, kind, now, **extra):
        event = {**{k: self.mission.expected[k] for k in ('session_id', 'task_id', 'epoch', 'command_id')},
                 'kind': kind, **extra}
        return self.mission.on_status(event, now)

    def test_every_commissioning_gate_and_stale_pose_blocks_motion(self):
        for key in REQUIRED:
            self.assertFalse(self.mission.start({**self.ready, key: False}, 1))
            self.assertEqual(self.mission.drain(), [])
        self.assertFalse(self.mission.start(self.ready, 2))

    def test_manual_preserves_view_and_progress_and_cursor_never_resumes(self):
        old = self.start(); self.mission.set_view(2)
        self.mission.set_manual(True, 1.1)
        self.assertEqual(self.mission.state, 'manual_paused')
        self.assertEqual(self.mission.view, 'list')
        self.assertFalse(self.mission.resume(self.ready, 1.2))
        self.mission.drain(); self.mission.set_manual(False, 1.3)
        self.assertEqual(self.mission.state, 'paused'); self.assertEqual(self.mission.drain(), [])
        self.assertFalse(self.mission.on_status({**old, 'kind': 'arrived', 'stopped': True}, 1.3))
        self.mission.set_pose(pose(.3), 1.3)
        self.assertTrue(self.mission.resume(self.ready, 1.31))
        self.assertEqual(self.mission.drain()[0]['goal_mm'], [1000, 0])

    def test_layer_commit_waits_for_actual_stop_and_matching_version(self):
        self.start(); layer = build([{**detection(), 'confirmed': True}], .2)
        self.mission.submit_layer(layer, 1.1)
        self.assertEqual(self.mission.state, 'await_stop')
        self.assertFalse(self.ack('stopped', 1.11, stopped=False))
        self.assertTrue(self.ack('stopped', 1.12, stopped=True))
        self.assertEqual(self.mission.state, 'await_layer')
        self.assertFalse(self.ack('layer_committed', 1.13, layer_version=9))
        self.assertTrue(self.ack('layer_committed', 1.14, layer_version=1))
        self.assertEqual(self.mission.expected['layer_version'], 1)
        with self.assertRaises(ValueError): self.mission.submit_layer({**layer, 'crc32': 0}, 1.2)

    def test_actual_outbound_trace_is_reversed_only_after_stopped_arrival(self):
        self.start()
        for index in range(1, 5): self.mission.set_pose(pose(index*.25), 1+index*.05)
        self.assertFalse(self.ack('arrived', 1.21, stopped=False))
        self.assertTrue(self.ack('arrived', 1.22, stopped=True))
        self.assertEqual(self.mission.state, 'await_return_layer')
        self.assertEqual(self.mission.return_points, [[.75, 0], [.5, 0], [.25, 0], [0, 0]])
        self.mission.submit_layer(build([], .1, corridor=self.mission.trace, corridor_half_width_m=.4), 1.23)
        self.ack('layer_committed', 1.24, layer_version=1)
        for index in range(4):
            self.ack('accepted', 1.25+index*.02)
            self.ack('arrived', 1.26+index*.02, stopped=True)
        self.assertEqual(self.mission.state, 'completed')
        with self.assertRaises(ValueError): self.mission.select_goal([2, 0])
        self.mission.new_task(); self.assertEqual(self.mission.state, 'edit')

    def test_localization_reset_and_link_timeout_never_finish_task(self):
        self.start(); self.mission.tick(1.6)
        self.assertEqual(self.mission.state, 'paused')
        self.assertEqual(self.mission.reason, 'control_or_pose_timeout')
        self.mission.set_pose(pose(epoch=2), 1.7)
        self.assertEqual(self.mission.state, 'invalidated')

    def test_no_path_retains_current_destination_and_corner_sample_is_not_lost(self):
        self.start(); self.ack('no_path', 1.1)
        self.assertEqual(self.mission.state, 'no_path'); self.assertEqual(self.mission.goal, [1, 0])
        self.ack('running', 1.11)
        self.mission.set_pose(pose(.08, yaw=math.pi/2), 1.12)
        self.assertEqual(self.mission.trace[-1], [.08, 0])


class InspectionRendererTests(unittest.TestCase):
    def test_all_three_views_and_rgb565_endianness(self):
        mission = {'state': 'edit', 'trace_m': [], 'goal_m': None}
        for view in ('camera', 'map', 'list'):
            output = render(view, mission, [])
            self.assertEqual(output.shape, (420, 816, 3)); self.assertEqual(len(rgb565(output)), 685440)
        red = np.zeros((420, 816, 3), np.uint8); red[:, :, 2] = 255
        self.assertEqual(rgb565(red)[:2], b'\x00\xf8')

    def test_detection_boxes_are_bound_to_the_actual_camera_frame(self):
        mission = {'state': 'outbound'}
        camera = {'bgr': np.zeros((480, 640, 3), np.uint8), 'sequence': 2}
        base = render('camera', mission, [], camera=camera)
        stale = {'bbox': [100, 100, 200, 200], 'class': 'tape', 'frame_sequence': 1}
        np.testing.assert_array_equal(base, render('camera', mission, [], camera=camera, detections=[stale]))
        self.assertFalse(np.array_equal(base, render('camera', mission, [], camera=camera,
                                                   detections=[{**stale, 'frame_sequence': 2}])))


if __name__ == '__main__': unittest.main()
