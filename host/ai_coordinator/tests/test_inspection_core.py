"""Metric geometry, actual receipts and manual priority; no hardware opens."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

import numpy as np

from ai_coordinator.device_roles import choose, camera_config
from ai_coordinator.inspection.commissioning import TimedCalibration as Calibration
from ai_coordinator.inspection.grid import build, bit, cell, PLANE_BYTES
from ai_coordinator.inspection.objects import Objects
from ai_coordinator.inspection.mission import InspectionMission, REQUIRED
from ai_coordinator.inspection.render import render, rgb565, list_entries, map_pixel


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

    def test_demo_stopped_projection_uses_h_and_pose_without_timing_acceptance(self):
        p=profile();p['timing_verified']=False;p['xy_error_m']=None
        point=Calibration(p).locate_stopped(dict(u=640,v=660,ground_verified=True),[1280,720],'camera-1',
            {**pose(2,1,yaw=math.pi/2),'at_s':-1000},camera_metadata={})
        np.testing.assert_allclose(point['xy_m'],[2,2],atol=1e-8)

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

    def test_latest_map_return_does_not_block_off_trace_alternative_roads(self):
        layer = build([], .1)
        self.assertFalse(bit(layer['cost'], *cell((0, 1))))
        self.assertFalse(bit(layer['cost'], *cell((1, 1))))
        self.assertFalse(bit(layer['cost'], *cell((-2, 3))))
        self.assertFalse(any(layer['raw']))
        self.assertFalse(any(layer['cost']))


class InspectionMissionTests(unittest.TestCase):
    def test_pose_history_retains_two_seconds_and_selects_nearest_sample(self):
        m=InspectionMission()
        for t in range(61):m.set_pose(pose(t/10),t/10)
        self.assertEqual(len(m.pose_history),21)
        self.assertEqual(m.pose_history[0]['at_s'],4)
        self.assertEqual(m.pose_at(5.71)['at_s'],5.7)
        frozen=m.pose_at(5.71);m.set_pose(pose(6),9)
        self.assertEqual(frozen['x_m'],5.7)
        m.set_pose(pose(0,epoch=2),9.1)
        self.assertEqual(len(m.pose_history),1)

    def test_stop_receipt_waits_point_three_seconds_before_three_frame_phase(self):
        self.start();self.mission.request_observation(1.1)
        self.ack('stopped',1.2,stopped=True)
        self.mission.tick(1.499);self.assertEqual(self.mission.state,'observe_wait')
        self.mission.tick(1.5);self.assertEqual(self.mission.state,'observing')

    def setUp(self):
        self.mission = InspectionMission(session_id=123)
        self.ready = {key: True for key in REQUIRED}
        self.mission.set_pose(pose(), 1)
        self.mission.add_waypoint([1, 0]);self.mission.prepare_start()

    def start(self):
        self.assertTrue(self.mission.start(self.ready, 1))
        command = self.mission.drain()[-1]
        self.assertEqual(command['cruise_speed_mm_s'], self.mission.cruise_speed_mm_s)
        self.ack('accepted', 1.01)
        return command

    def ack(self, kind, now, **extra):
        event = {**{k: self.mission.expected[k] for k in ('session_id', 'task_id', 'epoch', 'command_id')},
                 'kind': kind, **extra}
        return self.mission.on_status(event, now)

    def test_required_devices_block_start_but_pose_age_is_not_checked(self):
        for key in REQUIRED:
            self.assertFalse(self.mission.start({**self.ready, key: False}, 1))
            self.assertEqual(self.mission.drain(), [])
        self.assertTrue(self.mission.start(self.ready, 2))

    def test_rk_configured_cruise_survives_pause_and_resume(self):
        self.mission=InspectionMission(session_id=123,cruise_speed_mm_s=420)
        self.mission.set_pose(pose(),1)
        self.mission.add_waypoint([1,0]);self.mission.prepare_start()
        self.assertEqual(self.start()['cruise_speed_mm_s'],420)
        self.mission.pause(1.1);self.mission.drain()
        self.assertTrue(self.mission.resume(self.ready,1.2))
        self.assertEqual(self.mission.drain()[0]['cruise_speed_mm_s'],420)

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

    def test_manual_resume_refreshes_layer_before_any_new_goto(self):
        self.start();self.mission.set_manual(True,1.1);self.mission.drain()
        self.mission.set_manual(False,1.2);self.mission.set_pose(pose(.3),1.2)
        layer=build([{**detection(),'confirmed':True}],.2)
        self.assertTrue(self.mission.resume({**self.ready,'layer':False},1.21,layer=layer))
        actions=self.mission.drain();self.assertEqual([a['kind'] for a in actions],['pause'])
        self.assertTrue(self.ack('stopped',1.22,stopped=True))
        self.assertEqual(self.mission.state,'await_layer')
        self.assertEqual(self.mission.drain()[0]['kind'],'commit_layer')
        self.assertTrue(self.ack('layer_committed',1.23,layer_version=1))
        self.assertEqual(self.mission.drain()[0]['kind'],'goto')

    def test_immediate_stopped_arrival_is_sufficient_without_intermediate_accepted_report(self):
        self.assertTrue(self.mission.start(self.ready,1));self.mission.drain()
        self.mission.set_pose(pose(1),1.1)
        self.assertTrue(self.ack('arrived',1.11,stopped=True))
        self.assertEqual(self.mission.state,'await_return_layer')
        self.assertEqual(self.mission.return_goal,[0,0])

    def test_return_and_manual_detour_traces_are_recorded_without_moving_start(self):
        self.start();self.mission.set_manual(True,1.1)
        self.mission.set_pose(pose(.4),1.12)
        self.assertEqual(self.mission.trace[-1],[.4,0]);self.assertEqual(self.mission.start_point,[0,0])
        self.mission.set_manual(False,1.13);self.mission.resume(self.ready,1.14);self.mission.drain()
        self.ack('accepted',1.15);self.mission.set_pose(pose(1),1.16);self.ack('arrived',1.17,stopped=True)
        self.mission.submit_layer(build([],.2),1.18);self.mission.drain();self.ack('layer_committed',1.19,layer_version=1)
        self.ack('accepted',1.20);self.mission.set_pose(pose(.5),1.21)
        self.assertEqual(self.mission.return_trace[-1],[.5,0]);self.assertEqual(self.mission.trace[-1],[1,0])

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

    def test_latest_map_return_targets_actual_start_after_stopped_arrival(self):
        self.start()
        for index in range(1, 5): self.mission.set_pose(pose(index*.25), 1+index*.05)
        self.assertFalse(self.ack('arrived', 1.21, stopped=False))
        self.assertTrue(self.ack('arrived', 1.22, stopped=True))
        self.assertEqual(self.mission.state, 'await_return_layer')
        self.assertEqual(self.mission.return_goal, [0, 0])
        self.assertEqual(self.mission.expected['return_policy'],'latest_map')
        self.assertEqual(self.mission.expected['return_goal_m'],[0,0])
        self.mission.submit_layer(build([], .1), 1.23)
        self.ack('layer_committed', 1.24, layer_version=1)
        self.assertEqual(self.mission.expected['goal_mm'],[0,0])
        self.ack('accepted', 1.25)
        self.ack('arrived', 1.26, stopped=True)
        self.assertEqual(self.mission.state, 'completed')
        with self.assertRaises(ValueError): self.mission.add_waypoint([2, 0])
        self.mission.new_task(); self.assertEqual(self.mission.state, 'edit')
        self.assertIsNone(self.mission.start_point)

    def test_return_origin_survives_manual_detour_and_latest_map_no_path(self):
        self.mission.set_pose(pose(.4,.2),1)
        self.start()
        self.mission.set_pose(pose(1,0),1.2)
        self.ack('arrived',1.21,stopped=True)
        # No outbound corridor is added to this refreshed visual layer.
        self.mission.submit_layer(build([], .1),1.22)
        self.ack('layer_committed',1.23,layer_version=1)
        self.ack('accepted',1.24);self.ack('no_path',1.25)
        self.assertEqual(self.mission.state,'no_path')
        self.assertEqual(self.mission.expected['goal_mm'],[400,200])
        self.ack('running',1.26)
        self.assertEqual(self.mission.state,'returning')
        self.mission.set_manual(True,1.27);self.mission.drain()
        self.mission.set_pose(pose(.7,.8),1.28)
        self.mission.set_manual(False,1.29)
        self.assertEqual(self.mission.drain(),[])
        self.assertTrue(self.mission.resume(self.ready,1.3))
        self.assertEqual(self.mission.drain()[-1]['goal_mm'],[400,200])
        self.assertEqual(self.mission.snapshot()['start_m'],[.4,.2])

    def test_pose_age_does_not_pause_and_localization_reset_invalidates_task(self):
        self.start(); self.mission.tick(1.6)
        self.assertEqual(self.mission.state, 'outbound')
        self.mission.set_pose(pose(epoch=2), 1.7)
        self.assertEqual(self.mission.state, 'invalidated')

    def test_no_path_retains_current_destination_and_corner_sample_is_not_lost(self):
        self.start(); self.ack('no_path', 1.1)
        self.assertEqual(self.mission.state, 'no_path'); self.assertEqual(self.mission.goal, [1, 0])
        self.ack('running', 1.11)
        self.mission.set_pose(pose(.08, yaw=math.pi/2), 1.12)
        self.assertEqual(self.mission.trace[-1], [.08, 0])

    def route(self):
        self.mission.state='edit';self.mission.add_waypoint([2,1]);self.mission.add_waypoint([3,0]);self.mission.prepare_start()

    def test_three_waypoints_advance_once_each_then_return_directly_to_start(self):
        self.route();self.mission.set_pose(pose(.2,.1),1);self.start()
        for i,xy in enumerate(([1,0],[2,1],[3,0])):
            old=dict(self.mission.expected);self.mission.set_pose(pose(*xy),2+i)
            self.assertTrue(self.ack('arrived',2.01+i,stopped=True))
            self.assertFalse(self.mission.on_status({**old,'kind':'arrived','stopped':True},2.02+i))
            self.assertEqual(self.mission.waypoint_index,i+1)
            if i<2:
                self.assertEqual(self.mission.expected['goal_mm'],[[2000,1000],[3000,0]][i])
                self.assertEqual(self.mission.state,'await_start')
                self.ack('accepted',2.03+i)
        self.assertEqual(self.mission.state,'await_return_layer')
        self.assertEqual(self.mission.return_goal,[.2,.1])
        self.assertEqual([p['number'] for p in self.mission.waypoint_arrivals],[1,2,3])
        self.assertAlmostEqual(self.mission.waypoint_arrivals[-1]['elapsed_s'],3.01)
        task=self.mission.task_id
        self.mission.submit_layer(build([],.2),5)
        self.ack('layer_committed',5.1,layer_version=1)
        self.assertEqual(self.mission.expected['goal_mm'],[200,100])
        self.ack('arrived',6,stopped=True)
        self.assertEqual(self.mission.state,'completed');self.assertEqual(self.mission.task_id,task)
        self.assertEqual(self.mission.snapshot()['waypoints_completed'],3)

    def test_manual_on_second_waypoint_resumes_same_point_without_advancing(self):
        self.route();self.start();self.ack('arrived',1.1,stopped=True);self.ack('accepted',1.11)
        self.mission.drain();old=dict(self.mission.expected)
        self.mission.set_manual(True,1.2);self.mission.drain()
        self.mission.set_pose(pose(.5,.4),1.3);self.mission.set_manual(False,1.4)
        self.assertEqual(self.mission.drain(),[])
        self.assertFalse(self.mission.on_status({**old,'kind':'arrived','stopped':True},1.5))
        self.assertEqual(self.mission.waypoint_index,1)
        self.assertTrue(self.mission.resume(self.ready,1.6))
        self.assertEqual(self.mission.expected['goal_mm'],[2000,1000])
        self.assertEqual(self.mission.start_point,[0,0])

    def test_observation_on_second_point_restarts_that_point_after_layer_commit(self):
        self.route();self.start();self.ack('arrived',1.1,stopped=True);self.ack('accepted',1.11)
        self.mission.request_observation(1.2)
        self.assertFalse(self.ack('arrived',1.21,stopped=True))
        self.ack('stopped',1.22,stopped=True);self.mission.tick(1.72)
        self.mission.finish_observation(build([],.2),1.8)
        self.ack('layer_committed',1.9,layer_version=1)
        self.assertEqual(self.mission.expected['goal_mm'],[2000,1000]);self.assertEqual(self.mission.waypoint_index,1)
        self.assertEqual(len(self.mission.waypoint_arrivals),1)

    def test_blocked_second_point_never_skips_to_third(self):
        self.route();self.start();self.ack('arrived',1.1,stopped=True);self.ack('no_path',1.2)
        self.mission.tick(100)
        self.assertEqual(self.mission.state,'no_path');self.assertEqual(self.mission.waypoint_index,1)
        self.assertEqual(self.mission.goal,[2,1]);self.ack('running',101)
        self.assertEqual(self.mission.state,'outbound')

    def test_route_is_frozen_after_review_and_new_task_clears_progress(self):
        for state in ('ready','outbound','paused','manual_paused','completed'):
            self.mission.state=state
            for call in (lambda:self.mission.add_waypoint([2,0]),self.mission.undo_waypoint,self.mission.clear_waypoints):
                with self.subTest(state=state),self.assertRaises(ValueError):call()
        self.mission.new_task()
        self.assertEqual(self.mission.waypoints,[]);self.assertEqual(self.mission.waypoint_index,0)
        self.assertEqual(self.mission.waypoint_arrivals,[]);self.assertIsNone(self.mission.started_at)
        with self.assertRaises(ValueError):self.mission.prepare_start()

    def test_cancel_keeps_partial_route_results_for_report(self):
        self.route();self.start();self.ack('arrived',1.1,stopped=True)
        self.mission.cancel(1.2)
        report=self.mission.snapshot()
        self.assertEqual(report['waypoint_count'],3);self.assertEqual(report['waypoints_completed'],1)
        self.assertEqual(report['waypoint_index'],1);self.assertEqual(report['state'],'cancelled')
        self.assertFalse(any(c['kind']=='goto' for c in self.mission.drain()))


class InspectionRendererTests(unittest.TestCase):
    def test_numbered_waypoints_have_distinct_progress_colours_and_text(self):
        points=[[0,0],[2,0],[4,0]]
        mission=dict(state='outbound',phase='outbound',waypoints_m=points,waypoint_index=1,
            waypoint_count=3,waypoints_completed=1,waypoint_arrivals=[])
        output=render('map',mission,[],calibration_ready=True)
        for xy,colour in zip(points,((80,220,80),(0,220,255),(255,210,150))):
            x,y=map_pixel(xy)
            self.assertTrue(np.any(np.all(output[y-9:y+10,x-9:x+10]==colour,axis=2)))
        lines=list_entries(mission,[])
        self.assertTrue(any('P1' in s and '已完成' in s for s in lines))
        self.assertTrue(any('P2' in s and '当前点' in s for s in lines))
        self.assertTrue(any('P3' in s and '待执行' in s for s in lines))

    def test_edit_panel_uses_blue_fill_and_yellow_text(self):
        output=render('map',dict(state='edit',waypoints_m=[],edit_action=2),[],calibration_ready=True)
        np.testing.assert_array_equal(output[144+2*40-5,440],[170,75,0])
        self.assertTrue(np.any(np.all(output[224:240,456:600]==[0,240,255],axis=2)))

    def test_text_view_can_scroll_to_targets_after_long_waypoint_list(self):
        mission=dict(state='edit',waypoints_m=[[i*.1,0] for i in range(20)])
        objects=[dict(id=1,confirmed=True,xy_m=[1,0],**{'class':'tape'})]
        lines=list_entries(mission,objects)
        self.assertIn('胶带',lines[-1])
        self.assertFalse(np.array_equal(render('list',mission,objects),render('list',mission,objects,scroll=1000)))

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
