import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,Mock,patch
from ai_coordinator.inspection.controller import InspectionController
from ai_coordinator.inspection import wire
from ai_coordinator.inspection.grid import PLANE_BYTES
import threading

class Video:
    def __init__(self,*args):
        self.ready=True;self.capturing=True;self.done=threading.Event();self.error=None;self.started=0;self.reason=None
    def offer(self,snapshot,report=None):self.latest=snapshot
    def finish(self,reason,report):self.reason=reason;self.capturing=False;self.done.set()
    def status(self):return {'ready':self.ready,'capturing':self.capturing,'end_reason':self.reason}
    def wait(self,*args):pass

class Port:
    fd=1
    def __init__(self):self.frames=[]
    def enqueue(self,data,**kwargs):self.frames.append((data,kwargs))
    def drop(self,tag):return 0
    def flush(self):return 0

class ControllerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.port=Port()
        self.rt=SimpleNamespace(cfg={'inspection':{'enabled':True}},gateway=SimpleNamespace(transport=self.port),
            ai=SimpleNamespace(active=True),run_dir=Path(self.tmp.name),workers={},cue=SimpleNamespace(play=AsyncMock()),
            announcements=SimpleNamespace(_clear=lambda x:None,begin_capture=lambda:None,end_capture=lambda:None))
        self.c=InspectionController(self.rt,recording_factory=Video);self.session=self.c.mission.session_id
        self.packet(wire.CAPS,[self.session,1,1,0,15,0],1)
        self.input(1,0,0,1)
        self.packet(wire.STATE,[self.session,0,1,0,2,0,1|4|8,0],1)
        self.controls(1,1)
    async def asyncTearDown(self):
        await self.c.close();self.tmp.cleanup()
    def packet(self,kind,words,now):self.c.on_packet(dict(kind=kind,words=words),now)
    def input(self,seq,confirms,backs,now,flags=1|4|128|256|(1<<4)):
        self.packet(wire.INPUT,[self.session,1,seq,flags,0,(backs<<16)|confirms,confirms,180|(4<<16)],now)
    def controls(self,sequence,now,manual=False,record=False,mode='visual_inspection'):
        self.c.on_ai_controls(dict(sequence=sequence,enabled=True,manual=manual,estop=False,record=record,mode=mode),now)
    def messages(self):
        reader=wire.Reader();return [m for data,kw in self.port.frames for m in reader.feed(data,now=1)]
    def moving(self):
        m=self.c.mission;m.epoch=1;m.task_id=1;m.state='outbound';m.goal=[1,0];m.start_point=[0,0]
        m.command_id=1;m.expected=dict(session_id=self.session,task_id=1,epoch=1,command_id=1,kind='goto')
        m.pose=dict(epoch=1,x_m=0,y_m=0,yaw_rad=0,at_s=1);m.last_status=1
    async def test_uncommissioned_goal_can_be_edited_but_cannot_emit_goto(self):
        self.c.radar=b'\0'*PLANE_BYTES;self.c.last_map=1
        self.assertTrue(self.c.confirm(1));self.assertEqual(self.c.mission.state,'ready')
        self.assertFalse(self.c.confirm(1.1));self.c._actions(1.1)
        self.assertFalse(any(m['kind']==wire.CONTROL and m['words'][4]&255==wire.OP_GOTO for m in self.messages()))
    async def test_manual_retains_goal_and_requires_explicit_resume(self):
        self.moving();self.c.last_camera=1.;self.c.camera={'old':True};self.controls(2,1.1,manual=True)
        self.assertIsNone(self.c.last_camera);self.assertIsNone(self.c.camera)
        self.assertEqual(self.c.mission.state,'manual_paused');self.assertEqual(self.c.mission.goal,[1,0])
        self.input(2,0,0,1.2);self.controls(3,1.2)
        self.assertEqual(self.c.mission.state,'paused')
        self.assertFalse(any(a['kind']=='goto' for a in self.c.mission.drain()))
    async def test_manual_and_page_switch_keep_video_but_ai_exit_finishes_it(self):
        self.moving();self.c._begin_video(1)
        self.controls(2,1.1,manual=True);self.assertTrue(self.c.video_required)
        self.input(2,0,0,1.2);self.controls(3,1.2);self.assertTrue(self.c.video_required)
        self.controls(4,1.3,mode='chatbot')
        self.assertFalse(self.c.video_required);self.assertEqual(self.c.video.reason,'ai_module_exit')
        self.assertEqual(self.c.mission.state,'cancelled')
    async def test_missing_video_readiness_defers_goto_and_encoder_error_cancels(self):
        self.moving();self.c.mission.actions.append(dict(self.c.mission.expected,at_s=1,goal_mm=[1000,0],layer_version=1,speed_code=30))
        self.c._begin_video(1);self.c.video.ready=False;self.c._actions(1.01)
        self.assertFalse(any(m['kind']==wire.CONTROL and m['words'][4]&255==wire.OP_GOTO for m in self.messages()))
        self.c.video.error='disk full';self.c._video_tick(1.02);self.c._actions(1.02)
        self.assertEqual(self.c.mission.state,'cancelled')
        self.assertFalse(any(m['kind']==wire.CONTROL and m['words'][4]&255==wire.OP_GOTO for m in self.messages()))
    async def test_link_loss_closes_video_before_discarding_task_coordinates(self):
        self.moving();self.c._begin_video(1);self.c.link_lost('link_lost',1.1)
        self.assertEqual(self.c.video.reason,'link_lost');self.assertFalse(self.c.video_required)
    async def test_encoder_failure_between_tick_and_dispatch_still_finalizes_report(self):
        self.moving();self.c._begin_video(1);self.c.video.ready=False;self.c.video.error='encoder died'
        self.c.mission.actions.append(dict(self.c.mission.expected,at_s=1,goal_mm=[1000,0],layer_version=1,speed_code=30))
        self.c._actions(1.01)
        self.assertEqual(self.c.mission.state,'cancelled');self.assertIsNotNone(self.c.report_path)
        self.assertFalse(self.c.video_required)
    async def test_idle_preview_never_creates_video(self):
        self.c.tick(1.1);self.assertIsNone(self.c.video)
    async def test_standalone_recognition_emits_only_camera_display_without_inspection_authority(self):
        import time,numpy as np
        class USB:
            def __init__(self):self.frames=[]
            def status(self):return {'verified':True}
            def bind(self,*args):pass
            def drain_maps(self):return []
            def drop_controls(self):pass
            def offer(self,frame):self.frames.append(frame);return True
            def close(self):pass
        self.c.usb=USB();self.rt.ai.vision=True;self.rt.ai.target=None
        self.input(2,0,0,1.1,flags=4|128) # Camera view; neither child-open nor permission.
        self.controls(2,1.1,mode='object_recognition')
        self.c.camera={'bgr':np.zeros((72,128,3),np.uint8),'sequence':1};self.c.last_camera=time.monotonic()
        self.c.detections=[{'class':'tape','confidence':.9,'bbox':[10,10,30,30],'frame_sequence':1}]
        self.c.tick(1.12);await self.c.render_job
        self.assertTrue(self.c.recognition_display);self.assertFalse(self.c.active)
        self.assertEqual(len(self.c.usb.frames),1);self.assertEqual(self.c.usb.frames[0].view,0)
        self.assertEqual(len(self.c.usb.frames[0].payload),685440)
        self.assertIsNone(self.c.video);self.assertIsNone(self.c.layer)
        self.assertFalse(any(m['kind']==wire.CONTROL and m['words'][4]&255==wire.OP_GOTO for m in self.messages()))
        self.controls(3,1.2,manual=True,mode='object_recognition')
        self.assertFalse(self.c.recognition_display)
    async def test_repeated_timeout_does_not_flood_pause_commands(self):
        self.moving();self.c.pause(1.1,'camera_timeout');command=self.c.mission.command_id
        for n in range(20):self.c.pause(1.2+n*.02,'camera_timeout')
        self.assertEqual(self.c.mission.command_id,command)
    async def test_no_path_receipt_can_recover_using_existing_device_planner(self):
        self.moving();self.c.mission.state='no_path'
        self.packet(wire.STATE,[self.session,1,1,1,1,0,4|8,0],1.1)
        self.assertEqual(self.c.mission.state,'outbound')
    async def test_back_pauses_then_cancels_without_closing_a_moving_page(self):
        self.moving();self.c.back(1.1);self.assertEqual(self.c.mission.state,'paused')
        self.c.back(1.2);self.assertEqual(self.c.mission.state,'cancelled')
        self.c.stopped=False;self.c.back(1.3);self.c._actions(1.3)
        self.assertFalse(any(m['kind']==wire.UI_CLOSE for m in self.messages()))
    async def test_cancel_closes_child_only_after_matching_stopped_receipt(self):
        self.moving();self.c.back(1.1);self.c.back(1.2);self.c.tick(1.21)
        self.assertFalse(any(m['kind']==wire.UI_CLOSE for m in self.messages()))
        expected=self.c.mission.expected
        self.packet(wire.STATE,[self.session,expected['task_id'],1,expected['command_id'],2,0,1|4|8,0],1.22)
        self.c.tick(1.23)
        self.assertTrue(any(m['kind']==wire.UI_CLOSE for m in self.messages()))
    async def test_manual_discards_a_pending_page_close(self):
        self.moving();self.c.back(1.1);self.c.back(1.2);self.assertIsNotNone(self.c.want_close)
        self.controls(2,1.21,manual=True);self.assertIsNone(self.c.want_close)

    async def test_telemetry_timeout_rebinds_session_even_if_fpga_epoch_is_reused(self):
        self.moving();old=self.c.mission.session_id;self.c.tick(1.6)
        self.assertNotEqual(self.c.mission.session_id,old)
        self.assertEqual(self.c.mission.state,'invalidated');self.assertIsNone(self.c.caps)
        self.assertTrue(any(m['kind']==wire.HELLO and m['words'][0]==self.c.mission.session_id for m in self.messages()))
        self.assertFalse(any(m['kind']==wire.CONTROL and m['words'][4]&255==wire.OP_GOTO for m in self.messages()))
    async def test_initial_cursor_uses_actual_current_pose(self):
        self.packet(wire.POSE,[1,1,1000000,200,300,0,1],1.1)
        self.assertEqual(self.c.cursor,[.2,.3])

    async def test_caps_without_state_still_retries_hello_after_fpga_reset(self):
        self.c.actual=None;self.c.tick(2)
        self.assertTrue(any(m['kind']==wire.HELLO for m in self.messages()))

    async def test_localization_reset_archives_old_coordinates_before_discarding_pose(self):
        import json
        self.moving();self.c._epoch(2,1.1)
        report=json.loads(Path(self.c.report_path).read_text(encoding='utf-8'))
        self.assertEqual(report['mission']['epoch'],1)
        self.assertEqual(report['mission']['start_m'],[0,0])
        self.assertEqual(report['mission']['reason'],'localization_reset')
        self.assertEqual(self.c.mission.state,'invalidated')
        self.assertIsNone(self.c.mission.pose);self.assertIsNone(self.c.actual)
        self.assertFalse(self.c.mission.drain())

    async def test_input_counter_gap_pauses_without_replaying_multiple_gestures(self):
        self.moving();self.input(2,3,0,1.1)
        self.assertEqual(self.c.mission.state,'paused');self.assertEqual(self.c.mission.reason,'input_gesture_gap')
    async def test_stale_session_packets_cannot_close_or_change_task(self):
        old=self.c.context;self.packet(wire.INPUT,[self.session^1,1,999,0,0,0,0,0],1.1)
        self.assertEqual(self.c.context,old);self.assertEqual(self.c.mission.state,'edit')
    async def test_page_exit_revokes_active_host_task(self):
        self.moving();self.input(2,0,0,1.1,flags=4|128|(1<<4))
        self.assertFalse(self.c.active);self.assertEqual(self.c.mission.state,'paused')
    async def test_voice_start_does_not_replace_physical_confirmation(self):
        self.c.mission.state='ready';self.c.mission.goal=[1,0]
        self.rt.workers['asr']=SimpleNamespace(call=AsyncMock(return_value={'result':{'text':'开始巡检'}}))
        with patch('ai_coordinator.switched_capture.capture',new=AsyncMock()):
            await self.c._record(self.c.generation,asyncio.Event())
        self.assertEqual(self.c.mission.state,'ready');self.assertIn('SWA',self.c.notice)
    async def test_new_localization_epoch_discards_old_geometry_and_camera(self):
        self.moving();self.c.radar=b'\0'*PLANE_BYTES
        self.packet(wire.CAPS,[self.session,1,2,0,15,0],1.1)
        self.assertEqual(self.c.mission.state,'invalidated');self.assertIsNone(self.c.radar);self.assertEqual(self.c.epoch,2)
    async def test_complete_host_protocol_returns_to_actual_start_and_saves_report(self):
        from ai_coordinator.inspection.grid import build
        import json,zlib
        class USB:
            def __init__(self):self.frames=[]
            def status(self):return {'verified':True}
            def bind(self,*binding):pass
            def offer(self,frame):self.frames.append(frame);return True
            def drain_maps(self):return []
            def drop_controls(self):self.frames.clear()
            def close(self):pass
        usb=USB();self.c.usb=usb;self.c.next_render=float('inf')
        self.c.readiness=lambda now:{key:True for key in ('calibration','timing','camera','fpga','map','layer','speed_mapping','vehicle')}
        self.c.layer=build([],.2);self.c.layer_crc=self.c.layer['crc32'];self.c.mission.layer_version=1
        self.c.mission.set_pose(dict(epoch=1,x_m=.2,y_m=0,yaw_rad=0),1)
        self.c.radar=b'\0'*PLANE_BYTES;self.c.cursor=[1,0]
        self.assertTrue(self.c.confirm(1));self.c.input['user_token']=2
        self.assertTrue(self.c.confirm(1.01));self.c.tick(1.01)
        goto=[m for m in self.messages() if m['kind']==wire.CONTROL][-1]
        self.assertEqual(goto['words'][4]&255,wire.OP_GOTO)
        task,command=goto['words'][1],goto['words'][3]
        self.packet(wire.STATE,[self.session,task,1,command,1,1,4|8|16,0],1.02)
        self.c.mission.set_pose(dict(epoch=1,x_m=1,y_m=0,yaw_rad=0),1.1)
        self.packet(wire.STATE,[self.session,task,1,command,3,1,1|4|8|16,0],1.11)
        self.assertEqual(self.c.mission.state,'await_return_layer')
        self.c.tick(1.12);self.c.tick(1.13)
        payload=bytearray();identity=None;committed=False
        now=1.13
        for _ in range(40):
            while usb.frames:
                frame=usb.frames.pop(0);message=wire.Reader().feed(frame.payload,now=now)[0];words=message['words']
                if message['kind']==wire.LAYER_BEGIN:identity=words
                elif message['kind']==wire.LAYER_DATA:
                    self.assertEqual(words[3]*4,len(payload))
                    payload.extend(b''.join(w.to_bytes(4,'little') for w in words[4:]))
                elif message['kind']==wire.LAYER_COMMIT:
                    self.assertEqual(len(payload),16384);self.assertEqual(zlib.crc32(payload)&0xffffffff,identity[5])
                    command=words[3]
                    self.packet(wire.STATE,[self.session,task,1,command,6,words[4],1|4|8|16,0],now)
                    committed=True
            if committed:break
            now+=.001;self.c.tick(now)
        self.assertTrue(committed);self.c.tick(now+.001)
        goto=[m for m in self.messages() if m['kind']==wire.CONTROL][-1]
        self.assertEqual(goto['words'][5:7],[132,128]) # actual .2m start, not editor origin.
        command=goto['words'][3]
        self.packet(wire.STATE,[self.session,task,1,command,1,2,4|8|16,0],1.2)
        self.c.mission.set_pose(dict(epoch=1,x_m=.2,y_m=0,yaw_rad=0),1.21)
        self.packet(wire.STATE,[self.session,task,1,command,3,2,1|4|8|16,0],1.22)
        self.assertEqual(self.c.mission.state,'completed')
        report=json.loads(Path(self.c.report_path).read_text(encoding='utf-8'))
        self.assertEqual(report['mission']['start_m'],[.2,0]);self.assertEqual(report['return_policy'],'latest_map')

    async def test_only_confirmed_located_objects_reach_speech_during_patrol(self):
        import cv2,numpy as np
        self.moving();self.c.cfg['ground_anchor_policies']={'tape':{'measured':True,'anchor':'bottom_center'}}
        self.c.cfg['object_footprints_m']={'tape':.1}
        pose=dict(x_m=0,y_m=0,yaw_rad=0)
        self.c.clock=SimpleNamespace(estimate=lambda now:{})
        self.c.poses=SimpleNamespace(at=lambda *args:(pose,0),items=[])
        self.c.calibration.locate=Mock(return_value=dict(xy_m=[1,0],xy_error_m=.02))
        self.c._record_evidence=Mock(return_value=True);self.c.discovery_speech.notify=Mock()
        path=Path(self.tmp.name)/'camera.jpg';cv2.imwrite(str(path),np.zeros((72,128,3),np.uint8))
        with patch('ai_coordinator.inspection.controller.capture_time',return_value=(1,0)):
            for seq in range(1,4):
                await self.c.observe(dict(image=str(path),width=128,height=72,sequence=seq),
                    {'detections':[{'class':'tape','confidence':.9,'bbox':[10,10,30,30]}]},1+seq*.1)
                binding,objects=self.c.discovery_speech.notify.call_args.args
                self.assertEqual(len(objects),int(seq==3));self.assertTrue(self.c._speech_allowed(binding))
        self.assertEqual(objects[0]['id'],1)
        self.c.mission.state='paused';self.assertFalse(self.c._speech_allowed(binding))
        self.controls(2,1.4,manual=True);self.assertFalse(self.c._speech_allowed(binding))
        self.c.active=True;self.c.mission.manual=False;self.c.mission.state='completed'
        self.assertFalse(self.c._speech_allowed(binding))

    async def test_camera_preview_without_calibration_never_publishes_coordinates(self):
        import cv2
        import numpy as np
        path=Path(self.tmp.name)/'camera.jpg';cv2.imwrite(str(path),np.zeros((72,128,3),dtype=np.uint8))
        frame=dict(image=str(path),width=128,height=72,sequence=1)
        await self.c.observe(frame,{'detections':[{'class':'tape','confidence':.9,'bbox':[10,10,30,30]}]},1.1)
        self.assertEqual(len(self.c.detections),1);self.assertFalse(self.c.objects.items)
        self.assertFalse(self.c.readiness(1.1)['calibration'])

if __name__=='__main__':unittest.main()
