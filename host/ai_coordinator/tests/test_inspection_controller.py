import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,patch
from ai_coordinator.inspection.controller import InspectionController
from ai_coordinator.inspection import wire
from ai_coordinator.inspection.grid import PLANE_BYTES

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
        self.c=InspectionController(self.rt);self.session=self.c.mission.session_id
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
        self.moving();self.controls(2,1.1,manual=True)
        self.assertEqual(self.c.mission.state,'manual_paused');self.assertEqual(self.c.mission.goal,[1,0])
        self.input(2,0,0,1.2);self.controls(3,1.2)
        self.assertEqual(self.c.mission.state,'paused')
        self.assertFalse(any(a['kind']=='goto' for a in self.c.mission.drain()))
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

    async def test_camera_preview_without_calibration_never_publishes_coordinates(self):
        import cv2
        import numpy as np
        path=Path(self.tmp.name)/'camera.jpg';cv2.imwrite(str(path),np.zeros((72,128,3),dtype=np.uint8))
        frame=dict(image=str(path),width=128,height=72,sequence=1)
        await self.c.observe(frame,{'detections':[{'class':'tape','confidence':.9,'bbox':[10,10,30,30]}]},1.1)
        self.assertEqual(len(self.c.detections),1);self.assertFalse(self.c.objects.items)
        self.assertFalse(self.c.readiness(1.1)['calibration'])

if __name__=='__main__':unittest.main()
