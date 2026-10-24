"""No motors/serial/model loads: grammar, wire compatibility and async admission."""
import asyncio
import hashlib
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.motion_intents import inspect_motion
from ai_coordinator.motion_reference.mot1 import decode_frames,crc16
from ai_coordinator.gateway import FrameTicket
from ai_coordinator.tests import test_runtime as runtime_fixtures


class MotionIntentTests(unittest.TestCase):
    def test_reference_snapshot_matches_collaborator_hashes(self):
        root=Path(__file__).resolve().parents[1]/'motion_reference'
        manifest=json.loads((root/'ORIGIN.json').read_text(encoding='utf-8'))
        for name,item in manifest['files'].items():
            data=(root/name).read_bytes()
            # Git can normalize CRLF while the original-source receipt retains
            # its raw hash. Check the canonical LF snapshot when published.
            expected=item.get('sha256_lf')
            self.assertEqual(hashlib.sha256(data.replace(b'\r\n',b'\n') if expected else data).hexdigest(),expected or item['sha256'])
        self.assertEqual(crc16(b'123456789'),0x29b1)

    def test_bounded_commands_keep_mot1_wire_compatible(self):
        cases=[('前进',{'cmd':'forward','duration_s':.8},1,800),
               ('请小车后退半秒吧',{'cmd':'backward','duration_s':.5},2,500),
               ('向前走',{'cmd':'forward','duration_s':.8},1,800),
               ('前进一点',{'cmd':'forward','duration_s':.8},1,800),
               ('请前进零点八秒',{'cmd':'forward','duration_s':.8},1,800),
               ('左转',{'cmd':'turn','direction':'left','angle_deg':90.},3,90),
               ('向右转四十五度',{'cmd':'turn','direction':'right','angle_deg':45.},4,45),
               ('左转一百零五度',{'cmd':'turn','direction':'left','angle_deg':105.},3,105),
               ('原地顺时针转一圈',{'cmd':'spin','angle_deg':360.},5,360),
               ('原地逆时针转一圈',{'cmd':'spin','angle_deg':-360.},5,-360),
               ('逆时针转半圈',{'cmd':'spin','angle_deg':-180.},5,-180),
               ('开启巡航',{'cmd':'patrol_start'},6,0),('关闭巡航',{'cmd':'patrol_stop'},7,0)]
        for text,intent,cmd,value in cases:
            with self.subTest(text=text):
                decision=inspect_motion(text);self.assertTrue(decision['valid'])
                self.assertEqual(decision['intent'],intent)
                frames,remainder,errors=decode_frames(bytes.fromhex(decision['wire_preview']))
                self.assertEqual((len(frames),remainder,errors),(1,b'',0))
                self.assertEqual((frames[0].cmd,frames[0].value),(cmd,value))
                self.assertTrue(decision['execution_supported'])

    def test_distance_stage_one_is_explicit_and_never_clipped(self):
        policy={'calibrated':True,'calibrated_speed_mps':.5,'min_speed_mps':.05,
                'max_speed_mps':.5,'max_distance_m':2.,'max_duration_s':5.}
        value=inspect_motion('前进两米',distance=policy,speed_code=10)
        self.assertTrue(value['valid']);self.assertEqual(value['reason'],'timed_distance_estimate')
        self.assertEqual(value['intent']['distance_m'],2.)
        self.assertEqual(value['intent']['distance_target_m'],2.)
        self.assertEqual(value['intent']['distance_mode'],'timed_estimate')
        self.assertEqual(value['intent']['estimated_speed_mps'],.5)
        self.assertEqual(value['intent']['duration_s'],4.)
        self.assertFalse(value['arrival_verified'])
        frames,_,errors=decode_frames(bytes.fromhex(value['wire_preview']))
        self.assertEqual((frames[0].cmd,frames[0].value),(1,4000));self.assertEqual(errors,0)
        # No calibrated speed: preserve the distance, but never invent a
        # duration or a wire frame.
        for missing_policy in (None,{}, {'calibrated':False,'calibrated_speed_mps':.5}):
            with self.subTest(policy=missing_policy):
                missing=inspect_motion('后退一米',distance=missing_policy,speed_code=10)
                self.assertTrue(missing['valid']);self.assertEqual(missing['reason'],'distance_speed_uncalibrated')
                self.assertIsNone(missing['wire_preview'])
                self.assertEqual(missing['intent']['distance_m'],1.)
                self.assertNotIn('duration_s',missing['intent'])
        # At 0.1 m/s, 2 m needs 20 s.  The documented limit is 5 s, so the
        # request is refused explicitly instead of being clipped or split.
        too_long=inspect_motion('前进两米',distance={**policy,'calibrated_speed_mps':.1},speed_code=10)
        self.assertFalse(too_long['valid']);self.assertEqual(too_long['reason'],'distance_exceeds_max_duration')
        self.assertIsNone(too_long['wire_preview'])
        bad_speed=inspect_motion('前进一米',distance={**policy,'calibrated_speed_mps':.9},speed_code=10)
        self.assertFalse(bad_speed['valid']);self.assertEqual(bad_speed['reason'],'distance_speed_out_of_range')

    def test_distance_negative_compound_and_out_of_range_are_refused(self):
        policy={'calibrated':True,'calibrated_speed_mps':.5}
        for text in ('前进两米','不要前进两米','前进然后左转','前进0米','前进三米'):
            with self.subTest(text=text):
                if text=='前进两米':
                    self.assertTrue(inspect_motion(text,distance=policy)['valid']);continue
                value=inspect_motion(text,distance=policy)
                self.assertIsNotNone(value)
                self.assertFalse(value['valid'])

    def test_bad_numbers_are_never_sent(self):
        cases=['前进100米','前进10秒','左转999度','前进负一秒','前进-1秒','前进01秒',
               '左转一二度','左转一百一度','顺时针转90','顺时针转度','左转九十点五度',
               '逆时针转0.333圈','原地逆时针转两圈','前进NaN秒','前进一万秒']
        for text in cases:
            with self.subTest(text=text):
                value=inspect_motion(text)
                self.assertIsNotNone(value)
                self.assertFalse(value['valid']);self.assertNotIn('wire_preview',value)

    def test_original_search_domain_is_left_for_c1b(self):
        for text in ('找螺丝刀','寻找胶带，找不到就找AD钙奶，然后返回','找到胶带后停止搜索','物品已经拿走'):
            self.assertIsNone(inspect_motion(text))


class FakePort:
    """Whole-frame fake matching the SerialPort ticket contract."""
    def __init__(self):
        self.fd=1;self.device='/dev/serial/by-id/fake';self.frames=[];self.sent=[]
    def open(self):self.fd=1
    def enqueue(self,data,priority=False,tag='fam1'):
        ticket=FrameTicket(data,tag)
        if priority:self.frames.insert(0,ticket)
        else:self.frames.append(ticket)
        return ticket
    def drop(self,tag):self.frames=[frame for frame in self.frames if frame.tag!=tag]
    def flush(self):
        count=0
        while self.frames:
            ticket=self.frames.pop(0);ticket.written=len(ticket.data)
            self.sent.append(ticket.data);count+=len(ticket.data)
        return count
    def read(self):raise BlockingIOError
    def close(self):self.fd=None
    def snapshot(self):return {'open':self.fd is not None}


class FakeGateway:
    mode='serial'
    def __init__(self,port=None):self.transport=port or FakePort();self.capabilities=set()
    def start(self):self.transport.open()
    def poll(self,now):return []
    def dispatch(self,command):return {'sent':False,'reason':'test'}
    def snapshot(self):return {'motion_authorized':False,'state':'test'}
    def close(self):self.transport.close()


class PartialPort(FakePort):
    """Force short writes and expose a failure switch."""
    def __init__(self,limit=3):super().__init__();self.limit=limit;self.fail=False
    def flush(self):
        if self.fail:raise OSError('injected serial write failure')
        count=0
        while self.frames:
            ticket=self.frames[0];chunk=ticket.data[ticket.written:ticket.written+self.limit]
            ticket.written+=len(chunk);self.sent.append(chunk);count+=len(chunk)
            if ticket.complete:self.frames.pop(0)
            else:break
        return count


class MotionRuntimeTests(unittest.IsolatedAsyncioTestCase):
    # Reuse only setup/helpers, not the base class's test methods.
    asyncSetUp=runtime_fixtures.RuntimeTests.asyncSetUp
    asyncTearDown=runtime_fixtures.RuntimeTests.asyncTearDown
    done=runtime_fixtures.RuntimeTests.done

    def live(self):
        self.runtime.core.mode='live'
        self.runtime.cfg['gateway']={'mode':'serial','device':'/dev/serial/by-id/fake','allow_motion':False}
        self.runtime.cfg['voice_motion']={'enabled':True,'live_commissioned':True,'backend':'mot1_shared_serial',
            'speed':10,'heartbeat_s':.05,'turn_window_s':.3,'drive_margin_s':.05,
            'distance':{'calibrated':True,'calibrated_speed_mps':.5,'min_speed_mps':.05,
                        'max_speed_mps':.5,'max_distance_m':2.,'max_duration_s':5.}}
        self.runtime.gateway=FakeGateway();self.runtime.gateway.start()

    async def asyncTearDown(self):
        self.runtime.motion.closed=True  # teardown must not queue a second STOP
        await runtime_fixtures.RuntimeTests.asyncTearDown(self)

    async def test_text_preview_does_not_load_qwen_or_send_either_protocol(self):
        with patch.object(self.runtime.gateway,'_queue') as queue:
            result=self.runtime.submit_text('原地逆时针转一圈',plan_only=True)
            self.assertEqual(result['motion']['intent'],{'cmd':'spin','angle_deg':-360.})
            self.assertFalse(result['sent']);self.assertFalse(result['executed']);queue.assert_not_called()
        self.assertEqual(self.runtime.workers['llm'].calls,[])
        self.assertEqual(self.runtime.core.state,'idle');self.assertIsNone(self.runtime.last_motion)

    async def test_asr_result_and_text_share_motion_policy(self):
        self.runtime.workers['asr'].result={'text':'请前进零点八秒','status':'recognized'}
        path=self.root/'input.wav';path.write_bytes(b'ASR input boundary stub')
        result=await self.done(self.runtime.submit_audio({'wav':str(path),'plan_only':True}))
        self.assertEqual(result['status'],'done')
        self.assertEqual(result['result']['parsed']['preset_control']['motion']['intent'],{'cmd':'forward','duration_s':.8})
        self.assertEqual(self.runtime.workers['llm'].calls,[])

    async def test_camera_independent_patrol_preview_and_stop(self):
        self.runtime.cfg['voice_motion']={'enabled':True,'patrol_target':'fpga_preset'}
        result=self.runtime.submit_text('开启巡航',plan_only=True)
        self.assertEqual(result['motion']['intent']['cmd'],'patrol_start')
        self.assertNotIn('camera',self.runtime.workers);self.assertFalse(result['sent'])
        stopped=self.runtime.submit_text('关闭巡航')
        self.assertEqual(stopped['status'],'stopped');self.assertFalse(stopped['motion_authorized'])

    async def test_enabled_live_setting_alone_cannot_bypass_commissioning(self):
        self.runtime.cfg['voice_motion']={'enabled':True,'patrol_target':'fpga_preset'}
        self.runtime.core.mode='live'
        with patch.object(self.runtime.gateway,'_queue') as queue:
            result=self.runtime.submit_text('前进')
            self.assertEqual(result['reason'],'motion_not_commissioned')
            self.assertFalse(result['sent']);queue.assert_not_called()
        self.assertFalse(self.runtime.snapshot()['voice_motion']['live_commissioned'])

    async def test_distance_and_invalid_motion_never_fall_back_to_guessing(self):
        for text in ('前进两米','不要前进','前进然后左转'):
            result=self.runtime.submit_text(text)
            self.assertEqual(result['operation'],'motion');self.assertFalse(result['sent'])
        self.assertEqual(self.runtime.workers['llm'].calls,[])

    async def test_hold_keeps_preview_and_stop_available(self):
        self.runtime.cfg['voice_motion']={'enabled':True,'patrol_target':'fpga_preset'}
        self.runtime.cfg['model_loads_enabled']=False
        result=self.runtime.submit_text('前进')
        self.assertEqual(result['reason'],'model_and_motion_hold');self.assertFalse(result['sent'])
        self.assertEqual(self.runtime.submit_text('急停')['status'],'stopped')

    async def test_voice_motion_stop_returns_its_audio_job_and_cancels_other_work(self):
        self.runtime.workers['asr'].result={'text':'关闭巡航','status':'recognized'}
        path=self.root/'stop.wav';path.write_bytes(b'ASR boundary stub')
        job=await self.done(self.runtime.submit_audio({'wav':str(path)}))
        self.assertEqual(job['status'],'done')
        self.assertEqual(job['result']['parsed']['preset_control']['status'],'stopped')
        self.assertEqual(self.runtime.core.state,'stopped')

    async def test_pending_model_answer_is_superseded_by_a_motion_request(self):
        gate=asyncio.Event();worker=self.runtime.workers['llm'];worker.gate=gate
        job=self.runtime.submit_text('寻找胶带');await worker.started.wait()
        result=self.runtime.submit_text('右转90度');self.assertFalse(result['sent'])
        gate.set();old=await self.done(job)
        self.assertTrue(old['result']['stale']);self.assertEqual(self.runtime.core.state,'idle')

    async def test_existing_mission_keeps_control_ownership(self):
        await self.done(self.runtime.submit_text('找胶带'))
        task=self.runtime.core.task_id
        self.runtime.cfg['voice_motion']={'enabled':True}
        result=self.runtime.submit_text('左转')
        self.assertEqual(result['reason'],'existing_mission_owns_control')
        self.assertEqual(self.runtime.core.task_id,task);self.assertFalse(result['sent'])

    async def test_live_text_executes_mot1_and_never_claims_completion(self):
        self.live()
        result=self.runtime.submit_text('前进')
        self.assertTrue(result['accepted']);self.assertTrue(result['sent'])
        self.assertTrue(result['frame_queued']);self.assertFalse(result['executed'])
        self.assertFalse(result['device_reply_verified']);self.assertFalse(result['action_complete_verified'])
        self.assertFalse(result['physical_stop_verified']);self.assertFalse(result['physical_task_completed'])
        frames,_,errors=decode_frames(b''.join(self.runtime.gateway.transport.sent));self.assertEqual(errors,0)
        self.assertEqual((frames[0].cmd,frames[0].value),(1,800))
        self.runtime.stop();await asyncio.sleep(.12)
        self.assertEqual(decode_frames(b''.join(self.runtime.gateway.transport.sent))[0][-1].cmd,0)

    async def test_plan_only_live_config_never_writes(self):
        self.live()
        result=self.runtime.submit_text('前进',plan_only=True)
        self.assertTrue(result['preview_only']);self.assertFalse(result['sent'])
        self.assertEqual(self.runtime.gateway.transport.sent,[])

    async def test_asr_live_execution_uses_same_backend(self):
        self.live();self.runtime.workers['asr'].result={'text':'后退','status':'recognized'}
        path=self.root/'live.wav';path.write_bytes(b'stub')
        job=await self.done(self.runtime.submit_audio({'wav':str(path)}))
        self.assertEqual(job['status'],'done');self.assertEqual(self.runtime.workers['llm'].calls,[])
        frames,_,_=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertTrue(any(frame.cmd==2 for frame in frames))

    async def test_heartbeat_continues_during_action_and_stop_preempts(self):
        self.live();self.runtime.cfg['voice_motion']['heartbeat_s']=.05
        result=self.runtime.submit_text('前进2秒')
        await asyncio.sleep(.2)
        frames,_,_=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertGreaterEqual(sum(frame.cmd==8 for frame in frames),2)
        before=len(self.runtime.gateway.transport.sent)
        stopped=self.runtime.stop('operator_test')
        self.assertTrue(stopped['motion_stop']['stop_frame_sent'])
        self.assertFalse(stopped['motion_stop']['physical_stop_verified'])
        await asyncio.sleep(.12)
        frames,_,_=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertEqual(frames[-1].cmd,0);self.assertGreaterEqual(len(self.runtime.gateway.transport.sent),before)

    async def test_new_action_drops_old_heartbeats_and_late_task(self):
        self.live();self.runtime.submit_text('前进2秒');await asyncio.sleep(.12)
        self.runtime.submit_text('后退1秒');await asyncio.sleep(.15)
        frames,_,_=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertTrue(any(frame.cmd==2 for frame in frames))
        self.assertFalse(any(frame.cmd==1 for frame in frames[-3:]))
        self.runtime.stop()

    async def test_distance_requires_calibration_and_estimates_only(self):
        self.live();self.runtime.cfg['voice_motion']['distance']={'calibrated':True,'calibrated_speed_mps':.5}
        result=self.runtime.submit_text('前进一米')
        self.assertTrue(result['accepted']);self.assertTrue(result['distance']['arrival_verified'] is False)
        frames,_,_=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertEqual(frames[0].value,2000)
        self.runtime.stop()
        self.runtime.gateway.transport.sent.clear()
        self.runtime.cfg['voice_motion']['distance']={'calibrated':False}
        result=self.runtime.submit_text('后退一米')
        self.assertEqual(result['reason'],'distance_speed_uncalibrated')
        self.assertEqual(self.runtime.gateway.transport.sent,[])

    async def test_no_open_serial_never_sends(self):
        self.live();self.runtime.gateway.transport.fd=None
        result=self.runtime.submit_text('前进')
        self.assertEqual(result['reason'],'serial_not_open')
        self.assertFalse(result['sent']);self.assertEqual(self.runtime.gateway.transport.sent,[])

    async def test_heartbeat_task_is_not_blocked_by_a_slow_model(self):
        self.live();gate=asyncio.Event();self.runtime.workers['llm'].gate=gate
        pending=self.runtime.submit_text('找胶带');await self.runtime.workers['llm'].started.wait()
        self.runtime.submit_text('前进2秒');await asyncio.sleep(.22)
        frames,_,_=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertGreaterEqual(sum(frame.cmd==8 for frame in frames),2)
        self.runtime.stop();gate.set()
        self.assertEqual((await self.done(pending))['status'],'cancelled')

    async def test_short_serial_writes_are_completed_but_still_unverified(self):
        self.live();port=PartialPort(5);self.runtime.gateway=FakeGateway(port)
        result=self.runtime.submit_text('前进')
        self.assertTrue(result['accepted']);self.assertFalse(result['sent'])
        self.assertLess(result['frame_bytes_written'],12)
        await asyncio.sleep(.2)
        self.assertTrue(result['sent'])
        self.assertFalse(result['action_complete_verified'])
        self.runtime.stop()
        for _ in range(4):port.flush()
        frames,_,_=decode_frames(b''.join(port.sent))
        self.assertEqual(frames[-1].cmd,0);self.assertFalse(self.runtime.motion.last_result['physical_stop_verified'])

    async def test_serial_write_failure_never_claims_acceptance(self):
        self.live();port=PartialPort(12);port.fail=True;self.runtime.gateway=FakeGateway(port)
        result=self.runtime.submit_text('前进')
        self.assertFalse(result['accepted']);self.assertFalse(result['sent'])
        self.assertEqual(result['action_state'],'failed');self.assertIn('serial_write_failed',result['reason'])
        self.assertFalse(result['physical_stop_verified'])
        port.fail=False;self.runtime.motion.stop_now('test_cleanup')

    async def test_link_loss_during_action_is_failed_not_completed(self):
        self.live();result=self.runtime.submit_text('前进2秒')
        self.assertTrue(result['accepted']);self.runtime.gateway.transport.fd=None
        await asyncio.sleep(.12)
        self.assertEqual(result['action_state'],'failed')
        self.assertFalse(result['action_complete_verified'])
        self.assertFalse(result['physical_stop_verified'])
        self.assertFalse(result['physical_task_completed'])

    async def test_service_close_attempts_stop_without_claiming_physical_stop(self):
        self.live();self.runtime.submit_text('前进2秒')
        self.runtime.motion.close()
        frames,_,errors=decode_frames(b''.join(self.runtime.gateway.transport.sent))
        self.assertEqual(errors,0);self.assertEqual(frames[-1].cmd,0)
        self.assertFalse(self.runtime.motion.last_result['physical_stop_verified'])
        self.assertTrue(self.runtime.motion.last_result['stop_frame_sent'])

    async def test_unknown_reply_is_not_an_ack_or_completion(self):
        self.live();result=self.runtime.submit_text('前进')
        self.runtime.gateway.transport.reply=b'ACK\r\n'
        await asyncio.sleep(.05)
        self.assertFalse(result['device_reply_verified'])
        self.assertFalse(result['action_complete_verified'])
        self.assertFalse(result['physical_task_completed'])
        self.runtime.stop()

    async def test_voice_ownership_blocks_new_mission(self):
        self.live();self.runtime.submit_text('前进');self.assertTrue(self.runtime.motion.owns_control)
        self.runtime.stop();self.assertTrue(self.runtime.motion.owns_control)
        self.runtime.workers['llm'].result={'content':'{"i":"search","t":"tape"}','finish_reason':'stop'}
        result=await self.done(self.runtime.submit_text('找胶带'))
        self.assertEqual(result['result']['admission']['reason'],'voice_motion_holds_control_until_reset')


if __name__=='__main__':unittest.main()
