import asyncio
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from ai_coordinator.ai_link import ControlReader, display_frames, glyph, font_rows, BEGIN, GLYPH, COMMIT,FONT
from ai_coordinator.ai_module import AiModule
from ai_coordinator.motion_reference.mot1 import Frame, crc16
from ai_coordinator.runtime import Runtime


class LinkTests(unittest.TestCase):
    def test_fragmented_control_crc_and_csv_isolation(self):
        reader = ControlReader()
        frame = Frame(255, 0x50, 19, 2 | 4 | (2 << 3)).encode()
        data = b'100,20,0,1500\r\n' + frame
        result = []
        for byte in data:
            result += reader.feed(bytes([byte]))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['mode'], 'chat')
        self.assertTrue(result[0]['record'])
        self.assertEqual(result[0]['selected'], 2)
        damaged = bytearray(frame);damaged[8] ^= 1
        self.assertEqual(reader.feed(damaged), [])
        self.assertEqual(reader.errors, 1)
        self.assertEqual(reader.feed(Frame(0, 0x50, 19, 3).encode())[0]['mode'], 'visual_inspection')
        self.assertEqual(reader.feed(Frame(0, 0x50, 19, 24).encode())[0]['selected'], 3)
        self.assertEqual(reader.feed(Frame(0, 1, 800, 30).encode()), [])

    def test_display_has_ordered_transaction_and_gb2312(self):
        frames,cache=display_frames(['前进2秒', '聊天机器人'],17,font_rows())
        decoded = [struct.unpack('>BBhBB', x[4:10]) for x in frames]
        self.assertEqual(decoded[0], (17, BEGIN, 51, 0, 0))
        self.assertEqual(decoded[-1], (17, COMMIT, 51, 0, 0))
        glyphs=[x for x in decoded if x[1]==GLYPH]
        uploads=[x for x in decoded if x[1]==FONT]
        self.assertEqual(len(uploads),len(cache)*16)
        for i, item in enumerate(glyphs):
            self.assertEqual(item[0:2], (17, GLYPH))
            self.assertEqual(item[3] + 256*item[4], i)
        self.assertEqual(glyphs[0][2],cache[glyph('前')])
        repeated,_=display_frames(['前进2秒','聊天机器人'],18,font_rows(),cache)
        self.assertFalse(any(frame[5]==FONT for frame in repeated))
        self.assertEqual(glyph('🙂'), glyph('?'))
        for frame in frames:
            self.assertEqual(crc16(frame[:10]), int.from_bytes(frame[10:], 'big'))

    def test_explicit_same_mode_confirmation_toggle_is_carried(self):
        reader=ControlReader()
        event=reader.feed(Frame(1,0x50,19,32|1).encode())[0]
        self.assertTrue(event['confirm_toggle'])
        self.assertEqual(event['mode'],'object_recognition')
        self.assertEqual(reader.feed(Frame(2,0x50,19,64).encode()),[])

    def test_reset_marker_keeps_display_bank_and_ack_independent(self):
        event=ControlReader().feed(Frame(0,0x50,0x0760,0).encode())[0]
        self.assertTrue(event['reset_pending'])
        self.assertEqual((event['display_ack'],event['display_bank']),(7,1))
        self.assertEqual(ControlReader().feed(Frame(0,0x50,128,0).encode()),[])


class Port:
    fd = 1
    def __init__(self):self.frames = []
    def enqueue(self, data, **kwargs):self.frames.append((data, kwargs))
    def drop(self, tag):pass
    def flush(self):pass


class ControllerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.stops = []
        root = Path(self.directory.name)
        self.rt = types.SimpleNamespace(
            cfg={'ai_module': {'enabled': True}, 'models': {}, 'model_loads_enabled': True},
            config=root/'config.json',run_dir=root, gateway=types.SimpleNamespace(transport=Port()),
            audit=lambda x:None,stop=lambda reason:self.stops.append(reason),
            camera_task=None,image_pump=None,workers={},camera_state='disabled')
        self.ai = AiModule(self.rt)
        self.ai.clock=lambda:1.
        async def vision(enabled):pass
        async def release(previous=None):pass
        self.ai._set_vision = vision
        self.ai._release_models = release
        self.ai._release_chat = release
        self.starts = []
        self.started = asyncio.Queue()
        async def utterance(epoch,chat,released):
            self.starts.append((epoch,chat))
            self.started.put_nowait((epoch,chat))
            await released.wait()
        self.ai._utterance = utterance
        self.seq = 0

    async def asyncTearDown(self):
        await self.ai.close()
        self.directory.cleanup()

    def controls(self, **changes):
        self.seq = (self.seq+1) & 255
        value = {'kind':'ai_controls','sequence':self.seq,'enabled':True,'rc_valid':True,
                 'manual':False,'estop':False,'swb_valid':True,'mode':'semantic_navigation',
                 'selected':0,'chat':False,'vision':False,'record':False}
        value.update(changes)
        self.ai.on_controls(value, 1.)
        return value

    async def test_startup_high_requires_up_and_only_one_capture_per_down(self):
        self.controls(record=True)
        await asyncio.sleep(0)
        self.assertEqual(self.starts, [])
        self.controls(record=False)
        self.controls(record=True)
        await asyncio.wait_for(self.started.get(), .5)
        self.assertEqual(len(self.starts), 1)
        self.controls(record=True)
        await asyncio.sleep(0)
        self.assertEqual(len(self.starts), 1)
        self.controls(record=False)
        await asyncio.sleep(0)
        self.assertTrue(self.ai.released.is_set())

    async def test_fourth_mode_does_not_record_or_drive_through_legacy_semantics(self):
        self.controls(mode='visual_inspection',selected=3)
        self.controls(mode='visual_inspection',selected=3,record=True)
        await asyncio.sleep(.01)
        self.assertEqual(self.starts, [])
        self.assertFalse(self.ai.vision)
        self.assertEqual(self.ai.rejection(), 'visual_inspection_mode_cannot_control_motion')
        self.assertTrue(any('视觉巡检' in line for line in self.ai.lines()))

    async def test_mode_change_cancels_capture_and_cannot_publish_old_result(self):
        self.controls();self.controls(record=True)
        await asyncio.wait_for(self.started.get(), .5)
        epoch = self.ai.epoch
        self.controls(mode='chat', chat=True, selected=2, record=True)
        await asyncio.sleep(0)
        self.assertFalse(self.ai._current(epoch, False))
        self.assertEqual(self.ai.rejection(), 'chat_mode_cannot_control_motion')
        self.assertFalse(self.ai.record_armed)
        self.controls(record=False, mode='chat', chat=True, selected=2)
        self.controls(record=True, mode='chat', chat=True, selected=2)
        new_start = await asyncio.wait_for(self.started.get(), .5)
        self.assertTrue(new_start[1])

    async def test_timeout_and_duplicate_packets_do_not_extend_lease(self):
        event = self.controls()
        self.ai.on_controls(event, 1.4)
        self.ai.tick(1.51)
        self.assertFalse(self.ai.active)
        self.assertIn('ai_control_link_timeout', self.stops)
        self.assertFalse(self.ai.record_armed)
        self.assertIsNone(self.ai.sequence)

    async def test_manual_estop_and_master_off_close_gate(self):
        for change in ({'manual':True}, {'estop':True}, {'enabled':False}, {'rc_valid':False}):
            self.controls()
            self.assertTrue(self.ai.active)
            self.controls(**change)
            self.assertFalse(self.ai.active)
            self.assertIsNotNone(self.ai.rejection())
        self.controls(mode='object_recognition',vision=True,selected=1)
        self.assertTrue(self.ai.vision)
        self.assertEqual(self.ai.rejection(), 'object_recognition_mode_cannot_control_motion')

    async def test_serial_loss_requires_master_off_before_reentry(self):
        self.controls();self.controls(record=True)
        await asyncio.wait_for(self.started.get(),.5)
        self.ai.link_lost('usb_unplug')
        self.assertFalse(self.ai.active)
        self.controls(record=True)
        self.assertFalse(self.ai.active)
        self.controls(enabled=False)
        self.controls(record=False)
        self.assertTrue(self.ai.active)
        self.assertEqual(len(self.starts),1)

    async def test_fpga_reset_clears_font_and_chat_state_accepts_sequence_zero(self):
        self.seq=99;self.controls(mode='chat',chat=True,selected=2)
        self.ai.instruction='旧指令';self.ai.reply='旧回复';self.ai.history=[{'role':'user','content':'旧对话'}]
        self.ai.font_caches=[{1:2},{3:4}];self.ai.display_pending={'old':True}
        self.ai.fpga_reset()
        self.assertEqual(self.ai.font_caches,[{},{}]);self.assertIsNone(self.ai.display_pending)
        self.assertEqual(self.ai.history,[]);self.assertEqual(self.ai.reply,'');self.assertEqual(self.ai.selected,0)
        self.seq=255;self.controls(enabled=False)
        self.assertEqual(self.ai.sequence,0);self.assertFalse(self.ai.active);self.assertFalse(self.ai.rearm_required)
        self.controls(record=True)
        await asyncio.sleep(0)
        self.assertEqual(self.starts,[])

    async def test_runtime_reset_handshake_precedes_sequence_filter_and_runs_once(self):
        self.controls();self.ai.sequence=100;self.ai.font_caches=[{1:2},{3:4}]
        self.rt.ai=self.ai;self.rt.inspection=Mock();self.rt.motion=Mock();self.rt.core=Mock()
        self.rt.tasks=set();self.rt.generation=0;self.rt.fpga_reset_pending=False;self.rt.announcements=Mock()
        event=ControlReader().feed(Frame(0,0x50,64|2|16,0).encode())[0]
        Runtime.on_ai_controls(self.rt,event,1.)
        self.assertEqual(self.ai.sequence,0);self.assertEqual(self.ai.font_caches,[{},{}])
        self.rt.inspection.fpga_reset.assert_called_once_with(1.)
        self.assertEqual(self.rt.gateway.transport.frames[-1][0],Frame(0,0x43,1,0).encode())
        Runtime.on_ai_controls(self.rt,event,1.01)
        self.rt.inspection.fpga_reset.assert_called_once()
        normal=ControlReader().feed(Frame(1,0x50,2|16,0).encode())[0]
        Runtime.on_ai_controls(self.rt,normal,1.02)
        Runtime.on_ai_controls(self.rt,event,1.03)
        self.assertEqual(self.rt.inspection.fpga_reset.call_count,2)

    async def test_manual_preserves_page_history_and_requires_fresh_recording(self):
        self.controls(mode='chat',chat=True,selected=2)
        self.ai.instruction='你好';self.ai.reply='你好，我在这里。'
        self.ai.history=[{'role':'user','content':'你好'}]
        self.controls(mode='chat',chat=True,selected=2,record=True)
        await asyncio.wait_for(self.started.get(),.5)
        self.ai.instruction='你好';self.ai.reply='你好，我在这里。'
        epoch=self.ai.epoch
        self.controls(mode='chat',chat=True,selected=2,manual=True,record=True)
        await asyncio.sleep(0)
        self.assertTrue(self.ai.controls['enabled'])
        self.assertTrue(self.ai.paused_manual)
        self.assertFalse(self.ai._current(epoch,True))
        self.assertEqual(self.ai.reply,'你好，我在这里。')
        self.assertEqual(len(self.ai.history),1)
        self.controls(mode='chat',chat=True,selected=2,record=True)
        self.assertFalse(self.ai.record_armed)
        self.assertEqual(len(self.starts),1)
        self.controls(mode='chat',chat=True,selected=2,record=False)
        self.controls(mode='chat',chat=True,selected=2,record=True)
        await asyncio.wait_for(self.started.get(),.5)
        self.assertEqual(len(self.starts),2)

    async def test_manual_vision_waits_for_new_confirmation_after_cursor_return(self):
        self.controls(mode='object_recognition',vision=True,selected=1,confirm_toggle=False)
        self.assertTrue(self.ai.vision)
        self.controls(mode='object_recognition',vision=True,selected=1,manual=True,confirm_toggle=False)
        self.assertFalse(self.ai.vision)
        self.controls(mode='object_recognition',vision=True,selected=1,confirm_toggle=False)
        self.assertTrue(self.ai.active)
        self.assertFalse(self.ai.vision)
        self.assertTrue(self.ai.vision_resume_required)
        self.controls(mode='object_recognition',vision=True,selected=1,confirm_toggle=True)
        self.assertTrue(self.ai.vision)
        self.assertFalse(self.ai.vision_resume_required)

    async def test_boot_loading_blocks_held_record_until_models_ready_and_fresh_gesture(self):
        self.rt.startup=types.SimpleNamespace(ready=False,state='loading')
        self.controls(record=True)
        self.assertFalse(self.ai.active)
        self.assertIn('模型加载中',self.ai.lines()[3])
        self.rt.startup.ready=True
        self.controls(record=True)
        self.assertFalse(self.ai.record_armed)
        self.controls(record=False);self.controls(record=True)
        await asyncio.wait_for(self.started.get(),.5)
        self.assertEqual(len(self.starts),1)

    async def test_same_mode_confirm_cancels_old_work_without_replaying_held_record(self):
        self.controls(confirm_toggle=False);self.controls(record=True,confirm_toggle=False)
        await asyncio.wait_for(self.started.get(),.5)
        epoch=self.ai.epoch
        self.controls(record=True,confirm_toggle=True)
        self.assertGreater(self.ai.epoch,epoch)
        self.assertFalse(self.ai.record_armed)
        self.assertEqual(len(self.starts),1)

    async def test_boot_preloaded_models_are_not_unloaded_by_idle_ai_release(self):
        from unittest.mock import AsyncMock
        self.rt.cfg['startup']={'enabled':True}
        self.rt.workers={'asr':types.SimpleNamespace(close=AsyncMock())}
        self.ai.active=False;self.ai.chat=False
        with patch.object(self.ai.chat_worker,'close',new_callable=AsyncMock) as chat_close, \
             patch.object(self.ai.tts_worker,'close',new_callable=AsyncMock) as tts_close:
            await AiModule._release_models(self.ai)
            await AiModule._release_chat(self.ai)
            chat_close.assert_not_awaited();tts_close.assert_not_awaited()
            self.rt.workers['asr'].close.assert_not_awaited()

    async def test_service_restart_requires_observing_master_off_before_new_session(self):
        await self.ai.close()
        self.rt.cfg['startup']={'enabled':True}
        self.ai=AiModule(self.rt);self.ai.clock=lambda:1.
        self.controls(record=True)
        self.assertTrue(self.ai.rearm_required)
        self.assertFalse(self.ai.active)
        self.controls(enabled=False)
        self.controls(record=True)
        self.assertTrue(self.ai.active)
        self.assertFalse(self.ai.record_armed)
        self.assertIsNone(self.ai.work)

    async def test_write_failure_closes_link_and_requires_new_master_confirmation(self):
        self.controls()
        closed=[]
        self.rt.gateway.close=lambda:closed.append(True)
        self.rt.gateway.transport.flush=lambda:(_ for _ in ()).throw(OSError('unplugged'))
        self.ai.tick(1.1)
        self.assertEqual(closed,[True])
        self.assertFalse(self.ai.active)
        self.assertTrue(self.ai.rearm_required)
        self.assertIn('unplugged',self.ai.error)
        self.controls()
        self.assertFalse(self.ai.active)

    async def test_missing_high_speed_camera_never_blocks_navigation_or_chat(self):
        self.ai._set_vision=AiModule._set_vision.__get__(self.ai)
        self.controls(mode='object_recognition',vision=True,selected=1)
        await asyncio.gather(*list(self.ai.cleanup))
        self.assertEqual(self.rt.camera_state,'unavailable')
        self.assertEqual(self.ai.state,'error')
        self.controls(mode='chat',chat=True,selected=2)
        await asyncio.gather(*list(self.ai.cleanup))
        self.assertTrue(self.ai.active and self.ai.chat)
        self.assertEqual(self.ai.state,'idle')
        self.assertIsNone(self.ai.error)
        self.assertTrue(self.ai.record_armed)
        self.controls(mode='semantic_navigation',chat=False,selected=0)
        await asyncio.gather(*list(self.ai.cleanup))
        self.assertTrue(self.ai.active)
        self.assertIsNone(self.ai.rejection())
        self.assertIsNone(self.ai.error)
        self.assertTrue(self.ai.record_armed)

    async def test_actual_runtime_http_text_path_honors_master_and_chat_gate(self):
        root = Path(self.directory.name)/'runtime'
        root.mkdir()
        cfg = {'root':str(root),'mode':'shadow','models':{'llm':{}},
               'gateway':{'mode':'shadow'},'ai_module':{'enabled':True},
               'model_loads_enabled':True,'voice_motion':{'enabled':True,'live_commissioned':True}}
        path = root/'config.json';path.write_text(json.dumps(cfg))
        rt = Runtime(path)
        try:
            self.assertIn('ai_master', (await rt.parse('找胶带',rt.generation))['reason'])
            rt.ai.active=True;rt.ai.mode='chat';rt.ai.chat=True
            rt.ai.last_rx=rt.ai.clock()
            self.assertIn('chat_mode', (await rt.parse('前进',rt.generation))['reason'])
        finally:
            await rt.close()

    async def test_page_reports_active_motion_and_keeps_chat_state_separate(self):
        self.controls()
        self.rt.motion=types.SimpleNamespace(snapshot=lambda:{'busy':True})
        self.assertIn('计时中',self.ai.lines()[3])
        self.controls(mode='chat',chat=True,selected=2)
        self.ai.state='chat_thinking'
        self.assertIn('聊天思考中',self.ai.lines()[3])

    async def test_real_utterance_finalizes_capture_before_asr_and_navigation(self):
        events=[];capturing=asyncio.Event()
        async def cue(name):events.append('cue_'+name)
        async def capture(device,path,released,**kwargs):
            events.append('record');capturing.set();await released.wait()
            path.write_bytes(b'finalized wav');events.append('wav_finalized');return {'duration_s':1.}
        async def asr(request,**kwargs):
            self.assertEqual(Path(request['wav']).read_bytes(),b'finalized wav')
            events.append('asr');return {'result':{'status':'recognized','text':'前进2秒'}}
        async def parse(text,generation,plan_only):events.append('navigate');return {}
        self.rt.announcements=types.SimpleNamespace(_clear=lambda x:None,microphone_blocked=False,
            begin_capture=lambda:events.append('begin'),end_capture=lambda:events.append('end'))
        self.rt.cue=types.SimpleNamespace(play=cue)
        self.rt.workers={'asr':types.SimpleNamespace(call=asr)}
        self.rt.parse=parse;self.rt.generation=0
        self.ai._utterance=AiModule._utterance.__get__(self.ai)
        with patch('ai_coordinator.ai_module.capture',capture):
            self.controls();self.controls(record=True)
            await asyncio.wait_for(capturing.wait(),.5)
            self.assertNotIn('asr',events)
            self.controls(record=False)
            await asyncio.wait_for(self.ai.work,.5)
        self.assertEqual(events,['begin','cue_start','record','wav_finalized','end','cue_stop','asr','navigate'])
        self.assertIn('ai_recording_started',self.stops)
        self.assertFalse(list((self.rt.run_dir/'ai_audio').glob('*.wav')))

    async def test_reconnect_is_bounded_and_does_not_resume_tasks(self):
        root=Path(self.directory.name)/'reconnect';root.mkdir()
        config=root/'config.json'
        config.write_text(json.dumps({'root':str(root),'mode':'shadow','models':{'llm':{}},
            'gateway':{'mode':'shadow'},'ai_module':{'enabled':True}}))
        rt=Runtime(config)
        try:
            rt.cfg['gateway']['mode']='serial'
            with patch.object(rt.gateway,'start',side_effect=[OSError('unplugged'),None]) as opened:
                self.assertFalse(rt.reconnect_ai_link(1.))
                self.assertFalse(rt.reconnect_ai_link(1.5))
                self.assertEqual(opened.call_count,1)
                self.assertTrue(rt.reconnect_ai_link(2.01))
                self.assertFalse(rt.ai.active)
                self.assertIsNone(rt.ai.work)
        finally:await rt.close()


if __name__ == '__main__':unittest.main()
