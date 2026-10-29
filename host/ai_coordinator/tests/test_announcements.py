import asyncio
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import wave
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.announcements import Announcements,ALSAPlayer,catalog,validate_assets
from ai_coordinator.core import Mission

def event(code,task='one',identifier=None):
    return {'id':identifier or task+code,'task_id':task,'goal_id':'goal','code':code,'simulated':True}

class AnnouncementTests(unittest.IsolatedAsyncioTestCase):
    async def test_prepared_clip_completion_waits_for_playback_and_cancel_returns_false(self):
        self.speech.prepared_player=self.speech.player
        clip={'path':self.root/'found_tape.wav','duration_s':.1}
        future=self.speech.emit_clip(event('inspection_found',identifier='prepared-1'),clip,'发现胶带')
        await self.started.wait();self.assertFalse(future.done())
        self.speech.cancel_task('one','manual')
        self.assertFalse(await future);self.release.set();await self.drain()
        self.started.clear()
        future=self.speech.emit_clip(event('inspection_found',identifier='prepared-2'),clip,'发现胶带')
        self.assertTrue(await future);await self.drain()

    async def asyncSetUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.log=[]
        clips={}
        for key,text in catalog().items():
            path=self.root/(key+'.wav')
            with wave.open(str(path),'wb') as wav:
                wav.setparams((1,2,16000,0,'NONE','not compressed'));wav.writeframes(b'\0\0'*1600)
            clips[key]={'file':path.name,'text':text,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        (self.root/'manifest.json').write_text(json.dumps({'v':1,'clips':clips}),encoding='utf-8')
        self.started=asyncio.Event();self.release=asyncio.Event();self.played=[];self.cancelled=[]
        async def player(clip,ev):
            self.played.append(ev);self.started.set()
            try:await self.release.wait()
            except asyncio.CancelledError:self.cancelled.append(ev);raise
        self.speech=Announcements({'mode':'alsa','assets':str(self.root)},self.log.append,player)

    async def asyncTearDown(self):await self.speech.close();self.temp.cleanup()

    async def drain(self):
        if self.speech.pump:await asyncio.wait_for(self.speech.pump,1)

    async def test_stop_preempts_playback_and_discards_old_queue(self):
        self.speech.emit(event('search_tape'));await asyncio.wait_for(self.started.wait(),1)
        self.speech.emit(event('found_tape'));self.speech.emit(event('completed'))
        self.speech.emit(event('stop_requested'));self.release.set();await self.drain()
        self.assertEqual([e['code'] for e in self.played],['search_tape','stop_requested'])
        self.assertEqual(len(self.cancelled),1)

    async def test_new_task_preempts_and_duplicate_is_not_repeated(self):
        self.speech.emit(event('search_tape'));await asyncio.wait_for(self.started.wait(),1)
        self.speech.emit(event('completed'))
        new=event('accepted_screwdriver','two');self.speech.emit(new);self.speech.emit(new)
        self.release.set();await self.drain()
        self.assertEqual([e['code'] for e in self.played],['search_tape','accepted_screwdriver'])

    async def test_capture_and_speech_are_mutually_exclusive(self):
        self.speech.begin_capture();self.speech.emit(event('search_tape'))
        await asyncio.sleep(0);self.assertFalse(self.played)
        self.speech.end_capture();await asyncio.wait_for(self.started.wait(),1)
        with self.assertRaises(RuntimeError):self.speech.begin_capture()
        self.release.set();await self.drain()
        self.assertTrue(self.speech.microphone_blocked)  # acoustic tail guard

    async def test_player_failure_does_not_fail_task_loop(self):
        async def broken(clip,ev):raise OSError('speaker unplugged')
        self.speech.player=broken;self.speech.emit(event('search_tape'));await self.drain()
        self.assertIn('speaker unplugged',self.speech.last_error)
        self.assertFalse(self.speech.snapshot()['speaking'])

    async def test_log_mode_never_opens_audio_and_marks_simulation(self):
        speech=Announcements({'mode':'log'},self.log.append)
        try:
            speech.emit(event('completed'));await speech.pump
            item=next(e for e in self.log if e['type']=='speech_would_play')['event']
            self.assertEqual(item['text'],'模拟任务。任务已完成。')
        finally:await speech.close()

    async def test_catalog_tamper_and_default_device_rejected(self):
        with self.assertRaises(ValueError):ALSAPlayer('default')
        path=self.root/'completed.wav';path.write_bytes(b'corrupted')
        with self.assertRaises(ValueError):validate_assets(self.root)

    async def test_missing_assets_degrades_to_log(self):
        speech=Announcements({'mode':'alsa','assets':str(self.root/'missing'),'device':'plughw:CARD=USB,DEV=0'},self.log.append)
        try:
            self.assertEqual(speech.mode,'log');self.assertIsNotNone(speech.last_error)
            speech.emit(event('completed'));await speech.pump
        finally:await speech.close()

    async def test_stale_queue_is_not_spoken(self):
        self.speech.begin_capture();self.speech.emit(event('completed'))
        self.speech.queue[0]['queued_at_s']-=11;self.speech.end_capture();await self.drain()
        self.assertFalse(self.played)

    async def test_prepared_ai_clip_plays_with_legacy_log_policy_and_is_cancelable(self):
        speech=Announcements({'mode':'log'},self.log.append)
        speech.prepared_player=self.speech.player
        ev={**event('inspection_found'),'simulated':False}
        try:
            speech.emit_clip(ev,self.speech.clips['completed'],'发现胶带')
            await asyncio.wait_for(self.started.wait(),1)
            self.assertTrue(speech.microphone_blocked)
            with self.assertRaises(RuntimeError):speech.begin_capture()
            speech.cancel_task('other','unrelated');self.assertFalse(self.cancelled)
            speech.cancel_task(ev['task_id'],'manual');await speech.pump
            self.assertEqual(len(self.cancelled),1)
            speech.emit_clip(ev,self.speech.clips['completed'],'发现胶带')
            self.assertFalse(speech.queue) # no replay of interrupted object
        finally:await speech.close()

    async def test_prepared_ai_clip_waits_for_capture_and_expires_from_discovery_time(self):
        speech=Announcements({'mode':'log'},self.log.append);speech.prepared_player=self.speech.player
        ev={**event('inspection_found'),'simulated':False}
        try:
            speech.begin_capture();speech.emit_clip(ev,self.speech.clips['completed'],'发现胶带')
            await asyncio.sleep(0);self.assertFalse(self.played)
            speech.queue[0]['discovered_at_s']=__import__('time').monotonic()-11
            speech.end_capture();await speech.pump;self.assertFalse(self.played)
        finally:await speech.close()

    async def test_actual_private_playback_process_is_reaped_on_cancel(self):
        real_exec=asyncio.create_subprocess_exec;created=asyncio.Event();children=[];argv=[]
        async def fake_aplay(*args,**kwargs):
            argv.extend(args)
            proc=await real_exec(sys.executable,'-c','import time; time.sleep(10)',**kwargs)
            children.append(proc);created.set();return proc
        player=ALSAPlayer('plughw:CARD=USB,DEV=0');clip=self.speech.clips['completed']
        with patch('ai_coordinator.announcements.asyncio.create_subprocess_exec',side_effect=fake_aplay):
            pending=asyncio.create_task(player(clip,event('completed')))
            await asyncio.wait_for(created.wait(),2);pending.cancel()
            await asyncio.gather(pending,return_exceptions=True)
        self.assertIsNotNone(children[0].returncode)
        self.assertEqual(argv[:4],['aplay','-q','-D','plughw:CARD=USB,DEV=0'])

class MissionSpeechTests(unittest.TestCase):
    def test_found_and_completion_require_correct_device_receipts(self):
        core=Mission(mode='simulation');core.submit({'i':'search','t':'tape','r':True},1)
        self.assertEqual([e['code'] for e in core.drain_notices()],['accepted_tape'])
        action=core.drain()[0];core.on_status({**action,'kind':'search_started'},1.1)
        self.assertEqual([e['code'] for e in core.drain_notices()],['search_tape'])
        for seq,now in enumerate((1.2,1.3)):
            core.on_frame({'task_id':core.task_id,'goal_id':core.goal_id,'sequence':seq,'captured_at_s':now,
                'width':640,'height':480,'detections':[{'class':'tape','confidence':.9,'bbox':[1,1,20,20]}]},now)
        self.assertEqual(core.drain_notices(),[])  # visual evidence alone cannot say found
        found=core.drain()[0];core.on_status({**found,'kind':'target_accepted'},1.4)
        self.assertEqual([e['code'] for e in core.drain_notices()],['found_tape','returning'])
        home=core.drain()[0];core.on_status({**home,'kind':'home_arrived','task_id':'old'},1.5)
        self.assertEqual(core.drain_notices(),[])
        core.on_status({**home,'kind':'home_arrived'},1.6)
        notices=core.drain_notices();self.assertEqual([e['code'] for e in notices],['completed'])
        self.assertTrue(all(e['simulated'] for e in notices))

    def test_stop_request_never_claims_physical_stop(self):
        core=Mission(mode='live');core.stop(1)
        self.assertEqual(core.drain_notices()[0]['code'],'stop_requested')
