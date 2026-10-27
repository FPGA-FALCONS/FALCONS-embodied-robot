import asyncio
from pathlib import Path
import tempfile
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,patch
import wave

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

from ai_coordinator.inspection.discovery_speech import DiscoverySpeech
from ai_coordinator.inspection.objects import Objects


class DiscoverySpeechTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.run=self.root/'runs'/'test';self.run.mkdir(parents=True)
        self.events=[];self.logs=[];self.calls=[];self.cancelled=[];self.valid=True
        async def synth(request,timeout):
            self.assertTrue(Path(request['output']).is_relative_to(self.run))
            self.calls.append(request)
            with wave.open(request['output'],'wb') as out:
                out.setparams((1,2,24000,0,'NONE','not compressed'));out.writeframes(b'\0\0'*2400)
            return {'result':{}}
        self.rt=SimpleNamespace(cfg={'root':str(self.root)},run_dir=self.run,audit=self.logs.append,
            ai=SimpleNamespace(tts_worker=SimpleNamespace(call=AsyncMock(side_effect=synth))),
            announcements=SimpleNamespace(emit_clip=lambda *args:self.events.append(args),
                cancel_task=lambda *args:self.cancelled.append(args)))
        self.speech=DiscoverySpeech(self.rt,lambda binding:self.valid and binding[1]>0)
        self.binding=(123,1,2)
        self.device=patch('ai_coordinator.device_roles.audio_device',return_value='plughw:CARD=USB,DEV=0')
        self.resolve=self.device.start()

    async def asyncTearDown(self):
        await self.speech.close();self.device.stop();self.tmp.cleanup()

    async def drain(self):
        if self.speech.pump:await self.speech.pump

    def notify(self,identifier=1,**fields):
        self.speech.notify(self.binding,[dict(id=identifier,**{'class':'tape','confirmed':True,**fields})])

    async def test_real_three_frame_confirmation_once_per_object_through_return_and_new_task(self):
        objects=Objects()
        located=dict(**{'class':'tape'},xy_m=[1,0],radius_m=.1,xy_error_m=.02)
        for seq in range(1,7):
            objects.observe([located],seq,2,seq*.1)
            self.speech.notify(self.binding,objects.snapshot());await self.drain()
            self.assertEqual(len(self.events),int(seq>=3))
        self.assertEqual(self.events[0][2],'发现胶带')
        # Same class at a different location is a distinct confirmed object.
        for seq in range(7,10):
            objects.observe([{**located,'xy_m':[2,0]}],seq,2,seq*.1)
            self.speech.notify(self.binding,objects.snapshot());await self.drain()
        self.assertEqual(len(self.events),2);self.assertEqual(len(self.calls),1) # cached synthesis
        self.binding=(123,2,2);self.notify();await self.drain()
        self.assertEqual(len(self.events),3);self.assertEqual(len(self.calls),1)

    async def test_takeover_during_synthesis_discards_playback_without_canceling_shared_model(self):
        started=asyncio.Event();release=asyncio.Event();real=self.rt.ai.tts_worker.call.side_effect
        async def delayed(*args,**kw):started.set();await release.wait();return await real(*args,**kw)
        self.rt.ai.tts_worker.call.side_effect=delayed
        self.notify();await started.wait();self.valid=False;self.speech.cancel('manual')
        release.set();await self.drain();self.assertEqual(self.events,[])
        self.valid=True;self.notify();await self.drain();self.assertEqual(self.events,[])
        self.notify(2);await self.drain();self.assertEqual(len(self.events),1)

    async def test_missing_speaker_skips_once_without_synthesis_or_replug_replay(self):
        self.resolve.side_effect=ValueError('speaker unplugged')
        self.notify();await self.drain();self.assertEqual(self.calls,[])
        self.assertIn('speaker unplugged',self.speech.last_error)
        self.resolve.side_effect=None;self.notify();await self.drain();self.assertEqual(self.events,[])
        self.notify(2);await self.drain();self.assertEqual(len(self.events),1)

    async def test_preview_unknown_class_and_unconfirmed_objects_do_not_speak(self):
        self.binding=(123,0,2);self.notify();await self.drain()
        self.binding=(123,1,2);self.notify(confirmed=False);self.notify(**{'class':'unknown'})
        await self.drain();self.assertEqual(self.calls,[]);self.assertEqual(self.events,[])

    async def test_synthesis_failure_is_bounded_and_does_not_retry_same_object(self):
        self.rt.ai.tts_worker.call.side_effect=RuntimeError('TTS unavailable')
        self.notify();await self.drain();self.notify();await self.drain()
        self.assertEqual(self.rt.ai.tts_worker.call.await_count,1)
        self.assertIn('TTS unavailable',self.speech.last_error);self.assertEqual(self.events,[])
        self.assertFalse(list(self.root.rglob('*.pending.wav')))

    async def test_expired_and_canceled_items_do_not_play(self):
        self.notify();self.speech.queue[0]['at_s']-=11;await self.drain();self.assertEqual(self.events,[])
        self.notify(2);self.speech.cancel('ai_exit');await self.drain();self.assertEqual(self.events,[])

    async def test_custom_class_name_and_disabled_option(self):
        self.rt.cfg['inspection']={'discovery_speech':{'names':{'scissors':'剪刀'}}}
        await self.speech.close();self.speech=DiscoverySpeech(self.rt,lambda b:True)
        self.notify(**{'class':'scissors'});await self.drain();self.assertEqual(self.events[0][2],'发现剪刀')
        await self.speech.close();self.rt.cfg['inspection']['discovery_speech']['enabled']=False
        self.speech=DiscoverySpeech(self.rt,lambda b:True);self.notify(2);await self.drain()
        self.assertEqual(len(self.events),1)
