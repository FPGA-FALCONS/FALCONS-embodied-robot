import asyncio
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import AsyncMock, patch
import wave

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.startup import StartupModels
from ai_coordinator.install_autostart import prepare, unit_text, checked_manifest


class FakeWorker:
    def __init__(self, name, order):
        self.name=name;self.order=order;self.proc=None;self.load_s=.01
        self.failure=False;self.gate=None

    async def call(self, request, timeout=1):
        assert request=={'op':'health'}
        self.order.append(self.name)
        if self.gate:await self.gate.wait()
        if self.failure:raise RuntimeError('test model failure')
        self.proc=types.SimpleNamespace(returncode=None)
        return {'result':{'health':'ready'}}


class StartupTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp=tempfile.TemporaryDirectory();self.order=[];self.audit=[]
        self.rt=types.SimpleNamespace(cfg={'root':self.temp.name,'model_loads_enabled':True,
            'startup':{'enabled':True,'retry_s':.01,'check_interval_s':.01},
            'models':{'asr':{},'llm':{},'chat':{},'tts':{},'vision':{}}},
            ai=types.SimpleNamespace(active=False,work=None),audit=self.audit.append,
            model_server=types.SimpleNamespace(proc=None))
        self.rt.workers={n:FakeWorker(n,self.order) for n in ('asr','llm','vision')}
        self.rt.ai.chat_worker=FakeWorker('chat',self.order)
        self.rt.ai.tts_worker=FakeWorker('tts',self.order)
        async def server():pass
        self.rt._ensure_server=server
        self.startup=StartupModels(self.rt)
        self.tone=AsyncMock(side_effect=lambda:self.order.append('tone'))
        self.startup.tone=self.tone

    async def asyncTearDown(self):
        await self.startup.close();self.temp.cleanup()

    async def wait_for(self,predicate):
        for _ in range(200):
            if predicate():return
            await asyncio.sleep(.005)
        self.fail('startup did not reach expected state')

    async def test_all_required_models_ready_before_single_tone_and_reload(self):
        self.startup.start()
        await self.wait_for(lambda:self.startup.ready)
        self.assertEqual(self.order,['asr','llm','chat','tts','vision','tone'])
        self.rt.workers['asr'].proc=None
        await self.wait_for(lambda:self.order.count('asr')==2)
        self.assertEqual(self.tone.await_count,1)
        self.assertTrue(self.startup.cue_played)

    async def test_required_failure_blocks_ready_and_tone_until_recovery(self):
        self.rt.ai.chat_worker.failure=True
        self.startup.start()
        await self.wait_for(lambda:'chat' in self.startup.errors)
        self.assertFalse(self.startup.ready);self.tone.assert_not_awaited()
        self.rt.ai.chat_worker.failure=False
        await self.wait_for(lambda:self.startup.ready)
        self.assertNotIn('chat',self.startup.errors)
        self.tone.assert_awaited_once()

    async def test_optional_vision_failure_and_absent_camera_do_not_block_voice_models(self):
        self.rt.workers['vision'].failure=True
        self.startup.start()
        await self.wait_for(lambda:self.startup.ready)
        self.assertIn('vision',self.startup.errors)
        self.tone.assert_awaited_once()

    async def test_failed_speaker_retries_without_unloading_or_repeating_successful_tone(self):
        self.startup.tone=AsyncMock(side_effect=[RuntimeError('unplugged'),None])
        self.startup.start()
        await self.wait_for(lambda:self.startup.ready)
        self.assertFalse(self.startup.cue_played)
        await self.wait_for(lambda:self.startup.cue_played)
        await asyncio.sleep(.025)
        self.assertEqual(self.startup.tone.await_count,2)
        self.assertEqual(self.order,['asr','llm','chat','tts','vision'])

    async def test_disabled_model_loads_do_not_load_or_sound(self):
        self.rt.cfg['model_loads_enabled']=False
        self.startup.start()
        await self.wait_for(lambda:bool(self.startup.errors))
        self.assertEqual(self.order,[]);self.tone.assert_not_awaited()
        self.assertFalse(self.startup.ready)

    async def test_close_cancels_a_blocked_health_call(self):
        self.rt.workers['asr'].gate=asyncio.Event()
        self.startup.start()
        await self.wait_for(lambda:self.order==['asr'])
        await self.startup.close()
        self.assertIsNone(self.startup.task);self.tone.assert_not_awaited()

    async def test_ready_tone_is_one_short_pcm_note_on_explicit_speaker(self):
        self.rt.cfg['ai_module']={'speaker_device':'plughw:CARD=Device,DEV=0'}
        played=AsyncMock()
        with patch('ai_coordinator.startup.ALSAPlayer',return_value=played) as factory:
            await StartupModels.tone(self.startup)
        factory.assert_called_once_with('plughw:CARD=Device,DEV=0')
        clip=played.call_args.args[0]
        with wave.open(str(clip['path'])) as wav:
            self.assertEqual(wav.getparams()[:3],(1,2,44100))
            self.assertAlmostEqual(wav.getnframes()/wav.getframerate(),.2)


class InstallerTests(unittest.TestCase):
    def test_release_manifest_rejects_modified_source(self):
        import hashlib
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'worker.py';source.write_bytes(b'checked source')
            entries={'worker.py':{'sha256':hashlib.sha256(source.read_bytes()).hexdigest()}}
            (root/'MANIFEST.json').write_text(json.dumps(entries))
            self.assertEqual(checked_manifest(root)[0],entries)
            source.write_bytes(b'modified source')
            with self.assertRaises(ValueError):checked_manifest(root)

    def test_release_manifest_cannot_reference_an_external_file(self):
        import hashlib
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);release=root/'release';release.mkdir()
            source=root/'outside.py';source.write_bytes(b'outside')
            entries={'../outside.py':{'sha256':hashlib.sha256(source.read_bytes()).hexdigest()}}
            (release/'MANIFEST.json').write_text(json.dumps(entries))
            with self.assertRaises(ValueError):checked_manifest(release)

    def test_prepare_preserves_motion_and_model_assets(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'config.json'
            cfg={'root':temp,'mode':'live','model_loads_enabled':True,'ai_module':{'enabled':True},
                 'voice_motion':{'speed':30,'enabled':True},'camera':{'enabled':False},
                 'models':{n:{'existing_path':n} for n in ('asr','llm','chat','tts')}}
            p.write_text(json.dumps(cfg))
            changed,unit=prepare(p)
            self.assertEqual({k:v for k,v in changed.items() if k!='startup'},cfg)
            self.assertEqual(json.loads(p.read_text()),cfg)
            self.assertTrue(changed['startup']['enabled'])
            self.assertIn('MemoryMax=6G',unit)
            self.assertIn('WantedBy=default.target',unit)
            self.assertNotIn('ssh',unit)

    def test_systemd_paths_are_quoted_and_percent_escaped(self):
        unit=unit_text('/home/user/release x/start.sh','/home/user/100%/config.json')
        self.assertIn('"/home/user/release x/start.sh"',unit)
        self.assertIn('100%%',unit)
        with self.assertRaises(ValueError):unit_text('/a\nExecStart=bad','/config')


if __name__=='__main__':unittest.main()
