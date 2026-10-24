import asyncio
from pathlib import Path
import sys
import tempfile
import unittest
import wave
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from ai_coordinator.recording_cue import CUES, RecordingCue, duration, write_cue

DEVICE = 'plughw:CARD=Device,DEV=0'


class _Process:
    def __init__(self, code=0, hang=False):
        self.code = code
        self.hang = hang
        self.returncode = None
        self.terminated = False
        self.killed = False

    async def wait(self):
        if self.terminated or self.killed:
            return self.returncode
        if self.hang:
            await asyncio.sleep(30)
        self.returncode = self.code
        return self.code

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def kill(self):
        self.killed = True
        self.returncode = -9


class CueAssetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def read(self, path):
        with wave.open(str(path), 'rb') as wav:
            frames = wav.readframes(wav.getnframes())
            return wav.getparams(), frames

    def test_written_cues_are_pcm16_mono_44k_with_expected_length(self):
        for name, segments in CUES.items():
            path = write_cue(self.root / (name + '.wav'), segments)
            params, frames = self.read(path)
            self.assertEqual(params.nchannels, 1)
            self.assertEqual(params.sampwidth, 2)
            self.assertEqual(params.framerate, 44100)
            self.assertEqual(params.comptype, 'NONE')
            expected = duration(segments)
            self.assertAlmostEqual(len(frames) / 2 / 44100, expected, places=3)
            self.assertGreater(len(frames), 0)

    def test_cue_is_audible_tone_flanked_by_silence_and_no_temporary_left(self):
        path = write_cue(self.root / 'start.wav', CUES['start'])
        _, frames = self.read(path)
        samples = [int.from_bytes(frames[i:i + 2], 'little', signed=True)
                   for i in range(0, len(frames), 2)]
        self.assertGreater(max(samples), 10000)
        self.assertLess(max(samples), 32767)
        self.assertGreater(max(abs(value) for value in samples[:len(samples) // 2]), 10000)
        quiet = samples[int(.11 * 44100):int(.14 * 44100)]
        self.assertTrue(all(abs(value) < 200 for value in quiet), 'gap between tones must stay silent')
        self.assertFalse((self.root / 'start.wav.tmp').exists())

    def test_start_and_stop_are_distinct_and_bounded(self):
        self.assertNotEqual(CUES['start'], CUES['stop'])
        for name, segments in CUES.items():
            self.assertTrue(.15 <= duration(segments) <= .8, name)


class CuePlaybackTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.log = []

    async def asyncTearDown(self):
        self.temp.cleanup()

    def cue(self, **overrides):
        cfg = {'enabled': True, 'device': DEVICE}
        cfg.update(overrides)
        return RecordingCue(cfg, self.log.append, self.root)

    async def test_disabled_cue_writes_nothing_and_plays_nothing(self):
        cue = RecordingCue({}, self.log.append, self.root)
        self.assertFalse(cue.enabled)
        self.assertFalse(await cue.play('start'))
        self.assertFalse(any((self.root / 'cues').glob('*.wav')) if (self.root / 'cues').exists() else False)
        self.assertFalse(cue.snapshot()['enabled'])

    async def test_explicit_alsa_device_is_mandatory(self):
        for device in (None, '', 'default', 'hw:0', 'plughw:CARD=X'):
            with self.assertRaises(ValueError):
                self.cue(device=device)

    async def test_play_uses_aplay_with_the_configured_card(self):
        cue = self.cue()
        created = []

        async def fake(*args, **kwargs):
            created.append((args, kwargs))
            return _Process(0)

        with patch('ai_coordinator.recording_cue.asyncio.create_subprocess_exec', side_effect=fake):
            self.assertTrue(await cue.play('start'))
        args = created[0][0]
        self.assertEqual(args[:4], ('aplay', '-q', '-D', DEVICE))
        self.assertEqual(Path(args[4]), cue.paths['start'])
        self.assertEqual(cue.plays, 1)
        self.assertEqual([item['type'] for item in self.log], ['recording_cue_played'])

    async def test_playback_failure_is_recorded_and_never_raises(self):
        cue = self.cue()

        async def fake(*args, **kwargs):
            return _Process(1)

        with patch('ai_coordinator.recording_cue.asyncio.create_subprocess_exec', side_effect=fake):
            self.assertFalse(await cue.play('stop'))
        self.assertIn('exit 1', cue.last_error)
        self.assertEqual(cue.plays, 0)
        self.assertEqual(self.log[-1]['type'], 'recording_cue_failed')

    async def test_missing_sound_card_does_not_block_the_capture_window(self):
        cue = self.cue()

        async def fake(*args, **kwargs):
            raise FileNotFoundError('aplay')

        with patch('ai_coordinator.recording_cue.asyncio.create_subprocess_exec', side_effect=fake):
            self.assertFalse(await cue.play('start'))
        self.assertIn('FileNotFoundError', cue.last_error)

    async def test_hung_playback_is_terminated_inside_the_bounded_window(self):
        cue = self.cue()
        process = _Process(0, hang=True)

        async def fake(*args, **kwargs):
            return process

        real_wait_for = asyncio.wait_for
        state = {'first': True}

        async def fake_wait_for(coroutine, timeout):
            if state['first']:
                state['first'] = False
                coroutine.close()
                raise asyncio.TimeoutError
            return await real_wait_for(coroutine, timeout)

        with patch('ai_coordinator.recording_cue.asyncio.create_subprocess_exec', side_effect=fake), \
             patch('ai_coordinator.recording_cue.asyncio.wait_for', side_effect=fake_wait_for):
            self.assertFalse(await cue.play('start'))
        self.assertTrue(process.terminated)

    async def test_cue_directory_defaults_under_the_deployment_root(self):
        cue = self.cue()
        self.assertEqual(cue.directory, self.root / 'cues')
        self.assertTrue((self.root / 'cues' / 'start.wav').is_file())
        snapshot = cue.snapshot()
        self.assertTrue(snapshot['enabled'])
        self.assertEqual(snapshot['device'], DEVICE)


if __name__ == '__main__':
    unittest.main()
