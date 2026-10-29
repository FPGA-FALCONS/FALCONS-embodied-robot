"""Short tones that bracket a microphone capture window.

Audible booking-keeping for the operator only. A cue never authorizes,
completes or verifies motion, and a failed cue never fails a capture.
"""
import asyncio
import math
import re
import struct
import time
import wave
from pathlib import Path

RATE = 44100
GAIN = 0.35
FADE_S = 0.008
# (frequency Hz, duration s) pairs; frequency 0 is silence.
CUES = {
    'start': ((880, .10), (0, .05), (1175, .14)),
    'stop': ((660, .16), (0, .05), (440, .26)),
}
DEVICE = r'(?:plug)?hw:CARD=[A-Za-z0-9_]+,DEV=\d+'


def _frames(frequency, duration, rate=RATE, gain=GAIN):
    count = int(round(duration * rate))
    fade = min(int(round(FADE_S * rate)), max(count // 2, 0))
    for index in range(count):
        envelope = 1.
        if fade > 0 and index < fade:
            envelope = index / fade
        elif fade > 0 and index >= count - fade:
            envelope = max(count - 1 - index, 0) / fade
        value = gain * envelope * math.sin(2 * math.pi * frequency * index / rate) if frequency else 0.
        yield int(max(-1., min(1., value)) * 32767)


def write_cue(path, segments, rate=RATE, gain=GAIN):
    """Write one PCM16 mono cue and return its path. Replaces atomically."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = b''.join(struct.pack('<h', sample)
                    for frequency, duration in segments
                    for sample in _frames(frequency, duration, rate, gain))
    temporary = path.with_name(path.name + '.tmp')
    with wave.open(str(temporary), 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(rate)
        wav.writeframes(data)
    temporary.replace(path)
    return path


def duration(segments):
    return sum(seconds for _, seconds in segments)


class RecordingCue:
    """Plays the start tone before a capture and the stop tone after it."""

    def __init__(self, cfg, audit, root=None, player=None):
        cfg = cfg or {}
        self.audit = audit
        self.enabled = cfg.get('enabled') is True
        self.device = cfg.get('device');self.selector=cfg.get('selector')
        self.directory = Path(cfg.get('directory') or (Path(root or '.') / 'cues'))
        self.last_error = None
        self.plays = 0
        self.paths = {}
        self.lengths = {}
        self.lock = asyncio.Lock()
        if not self.enabled:
            return
        if not isinstance(self.device, str) or not re.fullmatch(DEVICE, self.device):
            raise ValueError('Recording cue needs an explicit ALSA card and device')
        for name, segments in CUES.items():
            self.paths[name] = write_cue(self.directory / (name + '.wav'), segments)
            self.lengths[name] = duration(segments)

    async def play(self, name):
        if not self.enabled:
            return False
        if name not in self.paths:
            raise ValueError('Unknown recording cue')
        async with self.lock:
            try:
                from .device_roles import audio_device
                device=audio_device(self.selector,'speaker',self.device)
                proc = await asyncio.create_subprocess_exec(
                    'aplay', '-q', '-D', device, str(self.paths[name]),
                    stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
                try:
                    code = await asyncio.wait_for(proc.wait(), self.lengths[name] + 2)
                    if code:
                        raise RuntimeError('Cue playback failed (exit ' + str(code) + ')')
                finally:
                    if proc.returncode is None:
                        proc.terminate()
                        try:
                            await asyncio.wait_for(proc.wait(), .3)
                        except asyncio.TimeoutError:
                            proc.kill()
                            await proc.wait()
            except Exception as exc:
                self.last_error = type(exc).__name__ + ': ' + str(exc)[:200]
                self.audit({'type': 'recording_cue_failed', 'cue': name, 'error': self.last_error})
                return False
            self.plays += 1
            self.audit({'type': 'recording_cue_played', 'cue': name, 'at_s': time.monotonic()})
            return True

    def snapshot(self):
        return {'enabled': self.enabled, 'device': self.device, 'directory': str(self.directory),
                'plays': self.plays, 'last_error': self.last_error}
