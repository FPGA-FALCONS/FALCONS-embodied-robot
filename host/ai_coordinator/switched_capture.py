"""ALSA switch-to-record: SWB up ends the WAV before inference starts."""
import asyncio
from pathlib import Path
import re
import signal
import wave
import math


async def capture(device, path, released, maximum_s=30, rate=16000, channels=1):
    if not isinstance(device, str) or not re.fullmatch(r'(?:plug)?hw:CARD=[A-Za-z0-9_]+,DEV=\d+', device):
        raise ValueError('Recording requires an explicit ALSA device')
    if not .2 <= maximum_s <= 30 or rate != 16000 or channels != 1:
        raise ValueError('Expected bounded mono 16kHz capture')
    path = Path(path)
    proc = None
    waiter = None
    try:
        if released.is_set():
            raise RuntimeError('Recording released before capture started')
        proc = await asyncio.create_subprocess_exec(
            'arecord', '-q', '-D', device, '-t', 'wav', '-f', 'S16_LE',
            '-r', str(rate), '-c', str(channels), '-d', str(math.ceil(maximum_s)), str(path),
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE)
        waiter = asyncio.create_task(released.wait())
        exited = asyncio.create_task(proc.wait())
        try:
            await asyncio.wait((waiter, exited), timeout=maximum_s + .5,
                               return_when=asyncio.FIRST_COMPLETED)
            if proc.returncode is None:
                proc.send_signal(signal.SIGINT)  # arecord finalizes the WAV header.
            try:
                await asyncio.wait_for(exited, 2)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
                raise RuntimeError('Microphone did not stop after SWB release')
        finally:
            if not exited.done():
                exited.cancel()
                await asyncio.gather(exited, return_exceptions=True)
        with wave.open(str(path), 'rb') as wav:
            duration = wav.getnframes() / wav.getframerate()
            if wav.getparams()[:3] != (1, 2, rate) or not .2 <= duration <= maximum_s + .5:
                raise ValueError('Short or invalid microphone recording')
        return {'duration_s': duration, 'source': 'microphone', 'device': device}
    finally:
        if waiter:
            waiter.cancel()
            await asyncio.gather(waiter, return_exceptions=True)
        if proc and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), .5)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
