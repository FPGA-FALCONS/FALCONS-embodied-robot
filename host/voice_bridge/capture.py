"""Linux ALSA and Windows WinMM capture, without adding audio dependencies."""
import ctypes
from ctypes import wintypes
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time


def backend_name(name='auto'):
    if name == 'auto':
        name = 'winmm' if sys.platform == 'win32' else 'alsa'
    if name not in ('winmm', 'alsa'):
        raise ValueError('backend must be auto, winmm or alsa')
    return name


def _winmm():
    if sys.platform != 'win32':
        raise RuntimeError('WinMM capture is available on Windows only')
    return ctypes.WinDLL('winmm')


class Caps(ctypes.Structure):
    _fields_ = [('mid', ctypes.c_ushort), ('pid', ctypes.c_ushort), ('version', ctypes.c_uint32),
                ('name', ctypes.c_wchar * 32), ('formats', ctypes.c_uint32),
                ('channels', ctypes.c_ushort), ('reserved', ctypes.c_ushort)]


class Format(ctypes.Structure):
    _fields_ = [('tag', ctypes.c_ushort), ('channels', ctypes.c_ushort), ('rate', ctypes.c_uint32),
                ('bytes_sec', ctypes.c_uint32), ('block_align', ctypes.c_ushort),
                ('bits', ctypes.c_ushort), ('extra', ctypes.c_ushort)]


class Header(ctypes.Structure):
    _fields_ = [('data', ctypes.c_void_p), ('length', ctypes.c_uint32), ('recorded', ctypes.c_uint32),
                ('user', ctypes.c_size_t), ('flags', ctypes.c_uint32), ('loops', ctypes.c_uint32),
                ('next', ctypes.c_void_p), ('reserved', ctypes.c_size_t)]


def _bind(api):
    api.waveInOpen.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint32, ctypes.POINTER(Format),
                              ctypes.c_size_t, ctypes.c_size_t, ctypes.c_uint32]
    api.waveInGetDevCapsW.argtypes = [ctypes.c_size_t, ctypes.POINTER(Caps), ctypes.c_uint32]
    for name in ('waveInPrepareHeader', 'waveInUnprepareHeader', 'waveInAddBuffer'):
        getattr(api, name).argtypes = [ctypes.c_void_p, ctypes.POINTER(Header), ctypes.c_uint32]
    for name in ('waveInStart', 'waveInStop', 'waveInReset', 'waveInClose'):
        getattr(api, name).argtypes = [ctypes.c_void_p]


def _check(code, operation):
    if code:
        raise RuntimeError(f'{operation} failed (WinMM {code}); check device, privacy permission and USB connection')


def devices(backend='auto'):
    backend = backend_name(backend)
    if backend == 'winmm':
        api = _winmm(); _bind(api); result = []
        for index in range(api.waveInGetNumDevs()):
            caps = Caps(); _check(api.waveInGetDevCapsW(index, ctypes.byref(caps), ctypes.sizeof(caps)), 'query device')
            result.append({'id': index, 'name': caps.name, 'max_channels': caps.channels})
        return {'backend': backend, 'inputs': result}
    executable = shutil.which('arecord')
    if not executable:
        return {'backend': backend, 'available': False, 'inputs': [], 'error': 'arecord not installed; no environment changes made'}
    outputs = {}
    for flag, label in [('-l', 'hardware'), ('-L', 'pcm_names')]:
        proc = subprocess.run([executable, flag], capture_output=True, timeout=5)
        outputs[label] = proc.stdout.decode('utf-8', 'replace')
        if proc.returncode:
            outputs[label+'_error'] = proc.stderr.decode('utf-8', 'replace')
    return dict(backend=backend, available=True, **outputs)


def capture_alsa(device, duration, rate, channels):
    try:
        from .audio import Audio, read_wav
    except ImportError:
        from audio import Audio, read_wav
    executable = shutil.which('arecord')
    if not executable:
        raise RuntimeError('arecord unavailable; run devices/doctor in the existing board environment first')
    if rate is None:
        rate = 16000  # plughw can convert; raw hw may require explicit native rate/channels
    with tempfile.TemporaryDirectory(prefix='falcons-mic-') as temporary:
        path = Path(temporary)/'capture.wav'
        command = [executable, '-q', '-D', str(device), '-t', 'wav', '-f', 'S16_LE',
                   '-r', str(rate), '-c', str(channels), '-d', str(math.ceil(duration)), str(path)]
        try:
            proc = subprocess.run(command, capture_output=True, timeout=duration+8)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError('ALSA capture deadline exceeded; recording process stopped') from exc
        if proc.returncode:
            raise RuntimeError('ALSA capture failed: '+proc.stderr.decode('utf-8', 'replace')[:600])
        audio = read_wav(path)
        if audio.duration < duration-.1:
            raise RuntimeError('short ALSA recording; possible USB disconnect')
        audio = Audio(audio.sample_rate, audio.samples[:round(duration*audio.sample_rate)])
    return audio, {'backend':'alsa', 'device':str(device), 'requested_rate':rate,
                   'channels':channels, 'actual_rate':audio.sample_rate}


def capture_winmm(device, duration, rate, channels):
    import numpy as np
    try:
        from .audio import Audio
    except ImportError:
        from audio import Audio
    api = _winmm(); _bind(api)
    device_id = 0xFFFFFFFF if str(device) == 'default' else int(device)
    if device_id != 0xFFFFFFFF and not 0 <= device_id < api.waveInGetNumDevs():
        raise ValueError('invalid microphone device id; run devices first')
    candidates = [rate] if rate else [16000, 48000, 44100]
    chosen = None
    for candidate in candidates:
        fmt = Format(1, channels, candidate, candidate*channels*2, channels*2, 16, 0)
        if api.waveInOpen(None, device_id, ctypes.byref(fmt), 0, 0, 1) == 0:  # WAVE_FORMAT_QUERY
            chosen = candidate; break
    if chosen is None:
        raise RuntimeError('device does not support requested PCM format; try --rate 48000 --channels 2')
    handle = ctypes.c_void_p(); prepared = False; opened = False
    buffer = ctypes.create_string_buffer(round(duration*chosen)*channels*2)
    header = Header(ctypes.cast(buffer, ctypes.c_void_p), len(buffer), 0, 0, 0, 0, None, 0)
    try:
        _check(api.waveInOpen(ctypes.byref(handle), device_id, ctypes.byref(fmt), 0, 0, 0), 'open microphone'); opened = True
        _check(api.waveInPrepareHeader(handle, ctypes.byref(header), ctypes.sizeof(header)), 'prepare microphone'); prepared = True
        _check(api.waveInAddBuffer(handle, ctypes.byref(header), ctypes.sizeof(header)), 'queue capture buffer')
        _check(api.waveInStart(handle), 'start microphone')
        deadline = time.monotonic()+duration+.5
        while not header.flags & 1 and time.monotonic() < deadline:  # WHDR_DONE
            time.sleep(.02)
        _check(api.waveInStop(handle), 'stop microphone')
        _check(api.waveInReset(handle), 'flush microphone')
        data = buffer.raw[:header.recorded]
        if len(data) < round(duration*chosen)*channels*2*.8:
            raise RuntimeError('short microphone recording; check USB connection and privacy permission')
    finally:
        if opened:
            api.waveInStop(handle); api.waveInReset(handle)
            if prepared:
                api.waveInUnprepareHeader(handle, ctypes.byref(header), ctypes.sizeof(header))
            api.waveInClose(handle)
    values = np.frombuffer(data, dtype='<i2').astype(np.float32).reshape(-1, channels)/32768
    return Audio(chosen, values), {'backend':'winmm', 'device':str(device), 'actual_rate':chosen, 'channels':channels}


def record(device='default', duration=5, rate=None, channels=1, backend='auto'):
    if not math.isfinite(duration) or not .2 <= duration <= 30:
        raise ValueError('recording duration must be .2..30 seconds')
    if type(channels) is not int or not 1 <= channels <= 8:
        raise ValueError('channels must be 1..8')
    if rate is not None and (type(rate) is not int or not 8000 <= rate <= 192000):
        raise ValueError('sample rate must be 8000..192000')
    name = backend_name(backend)
    return (capture_winmm if name == 'winmm' else capture_alsa)(device, duration, rate, channels)
