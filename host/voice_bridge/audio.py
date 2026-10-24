"""PCM input, channel selection, anti-aliased resampling and quality checks."""
from dataclasses import dataclass
from math import gcd
from pathlib import Path
import wave
import numpy as np


@dataclass
class Audio:
    sample_rate: int
    samples: np.ndarray  # float32, frames x channels; never implicitly mix AEC/reference channels

    def __post_init__(self):
        self.samples = np.asarray(self.samples, dtype=np.float32)
        if self.samples.ndim == 1:
            self.samples = self.samples[:, None]
        if self.sample_rate < 8000 or self.sample_rate > 192000:
            raise ValueError('sample rate must be between 8000 and 192000 Hz')
        if self.samples.ndim != 2 or not 1 <= self.samples.shape[1] <= 8 or not len(self.samples):
            raise ValueError('audio must contain frames and 1..8 channels')
        if not np.isfinite(self.samples).all():
            raise ValueError('non-finite audio samples')
        if len(self.samples) / self.sample_rate > 60:
            raise ValueError('audio exceeds 60 seconds')

    @property
    def duration(self):
        return len(self.samples) / self.sample_rate

    @property
    def channels(self):
        return self.samples.shape[1]


def read_wav(path):
    path = Path(path)
    if path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError('WAV exceeds 64 MiB')
    with wave.open(str(path), 'rb') as f:
        rate, channels, frames = f.getframerate(), f.getnchannels(), f.getnframes()
        if f.getsampwidth() != 2 or f.getcomptype() != 'NONE':
            raise ValueError('use uncompressed 16-bit PCM WAV')
        if channels < 1 or channels > 8 or frames < 1 or frames / rate > 60:
            raise ValueError('invalid channel count or duration')
        data = f.readframes(frames)
        if len(data) != frames * channels * 2:
            raise ValueError('truncated WAV payload')
    values = np.frombuffer(data, dtype='<i2').astype(np.float32).reshape(-1, channels) / 32768
    return Audio(rate, values)


def write_wav(path, audio):
    path = Path(path)
    if path.exists():
        raise FileExistsError(str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    values = np.rint(np.clip(audio.samples, -1, 32767/32768) * 32768).astype('<i2')
    # Exclusive creation prevents accidental overwrite of an earlier recording.
    with path.open('xb') as output:
        with wave.open(output, 'wb') as f:
            f.setnchannels(audio.channels); f.setsampwidth(2); f.setframerate(audio.sample_rate)
            f.writeframes(values.tobytes())


def mono16k(audio, channel=0):
    if channel < 0 or channel >= audio.channels:
        raise ValueError(f'channel {channel} is unavailable; input has {audio.channels}')
    values = audio.samples[:, channel]
    if audio.sample_rate != 16000:
        try:
            from scipy.signal import resample_poly
        except ImportError as exc:
            raise RuntimeError('resampling needs scipy in the separate voice environment') from exc
        divisor = gcd(audio.sample_rate, 16000)
        values = resample_poly(values, 16000 // divisor, audio.sample_rate // divisor)
    return Audio(16000, values)


def quality(audio, silence_dbfs=-55):
    values = audio.samples.astype(np.float64)
    rms = float(np.sqrt(np.mean(values * values)))
    dbfs = float(20 * np.log10(max(rms, 1e-12)))
    return {'duration_s': round(audio.duration, 4), 'sample_rate': audio.sample_rate,
            'channels': audio.channels, 'rms_dbfs': round(dbfs, 2),
            'peak': float(np.max(np.abs(values))),
            'clipping_fraction': float(np.mean(np.abs(values) >= 32760/32768)),
            'below_energy_threshold': dbfs < silence_dbfs,
            'note': 'Energy check only; motor noise is not speech detection.'}
