"""Local bounded ASR worker -> typed observation event, never a motion command."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import uuid
import numpy as np
try:
    from .audio import read_wav, write_wav, mono16k, quality
    from .preview import task_preview
    from .model import MODEL_FILES, model_status
    from .domain_vocab import command_text
except ImportError:
    from audio import read_wav, write_wav, mono16k, quality
    from preview import task_preview
    from model import MODEL_FILES, model_status
    from domain_vocab import command_text


def decode_local(wav, model_dir, threads=2, hotwords_file=None, hotwords_score=1.5, decoding_method='greedy_search'):
    """CPU only. Called inside a killable worker, not a microphone callback."""
    import sherpa_onnx
    status = model_status(model_dir)
    if not status['ready']:
        raise FileNotFoundError('ASR files missing: '+', '.join(status['missing']))
    files = {key:str(Path(model_dir)/name) for key,name in MODEL_FILES.items()}
    if decoding_method not in ('greedy_search','modified_beam_search'):
        raise ValueError('unsupported ASR decoding method')
    if not math.isfinite(hotwords_score) or not 0 <= hotwords_score <= 5:
        raise ValueError('hotword score must be finite and between 0 and 5')
    if hotwords_file:
        hotwords_file=Path(hotwords_file)
        if not hotwords_file.is_file() or hotwords_file.stat().st_size>65536:
            raise ValueError('hotwords must be an existing UTF-8 file under 64 KiB')
        decoding_method='modified_beam_search'
    rec = sherpa_onnx.OnlineRecognizer.from_transducer(**files, num_threads=threads,
        sample_rate=16000, feature_dim=80, decoding_method=decoding_method, provider='cpu',
        max_active_paths=4, modeling_unit='cjkchar',
        hotwords_file=str(hotwords_file) if hotwords_file else '', hotwords_score=hotwords_score)
    audio = read_wav(wav)
    if audio.sample_rate != 16000 or audio.channels != 1:
        raise ValueError('worker expects mono 16000 Hz PCM')
    stream = rec.create_stream()
    stream.accept_waveform(16000, audio.samples[:,0])
    stream.accept_waveform(16000, np.zeros(4800,dtype=np.float32))
    stream.input_finished()
    while rec.is_ready(stream):
        rec.decode_stream(stream)
    return rec.get_result(stream).strip()


def bounded_decode(wav, model_dir, timeout, threads=2, hotwords_file=None, hotwords_score=1.5, decoding_method='greedy_search'):
    command = [sys.executable, str(Path(__file__).with_name('voice.py')), '_asr-worker',
               '--wav', str(wav), '--model-dir', str(model_dir), '--threads', str(threads)]
    command += ['--decoding-method',decoding_method,'--hotwords-score',str(hotwords_score)]
    if hotwords_file:
        command += ['--hotwords-file',str(hotwords_file)]
    try:
        proc = subprocess.run(command,capture_output=True,timeout=timeout,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    except subprocess.TimeoutExpired as exc:
        # subprocess.run kills and waits for this private child on timeout.
        raise TimeoutError('ASR deadline exceeded; private worker stopped') from exc
    if proc.returncode:
        raise RuntimeError(proc.stderr.decode('utf-8','replace')[-700:])
    return json.loads(proc.stdout.decode('utf-8'))['text']


def process_wav(wav, model_dir, out_dir, channel=0, silence_dbfs=-55,
                asr_timeout=15, threads=2, llm_url=None, llm_timeout=5, decoder=None, source=None,
                hotwords_file=None, hotwords_score=1.5, decoding_method='greedy_search', domain_normalization=True):
    if not 0 < asr_timeout <= 120 or not 1 <= threads <= 8:
        raise ValueError('ASR timeout must be 0..120 seconds and threads 1..8')
    out = Path(out_dir); out.mkdir(parents=True,exist_ok=True)
    if (out/'result.json').exists() or (out/'asr16k.wav').exists():
        raise FileExistsError('output already contains results; select a fresh session directory')
    event = {'schema_version':1, 'type':'voice_result', 'session_id':uuid.uuid4().hex,
             'text':'', 'task_text':None, 'status':'pending', 'source':{'wav':str(Path(wav).resolve()), **(source or {})},
             'task_preview':None, 'requires_confirmation':True, 'execution_allowed':False,
             'motion_authorized':False, 'observation_only':True}
    start = time.monotonic()
    try:
        original = read_wav(wav); audio = mono16k(original,channel)
        event['source'].update(sample_rate=original.sample_rate, channels=original.channels, selected_channel=channel)
        event['quality'] = quality(audio,silence_dbfs)
        if event['quality']['below_energy_threshold']:
            event['status'] = 'no_speech_energy'
        else:
            normalized = out/'asr16k.wav'; write_wav(normalized,audio)
            decode_start = time.monotonic()
            function = decoder or bounded_decode
            if decoder is None:
                text = bounded_decode(normalized,model_dir,asr_timeout,threads,
                    hotwords_file,hotwords_score,decoding_method).strip()
            else:
                text = function(normalized,model_dir,asr_timeout,threads).strip()
            event['asr'] = {'engine':'sherpa-onnx', 'provider':'cpu', 'model_dir':str(Path(model_dir).resolve()),
                'elapsed_s':round(time.monotonic()-decode_start,4), 'timeout_s':asr_timeout, 'includes_worker_load':True}
            event['asr'].update(decoding_method='modified_beam_search' if hotwords_file else decoding_method,
                                hotwords_file=str(hotwords_file) if hotwords_file else None,
                                hotwords_score=hotwords_score if hotwords_file else None)
            event['text'] = text; event['status'] = 'recognized' if text else 'empty_transcript'
            if text:
                policy=command_text(text) if domain_normalization else {
                    'raw_text':text,'task_text':text,'blocked':False,'reason':'disabled','corrections':[]}
                event['domain_vocab']=policy;event['task_text']=policy['task_text']
                if policy['blocked']:
                    event['task_preview']={'task':{'i':'unknown'},'source':'domain_policy','reason':policy['reason']}
                else:
                    event['task_preview'] = task_preview(policy['task_text'],llm_url,llm_timeout)
    except Exception as exc:
        event['status'] = 'error'; event['error'] = type(exc).__name__+': '+str(exc)[:800]
    event['processing_s'] = round(time.monotonic()-start,4)
    with (out/'result.json').open('x',encoding='utf-8') as file:
        json.dump(event,file,ensure_ascii=False,indent=2)
    return event
