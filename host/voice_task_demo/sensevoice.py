"""Optional CPU SenseVoice worker; uses existing sherpa-onnx, no package changes."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'voice_bridge'))


def bounded_decode(wav,model_dir,timeout,threads=2,language='zh'):
    command=[sys.executable,str(Path(__file__).resolve()),'--wav',str(wav),
             '--model-dir',str(model_dir),'--threads',str(threads),
             '--language',str(language)]
    try:
        proc=subprocess.run(command,capture_output=True,timeout=timeout,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError('SenseVoice deadline exceeded; private worker stopped') from exc
    if proc.returncode:raise RuntimeError(proc.stderr.decode('utf-8','replace')[-700:])
    return json.loads(proc.stdout.decode('utf-8'))['text']


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--wav',type=Path,required=True)
    ap.add_argument('--model-dir',type=Path,required=True);ap.add_argument('--threads',type=int,default=2)
    ap.add_argument('--language',default='zh',help='zh/en/ja/ko/yue/auto')
    args=ap.parse_args()
    if not 1<=args.threads<=8:raise ValueError('threads must be 1..8')
    import sherpa_onnx
    from audio import read_wav
    audio=read_wav(args.wav)
    if audio.sample_rate!=16000 or audio.channels!=1:raise ValueError('Expected mono 16kHz PCM')
    recognizer=sherpa_onnx.OfflineRecognizer.from_sense_voice(
        model=str(args.model_dir/'model.int8.onnx'),tokens=str(args.model_dir/'tokens.txt'),
        num_threads=args.threads,provider='cpu',language=args.language,use_itn=True)
    stream=recognizer.create_stream();stream.accept_waveform(16000,audio.samples[:,0])
    recognizer.decode_stream(stream)
    # Language/emotion tags, when provided, are metadata rather than command words.
    import re
    text=re.sub(r'<\|[^|]*\|>','',stream.result.text).strip()
    sys.stdout.buffer.write(json.dumps({'text':text},ensure_ascii=False).encode('utf-8'))


if __name__=='__main__':main()
