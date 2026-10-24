"""PC-only Kokoro auditions/cache generation using an existing Sherpa installation.

No downloads or package installation. A cached voice can be played by the board
without loading this model. Keep each output directory as a distinct version.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import wave

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.announcements import catalog,validate_assets

def write_audio(path,samples,rate,target_rate=None):
    import numpy as np
    if target_rate and target_rate!=rate:
        from scipy.signal import resample_poly
        divisor=math.gcd(rate,target_rate)
        samples=resample_poly(samples,target_rate//divisor,rate//divisor);rate=target_rate
    samples=np.asarray(samples,dtype=np.float32)
    if not len(samples) or not np.isfinite(samples).all():raise ValueError('Invalid generated audio')
    pcm=np.rint(np.clip(samples,-1,1)*32767).astype('<i2')
    with wave.open(str(path),'wb') as wav:
        wav.setparams((1,2,rate,0,'NONE','not compressed'));wav.writeframes(pcm.tobytes())
    return {'duration_s':len(pcm)/rate,'sample_rate':rate,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('operation',choices=['audition','cache']);ap.add_argument('--model-dir',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--sid',type=int,action='append',required=True)
    ap.add_argument('--speed',type=float,default=1.0);ap.add_argument('--threads',type=int,default=4)
    ap.add_argument('--text',default='下面是语音试听。你好，我是你的机器人助手。收到，开始寻找胶带。已经找到胶带，正在返回起点。')
    args=ap.parse_args()
    if not .7<=args.speed<=1.3 or not 1<=args.threads<=8:ap.error('Invalid speed or thread limit')
    if args.operation=='cache' and len(args.sid)!=1:ap.error('Cache requires exactly one voice')
    args.out.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import sherpa_onnx
    # Keep an ASCII junction spelling on Windows: native eSpeak uses narrow paths.
    model=args.model_dir.absolute()
    config=sherpa_onnx.OfflineTtsConfig(model=sherpa_onnx.OfflineTtsModelConfig(
        kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(model=str(model/'model.onnx'),
            voices=str(model/'voices.bin'),tokens=str(model/'tokens.txt'),
            data_dir=str(model/'espeak-ng-data'),
            lexicon=','.join(str(model/x) for x in ('lexicon-us-en.txt','lexicon-zh.txt'))),
        num_threads=args.threads,provider='cpu'),max_num_sentences=1)
    if not config.validate():raise ValueError('Kokoro configuration failed validation')
    started=time.monotonic();tts=sherpa_onnx.OfflineTts(config);load=time.monotonic()-started
    if any(sid<0 or sid>=tts.num_speakers for sid in args.sid):raise ValueError('Voice ID outside model range')
    receipt={'engine':'Kokoro multi-lang-v1_1 / Sherpa-ONNX','runtime':sherpa_onnx.__version__,
        'model_sha256':hashlib.sha256((model/'model.onnx').read_bytes()).hexdigest(),
        'sids':args.sid,'speed':args.speed,'threads':args.threads,'load_s':load,'generated':[],
        'pronunciation_overrides':{'AD钙奶':'艾迪钙奶'},
        'actual_speaker_validated':False,'physical_task_completed':False}
    texts=catalog() if args.operation=='cache' else {'audition':args.text}
    clips={}
    for sid in args.sid:
        for key,text in texts.items():
            # Phonetic spelling only; manifest/task spelling remains AD钙奶.
            spoken_text=text.replace('AD钙奶','艾迪钙奶')
            start=time.monotonic();audio=tts.generate(spoken_text,sid=sid,speed=args.speed)
            samples=np.asarray(audio.samples)
            filename=(key if args.operation=='cache' else 'voice_'+str(sid))+'.wav'
            info=write_audio(args.out/filename,samples,audio.sample_rate,16000 if args.operation=='cache' else None)
            info.update(key=key,sid=sid,text=text,spoken_text=spoken_text,file=filename,generation_s=time.monotonic()-start,
                        source_rate=audio.sample_rate)
            receipt['generated'].append(info)
            if args.operation=='cache':clips[key]={'text':text,'file':filename,'sha256':info['sha256']}
            print(json.dumps(info,ensure_ascii=True),flush=True)
            (args.out/'generation.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
    if clips:
        (args.out/'manifest.json').write_text(json.dumps({'v':1,'clips':clips},ensure_ascii=False,indent=2),encoding='utf-8')
        validate_assets(args.out)
    print('COMPLETE',len(receipt['generated']),flush=True)

if __name__=='__main__':main()
