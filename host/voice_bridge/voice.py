"""Entry point: USB microphone / WAV -> offline Chinese text -> task preview."""
import argparse
import datetime
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import sys
import uuid

try:
    from .capture import devices, record, backend_name
    from .model import model_status
except ImportError:
    from capture import devices, record, backend_name
    from model import model_status

ROOT = Path(__file__).resolve().parent


def default_model():
    if os.environ.get('FALCONS_ASR_MODEL_DIR'):
        return Path(os.environ['FALCONS_ASR_MODEL_DIR'])
    packaged = ROOT/'models/asr'
    if packaged.is_dir():return packaged
    # Development checkout only. Portable kits use models/asr or an explicit path.
    workspace = ROOT.parents[2]
    return workspace/'ai部分/ai_dual_model_poc/weights/asr'


def show(value):
    print(json.dumps(value,ensure_ascii=False,indent=2))


def session_directory(root):
    name = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
    path = Path(root)/name; path.mkdir(parents=True,exist_ok=False)
    return path


def main(argv=None):
    if hasattr(sys.stdout,'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    ap = argparse.ArgumentParser(description='FALCONS USB microphone and offline Chinese ASR; preview only')
    subs = ap.add_subparsers(dest='command',required=True)
    for name in ('devices','doctor','record','transcribe','listen','_asr-worker'):
        sub = subs.add_parser(name)
        if name in ('devices','doctor','record','listen'):
            sub.add_argument('--backend',choices=['auto','alsa','winmm'],default='auto')
        if name in ('record','listen'):
            sub.add_argument('--device',default='default')
            sub.add_argument('--seconds',type=float,default=5)
            sub.add_argument('--rate',type=int,default=None)
            sub.add_argument('--channels',type=int,default=1)
        if name in ('doctor','transcribe','listen','_asr-worker'):
            sub.add_argument('--model-dir',type=Path,default=default_model())
        if name in ('transcribe','listen','_asr-worker'):
            sub.add_argument('--threads',type=int,default=2)
            sub.add_argument('--decoding-method',choices=['greedy_search','modified_beam_search'],default='greedy_search')
            sub.add_argument('--hotwords-file',type=Path,help='UTF-8 domain words; enables modified beam search')
            sub.add_argument('--hotwords-score',type=float,default=1.5)
        if name in ('transcribe','_asr-worker'):
            sub.add_argument('--wav',type=Path,required=True)
        if name in ('record','transcribe','listen'):
            sub.add_argument('--out',type=Path,default=ROOT/'recordings')
        if name in ('transcribe','listen'):
            sub.add_argument('--disable-domain-normalization',action='store_true',help='diagnostic raw-text task parsing')
            sub.add_argument('--channel',type=int,default=0)
            sub.add_argument('--silence-dbfs',type=float,default=-55)
            sub.add_argument('--asr-timeout',type=float,default=15)
            sub.add_argument('--llm-url',help='optional explicit OpenAI-compatible local/LAN server base URL')
            sub.add_argument('--llm-timeout',type=float,default=5)
        if name=='listen':
            sub.add_argument('--once',action='store_true',help='one Enter-triggered recording then exit')
    args = ap.parse_args(argv)
    try:
        if args.command=='devices':
            show(devices(args.backend));return 0
        if args.command=='doctor':
            packages={}
            for name in ('numpy','scipy','sherpa_onnx'):
                packages[name] = importlib.metadata.version(name.replace('_','-')) if importlib.util.find_spec(name) else None
            try:inputs=devices(args.backend)
            except Exception as exc:inputs={'error':str(exc)}
            show({'python':sys.version,'platform':sys.platform,'packages':packages,
                  'model':model_status(args.model_dir),'audio':inputs,
                  'environment_changed':False,'motion_authorized':False})
            return 0
        try:
            from .audio import write_wav, quality
            from .pipeline import process_wav, decode_local
        except ImportError:
            from audio import write_wav, quality
            from pipeline import process_wav, decode_local
        if args.command=='_asr-worker':
            show({'text':decode_local(args.wav,args.model_dir,args.threads,args.hotwords_file,
                                     args.hotwords_score,args.decoding_method)});return 0
        if args.command=='transcribe':
            directory=session_directory(args.out)
            result=process_wav(args.wav,args.model_dir,directory,args.channel,args.silence_dbfs,
                args.asr_timeout,args.threads,args.llm_url,args.llm_timeout,
                hotwords_file=args.hotwords_file,hotwords_score=args.hotwords_score,decoding_method=args.decoding_method,
                domain_normalization=not args.disable_domain_normalization)
            show({'session_dir':str(directory.resolve()),**result})
            return 0 if result['status']=='recognized' else 2 if result['status']=='error' else 3
        while True:
            if args.command=='listen':
                response=input(f'Enter 开始录音（最多 {args.seconds:g} 秒）；输入 q 退出：').strip().lower()
                if response=='q':return 0
            directory=session_directory(args.out)
            print(f'录音中：{args.seconds:g} 秒，设备 {args.device}',file=sys.stderr,flush=True)
            audio,source=record(args.device,args.seconds,args.rate,args.channels,args.backend)
            path=directory/'capture.wav';write_wav(path,audio)
            if args.command=='record':
                show({'wav':str(path.resolve()),'source':source,'quality':quality(audio),'motion_authorized':False});return 0
            result=process_wav(path,args.model_dir,directory,args.channel,args.silence_dbfs,
                args.asr_timeout,args.threads,args.llm_url,args.llm_timeout,source=source,
                hotwords_file=args.hotwords_file,hotwords_score=args.hotwords_score,decoding_method=args.decoding_method,
                domain_normalization=not args.disable_domain_normalization)
            show({'session_dir':str(directory.resolve()),**result})
            if args.once:return 0 if result['status']=='recognized' else 2 if result['status']=='error' else 3
    except (EOFError,KeyboardInterrupt):
        print('录音已结束；未发送任何运动命令。',file=sys.stderr);return 130
    except Exception as exc:
        show({'status':'error','error':type(exc).__name__+': '+str(exc),
              'execution_allowed':False,'motion_authorized':False});return 2


if __name__=='__main__':
    raise SystemExit(main())
