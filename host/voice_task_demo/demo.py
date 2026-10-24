"""Generate or load speech, recognize it, parse a task, then replay fixed image scenes."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'voice_bridge'))
from pipeline import process_wav
from infer import PrivateParser
from executor import replay
from run_eval import build_vision
from safety_rules import guard

WORKSPACE=ROOT.parents[2]
DEFAULT_WORK=WORKSPACE/'ai部分/voice_domain_adaptation_20261024'


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser(description=__doc__)
    source=ap.add_mutually_exclusive_group(required=True);source.add_argument('--text');source.add_argument('--wav',type=Path)
    ap.add_argument('--work',type=Path,default=DEFAULT_WORK)
    ap.add_argument('--out',type=Path,default=None)
    ap.add_argument('--scene',choices=['primary','fallback','absent'],default='primary')
    ap.add_argument('--simulate-home-arrival',action='store_true',help='explicit simulated navigation acknowledgement')
    ap.add_argument('--voice',default='Microsoft Huihui Desktop')
    ap.add_argument('--asr-engine',choices=['transducer','sensevoice'],default='transducer')
    ap.add_argument('--asr-model',type=Path,default=WORKSPACE/'ai部分/ai_dual_model_poc/weights/asr')
    ap.add_argument('--images',type=Path,default=WORKSPACE/'ai部分/ai_dual_model_poc/data/holdout24/images')
    ap.add_argument('--yolo',type=Path,default=WORKSPACE/'ai部分/field_adaptation_20261021_v16_lab122/runs/v17s_replay/weights/best.pt')
    ap.add_argument('--gguf',type=Path,default=None,help='override text parser GGUF; default work/domain-Q8_0.gguf')
    ap.add_argument('--safety-guard',action='store_true',help='apply deterministic stop/unsupported-action guard before the parser')
    args=ap.parse_args();out=args.out or args.work/'demos'/uuid.uuid4().hex[:12];out.mkdir(parents=True,exist_ok=False)
    model=args.gguf or (args.work/'domain-Q8_0.gguf')
    wav=args.wav
    if args.text:
        tts=out/'tts';tts.mkdir()
        (tts/'cases.json').write_text(json.dumps([{'id':'utterance','text':args.text}],ensure_ascii=False),encoding='utf-8')
        (tts/'protocol.json').write_text(json.dumps({'voices':[args.voice]},ensure_ascii=False),encoding='utf-8')
        shell=shutil.which('pwsh') or shutil.which('powershell')
        if not shell:raise RuntimeError('Local Windows SAPI synthesis needs PowerShell; use --wav on Linux')
        result=subprocess.run([shell,'-NoProfile','-File',str(ROOT/'synthesize.ps1'),'-Work',str(tts),'-Out',str(out/'audio')],
                    capture_output=True,timeout=60,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if result.returncode:raise RuntimeError(result.stderr.decode('utf-8','replace'))
        wav=Path(json.loads((out/'audio/manifest.json').read_text(encoding='utf-8-sig'))[0]['wav'])
    if args.asr_engine=='sensevoice':
        from sensevoice import bounded_decode
        event=process_wav(wav,args.asr_model,out/'asr',decoder=bounded_decode)
    else:
        event=process_wav(wav,args.asr_model,out/'asr',hotwords_file=args.work/'hotwords.txt')
    policy=event.get('domain_vocab',{});task_text=event.get('task_text')
    # Hard stop/unsupported policies must see the raw transcript: an explicit
    # stop must win even when a banned-word policy would otherwise block parsing.
    hard=guard(event.get('text','')) if args.safety_guard and event['status']=='recognized' else None
    if hard is not None:
        answer={'task':hard,'valid':True,'source':'safety_guard','elapsed_s':0}
    elif not task_text or event['status']!='recognized':
        answer={'task':{'i':'unknown'},'source':'asr_or_domain_policy','valid':False}
    else:
        with PrivateParser(model,args.work,out/'parser') as parser:
            answer=parser.parse(task_text)
    task=answer['task'];vision={};scenes={}
    if task['i']=='search':
        vision=build_vision(args.images,args.yolo,out)
        # Select a deterministic replay scene, not a camera or measured search trajectory.
        if args.scene=='primary':scenes={'primary':task['t'],'fallback':task.get('f','')}
        elif args.scene=='fallback':
            distractor=next(t for t in ('ad_calcium_milk','nutri_express','tape','screwdriver') if t!=task['t'])
            scenes={'primary':distractor,'fallback':task.get('f','')}
        else:scenes={'primary':'missing','fallback':'missing'}
    execution=replay(task,scenes,vision,args.simulate_home_arrival)
    result={'wav':str(wav),'asr_engine':args.asr_engine,'parser_model':str(model),
            'safety_guard':args.safety_guard,'asr':event,'parser':answer,'execution':execution,
            'out':str(out),'synthetic_audio':bool(args.text),'image_scene_is_scripted':True,
            'motion_authorized':False,'physical_task_completed':False}
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'raw_asr':event['text'],'task_text':task_text,'task':task,'status':execution['status'],
                      'parser_model':str(model),'safety_guard':args.safety_guard,
                      'trace':execution['trace'],'out':str(out),'physical_task_completed':False},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
