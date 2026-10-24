"""Synthetic speech -> real ASR -> real Qwen -> real image detections -> task replay."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[0]/'voice_bridge'))
from audio import Audio,write_wav
from pipeline import process_wav
from preview import normalize
from executor import replay
from infer import PrivateParser


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def char_error(reference,hypothesis):
    reference=normalize(reference);hypothesis=normalize(hypothesis)
    previous=list(range(len(hypothesis)+1))
    for i,a in enumerate(reference,1):
        current=[i]
        for j,b in enumerate(hypothesis,1):
            current.append(min(current[-1]+1,previous[j]+1,previous[j-1]+(a!=b)))
        previous=current
    return {'edits':previous[-1],'reference_chars':len(reference)}


def expected_status(task):
    return {'search':'success','stop':'stopped','unknown':'needs_clarification','confirm':'confirmation_only'}[task['i']]


def build_vision(images,weights,out):
    # Freeze file selection BEFORE prediction; reused development pictures are not a blind set.
    scenes={target:sorted(images.glob(target+'_*.jpg'))[:2] for target in ('tape','screwdriver','ad_calcium_milk','nutri_express')}
    if any(len(paths)!=2 for paths in scenes.values()):raise ValueError('Need two existing pictures of each target')
    scenes['background']=scenes['ad_calcium_milk']  # Fixed distractor for searching tape.
    manifest={'model':str(weights),'model_sha256':sha(weights),'conf':.3,'imgsz':640,
              'images':{str(p):sha(p) for paths in scenes.values() for p in paths},
              'scenes':{key:[str(p) for p in paths] for key,paths in scenes.items()},
              'scope':'Reused still-image windows; background scene contains AD milk, not an empty real room.'}
    (out/'vision_protocol.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    from ultralytics import YOLO
    import cv2
    model=YOLO(str(weights));cache={}
    for paths in scenes.values():
        for path in paths:
            if str(path) in cache:continue
            start=time.perf_counter();prediction=model.predict(str(path),conf=.3,imgsz=640,device=0,verbose=False)[0]
            assert set(prediction.names.values())=={'tape','screwdriver','ad_calcium_milk','nutri_express'},prediction.names
            detections=[]
            for box,confidence,cls in zip(prediction.boxes.xyxy.tolist(),prediction.boxes.conf.tolist(),prediction.boxes.cls.tolist()):
                detections.append({'class':prediction.names[int(cls)],'confidence':float(confidence),'bbox':box})
            cache[str(path)]={'image':str(path),'detections':detections,'elapsed_s':time.perf_counter()-start}
            encoded=cv2.imencode('.jpg',prediction.plot())[1]
            (out/(path.stem+'_detections.jpg')).write_bytes(encoded.tobytes())
    result={key:[cache[str(p)] for p in paths] for key,paths in scenes.items()}
    (out/'vision_results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--asr-model',type=Path,required=True)
    ap.add_argument('--images',type=Path,required=True);ap.add_argument('--yolo',type=Path,required=True)
    ap.add_argument('--pilot',type=Path,required=True);args=ap.parse_args()
    out=args.out;out.mkdir(parents=True,exist_ok=False)
    protocol=json.loads((args.work/'protocol.json').read_text(encoding='utf-8'))
    for name,digest in protocol['files'].items():assert sha(args.work/name)==digest,name
    cases={c['id']:c for c in json.loads((args.work/'cases.json').read_text(encoding='utf-8'))}
    manifest=json.loads((args.work/'audio/manifest.json').read_text(encoding='utf-8-sig'))
    (out/'input_manifest.json').write_text(json.dumps({'protocol_sha256':sha(args.work/'protocol.json'),
        'audio':{r['id']:{'wav':r['wav'],'sha256':sha(r['wav']),'case_id':r['case_id']} for r in manifest},
        'model_sha256':{'pilot':sha(args.pilot/'student-Q8_0.gguf'),'domain':sha(args.work/'domain-Q8_0.gguf')},
        'asr_sha256':{p.name:sha(p) for p in args.asr_model.iterdir() if p.is_file()},
        'synthetic_development':True,'hardware_connected':False},ensure_ascii=False,indent=2),encoding='utf-8')
    vision=build_vision(args.images,args.yolo,out)
    transcripts=[]
    modes=[('greedy',{}),('beam',{'decoding_method':'modified_beam_search'}),
           ('hotwords',{'hotwords_file':args.work/'hotwords.txt','hotwords_score':protocol['asr']['score']})]
    with (out/'asr_results.jsonl').open('x',encoding='utf-8') as log:
        for mode,kwargs in modes:
            for index,source in enumerate(manifest):
                event=process_wav(source['wav'],args.asr_model,out/'asr'/mode/source['id'],**kwargs)
                row={'id':source['id'],'case_id':source['case_id'],'voice':source['voice'],'mode':mode,
                     'text':event['text'],'status':event['status'],'elapsed_s':event['processing_s'],
                     'error':event.get('error'),'asr':event.get('asr'),
                     'character_error':char_error(cases[source['case_id']]['text'],event['text'])}
                transcripts.append(row);log.write(json.dumps(row,ensure_ascii=False)+'\n');log.flush()
                if (index+1)%8==0:print('ASR',mode,index+1,'/',len(manifest),flush=True)
    # Actual zero-energy input checks the same pre-ASR gate, including hotword mode.
    import numpy as np
    silence=out/'silence.wav';write_wav(silence,Audio(16000,np.zeros(16000)))
    silent=process_wav(silence,args.asr_model,out/'silence',hotwords_file=args.work/'hotwords.txt')
    assert silent['status']=='no_speech_energy' and silent['task_preview'] is None
    model_paths={'pilot':args.pilot/'student-Q8_0.gguf','domain':args.work/'domain-Q8_0.gguf'}
    # A separate text route isolates ASR losses; it is NEVER counted as an audio success.
    inputs=[dict(r,route='audio') for r in transcripts]
    inputs += [{'id':'text_'+case['id'],'case_id':case['id'],'voice':None,'mode':'reference_text',
                'text':case['text'],'status':'recognized','elapsed_s':0,'route':'reference_text_diagnostic'}
               for case in cases.values()]
    all_results=[]
    for model,path in model_paths.items():
        with PrivateParser(path,args.work,out/model) as parser:
            with (out/(model+'_results.jsonl')).open('x',encoding='utf-8') as log:
                for index,item in enumerate(inputs):
                    case=cases[item['case_id']]
                    if item['status']=='recognized' and item['text']:
                        answer=parser.parse(item['text'])
                    else:answer={'task':{'i':'unknown'},'valid':False,'error':'ASR failed or empty','elapsed_s':0}
                    task=answer['task']
                    execution=replay(task,case['scenes'],vision,case['simulated_home_ack'])
                    semantic=answer['valid'] and task==case['expected']
                    outcome=bool(semantic and execution['status']==expected_status(case['expected']))
                    row={**item,'model':model,'expected':case['expected'],'parser':answer,
                         'semantic_correct':semantic,'expected_outcome_matched':outcome,'execution':execution,
                         'motion_authorized':False,'physical_task_completed':False}
                    all_results.append(row);log.write(json.dumps(row,ensure_ascii=False)+'\n');log.flush()
                    if (index+1)%32==0:print('TASK',model,index+1,'/',len(inputs),flush=True)
            # Last request deliberately expires; only this tool's private worker is stopped.
            timed_out=False
            try:parser.parse('先找胶带，找不到再找螺丝刀，找到后返回起点。',deadline=.001)
            except (TimeoutError,RuntimeError):timed_out=True
            if parser.proc.poll() is None:
                # A fast connection error may precede the timer; cleanup still belongs to this process.
                parser.proc.terminate()
            parser.proc.wait(timeout=5)
            (out/model/'deadline_probe.json').write_text(json.dumps({'timeout_observed':timed_out,
                'private_worker_exited':parser.proc.poll() is not None,'physical_task_completed':False},indent=2),encoding='utf-8')
    summary={'date':'2026-10-24','audio_count':len(manifest),'asr_modes':{},'task_results':{},
             'scope':'Synthetic voices + reused pictures + simulated navigation; not a real microphone/robot/RK3588 test.'}
    for mode,_ in modes:
        rows=[r for r in transcripts if r['mode']==mode]
        summary['asr_modes'][mode]={'n':len(rows),'errors':sum(r['status']!='recognized' for r in rows),
            'CER':sum(r['character_error']['edits'] for r in rows)/sum(r['character_error']['reference_chars'] for r in rows),
            'median_seconds':statistics.median(r['elapsed_s'] for r in rows)}
    for model in model_paths:
        summary['task_results'][model]={}
        for mode in ('greedy','beam','hotwords','reference_text'):
            rows=[r for r in all_results if r['model']==model and r['mode']==mode]
            summary['task_results'][model][mode]={'n':len(rows),'semantic_correct':sum(r['semantic_correct'] for r in rows),
                'expected_outcome_matched':sum(r['expected_outcome_matched'] for r in rows),
                'false_search':sum(r['parser']['task']['i']=='search' and r['expected']['i']!='search' for r in rows),
                'missed_stop':sum(r['expected']['i']=='stop' and r['parser']['task']['i']!='stop' for r in rows),
                'median_parser_seconds':statistics.median(r['parser']['elapsed_s'] for r in rows)}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
