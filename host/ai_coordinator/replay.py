"""Board model acceptance on synthetic speech/still images; FPGA actions simulated."""
import argparse
import json
from pathlib import Path
import platform
import os
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.main import api,wait_job
from ai_coordinator.core import Mission
from ai_coordinator.schema import TARGETS,dumps

def durable_json(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    if os.name=='posix':
        with path.open('rb') as f:os.fsync(f.fileno())
        durable_directory(path.parent)

def durable_directory(path):
    if os.name=='posix':
        fd=os.open(path,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)


def job(url,path,payload):
    request=api(url,path,payload);result=wait_job(url,request['job_id'],180)
    if result['status']!='done':raise RuntimeError(dumps(result))
    return result


def simulated_execution(spec,scenes,detections,home_ack):
    core=Mission(mode='simulation');now=time.monotonic();core.submit(spec,now);sent=[]
    for _ in range(12):
        actions=core.drain()
        if not actions:break
        for action in actions:
            sent.append(action);now+=.01
            def ack(kind):return {**action,'kind':kind}
            if action['kind']=='start_search':
                core.on_status(ack('search_started'),now)
                frames=detections.get(scenes.get(action['branch'],''),[])
                for index,frame in enumerate(frames):
                    now+=.1
                    core.on_frame({**frame,'task_id':core.task_id,'goal_id':core.goal_id,'sequence':index,
                        'captured_at_s':now,'replay':True,'source_timestamp_verified':False},now+.01)
                if core.state=='searching':core.on_status(ack('region_exhausted'),now+.02)
            elif action['kind']=='target_found':core.on_status(ack('target_accepted'),now)
            elif action['kind']=='return_home' and home_ack:core.on_status(ack('home_arrived'),now)
        if core.state=='returning' and not home_ack:core.tick(now+100)
    return {'mission':core.snapshot(),'commands':sent,'simulated_fpga_responses':True,'physical_task_completed':False}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--simulate-fpga',action='store_true',required=True);ap.add_argument('--limit',type=int,default=0)
    args=ap.parse_args();cfg=json.loads(args.config.read_text(encoding='utf-8'));root=Path(cfg['root'])
    args.out.mkdir(parents=True,exist_ok=False);durable_directory(args.out.parent)
    durable_json(args.out/'attempt.json',{'complete':False,'started_at':time.time(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip() if os.name=='posix' else None})
    url=f"http://127.0.0.1:{cfg.get('port',18765)}"
    cases={c['id']:c for c in json.loads((root/'samples/cases.json').read_text(encoding='utf-8'))}
    warm=job(url,'/warmup',{});durable_json(args.out/'warmup.json',warm)
    vision={};sources={}
    for target in TARGETS:
        vision[target]=[]
        for image in sorted((root/'samples/images').glob(target+'_*.jpg'))[:2]:
            result=job(url,'/image',{'image':str(image)});frame=result['result']['frame']
            vision[target].append(frame);sources[str(image)]=result
            durable_json(args.out/'vision.json',sources)
    vision['background']=vision['ad_calcium_milk']
    rows=[];wavs=sorted((root/'samples/audio').glob('*.wav'))
    if args.limit:wavs=wavs[:args.limit]
    with (args.out/'results.jsonl').open('x',encoding='utf-8') as log:
        durable_directory(args.out)
        for index,wav in enumerate(wavs):
            case_id=wav.stem.removeprefix('Microsoft_Huihui_Desktop_').removeprefix('Microsoft_Kangkang_');case=cases[case_id]
            started=time.monotonic()
            try:
                result=job(url,'/audio',{'wav':str(wav),'plan_only':True});parsed=result['result']['parsed'];answer=parsed['parser']
                correct=bool(not parsed['stale'] and answer['valid'] and answer['task']==case['expected'])
                execution=simulated_execution(answer['task'],case['scenes'],vision,case['simulated_home_ack'])
                expected_state={'search':'completed','stop':'stopped','unknown':'idle','confirm':'idle'}[case['expected']['i']]
                row={'wav':str(wav),'case_id':case_id,'result':result,'semantic_correct':correct,
                     'expected_outcome_matched':correct and execution['mission']['state']==expected_state,'execution':execution}
            except Exception as exc:row={'wav':str(wav),'case_id':case_id,'error':str(exc),'semantic_correct':False,'expected_outcome_matched':False}
            row.update(elapsed_s=time.monotonic()-started,physical_task_completed=False);rows.append(row)
            log.write(dumps(row)+'\n');log.flush()
            if os.name=='posix':os.fsync(log.fileno())
            print(dumps({'n':index+1,'total':len(wavs),'case':case_id,'correct':row['semantic_correct'],'task_match':row['expected_outcome_matched'],'elapsed_s':row['elapsed_s']}),flush=True)
    import statistics
    summary={'n':len(rows),'semantic_correct':sum(r['semantic_correct'] for r in rows),
        'expected_outcome_matched':sum(r['expected_outcome_matched'] for r in rows),
        'median_audio_to_parse_s':statistics.median(r['elapsed_s'] for r in rows),'platform':platform.platform(),
        'physical_task_completed':False,'synthetic_audio':True,'reused_development_images':True,'fpga_simulated':True,
        'runtime_status':api(url,'/status')}
    durable_json(args.out/'summary.json',summary)
    durable_json(args.out/'attempt.json',{'complete':True,'finished_at':time.time(),'n':len(rows)})
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
