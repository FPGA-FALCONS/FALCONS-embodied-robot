"""Real PC models through the integrated runtime; FPGA bytes explicitly simulated."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.runtime import Runtime
from ai_coordinator.reference_peer import SimulationGateway
from ai_coordinator.doctor import inspect_config


async def run(args):
    cfg=json.loads(args.config.read_text(encoding='utf-8'))
    if cfg.get('execution_profile')!='computer_replay' or cfg['models']['vision'].get('provider')!='onnx_cpu':
        raise ValueError('Acceptance only permits explicit CPU-ONNX computer replay profiles')
    verification=inspect_config(args.config,True)
    if not verification['artifacts_ok']:raise ValueError('Artifact verification failed')
    args.out.mkdir(parents=True,exist_ok=False)
    def save(name,obj):(args.out/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
    save('attempt.json',{'complete':False,'started_at':time.time(),'physical_task_completed':False})
    save('artifacts.json',verification)
    rt=Runtime(args.config);rt.gateway=SimulationGateway(rt.core.session_id,rt.audit)
    rows=[];vision={};started=time.monotonic()
    async def job(request,timeout=180):
        if 'job_id' not in request:return request
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            row=rt.jobs[request['job_id']]
            if row['status']!='running':
                if row['status']!='done':raise RuntimeError(json.dumps(row,ensure_ascii=True))
                return row['result']
            await asyncio.sleep(.02)
        raise TimeoutError('Integrated replay deadline')
    async def ack(kind):
        rt.gateway.respond(rt.gateway.commands[-1],kind);await asyncio.sleep(.06)
    def record(row):
        rows.append(row)
        with (args.out/'results.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        print(json.dumps({'stage':row['stage'],'id':row['id'],'correct':row.get('correct')},ensure_ascii=True),flush=True)
    try:
        await rt.start();save('warmup.json',await job(rt.warmup()))
        cases=json.loads(args.cases.read_text(encoding='utf-8'))
        for case in cases:
            t=time.monotonic();result=await job(rt.submit_text(case['text'],plan_only=True))
            answer=result['parser'];correct=answer['valid'] and answer['task']==case['expected']
            record({'stage':'text','id':case['id'],'text':case['text'],'expected':case['expected'],'result':result,
                    'correct':correct,'elapsed_s':time.monotonic()-t})
        # Actual ASR for a fixed cross-section; other 42-case audio work is retained.
        selected=('single_tape','single_screw','single_ad','single_nutri','fb_return_1','stop_plain','ab_injection')
        case_map={c['id']:c for c in cases}
        for identifier in selected:
            if identifier not in case_map:raise ValueError('Missing frozen audio acceptance case: '+identifier)
            files=list(args.audio.glob('*_'+identifier+'.wav'))
            if len(files)!=1:raise ValueError('Expected exactly one synthetic WAV for '+identifier)
            t=time.monotonic();result=await job(rt.submit_audio({'wav':str(files[0]),'plan_only':True}))
            answer=result.get('parsed',{}).get('parser',{})
            record({'stage':'audio','id':identifier,'synthetic_audio':True,'result':result,
                    'correct':answer.get('valid') is True and answer.get('task')==case_map[identifier]['expected'],
                    'elapsed_s':time.monotonic()-t})
        images=json.loads(args.images.read_text(encoding='utf-8'))
        for target,paths in images.items():
            vision[target]=[]
            for index,path in enumerate(paths):
                result=await job(rt.submit_image({'image':path}));vision[target].append(result)
                record({'stage':'vision','id':target+'_'+str(index),'result':result,
                        'correct':any(d['class']==target for d in result['frame']['detections'])})
        save('vision.json',vision)
        for text,fallback in (('寻找胶带，找到后返回起点',False),('先找胶带，找不到再找螺丝刀，找到后返回起点',True)):
            await job(rt.submit_text(text));await ack('search_started')
            if fallback:
                await job(rt.submit_image({'image':images['ad_calcium_milk'][0]}))
                await ack('region_exhausted');await ack('search_started')
            target='screwdriver' if fallback else 'tape'
            for path in images[target][:2]:await job(rt.submit_image({'image':path}))
            visual_confirmed=rt.core.state=='waiting_found_ack'
            if visual_confirmed:await ack('target_accepted');await ack('home_arrived')
            record({'stage':'mission','id':'fallback_return' if fallback else 'find_return',
                    'correct':visual_confirmed and rt.core.state=='completed','mission':rt.core.snapshot(),
                    'physical_task_completed':False,'simulated_fpga':True,'real_models':True})
        rt.stop();await asyncio.sleep(.05)
        record({'stage':'stop','id':'cancel','correct':rt.core.state=='stopped','mission':rt.core.snapshot()})
        await asyncio.sleep(.1)
        summary={'stages':{stage:{'n':sum(r['stage']==stage for r in rows),
                                   'correct':sum(r['stage']==stage and r['correct'] for r in rows)} for stage in sorted({r['stage'] for r in rows})},
                 'all_passed':all(r['correct'] for r in rows),'elapsed_s':time.monotonic()-started,
                 'physical_task_completed':False,'synthetic_audio':True,'reused_development_images':True,
                 'fpga_simulated':True,'runtime':rt.snapshot()}
        save('summary.json',summary);save('attempt.json',{'complete':True,'finished_at':time.time()})
        if not summary['all_passed']:raise RuntimeError('Integrated replay has failures; inspect summary')
    finally:
        await rt.close();save('final_status.json',{'private_workers_reaped':all(w.proc is None for w in rt.workers.values()),
                                                   'private_llm_reaped':rt.model_server.proc is None,'run_dir':str(rt.run_dir)})


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser()
    for key in ('config','cases','audio','images','out'):ap.add_argument('--'+key,type=Path,required=True)
    asyncio.run(run(ap.parse_args()))


if __name__=='__main__':main()
