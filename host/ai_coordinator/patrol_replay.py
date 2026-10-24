"""PC-only real-ASR/vision patrol demonstration; simulated FPGA and human pickup."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.runtime import Runtime
from ai_coordinator.reference_peer import SimulationGateway


async def run(args):
    cfg=json.loads(args.config.read_text(encoding='utf-8'))
    if cfg.get('execution_profile')!='computer_replay' or cfg['models']['vision'].get('provider')!='onnx_cpu':raise ValueError('PC replay only')
    args.out.mkdir(exist_ok=False);rt=Runtime(args.config);rt.gateway=SimulationGateway(rt.core.session_id,rt.audit)
    (args.out/'config.used.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
    (args.out/'attempt.json').write_text(json.dumps({'complete':False,'started_at':time.time()}),encoding='utf-8')
    # These are explicit simulator fixture values, not vehicle calibration.
    rt.cfg['patrol']={'enabled':True,'waypoints':['P1','P2'],'targets':['screwdriver'],'cycles':1,'stop_band_m':[.2,.4]}
    images=json.loads(args.images.read_text(encoding='utf-8'));steps=[]
    async def job(value):
        if 'job_id' not in value:return value
        for _ in range(6000):
            row=rt.jobs[value['job_id']]
            if row['status']!='running':
                if row['status']!='done':raise RuntimeError(json.dumps(row,ensure_ascii=True))
                return row['result']
            await asyncio.sleep(.01)
        raise TimeoutError('Patrol replay timed out')
    async def ack(kind,**evidence):
        action=next(c for c in reversed(rt.gateway.commands) if c['action_id']==rt.core.pending['action_id'])
        rt.gateway.respond(action,kind,**evidence);await asyncio.sleep(.05)
        steps.append({'event':kind,'state':rt.core.state})
    try:
        await rt.start()
        result=await job(rt.submit_audio({'wav':str(args.commands/'start.wav')}));steps.append({'start_audio':result})
        if rt.core.state!='patrolling':raise RuntimeError('ASR did not start configured patrol')
        for path in images['screwdriver'][:2]:
            steps.append({'actual_vision':await job(rt.submit_image({'image':path}))})
        if rt.core.state!='pausing_patrol':raise RuntimeError('Visual target was not confirmed')
        await ack('patrol_paused')
        await ack('approach_arrived',metric_stop_verified=True,stationary=True,clearance_m=.3)
        if rt.core.state!='waiting_pickup':raise RuntimeError('Not waiting for explicit pickup confirmation')
        result=await job(rt.submit_audio({'wav':str(args.commands/'pickup.wav')}));steps.append({'pickup_audio':result})
        if rt.core.state!='patrolling' or rt.gateway.commands[-1]['waypoint']!='P1':raise RuntimeError('Did not resume interrupted waypoint')
        await ack('waypoint_arrived');await ack('waypoint_arrived')
        await asyncio.sleep(.05)
        summary={'passed':rt.core.state=='completed','steps':steps,'snapshot':rt.snapshot(),
                 'real_asr':True,'real_vision':True,'llm_loaded':rt.model_server.proc is not None,
                 'fpga_simulated':True,'metric_arrival_simulated':True,'pickup_confirmation_audio_synthetic':True,
                 'physical_pickup_verified':False,'physical_task_completed':False}
        (args.out/'result.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        if not summary['passed']:raise RuntimeError('Patrol replay incomplete')
        (args.out/'attempt.json').write_text(json.dumps({'complete':True,'finished_at':time.time()}),encoding='utf-8')
        print(json.dumps({k:v for k,v in summary.items() if k not in ('steps','snapshot')},ensure_ascii=True))
    except BaseException as exc:
        (args.out/'failure.json').write_text(json.dumps({'error':repr(exc),'steps':steps,'snapshot':rt.snapshot()},ensure_ascii=False,indent=2),encoding='utf-8')
        raise
    finally:
        await rt.close()
        (args.out/'final_status.json').write_text(json.dumps({'workers_reaped':all(w.proc is None for w in rt.workers.values()),'llm_reaped':rt.model_server.proc is None,'run_dir':str(rt.run_dir)},indent=2),encoding='utf-8')


def main():
    ap=argparse.ArgumentParser()
    for key in ('config','images','commands','out'):ap.add_argument('--'+key,type=Path,required=True)
    asyncio.run(run(ap.parse_args()))


if __name__=='__main__':main()
