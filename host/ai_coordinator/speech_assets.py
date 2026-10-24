"""Prepare/check cached speech or render a simulated sequence without hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import wave

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.announcements import catalog,validate_assets
from ai_coordinator.core import Mission

def simulated_demo():
    core=Mission(mode='simulation');core.submit({'i':'search','t':'tape','r':True},1);events=core.drain_notices()
    start=core.drain()[0];core.on_status({**start,'kind':'search_started'},1.1);events+=core.drain_notices()
    for seq,now in enumerate((1.2,1.3)):
        core.on_frame({'task_id':core.task_id,'goal_id':core.goal_id,'sequence':seq,'captured_at_s':now,
            'width':640,'height':480,'detections':[{'class':'tape','confidence':.9,'bbox':[1,1,20,20]}]},now)
    found=core.drain()[0];core.on_status({**found,'kind':'target_accepted'},1.4);events+=core.drain_notices()
    home=core.drain()[0];core.on_status({**home,'kind':'home_arrived'},1.5);events+=core.drain_notices()
    return events,core.snapshot()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('operation',choices=['catalog','index','check','demo'])
    ap.add_argument('--assets',type=Path,required=True);ap.add_argument('--out',type=Path);args=ap.parse_args()
    if args.operation=='catalog':
        args.assets.mkdir(parents=True,exist_ok=True)
        (args.assets/'catalog.json').write_text(json.dumps(catalog(),ensure_ascii=False,indent=2),encoding='utf-8')
        print('Catalog written');return
    if args.operation=='index':
        clips={}
        for key,text in catalog().items():
            path=args.assets/(key+'.wav')
            clips[key]={'text':text,'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        (args.assets/'manifest.json').write_text(json.dumps({'v':1,'clips':clips},ensure_ascii=False,indent=2),encoding='utf-8')
    clips=validate_assets(args.assets)
    if args.operation=='demo':
        if args.out is None:ap.error('--out is required for demo')
        # Prefixes remain audible: these are simulated states, never an actual completed drive.
        events,mission=simulated_demo();sequence=['sim_'+e['code'] for e in events]
        args.out.parent.mkdir(parents=True,exist_ok=True)
        with wave.open(str(args.out),'wb') as output:
            output.setparams((1,2,16000,0,'NONE','not compressed'))
            for key in sequence:
                with wave.open(str(clips[key]['path']),'rb') as source:output.writeframes(source.readframes(source.getnframes()))
                output.writeframes(b'\0\0'*6400)
        args.out.with_suffix('.json').write_text(json.dumps({'simulated':True,'physical_task_completed':False,
            'clips':sequence,'text':[clips[k]['text'] for k in sequence],'events':events,'mission':mission},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'clips':len(clips),'total_s':sum(c['duration_s'] for c in clips.values()),
                      'bytes':sum(c['path'].stat().st_size for c in clips.values())}))

if __name__=='__main__':main()
