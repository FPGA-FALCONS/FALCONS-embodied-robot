"""Linux board installer for this isolated service; validates before activation.

No model inference, driver/package changes, system reboots, or FPGA flashing.
Intended to be invoked as orangepi, including through an explicitly selected ADB.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import urllib.request
import uuid
import zipfile

C1B_SHA='b13e07d5a96cb05f2d949cf91192246c8a8d01436e340992bc83be2476fa278b'


def digest(path):
    out=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):out.update(chunk)
    return out.hexdigest()


def durable(path,data):
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temp.open('w',encoding='utf-8') as stream:
        stream.write(data);stream.flush();os.fsync(stream.fileno())
    os.replace(temp,path)
    fd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


def install(args):
    root=args.root.absolute()
    if root!=Path('/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1'):raise ValueError('Unexpected deployment root')
    if not (root/'config.json').is_file():raise ValueError('Existing isolated coordinator deployment required')
    old_cfg=(root/'config.json').read_text();cfg=json.loads(old_cfg)
    previous_active=(root/'active_release.json').read_text()
    stamp=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    release=root/'releases'/('fullstack_'+stamp);release.mkdir(parents=True,exist_ok=False)
    with zipfile.ZipFile(args.bundle) as archive:
        manifest=json.loads(archive.read('manifest.json'))
        for name in archive.namelist():
            path=(release/name).resolve()
            if release.resolve() not in path.parents:raise ValueError('Bundle path escape')
        if set(archive.namelist())!=set(manifest)|{'manifest.json'}:raise ValueError('Bundle manifest mismatch')
        archive.extractall(release)
    for name,entry in manifest.items():
        p=release/name
        if p.stat().st_size!=entry['bytes'] or digest(p)!=entry['sha256']:raise ValueError('Bundle checksum failed: '+name)
    candidates=[root/'models/c1b-Q8_0.gguf',root.parent/'falcons_llm_cpu_20261024/c1b-Q8_0.gguf',
                root.parent/'falcons_llm_cpu_20261024/models/c1b-Q8_0.gguf']
    model=next((p for p in candidates if p.is_file() and digest(p)==C1B_SHA),None)
    if model is None:raise ValueError('Verified C1b missing; no model substitution permitted')
    target=root/'models/c1b-Q8_0.gguf'
    if model!=target:
        if target.exists():raise FileExistsError('Different target weights retained')
        shutil.copy2(model,target)
        with target.open('rb') as stream:os.fsync(stream.fileno())
        fd=os.open(target.parent,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    system=release/'system.txt';system.write_text((release/'host/ai_coordinator/profiles/c1b_system.txt').read_text(encoding='utf-8').rstrip('\n'),encoding='utf-8')
    cfg=copy.deepcopy(cfg);cfg.update(language_profile='zh-C1b-Q8-v1',model_loads_enabled=False,mode='shadow')
    cfg['gateway']={**cfg.get('gateway',{}),'mode':'shadow','allow_motion':False}
    cfg['camera']={**cfg.get('camera',{}),'enabled':False}
    voice_motion={**cfg.get('voice_motion',{}),'enabled':False,'live_commissioned':False,
                  'backend':'mot1_shared_serial','patrol_target':'fpga_preset','speed':10,
                  'heartbeat_s':.2,'turn_window_s':8.5,'drive_margin_s':.3}
    voice_motion.setdefault('distance',{'calibrated':False,'calibrated_speed_mps':None,
        'min_speed_mps':.05,'max_speed_mps':.5,'max_distance_m':2.,'max_duration_s':5.})
    cfg['voice_motion']=voice_motion
    cfg['announcements']={**cfg.get('announcements',{}),'mode':'log','assets':str(release/'speech'),'device':cfg.get('announcements',{}).get('device')}
    cfg['patrol']={'enabled':False,'waypoints':[],'targets':['screwdriver'],'cycles':1,'stop_band_m':None}
    llm=cfg['models']['llm'];llm['system']=str(system);llm['label']='C1b Q8 / llama.cpp CPU'
    command=list(llm['server_command']);command[command.index('-m')+1]=str(target)
    command[command.index('--chat-template-file')+1]=str(release/'models/shared_chat_template.jinja');llm['server_command']=command
    cfg['artifacts']={}
    paths={'llm':target,'system':system,'chat_template':release/'models/shared_chat_template.jinja',
           'asr_model':Path(cfg['models']['asr']['model']),'asr_tokens':Path(cfg['models']['asr']['tokens']),
           'vision':Path(cfg['models']['vision']['model'])}
    for name,p in paths.items():cfg['artifacts'][name]={'path':str(p),'bytes':p.stat().st_size,'sha256':digest(p)}
    staged=release/'config.staged.json';staged.write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
    env=dict(os.environ,PYTHONPATH=str(root/'python')+':'+str(release/'host'),PYTHONDONTWRITEBYTECODE='1',
             XDG_RUNTIME_DIR='/run/user/'+str(os.getuid()),DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/'+str(os.getuid())+'/bus')
    for label,argv in [('tests',['python3','-m','unittest','discover','-s',str(release/'host/ai_coordinator/tests'),'-v']),
                       ('camera_tests',['python3','-m','unittest','discover','-s',str(release/'host/fx2_camera/tests'),'-v']),
                       ('motion_tests',['python3','-m','unittest','discover','-s',str(release/'host/semantic_nav'),'-v']),
                       ('doctor',['python3',str(release/'host/ai_coordinator/doctor.py'),'--config',str(staged),'--deep'])]:
        result=subprocess.run(argv,env=env,capture_output=True,text=True,timeout=120)
        (release/(label+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        if result.returncode:raise RuntimeError('Staged '+label+' failed; active release retained')
    for p in release.rglob('*'):
        if p.is_file():
            with p.open('rb') as stream:os.fsync(stream.fileno())
    for p in sorted([release]+[p for p in release.rglob('*') if p.is_dir()],key=lambda p:len(p.parts),reverse=True):
        fd=os.open(p,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    fd=os.open(release.parent,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)
    if not args.activate:
        print(json.dumps({'staged':str(release),'active_changed':False,'model_loads_enabled':False}));return
    unit=Path.home()/'.config/systemd/user/falcons-ai-coordinator.service';old_unit=unit.read_text()
    if str(root) not in old_unit:raise ValueError('Existing unrelated service retained')
    lines=old_unit.splitlines();new_unit='\n'.join(('ExecStart=/usr/bin/python3 '+str(release/'host/ai_coordinator/main.py')+' --config '+str(root/'config.json')+' serve') if l.startswith('ExecStart=') else
        ('Environment=PYTHONPATH='+str(root/'python')+':'+str(release/'host')) if l.startswith('Environment=PYTHONPATH=') else l for l in lines)+'\n'
    durable(release/'config.previous.json',old_cfg);durable(release/'active.previous.json',previous_active);durable(release/'service.previous.txt',old_unit)
    def control(*command):
        subprocess.run(['systemctl','--user',*command],env=env,check=True,capture_output=True,text=True,timeout=20)
    try:
        control('stop','falcons-ai-coordinator.service')
        durable(root/'config.json',json.dumps(cfg,ensure_ascii=False,indent=2))
        durable(root/'active_release.json',json.dumps({'release':str(release),'manifest':manifest},indent=2))
        durable(unit,new_unit);control('daemon-reload');control('start','falcons-ai-coordinator.service')
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}));health=None
        for _ in range(30):
            try:
                with opener.open('http://127.0.0.1:18765/status',timeout=1) as response:health=json.load(response)
                if health.get('model_loads_enabled') is False and health.get('language_profile')=='zh-C1b-Q8-v1' and all(w['pid'] is None for w in health['workers'].values()):break
            except Exception:pass
            time.sleep(.1)
        else:raise RuntimeError('New idle service did not pass readiness check')
    except BaseException:
        durable(root/'config.json',old_cfg);durable(root/'active_release.json',previous_active);durable(unit,old_unit)
        control('daemon-reload');control('start','falcons-ai-coordinator.service');raise
    receipt={'release':str(release),'activated':True,'model_loads_enabled':False,'patrol_enabled':False,
             'voice':'Kokoro zf_001','mode':'shadow','voice_motion_live':False,
             'device_receipts_supported':False,'physical_task_completed':False}
    durable(release/'deployment_receipt.json',json.dumps(receipt,indent=2));print(json.dumps(receipt))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--bundle',type=Path,required=True);ap.add_argument('--root',type=Path,required=True);ap.add_argument('--activate',action='store_true')
    args=ap.parse_args()
    if os.name!='posix':raise RuntimeError('Linux board installer only')
    if os.getuid()==0:
        os.execvp('runuser',['runuser','-u','orangepi','--','python3',str(Path(__file__).absolute()),*sys.argv[1:]])
    install(args)


if __name__=='__main__':main()
