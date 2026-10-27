"""Stage and validate an inspection release using the RK's existing models.

No packages, model weights, drivers, FPGA Flash or FX2 EEPROM are installed.
Default operation stages/tests only. --activate updates the existing user
service with its existing transactional backup/rollback installer.
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
import zipfile


def prepare_config(current,defaults,release):
    cfg=copy.deepcopy(current);inspection={**copy.deepcopy(defaults),**copy.deepcopy(cfg.get('inspection',{}))}
    inspection['enabled']=True
    # Older deployments carry a complete inspection.camera dictionary. Merge
    # new camera defaults below it, rather than losing newly introduced fields.
    camera={**copy.deepcopy(defaults.get('camera',{})),**inspection['camera'],**cfg.get('camera',{})}
    camera.update(enabled=True,backend='v4l2_native',native_helper=str(release/'host/usb_camera_v4l2/native/falcons-uvc-capture'),
                  storage=str(Path(cfg['root'])/'camera_frames'))
    if camera.get('selector'):camera.pop('device',None)
    inspection['camera']=copy.deepcopy(camera)
    cfg['camera']=camera;cfg['inspection']=inspection
    # Identities already recorded in the vehicle's 2026-10-24 USB inventory.
    # Resolve ALSA card names afresh for each use, not by the current socket.
    cfg.setdefault('audio_roles',{'microphone':{'vid':0x4c4a,'pid':0x4c55},'speaker':{'vid':0x0d8c,'pid':0x0012}})
    # Existing ASR/semantic/Qwen/Kokoro paths, motion settings and audio roles
    # are preserved byte-for-byte as JSON values; no model substitution.
    return cfg


def apply_ground_homography(current, profile):
    """Explicit opt-in only; existing deployments never switch geometry implicitly."""
    from .inspection.calibration import Calibration
    cfg=copy.deepcopy(current);camera=cfg.get('camera',{})
    serial=camera.get('selector',{}).get('serial');size=camera.get('expected_size',[])
    if profile.get('projection_model')!='undistorted_homography':
        raise ValueError('Explicit undistorted homography calibration profile required')
    if (camera.get('focus')!=profile.get('focus') or camera.get('autofocus') is not False
            or profile.get('autofocus') is not False):
        raise ValueError('Camera and homography manual focus must match')
    problems=Calibration(profile).geometry_problems(size,serial)
    if problems:raise ValueError('Invalid homography geometry: '+','.join(problems))
    cfg['inspection']['calibration']=copy.deepcopy(profile)
    return cfg


def stage(bundle,config,release):
    current=json.loads(config.read_text(encoding='utf-8'))
    if config.resolve().parent!=Path(current['root']).resolve():raise ValueError('Existing deployment config/root mismatch')
    release.mkdir(parents=True,exist_ok=False)
    with zipfile.ZipFile(bundle) as archive:
        manifest=json.loads(archive.read('manifest.json'))
        if set(archive.namelist())!=set(manifest)|{'manifest.json'}:raise ValueError('Exact source manifest required')
        for name,entry in manifest.items():
            target=(release/name).resolve()
            if not target.is_relative_to(release.resolve()) or not name.startswith('host/'):raise ValueError('Source bundle path escape')
            if entry['bytes']>32*1024*1024:raise ValueError('Source member exceeds limit')
            data=archive.read(name)
            if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Bundle SHA256 mismatch')
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    compiler=shutil.which('cc')
    if not compiler:raise RuntimeError('Existing C compiler required; nothing will be installed')
    source=release/'host/usb_camera_v4l2/native/capture.c';helper=source.with_name('falcons-uvc-capture')
    subprocess.run([compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror',str(source),'-o',str(helper)],check=True)
    defaults=json.loads((release/'host/ai_coordinator/inspection/inspection.example.json').read_text(encoding='utf-8'))
    cfg=prepare_config(current,defaults,release)
    staged=release/'config.staged.json';staged.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    manifest[helper.relative_to(release).as_posix()]={'sha256':hashlib.sha256(helper.read_bytes()).hexdigest(),'bytes':helper.stat().st_size}
    (release/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    env={**os.environ,'PYTHONPATH':str(Path(cfg['root'])/'python')+':'+str(release/'host'),'PYTHONDONTWRITEBYTECODE':'1'}
    for folder in ['ai_coordinator/tests','fx2_camera/tests','semantic_nav']:
        with (release/(folder.replace('/','_')+'.tests.log')).open('w') as log:
            subprocess.run([sys.executable,'-m','unittest','discover','-s',str(release/'host'/folder)],
                           env=env,cwd=release,stdout=log,stderr=subprocess.STDOUT,check=True)
    return cfg


def activate(config,release,cfg,timeout=180):
    # Import this release's installer so its __file__-based manifest check and
    # systemd ExecStart both point to the staged code, never the previous one.
    if Path(__file__).resolve().parents[2]!=release.resolve():raise ValueError('Activate using the staged release module')
    from .install_autostart import install,unit_text,atomic
    result=install(config,cfg,unit_text(release/'host/ai_coordinator/start_board.sh',config))
    backup=Path(result['backup']);deadline=time.monotonic()+timeout
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}));url='http://127.0.0.1:%d/status'%cfg.get('port',18765)
    try:
        while time.monotonic()<deadline:
            try:
                with opener.open(url,timeout=2) as response:status=json.loads(response.read(1024*1024))
                if status.get('inspection',{}).get('enabled') and status.get('startup',{}).get('ready'):
                    result.update(models_ready=True,inspection_enabled=True,automatic_motion_started=False)
                    (release/'activation.json').write_text(json.dumps(result,indent=2),encoding='utf-8');return result
            except (OSError,ValueError):pass
            time.sleep(.5)
        raise TimeoutError('New release did not load required original models within deadline')
    except BaseException:
        subprocess.run(['systemctl','--user','stop','falcons-ai-coordinator.service'],check=False)
        atomic(config,(backup/'config.previous.json').read_bytes())
        atomic(Path.home()/'.config/systemd/user/falcons-ai-coordinator.service',(backup/'service.previous.txt').read_bytes())
        if (backup/'active.previous.json').exists():atomic(Path(cfg['root'])/'active_release.json',(backup/'active.previous.json').read_bytes())
        subprocess.run(['systemctl','--user','daemon-reload'],check=True)
        subprocess.run(['systemctl','--user','start','falcons-ai-coordinator.service'],check=True)
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--release',type=Path,required=True);parser.add_argument('--activate',action='store_true')
    parser.add_argument('--calibration-profile',type=Path,
                        help='Explicit installed undistorted-H profile; preserves all other settings')
    args=parser.parse_args()
    if args.bundle:
        cfg=stage(args.bundle,args.config,args.release)
        print(json.dumps({'staged':str(args.release),'existing_service_changed':False}),flush=True)
    else:cfg=json.loads((args.release/'config.staged.json').read_text(encoding='utf-8'))
    if args.calibration_profile:
        cfg=apply_ground_homography(cfg,json.loads(args.calibration_profile.read_text(encoding='utf-8')))
        (args.release/'config.staged.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if args.activate:print(json.dumps(activate(args.config,args.release,cfg)),flush=True)

if __name__=='__main__':main()
