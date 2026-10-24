"""Read-only readiness and artifact verification; never imports model runtimes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def inspect_config(config,deep=False):
    cfg=json.loads(Path(config).read_text(encoding='utf-8'));checks=[]
    for name,item in cfg.get('artifacts',{}).items():
        p=Path(item['path']);exists=p.is_file()
        valid=exists and p.stat().st_size==item['bytes']
        if valid and deep:
            digest=hashlib.sha256()
            with p.open('rb') as stream:
                for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
            valid=digest.hexdigest()==item['sha256']
        checks.append({'artifact':name,'exists':exists,'valid':valid,'sha256_checked':deep and exists})
    blockers=[]
    if not cfg.get('model_loads_enabled',True):blockers.append('板端模型负载暂停，等待重启原因排查')
    if not cfg.get('microphone',{}).get('device'):blockers.append('USB麦克风尚未配置或验收')
    if cfg.get('announcements',{}).get('mode')!='alsa':blockers.append('播报为日志模式，USB音箱尚未验收')
    if not cfg.get('camera',{}).get('enabled'):blockers.append('相机图像通路尚未配置或验收')
    if cfg.get('gateway',{}).get('mode')!='serial':blockers.append('FPGA任务回执通路尚未联调')
    if cfg.get('mode')!='live':blockers.append('当前为观察/仿真模式，未启用实车任务')
    artifacts_ok=bool(checks) and all(c['valid'] for c in checks)
    return {'language_profile':cfg.get('language_profile'),'artifacts_ok':artifacts_ok,
            'artifact_checks':checks,'model_loads_enabled':cfg.get('model_loads_enabled',True),
            'configured_for_live':not blockers,'physical_acceptance_verified':False,
            'blockers':blockers,'board_models_loaded':False,'devices_opened':False}


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--deep',action='store_true')
    args=ap.parse_args();result=inspect_config(args.config,args.deep)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['artifacts_ok']:raise SystemExit(1)


if __name__=='__main__':main()
