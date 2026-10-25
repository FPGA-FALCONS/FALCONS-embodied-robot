"""Build a self-verifying source/audio release; no weights or environment installs."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--speech',type=Path,required=True);ap.add_argument('--chat-template',type=Path,required=True)
    args=ap.parse_args();repo=Path(__file__).resolve().parents[2]
    if args.out.exists():raise FileExistsError('Retain previous deployment bundles')
    from ai_coordinator.announcements import validate_assets
    validate_assets(args.speech)
    files={}
    for folder in ('host/ai_coordinator','host/voice_bridge','host/fx2_camera',
                   'host/semantic_nav','host/usb_camera_v4l2','host/vision_training',
                   'host/vision_bridge','host/voice_task_demo'):
        for p in (repo/folder).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.py','.md','.json','.txt','.ps1','.sh','.cpp'):
                files[p.relative_to(repo).as_posix()]=p
    for p in args.speech.iterdir():
        if p.is_file() and p.suffix in ('.wav','.json'):files['speech/'+p.name]=p
    files['models/shared_chat_template.jinja']=args.chat_template
    files['host/README.md']=repo/'host/README.md'
    for p in (repo/'doc').glob('*2026102[45].md'):
        if p.is_file():files['doc/'+p.name]=p
    manifest={name:{'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'bytes':path.stat().st_size} for name,path in files.items()}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(args.out,'x',zipfile.ZIP_DEFLATED) as archive:
        for name,path in files.items():archive.write(path,name)
        archive.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    args.out.with_suffix('.manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'files':len(files),'bundle_bytes':args.out.stat().st_size,'sha256':hashlib.sha256(args.out.read_bytes()).hexdigest()}))


if __name__=='__main__':
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    main()
