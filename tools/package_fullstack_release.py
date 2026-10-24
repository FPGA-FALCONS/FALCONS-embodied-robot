"""Create a traceable teammate bundle from a committed project and build outputs.

No model loads, board access or programmer invocation. Source Git archive and
exact raw build inputs are both retained to account for Git EOL normalization.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--build-status',type=Path,required=True);ap.add_argument('--build-inputs',type=Path,required=True)
    ap.add_argument('--ai-bundle',type=Path,required=True);ap.add_argument('--model-manifest',type=Path,required=True)
    ap.add_argument('--bitstream',type=Path);ap.add_argument('--revision',default='HEAD');args=ap.parse_args()
    repo=Path(__file__).resolve().parents[1]
    commit=subprocess.check_output(['git','rev-parse',args.revision],cwd=repo,text=True).strip()
    inputs=json.loads(args.build_inputs.read_text(encoding='utf-8'))
    for name,item in inputs.items():
        if digest(repo/name)!=item['sha256']:raise ValueError('Build input changed: '+name)
    status=json.loads(args.build_status.read_text(encoding='utf-8'))
    if args.bitstream and (not status.get('build_complete') or digest(args.bitstream)!=status.get('bitstream_sha256')):
        raise ValueError('Bitstream is not the completed verified build')
    if args.out.exists():raise FileExistsError('Retain prior release bundle')
    root=args.out.with_suffix('');root.mkdir(parents=True,exist_ok=False)
    for folder in ('source','ai','fpga','evidence'):(root/folder).mkdir()
    subprocess.run(['git','archive','--format=zip','--output='+str(root/'source'/f'FALCONS-{commit[:12]}.zip'),commit],cwd=repo,check=True)
    # These raw inputs are the exact bytes used by Gowin, including line endings.
    with zipfile.ZipFile(root/'fpga/build_inputs_exact.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in inputs:archive.write(repo/name,name)
    shutil.copy2(args.build_inputs,root/'fpga/BUILD_INPUTS.json')
    shutil.copy2(args.ai_bundle,root/'ai'/args.ai_bundle.name)
    shutil.copy2(args.model_manifest,root/'MODEL_MANIFEST.json')
    shutil.copy2(repo/'doc/全栈上板与队友下载_20261024.md',root/'START_HERE.md')
    for folder in ('ai_release_20261024','voice_motion_release_20261024'):
        shutil.copytree(repo/'doc/evidence'/folder,root/'evidence'/folder)
    if args.bitstream:
        name='falcons_motion_SRAM_only.fs';shutil.copy2(args.bitstream,root/'fpga'/name)
        status['bitstream_file']='fpga/'+name
    status.update(source_commit=commit,programming_operation='SRAM Program only; no Flash',
                  fpga_programmed=False,physical_vehicle_tested=False,
                  timing_signoff=bool(status.get('timing_passed')),
                  source_archive_note='Git archive is canonical source; exact build input bytes retained separately')
    (root/'BUILD_STATUS.json').write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
    (root/'SOURCE_COMMIT.txt').write_text(commit+'\n',encoding='ascii')
    inventory={'host_modules':sorted(p.name for p in (repo/'host').iterdir() if p.is_dir() and p.name!='__pycache__'),
               'fpga_module_packages':sorted(p.name for p in (repo/'vendor/verilog-reusable-modules/modules').iterdir() if p.is_dir())}
    (root/'MODULE_INVENTORY.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf-8')
    manifest={p.relative_to(root).as_posix():{'bytes':p.stat().st_size,'sha256':digest(p)}
              for p in root.rglob('*') if p.is_file()}
    (root/'SHA256SUMS').write_text(''.join(v['sha256']+'  '+name+'\n' for name,v in sorted(manifest.items())),encoding='utf-8')
    with zipfile.ZipFile(args.out,'x',zipfile.ZIP_DEFLATED) as archive:
        for p in root.rglob('*'):
            if p.is_file():archive.write(p,p.relative_to(root).as_posix())
    print(json.dumps({'archive':str(args.out),'bytes':args.out.stat().st_size,'sha256':digest(args.out),
                      'source_commit':commit,'files':len(manifest)+1,'timing_signoff':status['timing_signoff']}))


if __name__=='__main__':main()
