"""Fetch only model data from the sherpa-onnx maintainer's pinned public snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import requests

REPO='csukuangfj/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17'
FILES=('model.int8.onnx','tokens.txt','LICENSE','README.md')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    metadata_path=args.out/'upstream_metadata.json'
    if metadata_path.exists():metadata=json.loads(metadata_path.read_text(encoding='utf-8'))
    else:
        response=requests.get(f'https://huggingface.co/api/models/{REPO}?blobs=true',timeout=30)
        response.raise_for_status();metadata=response.json()
        metadata_path.write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    revision=metadata['sha']
    siblings={x['rfilename']:x for x in metadata['siblings']}
    def fetch(name):
        entry=siblings[name];destination=args.out/name
        url=f'https://huggingface.co/{REPO}/resolve/{revision}/{name}'
        partial=destination.with_suffix(destination.suffix+'.partial')
        if not destination.exists():
            for attempt in range(5):
                offset=partial.stat().st_size if partial.exists() else 0
                try:
                    with requests.get(url,stream=True,timeout=(30,60),headers={'Range':f'bytes={offset}-'} if offset else {}) as response:
                        response.raise_for_status()
                        if offset and (response.status_code!=206 or not response.headers.get('Content-Range','').startswith(f'bytes {offset}-')):
                            raise ValueError('Server did not honor resume offset; partial retained')
                        size=offset
                        with partial.open('ab' if offset else 'xb') as file:
                            for chunk in response.iter_content(1024*1024):
                                size+=len(chunk)
                                if size>300*1024*1024:raise ValueError('Unexpected model data size')
                                file.write(chunk)
                    break
                except requests.RequestException:
                    if attempt==4:raise
                    print(f'Retrying {name}, partial retained',flush=True)
            candidate=partial
        else:candidate=destination
        size=candidate.stat().st_size;digest=hashlib.sha256()
        with candidate.open('rb') as file:
            for chunk in iter(lambda:file.read(1024*1024),b''):digest.update(chunk)
        if entry.get('size') is not None and size!=entry['size']:raise ValueError('Size mismatch')
        expected=entry.get('lfs',{}).get('sha256')
        if expected and digest.hexdigest()!=expected:raise ValueError('SHA256 mismatch')
        if not expected:
            blob=hashlib.sha1(b'blob '+str(size).encode()+b'\0'+candidate.read_bytes()).hexdigest()
            if blob!=entry['blobId']:raise ValueError('Git blob hash mismatch')
        if candidate==partial:partial.rename(destination)
        print(f'Downloaded and verified {name}: {size} bytes',flush=True)
        return {'name':name,'url':url,'size':size,'sha256':digest.hexdigest(),'upstream':entry}
    with ThreadPoolExecutor(max_workers=4) as pool:files=list(pool.map(fetch,FILES))
    manifest={'repo':REPO,'revision':revision,'files':files,
        'reference':'https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html',
        'scope':'Data files only; no upstream code executed; no package installation'}
    (args.out/'download_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':main()
