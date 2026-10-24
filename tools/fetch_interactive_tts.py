"""Fetch the user-selected upstream Kokoro v1.1 assets, without installing packages."""
import concurrent.futures
import hashlib
from pathlib import Path
import time
import os
import subprocess
import urllib.request

URL='https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/kokoro-multi-lang-v1_1.tar.bz2'
SIZE=364816464
SHA256='a3f4c73d043860e3fd2e5b06f36795eb81de0fc8e8de6df703245edddd87dbad'


def main():
    root=Path(__file__).resolve().parents[1]/'build/ai_interactive_20261024'
    root.mkdir(parents=True,exist_ok=True)
    target=root/'kokoro-multi-lang-v1_1.tar.bz2'
    if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest()==SHA256:
        print('Already verified:',target);return
    parts=root/('tts_parts_'+SHA256[:12]);parts.mkdir(exist_ok=True)
    chunk=2*1024*1024
    def fetch(index):
        start=index*chunk;end=min(SIZE-1,start+chunk-1)
        part=parts/('%04d.part'%index)
        if part.exists() and part.stat().st_size==end-start+1:return part
        for attempt in range(3):
            try:
                if os.name=='nt':
                    env=dict(os.environ)
                    # curl does not read Windows' configured system proxy.
                    # Carry it through the environment, never into command logs.
                    proxy=urllib.request.getproxies().get('https')
                    if proxy:env['https_proxy']=proxy
                    p=subprocess.run(['curl.exe','--fail','--location','--silent','--show-error',
                        '--connect-timeout','10','--max-time','120','--max-filesize',str(end-start+1),
                        '--range','%d-%d'%(start,end),'--output',str(part),URL],capture_output=True,text=True,env=env)
                    if p.returncode or not part.exists() or part.stat().st_size!=end-start+1:
                        raise RuntimeError('Asset range download failed: '+p.stderr[-300:])
                    return part
                opener=urllib.request.build_opener()
                req=urllib.request.Request(URL,headers={'Range':'bytes=%d-%d'%(start,end)})
                with opener.open(req,timeout=30) as response:
                    if response.status!=206 or not response.headers.get('Content-Range','').startswith('bytes %d-'%start):
                        raise RuntimeError('Server did not honor bounded byte range')
                    raw=response.read(end-start+2)
                if len(raw)!=end-start+1:raise RuntimeError('Truncated asset range')
                part.write_bytes(raw)
                return part
            except Exception:
                if attempt==2:raise
                time.sleep(attempt+1)
    count=(SIZE+chunk-1)//chunk
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        futures={pool.submit(fetch,i):i for i in range(count)}
        done=0
        for future in concurrent.futures.as_completed(futures):
            future.result();done+=1
            if done%8==0 or done==count:print('TTS asset ranges: %d/%d'%(done,count),flush=True)
    temporary=target.with_suffix('.download')
    with temporary.open('wb') as out:
        for i in range(count):out.write((parts/('%04d.part'%i)).read_bytes())
    if temporary.stat().st_size!=SIZE or hashlib.sha256(temporary.read_bytes()).hexdigest()!=SHA256:
        raise ValueError('TTS archive checksum mismatch')
    temporary.replace(target)
    print('Verified:',target,SHA256,flush=True)


if __name__=='__main__':main()
