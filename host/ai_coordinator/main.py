"""Loopback API and CLI; hardware adapters are configured separately."""
import argparse
import asyncio
import json
from pathlib import Path
import signal
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.runtime import Runtime
from ai_coordinator.schema import loads,dumps


async def handle(reader,writer,runtime):
    status=200
    try:
        head=await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'),3)
        if len(head)>8192:raise ValueError('headers too large')
        lines=head.decode('ascii').split('\r\n');method,path,version=lines[0].split(' ')
        headers={}
        for line in lines[1:]:
            if not line:continue
            key,value=line.split(':',1);key=key.lower()
            if key in headers:raise ValueError('duplicate header')
            headers[key]=value.strip()
        if 'transfer-encoding' in headers:raise ValueError('chunked requests are unsupported')
        if 'origin' in headers:raise ValueError('Browser-origin commands are unsupported; use the local CLI')
        if method=='POST' and headers.get('content-type','').split(';')[0]!='application/json':raise ValueError('JSON content type required')
        size=int(headers.get('content-length','0'))
        if not 0<=size<=32768:raise ValueError('request too large')
        body=await asyncio.wait_for(reader.readexactly(size),3) if size else b'{}'
        payload=loads(body)
        if not isinstance(payload,dict):raise ValueError('body must be object')
        if method=='GET' and path in ('/health','/status'):result=runtime.snapshot()
        elif method=='GET' and path.startswith('/jobs/'):
            result=runtime.jobs.get(path[6:])
            if result is None:status=404;result={'error':'unknown job'}
        elif method=='POST' and path=='/text':
            if type(payload.get('plan_only',False)) is not bool:raise ValueError('plan_only must be boolean')
            result=runtime.submit_text(payload.get('text'),payload.get('plan_only',False));status=202
        elif method=='POST' and path=='/audio':result=runtime.submit_audio(payload);status=202
        elif method=='POST' and path=='/capture':result=runtime.submit_audio({**payload,'capture':True});status=202
        elif method=='POST' and path=='/image':result=runtime.submit_image(payload);status=202
        elif method=='POST' and path=='/stop':result=runtime.stop()
        elif method=='POST' and path=='/patrol':result=runtime.start_patrol(payload)
        elif method=='POST' and path=='/pickup-confirmed':result=runtime.confirm_pickup(payload)
        elif method=='POST' and path=='/warmup':result=runtime.warmup();status=202
        else:status=404;result={'error':'unknown operation'}
    except (ValueError,UnicodeError,asyncio.IncompleteReadError,asyncio.LimitOverrunError,asyncio.TimeoutError) as exc:
        status=400;result={'error':str(exc)[:300]}
    except Exception as exc:status=503;result={'error':type(exc).__name__+': '+str(exc)[:300]}
    try:
        data=dumps(result).encode('utf-8')
        writer.write(f'HTTP/1.1 {status} Result\r\nContent-Type: application/json; charset=utf-8\r\nContent-Length: {len(data)}\r\nConnection: close\r\n\r\n'.encode()+data)
        await asyncio.wait_for(writer.drain(),2)
    except (ConnectionError,asyncio.TimeoutError):pass
    finally:writer.close();await writer.wait_closed()


async def serve(config):
    runtime=Runtime(config);server=None;end=asyncio.Event()
    try:
        await runtime.start();loop=asyncio.get_running_loop()
        for sig in (signal.SIGINT,signal.SIGTERM):
            try:loop.add_signal_handler(sig,end.set)
            except NotImplementedError:pass
        server=await asyncio.start_server(lambda r,w:handle(r,w,runtime),'127.0.0.1',runtime.cfg.get('port',18765),limit=65536)
        print(dumps({'listening':'127.0.0.1','port':runtime.cfg.get('port',18765),'run_dir':str(runtime.run_dir)}),flush=True)
        await end.wait()
    finally:
        if server:server.close();await server.wait_closed()
        await runtime.close()


def api(url,path,payload=None):
    req=urllib.request.Request(url+path,data=dumps(payload).encode() if payload is not None else None,
        headers={'Content-Type':'application/json'})
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req,timeout=5) as response:return loads(response.read(1024*1024))
    except urllib.error.HTTPError as exc:raise RuntimeError(exc.read().decode('utf-8','replace')) from exc


def wait_job(url,job_id,timeout=120):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        job=api(url,'/jobs/'+job_id)
        if job['status']!='running':return job
        time.sleep(.1)
    raise TimeoutError('Job wait timed out; inspect status before resubmitting')


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('operation',choices=['serve','status','warmup','text','audio','image','capture','stop'])
    ap.add_argument('--text');ap.add_argument('--wav');ap.add_argument('--image');ap.add_argument('--plan-only',action='store_true')
    args=ap.parse_args()
    if args.operation=='serve':asyncio.run(serve(args.config));return
    cfg=loads(args.config.read_text(encoding='utf-8'));url=f"http://127.0.0.1:{cfg.get('port',18765)}"
    if args.operation=='status':result=api(url,'/status')
    else:
        payload={'text':args.text,'wav':args.wav,'image':args.image,'plan_only':args.plan_only}
        result=api(url,'/'+args.operation,payload)
        if result.get('job_id'):result=wait_job(url,result['job_id'])
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
