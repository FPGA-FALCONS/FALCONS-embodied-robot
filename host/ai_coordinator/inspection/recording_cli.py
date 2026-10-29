"""List, replay, recover or explicitly delete owned RK inspection recordings."""
import argparse
import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import re
from urllib.parse import unquote,urlsplit
from .recording import Lease,MEDIA,SESSION_ID,FORMAT,recover,session_path


def catalog(root):
    rows=[]
    if not Path(root).exists():return rows
    for p in sorted(Path(root).iterdir(),reverse=True):
        if p.is_dir() and not p.is_symlink() and SESSION_ID.fullmatch(p.name):
            try:rows.append({'id':p.name,**json.loads((p/'manifest.json').read_text(encoding='utf-8'))})
            except (OSError,ValueError):rows.append({'id':p.name,'status':'unreadable'})
    return rows


def delete(root,identifier):
    path=session_path(root,identifier);lease=Lease(path/'.lock')
    try:
        if json.loads((path/'manifest.json').read_text(encoding='utf-8')).get('format')!=FORMAT:
            raise ValueError('Unrecognized archive format; inspect manually')
        allowed={'.lock','manifest.json','manifest.json.tmp','report.json','report.json.tmp',
                 'report.latest.json','report.latest.json.tmp','frames.jsonl','frames.jsonl.tmp','encoder.log'}
        files=list(path.iterdir())
        if any(p.is_symlink() or not p.is_file() or (p.name not in allowed and not MEDIA.fullmatch(p.name)) for p in files):
            raise ValueError('Unexpected files in recording; inspect manually before deleting')
        for p in files:
            if p.name!='.lock':p.unlink()
    finally:lease.close()
    (path/'.lock').unlink();path.rmdir()


HTML='''<!doctype html><meta charset="utf-8"><title>巡检录像同步回放</title>
<style>body{background:#101828;color:#eee;font:16px sans-serif;margin:24px}video{width:49%;background:#000}button,input,select{margin:8px;padding:8px}#seek{width:65%}pre{white-space:pre-wrap}</style>
<h2>巡检录像：摄像头与地图同步回放</h2><select id="sessions"></select><button id="open">打开</button>
<div><video id="camera" muted playsinline></video><video id="map" muted playsinline></video></div>
<button id="play">播放 / 暂停</button><input id="seek" type="range" min="0" step="0.1" value="0"><span id="clock"></span>
<div><a id="report" target="_blank">巡检报告 JSON</a> · <a id="metadata" target="_blank">录像清单 / 分段下载</a> · <a id="timeline" target="_blank">逐帧时间与数据时效</a></div>
<pre id="info"></pre><script>
const $=id=>document.getElementById(id),c=$('camera'),m=$('map');let meta=null,parts=[],offsets=[],part=0,playing=false,loading=false;
async function list(){let rows=await(await fetch('/api')).json();for(let r of rows){let o=document.createElement('option');o.value=r.id;o.textContent=r.start_utc+' | '+r.status+' | '+(r.duration_s||0).toFixed(1)+'秒';$('sessions').appendChild(o)}}
async function load(index,t=0){loading=true;part=index;let id=$('sessions').value,p=parts[index];c.src='/'+id+'/'+p.camera;m.src='/'+id+'/'+p.map;await Promise.all([c,m].map(v=>new Promise((ok,no)=>{v.onloadedmetadata=ok;v.onerror=()=>no(Error('录像不可读取'));v.load()})));c.currentTime=m.currentTime=Math.min(p.frames/10-.001,Math.max(0,t+.0001));loading=false;if(playing)await Promise.all([c.play(),m.play()])}
$('open').onclick=async()=>{playing=false;c.pause();m.pause();let id=$('sessions').value;if(!id)return;meta=await(await fetch('/api/'+id)).json();parts=meta.segments||[];let n=0;offsets=parts.map(p=>{let x=n;n+=p.frames/10;return x});$('seek').max=n;$('info').textContent=JSON.stringify({状态:meta.status,终止原因:meta.end_reason,时基:meta.time_base,总帧数:meta.frames},null,2);for(let [id,file] of [['report',meta.status==='recording'?'report.latest.json':'report.json'],['metadata','manifest.json'],['timeline','frames.jsonl']])$(id).href='/'+$('sessions').value+'/'+file;if(parts.length)await load(0)};
$('play').onclick=async()=>{if(!parts.length||loading)return;playing=!playing;if(playing)await Promise.all([c.play(),m.play()]);else{c.pause();m.pause()}};
$('seek').onchange=async()=>{if(!parts.length||loading)return;let t=+$('seek').value,i=offsets.findLastIndex(x=>x<=t);c.pause();m.pause();await load(Math.max(0,i),Math.min(parts[i].frames/10,t-offsets[i]))};
c.onended=async()=>{if(playing&&part+1<parts.length)await load(part+1);else{playing=false;c.pause();m.pause()}};
function sync(){if(meta&&parts.length&&!loading){let t=Math.min(c.currentTime,parts[part].frames/10),global=offsets[part]+t;$('seek').value=global;$('clock').textContent=new Date(meta.start_unix_ns/1e6+global*1000).toISOString()+'  +'+global.toFixed(1)+'s';if(playing&&Math.abs(m.currentTime-t)>.03)m.currentTime=t}requestAnimationFrame(sync)}list();sync();
</script>'''


def handler(root):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            try:
                route=unquote(urlsplit(self.path).path)
                if route=='/':return self.reply(HTML.encode('utf-8'),'text/html; charset=utf-8')
                if route=='/api':return self.reply(json.dumps(catalog(root),ensure_ascii=False).encode(),'application/json')
                if route.startswith('/api/'):
                    path=session_path(root,route[5:])/'manifest.json'
                else:
                    parts=route.strip('/').split('/')
                    if len(parts)!=2:raise ValueError('Unknown route')
                    name=parts[1]
                    if name not in ('manifest.json','report.json','report.latest.json','frames.jsonl') and not MEDIA.fullmatch(name):raise ValueError('Unknown artifact')
                    path=session_path(root,parts[0])/name
                if path.is_symlink() or not path.is_file():raise ValueError('Missing artifact')
                size=path.stat().st_size;start,end=0,size-1;partial=self.headers.get('Range')
                if partial:
                    match=re.fullmatch(r'bytes=(\d+)-(\d*)',partial)
                    if not match:raise ValueError('One byte range required')
                    start=int(match[1]);end=min(end,int(match[2]) if match[2] else end)
                    if not 0<=start<=end<size:raise ValueError('Invalid range')
                self.send_response(206 if partial else 200);self.send_header('Content-Type','video/mp4' if path.suffix=='.mp4' else 'application/json; charset=utf-8')
                self.send_header('Cache-Control','no-store')
                self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
                if partial:self.send_header('Content-Range','bytes %d-%d/%d'%(start,end,size))
                self.end_headers()
                with path.open('rb') as stream:
                    stream.seek(start);remaining=end-start+1
                    while remaining:
                        data=stream.read(min(65536,remaining))
                        if not data:break
                        self.wfile.write(data);remaining-=len(data)
            except (OSError,ValueError):self.send_error(404)
        def reply(self,data,content_type):
            self.send_response(200);self.send_header('Content-Type',content_type);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def log_message(self,*args):pass
    return Handler


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path);p.add_argument('--config',type=Path)
    sub=p.add_subparsers(dest='operation',required=True);sub.add_parser('list')
    show=sub.add_parser('show');show.add_argument('identifier')
    remove=sub.add_parser('delete');remove.add_argument('identifier');remove.add_argument('--yes',action='store_true',required=True)
    sub.add_parser('recover');serve=sub.add_parser('serve');serve.add_argument('--bind',default='127.0.0.1');serve.add_argument('--port',type=int,default=18766)
    args=p.parse_args()
    if args.root:root=args.root
    elif args.config:root=Path(json.loads(args.config.read_text(encoding='utf-8'))['root'])/'inspection_recordings'
    else:p.error('--root or --config is required')
    if args.operation=='list':print(json.dumps(catalog(root),ensure_ascii=False,indent=2))
    elif args.operation=='show':print((session_path(root,args.identifier)/'manifest.json').read_text(encoding='utf-8'))
    elif args.operation=='delete':delete(root,args.identifier);print('Deleted '+args.identifier)
    elif args.operation=='recover':print(json.dumps(recover(root)))
    elif args.operation=='serve':
        server=ThreadingHTTPServer((args.bind,args.port),handler(root));print('http://%s:%d'%(args.bind,args.port),flush=True)
        try:server.serve_forever()
        except KeyboardInterrupt:pass
        finally:server.server_close()

if __name__=='__main__':main()
