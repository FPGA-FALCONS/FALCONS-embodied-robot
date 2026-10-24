"""Private, bounded llama.cpp text parser. No remote inference or hardware output."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import requests

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'voice_bridge'))
from preview import parse_task


class PrivateParser:
    def __init__(self,model,work,out,server='D:/llama/llama-server.exe',deadline=15):
        self.model=Path(model);self.work=Path(work);self.out=Path(out)
        self.server=server;self.deadline=deadline;self.proc=None;self.log=None
        self.session=requests.Session();self.session.trust_env=False

    def __enter__(self):
        self.out.mkdir(parents=True,exist_ok=False)
        sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
        self.url=f'http://127.0.0.1:{port}'
        command=[self.server,'-m',os.path.relpath(self.model,self.work),'-c','2048','-np','1',
            '--host','127.0.0.1','--port',str(port),'--jinja','--reasoning','off','--reasoning-budget','0',
            '--chat-template-file','shared_chat_template.jinja','--cache-ram','0','--ctx-checkpoints','0',
            '--check-tensors','--no-webui']
        self.log=(self.out/'server.log').open('x',encoding='utf-8')
        start=time.perf_counter()
        try:
            self.proc=subprocess.Popen(command,cwd=self.work,stdout=self.log,stderr=subprocess.STDOUT,
                                       creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            for _ in range(240):
                if self.proc.poll() is not None:raise RuntimeError('Private llama.cpp server failed; see server.log')
                try:
                    if self.session.get(self.url+'/health',timeout=.5).status_code==200:break
                except requests.RequestException:pass
                time.sleep(.25)
            else:raise TimeoutError('Model startup timed out')
            self.system=(self.work/'system.txt').read_text(encoding='utf-8')
            runtime={'command':command,'load_seconds':time.perf_counter()-start,
                     'props':self.session.get(self.url+'/props',timeout=5).json(),
                     'scope':'Host GPU Q8 only; no RK3588 inference'}
            (self.out/'runtime.json').write_text(json.dumps(runtime,ensure_ascii=False,indent=2),encoding='utf-8')
            return self
        except BaseException:
            self.__exit__(None,None,None);raise

    def parse(self,text,deadline=None):
        if not self.proc or self.proc.poll() is not None:raise RuntimeError('Private parser is not running')
        limit=self.deadline if deadline is None else deadline
        expired=threading.Event()
        def terminate():
            expired.set()
            if self.proc.poll() is None:self.proc.terminate()
        timer=threading.Timer(limit,terminate);timer.daemon=True
        start=time.perf_counter();timer.start()
        try:
            response=self.session.post(self.url+'/v1/chat/completions',json={
                'messages':[{'role':'system','content':self.system},{'role':'user','content':text}],
                'temperature':0,'top_p':1,'top_k':0,'min_p':0,'repeat_penalty':1,'seed':20261024,
                'max_tokens':80,'cache_prompt':False,'chat_template_kwargs':{'enable_thinking':False}},
                timeout=(2,max(.01,limit)))
            response.raise_for_status();response=response.json();choice=response['choices'][0]
            if expired.is_set() or time.perf_counter()-start>limit:raise TimeoutError('Parser deadline exceeded')
            content=choice['message'].get('content') or ''
            result={'content':content,'finish_reason':choice.get('finish_reason'),'usage':response.get('usage'),
                    'elapsed_s':round(time.perf_counter()-start,4),'reasoning':choice['message'].get('reasoning_content') or ''}
            try:
                if choice.get('finish_reason')!='stop':raise ValueError('incomplete answer')
                result['task']=parse_task(content,text);result['valid']=True
            except (ValueError,TypeError) as exc:
                result.update(task={'i':'unknown'},valid=False,error=str(exc))
            return result
        except requests.RequestException as exc:
            if expired.is_set():raise TimeoutError('Parser deadline exceeded; private worker stopped') from exc
            raise
        finally:timer.cancel()

    def __exit__(self,*args):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        if self.log:self.log.close()
        self.session.close()
