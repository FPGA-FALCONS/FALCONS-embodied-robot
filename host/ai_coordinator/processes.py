"""Owned, killable workers and an optional managed llama.cpp process."""
import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

PREFIX=b'FALCONS_RPC '


class Worker:
    def __init__(self,kind,config,log_dir,command=None,startup_timeout=60):
        self.kind=kind;self.config=Path(config);self.log_dir=Path(log_dir)
        self.command=command or [sys.executable,str(Path(__file__).with_name('worker.py')),'--config',str(config),'--kind',kind]
        self.proc=None;self.lock=asyncio.Lock();self.counter=0;self.startup_timeout=startup_timeout
        self.load_s=None;self.log=None

    async def _line(self):
        while True:
            line=await self.proc.stdout.readline()
            if not line:raise RuntimeError(self.kind+' worker exited')
            if line.startswith(PREFIX):return json.loads(line[len(PREFIX):])
            self.log.write(line.decode('utf-8','replace')[:65536]);self.log.flush()

    async def _start(self):
        if self.proc and self.proc.returncode is None:return
        self.log_dir.mkdir(parents=True,exist_ok=True)
        if self.log:self.log.close()
        self.log=(self.log_dir/(self.kind+'.sdk.log')).open('a',encoding='utf-8')
        kwargs={'start_new_session':True} if os.name=='posix' else {'creationflags':getattr(subprocess,'CREATE_NO_WINDOW',0)}
        self.proc=await asyncio.create_subprocess_exec(*self.command,stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,stderr=self.log,limit=131072,**kwargs)
        try:
            ready=await asyncio.wait_for(self._line(),self.startup_timeout)
            if ready.get('ready') is not True:raise RuntimeError('worker did not become ready')
            self.load_s=ready.get('load_s')
        except BaseException:await self.close();raise

    async def call(self,request,timeout=15):
        # The request deadline includes time queued behind another request.
        async def invoke():
            async with self.lock:
                await self._start();self.counter+=1;identifier=self.counter
                self.proc.stdin.write((json.dumps({'id':identifier,**request},ensure_ascii=False)+'\n').encode('utf-8'))
                await self.proc.stdin.drain();response=await self._line()
                if response.get('id')!=identifier:raise RuntimeError('worker response correlation mismatch')
                if not response.get('ok'):raise RuntimeError(response.get('error','worker failure'))
                return response
        try:return await asyncio.wait_for(invoke(),timeout)
        except BaseException:
            # Only this instance's process group is stopped, never another experiment.
            await self.close();raise

    async def close(self):
        proc=self.proc;self.proc=None
        if proc and proc.returncode is None:
            if os.name=='posix':
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
            else:proc.terminate()
            try:await asyncio.wait_for(proc.wait(),2)
            except asyncio.TimeoutError:
                if os.name=='posix':
                    try:os.killpg(proc.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                else:proc.kill()
                await proc.wait()
        if self.log:self.log.close();self.log=None


class ModelServer:
    def __init__(self,cfg,log_dir):self.cfg=cfg;self.log_dir=Path(log_dir);self.proc=None;self.log=None

    async def start(self):
        command=self.cfg.get('server_command')
        if not command:return
        if not isinstance(command,list) or not all(isinstance(x,str) for x in command):raise ValueError('server_command must be argv list')
        self.log_dir.mkdir(parents=True,exist_ok=True);self.log=(self.log_dir/'llama.sdk.log').open('a',encoding='utf-8')
        kwargs={'start_new_session':True} if os.name=='posix' else {'creationflags':getattr(subprocess,'CREATE_NO_WINDOW',0)}
        self.proc=await asyncio.create_subprocess_exec(*command,cwd=self.cfg.get('server_cwd'),stdout=self.log,stderr=self.log,**kwargs)
        url=self.cfg['url'].rstrip('/')+'/health';deadline=time.monotonic()+90
        def healthy():
            try:
                opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
                with opener.open(url,timeout=.5) as response:return response.status==200
            except Exception:return False
        while time.monotonic()<deadline:
            if self.proc.returncode is not None:raise RuntimeError('managed model server exited; see SDK log')
            if await asyncio.to_thread(healthy):return
            await asyncio.sleep(.1)
        raise TimeoutError('managed model server startup timeout')

    async def close(self):
        if self.proc and self.proc.returncode is None:
            self.proc.terminate()
            try:await asyncio.wait_for(self.proc.wait(),3)
            except asyncio.TimeoutError:self.proc.kill();await self.proc.wait()
        self.proc=None
        if self.log:self.log.close();self.log=None
