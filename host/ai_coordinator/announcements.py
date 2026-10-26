"""Bounded, cancellable cached speech. Never authorizes or completes a mission."""
import asyncio
from collections import deque
import hashlib
import json
from pathlib import Path
import re
import time
import wave

NAMES={'tape':'胶带','screwdriver':'螺丝刀','ad_calcium_milk':'AD钙奶','nutri_express':'营养快线'}
PHRASES={
    **{'accepted_'+k:'已接收寻找'+v+'的任务。' for k,v in NAMES.items()},
    **{'search_'+k:'开始寻找'+v+'。' for k,v in NAMES.items()},
    **{'found_'+k:'已找到'+v+'。' for k,v in NAMES.items()},
    'returning':'正在返回起点。','completed':'任务已完成。',
    'stop_requested':'已请求停止任务。','timed_out':'任务超时，请检查。',
    'not_found':'搜索结束，未找到目标。','failed':'任务中断，请检查。',
    'patrol_started':'开始沿设定路径巡航。','patrol_paused':'发现目标，已暂停巡航。',
    'approaching':'正在靠近目标。','pickup_wait':'已到目标附近，请确认拾取完成。',
    'pickup_confirmed':'已收到人工拾取确认。','patrol_resumed':'继续巡航。',
}

def catalog():
    return {('sim_' if simulated else '')+key:('模拟任务。' if simulated else '')+text
            for simulated in (False,True) for key,text in PHRASES.items()}

def validate_assets(directory):
    directory=Path(directory).resolve();manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8-sig'))
    if manifest.get('v')!=1:raise ValueError('Unsupported speech manifest')
    for key,text in catalog().items():
        entry=manifest['clips'][key];path=(directory/entry['file']).resolve()
        if path.parent!=directory or path.suffix!='.wav' or entry['text']!=text:raise ValueError('Speech catalog mismatch')
        if not path.is_file() or path.stat().st_size>512000:raise ValueError('Missing or oversized speech clip')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=entry['sha256']:raise ValueError('Speech checksum mismatch')
        with wave.open(str(path),'rb') as wav:
            if wav.getparams()[:3]!=(1,2,16000) or wav.getcomptype()!='NONE':raise ValueError('Expected PCM16 mono 16kHz')
            duration=wav.getnframes()/wav.getframerate()
            if not .05<=duration<=8:raise ValueError('Invalid speech duration')
        entry['path']=path;entry['duration_s']=duration
    return manifest['clips']


class ALSAPlayer:
    def __init__(self,device,selector=None):
        if selector is None and (not isinstance(device,str) or not re.fullmatch(r'(?:plug)?hw:CARD=[A-Za-z0-9_]+,DEV=\d+',device)):
            raise ValueError('Choose an explicit ALSA CARD name and DEV number')
        self.device=device;self.selector=selector

    async def __call__(self,clip,event):
        from .device_roles import audio_device
        device=audio_device(self.selector,'speaker',self.device)
        proc=await asyncio.create_subprocess_exec('aplay','-q','-D',device,str(clip['path']),
            stdout=asyncio.subprocess.DEVNULL,stderr=asyncio.subprocess.DEVNULL)
        try:
            code=await asyncio.wait_for(proc.wait(),clip['duration_s']+2)
            if code:raise RuntimeError('ALSA playback failed (exit '+str(code)+')')
        finally:
            if proc.returncode is None:
                proc.terminate()
                try:await asyncio.wait_for(proc.wait(),.3)
                except asyncio.TimeoutError:proc.kill();await proc.wait()


class Announcements:
    def __init__(self,cfg,audit,player=None):
        self.cfg=cfg;self.audit=audit;self.mode=cfg.get('mode','log')
        if self.mode not in ('off','log','alsa'):raise ValueError('Invalid announcements mode')
        self.clips={};self.player=player;self.last_error=None
        if self.mode=='alsa':
            try:
                self.clips=validate_assets(cfg['assets'])
                self.player=player or ALSAPlayer(cfg.get('device'),selector=cfg.get('selector'))
            except (OSError,ValueError,KeyError) as exc:
                self.mode='log';self.last_error=type(exc).__name__+': '+str(exc)[:200]
                self.audit({'type':'speech_unavailable','fallback':'log','error':self.last_error})
        self.queue=deque();self.seen=deque(maxlen=256);self.task_id=None;self.epoch=0
        self.pump=None;self.playing=None;self.current=None;self.closed=False
        self.capture_active=False;self.quiet_until=0
        self.changed=asyncio.Event()

    def _clear(self,reason):
        self.epoch+=1
        while self.queue:self.audit({'type':'speech_dropped','reason':reason,'event':self.queue.popleft()})
        if self.playing:self.playing.cancel()

    def emit(self,event):
        if self.closed or self.mode=='off':return
        key=('sim_' if event['simulated'] else '')+event['code']
        if key not in catalog():raise ValueError('Unknown speech phrase')
        if event['id'] in self.seen:return
        self.seen.append(event['id'])
        if event['task_id']!=self.task_id:
            self._clear('task_replaced');self.task_id=event['task_id']
        if event['code'] in ('stop_requested','timed_out','not_found','failed'):self._clear('terminal_status')
        item={**event,'clip':key,'text':catalog()[key],'queued_at_s':time.monotonic(),'epoch':self.epoch}
        self.audit({'type':'speech_queued','event':item})
        if len(self.queue)>=8:self.audit({'type':'speech_dropped','reason':'queue_full','event':self.queue.popleft()})
        self.queue.append(item);self.changed.set()
        if self.pump is None or self.pump.done():self.pump=asyncio.create_task(self._run())

    @property
    def microphone_blocked(self):
        return self.mode=='alsa' and (self.playing is not None or time.monotonic()<self.quiet_until)

    def begin_capture(self):
        if self.microphone_blocked or self.capture_active:raise RuntimeError('Speech output or microphone busy; retry after the short announcement')
        self.capture_active=True

    def end_capture(self):self.capture_active=False;self.changed.set()

    async def _run(self):
        while self.queue and not self.closed:
            if self.capture_active:
                self.changed.clear()
                await self.changed.wait();continue
            item=self.queue.popleft()
            if item['epoch']!=self.epoch or time.monotonic()-item['queued_at_s']>10:
                self.audit({'type':'speech_dropped','reason':'stale','event':item});continue
            if self.mode=='log':
                self.audit({'type':'speech_would_play','event':item});continue
            self.current=item
            try:
                self.audit({'type':'speech_started','event':item})
                self.playing=asyncio.create_task(self.player(self.clips[item['clip']],item))
                await self.playing
                self.audit({'type':'speech_finished','event':item})
            except asyncio.CancelledError:
                self.audit({'type':'speech_cancelled','event':item})
                if self.closed:raise
            except Exception as exc:
                self.last_error=type(exc).__name__+': '+str(exc)[:200]
                self.audit({'type':'speech_unavailable','error':self.last_error,'event':item})
            finally:
                self.playing=None;self.current=None;self.quiet_until=time.monotonic()+.3

    def snapshot(self):
        return {'mode':self.mode,'queued':len(self.queue),'speaking':self.current is not None,
                'microphone_blocked':self.microphone_blocked,'capture_active':self.capture_active,
                'device':self.cfg.get('device'),'last_error':self.last_error}

    async def close(self):
        self.closed=True;self._clear('shutdown');self.changed.set()
        if self.pump:
            self.pump.cancel();await asyncio.gather(self.pump,return_exceptions=True)
