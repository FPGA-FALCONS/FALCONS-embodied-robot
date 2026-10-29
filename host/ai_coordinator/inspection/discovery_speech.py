"""Once per confirmed object and patrol; speech never blocks navigation."""
import asyncio
from collections import deque
import hashlib
import json
from pathlib import Path
import re
import time
import wave

from ..announcements import NAMES


def checked_clip(path):
    path=Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size>512000:
        raise ValueError('Missing or oversized discovery speech')
    with wave.open(str(path),'rb') as wav:
        duration=wav.getnframes()/wav.getframerate()
        if (wav.getnchannels()!=1 or wav.getsampwidth()!=2 or wav.getcomptype()!='NONE'
                or not .05<=duration<=8 or not 8000<=wav.getframerate()<=48000):
            raise ValueError('Expected short PCM16 mono discovery speech')
    return dict(path=str(path),duration_s=duration)


class DiscoverySpeech:
    def __init__(self,rt,allowed):
        self.rt=rt;self.allowed=allowed
        cfg=rt.cfg.get('inspection',{}).get('discovery_speech',{})
        self.enabled=cfg.get('enabled',True) is True
        self.names={**NAMES,**cfg.get('names',{})}
        if len(self.names)>128 or any(not isinstance(k,str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',k)
                or not isinstance(v,str) or not 1<=len(v)<=20 for k,v in self.names.items()):
            raise ValueError('Invalid discovery speech class names')
        self.root=Path(rt.cfg.get('root',rt.run_dir))/'inspection_speech'
        self.audit=getattr(rt,'audit',lambda event:None)
        self.binding=None;self.seen=set();self.pending=set();self.attempt=0;self.queue=deque();self.generation=0
        self.pump=None;self.preparing=False;self.closed=False;self.last_error=None

    @property
    def task_key(self):return 'inspection:'+':'.join(map(str,self.binding)) if self.binding else None

    def cancel(self,reason):
        self.generation+=1;self.queue.clear();self.pending.clear()
        if self.binding and hasattr(self.rt.announcements,'cancel_task'):
            self.rt.announcements.cancel_task(self.task_key,reason)

    def notify(self,binding,objects):
        if self.closed or not self.enabled or not self.allowed(binding):return
        if binding!=self.binding:
            self.cancel('inspection_task_replaced');self.binding=binding;self.seen.clear()
        for obj in objects:
            identifier=obj['id'];label=self.names.get(obj['class'])
            if not obj.get('confirmed') or identifier in self.seen or (self.generation,identifier) in self.pending or not label:continue
            if len(self.seen)>=512:return
            self.pending.add((self.generation,identifier))
            item=dict(id=identifier,text='发现'+label,at_s=time.monotonic(),binding=binding,generation=self.generation)
            if len(self.queue)>=8:
                dropped=self.queue.popleft();self.pending.discard((dropped['generation'],dropped['id']))
                self.audit(dict(type='inspection_speech_skipped',reason='queue_full',object_id=dropped['id']))
            self.queue.append(item)
        if self.queue and (self.pump is None or self.pump.done()):self.pump=asyncio.create_task(self._run())

    async def _clip(self,text):
        model=self.rt.cfg.get('models',{}).get('tts',{})
        key=hashlib.sha256(json.dumps([text,model],sort_keys=True,ensure_ascii=False).encode()).hexdigest()
        self.root.mkdir(parents=True,exist_ok=True)
        path=self.root/(key+'.wav')
        if path.exists():return checked_clip(path)
        # Finish/cancel only this synthesis. A takeover invalidates its playback
        # generation immediately, without waiting for TTS or unloading its model.
        private=self.rt.run_dir/'inspection_speech';private.mkdir(parents=True,exist_ok=True)
        pending=private/(key+'.pending.wav')
        try:
            await self.rt.ai.tts_worker.call(dict(text=text,output=str(pending)),timeout=15)
            clip=checked_clip(pending);pending.replace(path);clip['path']=str(path)
            return clip
        finally:pending.unlink(missing_ok=True)

    async def _run(self):
        while self.queue and not self.closed:
            item=self.queue.popleft();binding=item['binding'];generation=item['generation']
            if generation!=self.generation or not self.allowed(binding) or time.monotonic()-item['at_s']>10:
                self.pending.discard((generation,item['id']));continue
            self.preparing=True
            try:
                # Resolving by USB identity at both preparation and playback
                # prevents wrong-speaker fallback and delayed replay after replug.
                from ..device_roles import audio_device
                audio_device(self.rt.cfg.get('audio_roles',{}).get('speaker'),'speaker',
                             self.rt.cfg.get('ai_module',{}).get('speaker_device'))
                clip=await self._clip(item['text'])
                if generation!=self.generation or not self.allowed(binding) or time.monotonic()-item['at_s']>10:continue
                self.last_error=None
                self.attempt+=1
                played=await self.rt.announcements.emit_clip(dict(id=self.task_key+':'+str(item['id'])+':'+str(self.attempt),task_id=self.task_key,
                    goal_id=str(item['id']),code='inspection_found',simulated=False,discovered_at_s=item['at_s']),clip,item['text'])
                if played and binding==self.binding:self.seen.add(item['id'])
            except asyncio.CancelledError:raise
            except Exception as exc:
                self.last_error=type(exc).__name__+': '+str(exc)[:200]
                self.audit(dict(type='inspection_speech_unavailable',object_id=item['id'],error=self.last_error))
            finally:
                self.pending.discard((generation,item['id']));self.preparing=False

    def snapshot(self):
        return dict(enabled=self.enabled,queued=len(self.queue),preparing=self.preparing,
                    played_object_ids=sorted(self.seen),last_error=self.last_error)

    async def close(self):
        self.closed=True;self.cancel('inspection_shutdown')
        if self.pump:
            self.pump.cancel();await asyncio.gather(self.pump,return_exceptions=True)
