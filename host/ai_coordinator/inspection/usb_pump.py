"""Bounded high-speed worker; UART safety/heartbeat never waits on this thread.

One active immutable container survives partial OUT writes. Pending display
frames are latest-only; layer control has bounded priority at container
boundaries. A fresh diagnostic challenge must succeed after each reopen.
"""
from collections import deque
import threading
import time
import secrets
from .stream import Container, Reader, DISPLAY, CONTROL, MAP, DIAGNOSTIC, valid_diagnostic
from .usb_link import USBLink
from . import wire


class USBPump:
    def __init__(self, cfg, *, factory=USBLink, clock=time.monotonic):
        self.cfg=dict(cfg);self.factory=factory;self.clock=clock
        self.lock=threading.Lock();self.stop_event=threading.Event();self.thread=None
        self.controls=deque();self.display=None;self.maps=deque(maxlen=2)
        self.binding=None;self.link=None;self.active=None;self.offset=0
        self.reader=Reader();self.challenge=None;self.challenge_deadline=0
        self.verified=False;self.reopen_at=0.;self.error='not_started';self.generation=0

    def start(self):
        if self.thread and self.thread.is_alive():return
        self.stop_event.clear()
        self.thread=threading.Thread(target=self._run,name='inspection-usb',daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:self.thread.join(timeout=2.)
        if self.thread and self.thread.is_alive():raise RuntimeError('USB worker failed to stop')
        self._disconnect('closed')

    def bind(self, session, task, epoch, context):
        binding=(session,task,epoch,context)
        if any(type(v) is not int or not 0<=v<=0xffffffff for v in binding):
            raise ValueError('uint32 binding required')
        with self.lock:
            if binding!=self.binding:
                self.binding=binding;self.controls.clear();self.display=None;self.maps.clear()

    def offer(self, frame):
        if not isinstance(frame,Container) or frame.kind not in (DISPLAY,CONTROL):
            raise ValueError('Only display/layer containers may enter the runtime OUT queue')
        encoded=frame.encode()
        if frame.kind==CONTROL:
            reader=wire.Reader();messages=reader.feed(frame.payload)
            if (len(messages)!=1 or messages[0]['kind'] not in (wire.LAYER_BEGIN,wire.LAYER_DATA,wire.LAYER_COMMIT)
                    or wire.encode(messages[0]['kind'],messages[0]['words'],messages[0]['sequence'])!=frame.payload):
                raise ValueError('High-speed CONTROL must contain exactly one complete layer transaction')
        with self.lock:
            if (frame.session,frame.task,frame.epoch,frame.context)!=self.binding:return False
            item=(frame,encoded)
            if frame.kind==DISPLAY:self.display=item
            elif len(self.controls)<8:self.controls.append(item)
            else:return False
        return True

    def status(self):
        with self.lock:
            return {'verified':self.verified,'error':self.error,'generation':self.generation,
                    'pending_control':len(self.controls),'pending_display':self.display is not None}

    def drain_maps(self):
        with self.lock:result=list(self.maps);self.maps.clear();return result

    def _disconnect(self, reason):
        if self.link:
            try:self.link.close()
            except OSError:pass
            finally:self.link=None
        self.active=None;self.offset=0;self.challenge=None;self.reader=Reader()
        with self.lock:
            self.verified=False;self.error=str(reason)[:240]
            self.controls.clear();self.display=None;self.maps.clear()
        self.reopen_at=self.clock()+1.

    def _cycle(self):
        now=self.clock()
        if self.link is None:
            if now<self.reopen_at:return
            self.link=self.factory(self.cfg);self.reader=Reader()
            sequence=secrets.randbits(32)
            self.challenge=Container(DIAGNOSTIC,0,0,0,0,sequence,payload=secrets.token_bytes(32))
            self.active=(self.challenge,self.challenge.encode());self.offset=0
            self.challenge_deadline=now+2.
        if self.active is None:
            with self.lock:
                if self.verified:
                    if self.controls:self.active=self.controls.popleft()
                    elif self.display:self.active=self.display;self.display=None
            self.offset=0
        if self.active:
            data=self.active[1]
            self.offset=self.link.write_once(data,self.offset,timeout_ms=20)
            if not 0<=self.offset<=len(data):raise OSError('Invalid partial USB transfer progress')
            if self.offset==len(data):self.active=None;self.offset=0
        for frame in self.reader.feed(self.link.read_once(timeout_ms=2),now=now):
            if self.challenge is not None and valid_diagnostic(self.challenge,frame):
                self.challenge=None
                with self.lock:self.verified=True;self.error=None;self.generation+=1
            elif frame.kind==MAP:
                with self.lock:
                    if self.verified and (frame.session,frame.task,frame.epoch,frame.context)==self.binding:
                        self.maps.append(frame)
        if self.challenge is not None and now>=self.challenge_deadline:
            raise OSError('No verified AIF1 diagnostic round trip')

    def _run(self):
        try:
            while not self.stop_event.is_set():
                try:self._cycle()
                except (OSError,RuntimeError,ValueError) as exc:self._disconnect(exc)
                self.stop_event.wait(.001 if self.link else .1)
        finally:self._disconnect('closed')
