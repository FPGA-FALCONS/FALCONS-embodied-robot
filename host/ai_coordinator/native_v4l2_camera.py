"""Owned MMAP capture preserving Linux driver timestamp/sequence provenance.

No model or driver install. Select by existing USB identity rules; the helper
is compiled from the accompanying C source during deployment. Metric time is
still unverified until a matching measured timing profile is supplied.
"""
from pathlib import Path
import math
import struct
import subprocess
import threading
import time
import uuid
from .v4l2_camera import select_device, validate_config, configured_focus

HEADER=struct.Struct('<4s7I2Q')
MJPG=int.from_bytes(b'MJPG','little')
MAX_FRAME=32*1024*1024


def read_exact(stream,size):
    if not 0<size<=MAX_FRAME:raise ValueError('Bounded native frame required')
    result=bytearray()
    while len(result)<size:
        data=stream.read(size-len(result))
        if not data:raise EOFError('Native UVC stream ended')
        result.extend(data)
    return bytes(result)


def read_frame(stream,expected_size):
    raw=read_exact(stream,HEADER.size)
    magic,header,width,height,sequence,flags,length,fourcc,timestamp,received=HEADER.unpack(raw)
    if (magic!=b'FVC1' or header!=48 or (width,height)!=tuple(expected_size)
            or fourcc!=MJPG or not 4<=length<=MAX_FRAME or received==0):
        raise ValueError('Invalid native UVC header/mode')
    payload=read_exact(stream,length)
    if not payload.startswith(b'\xff\xd8') or not payload.endswith(b'\xff\xd9'):
        raise ValueError('Incomplete MJPEG image')
    clock='monotonic' if flags&0xe000==0x2000 else 'unknown'
    source='start_of_exposure' if flags&0x70000==0x10000 else 'end_of_frame' if flags&0x70000==0 else 'unknown'
    return payload,dict(width=width,height=height,source_sequence=sequence,driver_flags=flags,
                        driver_timestamp_s=timestamp/1e6,received_at_s=received/1e6,
                        timestamp_clock=clock,timestamp_source=source)


class Camera:
    def __init__(self,cfg):
        from .device_roles import camera_config
        cfg=camera_config(cfg)
        self.focus=configured_focus(cfg)
        width,height,fps,fourcc,self.warmup=validate_config(cfg)
        if fourcc!='MJPG' or fps!=int(fps):raise ValueError('Native capture requires probed MJPG and integer FPS')
        helper=Path(cfg.get('native_helper',''))
        if not helper.is_absolute() or not helper.is_file():raise ValueError('Explicit installed native helper required')
        self.selection=select_device(cfg.get('device'));self.size=(width,height)
        self.cfg=cfg;self.storage=Path(cfg['storage']);self.storage.mkdir(parents=True,exist_ok=True)
        self.proc=None;self.thread=None;self.condition=threading.Condition();self.stop=threading.Event()
        self.latest=None;self.error=None;self.sequence=0;self.delivered=0;self.started=time.monotonic()
        self.negotiated={'width':width,'height':height,'fps':fps,'fourcc':fourcc}
        if self.focus is not None:self.negotiated.update(focus=self.focus,autofocus=False)
        try:
            command=[str(helper),self.selection['device'],str(width),str(height),str(int(fps)),
                     '1' if cfg.get('fixed_frame_rate') is True else '0']
            if self.focus is not None:command.append(str(self.focus))
            self.proc=subprocess.Popen(command,stdout=subprocess.PIPE,bufsize=0)
            if select_device(cfg['device'])!=self.selection:raise RuntimeError('Camera identity changed during open')
            self.thread=threading.Thread(target=self._drain,name='uvc-mmap-latest',daemon=True);self.thread.start()
        except BaseException:self.close();raise

    def _drain(self):
        warmup=self.warmup
        try:
            while not self.stop.is_set():
                payload,metadata=read_frame(self.proc.stdout,self.size)
                if warmup:warmup-=1;continue
                with self.condition:
                    self.sequence+=1;self.latest=(self.sequence,payload,metadata);self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.error=(('Camera unavailable with manual focus=%d: '%self.focus) if self.focus is not None else '')+str(exc)
                self.condition.notify_all()

    def run(self,request):
        with self.condition:
            self.condition.wait_for(lambda:self.error or self.stop.is_set() or
                (self.latest is not None and self.latest[0]>self.delivered),timeout=1)
            if self.error:raise RuntimeError(self.error)
            if self.stop.is_set():raise RuntimeError('Camera closed')
            if self.latest is None:
                if time.monotonic()-self.started>10:raise RuntimeError('Camera first frame timeout')
                return {'status':'no_complete_frame'}
            sequence,payload,metadata=self.latest
            if sequence<=self.delivered:return {'status':'no_complete_frame'}
            self.delivered=sequence
        # An immutable per-request lease is retained until its consumer finishes.
        # Native capture drains into one in-memory latest slot, never disk backlog.
        path=self.storage/(uuid.uuid4().hex+'.jpg');path.write_bytes(payload)
        if self.focus is not None:
            metadata.update(focus_value=self.focus,focus_locked=True,autofocus=False,
                            focus_verification='v4l2_control_readback_each_frame')
        return {'status':'frame','image':str(path),'format':'image','byteorder':None,**metadata,
                'sequence':sequence,'backend':'v4l2_native','device':self.selection['device'],
                'negotiated':self.negotiated,'sequence_origin':'v4l2_buffer_sequence',
                'timestamp_origin':'v4l2_driver','source_timestamp_verified':False,'replay':False}

    def close(self):
        self.stop.set()
        with self.condition:self.condition.notify_all()
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=1)
        if self.thread:self.thread.join(timeout=1)
        if self.proc and self.proc.stdout:self.proc.stdout.close()
