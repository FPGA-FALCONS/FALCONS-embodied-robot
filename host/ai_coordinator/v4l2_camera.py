"""USB/UVC input in an owned process. No driver, firmware or package changes.

Continuously drain capture into one latest-frame slot so inference cannot build
an unbounded camera queue. Receipt time and host sequence are NOT sensor time
or sensor frame identity; live motion still requires separate commissioning.
"""
import math
from pathlib import Path
import re
import threading
import time
import uuid


def select_device(device,dev_root=Path('/dev'),sys_root=Path('/sys')):
    if not isinstance(device,str):raise ValueError('Explicit stable USB camera device path required')
    path=Path(device)
    if path.parent not in (dev_root/'v4l/by-id',dev_root/'v4l/by-path') or '..' in path.parts:
        raise ValueError('Use an explicit /dev/v4l/by-id or by-path USB camera link')
    node=path.resolve(strict=True)
    if node.parent!=dev_root or not re.fullmatch(r'video[0-9]+',node.name) or not node.is_char_device():
        raise ValueError('Selected link is not a video character device')
    ancestry=(sys_root/'class/video4linux'/node.name/'device').resolve(strict=True)
    parents=[ancestry,*ancestry.parents]
    if not any((p/'driver').is_symlink() and (p/'driver').resolve().name=='uvcvideo' for p in parents):
        raise ValueError('Selected device is not bound to the existing UVC driver')
    usb=next((p for p in parents if (p/'idVendor').is_file() and (p/'idProduct').is_file()),None)
    if usb is None:raise ValueError('Selected video node has no USB device ancestry')
    return {'device':str(path),'node':str(node),'usb_path':str(usb),
            'vid':int((usb/'idVendor').read_text().strip(),16),
            'pid':int((usb/'idProduct').read_text().strip(),16)}


def validate_config(cfg):
    size=cfg.get('expected_size')
    if (not isinstance(size,(list,tuple)) or len(size)!=2 or
        any(type(v) is not int or not 1<=v<=8192 for v in size) or size[0]*size[1]>16777216):
        raise ValueError('Explicit bounded expected_size [width,height] required')
    fps=cfg.get('fps')
    if type(fps) not in (int,float) or not math.isfinite(fps) or not 0<fps<=120:
        raise ValueError('Explicit finite camera fps in (0,120] required')
    fourcc=cfg.get('fourcc')
    if fourcc not in ('MJPG','YUYV'):raise ValueError('Select probed MJPG or YUYV mode explicitly')
    warmup=cfg.get('warmup_frames',5)
    if type(warmup) is not int or not 0<=warmup<=120:raise ValueError('Invalid warmup frame count')
    if not isinstance(cfg.get('storage'),str) or not cfg['storage']:raise ValueError('Camera storage required')
    return size[0],size[1],float(fps),fourcc,warmup


def configured_focus(cfg):
    value=cfg.get('focus')
    if value is None:
        if cfg.get('autofocus') is False:raise ValueError('Manual focus requires an explicit focus position')
        return None
    if type(value) is not int or not 0<=value<=2147483647:
        raise ValueError('Focus must be a nonnegative integer in the camera native range')
    if cfg.get('autofocus',False) is not False:
        raise ValueError('A fixed focus position requires autofocus=false')
    return value


class Camera:
    def __init__(self,cfg):
        from .device_roles import camera_config
        cfg=camera_config(cfg)
        if configured_focus(cfg) is not None:
            raise ValueError('Verified manual focus requires the v4l2_native backend')
        width,height,fps,fourcc,self.warmup=validate_config(cfg)
        self.selection=select_device(cfg.get('device'))
        import cv2
        self.cv2=cv2;self.cap=None;self.thread=None
        self.stop=threading.Event();self.condition=threading.Condition()
        self.latest=None;self.error=None;self.sequence=0;self.delivered=0
        self.started_at=time.monotonic()
        self.size=(width,height);self.storage=Path(cfg['storage'])
        try:
            self.cap=cv2.VideoCapture(self.selection['device'],cv2.CAP_V4L2)
            if not self.cap.isOpened():raise RuntimeError('Configured USB camera cannot be opened')
            if select_device(cfg['device'])!=self.selection:raise RuntimeError('USB identity changed while opening')
            self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*fourcc))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,width);self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
            self.cap.set(cv2.CAP_PROP_FPS,fps)
            actual=[self.cap.get(p) for p in (cv2.CAP_PROP_FRAME_WIDTH,cv2.CAP_PROP_FRAME_HEIGHT,
                                             cv2.CAP_PROP_FPS,cv2.CAP_PROP_FOURCC)]
            if (not all(math.isfinite(v) for v in actual) or actual[:2]!=[width,height] or
                abs(actual[2]-fps)>max(.5,fps*.02) or actual[3]!=cv2.VideoWriter_fourcc(*fourcc)):
                raise RuntimeError('Camera silently negotiated a different size/fps/fourcc: '+repr(actual))
            # Buffer hint may be unsupported; continuous draining remains authoritative.
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
            self.negotiated={'width':width,'height':height,'fps':actual[2],'fourcc':fourcc}
            self.storage.mkdir(parents=True,exist_ok=True)
            self.thread=threading.Thread(target=self._drain,name='uvc-latest-frame',daemon=True)
            self.thread.start()
        except BaseException:self.close();raise

    def _drain(self):
        try:
            import numpy as np
            warmup=self.warmup
            while not self.stop.is_set():
                good,frame=self.cap.read();received=time.monotonic()
                if self.stop.is_set():break
                if not good:raise RuntimeError('USB camera disconnected or read failed')
                if (not isinstance(frame,np.ndarray) or frame.dtype!=np.uint8 or
                    frame.shape!=(self.size[1],self.size[0],3)):
                    raise RuntimeError('Camera returned a different BGR image shape/type')
                if warmup:warmup-=1;continue
                with self.condition:
                    self.sequence+=1
                    self.latest=(self.sequence,received,frame.copy());self.condition.notify_all()
        except Exception as exc:
            with self.condition:self.error=str(exc);self.condition.notify_all()

    def run(self,request):
        with self.condition:
            self.condition.wait_for(lambda:self.error or self.stop.is_set() or
                                    (self.latest is not None and self.latest[0]>self.delivered),timeout=1)
            if self.error:raise RuntimeError(self.error)
            if self.stop.is_set():raise RuntimeError('USB camera closed')
            if self.latest is None:
                if time.monotonic()-self.started_at>10:raise RuntimeError('USB camera did not produce its first frame within 10 seconds')
                return {'status':'no_complete_frame'}
            if self.latest[0]<=self.delivered:
                if time.monotonic()-self.latest[1]>1:raise RuntimeError('USB camera latest frame stalled')
                return {'status':'no_complete_frame'}
            sequence,received,frame=self.latest
            if time.monotonic()-received>1:raise RuntimeError('USB camera latest frame stalled')
            self.delivered=sequence
        good,png=self.cv2.imencode('.png',frame,[self.cv2.IMWRITE_PNG_COMPRESSION,1])
        if not good:raise RuntimeError('Camera PNG encoding failed')
        path=self.storage/(uuid.uuid4().hex+'.png');path.write_bytes(png.tobytes())
        return {'status':'frame','image':str(path),'format':'image','byteorder':None,
                'width':self.size[0],'height':self.size[1],'sequence':sequence,'received_at_s':received,
                'backend':'v4l2','device':self.selection['device'],'negotiated':self.negotiated,
                'sequence_origin':'host_frame_counter','timestamp_origin':'host_receipt_only',
                'replay':False,'source_timestamp_verified':False}

    def close(self):
        self.stop.set()
        with self.condition:self.condition.notify_all()
        if self.thread:self.thread.join(timeout=.5)
        # Do not race release against a blocking native read; the parent Worker
        # terminates this owned process if the driver does not return.
        if self.cap and (not self.thread or not self.thread.is_alive()):self.cap.release();self.cap=None
