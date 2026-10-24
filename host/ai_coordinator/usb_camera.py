"""Optional FX2 Bulk-IN reader; no firmware loading, reset or driver detachment."""
import ctypes as C
import ctypes.util
from pathlib import Path
import time
import uuid
from .frames import FrameReader
from .stream_frames import StreamingFrameReader


class Camera:
    def __init__(self,cfg):
        self.handle=None;self.context=C.c_void_p();self.claimed=False;self.cfg=cfg
        protocol=cfg.get('protocol','fimg')
        if protocol not in ('fimg','fimg1','fimg2','fim2'):raise ValueError('Unknown explicit camera wire protocol')
        self.parser=(StreamingFrameReader(cfg.get('expected_size'),cfg.get('max_frame_age_s',2))
                     if protocol=='fim2' else FrameReader(expected_size=cfg.get('expected_size'),
                                                         versions=(2,) if protocol=='fimg2' else (1,2)))
        self.bytes_received=0;self.usb_calls=0
        from fx2_camera.selection import select_device
        self.selection=select_device(cfg)
        name=ctypes.util.find_library('usb-1.0')
        if not name:raise RuntimeError('Existing libusb-1.0 not found; no system installation attempted')
        self.lib=C.CDLL(name)
        signatures={
            'libusb_init':([C.POINTER(C.c_void_p)],C.c_int),
            'libusb_exit':([C.c_void_p],None),
            'libusb_get_device_list':([C.c_void_p,C.POINTER(C.POINTER(C.c_void_p))],C.c_ssize_t),
            'libusb_free_device_list':([C.POINTER(C.c_void_p),C.c_int],None),
            'libusb_get_bus_number':([C.c_void_p],C.c_uint8),
            'libusb_get_device_address':([C.c_void_p],C.c_uint8),
            'libusb_get_device_descriptor':([C.c_void_p,C.POINTER(C.c_ubyte)],C.c_int),
            'libusb_open':([C.c_void_p,C.POINTER(C.c_void_p)],C.c_int),
            'libusb_kernel_driver_active':([C.c_void_p,C.c_int],C.c_int),
            'libusb_claim_interface':([C.c_void_p,C.c_int],C.c_int),
            'libusb_release_interface':([C.c_void_p,C.c_int],C.c_int),
            'libusb_close':([C.c_void_p],None),
            'libusb_bulk_transfer':([C.c_void_p,C.c_ubyte,C.POINTER(C.c_ubyte),C.c_int,C.POINTER(C.c_int),C.c_uint],C.c_int)}
        for key,(args,ret) in signatures.items():getattr(self.lib,key).argtypes=args;getattr(self.lib,key).restype=ret
        if self.lib.libusb_init(C.byref(self.context))!=0:raise RuntimeError('libusb init failed')
        try:
            devices=C.POINTER(C.c_void_p)()
            count=self.lib.libusb_get_device_list(self.context,C.byref(devices))
            if count<0:raise RuntimeError('Cannot enumerate libusb devices')
            try:
                for index in range(count):
                    device=devices[index]
                    if (self.lib.libusb_get_bus_number(device)==self.selection['bus_number'] and
                            self.lib.libusb_get_device_address(device)==self.selection['device_address']):
                        descriptor=(C.c_ubyte*18)()
                        if self.lib.libusb_get_device_descriptor(device,descriptor)!=0:
                            raise RuntimeError('USB descriptor unavailable')
                        if (descriptor[8]|descriptor[9]<<8)!=cfg['vid'] or (descriptor[10]|descriptor[11]<<8)!=cfg['pid']:
                            raise RuntimeError('USB identity changed during selection')
                        handle=C.c_void_p()
                        if self.lib.libusb_open(device,C.byref(handle))!=0:raise RuntimeError('USB device inaccessible')
                        self.handle=handle
                        break
            finally:self.lib.libusb_free_device_list(devices,1)
            if not self.handle:raise RuntimeError('Configured FX2 device absent or inaccessible')
            if self.lib.libusb_kernel_driver_active(self.handle,cfg['interface'])==1:
                raise RuntimeError('USB interface already has a kernel driver; refusing detach')
            if self.lib.libusb_claim_interface(self.handle,cfg['interface'])!=0:raise RuntimeError('FX2 interface busy or unavailable')
            self.claimed=True;self.storage=Path(cfg['storage']);self.storage.mkdir(parents=True,exist_ok=True)
        except BaseException:self.close();raise

    def read_frames(self,timeout_s=.5):
        deadline=time.monotonic()+timeout_s;buffer=(C.c_ubyte*65536)()
        while time.monotonic()<deadline:
            count=C.c_int()
            ret=self.lib.libusb_bulk_transfer(self.handle,self.cfg['endpoint'],buffer,len(buffer),C.byref(count),100)
            self.usb_calls+=1
            if ret not in (0,-7):raise RuntimeError('FX2 bulk read failed: '+str(ret))
            if count.value:
                self.bytes_received+=count.value
                frames=self.parser.feed(C.string_at(buffer,count.value));received=time.monotonic()
                for frame in frames:frame['received_at_s']=received
                if frames:return frames
            else:self.parser.feed(b'')  # Expire a stalled incomplete FIM2 frame.
        return []

    def run(self,request):
        frames=self.read_frames()
        if not frames:return {'status':'no_complete_frame','parser_errors':self.parser.errors}
        latest=frames[-1];received=latest.pop('received_at_s')
        path=self.storage/(uuid.uuid4().hex+'.raw');path.write_bytes(latest.pop('data'))
        return {'status':'frame','image':str(path),'received_at_s':received,**latest,
            'timestamp_origin':'host_receipt_only','replay':False,'source_timestamp_verified':False,
            'parser_errors':self.parser.errors}

    def close(self):
        if self.handle:
            if self.claimed:self.lib.libusb_release_interface(self.handle,self.cfg['interface'])
            self.lib.libusb_close(self.handle);self.handle=None
        if self.context.value:self.lib.libusb_exit(self.context);self.context=C.c_void_p()
