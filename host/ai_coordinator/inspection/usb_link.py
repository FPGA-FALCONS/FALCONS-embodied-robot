"""Owned FX2 Bulk OUT/IN transport. No firmware, reset or driver detach.

Selection uses identity plus verified active endpoint descriptors, not a USB
socket number. Partial OUT writes return progress; callers retain the current
whole container and must never splice a new header into its unfinished body.
"""
import ctypes as C
import ctypes.util
import os
from pathlib import Path
from ..device_roles import choose
from fx2_camera.probe import inspect_usb


def select(cfg, inventory=None):
    selector=cfg.get('selector')
    devices=inspect_usb()['devices'] if inventory is None else inventory
    records=[]
    for item in devices:
        serial_path=Path('/sys/bus/usb/devices')/item['path']/'serial'
        records.append({**item,'role':'inspection_link','aliases':[item['path']],
                        'vid':int(item['vid'],16),'pid':int(item['pid'],16),
                        'serial':item.get('serial') or (serial_path.read_text().strip() if serial_path.is_file() else '')})
    selected=choose(selector,records,'inspection_link')
    if selected.get('speed')!='480':raise ValueError('Verified 480M FX2 link required')
    interface=cfg.get('interface',0)
    interfaces=[i for i in selected['interfaces'] if int(i['bInterfaceNumber'],16)==interface and int(i['bAlternateSetting'],16)==0]
    if len(interfaces)!=1 or interfaces[0].get('driver'):raise ValueError('Exclusive unbound active USB interface required')
    for address in (cfg.get('out_endpoint',2),cfg.get('in_endpoint',0x86)):
        if type(address) is not int or not 1<=address<=255:raise ValueError('Explicit endpoint address required')
        endpoints=[e for e in interfaces[0]['endpoints'] if int(e['bEndpointAddress'],16)==address and
                   int(e['bmAttributes'],16)&3==2 and int(e['wMaxPacketSize'],16)==512]
        if len(endpoints)!=1:raise ValueError('Missing matching 512-byte Bulk endpoint')
    if cfg.get('out_endpoint',2)&128 or not cfg.get('in_endpoint',0x86)&128:
        raise ValueError('Endpoint directions are reversed')
    return {**selected,'bus_number':int(selected['busnum']),'device_address':int(selected['devnum'])}


class USBLink:
    def __init__(self,cfg):
        if os.name!='posix':raise RuntimeError('Live USB link requires Linux')
        self.cfg=dict(cfg);self.handle=None;self.claimed=False;self.lock=None;self.context=C.c_void_p()
        self.selection=select(cfg)
        name=ctypes.util.find_library('usb-1.0')
        if not name:raise RuntimeError('Existing libusb-1.0 unavailable')
        self.lib=C.CDLL(name)
        signatures={
            'libusb_init':([C.POINTER(C.c_void_p)],C.c_int),
            'libusb_exit':([C.c_void_p],None),
            'libusb_get_device_list':([C.c_void_p,C.POINTER(C.POINTER(C.c_void_p))],C.c_ssize_t),
            'libusb_free_device_list':([C.POINTER(C.c_void_p),C.c_int],None),
            'libusb_open':([C.c_void_p,C.POINTER(C.c_void_p)],C.c_int),
            'libusb_get_device':([C.c_void_p],C.c_void_p),
            'libusb_get_bus_number':([C.c_void_p],C.c_uint8),
            'libusb_get_device_address':([C.c_void_p],C.c_uint8),
            'libusb_kernel_driver_active':([C.c_void_p,C.c_int],C.c_int),
            'libusb_claim_interface':([C.c_void_p,C.c_int],C.c_int),
            'libusb_release_interface':([C.c_void_p,C.c_int],C.c_int),
            'libusb_close':([C.c_void_p],None),
            'libusb_bulk_transfer':([C.c_void_p,C.c_ubyte,C.POINTER(C.c_ubyte),C.c_int,C.POINTER(C.c_int),C.c_uint],C.c_int)}
        for key,(args,ret) in signatures.items():getattr(self.lib,key).argtypes=args;getattr(self.lib,key).restype=ret
        try:
            import fcntl
            runtime=Path('/run/user')/str(os.getuid())
            if not runtime.is_dir():raise RuntimeError('User runtime directory unavailable')
            self.lock=(runtime/('falcons-inspection-%04x-%04x.lock'%(self.selection['vid'],self.selection['pid']))).open('a+')
            fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if self.lib.libusb_init(C.byref(self.context)):raise RuntimeError('libusb initialization failed')
            self.handle=self._open_selected()
            device=self.lib.libusb_get_device(self.handle)
            if (self.lib.libusb_get_bus_number(device),self.lib.libusb_get_device_address(device)) != \
                    (self.selection['bus_number'],self.selection['device_address']):
                raise OSError('USB identity changed while opening')
            if self.lib.libusb_kernel_driver_active(self.handle,cfg.get('interface',0))==1:
                raise OSError('Refusing to detach a kernel driver')
            if self.lib.libusb_claim_interface(self.handle,cfg.get('interface',0)):
                raise OSError('FX2 interface busy or inaccessible')
            self.claimed=True
        except BaseException:self.close();raise

    def _open_selected(self):
        # VID/PID can be shared by several physical FX2 boards. Open the exact
        # bus/address selected by serial/identity, never the first VID/PID hit.
        devices=C.POINTER(C.c_void_p)()
        count=self.lib.libusb_get_device_list(self.context,C.byref(devices))
        if count<0:raise OSError('USB enumeration failed: '+str(count))
        try:
            for index in range(count):
                device=devices[index]
                if (self.lib.libusb_get_bus_number(device),self.lib.libusb_get_device_address(device)) != \
                        (self.selection['bus_number'],self.selection['device_address']):continue
                handle=C.c_void_p()
                result=self.lib.libusb_open(device,C.byref(handle))
                if result or not handle:raise OSError('FX2 inaccessible; install the supplied narrow plugdev rule')
                return handle
            raise OSError('Selected FX2 disconnected while opening')
        finally:self.lib.libusb_free_device_list(devices,1)

    def write_once(self,data,offset=0,timeout_ms=20):
        if self.handle is None:raise OSError('FX2 closed')
        if not isinstance(data,bytes) or not 0<=offset<=len(data):raise ValueError('Whole immutable transfer and valid offset required')
        chunk=data[offset:offset+65536]
        if not chunk:return offset
        buffer=(C.c_ubyte*len(chunk)).from_buffer_copy(chunk);count=C.c_int()
        result=self.lib.libusb_bulk_transfer(self.handle,self.cfg.get('out_endpoint',2),buffer,len(chunk),C.byref(count),timeout_ms)
        if result not in (0,-7):raise OSError('FX2 Bulk OUT failed: '+str(result))
        return offset+count.value

    def read_once(self,timeout_ms=2):
        if self.handle is None:raise OSError('FX2 closed')
        buffer=(C.c_ubyte*65536)();count=C.c_int()
        result=self.lib.libusb_bulk_transfer(self.handle,self.cfg.get('in_endpoint',0x86),buffer,len(buffer),C.byref(count),timeout_ms)
        if result not in (0,-7):raise OSError('FX2 Bulk IN failed: '+str(result))
        return bytes(buffer[:count.value])

    def close(self):
        if self.handle:
            if self.claimed:self.lib.libusb_release_interface(self.handle,self.cfg.get('interface',0))
            self.lib.libusb_close(self.handle);self.handle=None;self.claimed=False
        if self.context.value:self.lib.libusb_exit(self.context);self.context=C.c_void_p()
        if self.lock:self.lock.close();self.lock=None

    def __enter__(self):return self
    def __exit__(self,*args):self.close()


def main():
    import argparse,json,time
    from .stream import Container,Reader,DIAGNOSTIC,valid_diagnostic
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--diagnostic',action='store_true',help='Explicit bounded data echo; no navigation commands')
    args=parser.parse_args()
    cfg=json.loads(args.config.read_text(encoding='utf-8'))
    cfg=cfg.get('inspection',cfg).get('highspeed',cfg)
    identity=select(cfg)
    if not args.diagnostic:
        print(json.dumps({'identity':identity,'operation':'descriptor_probe_only','firmware_changed':False}));return 0
    sequence=time.monotonic_ns()&0xffffffff
    frame=Container(DIAGNOSTIC,0,0,0,0,sequence,payload=b'FALCONS-AIF1-DIAG\0'+bytes(range(32)))
    wire=frame.encode();reader=Reader();deadline=time.monotonic()+3.5;offset=0
    with USBLink(cfg) as link:
        while time.monotonic()<deadline:
            if offset<len(wire):offset=link.write_once(wire,offset)
            for received in reader.feed(link.read_once()):
                if valid_diagnostic(frame,received):
                    print(json.dumps({'round_trip':True,'ifclk_hz':received.session,'bytes':len(wire),'motion_commands_sent':0,'firmware_changed':False}));return 0
    raise RuntimeError('No matching diagnostic echo; do not assume Slave FIFO compatibility')


if __name__=='__main__':raise SystemExit(main())
