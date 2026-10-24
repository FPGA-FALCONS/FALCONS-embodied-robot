from pathlib import Path
import ctypes as C
import json
import random
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.frames import encode_frame,encode_frame_v2,FrameReader
from ai_coordinator.usb_camera import Camera
from fx2_camera.selection import select_device
from fx2_camera.receive import main


CFG={'vid':0x04b4,'pid':0x00f1,'endpoint':0x86,'interface':0}


def populate(root, name='1-2', speed='480', kind='02'):
    device=root/name;device.mkdir()
    for key,value in {'idVendor':'04b4','idProduct':'00f1','speed':speed,'busnum':'1','devnum':'2'}.items():
        (device/key).write_text(value)
    interface=device/'interface0';interface.mkdir()  # Portable sysfs fixture (Windows forbids ':').
    for key,value in {'bInterfaceNumber':'00','bAlternateSetting':'0','bInterfaceClass':'ff',
                      'bInterfaceSubClass':'00','bInterfaceProtocol':'00'}.items():(interface/key).write_text(value)
    endpoint=interface/'ep_86';endpoint.mkdir()
    for key,value in {'bEndpointAddress':'86','bmAttributes':kind,'wMaxPacketSize':'0200'}.items():
        (endpoint/key).write_text(value)
    return interface


class SelectionTests(unittest.TestCase):
    def test_select_verified_endpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            populate(Path(tmp));device=select_device(CFG,tmp)
            self.assertEqual((device['bus_number'],device['device_address']),(1,2))

    def test_wrong_speed_interrupt_and_missing_endpoint_refused(self):
        for speed,kind,cfg in [('12','02',CFG),('480','03',CFG),('480','02',{**CFG,'endpoint':0x88})]:
            with self.subTest(speed=speed,kind=kind,cfg=cfg),tempfile.TemporaryDirectory() as tmp:
                populate(Path(tmp),speed=speed,kind=kind)
                with self.assertRaises(RuntimeError):select_device(cfg,tmp)

    def test_duplicate_vid_pid_requires_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            populate(Path(tmp));populate(Path(tmp),'1-3')
            with self.assertRaises(RuntimeError):select_device(CFG,tmp)
            self.assertEqual(select_device({**CFG,'usb_path':'1-3'},tmp)['path'],'1-3')

    def test_kernel_driver_refused(self):
        # Patch the read-only report so this test does not require Windows symlink privileges.
        with tempfile.TemporaryDirectory() as tmp:
            populate(Path(tmp));device=select_device(CFG,tmp)
            device['interfaces'][0]['driver']='cdc_acm'
            with patch('fx2_camera.selection.inspect_usb',return_value={'devices':[device]}):
                with self.assertRaises(RuntimeError):select_device(CFG,tmp)

    def test_invalid_selection(self):
        for values in ({**CFG,'vid':True},{**CFG,'endpoint':2},{**CFG,'interface':256}):
            with self.assertRaises(ValueError):select_device(values,'unavailable')


class FrameTests(unittest.TestCase):
    def test_random_usb_boundaries_garbage_crc_and_replay(self):
        wire=encode_frame(b'\x00\xf8\xe0\x07\x1f\x00',3,1,17)
        bad=bytearray(encode_frame(b'\0\0'*3,3,1,18));bad[-1]^=1
        source=b'garbageFIM'+wire+wire+bad+encode_frame(b'\xff\xff'*3,3,1,19)
        reader=FrameReader();frames=[];rng=random.Random(20261024)
        while source:
            count=rng.randrange(1,21);frames+=reader.feed(source[:count]);source=source[count:]
        self.assertEqual([f['sequence'] for f in frames],[17,19])
        self.assertEqual(frames[0]['data'],b'\x00\xf8\xe0\x07\x1f\x00')
        self.assertEqual(reader.errors,2)
        self.assertFalse(any(f['source_timestamp_verified'] for f in frames))

    def test_uint32_wrap_and_reset_require_new_receiver(self):
        reader=FrameReader()
        for seq in (0xfffffffe,0xffffffff,0,1):
            self.assertEqual(len(reader.feed(encode_frame(b'\0\0',1,1,seq))),1)
        self.assertEqual(reader.feed(encode_frame(b'\0\0',1,1,0)),[])
        self.assertEqual(len(FrameReader().feed(encode_frame(b'\0\0',1,1,0))),1)

    def test_encode_rejects_bad_fields(self):
        for width,height,seq in ((0,1,0),(True,1,0),(65536,1,0),(1,1,-1),(1,1,2**32)):
            with self.assertRaises(ValueError):encode_frame(b'\0\0',width,height,seq)

    def test_offline_cli_saves_rgb_preview_without_claiming_live(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'input.bin';out=root/'session'
            source.write_bytes(encode_frame(b'\0\xf8\xe0\x07\x1f\0',3,1,1))
            self.assertEqual(main(['--input',str(source),'--out',str(out)]),0)
            report=json.loads((out/'summary.json').read_text())
            row=json.loads((out/'frames.jsonl').read_text())
            self.assertFalse(report['hardware_tested']);self.assertTrue(row['replay'])
            self.assertFalse(row['source_timestamp_verified'])
            self.assertEqual(Path(row['preview']).read_bytes(),b'P6\n3 1\n255\n\xff\0\0\0\xff\0\0\0\xff')


class Function:
    def __init__(self,fn):self.fn=fn
    def __call__(self,*args):return self.fn(*args)


class FakeUSB:
    def __init__(self,data,*,driver=0,wrong_id=False,error=0):
        self.chunks=[data[:9],data[9:]];self.closed=False;self.released=False
        self.devices=(C.c_void_p*2)(123,None)
        def init(ptr):C.cast(ptr,C.POINTER(C.c_void_p))[0]=7;return 0
        def listing(ctx,ptr):C.cast(ptr,C.POINTER(C.POINTER(C.c_void_p)))[0]=self.devices;return 1
        def descriptor(dev,buf):
            buf[8]=0xb4;buf[9]=0x04;buf[10]=0xf2 if wrong_id else 0xf1;return 0
        def opening(dev,ptr):C.cast(ptr,C.POINTER(C.c_void_p))[0]=456;return 0
        def transfer(handle,ep,buf,size,count,timeout):
            chunk=self.chunks.pop(0) if self.chunks else b''
            for i,value in enumerate(chunk):buf[i]=value
            C.cast(count,C.POINTER(C.c_int))[0]=len(chunk)
            return error if error else -7  # libusb timeout may still deliver bytes.
        functions={'libusb_init':init,'libusb_exit':lambda ctx:None,'libusb_get_device_list':listing,
                   'libusb_free_device_list':lambda *args:None,'libusb_get_bus_number':lambda d:1,
                   'libusb_get_device_address':lambda d:2,'libusb_get_device_descriptor':descriptor,
                   'libusb_open':opening,'libusb_kernel_driver_active':lambda *args:driver,
                   'libusb_claim_interface':lambda *args:0,
                   'libusb_release_interface':lambda *args:setattr(self,'released',True) or 0,
                   'libusb_close':lambda *args:setattr(self,'closed',True),'libusb_bulk_transfer':transfer}
        for key,fn in functions.items():setattr(self,key,Function(fn))


class CameraTests(unittest.TestCase):
    def camera(self,usb,storage,**cfg):
        with patch('fx2_camera.selection.select_device',return_value={'bus_number':1,'device_address':2}),\
             patch('ai_coordinator.usb_camera.ctypes.util.find_library',return_value='fake'),\
             patch('ai_coordinator.usb_camera.C.CDLL',return_value=usb):
            return Camera({**CFG,'storage':storage,**cfg})

    def test_explicit_fimg2_usb_contract_skips_legacy_frame(self):
        usb=FakeUSB(encode_frame(b'\0\0',1,1,0)+encode_frame_v2(b'\0\xf8',1,1,1))
        with tempfile.TemporaryDirectory() as tmp:
            camera=self.camera(usb,tmp,protocol='fimg2',expected_size=[1,1])
            try:
                result=camera.run({})
                self.assertEqual(result['fimg_version'],2);self.assertEqual(result['sequence'],1)
                self.assertEqual(Path(result['image']).read_bytes(),b'\0\xf8')
                self.assertGreater(camera.parser.header_errors,0)
            finally:camera.close()

    def test_timeout_partial_bytes_decode_and_cleanup(self):
        usb=FakeUSB(encode_frame(b'\0\xf8',1,1,1))
        with tempfile.TemporaryDirectory() as tmp:
            camera=self.camera(usb,tmp)
            try:
                result=camera.run({});self.assertEqual(result['status'],'frame')
                self.assertEqual(Path(result['image']).read_bytes(),b'\0\xf8')
                self.assertFalse(result['source_timestamp_verified']);self.assertFalse(result['replay'])
            finally:camera.close()
        self.assertTrue(usb.closed and usb.released)

    def test_claim_does_not_detach_driver(self):
        usb=FakeUSB(b'',driver=1)
        with self.assertRaises(RuntimeError):self.camera(usb,'unused')
        self.assertTrue(usb.closed);self.assertFalse(usb.released)

    def test_identity_swap_and_disconnect_fail(self):
        usb=FakeUSB(b'',wrong_id=True)
        with self.assertRaises(RuntimeError):self.camera(usb,'unused')
        with tempfile.TemporaryDirectory() as tmp:
            usb=FakeUSB(b'',error=-4);camera=self.camera(usb,tmp)
            try:
                with self.assertRaises(RuntimeError):camera.run({})
            finally:camera.close()
        self.assertTrue(usb.closed)


if __name__=='__main__':unittest.main()
