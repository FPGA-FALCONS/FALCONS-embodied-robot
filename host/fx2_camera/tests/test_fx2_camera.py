import argparse
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from probe import hex_id,inspect_usb
from raw565 import convert,rgb888


class ProbeTests(unittest.TestCase):
    def populate(self,root):
        device=root/'1-2';device.mkdir()
        for name,value in {'idVendor':'04b4','idProduct':'00f1','speed':'480','product':'Example firmware'}.items():
            (device/name).write_text(value,encoding='utf-8')
        interface=device/'interface0';interface.mkdir()  # Portable fixture; Linux uses 1-2:1.0.
        for name,value in {'bInterfaceNumber':'00','bAlternateSetting':'0','bInterfaceClass':'ff','bInterfaceSubClass':'00','bInterfaceProtocol':'00'}.items():
            (interface/name).write_text(value,encoding='utf-8')
        for address,attributes in (('86','02'),('02','02'),('81','03')):
            ep=interface/('ep_'+address);ep.mkdir()
            for name,value in {'bEndpointAddress':address,'bmAttributes':attributes,'wMaxPacketSize':'0200'}.items():
                (ep/name).write_text(value,encoding='utf-8')
        return device

    def test_descriptor_directions_and_types(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.populate(Path(tmp));report=inspect_usb(tmp)
            device=report['devices'][0]
            eps={e['bEndpointAddress']:e for e in device['interfaces'][0]['endpoints']}
            self.assertTrue(eps['86']['bulk_in_candidate'])
            self.assertFalse(eps['02']['bulk_in_candidate'])
            self.assertFalse(eps['81']['bulk_in_candidate'])
            self.assertEqual(device['speed'],'480')
            self.assertFalse(report['firmware_loaded']);self.assertFalse(report['stream_tested'])

    def test_filter_and_unknown_pid_are_not_assumed(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.populate(Path(tmp))
            self.assertEqual(inspect_usb(tmp,vid='1a86')['devices'],[])
            self.assertEqual(inspect_usb(tmp,pid='8613')['devices'],[])

    def test_detached_endpoint_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            device=self.populate(Path(tmp))
            (device/'interface0/ep_86/bEndpointAddress').unlink()
            report=inspect_usb(tmp)
            self.assertTrue(report['errors'])
            self.assertFalse(report['devices'][0]['interfaces'][0]['endpoints'][-1]['bulk_in_candidate'])

    def test_missing_root_and_empty_bus(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(inspect_usb(tmp)['devices'],[])
            with self.assertRaises(FileNotFoundError):inspect_usb(Path(tmp)/'missing')

    def test_id_validation(self):
        self.assertEqual(hex_id('0x04B4'),'04b4')
        for value in ('10000','-1','not_a_vid'):
            with self.subTest(value=value),self.assertRaises(argparse.ArgumentTypeError):hex_id(value)


class ImageTests(unittest.TestCase):
    def test_known_colors_and_byte_orders(self):
        pixels=[0xf800,0x07e0,0x001f,0xffff,0x0000]
        expected=bytes([255,0,0,0,255,0,0,0,255,255,255,255,0,0,0])
        for order in ('little','big'):
            raw=b''.join(v.to_bytes(2,order) for v in pixels)
            self.assertEqual(rgb888(raw,5,1,order),expected)

    def test_truncated_and_extra_payload_rejected(self):
        for raw in (b'',b'\x00',b'\x00'*3,b'\x00'*4):
            with self.subTest(raw=raw),self.assertRaises(ValueError):rgb888(raw,1,1,'little')

    def test_dimension_limits_and_unsupported_order(self):
        for width,height in ((0,1),(-1,1),(4096,4096)):
            with self.subTest(width=width),self.assertRaises(ValueError):rgb888(b'',width,height,'little')
        with self.assertRaises(ValueError):rgb888(b'\x00\x00',1,1,'guess')

    def test_ppm_export_and_exclusive_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'frame.raw';dst=Path(tmp)/'frame.ppm'
            src.write_bytes(b'\x00\xf8')
            convert(src,dst,1,1,'little')
            self.assertEqual(dst.read_bytes(),b'P6\n1 1\n255\n\xff\x00\x00')
            with self.assertRaises(FileExistsError):convert(src,dst,1,1,'little')


if __name__=='__main__':unittest.main()
