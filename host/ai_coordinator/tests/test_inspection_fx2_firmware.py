import unittest
from ai_coordinator.inspection.fx2_firmware import reviewed,parse_iic,download,Record,IMAGE,IFCONFIG_IMMEDIATE


class FirmwareTests(unittest.TestCase):
    def test_exact_vendor_image_has_one_reviewed_30mhz_patch(self):
        original=parse_iic(IMAGE.read_bytes());patched,report=reviewed()
        before={r.address+k:v for r in original for k,v in enumerate(r.data)}
        after={r.address+k:v for r in patched for k,v in enumerate(r.data)}
        self.assertEqual([(a,before[a],after[a]) for a in before if before[a]!=after[a]],[(IFCONFIG_IMMEDIATE,0xe3,0xa3)])
        self.assertEqual(report['ifclk_hz'],30_000_000);self.assertFalse(report['writes_eeprom'])
    def test_unknown_truncated_and_register_records_are_rejected(self):
        image=IMAGE.read_bytes()
        with self.assertRaises(ValueError):reviewed(image[:-1]+b'\x01')
        with self.assertRaises(ValueError):parse_iic(image[:-1])
        with self.assertRaises(ValueError):download([Record(0xe601,b'\xa3')],lambda *x:1)
    def test_download_uses_only_cpucs_and_bounded_program_ram(self):
        records,_=reviewed();calls=[]
        def write(a,b):calls.append((a,b));return len(b)
        download(records,write)
        self.assertEqual(calls[0],(0xe600,b'\x01'));self.assertEqual(calls[-1],(0xe600,b'\0'))
        self.assertTrue(all(0<=a<a+len(b)<=0x4000 and 1<=len(b)<=64 for a,b in calls[1:-1]))
        downloaded={a+k:v for a,b in calls[1:-1] for k,v in enumerate(b)}
        self.assertEqual(downloaded,{r.address+k:v for r in records for k,v in enumerate(r.data)})
    def test_partial_transfer_releases_cpu_without_attempting_persistent_writes(self):
        calls=[]
        def write(a,b):
            calls.append((a,b));return 0 if a==0x20 else len(b)
        with self.assertRaises(OSError):download([Record(0x20,b'\x12\x34')],write)
        self.assertEqual(calls[-1],(0xe600,b'\0'))
        self.assertEqual(len(calls),3)
