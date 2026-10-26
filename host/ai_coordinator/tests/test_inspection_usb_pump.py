import unittest
from ai_coordinator.inspection.usb_pump import USBPump
from ai_coordinator.inspection.stream import Container,Reader,DIAGNOSTIC,DISPLAY,CONTROL,MAP,MAX_PAYLOAD,diagnostic_reply
from ai_coordinator.inspection import wire


class FakeLink:
    def __init__(self,cfg):
        self.reader=Reader();self.frames=[];self.incoming=bytearray();self.closed=False
    def write_once(self,data,offset,timeout_ms):
        end=min(len(data),offset+4096)
        for frame in self.reader.feed(data[offset:end],now=1):
            self.frames.append(frame)
            if frame.kind==DIAGNOSTIC:self.incoming.extend(diagnostic_reply(frame).encode())
        return end
    def read_once(self,timeout_ms):
        data=bytes(self.incoming[:17]);del self.incoming[:17];return data
    def close(self):self.closed=True


class PumpTests(unittest.TestCase):
    def setUp(self):
        self.now=1.;self.links=[]
        def factory(cfg):
            link=FakeLink(cfg);self.links.append(link);return link
        self.p=USBPump({},factory=factory,clock=lambda:self.now)
        self.p.bind(1,2,3,4)
    def tearDown(self):self.p.close()
    def cycle(self,n=1):
        for _ in range(n):self.p._cycle()
    def verified(self):
        self.cycle(10);self.assertTrue(self.p.status()['verified'])
    def display(self,sequence):
        return Container(DISPLAY,1,2,3,4,sequence,816,420,1632,0,b'\x12\x34'*(MAX_PAYLOAD//2))
    def control(self,sequence):
        return Container(CONTROL,1,2,3,4,sequence,payload=wire.encode(wire.LAYER_BEGIN,[1,2,3,4,5,6]))
    def test_challenge_priority_latest_only_and_partial_integrity(self):
        self.assertTrue(self.p.offer(self.display(1)));self.p.offer(self.display(2));self.p.offer(self.control(3))
        self.verified();self.cycle(200)
        self.assertEqual([x.kind for x in self.links[0].frames],[DIAGNOSTIC,CONTROL,DISPLAY])
        self.assertEqual(self.links[0].frames[-1],self.display(2))
        self.assertFalse(self.links[0].reader.buffer)
    def test_binding_discards_pending_but_never_splices_active_container(self):
        self.verified();old=self.display(5);self.p.offer(old);self.cycle()
        self.p.offer(self.control(7));self.p.bind(1,8,3,9)
        self.assertFalse(self.p.offer(self.display(6)))
        self.cycle(200)
        self.assertEqual(self.links[0].frames[-1],old)
        self.assertEqual(self.p.status()['pending_control'],0)
    def test_queue_bounds_and_no_movement_commands_on_usb(self):
        for _ in range(8):self.assertTrue(self.p.offer(self.control(1)))
        self.assertFalse(self.p.offer(self.control(2)))
        for payload in (wire.encode(wire.CONTROL,[1]*8),wire.encode(wire.HELLO,[1]),b'junk'*3,
                        wire.encode(wire.LAYER_COMMIT,[1]*5)*2):
            with self.subTest(payload=payload[:12]),self.assertRaises(ValueError):
                self.p.offer(Container(CONTROL,1,2,3,4,5,payload=payload))
    def test_reconnect_requires_new_challenge_and_discards_queued_work(self):
        self.verified();self.p.offer(self.display(1));self.p.offer(self.control(3))
        old=self.links[0];self.p._disconnect('unplugged')
        self.assertFalse(self.p.status()['verified']);self.assertTrue(old.closed)
        self.assertFalse(self.p.status()['pending_display']);self.assertEqual(self.p.status()['pending_control'],0)
        self.cycle();self.assertEqual(len(self.links),1)
        self.now+=2;self.verified();self.assertEqual(self.p.status()['generation'],2)
        self.assertNotEqual(old.frames[0].payload,self.links[1].frames[0].payload)
    def test_plain_firmware_loopback_is_not_an_fpga_protocol_receipt(self):
        self.cycle();self.links[0].incoming.clear()
        self.links[0].incoming.extend(self.links[0].frames[0].encode())
        self.cycle(10);self.assertFalse(self.p.status()['verified'])
        self.now+=4
        with self.assertRaisesRegex(OSError,'diagnostic'):self.cycle()
    def test_only_verified_bound_maps_are_retained_and_receive_queue_is_bounded(self):
        self.verified()
        for n in range(5):
            frame=Container(MAP,1,2,3,4,n,256,256,32,0,b'\0'*8192)
            self.links[0].incoming.extend(frame.encode())
        self.cycle(2500)
        self.assertEqual([f.sequence for f in self.p.drain_maps()],[3,4])
        self.assertFalse(self.p.drain_maps())
        wrong=Container(MAP,1,2,3,9,8,256,256,32,0,b'\0'*8192)
        self.links[0].incoming.extend(wrong.encode());self.cycle(500)
        self.assertFalse(self.p.drain_maps())


if __name__=='__main__':unittest.main()
