import os
from pathlib import Path
import select
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.gateway import Gateway
from ai_coordinator.schema import packet,PacketReader


@unittest.skipUnless(os.name=='posix','Linux pseudo-terminal serial integration')
class GatewayTests(unittest.TestCase):
    def setUp(self):
        import pty
        self.master,self.slave=pty.openpty();os.set_blocking(self.master,False);self.events=[]
        self.gateway=Gateway({'mode':'serial','device':'/dev/serial/by-id/test-peer','allow_motion':True},'session',self.events.append)
        descriptor=os.dup(self.slave)
        with patch('ai_coordinator.gateway.os.open',return_value=descriptor):self.gateway.start()
    def tearDown(self):self.gateway.close();os.close(self.master);os.close(self.slave)
    def receive(self,obj,now):
        os.write(self.master,packet(obj));select.select([self.gateway.fd],[],[],.2)
        return self.gateway.poll(now)
    def hello(self):
        return {'v':1,'kind':'hello','boot_id':'boot','sequence':1,
                'capabilities':['search_v1','target_accept_v1','return_home_v1','cancel_v1']}

    def test_no_transmit_before_compatible_hello(self):
        self.gateway.dispatch({'kind':'start_search','task_id':'task','goal_id':'goal','action_id':'a'})
        self.gateway.poll(10)
        self.assertEqual(self.events[-1]['reason'],'no_compatible_fpga_hello')
        with self.assertRaises(BlockingIOError):os.read(self.master,4096)

    def test_actual_wire_roundtrip_and_sequence_rejection(self):
        self.receive(self.hello(),10);self.assertTrue(self.gateway.ready)
        self.gateway.dispatch({'kind':'start_search','task_id':'t','goal_id':'g','action_id':'a'})
        self.gateway.poll(10.01)
        reader=PacketReader();messages=[];deadline=time.monotonic()+.5
        while len(messages)<2 and time.monotonic()<deadline:
            if select.select([self.master],[],[],max(0,deadline-time.monotonic()))[0]:
                messages.extend(reader.feed(os.read(self.master,8192)))
        self.assertEqual([m['kind'] for m in messages],['heartbeat','start_search'])
        ack={'v':1,'kind':'search_started','boot_id':'boot','session_id':'session','sequence':2,'task_id':'t','goal_id':'g','action_id':'a'}
        self.assertEqual(self.receive(ack,10.1),[ack]);self.assertEqual(self.receive(ack,10.2),[])
        self.assertEqual(self.receive({**ack,'sequence':3,'session_id':'old'},10.3),[])

    def test_link_timeout_revokes_pending_and_requires_hello(self):
        self.receive(self.hello(),10);self.gateway.dispatch({'kind':'start_search'})
        events=self.gateway.poll(11.1)
        self.assertIn({'kind':'link_lost'},events);self.assertFalse(self.gateway.ready);self.assertEqual(self.gateway.pending,b'')

    def test_missing_capability_blocks_command(self):
        self.receive({**self.hello(),'capabilities':['cancel_v1']},10)
        self.gateway.dispatch({'kind':'return_home'})
        self.assertEqual(self.events[-1]['reason'],'unsupported_fpga_capability')

    def test_new_fpga_boot_cancels_old_session_progress(self):
        self.receive(self.hello(),10)
        events=self.receive({**self.hello(),'boot_id':'new','sequence':0},10.1)
        self.assertIn({'kind':'link_reset'},events)

    def test_reopen_clears_partial_frames_and_requires_new_hello(self):
        self.receive(self.hello(),10)
        self.gateway.ai_reader.feed(b'MOT1\x01')
        self.gateway.close()
        descriptor=os.dup(self.slave)
        with patch('ai_coordinator.gateway.os.open',return_value=descriptor):self.gateway.start()
        self.assertFalse(self.gateway.ready)
        self.assertEqual(self.gateway.capabilities,set())
        self.assertFalse(self.gateway.ai_reader.buffer)
        self.receive(self.hello(),11)
        self.assertTrue(self.gateway.ready)


if __name__=='__main__':unittest.main()
