"""Linux pseudo-terminal tests: one serial owner, MOT1 frames, heartbeats and STOP.

These are emulated serial tests.  They prove byte scheduling and host state;
they do not prove that an FPGA received a frame or that a wheel moved.
"""
import asyncio
import os
from pathlib import Path
import select
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from ai_coordinator.gateway import Gateway
from ai_coordinator.motion_backend import MotionBackend
from ai_coordinator.motion_intents import inspect_motion
from ai_coordinator.motion_reference.mot1 import decode_frames
from ai_coordinator.schema import PacketReader,packet


@unittest.skipUnless(os.name=='posix','Linux pseudo-terminal serial integration')
class MotionPtyTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        import pty
        self.master,self.slave=pty.openpty()
        os.set_blocking(self.master,False);os.set_blocking(self.slave,False)
        self.events=[]
        self.gateway_cfg={'mode':'serial','device':'/dev/serial/by-id/pty-test','allow_motion':False,'link_timeout_s':1}
        self.gateway=Gateway(self.gateway_cfg,'pty-session',self.events.append)
        descriptor=os.dup(self.slave)
        with patch('ai_coordinator.gateway.os.open',return_value=descriptor):
            self.gateway.start()
        self.cfg={'gateway':self.gateway_cfg,
                  'voice_motion':{'enabled':True,'live_commissioned':True,'backend':'mot1_shared_serial',
                      'speed':10,'heartbeat_s':.05,'turn_window_s':.4,'drive_margin_s':.05,
                      'distance':{'calibrated':True,'calibrated_speed_mps':.5}}}
        self.backend=MotionBackend(lambda:self.gateway.transport,self.cfg,self.events.append)

    async def asyncTearDown(self):
        self.backend.closed=True
        self.gateway.close()
        os.close(self.master);os.close(self.slave)

    def read_available(self,timeout=.3):
        data=b''
        while select.select([self.master],[],[],timeout)[0]:
            try:chunk=os.read(self.master,4096)
            except BlockingIOError:break
            if not chunk:break
            data+=chunk;timeout=.02
        return data

    async def test_shared_owner_heartbeat_stop_and_no_ack_claims(self):
        decision=inspect_motion('前进',distance={'calibrated':True,'calibrated_speed_mps':.5},speed_code=10)
        result=self.backend.execute(decision,'前进')
        self.assertTrue(result['accepted']);self.assertTrue(result['sent'])
        self.assertIs(self.backend.transport,self.gateway.transport)
        await asyncio.sleep(.24)
        frames,remainder,errors=decode_frames(self.read_available())
        self.assertEqual(errors,0);self.assertEqual(remainder,b'')
        self.assertEqual(frames[0].cmd,1)
        self.assertGreaterEqual(sum(frame.cmd==8 for frame in frames),3)
        # A human-readable serial reply is telemetry, not a task receipt.
        os.write(self.master,b'ACK\r\n')
        self.gateway.poll(time.monotonic())
        self.assertFalse(result['device_reply_verified']);self.assertFalse(result['action_complete_verified'])
        self.assertFalse(result['physical_task_completed'])
        stop=self.backend.stop_now('pty_stop')
        self.assertTrue(stop['stop_frame_sent']);self.assertFalse(stop['physical_stop_verified'])
        await asyncio.sleep(.02)
        frames,_,errors=decode_frames(self.read_available())
        self.assertEqual(errors,0);self.assertEqual(frames[-1].cmd,0)

    async def test_second_flock_cannot_own_the_same_serial(self):
        # Same exclusive-lock contract used by send_motion.open_serial.
        import fcntl
        fd=os.open(os.ttyname(self.slave),os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        finally:os.close(fd)

    async def test_fam1_and_mot1_share_one_wire_without_interleaving(self):
        hello={'v':1,'kind':'hello','boot_id':'pty-boot','sequence':1,
               'capabilities':['search_v1','target_accept_v1','cancel_v1']}
        os.write(self.master,packet(hello))
        select.select([self.gateway.fd],[],[],.2)
        self.assertEqual(self.gateway.poll(time.monotonic()),[])
        self.assertTrue(self.gateway.ready)
        self.gateway.cfg['allow_motion']=True  # exercise FAM1 wire; MOT1 uses its own gate
        decision=inspect_motion('左转',distance=None,speed_code=10)
        self.backend.execute(decision,'左转')
        self.gateway.dispatch({'kind':'start_search','task_id':'t','goal_id':'g','action_id':'a'})
        self.gateway.poll(time.monotonic()+.01)
        raw=self.read_available()
        # Parse FAM1 packets and MOT1 frames independently; neither parser sees
        # a partial frame from the other.
        packets=PacketReader().feed(raw)
        frames,remainder,errors=decode_frames(raw)
        self.assertTrue(any(p.get('kind')=='start_search' for p in packets))
        self.assertTrue(any(frame.cmd==3 for frame in frames))
        # Each parser intentionally leaves bytes belonging to the other
        # protocol in its resynchronization remainder; no false decode occurs.
        self.assertEqual(errors,0)


if __name__=='__main__':unittest.main()
