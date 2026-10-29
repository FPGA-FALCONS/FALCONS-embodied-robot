import math
import unittest
from ai_coordinator.inspection.timebase import ClockSync,PoseBuffer,capture_time

class TimebaseTests(unittest.TestCase):
    def clock(self,host=10,offset=10):
        c=ClockSync(1,2);request=c.request(host)
        t2=round((host+offset+.003)*1e6)&0xffffffff;t3=(t2+2000)&0xffffffff
        self.assertTrue(c.reply([1,2,request[2],t2,t3],host+.008))
        return c
    def test_network_delay_bounds_and_staleness_are_preserved(self):
        c=self.clock();offset,error=c.estimate(10.01)
        self.assertAlmostEqual(offset,10,places=6);self.assertGreaterEqual(error,.003)
        host,uncertainty=c.to_host(20_004_000,10.01)
        self.assertAlmostEqual(host,10.004,places=6)
        with self.assertRaisesRegex(ValueError,'stale'):c.estimate(14)
    def test_foreign_session_epoch_and_replayed_nonce_cannot_adjust_clock(self):
        c=ClockSync(1,2);req=c.request(1)
        self.assertFalse(c.reply([9,2,req[2],100,200],1.01))
        self.assertFalse(c.reply([1,9,req[2],100,200],1.01))
        self.assertTrue(c.reply([1,2,req[2],1000000,1000001],1.01))
        self.assertFalse(c.reply([1,2,req[2],1000000,1000001],1.01))
    def test_32bit_device_wrap_keeps_host_time_continuous(self):
        c=self.clock(100,4294.960-100)
        req=c.request(100.020)
        self.assertTrue(c.reply([1,2,req[2],(4294983000&0xffffffff),(4294985000&0xffffffff)],100.028))
        at,error=c.to_host(4294984000&0xffffffff,100.03)
        self.assertAlmostEqual(at,100.024,places=6)
    def test_processing_or_delay_inconsistent_with_receipt_is_rejected(self):
        for t2,t3,received in [(1000000,900000,1.01),(1000000,1100000,1.01),(1000000,1000001,1.3)]:
            c=ClockSync(1,2);req=c.request(1)
            self.assertFalse(c.reply([1,2,req[2],t2,t3],received))
    def test_pose_interpolation_uses_device_time_and_shortest_yaw(self):
        c=self.clock();p=PoseBuffer(2)
        self.assertTrue(p.add([2,1,20000000,0,0,17900,1]))
        self.assertTrue(p.add([2,2,20100000,100,200,(-17900)&0xffffffff,1]))
        pose,error=p.at(10.05,c,10.12)
        self.assertAlmostEqual(pose['x_m'],.05);self.assertAlmostEqual(pose['y_m'],.1)
        self.assertAlmostEqual(abs(pose['yaw_rad']),math.pi)
        self.assertFalse(p.add([3,3,20100000,0,0,0,1]))
        self.assertFalse(p.add([2,1,20100000,0,0,0,1]))
        with self.assertRaisesRegex(ValueError,'bracketing'):p.at(10.15,c,10.16)
    def test_capture_profile_must_bind_exact_driver_clock_mode_and_identity(self):
        frame=dict(width=1280,height=720,negotiated=dict(width=1280,height=720,fps=30,fourcc='MJPG'),
                   timestamp_origin='v4l2_driver',timestamp_clock='monotonic',timestamp_source='start_of_exposure',
                   driver_timestamp_s=10,received_at_s=10.03)
        profile=dict(measured=True,camera_serial='camera1',image_size=[1280,720],mode=frame['negotiated'],
                     timestamp_source='start_of_exposure',driver_to_exposure_s=-.002,error_bound_s=.004)
        self.assertEqual(capture_time(frame,profile,'camera1'),(9.998,.004))
        for change in [dict(measured=False),dict(camera_serial='other'),dict(timestamp_source='end_of_frame'),
                       dict(mode={}),dict(error_bound_s=0)]:
            with self.subTest(change=change),self.assertRaises(ValueError):capture_time(frame,{**profile,**change},'camera1')
        with self.assertRaises(ValueError):capture_time({**frame,'timestamp_origin':'host_receipt_only'},profile,'camera1')

if __name__=='__main__':unittest.main()
