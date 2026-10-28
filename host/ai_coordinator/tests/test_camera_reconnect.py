import asyncio
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,Mock
from ai_coordinator.runtime import Runtime

class CameraReconnectTests(unittest.IsolatedAsyncioTestCase):
    async def test_camera_freezes_frame_time_pose_before_enqueueing_inference(self):
        from ai_coordinator.inspection.mission import InspectionMission
        from ai_coordinator.inspection.controller import InspectionController
        mission=InspectionMission()
        for t,x in [(12.,0),(12.2,1),(12.5,5)]:mission.set_pose(dict(epoch=1,x_m=x,y_m=0,yaw_rad=0),t)
        inspection=SimpleNamespace(mission=mission,observation_round=7)
        inspection.capture_pose=lambda f:InspectionController.capture_pose(inspection,f)
        frame=dict(status='frame',image='/tmp/frame.jpg',format='jpeg',width=128,height=72,
            byteorder=None,sequence=9,received_at_s=12.5)
        rt=Runtime.__new__(Runtime);rt.cfg={};rt.inspection=inspection;rt.camera_backend='v4l2_native'
        rt.core=SimpleNamespace(task_id=0,goal_id=0)
        rt.workers={'camera':SimpleNamespace(call=AsyncMock(side_effect=[{'result':frame},asyncio.CancelledError()]))}
        rt.enqueue_image=Mock();rt.audit=Mock()
        with self.assertRaises(asyncio.CancelledError):await rt._camera()
        sent=rt.enqueue_image.call_args.args[1]['camera_metadata']
        self.assertAlmostEqual(sent['dedup_pose_target_s'],12.2)
        self.assertEqual(sent['dedup_pose']['x_m'],1)
        self.assertEqual(sent['inspection_round'],7)

    async def test_failed_camera_reprobes_identity_without_resuming_motion_or_spawning_duplicates(self):
        rt=Runtime.__new__(Runtime);rt.ai=SimpleNamespace(vision=True,vision_lock=asyncio.Lock(),cleanup=set())
        rt.workers={'camera':object()};rt.camera_task=None;rt.next_camera_retry=0;rt.audit=lambda event:None
        rt._camera=AsyncMock()
        self.assertTrue(rt.retry_camera(1));self.assertFalse(rt.retry_camera(1.1))
        await rt.camera_task
        self.assertFalse(rt.retry_camera(1.2));self.assertTrue(rt.retry_camera(2.1));await rt.camera_task
        rt.ai.vision=False;self.assertFalse(rt.retry_camera(4))
        self.assertEqual(rt._camera.await_count,2)
    async def test_mode_cleanup_and_camera_reopen_cannot_race(self):
        rt=Runtime.__new__(Runtime);lock=asyncio.Lock();await lock.acquire()
        rt.ai=SimpleNamespace(vision=True,vision_lock=lock,cleanup=set());rt.workers={'camera':object()}
        rt.camera_task=None;rt.next_camera_retry=0;rt.audit=lambda event:None;rt._camera=AsyncMock()
        self.assertFalse(rt.retry_camera(1));lock.release()
        self.assertTrue(rt.retry_camera(2));await rt.camera_task
    async def test_inspection_video_keeps_camera_owner_during_manual_pause(self):
        rt=Runtime.__new__(Runtime);rt.ai=SimpleNamespace(vision=False,vision_lock=asyncio.Lock(),cleanup=set())
        rt.inspection=SimpleNamespace(video_required=True);rt.workers={'camera':object()}
        rt.camera_task=None;rt.next_camera_retry=0;rt.audit=lambda e:None;rt._camera=AsyncMock()
        self.assertTrue(rt.retry_camera(1));await rt.camera_task
        rt.inspection.video_required=False;self.assertFalse(rt.retry_camera(3))

if __name__=='__main__':unittest.main()
