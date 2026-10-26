import asyncio
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock
from ai_coordinator.runtime import Runtime

class CameraReconnectTests(unittest.IsolatedAsyncioTestCase):
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

if __name__=='__main__':unittest.main()
