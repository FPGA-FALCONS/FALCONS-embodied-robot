import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import threading
import urllib.request
from http.server import ThreadingHTTPServer
import numpy as np
from ai_coordinator.inspection.recording import Recording,Encoder,Lease,fragments,recover,segment_inventory,FORMAT
from ai_coordinator.inspection.recording_cli import delete,catalog,handler


def snapshot(state='outbound',manual=False):
    return dict(mission=dict(state=state,phase='outbound',manual=manual,trace_m=[],goal_m=[1,0],
                             pose=dict(x_m=0,y_m=0,yaw_rad=0)),objects=[],
                camera=dict(bgr=np.zeros((72,128,3),np.uint8),sequence=1),detections=[],
                radar=bytes(8192),sources={'camera_sequence':1})


def atom(kind,body):return struct.pack('>I4s',len(body)+8,kind)+body
def fragment(samples):return atom(b'moof',atom(b'traf',atom(b'trun',bytes(4)+struct.pack('>I',samples))))+atom(b'mdat',bytes(32))


class ArchiveTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def create(self):
        p=self.root/'20261027T010203Z_task1_1234abcd';p.mkdir()
        (p/'manifest.json').write_text(json.dumps({'format':FORMAT,'status':'recording'}));(p/'report.latest.json').write_text('{"mission":{"task_id":1}}')
        return p
    def test_recovery_trims_incomplete_tail_and_aligns_both_views(self):
        p=self.create();base=atom(b'ftyp',b'isom')+atom(b'moov',b'')
        (p/'camera-00000.mp4').write_bytes(base+fragment(1)+fragment(1)+b'broken')
        (p/'map-00000.mp4').write_bytes(base+fragment(1)+b'\0\0\0\x30mdatpartial')
        self.assertEqual(recover(self.root),[p.name])
        self.assertEqual(fragments(p/'camera-00000.mp4')[-1][1],1)
        self.assertEqual(fragments(p/'map-00000.mp4')[-1][1],1)
        self.assertEqual(json.loads((p/'manifest.json').read_text())['status'],'interrupted')
        self.assertTrue(json.loads((p/'report.json').read_text())['recording_interrupted'])
        self.assertFalse(recover(self.root))
    def test_active_recording_cannot_be_deleted_or_recovered(self):
        p=self.create();lease=Lease(p/'.lock')
        try:
            with self.assertRaises(OSError):delete(self.root,p.name)
            self.assertEqual(recover(self.root),[])
        finally:lease.close()
        delete(self.root,p.name);self.assertFalse(p.exists())
    def test_cleanup_requires_exact_owned_id_and_rejects_unexpected_files(self):
        p=self.create();(p/'user-note.txt').write_text('keep')
        with self.assertRaises(ValueError):delete(self.root,p.name)
        with self.assertRaises(ValueError):delete(self.root,'../other')
        self.assertEqual((p/'user-note.txt').read_text(),'keep')
        self.assertEqual(catalog(self.root)[0]['id'],p.name)
    def test_full_archive_refuses_start_without_creating_media(self):
        r=Recording(self.root,{'min_free_bytes':2**63},snapshot(),{'mission':{'session_id':1,'task_id':1,'epoch':1}})
        r.wait();self.assertIsNotNone(r.error);self.assertFalse(r.ready)
        self.assertEqual(list(self.root.iterdir()),[])
    def test_readonly_player_serves_exact_video_ranges_and_denies_path_escape(self):
        p=self.create();data=atom(b'ftyp',b'isom')+fragment(1);(p/'camera-00000.mp4').write_bytes(data)
        server=ThreadingHTTPServer(('127.0.0.1',0),handler(self.root));thread=threading.Thread(target=server.serve_forever);thread.start()
        url='http://127.0.0.1:%d'%server.server_port;op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with op.open(urllib.request.Request(url+'/'+p.name+'/camera-00000.mp4',headers={'Range':'bytes=0-7'})) as response:
                self.assertEqual(response.status,206);self.assertEqual(response.read(),data[:8])
            with self.assertRaises(urllib.error.HTTPError) as error:op.open(url+'/'+p.name+'/../manifest.json')
            error.exception.close()
            with op.open(url+'/api') as response:self.assertEqual(json.load(response)[0]['id'],p.name)
        finally:server.shutdown();server.server_close();thread.join()


@unittest.skipUnless(shutil.which('ffmpeg'),'Existing FFmpeg required')
class EncoderIntegrationTests(unittest.TestCase):
    def test_temporary_recording_lag_does_not_abort_session(self):
        class DelayedEncoder(Encoder):
            delayed=False
            def write(self,data):
                super().write(data)
                if not self.delayed:
                    self.delayed=True
                    time.sleep(1.2)
        with tempfile.TemporaryDirectory() as tmp:
            report={'mission':{'session_id':1,'task_id':1,'epoch':1}}
            r=Recording(tmp,{'min_free_bytes':1},snapshot(),report,writer_factory=DelayedEncoder)
            deadline=time.monotonic()+5
            try:
                while r.frames<15 and not r.error and time.monotonic()<deadline:time.sleep(.02)
            finally:r.finish('cancelled',report);r.wait()
            self.assertIsNone(r.error)
            self.assertGreaterEqual(r.frames,15)
            rows=[json.loads(x) for x in (r.path/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertGreater(max(x['sampled_monotonic_s']-x['scheduled_monotonic_s'] for x in rows),1)
            self.assertEqual([x['relative_ms'] for x in rows],list(range(0,len(rows)*100,100)))
            self.assertTrue(all(x['stream_frames']['camera']==x['stream_frames']['map'] for x in segment_inventory(r.path)))

    def test_waypoint_progress_pause_and_return_share_one_synchronised_video_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            current=snapshot();current['mission'].update(waypoints_m=[[1,0],[2,1]],waypoint_index=0,
                waypoint_count=2,waypoints_completed=0)
            report={'mission':{'session_id':1,'task_id':1,'epoch':1}}
            r=Recording(tmp,{'min_free_bytes':1,'segment_seconds':1},current,report)
            deadline=time.monotonic()+6
            try:
                for index,state,phase,completed,manual in ((0,'outbound','outbound',0,False),
                        (1,'outbound','outbound',1,False),(1,'manual_paused','outbound',1,True),
                        (2,'returning','returning',2,False)):
                    current=snapshot(state,manual);current['mission'].update(waypoints_m=[[1,0],[2,1]],
                        waypoint_index=index,waypoint_count=2,waypoints_completed=completed,phase=phase)
                    before=r.frames;r.offer(current)
                    while r.frames<before+4 and not r.error and time.monotonic()<deadline:time.sleep(.02)
            finally:r.finish('completed',{'mission':current['mission']});r.wait()
            self.assertIsNone(r.error)
            self.assertEqual(len(list(Path(tmp).glob('*/manifest.json'))),1)
            rows=[json.loads(s) for s in (r.path/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual({row['waypoint_index'] for row in rows},{0,1,2})
            self.assertTrue(any(row['manual'] and row['waypoint_index']==1 for row in rows))
            self.assertTrue(any(row['phase']=='returning' and row['waypoints_completed']==2 for row in rows))
            self.assertEqual([row['relative_ms'] for row in rows],list(range(0,len(rows)*100,100)))
            self.assertTrue(all(v['stream_frames']['camera']==v['stream_frames']['map'] for v in segment_inventory(r.path)))
            final=json.loads((r.path/'report.json').read_text(encoding='utf-8'))
            self.assertEqual(final['mission']['waypoints_m'],[[1,0],[2,1]])

    def test_actual_dual_mp4_segments_are_ten_hz_and_survive_manual_pause(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=Recording(tmp,{'min_free_bytes':1,'segment_seconds':1},snapshot(),
                        {'mission':{'session_id':1,'task_id':1,'epoch':1}})
            deadline=time.monotonic()+5
            try:
                while r.frames<7 and not r.error and time.monotonic()<deadline:time.sleep(.02)
                r.offer(snapshot('manual_paused',True))
                while r.frames<22 and not r.error and time.monotonic()<deadline:time.sleep(.02)
            finally:r.finish('cancelled',{'mission':{'state':'cancelled'}});r.wait()
            self.assertIsNone(r.error);self.assertTrue(r.ready)
            meta=json.loads((r.path/'manifest.json').read_text(encoding='utf-8'));self.assertEqual(meta['fps'],10)
            self.assertGreaterEqual(meta['frames'],22);self.assertGreaterEqual(len(meta['segments']),2)
            self.assertTrue(all(v['stream_frames']['camera']==v['stream_frames']['map'] for v in meta['segments']))
            rows=[json.loads(line) for line in (r.path/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual([x['relative_ms'] for x in rows],list(range(0,len(rows)*100,100)))
            self.assertTrue(any(x['manual'] for x in rows))
            self.assertTrue((r.path/'report.json').exists())
            self.assertEqual(recover(tmp),[])
            if shutil.which('ffprobe'):
                pts=[]
                for view in ('camera','map'):
                    data=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
                        '-count_frames','-show_entries','stream=r_frame_rate,nb_read_frames','-of','json',str(r.path/meta['segments'][0][view])]))
                    self.assertEqual(data['streams'][0]['r_frame_rate'],'10/1')
                    self.assertEqual(int(data['streams'][0]['nb_read_frames']),meta['segments'][0]['frames'])
                    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_frames',
                        '-show_entries','frame=best_effort_timestamp_time','-of','json',str(r.path/meta['segments'][0][view])]))
                    pts.append([float(f['best_effort_timestamp_time']) for f in probe['frames']])
                self.assertEqual(pts[0],pts[1]);self.assertTrue(all(abs(b-a-.1)<1e-5 for a,b in zip(pts[0],pts[0][1:])))
    def test_killed_parent_is_recovered_without_keeping_a_live_recorder(self):
        with tempfile.TemporaryDirectory() as tmp:
            code="""import sys,time
from ai_coordinator.inspection.recording import Recording
from ai_coordinator.tests.test_inspection_recording import snapshot
r=Recording(sys.argv[1],{'min_free_bytes':1,'segment_seconds':1},snapshot(),{'mission':{'session_id':1,'task_id':1,'epoch':1}})
while True:time.sleep(.1)
"""
            proc=subprocess.Popen([sys.executable,'-c',code,tmp],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            try:
                deadline=time.monotonic()+6;path=None
                while time.monotonic()<deadline:
                    paths=list(Path(tmp).glob('*/manifest.json'))
                    if paths:
                        path=paths[0].parent
                        if json.loads(paths[0].read_text(encoding='utf-8')).get('frames',0)>=11:break
                    time.sleep(.05)
                self.assertIsNotNone(path)
                proc.kill();proc.wait(timeout=3)
                deadline=time.monotonic()+4;result=[]
                while time.monotonic()<deadline and not result:result=recover(tmp);time.sleep(.05)
                self.assertEqual(result,[path.name])
                meta=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
                self.assertGreater(meta['frames'],0);self.assertEqual(meta['status'],'interrupted')
                self.assertTrue(all(x['stream_frames']['camera']==x['stream_frames']['map'] for x in meta['segments']))
                self.assertTrue((path/'report.json').exists())
            finally:
                if proc.poll() is None:proc.kill();proc.wait(timeout=3)
