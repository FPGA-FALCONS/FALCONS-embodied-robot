"""Paired 10 Hz annotated video; one encoder input gives both views identical PTS.

Only the RK inspection controller creates sessions. Bounded latest snapshots,
fragmented MP4 segments and per-session locks keep shutdown/recovery independent
of the UART, current display page, and the FPGA's original functions.
"""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import threading
import time
import uuid

FPS = 10
WIDTH, HEIGHT, VIDEO_HEIGHT = 816, 420, 448
SESSION_ID = re.compile(r'^\d{8}T\d{6}Z_task\d+_[0-9a-f]{8}$')
MEDIA = re.compile(r'^(camera|map)-(\d{5})\.mp4$')
FORMAT = 'falcons-inspection-paired-v1'


def boot_id():
    p=Path('/proc/sys/kernel/random/boot_id')
    return p.read_text().strip() if p.exists() else None


def atomic(path, value):
    path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w',encoding='utf-8') as out:
        json.dump(value,out,ensure_ascii=False,allow_nan=False,indent=2);out.flush();os.fsync(out.fileno())
    temporary.replace(path)


class Lease:
    def __init__(self,path):
        path=Path(path)
        if path.is_symlink():raise OSError('Refusing a symlink recording lock')
        fd=os.open(path,os.O_RDWR|os.O_CREAT|getattr(os,'O_NOFOLLOW',0),0o600)
        self.file=os.fdopen(fd,'a+b')
        try:
            if os.name=='posix':
                import fcntl
                fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
            else:
                import msvcrt
                self.file.seek(0);self.file.write(b'0');self.file.flush();self.file.seek(0)
                msvcrt.locking(self.file.fileno(),msvcrt.LK_NBLCK,1)
        except BaseException:self.file.close();raise
    def close(self):self.file.close()


def directory_bytes(root):
    return sum(p.stat().st_size for p in Path(root).rglob('*') if p.is_file() and not p.is_symlink())


def session_path(root,identifier):
    root=Path(root).resolve()
    if not SESSION_ID.fullmatch(identifier):raise ValueError('Exact recording identifier required')
    path=root/identifier
    if path.is_symlink() or path.resolve().parent!=root or not path.is_dir():raise ValueError('Recording outside archive')
    return path


def _atoms(stream,end):
    while stream.tell()+8<=end:
        start=stream.tell();header=stream.read(8);size,kind=struct.unpack('>I4s',header);head=8
        if size==1:
            raw=stream.read(8)
            if len(raw)!=8:return
            size=struct.unpack('>Q',raw)[0];head=16
        if size<head or start+size>end:return
        yield start,size,head,kind
        stream.seek(start+size)


def fragments(path):
    """Complete MP4 fragment boundaries and cumulative samples; no decoding."""
    boundaries=[];count=0;pending=None
    with Path(path).open('rb') as stream:
        end=Path(path).stat().st_size
        for start,size,head,kind in _atoms(stream,end):
            if kind==b'moof':
                samples=0
                for sub,n,h,k in _atoms(stream,start+size):
                    if k==b'traf':
                        for inner,m,ih,ik in _atoms(stream,sub+n):
                            if ik==b'trun' and m>=ih+8:
                                stream.read(4);samples+=struct.unpack('>I',stream.read(4))[0]
                pending=samples
            elif kind==b'mdat' and pending:
                count+=pending;boundaries.append((start+size,count));pending=None
    return boundaries


def segment_inventory(path):
    groups={}
    for p in Path(path).iterdir():
        match=MEDIA.fullmatch(p.name)
        if match and not p.is_symlink():groups.setdefault(int(match[2]),{})[match[1]]=p
    result=[]
    for index,streams in sorted(groups.items()):
        if set(streams)!= {'camera','map'}:continue
        counts={name:(fragments(p)[-1][1] if fragments(p) else 0) for name,p in streams.items()}
        if min(counts.values())>0:
            result.append(dict(index=index,camera=streams['camera'].name,map=streams['map'].name,
                               frames=min(counts.values()),stream_frames=counts))
    return result


def trim_timeline(path,frames):
    source=Path(path)/'frames.jsonl'
    if not source.exists():return
    temporary=source.with_suffix('.jsonl.tmp')
    with source.open(encoding='utf-8') as stream,temporary.open('w',encoding='utf-8') as out:
        for line in stream:
            try:event=json.loads(line)
            except ValueError:break
            if event.get('frame',frames)>=frames:break
            out.write(line)
        out.flush();os.fsync(out.fileno())
    temporary.replace(source)


def recover(root):
    """Repair only unlocked, interrupted owned sessions; preserve active/history."""
    root=Path(root);recovered=[]
    if not root.exists():return recovered
    for path in root.iterdir():
        if not path.is_dir() or path.is_symlink() or not SESSION_ID.fullmatch(path.name):continue
        try:
            lease=Lease(path/'.lock')
        except OSError:continue
        try:
            meta=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
            if any(p.is_symlink() for p in path.iterdir()):continue
            if meta.get('format')!=FORMAT:continue
            if meta.get('status') not in ('recording','starting','finalizing'):continue
            # Keep only matching complete fragments of each pair. An incomplete
            # trailing moof/mdat is discarded, never a completed earlier segment.
            groups={}
            for p in path.iterdir():
                match=MEDIA.fullmatch(p.name)
                if match and not p.is_symlink():groups.setdefault(int(match[2]),{})[match[1]]=p
            for index,streams in sorted(groups.items()):
                parts={name:fragments(p) for name,p in streams.items()}
                common=set(n for _,n in parts.get('camera',[])) & set(n for _,n in parts.get('map',[]))
                target=max(common,default=0)
                for name,p in streams.items():
                    if not target:p.unlink()
                    else:
                        end=next(end for end,n in parts[name] if n==target)
                        with p.open('r+b') as out:out.truncate(end);out.flush();os.fsync(out.fileno())
            inventory=segment_inventory(path);frames=sum(x['frames'] for x in inventory)
            trim_timeline(path,frames)
            meta.update(status='interrupted',end_reason='unclean_shutdown',frames=frames,segments=inventory,
                        duration_s=frames/FPS,recovered_at_utc=datetime.now(timezone.utc).isoformat(),
                        termination_kind='rk_reboot' if meta.get('boot_id')!=boot_id() else 'process_interrupted',
                        end_utc_estimated=datetime.fromtimestamp(meta['start_unix_ns']/1e9+frames/FPS,timezone.utc).isoformat()
                        if meta.get('start_unix_ns') else None)
            atomic(path/'manifest.json',meta)
            latest=path/'report.latest.json'
            report=json.loads(latest.read_text(encoding='utf-8')) if latest.exists() else {}
            report.update(recording_interrupted=True,recording_end_reason='unclean_shutdown')
            atomic(path/'report.json',report);recovered.append(path.name)
        except (OSError,ValueError,KeyError,struct.error):
            # Leave original bytes for inspection if this session cannot be read.
            continue
        finally:lease.close()
    return recovered


class Recording:
    def __init__(self,root,config,snapshot,report,*,clock=time.monotonic,writer_factory=None):
        self.root=Path(root);self.cfg=dict(config);self.clock=clock
        self.snapshot_data=snapshot;self.report=report;self.lock=threading.Lock()
        self.stop=threading.Event();self.done=threading.Event();self.ready=False;self.error=None
        self.frames=0;self.reason=None;self.path=None;self.started=clock();self.writer_factory=writer_factory or Encoder
        self.thread=threading.Thread(target=self._run,name='inspection-recorder',daemon=True)
        self.thread.start()
    @property
    def capturing(self):return not self.stop.is_set() and not self.done.is_set()
    def offer(self,snapshot,report=None):
        with self.lock:
            self.snapshot_data=snapshot
            if report is not None:self.report=report
    def finish(self,reason,report):
        with self.lock:self.reason=self.reason or reason;self.report=report
        self.stop.set()
    def status(self):
        return dict(path=str(self.path) if self.path else None,ready=self.ready,capturing=self.capturing,
                    done=self.done.is_set(),frames=self.frames,fps=FPS,error=self.error,end_reason=self.reason)
    def save_final_report(self,report):
        self.wait()
        if self.path:
            value={**report,'recording':self.status(),'recording_end_reason':self.reason or 'stopped'}
            atomic(self.path/'report.json',value)
    def wait(self,timeout=10):
        self.thread.join(timeout)
        if self.thread.is_alive():raise RuntimeError('Recording finalizer did not finish')
    def _pair(self,snapshot,elapsed,utc):
        import cv2
        import numpy as np
        from .render import render
        images=[]
        for view in ('camera','map'):
            image=render(view,snapshot['mission'],snapshot['objects'],camera=snapshot.get('camera'),
                         detections=snapshot.get('detections',[]),radar_raw=snapshot.get('radar'),
                         cursor=snapshot.get('cursor'),calibration_ready=snapshot.get('calibration_ready',False))
            canvas=np.full((VIDEO_HEIGHT,WIDTH,3),(32,24,16),np.uint8);canvas[:HEIGHT]=image
            label='%s  %s  +%.3fs  #%d'%(view.upper(),utc,elapsed,self.frames)
            age=snapshot.get('sources',{}).get('map_age_s')
            if (view=='camera' and snapshot.get('camera') is None) or (view=='map' and (age is None or age>1)):
                label+=' [STALE SOURCE]'
            cv2.putText(canvas,label,(8,440),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,240,240),1,cv2.LINE_AA)
            images.append(canvas)
        return np.concatenate(images,axis=1).tobytes()
    def _run(self):
        encoder=None;lease=None;timeline=None;meta=None
        start_utc=datetime.now(timezone.utc);utc_ns=time.time_ns()
        try:
            self.root.mkdir(mode=0o700,parents=True,exist_ok=True)
            free_limit=int(self.cfg.get('min_free_bytes',2*1024**3));quota=int(self.cfg.get('max_archive_bytes',16*1024**3))
            task_limit=int(self.cfg.get('max_task_bytes',4*1024**3));time_limit=float(self.cfg.get('max_duration_s',7200))
            if min(free_limit,quota,task_limit,time_limit)<=0:raise ValueError('Positive archive limits required')
            segment=self.cfg.get('segment_seconds',60)
            if type(segment) is not int or not 1<=segment<=3600:raise ValueError('Segment duration must be 1..3600 seconds')
            existing=directory_bytes(self.root)
            if existing>=quota or shutil.disk_usage(self.root).free<free_limit:raise OSError('Recording archive is full; export/clean records first')
            task=self.report['mission']['task_id'];identifier=start_utc.strftime('%Y%m%dT%H%M%SZ')+'_task%d_'%task+uuid.uuid4().hex[:8]
            self.path=self.root/identifier;self.path.mkdir(mode=0o700);lease=Lease(self.path/'.lock')
            self.started=self.clock();start_utc=datetime.now(timezone.utc);utc_ns=time.time_ns()
            meta=dict(version=1,format=FORMAT,status='starting',fps=FPS,width=WIDTH,height=VIDEO_HEIGHT,
                      owner_pid=os.getpid(),boot_id=boot_id(),segment_seconds=segment,
                      start_utc=start_utc.isoformat(),start_unix_ns=utc_ns,start_monotonic_s=self.started,
                      local_timezone=datetime.now().astimezone().tzname(),session=self.report['mission']['session_id'],
                      task=task,epoch=self.report['mission']['epoch'],frames=0,segments=[],
                      time_base='frame n: monotonic start + n/10 s; UTC derived from the same start',
                      limits=dict(min_free_bytes=free_limit,max_archive_bytes=quota,max_task_bytes=task_limit,max_duration_s=time_limit))
            atomic(self.path/'manifest.json',meta);atomic(self.path/'report.latest.json',self.report)
            encoder=self.writer_factory(self.path,self.cfg,lease.file.fileno())
            timeline=(self.path/'frames.jsonl').open('w',encoding='utf-8');next_sync=self.started
            while not self.stop.is_set():
                deadline=self.started+self.frames/FPS
                if self.stop.wait(max(0,deadline-self.clock())):break
                now=self.clock()
                if now-deadline>1:raise RuntimeError('Recording cannot maintain 10 Hz')
                if now-self.started>time_limit:raise OSError('Recording duration limit reached')
                with self.lock:snapshot=self.snapshot_data;report=self.report
                utc=datetime.fromtimestamp((utc_ns+round(self.frames*1e9/FPS))/1e9,timezone.utc).isoformat(timespec='milliseconds')
                encoder.write(self._pair(snapshot,self.frames/FPS,utc))
                timeline.write(json.dumps(dict(frame=self.frames,relative_ms=self.frames*100,utc=utc,
                    scheduled_monotonic_s=deadline,sampled_monotonic_s=now,phase=snapshot['mission']['phase'],
                    state=snapshot['mission']['state'],manual=snapshot['mission'].get('manual',False),
                    waypoint_index=snapshot['mission'].get('waypoint_index',0),
                    waypoint_count=snapshot['mission'].get('waypoint_count',0),
                    waypoints_completed=snapshot['mission'].get('waypoints_completed',0),
                    goal_m=snapshot['mission'].get('goal_m'),
                    sources=snapshot.get('sources',{})),ensure_ascii=False,allow_nan=False)+'\n')
                self.frames+=1
                if not self.ready:
                    inventory=segment_inventory(self.path)
                    self.ready=bool(inventory)
                if now>=next_sync:
                    size=directory_bytes(self.path)
                    if size>task_limit or existing+size>quota or shutil.disk_usage(self.root).free<free_limit:
                        raise OSError('Recording storage budget reached')
                    for p in self.path.glob('*.mp4'):
                        with p.open('r+b') as media:os.fsync(media.fileno())
                    timeline.flush();os.fsync(timeline.fileno())
                    meta.update(status='recording',frames=self.frames,duration_s=self.frames/FPS,segments=segment_inventory(self.path))
                    atomic(self.path/'report.latest.json',report);atomic(self.path/'manifest.json',meta)
                    next_sync=now+1
        except Exception as exc:self.error=type(exc).__name__+': '+str(exc)[:240];self.reason=self.reason or 'recording_error'
        finally:
            if encoder:
                try:encoder.close()
                except Exception as exc:self.error=self.error or str(exc)[:240];self.reason=self.reason or 'encoder_error'
            if timeline:
                try:timeline.flush();os.fsync(timeline.fileno());timeline.close()
                except OSError as exc:self.error=self.error or str(exc)[:240]
            try:
                if meta is not None:
                    inventory=segment_inventory(self.path)
                    if not inventory:
                        for p in self.path.glob('*.mp4'):
                            if MEDIA.fullmatch(p.name):p.unlink()
                    valid_frames=sum(x['frames'] for x in inventory)
                    if valid_frames<self.frames:trim_timeline(self.path,valid_frames)
                    self.frames=valid_frames
                    meta.update(status='error' if self.error else 'closed',error=self.error,end_reason=self.reason or 'stopped',
                                end_utc=datetime.now(timezone.utc).isoformat(),frames=self.frames,duration_s=self.frames/FPS,segments=inventory)
                    with self.lock:report=dict(self.report)
                    report.update(recording={**self.status(),'done':True,'capturing':False},recording_end_reason=meta['end_reason'])
                    atomic(self.path/'report.json',report);atomic(self.path/'manifest.json',meta)
            except Exception as exc:self.error=self.error or str(exc)[:240]
            if lease:lease.close()
            self.done.set()


class Encoder:
    def __init__(self,path,cfg,lock_fd):
        executable=shutil.which(cfg.get('ffmpeg','ffmpeg'))
        if not executable:raise RuntimeError('Existing FFmpeg with libx264 is required')
        self.log=(path/'encoder.log').open('wb')
        args=[executable,'-hide_banner','-loglevel','error','-nostdin','-filter_complex_threads','1','-f','rawvideo','-pixel_format','bgr24',
              '-video_size','%dx%d'%(WIDTH*2,VIDEO_HEIGHT),'-framerate',str(FPS),'-i','pipe:0',
              '-filter_complex','[0:v]split=2[c][m];[c]crop=816:448:0:0[cam];[m]crop=816:448:816:0[map]']
        for view,label in [('camera','cam'),('map','map')]:
            args+=['-map','['+label+']','-an','-c:v','libx264','-preset','ultrafast','-tune','zerolatency',
                   '-crf','24','-pix_fmt','yuv420p','-threads','2','-g','10','-bf','0','-sc_threshold','0',
                   '-f','segment','-segment_time',str(cfg.get('segment_seconds',60)),'-reset_timestamps','1',
                   '-segment_format','mp4','-segment_format_options',
                   'movflags=+empty_moov+frag_keyframe+default_base_moof:frag_duration=100000',str(path/(view+'-%05d.mp4'))]
        self.proc=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=self.log,
                                   **({'pass_fds':(lock_fd,)} if os.name=='posix' else {}))
        if os.name=='posix':os.set_blocking(self.proc.stdin.fileno(),False)
    def write(self,data):
        if self.proc.poll() is not None:raise RuntimeError('Encoder exited; inspect encoder.log')
        if os.name=='posix':
            import select
            offset=0;deadline=time.monotonic()+1
            while offset<len(data):
                wait=deadline-time.monotonic()
                if wait<=0 or not select.select([], [self.proc.stdin], [],wait)[1]:raise TimeoutError('Encoder input stalled')
                try:offset+=os.write(self.proc.stdin.fileno(),data[offset:offset+65536])
                except BlockingIOError:continue
        else:self.proc.stdin.write(data);self.proc.stdin.flush()
    def close(self):
        try:
            self.proc.stdin.close()
            try:code=self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill();self.proc.wait(timeout=2);raise RuntimeError('Encoder finalization timed out')
            if code:raise RuntimeError('Encoder failed with status %d'%code)
        finally:self.log.close()
