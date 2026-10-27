"""One RK inspection task controller using the coordinator's serial owner.

Models/capture stay in existing owned workers; USB only carries display/layers.
No calibration defaults and no legacy direct-drive command is used here.
"""
import asyncio
from collections import deque
import math
import json
import hashlib
from pathlib import Path
import time
import uuid
from . import wire
from .mission import InspectionMission,ACTIVE
from .evidence import Evidence
from .objects import Objects
from .calibration import Calibration,finite
from .grid import build,cell,bit
from .stream import Container,DISPLAY,CONTROL,route_points
from .usb_pump import USBPump
from .timebase import ClockSync,PoseBuffer,capture_time
from .recording import Recording,recover
from .discovery_speech import DiscoverySpeech


class InspectionController:
    def __init__(self,rt,*,usb_factory=USBPump,recording_factory=Recording):
        self.rt=rt;self.cfg=rt.cfg.get('inspection',{})
        self.enabled=self.cfg.get('enabled') is True
        source_timeout=self.cfg.get('recording',{}).get('source_timeout_s',5)
        if not finite(source_timeout) or not 1<=source_timeout<=60:raise ValueError('Video source timeout must be1..60 seconds')
        playback_port=self.cfg.get('recording',{}).get('playback_port',18766)
        if type(playback_port) is not int or (playback_port!=0 and not 1024<=playback_port<=65535):raise ValueError('Playback port must be0 or1024..65535')
        fps=self.cfg.get('highspeed',{}).get('display_fps',10)
        if not finite(fps) or not 1<=fps<=10:raise ValueError('Display fps must be1..10')
        self.render_interval=1/fps
        self.mission=InspectionMission();self.objects=Objects();self.calibration=Calibration(self.cfg.get('calibration',{}))
        self.usb=usb_factory(self.cfg['highspeed']) if self.enabled and self.cfg.get('highspeed',{}).get('enabled') is True else None
        self.input=None;self.actual=None;self.caps=None;self.clock=None;self.poses=None
        self.epoch=None;self.sequence=0;self.context=0;self.latest_ai=None;self.ai_sequence=None
        self.last_input=self.last_state=self.last_map=self.last_camera=None
        self.next_hello=self.next_heartbeat=self.next_clock=self.next_render=self.next_stick=0.
        self.cursor=[0.,0.];self.scroll=0;self.camera=None;self.detections=[];self.radar=None;self.planned_route=[];self.route_history=[];self.route_key=None;self.last_route=None
        self.notice='未标定：可观察和选点';self.error=None;self.instruction='';self.recording_phase=None;self.report_path=None;self.next_upload_retry=0.
        self.layer=None;self.layer_crc=None;self.layer_dirty=False;self.layer_job=None;self.upload=None;self.upload_info=None;self.upload_pending=None
        self.render_job=None;self.record_job=None;self.released=None;self.record_armed=False
        self.usb_was_verified=False
        self.evidence=Evidence(rt.run_dir/'inspection_evidence');self.evidence_jobs=set();self.evidence_error=None;self.evidence_files=[]
        self.want_close=None;self.next_close=0.
        self.video=None;self.recording_factory=recording_factory;self.video_next_report=0.;self.video_recovery=None
        self.video_camera_lease=False
        self.video_usb_lost_since=None
        self.archive_server=None;self.archive_thread=None;self.playback_error=None
        self.video_root=Path(rt.cfg.get('root',rt.run_dir))/'inspection_recordings'
        self.generation=0;self.active=False;self.stopped=False;self.closed=False
        self.recognition_display=False
        self.discovery_speech=DiscoverySpeech(rt,self._speech_allowed)
    def _speech_allowed(self,binding):
        return bool(self.active and not self.closed and self.mission.task_id and
            binding==(self.mission.session_id,self.mission.task_id,self.epoch) and
            not self.mission.manual and self.mission.state in ACTIVE)
    def start(self):
        if self.usb:self.usb.start()
        if self.enabled:
            from http.server import ThreadingHTTPServer
            import threading
            from .recording_cli import handler
            port=self.cfg.get('recording',{}).get('playback_port',18766)
            if port:
                try:
                    self.archive_server=ThreadingHTTPServer(('127.0.0.1',port),handler(self.video_root))
                    self.archive_thread=threading.Thread(target=self.archive_server.serve_forever,name='inspection-playback',daemon=True)
                    self.archive_thread.start()
                except OSError as exc:self.playback_error=str(exc)[:160]
            async def restore():
                try:
                    for delay in (0,2,4):
                        if delay:await asyncio.sleep(delay)
                        await asyncio.to_thread(recover,self.video_root)
                except OSError as exc:self.evidence_error='Recording recovery failed: '+str(exc)[:100]
            self.video_recovery=asyncio.create_task(restore())
    @property
    def video_required(self):return bool(self.video and self.video.capturing)
    def _video_snapshot(self,now):
        return dict(mission={**self.mission.snapshot(),'instruction':self.instruction,'recording_phase':self.recording_phase,
                             'planned_route':list(self.planned_route)},objects=self.objects.snapshot(),
                    camera=self.camera if self.last_camera is not None and now-self.last_camera<=.5 else None,
                    detections=list(self.detections),radar=self.radar,cursor=list(self.cursor),
                    calibration_ready=all(self.readiness(now).values()),
                    sources=dict(camera_age_s=None if self.last_camera is None else now-self.last_camera,
                                 map_age_s=None if self.last_map is None else now-self.last_map,
                                 camera_sequence=self.camera.get('sequence') if self.camera else None,
                                 pose=self.mission.pose,view=self.mission.view))
    def _begin_video(self,now):
        if self.video and not self.video.done.is_set():return
        try:
            self.video=self.recording_factory(self.video_root,self.cfg.get('recording',{}),self._video_snapshot(now),self._report())
            self.video_camera_lease=True
        except Exception as exc:
            self.evidence_error='Recording start failed: '+str(exc)[:120];self.mission.cancel(now,'recording_failed')
    def _finish_video(self,reason):
        if self.video and self.video.capturing:
            self.video.finish(reason,self._report())
        self._release_video_camera()
    def _release_video_camera(self):
        if self.video_camera_lease and not self.video_required:
            self.video_camera_lease=False
            if not getattr(self.rt.ai,'vision',False) and hasattr(self.rt.ai,'_task'):self.rt.ai._task(self.rt.ai._set_vision(False))
    def _video_tick(self,now):
        if not self.video:return
        if self.video.error and self.mission.state in ACTIVE|{'paused','manual_paused','fault'}:
            self.mission.cancel(now,'recording_failed');self.notice='录像异常，任务已取消，请检查存储'
            self._save_report()
        if self.video_required:
            if self.usb and self.usb.status()['verified']:self.video_usb_lost_since=None
            elif self.video_usb_lost_since is None:self.video_usb_lost_since=now
            timeout=self.cfg.get('recording',{}).get('source_timeout_s',5)
            last_camera=self.last_camera if self.last_camera is not None else self.video.started
            last_map=self.last_map if self.last_map is not None else self.video.started
            if self.mission.state in ACTIVE|{'paused','manual_paused','fault'} and (
                    now-last_camera>timeout or now-last_map>timeout or
                    (self.video_usb_lost_since is not None and now-self.video_usb_lost_since>timeout)):
                self.mission.cancel(now,'recording_source_lost')
            report=self._report() if now>=self.video_next_report else None
            self.video.offer(self._video_snapshot(now),report);self.video_next_report=now+1 if report else self.video_next_report
            if self.mission.state in ('completed','cancelled','invalidated'):
                self._save_report();self._finish_video(self.mission.reason or self.mission.state)
        self._release_video_camera()
    def send(self,kind,words,*,priority=False):
        port=self.rt.gateway.transport
        if port.fd is None:return False
        self.sequence=(self.sequence+1)&255
        port.enqueue(wire.encode(kind,words,self.sequence),priority=priority,tag='inspection')
        return True
    def _stop_recording(self):
        self.record_armed=False
        if self.released:self.released.set()
        if self.record_job and not self.record_job.done():self.record_job.cancel()
    def link_lost(self,reason,now):
        if not self.enabled:return
        self.discovery_speech.cancel(reason)
        self.generation+=1;self._stop_recording();self.want_close=None
        if self.mission.state in ACTIVE or self.mission.state in ('paused','manual_paused','fault'):
            self.mission.pause(now,reason);self.mission.state='invalidated';self.mission.reason=reason
            if self.epoch is not None:self._save_report()
        elif self.mission.state in ('edit','ready'):
            self.mission.state='edit';self.mission.goal=None
        self._finish_video(reason)
        # A reboot can reuse the FPGA's numeric epoch. Establish a fresh host
        # session and discard geometry, rather than assume localization survived.
        self.mission.actions.clear();self.mission.expected=None;self.mission.pose=None
        self.mission.session_id=(uuid.uuid4().int&0xffffffff) or 1
        self.active=False;self.recognition_display=False;self.caps=None;self.actual=None;self.input=None;self.clock=None;self.epoch=None
        self.radar=None;self.camera=None;self.detections=[];self.objects=Objects();self.layer=None;self.layer_crc=None
        self.upload=None;self.upload_info=None;self.upload_pending=None;self.last_input=self.last_state=None
        self.ai_sequence=None;self.next_hello=now;self.rt.gateway.transport.drop('inspection')
        if self.usb:self.usb.drop_controls()
        self.notice='连接重建：旧定位任务失效，报告已保留'
    def on_ai_controls(self,event,now):
        if not self.enabled:return
        seq=event['sequence']
        if self.ai_sequence is not None and not 0<((seq-self.ai_sequence)&255)<128:return
        self.ai_sequence=seq;self.latest_ai=event
        mode=event.get('mode')=='visual_inspection'
        recognition=bool(event.get('mode')=='object_recognition' and self.rt.ai.active and
                         getattr(self.rt.ai,'vision',False) and event.get('enabled') and
                         not event.get('manual') and not event.get('estop'))
        if recognition!=self.recognition_display:self.generation+=1
        self.recognition_display=recognition
        usable=bool(mode and self.input and self.input['open'] and self.input['permission'] and self.rt.ai.active and event.get('enabled') and not event.get('manual') and not event.get('estop'))
        was=self.active;self.active=usable
        if self.video_required and (not mode or not event.get('enabled') or event.get('estop')):
            self.mission.cancel(now,'ai_module_exit');self._save_report();self._finish_video('ai_module_exit')
        if event.get('manual'):
            self.want_close=None;self.last_camera=None;self.camera=None;self.mission.set_manual(True,now);self._stop_recording();self.notice='手动接管，保留任务；回光标后确认继续'
        elif self.mission.manual:self.mission.set_manual(False,now)
        if was and not usable:
            self.generation+=1;self._stop_recording();self.upload=None;self.upload_info=None;self.upload_pending=None
            self.mission.pause(now,'ai_mode_or_permission_changed')
        # Leaving the fourth entry pauses its task; it cannot own any of the
        # original three modes. A fresh local confirmation is required on return.
        if not usable:
            self.discovery_speech.cancel('inspection_not_active');return
        if not event.get('record'):
            if self.released:self.released.set()
            self.record_armed=True
        elif self.record_armed:
            self.record_armed=False
            if not self.stopped:
                self.pause(now,'recording_requires_stop');self.notice='先停稳，再上拨并下拨开始录音'
            elif not self.record_job or self.record_job.done():
                self.released=asyncio.Event();generation=self.generation
                self.record_job=asyncio.create_task(self._record(generation,self.released))
    def on_packet(self,message,now):
        if not self.enabled:return
        kind,words=message['kind'],message['words']
        try:
            if kind==wire.CAPS:
                if len(words)!=6 or words[0]!=self.mission.session_id or words[1]!=1:return
                self.caps=dict(epoch=words[2],layer=words[3],features=words[4],flags=words[5])
                if self.epoch!=words[2]:self._epoch(words[2],now)
            elif kind==wire.INPUT:
                event=wire.inputs(words)
                if event['session_id']!=self.mission.session_id:return
                if event['epoch']!=self.epoch:self._epoch(event['epoch'],now)
                old=self.input
                if event['manual']:
                    self.want_close=None;self.last_camera=None;self.camera=None;self.mission.set_manual(True,now)
                elif self.mission.manual:self.mission.set_manual(False,now)
                if old and (event['sequence']-old['sequence'])&0xffffffff>=0x80000000:return
                if self.active and (not event['open'] or not event['permission'] or event['manual'] or not event['rc_valid']):
                    self.pause(now,'inspection_page_closed');self.active=False;self.generation+=1;self._stop_recording()
                if self.video_required and (not event['open'] or not event['permission'] or not event['rc_valid']):
                    self.mission.cancel(now,'inspection_page_exit');self._save_report();self._finish_video('inspection_page_exit')
                self.input=event;self.context=event['sequence'];self.last_input=now
                self.mission.set_view(event['view'])
                if self.usb:self.usb.bind(self.mission.session_id,self.actual['task_id'] if self.actual else 0,self.epoch,self.context)
                if self.active and not event['manual'] and event['rc_valid'] and old:
                    confirms=(event['confirms']-old['confirms'])&65535;backs=(event['backs']-old['backs'])&65535
                    if confirms==1:self.confirm(now)
                    elif 1<confirms<32768:self.pause(now,'input_gesture_gap')
                    if backs==1:self.back(now)
                    elif 1<backs<32768:self.pause(now,'input_gesture_gap')
            elif kind==wire.STATE:
                state=wire.status(words)
                if state['session_id']!=self.mission.session_id or state['epoch']!=self.epoch:return
                self.actual=state;self.last_state=now;self.stopped=state['stopped']
                self.actual['flags']=words[6]
                if state['kind']=='fault':
                    self.upload=None;self.upload_info=None;self.upload_pending=None;self.next_upload_retry=now+1.
                if self.usb:self.usb.bind(self.mission.session_id,state['task_id'],self.epoch,self.context)
                if self.upload_info and state['kind']=='layer_committed' and state['layer_version']==self.upload_info['version']:
                    self.layer_crc=self.upload_info['crc'];self.upload_info=None;self.upload=None
                    self.mission.layer_version=state['layer_version']
                receipt={**state,'kind':'running'} if state['kind']=='accepted' and self.mission.state=='no_path' else state
                self.mission.on_status(receipt,now)
                if self.mission.state=='completed' and self.report_path is None:self._save_report()
                if self.mission.state=='completed':self._finish_video('completed')
                if self.mission.state in ('edit','ready','paused','manual_paused','fault'):self.mission.layer_version=state['layer_version']
            elif kind==wire.POSE:
                if self.poses and self.poses.add(words):
                    pose=self.poses.items[-1][2]
                    self.mission.set_pose(pose,now)
                    if not getattr(self,'cursor_initialized',False) and self.mission.state=='edit':
                        self.cursor=[pose['x_m'],pose['y_m']];self.cursor_initialized=True
            elif kind==wire.CLOCK_REPLY:
                if self.clock:self.clock.reply(words,now)
        except (ValueError,KeyError,TypeError) as exc:
            self.error=str(exc)[:160];self.pause(now,'invalid_device_telemetry')
    def _epoch(self,epoch,now):
        if type(epoch) is not int or not 0<epoch<=0xffffffff:raise ValueError('Invalid localization epoch')
        self.discovery_speech.cancel('localization_reset')
        if self.epoch is not None:
            self.mission.cancel(now,'localization_reset');self.mission.state='invalidated'
            self._save_report()  # Save old coordinates before replacing their frame.
            self._finish_video('localization_reset')
        self.mission.actions.clear();self.mission.expected=None;self.mission.pose=None;self.mission.epoch=epoch
        self.actual=None;self.last_state=None;self.want_close=None;self.cursor_initialized=False
        self._stop_recording()
        self.epoch=epoch;self.clock=ClockSync(self.mission.session_id,epoch);self.poses=PoseBuffer(epoch)
        self.objects=Objects();self.upload=None;self.upload_info=None;self.upload_pending=None;self.layer=None;self.layer_crc=None
        self.radar=None;self.camera=None;self.detections=[];self.planned_route=[];self.route_history=[];self.route_key=None;self.generation+=1
    def readiness(self,now):
        serial=self.cfg.get('camera',{}).get('selector',{}).get('serial') or self.rt.cfg.get('camera',{}).get('selector',{}).get('serial')
        size=self.cfg.get('camera',{}).get('expected_size',self.rt.cfg.get('camera',{}).get('expected_size',[]))
        classes=self.cfg.get('classes',['tape','screwdriver','ad_calcium_milk','nutri_express'])
        policies=self.cfg.get('ground_anchor_policies',{});footprints=self.cfg.get('object_footprints_m',{})
        anchors=bool(classes) and all(policies.get(name,{}).get('measured') is True and
            policies[name].get('anchor') in ('bottom_center','bbox_fraction') and finite(footprints.get(name)) and 0<footprints[name]<=1 for name in classes)
        geometry=not self.calibration.problems(size,serial) and anchors
        timing=False
        try:
            if self.clock and self.camera:
                _,ce=self.clock.estimate(now);_,fe=capture_time(self.camera['metadata'],self.cfg.get('capture_timing',{}),serial)
                timing=ce+fe<=.02
        except ValueError:pass
        vehicle=finite(self.cfg.get('vehicle_swept_radius_m')) and self.cfg['vehicle_swept_radius_m']>0 and self.cfg.get('vehicle_braking_verified') is True
        return dict(calibration=geometry,timing=timing,camera=self.last_camera is not None and 0<=now-self.last_camera<=.5,
            fpga=bool(not self.evidence_error and self.usb and self.usb.status()['verified'] and self.caps and self.actual and self.caps['features']&15==15 and self.last_state is not None and now-self.last_state<=.5),
            map=self.radar is not None and self.last_map is not None and now-self.last_map<=1.,
            layer=bool(self.actual and self.actual['flags']&16 and self.layer_crc is not None and self.layer and self.layer_crc==self.layer['crc32']),
            speed_mapping=self.cfg.get('speed_mapping_verified') is True,vehicle=bool(vehicle))
    def confirm(self,now):
        if not self.active or not self.input or self.input['manual']:return False
        state=self.mission.state
        if state=='edit':
            if self.input['view']!=1:self.notice='请用CH8切到地图后选点';return False
            if self.radar is None:self.notice='等待雷达地图';return False
            if bit(self.radar,*cell(self.cursor)):self.notice='目标位于障碍格，请换点';return False
            self.mission.select_goal(self.cursor);self.notice='再次确认开始巡检';return True
        if state=='ready':
            if self.evidence_error:self.notice='数据记录未就绪，请检查存储';return False
            if self.video_recovery and not self.video_recovery.done():self.notice='正在检查上次中断录像，请稍候';return False
            if self.video and not self.video.done.is_set():self.notice='上一段录像正在收尾';return False
            accepted=self.mission.start(self.readiness(now),now)
            if accepted:self._begin_video(now)
            self.notice='正在准备双路录像，随后开始任务' if accepted else '等待标定或设备条件';return accepted
        if state in ('paused','manual_paused','fault'):
            accepted=self.mission.resume(self.readiness(now),now,layer=self.layer);self.notice='请求继续' if accepted else '尚不满足恢复条件';return accepted
        if state in ACTIVE:self.pause(now,'operator_pause');return True
        if state=='completed':self.notice='巡检已完成，上拨返回可新建任务';return False
        if state in ('cancelled','invalidated') and self.stopped:
            self.mission.new_task();self.cursor=[self.mission.pose['x_m'],self.mission.pose['y_m']] if self.mission.pose else [0,0];self.report_path=None;self.notice='新任务：选点后确认';return True
        return False
    def pause(self,now,reason='operator_pause'):
        self.discovery_speech.cancel(reason)
        if not (self.mission.state in ('paused','manual_paused') and self.mission.reason==reason):
            self.mission.pause(now,reason)
        self.upload=None;self.upload_info=None;self.upload_pending=None
        if self.usb:self.usb.drop_controls()
    def back(self,now):
        if not self.active:return
        state=self.mission.state
        if state in ACTIVE:self.pause(now,'operator_back');return
        if state in ('paused','manual_paused','fault'):
            self.mission.cancel(now,'operator_cancel');self.want_close=dict(self.mission.expected);self._save_report();self.notice='正在取消，停稳后返回AI功能页';return
        if state=='ready':self.mission.state='edit';self.mission.goal=None;self.notice='继续编辑目标';return
        if not self.stopped:self.notice='等待停车回执';return
        if state in ('completed','cancelled','invalidated'):
            self.mission.new_task();self.cursor=[self.mission.pose['x_m'],self.mission.pose['y_m']] if self.mission.pose else [0,0];self.report_path=None;return
        if self.input:self.send(wire.UI_CLOSE,[self.mission.session_id,self.epoch,self.context,1])
    async def observe(self,frame,result,now):
        if not self.enabled:return
        generation=self.generation
        import cv2
        image=await asyncio.to_thread(cv2.imread,frame['image'])
        if generation!=self.generation:return
        if image is None or image.shape[:2]!=(frame['height'],frame['width']):raise ValueError('Camera JPEG geometry changed')
        self.camera=dict(bgr=image,sequence=frame['sequence'],metadata=frame);self.last_camera=now
        self.detections=[{**d,'frame_sequence':frame['sequence']} for d in result.get('detections',[])[:64]]
        if not self.active or self.clock is None or self.poses is None:return
        serial=self.cfg.get('camera',{}).get('selector',{}).get('serial') or self.rt.cfg.get('camera',{}).get('selector',{}).get('serial')
        try:
            captured,frame_error=capture_time(frame,self.cfg.get('capture_timing',{}),serial)
            pose,pose_error=self.poses.at(captured,self.clock,now)
            if frame_error+pose_error>.02:raise ValueError('Total image/pose time uncertainty exceeds20ms')
            located=[]
            for d in self.detections:
                policy=self.cfg.get('ground_anchor_policies',{}).get(d['class'],{})
                radius=self.cfg.get('object_footprints_m',{}).get(d['class'])
                if policy.get('measured') is not True or not finite(radius) or not 0<radius<=1:continue
                box=d['bbox'];mode=policy.get('anchor')
                if mode=='bottom_center':u,v=(box[0]+box[2])/2,box[3]-1
                elif mode=='bbox_fraction':
                    fractions=policy.get('uv')
                    if not isinstance(fractions,list) or len(fractions)!=2 or not all(finite(x) and 0<=x<=1 for x in fractions):continue
                    u,v=box[0]+(box[2]-box[0])*fractions[0],box[1]+(box[3]-box[1])*fractions[1]
                else:continue
                try:
                    location=self.calibration.locate(dict(u=u,v=v,ground_verified=True),[frame['width'],frame['height']],serial,pose,
                        epoch=self.epoch,captured_at=captured,pose_at=captured,now=now,source_timestamp_verified=True,camera_metadata=frame)
                except ValueError as exc:
                    # A point outside the measured H region must not discard
                    # other valid objects from this same frame.
                    self.error='Target projection skipped: '+str(exc)[:130]
                    continue
                located.append({**location,'class':d['class'],'radius_m':radius})
            confirmed=self.objects.observe(located,frame['sequence'],self.epoch,now)
            # Use confirmed object identities, not detections/classes per frame.
            self.discovery_speech.notify((self.mission.session_id,self.mission.task_id,self.epoch),
                [obj for obj in self.objects.snapshot() if obj['confirmed'] and obj['sequence']==frame['sequence']])
            if confirmed:
                data=await asyncio.to_thread(Path(frame['image']).read_bytes)
                if generation!=self.generation:return
                self._record_evidence('keyframe',data,dict(frame=frame,confirmed_ids=confirmed,detections=self.detections,
                    projections=located,pose=pose,device_poses=list(self.poses.items)[-8:],inference_completed_at_s=now,
                    clock_estimate=self.clock.estimate(now),worker_elapsed_s=result.get('worker_elapsed_s'),calibration=self.calibration.profile))
            if located:self.layer_dirty=True
        except ValueError as exc:self.error=str(exc)[:160]
    async def _make_layer(self,generation):
        try:
            # Tentative located objects also block navigation conservatively;
            # three-frame confirmation only controls report/red-dot publication.
            objects=[{**x,'confirmed':True} for x in self.objects.snapshot()]
            layer=await asyncio.to_thread(build,objects,self.cfg['vehicle_swept_radius_m'])
            if generation==self.generation:self.layer=layer
        except Exception as exc:self.error=str(exc)[:160]
    def _record_evidence(self,kind,payload,metadata):
        if len(self.evidence_jobs)>=4:self.evidence_error='Evidence queue full';return False
        metadata={'session':self.mission.session_id,'task':self.mission.task_id,'epoch':self.epoch,**metadata}
        async def write():
            try:
                self.evidence_files.append(await asyncio.to_thread(self.evidence.write,kind,payload,metadata))
                if self.report_path and (metadata['session'],metadata['task'],metadata['epoch']) == (self.mission.session_id,self.mission.task_id,self.epoch):
                    self._save_report()
            except Exception as exc:self.evidence_error=str(exc)[:160];self.error=self.evidence_error
        job=asyncio.create_task(write());self.evidence_jobs.add(job);job.add_done_callback(self.evidence_jobs.discard)
        return True
    def _begin_upload(self,layer,version,task,command):
        if not self._record_evidence('layer',layer['raw']+layer['cost'],dict(session=self.mission.session_id,task=task,epoch=self.epoch,
                command=command,version=version,crc32=layer['crc32'],user_token=self.input['user_token'] if self.input else None)):
            return
        self.upload_pending=None
        self.upload=iter(wire.layer_messages(self.mission.session_id,self.epoch,version,layer,task_id=task,command_id=command))
        self.upload_info=dict(version=version,crc=layer['crc32'],task=task,command=command)
    def _upload_tick(self):
        if not self.upload or not self.usb or not self.usb.status()['verified']:return
        # Keep one not-yet-admitted transaction; no advancing a generator on a full queue.
        if not hasattr(self,'upload_pending'):self.upload_pending=None
        for _ in range(6):
            if self.upload_pending is None:
                try:self.upload_pending=next(self.upload)
                except StopIteration:self.upload=None;return
            kind,words=self.upload_pending
            frame=Container(CONTROL,self.mission.session_id,self.actual['task_id'],self.epoch,self.context,
                self.sequence,view=self.input['view'],payload=wire.encode(kind,words,self.sequence))
            if not self.usb.offer(frame):return
            self.sequence=(self.sequence+1)&255;self.upload_pending=None
    def _actions(self,now):
        for action in self.mission.drain():
            kind=action['kind']
            if kind in ('goto','pause','cancel'):
                if not self.active and kind=='goto':continue
                if kind=='goto':
                    if self.video is None:self._begin_video(now)
                    if self.video is None:continue
                    if not self.video.ready:
                        if self.video.error:
                            self.mission.cancel(now,'recording_failed');self._save_report();self._finish_video('recording_failed')
                        else:self.mission.actions.append(action)
                        continue
                commissioned=all(self.readiness(now).values()) if kind=='goto' else self.cfg.get('speed_mapping_verified') is True
                if kind=='goto':commissioned=commissioned and self.mission.pose is not None and 0<=now-self.mission.pose['at_s']<=.5
                if kind=='goto' and not commissioned:
                    self.mission.pause(now,'conditions_changed_before_goto');continue
                self.send(wire.CONTROL,wire.control(action,user_token=self.input['user_token'] if self.input else 0,commissioned=commissioned),priority=kind!='goto')
                if kind=='goto':self.mission.last_status=now
            elif kind=='commit_layer':
                self._begin_upload(self.mission.pending_layer,action['layer_version'],action['task_id'],action['command_id'])
            elif kind=='prepare_return_layer':
                if self.layer:self.mission.submit_layer(self.layer,now)
                else:self.pause(now,'return_layer_unavailable')
    async def _render(self,generation,binding,recognition=False):
        try:
            from .render import render,render_recognition,rgb565
            mission={**self.mission.snapshot(),'recording_phase':self.recording_phase,'instruction':self.instruction,'planned_route':self.planned_route};objects=self.objects.snapshot();camera=self.camera if self.last_camera is not None and time.monotonic()-self.last_camera<=.5 else None;detections=list(self.detections)
            if recognition:
                target=getattr(self.rt.ai,'target',None)
                if target:detections=[d for d in detections if d['class']==target]
                phase=getattr(self.rt.ai,'state',None)
                image=await asyncio.to_thread(render_recognition,camera,detections,
                    instruction=getattr(self.rt.ai,'instruction',''),recording_phase=phase)
            else:
                image=await asyncio.to_thread(render,self.mission.view,mission,objects,camera=camera,detections=detections,
                    radar_raw=self.radar,cursor=self.cursor,scroll=self.scroll,calibration_ready=all(self.readiness(time.monotonic()).values()))
            payload=await asyncio.to_thread(rgb565,image)
            if generation==self.generation and self.usb and self.actual and recognition==self.recognition_display and binding==(self.mission.session_id,self.actual['task_id'],self.epoch,self.context):
                self.usb.offer(Container(DISPLAY,*binding,sequence=time.monotonic_ns()&0xffffffff,width=816,height=420,stride=1632,
                    view=0 if recognition else self.input['view'],payload=payload))
        except Exception as exc:self.error=str(exc)[:160]
    def tick(self,now):
        if not self.enabled or self.closed:return
        self._video_tick(now)
        if self.mission.state in ('completed','cancelled','invalidated','fault'):
            self.discovery_speech.cancel('inspection_terminal')
        if self.video_required and (self.last_state is None or now-self.last_state>.5):
            self.link_lost('inspection_telemetry_timeout',now)
        if self.rt.gateway.transport.fd is None:return
        if self.last_state is not None and now-self.last_state>.5:
            self.link_lost('inspection_telemetry_timeout',now)
        if (self.caps is None or self.actual is None) and now>=self.next_hello:
            self.send(wire.HELLO,[self.mission.session_id],priority=True);self.next_hello=now+1.
        if self.actual and now>=self.next_heartbeat:
            self.send(wire.HEARTBEAT,[self.mission.session_id,self.actual['task_id'],self.epoch,self.actual['command_id']]);self.next_heartbeat=now+.1
        if self.clock and self.caps and self.caps['features']&8 and now>=self.next_clock:
            request=self.clock.request(now)
            if request:self.send(wire.CLOCK_REQUEST,request)
            self.next_clock=now+.5
        if self.input and self.last_input is not None and now-self.last_input>.5:
            self.pause(now,'physical_input_timeout');self.active=False;self._stop_recording()
        verified=bool(self.usb and self.usb.status()['verified'])
        if self.usb_was_verified and not verified:
            self.pause(now,'highspeed_link_lost');self.layer_crc=None
        self.usb_was_verified=verified
        if self.evidence_error:self.pause(now,'evidence_recording_failed');self.notice='数据记录失败，任务暂停，请检查存储'
        preparing=self.video_required and not self.video.ready and now-self.video.started<3
        if not preparing:self.mission.tick(now)
        if self.active and self.mission.state in ACTIVE:
            ready=self.readiness(now)
            missing=[key for key in ('calibration','timing','camera','fpga','speed_mapping','vehicle') if not ready[key]]
            if missing:self.pause(now,'runtime_conditions_lost:'+','.join(missing))
        if self.usb:
            for frame in getattr(self.usb,'drain_routes',lambda:[])():
                try:
                    points=route_points(frame);key=(frame.sequence>>16,frame.payload)
                    self.planned_route=points;self.last_route=now
                    if key!=self.route_key:
                        self.route_key=key;self.route_history.append(dict(sequence=frame.sequence,phase=self.mission.phase,at_s=now,points=points))
                        self.route_history=self.route_history[-512:]
                except ValueError as exc:self.error=str(exc)
            if self.last_route is not None and now-self.last_route>1:self.planned_route=[]
            for frame in self.usb.drain_maps():
                if self.input and frame.view==self.input['view']:
                    self.radar=frame.payload;self.last_map=now
        if not self.active:
            if (self.recognition_display and self.usb and self.usb.status()['verified'] and self.input and self.input['view']==0
                    and self.actual and now>=self.next_render and (not self.render_job or self.render_job.done())):
                binding=(self.mission.session_id,self.actual['task_id'],self.epoch,self.context)
                self.render_job=asyncio.create_task(self._render(self.generation,binding,recognition=True));self.next_render=now+self.render_interval
            self._actions(now);return
        if self.input and now>=self.next_stick:
            if self.mission.state=='edit' and self.input['view']==1:
                steer,throttle=self.input['steer'],self.input['throttle']
                if abs(steer)>250 or abs(throttle)>250:
                    self.cursor=[max(-6.375,min(6.375,self.cursor[0]+(-.05 if steer>250 else .05 if steer<-250 else 0))),
                                 max(-6.375,min(6.375,self.cursor[1]+(.05 if throttle>250 else -.05 if throttle<-250 else 0)))]
                    self.cursor_initialized=True;self.mission.state='edit'
            elif self.input['view']==2:
                if self.input['throttle']>250:self.scroll=max(0,self.scroll-1)
                elif self.input['throttle']<-250:self.scroll=min(116,self.scroll+1)
            self.next_stick=now+.12
        radius=self.cfg.get('vehicle_swept_radius_m')
        if (self.layer is None or self.layer_dirty) and finite(radius) and radius>0 and (not self.layer_job or self.layer_job.done()):
            self.layer_dirty=False;self.layer_job=asyncio.create_task(self._make_layer(self.generation))
        if self.layer and not self.upload_info and self.actual and self.input:
            changed=self.layer_crc!=self.layer['crc32']
            if changed and self.mission.state in ('outbound','returning','no_path'):
                self.mission.submit_layer(self.layer,now)
            elif changed and now>=self.next_upload_retry and self.mission.state in ('edit','ready') and self.stopped and self.actual['flags']&8:
                self._begin_upload(self.layer,self.actual['layer_version']+1,self.actual['task_id'],self.actual['command_id'])
        self._actions(now);self._upload_tick()
        if self.want_close and self.actual and self.stopped and now>=self.next_close:
            if all(self.actual.get(k)==self.want_close[k] for k in ('session_id','task_id','epoch','command_id')):
                self._save_report();self.send(wire.UI_CLOSE,[self.mission.session_id,self.epoch,self.context,1]);self.next_close=now+.5
        if self.usb and self.usb.status()['verified'] and self.input and self.actual and now>=self.next_render and (not self.render_job or self.render_job.done()):
            binding=(self.mission.session_id,self.actual['task_id'],self.epoch,self.context)
            self.render_job=asyncio.create_task(self._render(self.generation,binding));self.next_render=now+self.render_interval
    async def _record(self,generation,released):
        from ..switched_capture import capture
        path=self.rt.run_dir/'inspection_audio'/(uuid.uuid4().hex+'.wav');path.parent.mkdir(parents=True,exist_ok=True)
        capturing=False
        try:
            self.discovery_speech.cancel('inspection_recording')
            self.rt.announcements._clear('inspection_recording')
            for _ in range(20):
                if not getattr(self.rt.announcements,'microphone_blocked',False):break
                await asyncio.sleep(.05)
            self.rt.announcements.begin_capture();capturing=True
            await self.rt.cue.play('start');self.notice='录音中，上拨结束';self.recording_phase='recording'
            from ..device_roles import audio_device
            device=audio_device(self.rt.cfg.get('audio_roles',{}).get('microphone'),'microphone',self.rt.cfg.get('microphone',{}).get('device'))
            await capture(device,path,released,maximum_s=30)
            self.rt.announcements.end_capture();capturing=False;await self.rt.cue.play('stop')
            self.recording_phase='recognizing'
            response=await self.rt.workers['asr'].call({'wav':str(path)},timeout=self.rt.cfg.get('asr_timeout_s',30))
            if generation!=self.generation or not self.active:return
            text=response['result'].get('text','').strip().rstrip('。！!');self.instruction=text
            if text in ('暂停','暂停巡检','停止'):self.pause(time.monotonic(),'voice_pause')
            elif text in ('继续','继续巡检','开始巡检'):self.notice='请用SWA确认开始或继续，语音不会代替物理确认'
            elif text in ('取消','取消巡检'):self.mission.cancel(time.monotonic(),'voice_cancel')
            else:self.notice='可说：暂停巡检、继续巡检、取消巡检；开始仍需手柄确认'
        except asyncio.CancelledError:raise
        except Exception as exc:self.error=str(exc)[:160];self.notice='录音暂不可用，仍可用手柄'
        finally:
            self.recording_phase=None
            if capturing:self.rt.announcements.end_capture()
            path.unlink(missing_ok=True)
    def _save_report(self):
        report=self._report()
        if self.video:
            self.video.offer(self._video_snapshot(time.monotonic()),report)
            if (self.video.done.is_set() or self.video.error) and hasattr(self.video,'save_final_report'):
                video=self.video
                async def update_video_report():
                    try:await asyncio.to_thread(video.save_final_report,report)
                    except (OSError,RuntimeError) as exc:self.evidence_error='Video report write failed: '+str(exc)[:100]
                job=asyncio.create_task(update_video_report());self.evidence_jobs.add(job);job.add_done_callback(self.evidence_jobs.discard)
        directory=self.rt.run_dir/'inspection_reports'
        path=directory/('task-%d-epoch-%d.json'%(self.mission.task_id,self.epoch))
        try:
            directory.mkdir(parents=True,exist_ok=True)
            temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');temporary.replace(path)
            self.report_path=str(path)
        except OSError as exc:self.evidence_error='Report write failed: '+str(exc)[:120]
    def _report(self):
        profile=json.dumps(self.cfg.get('calibration',{}),sort_keys=True,ensure_ascii=False).encode()
        return dict(version=1,world_frame='FPGA navigation map; x forward at origin, y left; metres',
            mission=self.mission.snapshot(),objects=self.objects.snapshot(),calibration_sha256=hashlib.sha256(profile).hexdigest(),
            discovery_speech=self.discovery_speech.snapshot(),
            return_policy='latest_map',fpga_route_history=self.route_history,evidence=[entry for entry in self.evidence_files if entry.get('epoch')==self.epoch],device_state=self.actual,
            recording=self.video.status() if self.video else None,
            motion_completion_source='authenticated FPGA stopped-arrival receipt' if self.mission.state=='completed' else 'task not completed; inspect device_state')
    def snapshot(self):
        camera=self.cfg.get('camera',self.rt.cfg.get('camera',{}))
        serial=camera.get('selector',{}).get('serial')
        size=camera.get('expected_size',[])
        profile=self.calibration.profile
        return dict(enabled=self.enabled,active=self.active,mission=self.mission.snapshot(),readiness=self.readiness(time.monotonic()),
            notice=self.notice,error=self.error,report=self.report_path,usb=self.usb.status() if self.usb else None,
            actual=self.actual,objects=self.objects.snapshot(),recording=self.video.status() if self.video else None,
            recognition_display=self.recognition_display,discovery_speech=self.discovery_speech.snapshot(),
            archive=dict(root=str(self.video_root),url='http://127.0.0.1:%d'%self.archive_server.server_port if self.archive_server else None,error=self.playback_error),
            calibration_model=dict(projection_model=profile.get('projection_model','pinhole_ground'),
                static_geometry_config_ready=not self.calibration.geometry_problems(size,serial),
                pending=self.calibration.problems(size,serial),
                pixel_domain=profile.get('homography',{}).get('pixel_domain'),focus=profile.get('focus'),
                max_range_m=profile.get('max_range_m'),ground_hull_cm=profile.get('homography',{}).get('ground_hull_cm')),
            serial_owner='coordinator',capture_timing_commissioned=self.readiness(time.monotonic())['timing'])
    def lines(self):
        from .render import STATES
        labels=dict(calibration='相机和地面标定',timing='拍摄及位姿对时',camera='相机',fpga='接口和回执',map='雷达地图',
                    layer='视觉障碍层',speed_mapping='速度标定',vehicle='车体及制动标定')
        state=self.mission.snapshot();ready=self.readiness(time.monotonic());missing=[k for k,v in ready.items() if not v]
        return ['视觉巡检：摄像头 / 雷达地图 / 文字','状态：'+STATES.get(state['state'],state['state']),self.notice,
            'CH8切换视图，摇杆选点，SWA确认','返回：暂停 → 取消 → 新建/返回',
            '目标：'+str(state['goal_m']),'起点：'+str(state['start_m']),
            '已定位物体：'+str(sum(x['confirmed'] for x in self.objects.items)),
            '指令：'+self.instruction,'待就绪：'+'、'.join(labels.get(k,k) for k in missing),'请先完成标定后启用自动巡检' if missing else '设备已就绪']
    async def close(self):
        await self.discovery_speech.close()
        if self.enabled and self.actual:
            try:
                self.pause(time.monotonic(),'service_shutdown');self._actions(time.monotonic());self.rt.gateway.transport.flush()
            except (OSError,RuntimeError):pass
        if self.video_required:
            self.mission.cancel(time.monotonic(),'service_shutdown');self._finish_video('service_shutdown')
        if self.video:
            try:await asyncio.to_thread(self.video.wait)
            except RuntimeError as exc:self.evidence_error=str(exc)
        if self.video_recovery:await self.video_recovery
        if self.archive_server:
            await asyncio.to_thread(self.archive_server.shutdown);self.archive_server.server_close();self.archive_thread.join(timeout=1)
        self.closed=True;self._stop_recording()
        jobs=[j for j in (self.record_job,self.render_job,self.layer_job) if j]
        for job in jobs:job.cancel()
        if jobs:await asyncio.gather(*jobs,return_exceptions=True)
        if self.evidence_jobs:await asyncio.gather(*list(self.evidence_jobs),return_exceptions=True)
        if self.usb:await asyncio.to_thread(self.usb.close)
