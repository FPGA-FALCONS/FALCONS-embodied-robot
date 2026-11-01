"""One RK inspection task controller using the coordinator's serial owner.

Models/capture stay in existing owned workers; USB only carries display/layers.
Demo localization: STOP receipt, .3s wait, three frames, fixed stopped pose.
"""
import asyncio
from collections import deque
from copy import deepcopy
import math
import json
import hashlib
from pathlib import Path
import time
import uuid
from . import wire
from .mission import InspectionMission,ACTIVE,CAMERA_DELAY_SECONDS,SETTLE_SECONDS,POSE_HISTORY_SECONDS
from ..ground_target import base_to_map
from .evidence import Evidence
from .objects import Objects,GROUND_ANCHORS,ground_anchor
from .calibration import Calibration,finite
from .grid import build,cell,bit
from .stream import Container,DISPLAY,CONTROL,route_points
from .usb_pump import USBPump
from .recording import Recording,recover
from .discovery_speech import DiscoverySpeech
from ..schema import DETECTION_CONFIDENCE_THRESHOLD


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
        self.mission=InspectionMission(cruise_speed_mm_s=self.cfg.get('cruise_speed_mm_s',300))
        self.dedup_distance_m=self.cfg.get('dedup_distance_m',.30)
        self.objects=Objects();self.calibration=Calibration(self.cfg.get('calibration',{}))
        self.observation_round=0;self.observation_pose=None
        self.observation_objects=None;self.observation_frames=0;self.observation_attempts=0
        self.usb=usb_factory(self.cfg['highspeed']) if self.enabled and self.cfg.get('highspeed',{}).get('enabled') is True else None
        self.input=None;self.actual=None;self.caps=None
        self.epoch=None;self.sequence=0;self.context=0;self.latest_ai=None
        self.last_input=self.last_state=self.last_map=self.last_camera=None
        self.next_hello=self.next_heartbeat=self.next_render=self.next_stick=0.
        self.cursor=[0.,0.];self.scroll=0;self.camera=None;self.detections=[];self.radar=None;self.planned_route=[];self.route_history=[];self.route_key=None;self.last_route=None
        self.edit_action=None;self.edit_stick_ready=False
        self.notice='地图移动准心，SWA打开路径点菜单';self.error=None;self.instruction='';self.recording_phase=None;self.report_path=None;self.next_upload_retry=0.
        self.layer=None;self.layer_crc=None;self.layer_dirty=False;self.upload=None;self.upload_info=None;self.upload_pending=None
        self.render_job=None;self.record_job=None;self.released=None;self.record_armed=False
        self.usb_was_verified=False
        self.evidence=Evidence(rt.run_dir/'inspection_evidence');self.evidence_jobs=set();self.storage_check=None;self.evidence_error=None;self.evidence_files=[]
        self.exit_requested=False
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
    def _display_mission(self):
        return {**self.mission.snapshot(),'instruction':self.instruction,'recording_phase':self.recording_phase,
                'planned_route':list(self.planned_route),'edit_action':self.edit_action,'notice':self.notice}
    def _video_snapshot(self,now):
        return dict(mission=self._display_mission(),objects=self.objects.snapshot(),
                    camera=self.camera,
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
    def fpga_reset(self,now):
        self.link_lost('fpga_reset',now)
        # link_lost archives the old report/video before replacing coordinates.
        self.mission=InspectionMission(session_id=self.mission.session_id,cruise_speed_mm_s=self.mission.cruise_speed_mm_s)
        self.latest_ai=None;self.context=0;self.stopped=False
        self.cursor=[0.,0.];self.cursor_initialized=False;self.scroll=0;self.edit_stick_ready=False
        self.planned_route=[];self.route_history=[];self.route_key=None;self.last_route=None
        self.report_path=None;self.instruction='';self.error=None;self.recording_phase=None
        self.observation_objects=None;self.observation_frames=0;self.observation_attempts=0
        self.layer_dirty=False;self.next_upload_retry=0.
        self.last_map=self.last_camera=None
        if self.render_job and not self.render_job.done():self.render_job.cancel()
        if self.usb:self.usb.bind(self.mission.session_id,0,0,0)
        self.notice='已复位，请重新开启AI并选择任务'
    def link_lost(self,reason,now):
        if not self.enabled:return
        self.discovery_speech.cancel(reason)
        self.generation+=1;self._stop_recording();self.exit_requested=False
        if self.mission.state in ACTIVE or self.mission.state in ('paused','manual_paused','fault'):
            self.mission.pause(now,reason);self.mission.state='invalidated';self.mission.reason=reason
            if self.epoch is not None:self._save_report()
        elif self.mission.state in ('edit','ready'):
            self.mission.state='edit';self.mission.goal=None;self.mission.clear_waypoints()
        self.edit_action=None
        self._finish_video(reason)
        # A reboot can reuse the FPGA's numeric epoch. Establish a fresh host
        # session and discard geometry, rather than assume localization survived.
        self.mission.actions.clear();self.mission.expected=None;self.mission.pose=None;self.mission.pose_history.clear()
        self.mission.session_id=(uuid.uuid4().int&0xffffffff) or 1
        self.active=False;self.recognition_display=False;self.caps=None;self.actual=None;self.input=None;self.epoch=None
        self.radar=None;self.camera=None;self.detections=[];self.objects=Objects();self.layer=None;self.layer_crc=None
        self.observation_round+=1;self.observation_pose=None
        self.upload=None;self.upload_info=None;self.upload_pending=None;self.last_input=self.last_state=None
        self.latest_ai=None;self.next_hello=now;self.rt.gateway.transport.drop('inspection')
        if self.usb:self.usb.drop_controls()
        self.notice='连接重建：旧定位任务失效，报告已保留'
    def _manual(self,enabled,now):
        if enabled and not self.mission.manual:
            self.observation_objects=None;self.exit_requested=False
            self.discovery_speech.cancel('manual_takeover');self._stop_recording()
            self.notice='手动接管，保留任务；回光标后确认继续'
        self.mission.set_manual(enabled,now)

    def _check_storage(self):
        import os,shutil,tempfile
        from .recording import directory_bytes
        self.evidence.check_storage()
        self.video_root.mkdir(parents=True,exist_ok=True)
        cfg=self.cfg.get('recording',{})
        if (directory_bytes(self.video_root)>=cfg.get('max_archive_bytes',16*1024**3) or
                shutil.disk_usage(self.video_root).free<cfg.get('min_free_bytes',2*1024**3)):
            raise OSError('录像存储空间不足，请先导出或清理')
        with tempfile.TemporaryFile(dir=self.video_root) as f:
            f.write(b'falcons storage check');f.flush();os.fsync(f.fileno())

    async def _retry_storage(self):
        error=self.evidence_error
        try:
            await asyncio.to_thread(self._check_storage)
            if self.evidence_error==error:
                self.evidence_error=None;self.error=None
                self.notice='记录资源已恢复，请再次SWA确认'
        except (OSError,ValueError) as exc:
            self.evidence_error=str(exc)[:160];self.notice='存储仍不可用，请清理后再次SWA'

    def on_ai_controls(self,event,now):
        if not self.enabled:return
        previous=self.latest_ai;self.latest_ai=event
        mode=event.get('mode')=='visual_inspection'
        if mode and event.get('enabled') and (previous is None or not previous.get('enabled') or
                previous.get('mode')!='visual_inspection' or previous.get('confirm_toggle')!=event.get('confirm_toggle')):
            self._enter_page()
            if previous is not None:self.input=None  # First new-page snapshot establishes its button counters.
        recognition=bool(event.get('mode')=='object_recognition' and self.rt.ai.active and
                         getattr(self.rt.ai,'vision',False) and event.get('enabled') and
                         not event.get('manual') and not event.get('estop'))
        if recognition!=self.recognition_display:self.generation+=1
        self.recognition_display=recognition
        usable=bool(mode and self.input and self.input['open'] and self.input['permission'] and self.rt.ai.active and event.get('enabled') and not event.get('manual') and not event.get('estop'))
        was=self.active;self.active=usable
        if not usable:self.edit_action=None
        if self.video_required and (not mode or not event.get('enabled') or event.get('estop')):
            self.mission.cancel(now,'ai_module_exit');self._save_report();self._finish_video('ai_module_exit')
        if event.get('manual'):
            self._manual(True,now)
        elif self.mission.manual:self._manual(False,now)
        if was and not usable:
            self.generation+=1;self._stop_recording();self.upload=None;self.upload_info=None;self.upload_pending=None
            self.mission.pause(now,'ai_mode_or_permission_changed')
        # Leaving the fourth entry pauses its task; it cannot own any of the
        # original three modes. A fresh local confirmation is required on return.
        if not usable:
            self.discovery_speech.cancel('inspection_not_active');return
        if self.exit_requested:
            self._stop_recording();return
        if not event.get('record'):
            if self.released:self.released.set()
            self.record_armed=True
        elif self.record_armed:
            self.record_armed=False
            if self.mission.state in ACTIVE:self.pause(now,'recording_requires_stop')
            if not self.stopped:
                self.pause(now,'recording_requires_stop');self.notice='先停稳，再上拨并下拨开始录音'
            elif not self.record_job or self.record_job.done():
                self.released=asyncio.Event();generation=self.generation
                self.record_job=asyncio.create_task(self._record(generation,self.released))
    def _enter_page(self):
        self.exit_requested=False;self.edit_action=None;self._stop_recording()
        self.notice='地图移动准心，SWA打开路径点菜单'
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
                    self._manual(True,now)
                elif self.mission.manual:self._manual(False,now)
                if self.active and (not event['open'] or not event['permission'] or event['manual'] or not event['rc_valid']):
                    self.pause(now,'inspection_page_closed');self.active=False;self.generation+=1;self._stop_recording()
                if self.video_required and (not event['open'] or not event['permission'] or not event['rc_valid']):
                    self.mission.cancel(now,'inspection_page_exit');self._save_report();self._finish_video('inspection_page_exit')
                self.input=event;self.context=event['sequence'];self.last_input=now
                if event['open'] and (old is None or not old['open']):self._enter_page()
                if not event['open']:self.exit_requested=False
                if event['manual'] or (old and old['view']!=event['view']):self.edit_action=None
                self.mission.set_view(event['view'])
                if self.usb:self.usb.bind(self.mission.session_id,self.actual['task_id'] if self.actual else 0,self.epoch,self.context)
                if event['open'] and not event['manual'] and event['rc_valid'] and old and old['open']:
                    if event['confirms']!=old['confirms']:self.confirm(now)
                    if event['backs']!=old['backs']:self.back(now)
                self._close_page()
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
                self._close_page()
                if self.mission.state=='completed' and self.report_path is None:self._save_report()
                if self.mission.state=='completed':self._finish_video('completed')
                if self.mission.state in ('edit','ready','paused','manual_paused','fault'):self.mission.layer_version=state['layer_version']
            elif kind==wire.POSE:
                epoch,sequence,stamp,x,y,yaw,valid=words
                signed=lambda v:v-0x100000000 if v&0x80000000 else v
                pose=dict(epoch=epoch,x_m=signed(x)/1000,y_m=signed(y)/1000,yaw_rad=math.radians(signed(yaw)/100))
                self.mission.set_pose(pose,now)
                if not getattr(self,'cursor_initialized',False) and self.mission.state=='edit':
                    self.cursor=[pose['x_m'],pose['y_m']];self.cursor_initialized=True
        except (ValueError,KeyError,TypeError) as exc:
            self.error=str(exc)[:160];self.pause(now,'invalid_device_telemetry')
    def _epoch(self,epoch,now):
        if type(epoch) is not int or not 0<epoch<=0xffffffff:raise ValueError('Invalid localization epoch')
        self.discovery_speech.cancel('localization_reset')
        if self.epoch is not None:
            self.mission.cancel(now,'localization_reset');self.mission.state='invalidated'
            self._save_report()  # Save old coordinates before replacing their frame.
            self._finish_video('localization_reset')
            self.mission.new_task();self.mission.manual=False
            self.report_path=None;self.instruction='';self.recording_phase=None
        self.mission.actions.clear();self.mission.expected=None;self.mission.pose=None;self.mission.epoch=epoch;self.mission.pose_history.clear()
        self.actual=None;self.last_state=None;self.exit_requested=False;self.cursor_initialized=False
        self.edit_action=None
        self._stop_recording()
        self.epoch=epoch;self.observation_round+=1;self.observation_pose=None
        self.objects=Objects();self.upload=None;self.upload_info=None;self.upload_pending=None;self.layer=None;self.layer_crc=None
        self.radar=None;self.camera=None;self.detections=[];self.planned_route=[];self.route_history=[];self.route_key=None;self.generation+=1
    def readiness(self,now):
        serial=self.cfg.get('camera',{}).get('selector',{}).get('serial') or self.rt.cfg.get('camera',{}).get('selector',{}).get('serial')
        size=self.cfg.get('camera',{}).get('expected_size',self.rt.cfg.get('camera',{}).get('expected_size',[]))
        return dict(calibration=not self.calibration.geometry_problems(size,serial),camera=self.camera is not None,
            fpga=bool(self.usb and self.usb.status()['verified'] and self.actual),map=self.radar is not None,
            layer=bool(self.actual and self.actual['flags']&16 and self.layer_crc is not None and self.layer and self.layer_crc==self.layer['crc32']))
    def confirm(self,now):
        if not self.active or not self.input or self.input['manual'] or self.exit_requested:return False
        state=self.mission.state
        if self.evidence_error and state in ('ready','paused','fault','completed','cancelled','invalidated'):
            if not self.storage_check or self.storage_check.done():self.storage_check=asyncio.create_task(self._retry_storage())
            self.notice='正在重新检查记录资源，完成后再次SWA确认';return False
        if state=='edit':
            if self.input['view']!=1:self.notice='请用CH8切到地图后选点';return False
            if self.edit_action is None:
                self.edit_action=0;self.edit_stick_ready=False;return True
            action=self.edit_action;self.edit_action=None
            if action==0:
                if self.radar is None:self.notice='等待雷达地图';return False
                if bit(self.radar,*cell(self.cursor)):self.notice='目标位于障碍格，请换点';return False
                self.mission.add_waypoint(self.cursor);self.notice='已添加P%d，继续选点或打开菜单开始'%len(self.mission.waypoints)
            elif action==1:
                self.mission.undo_waypoint();self.notice='已撤销末点，剩余%d点'%len(self.mission.waypoints)
            elif action==2:
                self.mission.clear_waypoints();self.notice='已清空路径点'
            else:
                if not self.mission.waypoints:self.notice='请先添加路径点';return False
                self.mission.prepare_start();self.notice='共%d点，完成后自动返航；再次SWA开始'%len(self.mission.waypoints)
            return True
        if state=='ready':
            if self.video_recovery and not self.video_recovery.done():self.notice='正在检查上次中断录像，请稍候';return False
            if self.video and not self.video.done.is_set():self.notice='上一段录像正在收尾';return False
            ready=self.readiness(now);accepted=self.mission.start(ready,now)
            if accepted:
                self.route_history=[];self.route_key=None;self.planned_route=[]
                self._begin_video(now)
            labels=dict(calibration='相机标定',camera='相机画面',fpga='FX2接口',map='雷达地图',layer='视觉图层')
            self.notice='正在准备双路录像，随后开始任务' if accepted else '尚缺：'+'、'.join(labels[k] for k,v in ready.items() if not v)
            return accepted
        if state in ('paused','manual_paused','fault'):
            accepted=self.mission.resume(self.readiness(now),now,layer=self.layer)
            if accepted:self.discovery_speech.notify((self.mission.session_id,self.mission.task_id,self.epoch),self.objects.snapshot())
            self.notice='请求继续' if accepted else '尚不满足恢复条件';return accepted
        if state in ACTIVE:self.pause(now,'operator_pause');return True
        if state in ('completed','cancelled','invalidated') and (self.stopped or (self.actual and not self.actual['flags']&32)):
            self.mission.new_task();self.objects=Objects();self.observation_objects=None;self.observation_frames=0;self.observation_attempts=0
            self.layer=None;self.layer_crc=None;self.layer_dirty=True
            self.cursor=[self.mission.pose['x_m'],self.mission.pose['y_m']] if self.mission.pose else [0,0];self.report_path=None;self.notice='新任务：SWA打开路径点菜单';return True
        return False
    def pause(self,now,reason='operator_pause'):
        self.discovery_speech.cancel(reason)
        self.observation_objects=None
        if not (self.mission.state in ('paused','manual_paused') and self.mission.reason==reason):
            self.mission.pause(now,reason)
        self.upload=None;self.upload_info=None;self.upload_pending=None
        if self.usb:self.usb.drop_controls()
    def back(self,now):
        if not self.input or not self.input['open'] or self.input['manual']:return
        if self.edit_action is not None:self.edit_action=None;return
        if self.exit_requested:return
        state=self.mission.state
        if state in ACTIVE:self.pause(now,'operator_back');return
        if state in ('paused','manual_paused','fault'):
            self.mission.cancel(now,'operator_cancel');self._save_report();self._finish_video('operator_cancel')
        if state=='ready':self.mission.state='edit';self.mission.goal=None;self.notice='继续编辑，已选路径点保留';return
        if state in ('edit','completed','cancelled','invalidated') and self.actual and self.actual['flags']&32:
            self.mission._command('cancel',now,reason='inspection_page_exit')
        self.exit_requested=True
        self._stop_recording()
        self.upload=None;self.upload_info=None;self.upload_pending=None
        if self.usb:self.usb.drop_controls()
        self.notice='正在返回AI功能页'
        self._close_page()
    def _close_page(self):
        if self.exit_requested and self.actual and self.input and not self.input['manual']:
            if self.stopped or not self.actual['flags']&32:
                self.send(wire.UI_CLOSE,[self.mission.session_id,self.epoch,self.context,1])
    def capture_pose(self,frame):
        frame['dedup_pose_target_s']=frame['received_at_s']-CAMERA_DELAY_SECONDS
        frame['dedup_pose']=self.mission.pose_at(frame['dedup_pose_target_s'])

    async def observe(self,frame,result,now):
        if not self.enabled:return
        generation=self.generation
        import cv2
        image=await asyncio.to_thread(cv2.imread,frame['image'])
        if generation!=self.generation:return
        if image is None or image.shape[:2]!=(frame['height'],frame['width']):raise ValueError('Camera JPEG geometry changed')
        self.camera=dict(bgr=image,sequence=frame['sequence'],metadata=frame);self.last_camera=now
        self.detections=[{**d,'frame_sequence':frame['sequence']} for d in result.get('detections',[])[:64]]
        detections=[d for d in self.detections if d['confidence']>=DETECTION_CONFIDENCE_THRESHOLD]
        if not self.active:return
        if self.mission.state in ('await_start','outbound','returning','no_path'):
            if any(not self._known_target(d,frame) for d in detections):
                self.mission.request_observation(now);self.notice='发现新目标，停车后等待0.3秒定位'
            return
        if self.mission.state!='observing' or frame['inspection_round']!=self.observation_round:return
        serial=self.cfg['camera']['selector']['serial']
        located=[];rejected=[]
        for d in detections:
            try:
                if self._known_target(d,frame):continue
                point=self.calibration.locate_stopped(ground_anchor(d),
                    [frame['width'],frame['height']],serial,self.observation_pose,camera_metadata=frame)
                cell(point['xy_m'])
            except ValueError as exc:
                rejected.append(dict(target=d['class'],reason=str(exc)));continue
            located.append({**point,'class':d['class'],'radius_m':.05})
        objects=self.observation_objects
        objects.observe(located,objects.last_sequence+1,self.epoch,now,consecutive=False)
        self.observation_attempts+=1
        if located:self.observation_frames+=1
        pending=[o for o in objects.items if not o['confirmed']]
        new=[o for o in objects.items if o['confirmed'] and not any(old['confirmed'] and old['id']==o['id'] for old in self.objects.items)]
        if (pending or not new) and self.observation_attempts<10:
            self.notice='定位采样：有效%d帧，已处理%d/10张'%(self.observation_frames,self.observation_attempts)
            return
        self.objects=objects;self.observation_objects=None
        self._record_evidence('keyframe',Path(frame['image']).read_bytes(),dict(frame=frame,
            confirmed_ids=[o['id'] for o in new],projections=located,rejected=rejected,pose=self.observation_pose,
            localization='stop receipt + .3s wait + three valid samples per object',calibration=self.calibration.profile))
        self.layer=build(self.objects.snapshot(),.20);self.layer_dirty=False
        if not new:
            self.pause(now,'insufficient_localization_samples')
            self.notice='目标有效定位不足3帧，任务暂停；调整后SWA重试';return
        self.discovery_speech.notify((self.mission.session_id,self.mission.task_id,self.epoch),new)
        self.mission.finish_observation(self.layer,now)
        self.notice='已确认%d个目标；%d个样本不足，未入障碍层'%(len(new),len(pending)) if pending else '目标定位完成，提交障碍层后继续'
    def _known_target(self,d,frame):
        candidates=[obj for obj in self.objects.items if obj['confirmed'] and obj['class']==d['class']]
        if not candidates:return False
        pose=frame['dedup_pose']
        if pose is None:return False
        serial=self.cfg['camera']['selector']['serial']
        # Approximate H projection is only an association hint, including
        # outside the measured hull; formal stopped coordinates keep its bounds.
        try:
            point=self.calibration.project(ground_anchor(d),[frame['width'],frame['height']],serial,
                camera_metadata=frame,check_region=False)
        except ValueError:return False
        xy=base_to_map(point['point_base_m'],(pose['x_m'],pose['y_m'],pose['yaw_rad']))[:2]
        return any(math.dist(xy,obj['xy_m'])<self.dedup_distance_m for obj in candidates)
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
                commissioned=True
                self.send(wire.CONTROL,wire.control(action,user_token=self.input['user_token'] if self.input else 0,commissioned=commissioned),priority=kind!='goto')
                if kind=='goto':self.mission.last_status=now;self.planned_route=[];self.route_key=None
            elif kind=='commit_layer':
                self._begin_upload(self.mission.pending_layer,action['layer_version'],action['task_id'],action['command_id'])
            elif kind=='prepare_return_layer':
                if self.layer:self.mission.submit_layer(self.layer,now)
                else:self.pause(now,'return_layer_unavailable')
    def _edit_stick(self,steer,throttle):
        if self.edit_action is not None:
            if abs(throttle)<=250:self.edit_stick_ready=True
            elif self.edit_stick_ready:
                self.edit_action=(self.edit_action+(-1 if throttle>250 else 1))%4
                self.edit_stick_ready=False
        elif abs(steer)>250 or abs(throttle)>250:
            self.cursor=[max(-6.375,min(6.375,self.cursor[0]+(-.15 if steer>250 else .15 if steer<-250 else 0))),
                         max(-6.375,min(6.375,self.cursor[1]+(.15 if throttle>250 else -.15 if throttle<-250 else 0)))]
            self.cursor_initialized=True
    async def _render(self,generation,binding,recognition=False):
        try:
            from .render import render,render_recognition,rgb565
            mission=self._display_mission();objects=self.objects.snapshot();camera=self.camera;detections=list(self.detections)
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
        if self.rt.gateway.transport.fd is None:return
        if (self.caps is None or self.actual is None) and now>=self.next_hello:
            self.send(wire.HELLO,[self.mission.session_id],priority=True);self.next_hello=now+1.
        if self.actual and now>=self.next_heartbeat:
            self.send(wire.HEARTBEAT,[self.mission.session_id,self.actual['task_id'],self.epoch,self.actual['command_id']]);self.next_heartbeat=now+.1
        verified=bool(self.usb and self.usb.status()['verified'])
        if self.usb_was_verified and not verified:
            self.pause(now,'highspeed_link_lost');self.layer_crc=None
        self.usb_was_verified=verified
        if self.evidence_error:self.pause(now,'evidence_recording_failed');self.notice='数据记录失败，任务暂停，请检查存储'
        preparing=self.video_required and not self.video.ready and now-self.video.started<3
        previous=self.mission.state
        if not preparing:self.mission.tick(now)
        if previous=='observe_wait' and self.mission.state=='observing':
            self.observation_round+=1;self.observation_pose=deepcopy(self.mission.pose)
            self.observation_objects=deepcopy(self.objects);self.observation_frames=0;self.observation_attempts=0
            for obj in self.observation_objects.items:
                if not obj['confirmed']:obj['hits']=0
        if self.usb:
            for frame in getattr(self.usb,'drain_routes',lambda:[])():
                try:
                    points=route_points(frame);key=(frame.sequence>>16,frame.payload)
                    self.planned_route=points;self.last_route=now
                    if key!=self.route_key:
                        self.route_key=key;self.route_history.append(dict(sequence=frame.sequence,phase=self.mission.phase,
                            waypoint_index=self.mission.waypoint_index,at_s=now,points=points))
                        self.route_history=self.route_history[-512:]
                except ValueError as exc:self.error=str(exc)
            for frame in self.usb.drain_maps():
                if self.input and frame.view==self.input['view']:
                    self.radar=frame.payload;self.last_map=now
        if self.exit_requested:
            self._actions(now);return
        if not self.active:
            if (self.recognition_display and self.usb and self.usb.status()['verified'] and self.input and self.input['view']==0
                    and self.actual and now>=self.next_render and (not self.render_job or self.render_job.done())):
                binding=(self.mission.session_id,self.actual['task_id'],self.epoch,self.context)
                self.render_job=asyncio.create_task(self._render(self.generation,binding,recognition=True));self.next_render=now+self.render_interval
            self._actions(now);return
        if self.input and now>=self.next_stick:
            if self.mission.state=='edit' and self.input['view']==1:
                self._edit_stick(self.input['steer'],self.input['throttle'])
            elif self.input['view']==2:
                from .render import list_entries
                if self.input['throttle']>250:self.scroll=max(0,self.scroll-1)
                elif self.input['throttle']<-250:self.scroll=min(max(0,len(list_entries(self.mission.snapshot(),self.objects.snapshot()))-12),self.scroll+1)
            self.next_stick=now+.12
        if self.layer is None or self.layer_dirty:
            self.layer=build(self.objects.snapshot(),.20);self.layer_dirty=False
        if self.layer and not self.upload_info and self.actual and self.input:
            changed=self.layer_crc!=self.layer['crc32']
            if changed and self.mission.state in ('outbound','returning','no_path'):
                self.mission.submit_layer(self.layer,now)
            elif changed and now>=self.next_upload_retry and self.mission.state in ('edit','ready') and self.stopped and self.actual['flags']&8:
                self._begin_upload(self.layer,self.actual['layer_version']+1,self.actual['task_id'],self.actual['command_id'])
        self._actions(now);self._upload_tick()
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
        path=directory/('session-%d-task-%d-epoch-%d.json'%(self.mission.session_id,self.mission.task_id,self.epoch))
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
            ground_anchors=dict(GROUND_ANCHORS),
            deduplication=dict(distance_m=self.dedup_distance_m,pose_history_s=POSE_HISTORY_SECONDS,pose_offset_s=CAMERA_DELAY_SECONDS,
                pose_time_origin='native frame received_at_s before inference',pose_selection='nearest',rough_points_published=False),
            demo=dict(camera_delay_assumed_s=CAMERA_DELAY_SECONDS,settle_s=SETTLE_SECONDS,frames=3,vehicle_radius_m=.20,
                      object_radius_m=.05,location_margin_m=.03),
            return_policy='latest_map',fpga_route_history=self.route_history,evidence=[entry for entry in self.evidence_files if (entry.get('session'),entry.get('task'),entry.get('epoch'))==(self.mission.session_id,self.mission.task_id,self.epoch)],device_state=self.actual,
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
            edit_action=self.edit_action,
            exit_requested=self.exit_requested,
            recognition_display=self.recognition_display,discovery_speech=self.discovery_speech.snapshot(),
            archive=dict(root=str(self.video_root),url='http://127.0.0.1:%d'%self.archive_server.server_port if self.archive_server else None,error=self.playback_error),
            calibration_model=dict(projection_model=profile.get('projection_model','pinhole_ground'),
                static_geometry_config_ready=not self.calibration.geometry_problems(size,serial),
                pending=self.calibration.geometry_problems(size,serial),
                pixel_domain=profile.get('homography',{}).get('pixel_domain'),focus=profile.get('focus'),
                max_range_m=profile.get('max_range_m'),ground_hull_cm=profile.get('homography',{}).get('ground_hull_cm')),
            deduplication=dict(distance_m=self.dedup_distance_m,pose_history_s=POSE_HISTORY_SECONDS,
                pose_offset_s=CAMERA_DELAY_SECONDS,pose_samples=len(self.mission.pose_history)),
            serial_owner='coordinator',localization='stop_then_wait_0.3s_then_three_frames',
            observation_round=self.observation_round,observation_frames=self.observation_frames,observation_attempts=self.observation_attempts)
    def lines(self):
        from .render import STATES
        labels=dict(calibration='相机和地面标定',camera='相机',fpga='接口和回执',map='雷达地图',layer='视觉障碍层')
        state=self.mission.snapshot();ready=self.readiness(time.monotonic());missing=[k for k,v in ready.items() if not v]
        return ['视觉巡检：摄像头 / 雷达地图 / 文字','状态：'+STATES.get(state['state'],state['state']),self.notice,
            'CH8切换视图，SWA打开选点操作菜单','返回：关闭菜单 / 暂停 / 取消并退出',
            '路径点：%d/%d已完成'%(state['waypoints_completed'],state['waypoint_count']),
            '当前目标：'+str(state['goal_m']),'起点：'+str(state['start_m']),
            '已定位物体：'+str(sum(x['confirmed'] for x in self.objects.items)),
            '指令：'+self.instruction,'待就绪：'+'、'.join(labels.get(k,k) for k in missing),'等待相机、地图和接口就绪' if missing else '巡检速度：%d mm/s'%self.mission.cruise_speed_mm_s]
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
        if self.storage_check:await self.storage_check
        if self.archive_server:
            await asyncio.to_thread(self.archive_server.shutdown);self.archive_server.server_close();self.archive_thread.join(timeout=1)
        self.closed=True;self._stop_recording()
        jobs=[j for j in (self.record_job,self.render_job) if j]
        for job in jobs:job.cancel()
        if jobs:await asyncio.gather(*jobs,return_exceptions=True)
        if self.evidence_jobs:await asyncio.gather(*list(self.evidence_jobs),return_exceptions=True)
        if self.usb:await asyncio.to_thread(self.usb.close)
