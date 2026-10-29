"""Formal MOT1 execution backend over the coordinator's single serial owner.

The backend never opens a second UART.  It shares the FAM1 gateway's
``SerialPort``, queues only complete 12-byte MOT1 frames, and keeps a
dedicated heartbeat task so ASR/vision/LLM/TTS work cannot starve the FPGA's
500 ms watchdog.

Truthful state boundary: a completed serial ticket means the 12 bytes reached
the OS serial buffer.  The released FPGA bridge has **no task-level MOT1
acknowledgment**.  Therefore this module never sets ``executed``,
``action_complete_verified`` or ``physical_stop_verified`` from a write or a
timer.  It records an unverified host window and an explicit STOP attempt.
"""
from __future__ import annotations

import asyncio
import time
import uuid
from .motion_reference.mot1 import (BACKWARD,FORWARD,PATROL_START,PATROL_STOP,
                                     SPIN,STOP,TURN_LEFT,TURN_RIGHT,Frame,from_intent,heartbeat)
from .motion_intents import DRIVE_SPEED_CODE,motion_frames

VERSION='mot1-shared-serial-1'
WATCHDOG_MS=500


class MotionBackend:
    def __init__(self,transport_provider,config,audit):
        self._transport_provider=transport_provider
        self.cfg=config
        self.audit=audit
        self._task=None
        self._current=None
        self._epoch=0
        self._seq=0
        self.last_result=None
        self._owns_control=False
        self.closed=False

    @property
    def busy(self):return self._current is not None
    @property
    def owns_control(self):return self._owns_control

    # -------------------------------------------------------------- config
    def _voice(self):return self.cfg.get('voice_motion',{}) or {}
    def _gateway(self):return self.cfg.get('gateway',{}) or {}

    @property
    def transport(self):
        try:return self._transport_provider()
        except Exception:return None

    def _speed_code(self):
        value=self._voice().get('speed',10)
        if type(value) is not int or not 1<=value<=40:raise ValueError('voice_motion.speed must be an integer 1..40')
        return value

    def _heartbeat_s(self):
        value=self._voice().get('heartbeat_s',.2)
        if type(value) not in (int,float) or not .05<=float(value)<=.25:
            raise ValueError('voice_motion.heartbeat_s must be 0.05..0.25 (RTL watchdog is 0.5 s)')
        return float(value)

    def _turn_window_s(self):
        value=self._voice().get('turn_window_s',8.5)
        if type(value) not in (int,float) or not .1<=float(value)<=9.5:
            raise ValueError('voice_motion.turn_window_s must be 0.1..9.5')
        return float(value)

    def _drive_margin_s(self):
        value=self._voice().get('drive_margin_s',.3)
        if type(value) not in (int,float) or not 0<=float(value)<=1:
            raise ValueError('voice_motion.drive_margin_s must be 0..1')
        return float(value)

    def distance_policy(self):return {'max_distance_m':self._voice().get('distance',{}).get('max_distance_m',2.),'estimated_speed_mps':.3}

    def settings(self):
        return {'enabled':self._voice().get('enabled') is True,
                'live_commissioned':self._voice().get('live_commissioned') is True,
                'backend':self._voice().get('backend','mot1_shared_serial'),
                'speed':self._voice().get('speed',10),
                'drive_speed':DRIVE_SPEED_CODE,
                'heartbeat_s':self._voice().get('heartbeat_s',.2),
                'turn_window_s':self._voice().get('turn_window_s',8.5),
                'drive_margin_s':self._voice().get('drive_margin_s',.3)}

    def readiness_reason(self):
        voice=self._voice();gateway=self._gateway();transport=self.transport
        if voice.get('enabled') is not True:return 'voice_motion_disabled'
        if voice.get('backend','mot1_shared_serial')!='mot1_shared_serial':return 'motion_backend_not_configured'
        if voice.get('live_commissioned') is not True:return 'motion_not_commissioned'
        if gateway.get('mode')!='serial':return 'serial_gateway_not_live'
        device=gateway.get('device')
        if not isinstance(device,str) or not device.startswith('/dev/serial/by-id/'):return 'serial_device_not_configured'
        if transport is None:return 'serial_transport_unavailable'
        if getattr(transport,'fd',None) is None:return 'serial_not_open'
        try:self._speed_code();self._heartbeat_s();self._turn_window_s();self._drive_margin_s()
        except ValueError as exc:return 'invalid_motion_config: '+str(exc)
        return None

    def _next_seq(self):
        self._seq=(self._seq+1)&0xff
        return self._seq

    # ------------------------------------------------------------ results
    def _base(self,decision,raw_text,plan_only):
        result={'operation':'motion','raw_text':raw_text,'motion':decision,
                'policy_version':VERSION,'plan_only':plan_only,
                'accepted':False,'sent':False,'executed':False,
                'frame_queued':False,'frame_bytes_written':0,
                'device_reply_verified':False,'action_complete_verified':False,
                'completion_receipt':False,'physical_stop_verified':False,
                'physical_task_completed':False,'motion_authorized':False,
                'action_state':'not_started','reason':None}
        if decision.get('distance_estimate'):
            intent=decision.get('intent',{})
            result['distance']={'mode':'timed_estimate','target_m':intent.get('distance_target_m',intent.get('distance_m')),
                                'estimated_speed_mps':intent.get('estimated_speed_mps'),
                                'estimated_duration_s':intent.get('duration_s'),
                                'arrival_verified':False,
                                'text':'按设定速度估算时长；到达距离未验证'}
            if intent.get('estimated_speed_mps') is not None:
                result['operator_message']=(f"按设定 {intent['estimated_speed_mps']:.2f} m/s 估算约 "
                    f"{intent['duration_s']:.1f} s，目标 {intent.get('distance_target_m',intent.get('distance_m')):.2f} m；"
                    "到达未验证。")
        return result

    def _sync(self,action):
        ticket=action['ticket'];result=action['result']
        result.update(frame_queued=True,frame_bytes_written=ticket.written,
                      sent=ticket.complete,action_state='heartbeat_window' if ticket.complete else 'queued')
        if action.get('heartbeats'):result['heartbeats_sent']=action['heartbeats']

    def _public(self,action):return action['result'] if action else None

    def preview(self,decision,raw_text,plan_only=True):
        result=self._base(decision,raw_text,plan_only)
        result.update(preview_only=True,reason='plan_only' if plan_only else decision.get('reason'))
        return result

    # ------------------------------------------------------------ lifecycle
    def _cancel_current(self,reason):
        old=self._current;self._current=None
        task=self._task;self._task=None
        if task is not None and not task.done():task.cancel()
        if old:
            old['result'].update(action_state=reason,sent=old['result'].get('sent',False),
                                 executed=False,action_complete_verified=False,
                                 physical_stop_verified=False,physical_task_completed=False)
        return old

    def _stop_frame(self,reason,priority=True,tag='motion_stop'):
        transport=self.transport
        if transport is None or getattr(transport,'fd',None) is None:return None,0,None
        try:
            transport.drop('motion')
            ticket=transport.enqueue(Frame(self._next_seq(),STOP,0,0).encode(),priority=priority,tag=tag)
            transport.flush()
            return ticket,ticket.written,None
        except (OSError,RuntimeError,ValueError) as exc:
            return None,0,type(exc).__name__+': '+str(exc)[:200]

    def stop_now(self,reason='operator_stop'):
        """Synchronous, best-effort STOP.  Never claims physical verification."""
        old=self._cancel_current('stopped')
        self._epoch+=1
        transport=self.transport
        allowed=(old is not None) or (self._voice().get('enabled') is True and
                 self._voice().get('live_commissioned') is True and
                 self._gateway().get('mode')=='serial' and
                 transport is not None and getattr(transport,'fd',None) is not None)
        ticket=written=error=None
        if allowed:
            self._owns_control=True
            ticket,written,error=self._stop_frame(reason)
        stop={'operation':'motion_stop','reason':reason,'backend':VERSION,
              'stop_frame_queued':ticket is not None,'stop_frame_bytes_written':written,
              'stop_frame_sent':bool(ticket and ticket.complete),
              'device_reply_verified':False,'action_complete_verified':False,
              'physical_stop_verified':False,'physical_task_completed':False,
              'motion_authorized':bool(ticket),
              'error':error}
        if old:old['result'].update(stop_frame_sent=stop['stop_frame_sent'],
            stop_frame_bytes_written=written,stop_error=error)
        self.last_result=stop
        return stop

    def link_lost(self,reason='serial_link_lost'):
        old=self._cancel_current('link_lost')
        self._epoch+=1
        result=old['result'] if old else {'operation':'motion','action_state':'link_lost'}
        result.update(action_state='link_lost',reason=reason,executed=False,
                      action_complete_verified=False,physical_stop_verified=False,
                      physical_task_completed=False)
        self.last_result=result
        return result

    def close(self):
        if self.closed:return
        self.stop_now('service_shutdown')
        self.closed=True

    # ------------------------------------------------------------ execution
    def execute(self,decision,raw_text,plan_only=False):
        result=self._base(decision,raw_text,plan_only)
        if plan_only:
            result['reason']='plan_only';return result
        if not decision.get('valid'):
            result['reason']=decision.get('reason','invalid_motion');return result
        if decision.get('wire_preview') is None:
            result['reason']=decision.get('reason','no_mot1_frame');return result
        reason=self.readiness_reason()
        if reason:
            result['reason']=reason;return result
        transport=self.transport
        try:
            frames=motion_frames(decision['intent'],seq=self._next_seq(),speed=self._speed_code())
            frame=frames[0]
            self._seq=frames[-1].seq
            if frame is None:raise ValueError('Reviewed intent has no MOT1 mapping')
            if frame.cmd in (FORWARD,BACKWARD) and not 1<=frame.value<=5000:raise ValueError('drive duration outside 1..5000 ms')
            if frame.cmd in (TURN_LEFT,TURN_RIGHT,SPIN) and not 1<=abs(frame.value)<=360:raise ValueError('turn angle outside 1..360 degrees')
            if frame.cmd==PATROL_START:raise ValueError('patrol_start is not connected to a real route')
            if frame.cmd==PATROL_STOP:raise ValueError('patrol_stop must use the explicit stop path')
        except (ValueError,TypeError) as exc:
            result['reason']='motion_frame_rejected: '+str(exc)[:200];return result

        # Cancel the previous host action and remove its queued heartbeats.
        self._cancel_current('superseded')
        self._epoch+=1;epoch=self._epoch
        ticket=None
        try:
            transport.drop('motion')
            ticket=transport.enqueue(frame.encode(),tag='motion')
            transport.flush()
        except (OSError,RuntimeError,ValueError) as exc:
            self._stop_frame('motion_queue_failure')
            result['reason']='serial_write_failed: '+type(exc).__name__+': '+str(exc)[:200]
            result['action_state']='failed';self.last_result=result;return result
        action_id=uuid.uuid4().hex
        self._owns_control=True
        result.update(accepted=True,action_id=action_id,reason='accepted_no_device_receipt',
                      motion_authorized=True,action_state='queued')
        window=(sum(f.value for f in frames)/1000.0+self._drive_margin_s()) if frame.cmd in (FORWARD,BACKWARD) else self._turn_window_s()
        action={'id':action_id,'epoch':epoch,'ticket':ticket,'result':result,
                'deadline':time.monotonic()+window,'window_s':window,'heartbeats':0,
                'remaining_frames':frames[1:],'segment_end':time.monotonic()+frame.value/1000,
                'segments_sent':1}
        self._current=action;self.last_result=result;self._sync(action)
        try:
            self._task=asyncio.create_task(self._run(action))
        except RuntimeError as exc:
            self._current=None;self._stop_frame('motion_task_start_failure')
            result.update(accepted=False,reason='event_loop_unavailable: '+str(exc)[:200],action_state='failed')
        return result

    def _advance_segment(self,action,now):
        if action['epoch']!=self._epoch or not action['remaining_frames'] or now<action['segment_end']:return
        frame=action['remaining_frames'].pop(0)
        # Use the shared sequence after any intervening heartbeats.
        frame=Frame(self._next_seq(),frame.cmd,frame.value,frame.speed,frame.flags)
        action['ticket']=self.transport.enqueue(frame.encode(),tag='motion')
        self.transport.flush()
        action['segments_sent']+=1
        action['result']['segments_sent']=action['segments_sent']
        action['segment_end']=now+frame.value/1000
        action['deadline']=action['segment_end']+sum(f.value for f in action['remaining_frames'])/1000+self._drive_margin_s()

    async def _run(self,action):
        try:
            while True:
                if self._epoch!=action['epoch']:return
                transport=self.transport
                if transport is None or getattr(transport,'fd',None) is None:
                    raise OSError('serial link lost during motion')
                transport.flush();self._sync(action)
                self._advance_segment(action,time.monotonic())
                remaining=action['deadline']-time.monotonic()
                if remaining<=0:break
                delay=min(self._heartbeat_s(),remaining)
                if action['remaining_frames']:delay=min(delay,max(0,action['segment_end']-time.monotonic()))
                await asyncio.sleep(delay)
                if self._epoch!=action['epoch']:return
                transport.enqueue(heartbeat(self._next_seq()).encode(),tag='motion')
                transport.flush();action['heartbeats']+=1;self._sync(action)
            if self._epoch!=action['epoch']:return
            ticket,written,error=self._stop_frame('heartbeat_window_elapsed')
            action['result'].update(action_state='host_window_elapsed',
                stop_frame_queued=ticket is not None,stop_frame_bytes_written=written,
                stop_frame_sent=bool(ticket and ticket.complete),stop_error=error,
                reason='host_window_elapsed_no_device_receipt',
                action_complete_verified=False,physical_stop_verified=False,
                physical_task_completed=False,executed=False)
        except asyncio.CancelledError:
            return
        except Exception as exc:
            if self._epoch==action['epoch']:
                ticket,written,error=self._stop_frame('motion_exception')
                action['result'].update(action_state='failed',
                    reason='motion_exception: '+type(exc).__name__+': '+str(exc)[:200],
                    stop_frame_sent=bool(ticket and ticket.complete),stop_error=error,
                    action_complete_verified=False,physical_stop_verified=False,
                    physical_task_completed=False,executed=False)
        finally:
            if self._current is action:self._current=None
            if self._task is asyncio.current_task():self._task=None

    # ------------------------------------------------------------ telemetry
    def snapshot(self):
        current=self._public(self._current)
        return {'enabled':self._voice().get('enabled') is True,
                'live_commissioned':self._voice().get('live_commissioned') is True,
                'backend':self._voice().get('backend','mot1_shared_serial'),
                'policy_version':VERSION,'transport':'shared_fam1_mot1_serial',
                'serial_owner':'coordinator',
                'device_configured':isinstance(self._gateway().get('device'),str) and
                    self._gateway().get('device','').startswith('/dev/serial/by-id/'),
                'serial_open':self.transport is not None and getattr(self.transport,'fd',None) is not None,
                'heartbeat_s':self._voice().get('heartbeat_s',.2),'rtl_watchdog_ms':WATCHDOG_MS,
                'device_receipts_supported':False,'action_complete_verified':False,
                'physical_stop_verified':False,'patrol_start_connected':False,
                'tof_collision_protection':False,
                'settings':self.settings(),'readiness_reason':self.readiness_reason(),
                'busy':self.busy,'owns_control':self.owns_control,
                'distance':{'mode':'timed_estimate','policy':self.distance_policy(),
                            'arrival_verified':False},
                'last_instruction':self.last_result,'action':current}
