"""Physical AI master switch, mode selection and release-to-transcribe audio.

Controls have a short lease. A mode change, manual takeover, emergency stop or
lost link cancels old work before it can publish speech or authorize motion.
"""
import asyncio
from collections import deque
import uuid
import time
from .ai_link import COLS, display_frames, font_rows
from .announcements import ALSAPlayer, NAMES
from .motion_reference.mot1 import Frame
from .processes import ModelServer, Worker
from .switched_capture import capture
from .device_roles import audio_device


def wrapped(text, rows):
    text = str(text).replace('\n', ' ').replace('\r', ' ')
    return [text[i * COLS:(i + 1) * COLS] for i in range(rows)]


class AiModule:
    def __init__(self, runtime):
        self.rt = runtime
        self.cfg = runtime.cfg.get('ai_module', {})
        self.enabled = self.cfg.get('enabled') is True
        self.clock = time.monotonic
        self.atlas = font_rows() if self.enabled else None
        self.font_caches = [{},{}]
        self.display_pending = None
        self.display_ack = self.display_bank = 0
        self.controls = None
        self.last_rx = None
        self.sequence = None
        self.active = False
        self.paused_manual = False
        self.vision_resume_required = False
        self.confirm_toggle = None
        self.rearm_required = runtime.cfg.get('startup', {}).get('enabled') is True
        self.chat = False
        self.mode = 'semantic_navigation'
        self.selected = 0
        self.vision = False
        self.record_armed = False
        self.record_high = False
        self.released = asyncio.Event()
        self.work = None
        self.cleanup = set()
        self.vision_lock = asyncio.Lock()
        self.epoch = 0
        self.state = 'off'
        self.error = None
        self.instruction = ''
        self.reply = ''
        self.detections = []
        self.target = None
        self.history = []
        self.display = deque()
        self.display_key = None
        self.transaction = 0
        self.last_ping = 0.
        self.last_display = 0.
        self.chat_worker = Worker('chat', runtime.config, runtime.run_dir / 'chat')
        self.tts_worker = Worker('tts', runtime.config, runtime.run_dir / 'tts')
        self.chat_server = ModelServer(runtime.cfg.get('models', {}).get('chat', {}), runtime.run_dir / 'chat')

    def rejection(self, need_vision=False):
        if not self.enabled:
            return None
        if not self.active or self.last_rx is None or self.clock()-self.last_rx > self.cfg.get('control_timeout_s',.5):
            return 'ai_master_off_or_control_link_unavailable'
        if self.mode != 'semantic_navigation':
            return self.mode + '_mode_cannot_control_motion'
        if need_vision and not self.vision:
            return 'object_recognition_disabled'
        return None

    def _task(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.cleanup.add(task)
        task.add_done_callback(self.cleanup.discard)
        return task

    def _cancel(self, reason, preserve_view=False):
        self.epoch += 1
        self.released.set()
        if self.work and not self.work.done():
            self.work.cancel()
        self.record_armed = False
        self.record_high = False
        if not preserve_view:
            self.instruction = self.reply = ''
        self.error = None
        self.rt.stop(reason)
        self.state = 'idle' if self.active else 'off'
        self.rt.audit({'type': 'ai_control_cancelled', 'reason': reason, 'epoch': self.epoch})

    def external_stop(self):
        if not self.enabled:
            return
        self.epoch += 1
        self.released.set()
        self.record_armed = False
        self.record_high = False
        if self.work and not self.work.done() and self.work is not asyncio.current_task():
            self.work.cancel()

    def on_controls(self, event, now):
        if not self.enabled:
            return
        seq = event['sequence']
        if self.sequence is not None and not 0 < ((seq - self.sequence) & 255) < 128:
            return
        self.sequence = seq
        self.last_rx = now
        self.controls = event
        toggle=event.get('confirm_toggle',False)
        confirmed=self.confirm_toggle is not None and toggle!=self.confirm_toggle
        self.confirm_toggle=toggle
        self.display_ack=event.get('display_ack',0)
        self.display_bank=event.get('display_bank',0)
        if not event['enabled'] and event['rc_valid'] and not event['manual'] and not event['estop']:
            self.rearm_required=False
        previous_pause=self.paused_manual
        paused=bool(event['enabled'] and event['manual'] and event['rc_valid'] and not event['estop'])
        startup=getattr(self.rt,'startup',None)
        usable = (event['enabled'] and event['rc_valid'] and event['swb_valid'] and
                  not self.rearm_required and
                  not event['manual'] and not event['estop'] and
                  (startup is None or startup.ready) and
                  self.rt.cfg.get('model_loads_enabled') is True)
        previous_mode=self.mode
        self.mode, self.selected = event['mode'], event['selected']
        chat = event['chat']
        if paused:
            self.vision_resume_required=True
        elif usable and (confirmed or event['mode']!='object_recognition'):
            self.vision_resume_required=False
        vision = bool(usable and not chat and (event['vision'] or (self.mode=='visual_inspection' and self.rt.cfg.get('inspection',{}).get('enabled') is True)) and not self.vision_resume_required)
        changed = ((usable, chat, vision, paused) != (self.active, self.chat, self.vision, self.paused_manual)
                   or (usable and (confirmed or previous_mode!=self.mode)))
        self.active, self.chat, self.vision = usable, chat, vision
        self.paused_manual=paused
        if changed:
            self._cancel('ai_manual_pause' if paused else 'ai_switch_or_mode_changed',
                         preserve_view=paused or previous_pause)
            if paused:self.state='paused_manual'
            elif usable and self.vision_resume_required:self.state='waiting_next'
            self.target = None
            self._task(self._set_vision(vision))
            if not usable:
                if not paused:
                    self.history = []
                    self._task(self._release_models(self.work))
            elif not chat:
                self._task(self._release_chat(self.work))
        if not usable:
            return
        if self.mode == 'visual_inspection':
            # The deterministic inspection controller owns child gestures and
            # recording. Do not interpret them as semantic drive requests.
            return
        if not event['record']:
            self.released.set()
            self.record_high = False
            self.record_armed = True
        elif self.record_armed and not self.record_high:
            self._cancel('ai_recording_started')
            self.record_high = True
            self.record_armed = False
            # A held down switch never queues repeated utterances.
            previous = self.work
            if previous and not previous.done():
                previous.cancel()
            self.released = asyncio.Event()
            self.work = asyncio.create_task(self._replace_utterance(previous, self.epoch, self.chat, self.released))

    async def _replace_utterance(self, previous, epoch, chat, released):
        if previous:
            await asyncio.gather(previous, return_exceptions=True)
        transitions = list(self.cleanup)
        if transitions:
            await asyncio.shield(asyncio.gather(*transitions, return_exceptions=True))
        if self._current(epoch, chat) and not released.is_set():
            await self._utterance(epoch, chat, released)

    async def _set_vision(self, enabled):
        async with self.vision_lock:
            await self._change_vision(enabled)

    async def _change_vision(self, enabled):
        if enabled != self.vision:
            return
        # A running inspection recording owns an independent camera/inference
        # lease through manual takeover and pauses. AI exits end that lease.
        if not enabled and getattr(getattr(self.rt,'inspection',None),'video_required',False):
            return
        if self.rt.camera_task:
            self.rt.camera_task.cancel()
            await asyncio.gather(self.rt.camera_task, return_exceptions=True)
            self.rt.camera_task = None
        if enabled != self.vision:
            return
        if enabled:
            if 'camera' not in self.rt.workers:
                self.error = 'camera_not_configured'
                self.rt.camera_state = 'unavailable'
                self.state = 'error'
            else:
                self.state = 'object_recognition'
                self.rt.camera_task = asyncio.create_task(self.rt._camera())
        else:
            if self.rt.image_pump:
                self.rt.image_pump.cancel()
                await asyncio.gather(self.rt.image_pump, return_exceptions=True)
            for name in ('camera', 'vision'):
                if name in self.rt.workers:
                    if name=='camera' or not self.rt.cfg.get('startup',{}).get('enabled'):
                        await self.rt.workers[name].close()
            self.rt.camera_state = 'disabled'
            self.detections = []

    async def _release_models(self, previous=None):
        if previous:
            await asyncio.gather(previous, return_exceptions=True)
        if not self.active and not self.rt.cfg.get('startup',{}).get('enabled'):
            await self.rt.workers['asr'].close()
            await self.chat_worker.close()
            await self.tts_worker.close()
            await self.chat_server.close()

    async def _release_chat(self, previous=None):
        if previous:
            await asyncio.gather(previous, return_exceptions=True)
        if not self.chat and not self.rt.cfg.get('startup',{}).get('enabled'):
            await self.chat_worker.close()
            await self.tts_worker.close()
            await self.chat_server.close()

    def _current(self, epoch, chat):
        return (self.active and self.epoch == epoch and self.chat == chat and self.last_rx is not None and
                self.clock()-self.last_rx<=self.cfg.get('control_timeout_s',.5))

    async def _utterance(self, epoch, chat, released):
        path = None
        output = None
        capturing = False
        try:
            # Wait for cancelled cached speech to stop before recording.
            self.rt.announcements._clear('ai_recording')
            for _ in range(20):
                if not self.rt.announcements.microphone_blocked:
                    break
                await asyncio.sleep(.05)
            self.rt.announcements.begin_capture()
            capturing = True
            await self.rt.cue.play('start')
            if not self._current(epoch, chat) or released.is_set():
                return
            self.state = 'recording'
            self.error = None
            private = self.rt.run_dir / 'ai_audio'
            private.mkdir(mode=0o700, parents=True, exist_ok=True)
            path = private / (uuid.uuid4().hex + '.wav')
            mic = self.rt.cfg.get('microphone', {})
            metadata = await capture(audio_device(self.rt.cfg.get('audio_roles',{}).get('microphone'),'microphone',mic.get('device')), path, released,
                                     maximum_s=self.cfg.get('max_recording_s', 30))
            self.rt.announcements.end_capture()
            capturing = False
            await self.rt.cue.play('stop')
            if not self._current(epoch, chat):
                return
            self.state = 'recognizing'
            response = await self.rt.workers['asr'].call({'wav': str(path)},
                timeout=self.rt.cfg.get('asr_timeout_s', 30))
            asr = response['result']
            if not self._current(epoch, chat):
                return
            if asr.get('status') != 'recognized':
                self.state = 'no_speech'
                return
            self.instruction = asr['text']
            self.rt.audit({'type': 'ai_utterance', 'text': self.instruction,
                           'chat': chat, 'capture': metadata})
            if self.mode == 'object_recognition':
                self.target = next((name for name, label in NAMES.items() if label in self.instruction), None)
                self.vision_resume_required=False
                self.vision=True
                await self._set_vision(True)
                if self.rt.camera_state!='unavailable':self.state = 'object_recognition'
                return
            if not chat:
                self.state = 'semantic_navigation'
                parsed = await self.rt.parse(self.instruction, self.rt.generation, False)
                self.rt.audit({'type': 'ai_navigation_result', 'result': parsed})
                control = parsed.get('preset_control', {})
                if control and not control.get('accepted') and control.get('operation') != 'stop':
                    self.error = control.get('reason')
                self.state = 'idle'
                return
            self.state = 'chat_thinking'
            if not self.rt.cfg.get('models', {}).get('chat'):
                raise RuntimeError('Qwen3.5-0.8B chat model is not configured')
            if self.rt.cfg['models']['chat'].get('provider') != 'rkllm' and (self.chat_server.proc is None or self.chat_server.proc.returncode is not None):
                await self.chat_server.start()
            response = await self.chat_worker.call({'text': self.instruction, 'history': self.history}, timeout=90)
            if not self._current(epoch, chat):
                return
            self.reply = response['result']['text']
            self.history = (self.history + [{'role': 'user', 'content': self.instruction},
                            {'role': 'assistant', 'content': self.reply}])[-8:]
            self.state = 'synthesizing'
            output = private / (uuid.uuid4().hex + '.wav')
            tts = await self.tts_worker.call({'text': self.reply, 'output': str(output)}, timeout=90)
            if not self._current(epoch, chat):
                return
            self.state = 'speaking'
            player = ALSAPlayer(self.cfg.get('speaker_device'),selector=self.rt.cfg.get('audio_roles',{}).get('speaker'))
            await player(tts['result'], {})
            self.state = 'idle'
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if self._current(epoch, chat):
                self.state = 'error'
                self.error = type(exc).__name__ + ': ' + str(exc)[:200]
                self.rt.audit({'type': 'ai_module_error', 'error': self.error})
        finally:
            if capturing:
                self.rt.announcements.end_capture()
            for file in (path, output):
                if file:
                    file.unlink(missing_ok=True)

    def observe_vision(self, detections):
        if self.vision:
            self.detections = [{'class': x['class'], 'confidence': x['confidence']}
                               for x in detections[:8] if self.target is None or x['class'] == self.target]

    def lines(self):
        if self.mode=='visual_inspection' and getattr(self.rt,'inspection',None) and self.rt.inspection.enabled:
            return self.rt.inspection.lines()
        found = ' '.join(NAMES.get(x['class'], x['class']) + ' %.0f%%' % (100*x['confidence'])
                         for x in self.detections)
        labels = {'off':'关闭','idle':'待命','recording':'录音中','recognizing':'语音识别中',
                  'chat_thinking':'聊天思考中','synthesizing':'语音合成中','speaking':'正在播报',
                  'no_speech':'未识别到语音','error':'功能暂不可用',
                  'semantic_navigation':'处理导航指令','object_recognition':'目标识别中',
                  'paused_manual':'手动接管，任务已暂停','waiting_next':'等待确认或新的录音任务'}
        mode = {'semantic_navigation':'语义导航','object_recognition':'目标识别','chat':'聊天机器人 Qwen3.5-0.8B',
                'visual_inspection':'视觉巡检'}[self.mode]
        requested = self.controls and self.controls.get('enabled')
        state = labels.get(self.state,self.state)
        motion=getattr(self.rt,'motion',None)
        if self.active and self.mode=='semantic_navigation' and motion and motion.snapshot().get('busy'):
            state='运动指令计时中'
        if requested and not self.active:
            state = '手动接管，任务已暂停' if self.controls.get('manual') else '急停' if self.controls.get('estop') else '等待遥控或模型就绪'
            startup=getattr(self.rt,'startup',None)
            if not self.controls.get('manual') and startup and not startup.ready:
                state='模型加载中，请稍候' if startup.state!='retrying' else '模型加载重试中'
            if self.rearm_required:state='请先返回再确认开启AI'
        return ['AI模块 总开关: ' + ('开启' if requested else '关闭'),
                'CH8选择 SWA确认/返回 SWB录音/结束',
                '功能: '+mode, '状态: '+state,
                '当前指令:'] + wrapped(self.instruction, 2) + ['识别目标:'] + wrapped(found, 2) + \
               ['聊天回复:'] + wrapped(self.reply, 5) + wrapped(self.error or '', 2)

    def fpga_reset(self):
        """Discard FPGA-dependent state without unloading the ready models."""
        if not self.enabled:return
        self.external_stop()
        self.active=self.vision=self.chat=False
        self.paused_manual=self.vision_resume_required=False
        self.rearm_required=True
        self.controls=self.last_rx=self.sequence=self.confirm_toggle=None
        self.mode='semantic_navigation';self.selected=0;self.state='off'
        self.instruction=self.reply='';self.detections=[];self.target=None;self.history=[];self.error=None
        self.display_ack=self.display_bank=self.transaction=0
        self._invalidate_display()
        self._task(self._set_vision(False))

    def link_lost(self,reason):
        if not self.enabled:return
        self.last_rx=None;self.sequence=None
        self.rearm_required=True
        self.active=self.vision=False
        self.paused_manual=False
        self.vision_resume_required=True
        self._invalidate_display()
        self._cancel(reason)
        self._task(self._set_vision(False))
        self._task(self._release_models(self.work))

    def tick(self, now):
        if not self.enabled:
            return
        if self.last_rx is not None and now-self.last_rx > self.cfg.get('control_timeout_s', .5):
            self.link_lost('ai_control_link_timeout')
        if self.vision and self.rt.camera_state=='unavailable':
            self.error='camera_unavailable';self.state='error'
        try:
            self._publish(now)
        except (OSError,RuntimeError) as exc:
            self.link_lost('ai_uart_write_failed')
            self.error='AI UART: '+str(exc)[:200]
            self.rt.gateway.close()

    def _invalidate_display(self):
        self.rt.gateway.transport.drop('ai_ui')
        self.font_caches=[{},{}]
        self.display_pending=None
        self.display.clear()
        self.display_key=None

    def _publish(self, now):
        transport = self.rt.gateway.transport
        if getattr(transport, 'fd', None) is None:
            return
        lines = self.lines()
        key = tuple(lines)
        pending=self.display_pending
        if pending and self.display_ack==pending['transaction'] and self.display_bank==pending['bank']:
            self.font_caches[pending['bank']]=pending['cache']
            self.display_key=pending['key'];self.last_display=now;self.display_pending=None
        elif pending and now>pending['deadline']:
            transport.drop('ai_ui');self.display.clear()
            self.font_caches[pending['bank']]={}
            self.display_pending=None;self.display_key=None
            self.rt.audit({'type':'ai_display_retry','transaction':pending['transaction']})
        if self.display_pending is None and (key != self.display_key or now-self.last_display > 5):
            self.transaction = (self.transaction + 1) & 255
            bank=1-self.display_bank
            frames,mapping=display_frames(lines,self.transaction,self.atlas,self.font_caches[bank])
            self.display=deque(frames)
            self.display_pending={'transaction':self.transaction,'bank':bank,'key':key,'cache':mapping,
                                  'deadline':now+3+len(frames)*12/2500}
        # Bounded batches leave the UART available for motion heartbeat and STOP.
        for _ in range(min(12, len(self.display))):
            transport.enqueue(self.display.popleft(), tag='ai_ui')
        if now-self.last_ping > .2:
            transport.enqueue(Frame(0, 0x43, 0, 0).encode(), tag='ai_ui_ping')
            self.last_ping = now
        transport.flush()

    def snapshot(self):
        return {'configured': self.enabled, 'active': self.active,
                'mode': self.mode, 'selected': self.selected,
                'paused_manual':self.paused_manual,'vision_resume_required':self.vision_resume_required,
                'vision_enabled': self.vision, 'state': self.state,
                'instruction': self.instruction, 'reply': self.reply,
                'detections': self.detections, 'last_error': self.error,
                'control_link_fresh': self.last_rx is not None and self.clock()-self.last_rx<=self.cfg.get('control_timeout_s',.5),
                'record_armed': self.record_armed}

    async def close(self):
        self.active = self.vision = False
        self.released.set()
        if self.work:
            self.work.cancel()
            await asyncio.gather(self.work, return_exceptions=True)
        for task in list(self.cleanup):
            task.cancel()
        await asyncio.gather(*list(self.cleanup), return_exceptions=True)
        await self.chat_worker.close()
        await self.tts_worker.close()
        await self.chat_server.close()
