"""Asynchronous coordination of speech, language, images and FPGA mission status."""
import asyncio
from collections import OrderedDict
import json
from pathlib import Path
import time
import uuid
from .core import Mission,Policy
from .patrol import PatrolMission,PATROL_CAPABILITIES
from .gateway import Gateway
from .processes import Worker,ModelServer
from .schema import dumps,loads,task as validate_task
from .announcements import Announcements
from voice_bridge.preview import parse_task
from .command_policy import inspect as inspect_command,immediate_stop,VERSION as POLICY_VERSION
from .motion_intents import inspect_motion,VERSION as MOTION_POLICY_VERSION


class Audit:
    def __init__(self,directory):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True);self.file=None;self.index=0;self.size=0
    def __call__(self,event):
        line=dumps({'at_s':time.monotonic(),**event})+'\n'
        if self.file is None or self.size+len(line.encode())>5*1024*1024:
            if self.file:self.file.close()
            self.index+=1;self.file=(self.directory/f'events-{self.index:05d}.jsonl').open('x',encoding='utf-8');self.size=0
            # Runtime ring; separate acceptance evidence is exported before rotation.
            if self.index>4:(self.directory/f'events-{self.index-4:05d}.jsonl').unlink(missing_ok=True)
        self.file.write(line);self.file.flush();self.size+=len(line.encode())
    def close(self):
        if self.file:self.file.close();self.file=None


class Runtime:
    def __init__(self,config):
        self.config=Path(config).resolve();self.cfg=loads(self.config.read_text(encoding='utf-8'))
        self.root=Path(self.cfg['root']).resolve();self.run_dir=self.root/'runs'/uuid.uuid4().hex
        self.audit=Audit(self.run_dir);self.core=PatrolMission(Policy(**self.cfg.get('policy',{})),self.cfg.get('mode','shadow'))
        self.announcements=Announcements(self.cfg.get('announcements',{}),self.audit)
        if self.core.mode=='live' and self.cfg.get('gateway',{}).get('mode')!='serial':raise ValueError('live mode requires serial gateway')
        self.gateway=Gateway(self.cfg.get('gateway',{}),self.core.session_id,self.audit)
        self.workers={k:Worker(k,self.config,self.run_dir) for k in ('asr','vision','llm')}
        self.model_server=ModelServer(self.cfg['models']['llm'],self.run_dir)
        self.generation=0;self.jobs=OrderedDict();self.tasks=set();self.parse_lock=asyncio.Lock();self.server_lock=asyncio.Lock()
        self.ticker=None;self.last_tick=None;self.max_tick_gap=0;self.latest_image=None;self.image_pump=None;self.lock_file=None
        self.camera_task=None
        self.camera_time_verified=False
        self.last_motion=None
        self.camera_backend=self.cfg.get('camera',{}).get('backend','fx2')
        self.camera_state='disabled'
        if self.cfg.get('camera',{}).get('enabled') is True:
            self.camera_state='hold'
            if self.cfg.get('model_loads_enabled',True) is True:
                self.workers['camera']=Worker('camera',self.config,self.run_dir);self.camera_state='configured'

    async def start(self):
        if self.root.exists() and __import__('os').name=='posix':
            import fcntl
            self.lock_file=(self.root/'coordinator.lock').open('a+')
            try:fcntl.flock(self.lock_file,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BaseException:self.lock_file.close();self.lock_file=None;raise RuntimeError('Another coordinator owns this deployment')
        self.gateway.start();self.ticker=asyncio.create_task(self._tick())
        if 'camera' in self.workers:self.camera_task=asyncio.create_task(self._camera())
        self.audit({'type':'coordinator_started','run_dir':str(self.run_dir),'mode':self.core.mode})

    async def _tick(self):
        while True:
            now=time.monotonic()
            if self.last_tick is not None:self.max_tick_gap=max(self.max_tick_gap,now-self.last_tick)
            self.last_tick=now
            for event in self.gateway.poll(now):
                if event.get('kind') in ('link_lost','link_reset') and self.core.state in self.core.ACTIVE:self.core.fail(now,event['kind'])
                else:self.core.on_status(event,now)
                self.audit({'type':'fpga_event','event':event})
            self.core.tick(now);self.dispatch()
            await asyncio.sleep(.02)

    def dispatch(self):
        for command in self.core.drain():self.gateway.dispatch(command)
        for notice in self.core.drain_notices():self.announcements.emit(notice)

    def checked_path(self,value):
        if not isinstance(value,str):raise ValueError('A local file path is required')
        path=Path(value).resolve();allowed=[Path(p).resolve() for p in self.cfg.get('input_roots',[str(self.root/'samples'),str(self.root/'recordings')])]
        if not any(path==r or r in path.parents for r in allowed):raise ValueError('Input outside configured sample/recording directories')
        if not path.is_file() or path.stat().st_size>32*1024*1024:raise ValueError('Missing or oversized input file')
        return str(path)

    def _spawn(self,coroutine,kind):
        active=sum(j['status']=='running' for j in self.jobs.values())
        if active>=8:
            coroutine.close();raise RuntimeError('Coordinator input queue full')
        identifier=uuid.uuid4().hex;entry={'id':identifier,'kind':kind,'status':'running','submitted_at_s':time.monotonic()}
        self.jobs[identifier]=entry
        while len(self.jobs)>64:
            oldest=next((key for key,value in self.jobs.items() if value['status']!='running'),None)
            if oldest is None:break
            self.jobs.pop(oldest)
        async def work():
            try:entry.update(status='done',result=await coroutine)
            except asyncio.CancelledError:entry.update(status='cancelled');raise
            except Exception as exc:entry.update(status='error',error=type(exc).__name__+': '+str(exc)[:800])
            finally:
                entry['finished_at_s']=time.monotonic();self.audit({'type':'job_finished','job':entry})
        pending=asyncio.create_task(work());self.tasks.add(pending);pending.add_done_callback(self.tasks.discard)
        return identifier

    def check_admission(self):
        if sum(j['status']=='running' for j in self.jobs.values())>=8:raise RuntimeError('Coordinator input queue full')

    async def _ensure_server(self):
        async with self.server_lock:
            if self.model_server.proc is None or self.model_server.proc.returncode is not None:
                try:await self.model_server.start()
                except BaseException:await self.model_server.close();raise

    async def _infer(self,text):
        if self.cfg.get('model_loads_enabled',True) is not True:raise RuntimeError('Model loads HOLD; inspect board stability before enabling')
        async def infer():
            async with self.parse_lock:
                await self._ensure_server()
                return await self.workers['llm'].call({'text':text},timeout=self.cfg.get('llm_timeout_s',15))
        try:return await asyncio.wait_for(infer(),self.cfg.get('llm_timeout_s',15))
        except BaseException:
            await self.workers['llm'].close();await self.model_server.close();raise

    async def parse(self,text,generation,plan_only=False):
        if not isinstance(text,str) or not 1<=len(text)<=1000:raise ValueError('Text must be 1..1000 characters')
        if generation==self.generation:
            control=self.preset_control(text,plan_only)
            if control is not None:return {'raw_text':text,'preset_control':control,'stale':False,'plan_only':plan_only}
        inspected=inspect_command(text);policy=inspected['domain'];answer=None
        if inspected['hard_task'] is not None:
            answer={'task':inspected['hard_task'],'valid':True,'source':inspected['source']}
        else:
            raw=(await self._infer(policy['task_text']))['result']
            answer={'raw':raw,'task':{'i':'unknown'},'valid':False,'source':'language_model'}
            try:
                if raw.get('finish_reason')!='stop' or raw.get('reasoning'):raise ValueError('incomplete or thinking output')
                validate_task(loads(raw['content']));parsed=parse_task(raw['content'],policy['task_text'])
                if parsed['i']=='stop':raise ValueError('stop lacks explicit immediate command evidence')
                answer.update(task=parsed,valid=True)
            except (ValueError,TypeError) as exc:answer['error']=str(exc)
        stale=generation!=self.generation
        result={'raw_text':text,'task_text':policy['task_text'],'domain_vocab':policy,'policy_version':POLICY_VERSION,
                'parser':answer,'stale':stale,'plan_only':plan_only}
        if not stale and not plan_only:
            needed={'search_v1','target_accept_v1','cancel_v1'}|({'return_home_v1'} if answer['task'].get('r') else set())
            if answer['task']['i']=='search' and self.core.mode=='live' and (not self.gateway.snapshot()['motion_authorized'] or not self.camera_time_verified or not needed<=self.gateway.capabilities):
                result['admission']={'accepted':False,'reason':'live_camera_time_or_fpga_not_commissioned'}
            else:result['admission']=self.core.submit(answer['task'],time.monotonic());self.dispatch()
        self.audit({'type':'parsed_instruction',**result});return result

    def submit_text(self,text,plan_only=False):
        if not isinstance(text,str) or not 1<=len(text)<=1000:raise ValueError('Text must be 1..1000 characters')
        # Stop admission is synchronous and independent of a busy model.
        if immediate_stop(text) and not plan_only:
            return self.stop('voice_exact_stop')
        control=self.preset_control(text,plan_only)
        if control is not None:return control
        self.check_admission()
        self.generation+=1;return {'job_id':self._spawn(self.parse(text,self.generation,plan_only),'text')}

    def preset_control(self,text,plan_only=False):
        operation={'拾取完成':'pickup-confirmed','物品已经拿走':'pickup-confirmed'}.get(text.strip().rstrip('。！!'))
        if operation:
            if plan_only:return {'operation':operation,'preview_only':True,'executed':False}
            return self.confirm_pickup({'task_id':self.core.task_id})
        decision=inspect_motion(text)
        if decision is None:return None
        intent=decision.get('intent',{});cmd=intent.get('cmd')
        if decision['valid'] and cmd in ('stop','patrol_stop'):
            if plan_only:return {'operation':'stop','preview_only':True,'executed':False,'motion':decision}
            return self.stop('voice_motion_stop')
        # Existing AI find-object patrol keeps its configured route and camera
        # admission. Camera-independent patrol is an explicit FPGA preset choice.
        cfg=self.cfg.get('voice_motion',{})
        if decision['valid'] and cmd=='patrol_start' and cfg.get('patrol_target','ai_search')=='ai_search':
            if plan_only:return {'operation':'patrol','preview_only':True,'executed':False,'motion':decision}
            if self.core.state in self.core.ACTIVE:raise RuntimeError('A mission is already active; stop before restarting the configured route')
            return self.start_patrol({})
        if plan_only:reason='plan_only'
        elif not decision['valid']:reason=decision['reason']
        elif cfg.get('enabled') is not True:reason='voice_motion_disabled'
        elif not decision.get('wire_preview'):reason=decision['reason']
        elif self.core.state in self.core.ACTIVE:reason='existing_mission_owns_control'
        elif self.cfg.get('model_loads_enabled',True) is not True:reason='model_and_motion_hold'
        elif self.core.mode!='live':reason='shadow_only'
        else:reason='motion_firmware_not_commissioned'
        result={'operation':'motion','raw_text':text,'motion':decision,'policy_version':MOTION_POLICY_VERSION,
                'preview_only':True,'plan_only':plan_only,'accepted':False,'sent':False,'executed':False,
                'motion_authorized':False,'physical_task_completed':False,'reason':reason}
        if not plan_only:
            self.generation+=1;self.last_motion=result
        self.audit({'type':'motion_instruction',**result});return result

    def start_patrol(self,request):
        preset=self.cfg.get('patrol',{})
        if preset.get('enabled') is not True:raise RuntimeError('Patrol preset not configured')
        if request:raise ValueError('Patrol uses configured waypoint IDs and targets; request body must be empty')
        if self.core.mode=='live' and (not self.camera_time_verified or not self.gateway.snapshot()['motion_authorized'] or
            not (set(PATROL_CAPABILITIES.values())|{'cancel_v1'})<=self.gateway.capabilities):raise RuntimeError('Live patrol requires commissioned camera and FPGA waypoint/local approach/cancel capabilities')
        self.generation+=1;self.core.start_patrol(preset['waypoints'],preset['targets'],preset.get('cycles',1),time.monotonic(),preset.get('stop_band_m'));self.dispatch()
        return {'accepted':True,'mission':self.core.snapshot(),'physical_pickup_verified':False}

    def confirm_pickup(self,request):
        if set(request)!={'task_id'}:raise ValueError('Pickup confirmation requires exactly the current task_id')
        result=self.core.confirm_pickup(request.get('task_id'),time.monotonic());self.dispatch();return result

    def submit_audio(self,request):
        if self.cfg.get('model_loads_enabled',True) is not True:raise RuntimeError('Model loads HOLD; audio inference disabled')
        payload={}
        if request.get('capture'):
            cfg=self.cfg.get('microphone',{})
            if not cfg.get('device'):raise RuntimeError('USB microphone not configured / pending hardware arrival')
            if self.announcements.microphone_blocked:raise RuntimeError('Announcement playing; microphone temporarily paused')
            payload={'capture':dict(cfg)}
        else:payload={'wav':self.checked_path(request.get('wav'))}
        plan_only=request.get('plan_only',False)
        if type(plan_only) is not bool:raise ValueError('plan_only must be boolean')
        self.check_admission();self.generation+=1;generation=self.generation
        async def work():
            capturing=bool(request.get('capture'))
            if capturing:self.announcements.begin_capture()
            try:response=await self.workers['asr'].call(payload,timeout=self.cfg.get('asr_timeout_s',15));asr=response['result']
            finally:
                if capturing:self.announcements.end_capture()
            if generation!=self.generation:return {'asr':asr,'stale':True}
            if asr.get('status')!='recognized':return {'asr':asr,'accepted':False}
            return {'asr':asr,'asr_elapsed_s':response['elapsed_s'],'parsed':await self.parse(asr['text'],generation,plan_only)}
        return {'job_id':self._spawn(work(),'audio')}

    def submit_image(self,request):
        if self.cfg.get('model_loads_enabled',True) is not True:raise RuntimeError('Model loads HOLD; image inference disabled')
        path=self.checked_path(request.get('image'));now=time.monotonic()
        # File ingress is always replay; callers cannot assert camera freshness.
        tag={'task_id':self.core.task_id,'goal_id':self.core.goal_id,'captured_at_s':now,'replay':True,
             'source_timestamp_verified':False}
        return self.enqueue_image({'image':path},tag)

    def enqueue_image(self,payload,tag):
        future=asyncio.get_running_loop().create_future()
        async def wait():return await future
        identifier=self._spawn(wait(),'image')
        if self.latest_image is not None:
            old=self.latest_image[2]
            if not old.done():old.set_result({'dropped':True,'reason':'superseded_by_newer_frame'})
            self._clean_image(self.latest_image[0])
        self.latest_image=(payload,tag,future)
        if self.image_pump is None or self.image_pump.done():
            self.image_pump=asyncio.create_task(self._images());self.tasks.add(self.image_pump)
            self.image_pump.add_done_callback(self.tasks.discard)
        return {'job_id':identifier}

    async def _images(self):
        while self.latest_image is not None:
            payload,tag,future=self.latest_image;self.latest_image=None
            try:
                response=await self.workers['vision'].call(payload,timeout=self.cfg.get('vision_timeout_s',10))
                event={**tag,**response['result'],'sequence':time.monotonic_ns()}
                if tag.get('source_timestamp_verified') is True and tag.get('replay') is False:self.camera_time_verified=True
                accepted=self.core.on_frame(event,time.monotonic());self.dispatch()
                if not future.done():future.set_result({'frame':event,'admitted_to_mission':accepted,'elapsed_s':response['elapsed_s']})
            except asyncio.CancelledError:
                if not future.done():future.cancel()
                raise
            except Exception as exc:
                if not future.done():future.set_exception(exc)
            finally:self._clean_image(payload)

    def _clean_image(self,payload):
        if payload.get('_temporary') is True:
            path=Path(payload['image']).resolve()
            if path.parent==(self.root/'camera_frames').resolve():path.unlink(missing_ok=True)

    async def _camera(self):
        try:
            self.camera_state='starting'
            while True:
                if self.cfg.get('model_loads_enabled',True) is not True:
                    self.camera_state='hold';return
                response=await self.workers['camera'].call({'op':'next_frame'},timeout=5);frame=response['result']
                if frame.get('status')=='frame':
                    self.camera_state='streaming'
                    payload={k:frame[k] for k in ('image','format','width','height','byteorder')};payload['_temporary']=True
                    tag={'task_id':self.core.task_id,'goal_id':self.core.goal_id,'captured_at_s':frame['received_at_s'],
                         'replay':False,'source_timestamp_verified':False,'timestamp_origin':'host_receipt_only',
                         'source_sequence':frame['sequence'],'camera_backend':self.camera_backend,
                         'sequence_origin':frame.get('sequence_origin','transport_sequence')}
                    self.enqueue_image(payload,tag)
                else:await asyncio.sleep(.02)
        except asyncio.CancelledError:raise
        except Exception as exc:
            self.camera_state='unavailable';self.audit({'type':'camera_unavailable','error':str(exc)[:500]})

    def stop(self,reason='operator_stop'):
        self.generation+=1;self.core.stop(time.monotonic(),reason);self.dispatch()
        # Do not wait for slow cancellation to enqueue the FPGA cancellation.
        try:current=asyncio.current_task()
        except RuntimeError:current=None
        for pending in list(self.tasks):
            if pending is not current:pending.cancel()
        async def cleanup():
            await self.workers['llm'].close();await self.model_server.close()
        task=asyncio.create_task(cleanup());self.tasks.add(task);task.add_done_callback(self.tasks.discard)
        return {'status':'stopped','motion_authorized':False,'mission':self.core.snapshot()}

    def warmup(self):
        if self.cfg.get('model_loads_enabled',True) is not True:raise RuntimeError('Model loads HOLD; warmup disabled')
        async def work():
            await self._ensure_server()
            results={}
            for key in ('asr','vision','llm'):
                response=await self.workers[key].call({'op':'health'},timeout=60)
                results[key]={'load_s':self.workers[key].load_s,'health':response['result']}
            return results
        return {'job_id':self._spawn(work(),'warmup')}

    def snapshot(self):
        gateway=self.gateway.snapshot();mission=self.core.snapshot();mission['motion_authorized']=gateway['motion_authorized']
        return {'status':'running','mode':self.core.mode,'mission':mission,'gateway':gateway,
            'workers':{k:{'pid':w.proc.pid if w.proc else None,'load_s':w.load_s} for k,w in self.workers.items()},
            'pending_jobs':sum(x['status']=='running' for x in self.jobs.values()),'max_tick_gap_s':self.max_tick_gap,
            'run_dir':str(self.run_dir),'microphone_configured':bool(self.cfg.get('microphone',{}).get('device')),
            'camera_configured':bool(self.cfg.get('camera',{}).get('enabled',False)),
            'camera_backend':self.camera_backend,
            'camera_state':self.camera_state,
            'fpga_camera_configured':self.camera_backend=='fx2' and bool(self.cfg.get('camera',{}).get('enabled',False)),
            'camera_source_time_verified':self.camera_time_verified,'announcements':self.announcements.snapshot(),
            'voice_motion':{'enabled':self.cfg.get('voice_motion',{}).get('enabled',False),
                            'policy_version':MOTION_POLICY_VERSION,'transport':'preview_only',
                            'live_commissioned':False,'last_instruction':self.last_motion},
            'policy_version':POLICY_VERSION,'language_profile':self.cfg.get('language_profile','unspecified'),
            'model_loads_enabled':self.cfg.get('model_loads_enabled',True)}

    async def close(self):
        if self.camera_task:self.camera_task.cancel()
        for task in list(self.tasks):task.cancel()
        if self.ticker:self.ticker.cancel()
        await asyncio.gather(*list(self.tasks),*([self.ticker] if self.ticker else []),*([self.camera_task] if self.camera_task else []),return_exceptions=True)
        if self.latest_image:self._clean_image(self.latest_image[0]);self.latest_image=None
        await self.announcements.close()
        for worker in self.workers.values():await worker.close()
        await self.model_server.close();self.gateway.close()
        self.audit({'type':'coordinator_stopped'});self.audit.close()
        if self.lock_file:self.lock_file.close();self.lock_file=None
