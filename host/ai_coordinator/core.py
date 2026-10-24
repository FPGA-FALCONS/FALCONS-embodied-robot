"""Deterministic mission state machine. Models and devices cannot block this object."""
from dataclasses import dataclass
import uuid
from .schema import task as validate_task,detection,number


@dataclass(frozen=True)
class Policy:
    ack_timeout_s:float=5
    search_timeout_s:float=60
    return_timeout_s:float=60
    frame_max_age_s:float=1
    confirm_frames:int=2
    confirm_gap_s:float=1

    def __post_init__(self):
        for key in ('ack_timeout_s','search_timeout_s','return_timeout_s','frame_max_age_s','confirm_gap_s'):
            if not number(getattr(self,key)) or getattr(self,key)<=0:raise ValueError('invalid deadline policy')
        if type(self.confirm_frames) is not int or not 1<=self.confirm_frames<=10:raise ValueError('invalid confirmation count')


class Mission:
    ACTIVE={'waiting_start','searching','waiting_found_ack','returning'}

    def __init__(self,policy=None,mode='shadow'):
        if mode not in ('shadow','simulation','live'):raise ValueError('invalid mode')
        self.policy=policy or Policy();self.mode=mode;self.session_id=uuid.uuid4().hex
        self.task_id=None;self.goal_id=None;self.state='idle';self.reason=None;self.spec=None
        self.target=None;self.branch=None;self.pending=None;self.search_action_id=None
        self.deadline=None;self.started_at=None;self.last_frame_seq=-1;self.last_captured=None
        self.last_match_at=None;self.streak=0;self.frames_seen=0;self.trace=[];self.actions=[]
        self.notices=[]

    def _notice(self,code):
        self.notices.append({'id':uuid.uuid4().hex,'task_id':self.task_id,'goal_id':self.goal_id,
                             'code':code,'simulated':self.mode!='live'})

    def drain_notices(self):
        out=self.notices;self.notices=[];return out

    def _state(self,state,reason=None):
        self.state=state;self.reason=reason;self.trace.append(state)
        if state=='searching':self._notice('search_'+self.target)
        elif state in ('returning','completed'):self._notice(state)
        elif state=='stopped':self._notice('stop_requested')
        elif state=='failed':self._notice({'action_deadline':'timed_out','search_region_exhausted':'not_found'}.get(reason,'failed'))

    def _action(self,kind,now,**fields):
        msg={'v':1,'kind':kind,'session_id':self.session_id,'task_id':self.task_id,
             'goal_id':self.goal_id,'action_id':uuid.uuid4().hex,'created_at_s':now,**fields}
        self.actions.append(msg);return msg

    def drain(self):
        out=self.actions;self.actions=[];return out

    def _start(self,target,branch,now):
        self.target=target;self.branch=branch;self.goal_id=uuid.uuid4().hex
        self.last_frame_seq=-1;self.last_captured=None;self.last_match_at=None;self.streak=0;self.frames_seen=0
        self.pending=self._action('start_search',now,target=target,branch=branch)
        self.search_action_id=self.pending['action_id'];self.started_at=now
        self.deadline=now+self.policy.ack_timeout_s;self._state('waiting_start')

    def submit(self,spec,now):
        spec=validate_task(spec)
        if spec['i']=='stop':self.stop(now);return
        if spec['i'] in ('unknown','confirm'):
            # An invalid/new question must not silently cancel an active mission.
            return {'status':'needs_clarification' if spec['i']=='unknown' else 'confirmation_only',
                    'active_task_unchanged':self.state in self.ACTIVE}
        if self.state in self.ACTIVE:self._action('cancel',now,reason='superseded')
        self.task_id=uuid.uuid4().hex;self.spec=spec;self.trace=[];self.reason=None
        self._start(spec['t'],'primary',now)
        self._notice('accepted_'+spec['t'])

    def stop(self,now,reason='explicit_stop'):
        if self.task_id:self._action('cancel',now,reason=reason)
        self.pending=None;self.deadline=None;self._state('stopped',reason)

    def fail(self,now,reason):
        if self.state in self.ACTIVE:self._action('cancel',now,reason=reason)
        self.pending=None;self.deadline=None;self._state('failed',reason)

    def on_status(self,event,now):
        if not isinstance(event,dict) or type(event.get('v')) is not int or event['v']!=1:return False
        if event.get('session_id')!=self.session_id or event.get('task_id')!=self.task_id or event.get('goal_id')!=self.goal_id:return False
        kind=event.get('kind');action_id=event.get('action_id')
        if kind=='search_started' and self.state=='waiting_start' and action_id==self.search_action_id:
            self.deadline=now+self.policy.search_timeout_s;self.started_at=now;self._state('searching');return True
        if kind=='region_exhausted' and self.state=='searching' and action_id==self.search_action_id:
            # Exhaustion comes from the navigation/search primitive, never an empty image.
            if not self.frames_seen or self.last_captured is None or now-self.last_captured>self.policy.frame_max_age_s:
                self.fail(now,'search_finished_without_fresh_vision');return True
            if self.branch=='primary' and 'f' in self.spec:self._start(self.spec['f'],'fallback',now)
            else:self.fail(now,'search_region_exhausted')
            return True
        if kind=='target_accepted' and self.state=='waiting_found_ack' and self.pending and action_id==self.pending['action_id']:
            self._notice('found_'+self.target)
            if self.spec.get('r'):
                self.pending=self._action('return_home',now);self.deadline=now+self.policy.return_timeout_s;self._state('returning')
            else:self.pending=None;self.deadline=None;self._state('completed')
            return True
        if kind=='home_arrived' and self.state=='returning' and self.pending and action_id==self.pending['action_id']:
            self.pending=None;self.deadline=None;self._state('completed');return True
        if kind=='action_failed' and self.state in self.ACTIVE and self.pending and action_id==self.pending['action_id']:
            self.fail(now,'fpga_action_failed');return True
        return False

    def on_frame(self,event,now):
        if self.state!='searching' or not isinstance(event,dict):return False
        if event.get('task_id')!=self.task_id or event.get('goal_id')!=self.goal_id:return False
        seq=event.get('sequence');captured=event.get('captured_at_s');width=event.get('width');height=event.get('height')
        if type(seq) is not int or seq<=self.last_frame_seq or not number(captured):return False
        if captured<self.started_at or captured>now+.05 or now-captured>self.policy.frame_max_age_s:return False
        if self.last_captured is not None and captured<=self.last_captured:return False
        if type(width) is not int or type(height) is not int or not 1<=width<=8192 or not 1<=height<=8192:return False
        if self.mode=='live' and (event.get('replay') is not False or event.get('source_timestamp_verified') is not True):return False
        detections=event.get('detections')
        if not isinstance(detections,list) or len(detections)>1000:return False
        self.last_frame_seq=seq;self.last_captured=captured;self.frames_seen+=1
        matches=[d for d in detections if detection(d,width,height) and d['class']==self.target]
        if not matches:self.streak=0;self.last_match_at=None;return True
        self.streak=self.streak+1 if self.last_match_at is not None and captured-self.last_match_at<=self.policy.confirm_gap_s else 1
        self.last_match_at=captured
        if self.streak>=self.policy.confirm_frames:
            best=max(matches,key=lambda d:d['confidence'])
            self.pending=self._action('target_found',now,target=self.target,frame_sequence=seq,
                captured_at_s=captured,width=width,height=height,detection=best)
            self.deadline=now+self.policy.ack_timeout_s;self._state('waiting_found_ack')
        return True

    def tick(self,now):
        if self.state in self.ACTIVE and self.deadline is not None and now>=self.deadline:self.fail(now,'action_deadline')

    def snapshot(self):
        return {'session_id':self.session_id,'task_id':self.task_id,'goal_id':self.goal_id,'state':self.state,
            'task':self.spec,'target':self.target,'branch':self.branch,'reason':self.reason,'trace':list(self.trace),
            'frames_seen':self.frames_seen,'simulation_only':self.mode=='simulation',
            'motion_authorized':False,'physical_task_completed':self.mode=='live' and self.state=='completed'}
