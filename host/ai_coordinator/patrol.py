"""Waypoint patrol interrupted by object inspection; navigation stays on FPGA.

No bbox-to-distance conversion or motor commands. Approach success requires a
metric stop and stationary receipt from the execution layer. Pickup in this
version is an explicit operator confirmation, not a claimed gripper action.
"""
import uuid
from .core import Mission
from .schema import TARGETS,detection,number

PATROL_CAPABILITIES={'goto_waypoint':'waypoint_v1','pause_patrol':'pause_patrol_v1',
                     'approach_target':'local_approach_v1','target_observation':'local_approach_v1'}


def overlap(a,b):
    w=max(0,min(a[2],b[2])-max(a[0],b[0]));h=max(0,min(a[3],b[3])-max(a[1],b[1]))
    inter=w*h;union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union>0 else 0


class PatrolMission(Mission):
    PATROL_ACTIVE={'patrolling','pausing_patrol','approaching','waiting_pickup'}
    ACTIVE=Mission.ACTIVE|PATROL_ACTIVE

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.patrol_enabled=False;self.route=[];self.route_index=0
        self.allowed=[];self.cycles=1;self.lap=0;self.locked=None;self.last_observation=None;self.last_sent=0
        self.suppressed=None;self.absent_frames=0;self.operator_confirmations=0
        self.stop_band=None

    def submit(self,spec,now):
        if spec.get('i') in ('search','stop'):self.patrol_enabled=False
        return super().submit(spec,now)

    def start_patrol(self,route,targets,cycles,now,stop_band=None):
        if not isinstance(route,list) or not 1<=len(route)<=32 or not all(isinstance(x,str) and 0<len(x)<=64 for x in route):raise ValueError('Expected 1..32 configured waypoint IDs')
        if not isinstance(targets,list) or not targets or len(set(targets))!=len(targets) or any(x not in TARGETS for x in targets):raise ValueError('Invalid inspection target classes')
        if type(cycles) is not int or not 1<=cycles<=100:raise ValueError('Invalid patrol cycle count')
        if stop_band is not None and (not isinstance(stop_band,list) or len(stop_band)!=2 or not all(number(x) for x in stop_band) or not 0<stop_band[0]<stop_band[1]):raise ValueError('Invalid metric stop band')
        if self.mode=='live' and stop_band is None:raise ValueError('Measured vehicle stop band required for live approach')
        if self.state in self.ACTIVE:self._action('cancel',now,reason='superseded')
        self.task_id=uuid.uuid4().hex;self.route=list(route);self.route_index=0;self.allowed=list(targets)
        self.cycles=cycles;self.lap=0;self.patrol_enabled=True;self.spec={'i':'patrol','targets':targets,'pickup':'operator_confirmation'}
        self.stop_band=stop_band
        self.trace=[];self.reason=None;self.suppressed=None;self.operator_confirmations=0
        self.frames_seen=0
        self._waypoint(now);self._notice('patrol_started')

    def _waypoint(self,now):
        self.goal_id=uuid.uuid4().hex;self.target=None;self.locked=None;self.streak=0
        self.started_at=now;self.last_frame_seq=-1;self.last_captured=None;self.last_observation=None;self.last_match_at=None
        self.pending=self._action('goto_waypoint',now,waypoint=self.route[self.route_index],route_index=self.route_index)
        self.deadline=now+self.policy.search_timeout_s;self._state('patrolling')

    def on_frame(self,event,now):
        if not self.patrol_enabled:return super().on_frame(event,now)
        if self.state not in ('patrolling','approaching') or not isinstance(event,dict):return False
        if event.get('task_id')!=self.task_id or event.get('goal_id')!=self.goal_id:return False
        seq=event.get('sequence');captured=event.get('captured_at_s');w=event.get('width');h=event.get('height')
        if type(seq) is not int or seq<=self.last_frame_seq or not number(captured):return False
        if captured<self.started_at or captured>now+.05 or now-captured>self.policy.frame_max_age_s:return False
        if self.last_captured is not None and captured<=self.last_captured:return False
        if type(w) is not int or type(h) is not int or not 1<=w<=8192 or not 1<=h<=8192:return False
        if self.mode=='live' and (event.get('replay') is not False or event.get('source_timestamp_verified') is not True):return False
        detections=event.get('detections')
        if not isinstance(detections,list) or len(detections)>1000:return False
        self.last_frame_seq=seq;self.last_captured=captured;self.frames_seen+=1
        matches=[d for d in detections if detection(d,w,h) and d['class'] in self.allowed]
        if self.suppressed:
            self.absent_frames=0 if any(d['class']==self.suppressed for d in matches) else self.absent_frames+1
            if self.absent_frames>=2:self.suppressed=None
            matches=[d for d in matches if d['class']!=self.suppressed]
        if self.locked:
            matches=[d for d in matches if d['class']==self.locked['class'] and overlap(d['bbox'],self.locked['bbox'])>=.2]
        if not matches:
            self.streak=0
            if self.state=='patrolling':self.locked=None;self.last_match_at=None
            return True
        best=max(matches,key=lambda x:x['confidence'])
        self.streak=self.streak+1 if self.last_match_at is not None and captured-self.last_match_at<=self.policy.confirm_gap_s else 1
        self.locked=dict(best);self.target=best['class'];self.last_match_at=captured
        self.last_observation={'target':self.target,'detection':dict(best),'width':w,'height':h,
                               'captured_at_s':captured,'frame_sequence':seq}
        if self.state=='patrolling' and self.streak>=self.policy.confirm_frames:
            self.pending=self._action('pause_patrol',now,waypoint=self.route[self.route_index])
            self.deadline=now+self.policy.ack_timeout_s;self._state('pausing_patrol')
        elif self.state=='approaching' and now-self.last_sent>=.1:
            self._action('target_observation',now,approach_action_id=self.pending['action_id'],**self.last_observation)
            self.last_sent=now
        return True

    def on_status(self,event,now):
        if not self.patrol_enabled:return super().on_status(event,now)
        if not isinstance(event,dict) or type(event.get('v')) is not int or event['v']!=1:return False
        if any(event.get(k)!=getattr(self,k) for k in ('session_id','task_id','goal_id')):return False
        if not self.pending or event.get('action_id')!=self.pending['action_id']:return False
        kind=event.get('kind')
        if kind=='action_failed':self.fail(now,'fpga_action_failed');return True
        if kind=='waypoint_arrived' and self.state=='patrolling':
            self.route_index+=1
            if self.route_index==len(self.route):self.route_index=0;self.lap+=1
            if self.lap>=self.cycles:self.pending=None;self.deadline=None;self._state('completed')
            else:self._waypoint(now)
            return True
        if kind=='patrol_paused' and self.state=='pausing_patrol':
            if not self.last_observation or now-self.last_match_at>self.policy.frame_max_age_s:self.fail(now,'stale_approach_target');return True
            self._notice('patrol_paused')
            self.pending=self._action('approach_target',now,requires_metric_stop=True,stop_band_m=self.stop_band,
                                      observation_ttl_ms=int(self.policy.frame_max_age_s*1000),**self.last_observation)
            self.deadline=now+self.policy.return_timeout_s;self._state('approaching');self._notice('approaching');return True
        if kind=='approach_arrived' and self.state=='approaching':
            # Image size alone is never sufficient proof of a safe stand-off.
            clearance=event.get('clearance_m')
            if event.get('metric_stop_verified') is not True or event.get('stationary') is not True or not number(clearance) or clearance<=0:return False
            if self.stop_band is None or not self.stop_band[0]<=clearance<=self.stop_band[1]:return False
            if self.last_match_at is None or now-self.last_match_at>self.policy.frame_max_age_s:return False
            self._notice('found_'+self.target)
            self._state('waiting_pickup');self.deadline=now+60;self._notice('pickup_wait');return True
        return False

    def confirm_pickup(self,task_id,now):
        if not self.patrol_enabled or self.state!='waiting_pickup' or task_id!=self.task_id:raise ValueError('No matching stopped pickup task')
        self.operator_confirmations+=1;self.suppressed=self.target;self.absent_frames=0
        self._notice('pickup_confirmed');self._waypoint(now);self._notice('patrol_resumed')
        return {'accepted':True,'source':'operator_confirmation','physical_pickup_verified':False}

    def tick(self,now):
        if self.patrol_enabled and self.state=='approaching' and (self.last_match_at is None or now-self.last_match_at>self.policy.frame_max_age_s):
            self.fail(now,'target_lost_during_approach');return
        super().tick(now)

    def snapshot(self):
        result=super().snapshot()
        if self.patrol_enabled:result['patrol']={'route':self.route,'route_index':self.route_index,'lap':self.lap,
            'cycles':self.cycles,'targets':self.allowed,'operator_confirmations':self.operator_confirmations,
            'pickup_mode':'operator_confirmation','physical_pickup_verified':False,'target_map_position_required':False}
        return result
