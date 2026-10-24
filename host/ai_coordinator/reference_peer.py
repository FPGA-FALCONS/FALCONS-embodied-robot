"""FAM1 device reference and byte-stream simulator. No hardware or navigation.

Acceptance is NEVER generated automatically: the navigation adapter must report
an actual event. In tests the caller deliberately supplies simulated events.
This is an executable protocol reference for the FPGA/soft-core implementer.
"""
import uuid
from .gateway import CAPABILITIES
from .schema import PacketReader,packet,TARGETS

EVENTS={'start_search':{'search_started','region_exhausted','action_failed'},
        'target_found':{'target_accepted','action_failed'},
        'return_home':{'home_arrived','action_failed'},'cancel':{'cancelled'}}
EVENTS.update({'goto_waypoint':{'waypoint_arrived','action_failed'},'pause_patrol':{'patrol_paused','action_failed'},
               'approach_target':{'approach_arrived','action_failed'},'target_observation':set()})


class MissionPeer:
    def __init__(self):
        self.reader=PacketReader();self.boot_id=uuid.uuid4().hex;self.rx_seq=-1;self.tx_seq=0
        self.session=None;self.requests=[];self.actions={};self.cancelled_tasks=set()

    def encode(self,event):
        self.tx_seq+=1
        return packet({'v':1,'boot_id':self.boot_id,'session_id':self.session,'sequence':self.tx_seq,**event})

    def hello(self):
        return self.encode({'kind':'hello','capabilities':sorted(CAPABILITIES.values())})

    def feed(self,data):
        replies=[]
        for obj in self.reader.feed(data):
            if type(obj.get('v')) is not int or obj['v']!=1:continue
            seq=obj.get('sequence');session=obj.get('session_id');kind=obj.get('kind')
            if type(seq) is not int or seq<0 or not isinstance(session,str) or not session:continue
            if self.session is None:
                if kind!='heartbeat':continue
                self.session=session
            if session!=self.session or seq<=self.rx_seq:continue
            self.rx_seq=seq
            if kind=='heartbeat':replies.append(self.encode({'kind':'heartbeat'}));continue
            if kind not in EVENTS:continue
            if not all(isinstance(obj.get(k),str) and 0<len(obj[k])<=128 for k in ('task_id','goal_id','action_id')):continue
            if kind in ('start_search','target_found') and obj.get('target') not in TARGETS:continue
            if obj['action_id'] in self.actions:continue
            if kind!='cancel' and obj['task_id'] in self.cancelled_tasks:continue
            if kind=='cancel':self.cancelled_tasks.add(obj['task_id'])
            self.actions[obj['action_id']]=obj;self.requests.append(obj)
        return b''.join(replies)

    def report(self,action_id,kind,**evidence):
        action=self.actions.get(action_id)
        if action is None or kind not in EVENTS[action['kind']]:raise ValueError('Unexpected navigation result')
        if action['kind']!='cancel' and action['task_id'] in self.cancelled_tasks:raise ValueError('Cancelled task cannot report progress')
        return self.encode({**evidence,**{k:action[k] for k in ('task_id','goal_id','action_id')},'kind':kind})


class SimulationGateway:
    """Runtime-compatible transport exercising actual FAM1 encode/decode bytes."""
    def __init__(self,session_id,emit):
        self.session_id=session_id;self.emit=emit;self.peer=MissionPeer();self.reader=PacketReader()
        self.sequence=0;self.events=[];self.commands=[];self.wire_frames=0;self.closed=False
        self.reader.feed(self.peer.hello())
        self.send({'kind':'heartbeat'})

    def start(self):pass

    def send(self,message):
        self.sequence+=1;wire=packet({'v':1,'session_id':self.session_id,'sequence':self.sequence,**message})
        # Exercise arbitrary USB/serial fragmentation, not object passthrough.
        for offset in range(0,len(wire),7):
            response=self.peer.feed(wire[offset:offset+7])
            if response:self.reader.feed(response)
        self.wire_frames+=1

    def dispatch(self,command):
        self.send(command);self.commands.append(command)
        self.emit({'type':'gateway_command','command':command,'sent':False,'simulated_wire':True,'motion_authorized':False})

    def respond(self,action,kind,**evidence):
        wire=self.peer.report(action['action_id'],kind,**evidence)
        for offset in range(0,len(wire),11):self.events.extend(self.reader.feed(wire[offset:offset+11]))
        self.wire_frames+=1

    def poll(self,now):
        events=self.events;self.events=[];return events

    def snapshot(self):
        return {'state':'simulation','motion_authorized':False,'wire_protocol':'FAM1 simulated bytes',
                'wire_frames':self.wire_frames,'capabilities':sorted(CAPABILITIES.values())}

    def close(self):self.closed=True
