"""Bounded clock exchange and interpolation; receipt time never becomes pose time."""
from collections import deque
import math
import secrets
from .calibration import finite


def delta32(a,b):
    value=(a-b)&0xffffffff
    return value-0x100000000 if value&0x80000000 else value


class ClockSync:
    def __init__(self,session,epoch):
        self.session,self.epoch=session,epoch
        self.pending={};self.samples=deque(maxlen=16);self.reference=None
    def request(self,now):
        if not finite(now):raise ValueError('Finite clock time required')
        self.pending={k:v for k,v in self.pending.items() if 0<=now-v<2}
        if len(self.pending)>=4:return None
        nonce=secrets.randbits(32) or 1
        self.pending[nonce]=now
        return [self.session,self.epoch,nonce]
    def reply(self,words,now):
        if len(words)!=5 or words[:2]!=[self.session,self.epoch] or words[2] not in self.pending:return False
        t1=self.pending.pop(words[2]);t2raw,t3raw=words[3:]
        if not 0<=t2raw<=0xffffffff or not 0<=t3raw<=0xffffffff:return False
        processing=delta32(t3raw,t2raw)/1e6;rtt=now-t1
        if not finite(now) or not 0<=processing<=rtt<=.25:return False
        if self.reference is None:t2=t2raw/1e6
        else:t2=self.reference[1]+delta32(t2raw,self.reference[0])/1e6
        t3=t2+processing
        lower,upper=t3-now,t2-t1
        if upper<lower:return False
        self.reference=(t3raw,t3)
        self.samples.append((now,(lower+upper)/2,(upper-lower)/2+1e-6))
        return True
    def estimate(self,now):
        candidates=[(error+(now-at)*.0002,offset) for at,offset,error in self.samples if 0<=now-at<=3]
        if not candidates:raise ValueError('Clock synchronization stale or absent')
        error,offset=min(candidates)
        if error>.02:raise ValueError('Clock uncertainty exceeds20ms')
        return offset,error
    def to_host(self,stamp,now):
        if self.reference is None or type(stamp) is not int or not 0<=stamp<=0xffffffff:raise ValueError('Valid device clock required')
        offset,error=self.estimate(now)
        raw,extended=self.reference
        return extended+delta32(stamp,raw)/1e6-offset,error


class PoseBuffer:
    def __init__(self,epoch):self.epoch=epoch;self.items=deque(maxlen=256)
    def add(self,words):
        if len(words)!=7 or words[0]!=self.epoch or words[6]!=1:return False
        def signed(v):return v-0x100000000 if v&0x80000000 else v
        epoch,seq,stamp,x,y,yaw,valid=words
        if self.items and delta32(seq,self.items[-1][0])<=0:return False
        pose={'epoch':epoch,'x_m':signed(x)/1000,'y_m':signed(y)/1000,'yaw_rad':math.radians(signed(yaw)/100)}
        if not all(-6.4<=pose[k]<6.4 for k in ('x_m','y_m')):return False
        self.items.append((seq,stamp,pose));return True
    def at(self,when,clock,now):
        if not finite(when) or not 0<=now-when<=.5:raise ValueError('Capture stale')
        samples=[]
        for seq,stamp,pose in self.items:
            at,error=clock.to_host(stamp,now);samples.append((at,error,pose))
        for left,right in zip(samples,samples[1:]):
            a,ea,pa=left;b,eb,pb=right
            if a<=when<=b and 0<b-a<=.2:
                weight=(when-a)/(b-a)
                yaw=pa['yaw_rad']+weight*((pb['yaw_rad']-pa['yaw_rad']+math.pi)%(2*math.pi)-math.pi)
                return {**pa,'x_m':pa['x_m']+(pb['x_m']-pa['x_m'])*weight,
                        'y_m':pa['y_m']+(pb['y_m']-pa['y_m'])*weight,'yaw_rad':yaw},max(ea,eb)
        raise ValueError('No bracketing device poses; extrapolation forbidden')


def capture_time(frame,profile,camera_serial):
    if (profile.get('measured') is not True or not camera_serial or profile.get('camera_serial')!=camera_serial
            or profile.get('image_size')!=[frame.get('width'),frame.get('height')]
            or profile.get('mode')!=frame.get('negotiated') or frame.get('timestamp_origin')!='v4l2_driver'
            or frame.get('timestamp_clock')!='monotonic' or profile.get('timestamp_source')!=frame.get('timestamp_source')):
        raise ValueError('Camera timing profile does not match this source/mode')
    bias,error=profile.get('driver_to_exposure_s'),profile.get('error_bound_s')
    if not finite(bias) or not -.5<=bias<=.5 or not finite(error) or not 0<error<=.02:
        raise ValueError('Measured exposure offset and error bound required')
    stamp=frame.get('driver_timestamp_s');received=frame.get('received_at_s')
    if not finite(stamp) or not finite(received) or not 0<=received-stamp<=1:
        raise ValueError('Driver timestamp is not a recent monotonic sample')
    return stamp+bias,error
