"""Single mission gateway. Missing/legacy FPGA firmware stays read-only."""
import asyncio
import os
import time
from .schema import PacketReader,packet

CAPABILITIES={'start_search':'search_v1','target_found':'target_accept_v1','return_home':'return_home_v1','cancel':'cancel_v1'}
CAPABILITIES.update({'goto_waypoint':'waypoint_v1','pause_patrol':'pause_patrol_v1',
                     'approach_target':'local_approach_v1','target_observation':'local_approach_v1'})


class Gateway:
    def __init__(self,cfg,session_id,emit):
        self.cfg=cfg;self.session_id=session_id;self.emit=emit;self.fd=None;self.reader=PacketReader()
        self.capabilities=set();self.peer_boot=None;self.last_rx=None;self.rx_seq=-1;self.tx_seq=0
        self.last_heartbeat=0;self.pending=bytearray();self.state='shadow' if cfg.get('mode','shadow')=='shadow' else 'disconnected'

    def start(self):
        if self.cfg.get('mode','shadow')=='shadow':return
        if os.name!='posix':raise RuntimeError('Serial gateway requires Linux')
        import termios
        import fcntl
        device=self.cfg.get('device','')
        if not device.startswith('/dev/serial/by-id/'):raise ValueError('Select an explicit /dev/serial/by-id device')
        # No DTR toggle, reset, firmware upload or competing serial owner.
        self.fd=os.open(device,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            attrs=termios.tcgetattr(self.fd);attrs[0]=0;attrs[1]=0;attrs[2]=termios.CS8|termios.CREAD|termios.CLOCAL;attrs[3]=0
            attrs[4]=termios.B115200;attrs[5]=termios.B115200;attrs[6][termios.VMIN]=0;attrs[6][termios.VTIME]=0
            termios.tcsetattr(self.fd,termios.TCSANOW,attrs);self.state='awaiting_firmware_hello'
        except BaseException:self.close();raise

    @property
    def ready(self):return self.fd is not None and self.state=='ready'

    def _queue(self,message):
        if len(self.pending)>16384:raise RuntimeError('FPGA transmit queue full')
        self.tx_seq+=1
        self.pending.extend(packet({**message,'v':1,'session_id':self.session_id,'sequence':self.tx_seq}))

    def dispatch(self,command):
        result={'type':'gateway_command','command':command,'sent':False,'motion_authorized':False}
        if self.cfg.get('mode','shadow')=='shadow':result['reason']='shadow_only'
        elif not self.ready:result['reason']='no_compatible_fpga_hello'
        elif self.cfg.get('allow_motion') is not True:result['reason']='mission_output_disabled'
        elif CAPABILITIES.get(command['kind']) not in self.capabilities:result['reason']='unsupported_fpga_capability'
        else:self._queue(command);result.update(queued=True,reason='queued_for_serial',motion_authorized=True)
        self.emit(result);return result

    def poll(self,now):
        if self.fd is None:return []
        events=[]
        try:
            while True:
                try:data=os.read(self.fd,8192)
                except BlockingIOError:break
                if not data:break
                for event in self.reader.feed(data):
                    if event.get('v')!=1:continue
                    seq=event.get('sequence')
                    if type(seq) is not int or seq<0:continue
                    if event.get('kind')=='hello':
                        boot=event.get('boot_id');caps=event.get('capabilities')
                        if not isinstance(boot,str) or not boot or len(boot)>128 or not isinstance(caps,list) or not all(isinstance(c,str) for c in caps):continue
                        if self.peer_boot is not None and boot!=self.peer_boot:
                            events.append({'kind':'link_reset'});self.pending.clear();self.rx_seq=-1
                        if boot==self.peer_boot and seq<=self.rx_seq:continue
                        self.peer_boot=boot;self.capabilities=set(caps);self.rx_seq=seq;self.last_rx=now;self.state='ready'
                        self.emit({'type':'fpga_hello','capabilities':sorted(self.capabilities),'boot_id':boot});continue
                    if self.state!='ready' or event.get('boot_id')!=self.peer_boot or event.get('session_id')!=self.session_id or seq<=self.rx_seq:continue
                    self.rx_seq=seq;self.last_rx=now;events.append(event)
            if self.last_rx is not None and now-self.last_rx>=self.cfg.get('link_timeout_s',1):
                self.state='link_timeout';self.pending.clear();events.append({'kind':'link_lost'});self.last_rx=None
            if self.ready and now-self.last_heartbeat>=.1:
                self._queue({'kind':'heartbeat'});self.last_heartbeat=now
            if self.pending and self.ready:
                try:sent=os.write(self.fd,self.pending);del self.pending[:sent]
                except BlockingIOError:pass
        except (OSError,RuntimeError) as exc:
            self.emit({'type':'gateway_error','error':str(exc)});self.close();events.append({'kind':'link_lost'})
        return events

    def snapshot(self):
        return {'state':self.state,'capabilities':sorted(self.capabilities),'peer_boot':self.peer_boot,
            'motion_authorized':self.ready and self.cfg.get('allow_motion') is True,'wire_protocol':'FAM1 mission-v1 (requires matching firmware)',
            'rx_errors':self.reader.errors,'tx_sequence':self.tx_seq}

    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None
        self.state='closed';self.pending.clear()
