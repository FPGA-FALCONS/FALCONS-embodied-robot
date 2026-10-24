"""Single mission gateway and the one reviewed serial owner.

The same file descriptor carries FAM1 mission frames and MOT1 bounded motion
frames.  Frames are queued as whole units so a MOT1 STOP can never be spliced
into a partially written FAM1 packet (or the reverse).  A second process is
excluded with ``flock``; the standalone sender therefore cannot drive the
same UART while the coordinator owns it.
"""
from collections import deque
import os
from .schema import PacketReader,packet

CAPABILITIES={'start_search':'search_v1','target_found':'target_accept_v1','return_home':'return_home_v1','cancel':'cancel_v1'}
CAPABILITIES.update({'goto_waypoint':'waypoint_v1','pause_patrol':'pause_patrol_v1',
                     'approach_target':'local_approach_v1','target_observation':'local_approach_v1'})


class FrameTicket:
    """Completion evidence for one whole serial frame, not device execution."""
    __slots__=('data','tag','written')
    def __init__(self,data,tag):
        self.data=bytes(data);self.tag=tag;self.written=0
    @property
    def complete(self):return self.written>=len(self.data)


class SerialPort:
    """Non-blocking 115200 8N1 owner with whole-frame scheduling.

    ``enqueue`` accepts one complete frame and returns a ticket.  A priority
    frame is placed after any frame already partially written but ahead of all
    queued frames.  This preserves byte integrity while making STOP the next
    complete frame.  A completed ticket means bytes reached the OS serial
    buffer; it is not an FPGA acknowledgment or physical motion receipt.
    """
    def __init__(self,device):
        self.device=device
        self.fd=None
        self.current=None
        self.frames=deque()
        self.tx_bytes=0
        self.rx_bytes=0
        self.tx_frames=0
        self.last_error=None

    def open(self):
        if os.name!='posix':raise RuntimeError('Serial gateway requires Linux')
        if not isinstance(self.device,str) or not self.device.startswith('/dev/serial/by-id/'):
            raise ValueError('Select an explicit /dev/serial/by-id device')
        import termios
        import fcntl
        fd=os.open(self.device,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            attrs=termios.tcgetattr(fd);attrs[0]=0;attrs[1]=0
            attrs[2]=termios.CS8|termios.CREAD|termios.CLOCAL;attrs[3]=0
            attrs[4]=termios.B115200;attrs[5]=termios.B115200
            attrs[6][termios.VMIN]=0;attrs[6][termios.VTIME]=0
            termios.tcsetattr(fd,termios.TCSANOW,attrs)
            self.fd=fd
        except BaseException:
            os.close(fd);raise

    @property
    def pending_count(self):return len(self.frames)+(1 if self.current is not None else 0)

    @property
    def pending_bytes(self):
        current=b'' if self.current is None else self.current.data[self.current.written:]
        return current+b''.join(ticket.data for ticket in self.frames)

    def enqueue(self,frame,priority=False,tag='fam1'):
        if self.fd is None:raise OSError('serial link is not open')
        if not isinstance(frame,(bytes,bytearray)) or not frame:raise ValueError('serial frame must be non-empty bytes')
        queued=len(self.pending_bytes)
        if queued+len(frame)>32768:raise RuntimeError('serial transmit queue full')
        item=FrameTicket(frame,tag)
        if priority:self.frames.appendleft(item)
        else:self.frames.append(item)
        return item

    def drop(self,tag):
        """Remove queued frames with this tag; never interrupt the current frame."""
        if not self.frames:return 0
        kept=deque(ticket for ticket in self.frames if ticket.tag!=tag)
        removed=len(self.frames)-len(kept);self.frames=kept;return removed

    def flush(self):
        if self.fd is None:raise OSError('serial link is not open')
        written=0
        while True:
            if self.current is None:
                if not self.frames:break
                self.current=self.frames.popleft()
            chunk=self.current.data[self.current.written:]
            try:sent=os.write(self.fd,chunk)
            except BlockingIOError:break
            if sent<=0:raise OSError('serial device stopped accepting bytes')
            self.current.written+=sent;written+=sent;self.tx_bytes+=sent
            if self.current.complete:
                self.tx_frames+=1;self.current=None
        return written

    def read(self,limit=8192):
        if self.fd is None:return b''
        # VMIN=0/VTIME=0 makes a tty return b'' when no bytes are ready; this
        # is not EOF.  A real unplug surfaces as OSError/EIO on read or write.
        data=os.read(self.fd,limit)
        self.rx_bytes+=len(data)
        return data

    def close(self):
        fd=self.fd;self.fd=None;self.current=None;self.frames.clear()
        if fd is not None:
            try:os.close(fd)
            except OSError as exc:self.last_error=str(exc)

    def snapshot(self):
        return {'device':self.device,'open':self.fd is not None,'tx_frames':self.tx_frames,
                'tx_bytes':self.tx_bytes,'rx_bytes':self.rx_bytes,
                'queued_frames':self.pending_count,'last_error':self.last_error}


class Gateway:
    def __init__(self,cfg,session_id,emit,transport=None):
        self.cfg=cfg;self.session_id=session_id;self.emit=emit
        self.transport=transport or SerialPort(cfg.get('device',''))
        self.reader=PacketReader()
        self.capabilities=set();self.peer_boot=None;self.last_rx=None;self.rx_seq=-1;self.tx_seq=0
        self.last_heartbeat=0
        self.state='shadow' if cfg.get('mode','shadow')=='shadow' else 'disconnected'

    @property
    def fd(self):return self.transport.fd

    @property
    def pending(self):return self.transport.pending_bytes

    def start(self):
        if self.cfg.get('mode','shadow')=='shadow':return
        # No DTR toggle, reset, firmware upload or competing serial owner.
        try:self.transport.open()
        except BaseException:self.transport.close();raise
        self.state='awaiting_firmware_hello'

    @property
    def ready(self):return self.fd is not None and self.state=='ready'

    def _queue(self,message):
        self.tx_seq+=1
        self.transport.enqueue(packet({**message,'v':1,'session_id':self.session_id,'sequence':self.tx_seq}),tag='fam1')

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
                try:data=self.transport.read()
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
                            events.append({'kind':'link_reset'});self.transport.drop('fam1');self.rx_seq=-1
                        if boot==self.peer_boot and seq<=self.rx_seq:continue
                        self.peer_boot=boot;self.capabilities=set(caps);self.rx_seq=seq;self.last_rx=now;self.state='ready'
                        self.emit({'type':'fpga_hello','capabilities':sorted(self.capabilities),'boot_id':boot});continue
                    if self.state!='ready' or event.get('boot_id')!=self.peer_boot or event.get('session_id')!=self.session_id or seq<=self.rx_seq:continue
                    self.rx_seq=seq;self.last_rx=now;events.append(event)
            if self.last_rx is not None and now-self.last_rx>=self.cfg.get('link_timeout_s',1):
                self.state='link_timeout';self.transport.drop('fam1');events.append({'kind':'link_lost'});self.last_rx=None
            if self.ready and now-self.last_heartbeat>=.1:
                self._queue({'kind':'heartbeat'});self.last_heartbeat=now
            self.transport.flush()
        except (OSError,RuntimeError) as exc:
            self.emit({'type':'gateway_error','error':str(exc)[:300]});self.close();events.append({'kind':'link_lost'})
        return events

    def snapshot(self):
        port=self.transport.snapshot()
        return {'state':self.state,'capabilities':sorted(self.capabilities),'peer_boot':self.peer_boot,
            'motion_authorized':self.ready and self.cfg.get('allow_motion') is True,
            'wire_protocol':'FAM1 mission-v1 (requires matching firmware)','serial_owner':'coordinator',
            'serial':port,'device_receipts_supported':False,
            'rx_errors':self.reader.errors,'tx_sequence':self.tx_seq}

    def close(self):
        self.transport.close();self.state='closed'
