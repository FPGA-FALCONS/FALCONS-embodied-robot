"""Parse a Chinese motion phrase and send MOT1 frames to the FPGA.

Examples:
    python3 send_motion.py --dry-run "前进"
    python3 send_motion.py --device /dev/serial/by-id/usb-... "原地顺时针转一圈"

Linux only for the real serial path; --dry-run works on Windows.  It never
changes DTR, resets the board, uploads firmware or installs packages.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

HOST=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HOST))
from semantic_nav.mot1 import Frame, from_intent, heartbeat, STOP
from ai_coordinator.motion_intents import inspect_motion


def prepare_frame(text,seq=1,speed=40,distance=None):
    if type(seq) is not int or not 0<=seq<=255:raise ValueError('seq must be 0..255')
    if type(speed) is not int or not 1<=speed<=40:raise ValueError('initial test speed code must be 1..40')
    decision=inspect_motion(text,distance=distance,speed_code=speed)
    if decision is None or not decision['valid']:raise ValueError('Not a single supported immediate motion command')
    if not decision.get('wire_preview'):raise ValueError(decision['reason'])
    return decision['intent'],from_intent(decision['intent'],seq,speed)


def write_frame(fd,wire):
    import select
    pending=memoryview(wire);deadline=time.monotonic()+1
    while pending:
        if time.monotonic()>=deadline:raise TimeoutError('MOT1 serial write stalled')
        try:sent=os.write(fd,pending)
        except BlockingIOError:
            select.select([], [fd], [], .02);continue
        if sent<=0:raise OSError('MOT1 serial device stopped accepting bytes')
        pending=pending[sent:]


def open_serial(device):
    if os.name != 'posix':
        raise RuntimeError('real serial path requires Linux')
    if not device.startswith('/dev/serial/by-id/'):
        raise ValueError('use an explicit /dev/serial/by-id device')
    import termios
    import fcntl
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd
    except BaseException:
        os.close(fd)
        raise


def main(argv=None):
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('text', nargs='+')
    parser.add_argument('--device')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--speed', type=int, default=10)
    parser.add_argument('--seq', type=int, default=1)
    parser.add_argument('--distance-speed', type=float)
    parser.add_argument('--distance-calibrated', action='store_true')
    args = parser.parse_args(argv)
    distance=None
    if args.distance_speed is not None:
        distance={'calibrated':args.distance_calibrated,'calibrated_speed_mps':args.distance_speed}
    try:intent,frame=prepare_frame(''.join(args.text),args.seq,args.speed,distance)
    except ValueError as exc:
        print(json.dumps({'error':str(exc),'sent':False,'executed':False,'physical_task_completed':False,
                          'distance_estimate':bool(distance)},ensure_ascii=False))
        return 2
    wire = frame.encode()
    output = {'intent': intent, 'frame': frame.__dict__, 'hex': wire.hex(),
              'preview_only':bool(args.dry_run or not args.device),'physical_task_completed':False,
              'distance_estimate':bool(intent.get('distance_mode')=='timed_estimate'),
              'arrival_verified':False}
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if args.dry_run or not args.device:
        return 0
    fd = open_serial(args.device)
    seq=args.seq
    try:
        write_frame(fd,wire)
        duration = 0.0
        if frame.cmd in (1, 2):
            duration = frame.value / 1000.0
        elif frame.cmd in (3, 4, 5):
            duration = 6.0
        deadline = time.monotonic() + duration + 1.0
        while time.monotonic() < deadline:
            seq = (seq + 1) & 0xFF
            write_frame(fd,heartbeat(seq).encode())
            time.sleep(0.2)
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if chunk:print(json.dumps({'unverified_serial_rx_hex':chunk[:256].hex(),'execution_receipt':False}))
    finally:
        # Explicit bounded test ends in STOP, including interrupt/error cleanup.
        # The commissioned FPGA must retain ownership/zero targets afterward.
        try:write_frame(fd,Frame((seq+1)&0xff,STOP,0,0).encode())
        except (OSError,TimeoutError) as exc:print(json.dumps({'stop_write_error':str(exc),'physical_stop_verified':False}))
        os.close(fd)
    return 0


if __name__ == '__main__':
    sys.exit(main())
