"""MOT1: tiny binary motion frame between RK3588 and the FPGA.

Wire frame (big-endian):
    offset 0  magic  b'MOT1'
    offset 4  seq    uint8
    offset 5  cmd    uint8
    offset 6  value  int16   (duration ms, or signed angle deg)
    offset 8  speed  uint8   (0..255; FPGA converts/limits)
    offset 9  flags  uint8
    offset 10 crc16  uint16  (CRC-16/CCITT-FALSE over bytes 0..9)

This is deliberately separate from the higher-level FAM1 JSON mission protocol:
MOT1 only carries bounded motion primitives, never arbitrary tasks.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b'MOT1'
HEADER = 10
FRAME = 12

STOP = 0
FORWARD = 1
BACKWARD = 2
TURN_LEFT = 3
TURN_RIGHT = 4
SPIN = 5
PATROL_START = 6
PATROL_STOP = 7
HEARTBEAT = 8

CMD_NAMES = {
    STOP: 'stop', FORWARD: 'forward', BACKWARD: 'backward',
    TURN_LEFT: 'turn_left', TURN_RIGHT: 'turn_right', SPIN: 'spin',
    PATROL_START: 'patrol_start', PATROL_STOP: 'patrol_stop',
    HEARTBEAT: 'heartbeat',
}


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    seq: int
    cmd: int
    value: int
    speed: int
    flags: int = 0

    def encode(self):
        body = MAGIC + struct.pack('>BBhBB', self.seq & 0xFF, self.cmd & 0xFF,
                                   self.value, self.speed & 0xFF,
                                   self.flags & 0xFF)
        return body + struct.pack('>H', crc16(body))


def decode_frames(buffer):
    """Return (frames, remainder, errors); resynchronizes on MOT1 magic."""
    data = bytes(buffer)
    frames = []
    errors = 0
    index = 0
    while True:
        start = data.find(MAGIC, index)
        if start < 0:
            return frames, data[index:], errors
        if len(data) - start < FRAME:
            return frames, data[start:], errors
        body = data[start:start + HEADER]
        crc = struct.unpack('>H', data[start + HEADER:start + FRAME])[0]
        if crc16(body) != crc:
            errors += 1
            index = start + 1
            continue
        seq, cmd, value, speed, flags = struct.unpack('>BBhBB', body[4:])
        if cmd not in CMD_NAMES:
            errors += 1
            index = start + FRAME
            continue
        frames.append(Frame(seq, cmd, value, speed, flags))
        index = start + FRAME


def from_intent(intent, seq=0, speed=40):
    """Map a voice_nav intent to a MOT1 frame (or None for unknown)."""
    cmd = intent.get('cmd')
    if cmd == 'stop':
        return Frame(seq, STOP, 0, 0)
    if cmd in ('forward', 'backward'):
        duration = int(round(float(intent.get('duration_s', 0.8)) * 1000))
        duration = max(50, min(5000, duration))
        return Frame(seq, FORWARD if cmd == 'forward' else BACKWARD,
                     duration, speed)
    if cmd == 'turn':
        angle = int(round(float(intent.get('angle_deg', 90))))
        angle = max(5, min(360, abs(angle)))
        return Frame(seq, TURN_LEFT if intent.get('direction') == 'left' else TURN_RIGHT,
                     angle, speed)
    if cmd == 'spin':
        angle = int(round(float(intent.get('angle_deg', 360))))
        angle = max(-720, min(720, angle))
        if angle == 0:
            raise ValueError('spin angle must be nonzero')
        return Frame(seq, SPIN, angle, speed)
    if cmd == 'patrol_start':
        return Frame(seq, PATROL_START, 0, 0)
    if cmd == 'patrol_stop':
        return Frame(seq, PATROL_STOP, 0, 0)
    return None


def heartbeat(seq=0):
    return Frame(seq, HEARTBEAT, 0, 0)
