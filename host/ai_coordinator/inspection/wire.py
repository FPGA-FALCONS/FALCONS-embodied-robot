"""Inspection transaction v1 inside isolated reserved MOT1 envelopes.

0x60 BEGIN(type,count,zero), 0x61 DATA(uint32 BE), 0x62 END(CRC32).
This envelope is already isolated from the ASCII tuning parser. It never uses
motion opcodes 0..8. The bounded UART transport is useful for diagnostics; the
same transaction format can be delivered by the high-speed OUT receiver.
"""
import struct
import time
import zlib
from ..motion_reference.mot1 import Frame, crc16

BEGIN, DATA, END = 0x60, 0x61, 0x62
HELLO, CONTROL, LAYER_BEGIN, LAYER_DATA, LAYER_COMMIT, HEARTBEAT = range(1, 7)
UI_CLOSE = 7
CAPS, STATE, POSE, INPUT, MAP_DATA, MAP_BEGIN, MAP_END = range(0x80, 0x87)
MAX_WORDS = 32
OP_GOTO, OP_PAUSE, OP_CANCEL = 1, 2, 3


def envelope(sequence, command, payload):
    if not isinstance(payload, bytes) or len(payload) != 4:
        raise ValueError('Envelope payload must be four bytes')
    return Frame(sequence, command, struct.unpack('>h', payload[:2])[0], payload[2], payload[3]).encode()


def encode(kind, words, sequence=0):
    if type(kind) is not int or not 1 <= kind <= 255 or not 1 <= len(words) <= MAX_WORDS:
        raise ValueError('Inspection messages contain 1..32 words')
    if any(type(w) is not int or not 0 <= w <= 0xffffffff for w in words):
        raise ValueError('Inspection words must be uint32')
    header = struct.pack('>BBH', kind, len(words), 0)
    payload = b''.join(struct.pack('>I', w) for w in words)
    frames = [envelope(sequence, BEGIN, header)]
    frames.extend(envelope(sequence, DATA, payload[n:n+4]) for n in range(0, len(payload), 4))
    frames.append(envelope(sequence, END, struct.pack('>I', zlib.crc32(header+payload) & 0xffffffff)))
    return b''.join(frames)


class Reader:
    def __init__(self):
        self.buffer = bytearray(); self.pending = None; self.errors = 0; self.updated = 0.

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.pending is not None and now-self.updated > 1:
            self.pending = None; self.errors += 1
        self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 4:
            start = self.buffer.find(b'MOT1')
            if start < 0:
                self.buffer = self.buffer[-3:]; break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 12:
                break
            frame = bytes(self.buffer[:12]); del self.buffer[:12]
            if crc16(frame[:10]) != struct.unpack('>H', frame[10:])[0]:
                self.pending = None; self.errors += 1; continue
            seq, cmd, payload = frame[4], frame[5], frame[6:10]
            if cmd not in (BEGIN, DATA, END):
                continue
            self.updated = now
            if cmd == BEGIN:
                kind, count, reserved = struct.unpack('>BBH', payload)
                self.pending = {'kind': kind, 'count': count, 'sequence': seq, 'payload': bytearray(payload)} \
                    if kind and 1 <= count <= MAX_WORDS and reserved == 0 else None
                if self.pending is None: self.errors += 1
            elif self.pending is None or seq != self.pending['sequence']:
                self.pending = None; self.errors += 1
            elif cmd == DATA:
                if len(self.pending['payload']) >= 4*(self.pending['count']+1):
                    self.pending = None; self.errors += 1
                else:
                    self.pending['payload'].extend(payload)
            else:
                p = self.pending; self.pending = None
                if len(p['payload']) != 4*(p['count']+1) or zlib.crc32(p['payload']) & 0xffffffff != struct.unpack('>I', payload)[0]:
                    self.errors += 1; continue
                messages.append({'kind': p['kind'], 'sequence': seq,
                                 'words': list(struct.unpack('>'+'I'*p['count'], p['payload'][4:]))})
        return messages


def layer_messages(session, epoch, version, layer, *, task_id=0, command_id=0):
    payload = layer['raw']+layer['cost']
    if len(payload) != 16384 or zlib.crc32(payload) & 0xffffffff != layer['crc32']:
        raise ValueError('Complete validated raw/cost layer required')
    yield LAYER_BEGIN, [session, task_id, epoch, command_id, version, layer['crc32']]
    for offset in range(0, len(payload), 112):
        block = payload[offset:offset+112]
        # Bit zero is the first grid cell; words here intentionally encode
        # payload bytes little-endian, unlike the transaction word transport.
        words = [int.from_bytes(block[n:n+4], 'little') for n in range(0, len(block), 4)]
        yield LAYER_DATA, [session, epoch, version, offset//4, *words]
    yield LAYER_COMMIT, [session, task_id, epoch, command_id, version]


def control(command, *, user_token=0, commissioned=False):
    from .grid import cell
    operations = {'goto': OP_GOTO, 'pause': OP_PAUSE, 'cancel': OP_CANCEL}
    if command['kind'] not in operations or type(user_token) is not int or not 0 <= user_token <= 65535:
        raise ValueError('Unsupported inspection action/token')
    op = operations[command['kind']]
    flags = op | (0x100 if commissioned is True else 0) | (user_token << 16)
    if command.get('reason') == 'layer_update' and commissioned is True:
        flags |= 0x200
    goal = cell([v/1000 for v in command['goal_mm']]) if op == OP_GOTO else (0, 0)
    return [command[k] for k in ('session_id', 'task_id', 'epoch', 'command_id')] + [
        flags, *goal, command.get('layer_version', 0) | (30 << 16)]


def status(words):
    if len(words) != 8:
        raise ValueError('Inspection status contains eight words')
    names = {0: 'hello', 1: 'accepted', 2: 'stopped', 3: 'arrived', 4: 'no_path',
             5: 'fault', 6: 'layer_committed', 7: 'uploading'}
    if words[4] not in names:
        raise ValueError('Unknown inspection execution status')
    return {'session_id': words[0], 'task_id': words[1], 'epoch': words[2], 'command_id': words[3],
            'kind': names[words[4]], 'layer_version': words[5], 'stopped': bool(words[6] & 1),
            'manual': bool(words[6] & 2), 'error_code': words[7]}


def inputs(words):
    if len(words) != 8 or words[3] & ~0x1f7 or (words[3] >> 4) & 3 == 3 or words[6] > 65535:
        raise ValueError('Invalid inspection physical input snapshot')
    def signed16(value):
        return value-65536 if value & 0x8000 else value
    steer, throttle = signed16(words[4] & 65535), signed16(words[4] >> 16)
    if not -2048 <= steer <= 2047 or not -2048 <= throttle <= 2047:
        raise ValueError('Joystick outside signed 12-bit range')
    return dict(session_id=words[0], epoch=words[1], sequence=words[2],
                open=bool(words[3] & 1), manual=bool(words[3] & 2), master=bool(words[3] & 4),
                view=(words[3] >> 4) & 3, record=bool(words[3] & 64),
                rc_valid=bool(words[3] & 128), permission=bool(words[3] & 256),
                steer=steer, throttle=throttle, confirms=words[5] & 65535, backs=words[5] >> 16,
                user_token=words[6], vehicle_width_mm=words[7] & 65535, forward_check_cells=words[7] >> 16)
