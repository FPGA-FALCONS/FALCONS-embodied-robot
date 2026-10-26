"""AI-only high-speed containers. No FPGA camera pixels use this protocol.

48-byte little-endian header; header CRC covers its first 44 bytes, payload
has its own IEEE CRC32. Every container is even-sized for the 16-bit FIFO.
Session/epoch/context bind a rendered frame to the actual task and UI view.
"""
from dataclasses import dataclass, replace
import struct
import time
import zlib

HEADER = struct.Struct('<4sBBHIIIIIIHHHHII')
MAGIC = b'AIF1'
DISPLAY, CONTROL, MAP, DIAGNOSTIC = 1, 2, 3, 4
MAX_PAYLOAD = 816*420*2
DIAGNOSTIC_SIGNATURE = 0x46504741


def diagnostic_reply(challenge,frequency_hz=30_000_000):
    if not isinstance(challenge,Container) or challenge.kind!=DIAGNOSTIC:
        raise ValueError('A diagnostic challenge is required')
    # A plain USB firmware byte loopback must not masquerade as FPGA AIF1.
    # The FPGA changes the sequence and recomputes the header CRC.
    return replace(challenge,session=frequency_hz,sequence=challenge.sequence ^ DIAGNOSTIC_SIGNATURE)


def valid_diagnostic(challenge,reply):
    return isinstance(reply,Container) and 29_400_000<=reply.session<=30_600_000 and reply==diagnostic_reply(challenge,reply.session)


@dataclass(frozen=True)
class Container:
    kind: int
    session: int
    task: int
    epoch: int
    context: int
    sequence: int
    width: int = 0
    height: int = 0
    stride: int = 0
    view: int = 0
    payload: bytes = b''

    def encode(self):
        identifiers = (self.session, self.task, self.epoch, self.context, self.sequence)
        if any(type(v) is not int or not 0 <= v <= 0xffffffff for v in identifiers):
            raise ValueError('Container identifiers must be uint32')
        if any(type(v) is not int or not 0 <= v <= 65535 for v in (self.width, self.height, self.stride, self.view)):
            raise ValueError('Container geometry must be uint16')
        if not isinstance(self.payload, bytes):
            raise ValueError('Immutable payload required')
        validate(self.kind, len(self.payload), self.width, self.height, self.stride, self.view)
        prefix = HEADER.pack(MAGIC, 1, self.kind, HEADER.size, *identifiers, len(self.payload),
                             self.width, self.height, self.stride, self.view,
                             zlib.crc32(self.payload) & 0xffffffff, 0)[:44]
        return prefix + struct.pack('<I', zlib.crc32(prefix) & 0xffffffff) + self.payload


def validate(kind, size, width, height, stride, view):
    if size > MAX_PAYLOAD or size < 2 or size & 1 or view > 2:
        raise ValueError('Invalid payload bounds/view')
    if kind == DISPLAY:
        if (width, height, stride, size) != (816, 420, 1632, MAX_PAYLOAD):
            raise ValueError('Display must be 816x420 RGB565 little-endian')
    elif kind == MAP:
        if (width, height, stride, size) != (256, 256, 32, 8192):
            raise ValueError('Only complete 256x256 LSB-first radar maps are published')
    elif kind in (CONTROL, DIAGNOSTIC):
        if width or height or stride or size > (408 if kind == CONTROL else 512):
            raise ValueError('Invalid bounded control/diagnostic container')
        if kind == CONTROL and size % 12:
            raise ValueError('Control payload must contain whole MOT1 envelopes')
    else:
        raise ValueError('Unknown high-speed container kind')


class Reader:
    def __init__(self, timeout_s=1.):
        self.buffer = bytearray(); self.updated = None; self.errors = 0
        self.timeout_s = timeout_s

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.buffer and self.updated is not None and now-self.updated > self.timeout_s:
            self.buffer.clear(); self.errors += 1
        if data:
            self.updated = now
        # A caller feeds bounded libusb chunks. Processing here also bounds
        # junk input, partial frames and memory held after a disconnect.
        result = []
        for offset in range(0, len(data), 65536):
            self.buffer.extend(data[offset:offset+65536])
            while True:
                begin = self.buffer.find(MAGIC)
                if begin < 0:
                    self.buffer = self.buffer[-3:]; break
                if begin:
                    del self.buffer[:begin]
                if len(self.buffer) < HEADER.size:
                    break
                h = HEADER.unpack_from(self.buffer)
                try:
                    if h[1] != 1 or h[3] != HEADER.size or zlib.crc32(self.buffer[:44]) & 0xffffffff != h[-1]:
                        raise ValueError('Header CRC/version mismatch')
                    validate(h[2], h[9], *h[10:14])
                except ValueError:
                    self.errors += 1; del self.buffer[:4]; continue
                size = HEADER.size+h[9]
                if len(self.buffer) < size:
                    break
                payload = bytes(self.buffer[HEADER.size:size])
                if zlib.crc32(payload) & 0xffffffff != h[-2]:
                    # Search again so a following valid frame can recover a
                    # truncated transfer, rather than trusting a corrupt body.
                    self.errors += 1; del self.buffer[:4]; continue
                del self.buffer[:size]
                result.append(Container(h[2], *h[4:9], *h[10:14], payload))
        return result
