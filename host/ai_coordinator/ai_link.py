"""AI controls and display on the existing, exclusively owned UART.

Reserved MOT1 envelope commands 0x40..0x44 update the display; 0x50 carries
physical controls and display commit receipts. They are not motion receipts.
"""
import struct
import json
from pathlib import Path
from .motion_reference.mot1 import Frame, crc16

BEGIN, GLYPH, COMMIT, FONT, CONTROL = 0x40, 0x41, 0x42, 0x44, 0x50
COLS, ROWS = 46, 18
CELLS = COLS * ROWS


def glyph(character):
    """ASCII followed by the A1..F7 / A1..FE grid in the host bitmap atlas."""
    if 32 <= ord(character) <= 126:
        return ord(character) - 32
    try:
        raw = character.encode('gb2312')
    except UnicodeError:
        return ord('?') - 32
    if len(raw) == 2 and 0xa1 <= raw[0] <= 0xf7 and 0xa1 <= raw[1] <= 0xfe:
        return 95 + (raw[0] - 0xa1) * 94 + raw[1] - 0xa1
    return ord('?') - 32


def font_rows():
    data=json.loads(Path(__file__).with_name('ai_font_rows.json').read_text(encoding='utf-8'))
    if data.get('format')!='ascii-gb2312-16x16' or len(data['glyphs'])!=8273:
        raise ValueError('AI text atlas format mismatch')
    return data['glyphs']


def display_frames(lines, transaction, atlas, cache=None):
    cells = []
    for line in list(lines)[:ROWS]:
        cells.extend(glyph(c) for c in str(line)[:COLS].ljust(COLS))
    # Preserve row positions; trailing blank cells are already hidden by the
    # committed count. Avoid spending UART time redrawing empty bottom rows.
    while cells and cells[-1]==glyph(' '):cells.pop()
    needed=list(dict.fromkeys(cells))
    # More than 512 distinct glyphs cannot fit this bounded screen cache.
    if len(needed)>512:
        allowed=set(needed[:511]);allowed.add(glyph('?'))
        cells=[x if x in allowed else glyph('?') for x in cells]
        needed=list(dict.fromkeys(cells))
    mapping={key:value for key,value in (cache or {}).items() if key in needed}
    free=iter(i for i in range(512) if i not in set(mapping.values()))
    frames=[Frame(transaction,BEGIN,len(cells),0).encode()]
    for key in needed:
        if key in mapping:continue
        slot=next(free);mapping[key]=slot
        rows=atlas[key]
        if len(rows)!=16 or any(type(x) is not int or not 0<=x<=65535 for x in rows):
            raise ValueError('Invalid AI glyph bitmap')
        for row,word in enumerate(rows):
            value=word if word<32768 else word-65536
            frames.append(Frame(transaction,FONT,value,slot&255,((slot>>8)<<4)|row).encode())
    for index, value in enumerate(cells):
        frames.append(Frame(transaction,GLYPH,mapping[value],index&255,index>>8).encode())
    frames.append(Frame(transaction,COMMIT,len(cells),0).encode())
    return frames,mapping


class ControlReader:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while True:
            start = self.buffer.find(b'MOT1')
            if start < 0:
                del self.buffer[:-3]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 12:
                break
            frame = bytes(self.buffer[:12])
            if crc16(frame[:10]) != int.from_bytes(frame[10:], 'big'):
                self.errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:12]
            seq, command, raw_flags, controls, reserved = struct.unpack('>BBHBB', frame[4:10])
            flags=raw_flags&255
            mode, selected = controls & 3, (controls >> 3) & 3
            if command != CONTROL or flags & ~63 or controls & ~63 or reserved or mode > 2 or selected > 2:
                continue
            result.append({'kind': 'ai_controls', 'sequence': seq,
                           'enabled': bool(flags & 1), 'rc_valid': bool(flags & 2),
                           'manual': bool(flags & 4), 'estop': bool(flags & 8),
                           'swb_valid': bool(flags & 16), 'selected': selected,
                           'confirm_toggle':bool(controls & 32),
                           'display_ack':raw_flags>>8,'display_bank':(flags>>5)&1,
                           'mode': ('semantic_navigation','object_recognition','chat')[mode],
                           'chat': mode == 2, 'record': bool(controls & 4), 'vision': mode == 1})
        return result
