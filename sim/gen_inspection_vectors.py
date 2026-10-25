"""Regenerate checked-in host-to-RTL vectors; no camera/model/serial access."""
from pathlib import Path
import sys
import zlib

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'host'))
from ai_coordinator.inspection.wire import encode, CONTROL, HELLO, UI_CLOSE
from ai_coordinator.inspection.grid import PLANE_BYTES, set_bit

out = root/'sim/vectors'
out.mkdir(exist_ok=True)
out.joinpath('inspection_control.hex').write_text(''.join(f'{b:02x}\n' for b in
    encode(CONTROL, [1, 2, 3, 4, 5, 0xffffffff, 7, 30], 77)), encoding='ascii')
out.joinpath('inspection_application.hex').write_text(''.join(f'{b:02x}\n' for b in
    encode(HELLO, [5], 1) + encode(CONTROL, [5, 1, 1, 1, 0x00010101, 140, 128, 0x001e0001], 2) +
    encode(UI_CLOSE, [5, 1, 1, 1], 3) + encode(UI_CLOSE, [5, 1, 2, 1], 4) +
    encode(UI_CLOSE, [5, 1, 1, 1], 5)), encoding='ascii')
raw = bytearray(PLANE_BYTES)
for y in range(4, 13): set_bit(raw, 8, y)
payload = bytes(raw+raw)
words = [zlib.crc32(payload) & 0xffffffff]+[int.from_bytes(payload[i:i+4], 'little') for i in range(0, len(payload), 4)]
out.joinpath('inspection_wall_words.hex').write_text(''.join(f'{w:08x}\n' for w in words), encoding='ascii')
