"""Host CRC vectors for production-size display and bounded FPGA diagnostics."""
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'host'))
from ai_coordinator.inspection.stream import Container,DISPLAY,DIAGNOSTIC,CONTROL,MAX_PAYLOAD,MAP,diagnostic_reply
from ai_coordinator.inspection.wire import encode,LAYER_BEGIN

challenge=Container(DIAGNOSTIC,1,2,3,7,42,payload=bytes(range(32)))
diag=challenge.encode()
display=Container(DISPLAY,1,2,3,7,42,816,420,1632,1,bytes(n&255 for n in range(MAX_PAYLOAD))).encode()[:48]
control=Container(CONTROL,1,2,3,7,43,payload=encode(LAYER_BEGIN,[1,2,3,4,5,6])).encode()
words=lambda b:[int.from_bytes(b[n:n+2],'little') for n in range(0,len(b),2)]
values=words(diag)+words(display)+words(control)
(root/'sim/vectors/inspection_stream.hex').write_text(''.join(f'{v:04x}\n' for v in values),encoding='ascii',newline='\n')
(root/'sim/vectors/inspection_diagnostic_reply.hex').write_text(''.join(f'{v:04x}\n' for v in words(diagnostic_reply(challenge).encode())),encoding='ascii',newline='\n')
print('Vector words:',len(values),'diagnostic 0..39 / display header 40..63 / control 64..135')

map_payload=b''.join((n^0x5a5a).to_bytes(2,'little') for n in range(4096))
map_frame=Container(MAP,1,2,3,7,0x00050006,256,256,32,1,map_payload)
(root/'sim/vectors/inspection_map_stream.hex').write_text(''.join(f'{v:04x}\n' for v in words(map_frame.encode())),encoding='ascii')
print('Map CRC:',hex(__import__('zlib').crc32(map_payload)))
