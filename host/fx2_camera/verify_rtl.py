"""Cross-check sampled RTL FX2 bytes against the production Python FIMG codec.

Input is the standalone testbench evidence, including an interrupted frame and
an explicit FPGA reset. A receiver restart is required at that known boundary.
"""
import argparse
import json
from pathlib import Path
import random
import sys

if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.frames import encode_frame,FrameReader


def golden(entry):
    step=entry['downsample'];width=entry['source_width'];height=entry['source_height'];tag=entry['tag']
    pixels=b''.join((((tag<<8)+y*width+x)&65535).to_bytes(2,'little')
                    for y in range(0,height,step) for x in range(0,width,step))
    return encode_frame(pixels,entry['width'],entry['height'],entry['sequence'])


def decode_chunks(reader,wire):
    rng=random.Random(3588);frames=[]
    while wire:
        count=rng.randrange(1,79);frames+=reader.feed(wire[:count]);wire=wire[count:]
    return frames


def verify(directory):
    entries=json.loads((directory/'expected_frames.json').read_text(encoding='utf-8'))
    if len(entries)!=3:raise ValueError('Expected three complete testbench frames')
    complete=[golden(e) for e in entries]
    aborted=golden({**entries[0],'tag':6,'sequence':2})[:10]
    wire=(directory/'wire.bin').read_bytes()
    if wire!=complete[0]+complete[1]+aborted+complete[2]:
        raise AssertionError('RTL bus bytes differ from independent Python header/CRC/pixel encoding')
    first=FrameReader();prior=decode_chunks(first,complete[0]+complete[1])
    if [f['sequence'] for f in prior]!=[0,1]:raise AssertionError('Initial session did not decode')
    if decode_chunks(first,aborted+complete[2]):raise AssertionError('Reset sequence accepted in old session')
    fresh=decode_chunks(FrameReader(),aborted+complete[2])
    if len(fresh)!=1 or fresh[0]['data']!=complete[2][24:]:raise AssertionError('Explicit new session failed recovery')
    return {'scenario':directory.name,'wire_bytes':len(wire),'complete_frames_verified':3,
            'crc_pixels_and_header_equal':True,'reset_requires_new_receiver':True,
            'hardware_tested':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--sim',type=Path,required=True)
    args=parser.parse_args();results=[verify(args.sim/name) for name in ('small','multi_packet','exact_packet')]
    report={'scenarios':results,'status':'PASS','hardware_tested':False}
    (args.sim/'python_crosscheck.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
