"""Check a saved board parser against the current FIMG producer contract.

Runs only a saved, user-owned Python parser. Does not connect to a board,
load models, install packages or modify an active deployment.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ai_coordinator.frames import encode_frame,encode_frame_v2,FrameReader


def load_parser(path):
    spec=importlib.util.spec_from_file_location('saved_board_frame_parser',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.FrameReader


def accepted(reader_type,wire):
    reader=reader_type();frames=[]
    for offset in range(0,len(wire),65536):frames+=reader.feed(wire[offset:offset+65536])
    return {'frames':len(frames),'errors':reader.errors,
            'sizes':[[f['width'],f['height']] for f in frames],
            'payload_equal':bool(frames) and frames[0]['data']==b'\x00\xf8'*(640*375)}


def review(board_parser):
    data=b'\x00\xf8'*(640*375)
    saved=load_parser(board_parser)
    contracts={}
    for name,wire in [('fimg_v1',encode_frame(data,640,375,1)),('fimg_v2',encode_frame_v2(data,640,375,1))]:
        contracts[name]={'saved_board_parser':accepted(saved,wire),'current_parser':accepted(FrameReader,wire)}
    return {'saved_board_parser_sha256':hashlib.sha256(board_parser.read_bytes()).hexdigest(),
            'contracts':contracts,'fixture':'synthetic red RGB565 640x375 frame',
            'hardware_usb_tested':False,'board_modified':False,'models_loaded':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board-parser',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    report=review(args.board_parser)
    (args.out/'parser_contract.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
