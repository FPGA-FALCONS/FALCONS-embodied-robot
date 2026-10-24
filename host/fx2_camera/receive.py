"""Receive FIMG camera frames via verified FX2 Bulk-IN, or decode an offline wire capture.

Creates a new session directory; never loads firmware, detaches a driver or installs packages.
"""
import argparse
import json
from pathlib import Path
import sys
import time

if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from ai_coordinator.frames import FrameReader
from ai_coordinator.stream_frames import StreamingFrameReader
from fx2_camera.raw565 import convert


def save_frame(frame, directory, *, replay, received_at_s=None):
    stem=f"frame_{frame['sequence']:010d}"
    path=directory/(stem+'.raw')
    with path.open('xb') as stream:stream.write(frame['data'])
    convert(path,directory/(stem+'.ppm'),frame['width'],frame['height'],'little')
    record={k:v for k,v in frame.items() if k!='data'}
    record.update({'image':str(path.resolve()),'preview':str((directory/(stem+'.ppm')).resolve()),
                   'replay':replay,'received_at_s':received_at_s,
                   'timestamp_origin':'file_replay' if replay else 'host_receipt_only',
                   'source_timestamp_verified':False})
    with (directory/'frames.jsonl').open('a',encoding='utf-8') as stream:
        stream.write(json.dumps(record,ensure_ascii=False)+'\n')
    return record


def offline(source, directory, limit, protocol='fimg1',benchmark=False):
    reader=StreamingFrameReader() if protocol=='fim2' else FrameReader(versions=(2,) if protocol=='fimg2' else (1,2))
    records=[];count=0;total=0;start=time.monotonic()
    with source.open('rb') as stream:
        while chunk:=stream.read(65536):
            total+=len(chunk)
            for frame in reader.feed(chunk):
                if not benchmark:records.append(save_frame(frame,directory,replay=True))
                count+=1
                if count>=limit:break
            if count>=limit:break
    elapsed=time.monotonic()-start
    return {'frames':count,'parser_errors':reader.errors,'trailing_bytes':len(reader.buffer),
            'source':'offline_wire_capture','hardware_tested':False,'wire_protocol':protocol,
            'benchmark_no_image_io':benchmark,'elapsed_s':elapsed,'bytes_read':total,
            'parser_MB_s':total/max(elapsed,1e-9)/1e6,'note':'File/parser speed is not USB hardware throughput'}


def benchmark_usb(camera,limit,seconds):
    start=time.monotonic();deadline=start+seconds;count=0;first=None;last=None;gaps=0;sizes=set()
    while count<limit and time.monotonic()<deadline:
        for frame in camera.read_frames():
            seq=frame['sequence'];sizes.add((frame['width'],frame['height']))
            if first is None:first=seq
            if last is not None:gaps+=max(0,((seq-last)&0xffffffff)-1)
            last=seq;count+=1
    elapsed=time.monotonic()-start
    return {'frames':count,'parser_errors':camera.parser.errors,'source':'usb_bulk_in',
            'hardware_tested':count>0,'usb_selection':camera.selection,'elapsed_s':elapsed,
            'wire_MB_s':camera.bytes_received/max(elapsed,1e-9)/1e6,'complete_fps':count/max(elapsed,1e-9),
            'bytes_read':camera.bytes_received,'usb_calls':camera.usb_calls,'image_sizes':sorted(sizes),
            'first_sequence':first,'last_sequence':last,'sequence_gaps':gaps,
            'aborted_frames':getattr(camera.parser,'aborted_frames',None),
            'crc_errors':getattr(camera.parser,'crc_errors',None),'benchmark_no_image_io':True,
            'source_timestamp_verified':False,'note':'Complete-frame receive rate; capture latency is not measured'}


def live(cfg, directory, limit, seconds,benchmark=False):
    from ai_coordinator.usb_camera import Camera
    cfg={**cfg,'storage':str(directory/'incoming')}
    camera=Camera(cfg);count=0;deadline=time.monotonic()+seconds
    try:
        if benchmark:return benchmark_usb(camera,limit,seconds)
        while count<limit and time.monotonic()<deadline:
            frame=camera.run({'op':'next_frame'})
            if frame['status']!='frame':continue
            path=Path(frame['image'])
            try:
                frame['data']=path.read_bytes()
                save_frame(frame,directory,replay=False,received_at_s=frame['received_at_s'])
            finally:path.unlink(missing_ok=True)
            count+=1
        return {'frames':count,'parser_errors':camera.parser.errors,'source':'usb_bulk_in',
                'hardware_tested':count>0,'usb_selection':camera.selection}
    finally:camera.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--input',type=Path,help='Offline FIMG wire file; never a live source')
    mode.add_argument('--config',type=Path,help='Verified USB selection JSON (or coordinator camera section)')
    parser.add_argument('--out',type=Path,required=True,help='New session directory')
    parser.add_argument('--frames',type=int,default=10)
    parser.add_argument('--seconds',type=float,default=20)
    parser.add_argument('--protocol',choices=('fimg','fimg1','fimg2','fim2'),help='fimg2 requires FIMG version 2; fim2 is a separate experimental protocol')
    parser.add_argument('--benchmark',action='store_true',help='Measure complete receive rate without raw/PPM disk writes or inference')
    args=parser.parse_args(argv)
    if not 1<=args.frames<=1000000 or not 0<args.seconds<=3600:parser.error('Invalid frame/time limit')
    args.out.mkdir(parents=True,exist_ok=False)
    try:
        if args.input:result=offline(args.input,args.out,args.frames,args.protocol or 'fimg1',args.benchmark)
        else:
            cfg=json.loads(args.config.read_text(encoding='utf-8-sig'))
            cfg=cfg.get('camera',cfg)
            if args.protocol:cfg={**cfg,'protocol':args.protocol}
            result=live(cfg,args.out,args.frames,args.seconds,args.benchmark)
        result.update({'firmware_loaded':False,'environment_changed':False,'motion_authorized':False})
        (args.out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False))
        return 0 if result['frames'] else 2
    except (OSError,ValueError,RuntimeError) as exc:
        (args.out/'error.json').write_text(json.dumps({'error':str(exc)},ensure_ascii=False),encoding='utf-8')
        parser.exit(2,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
