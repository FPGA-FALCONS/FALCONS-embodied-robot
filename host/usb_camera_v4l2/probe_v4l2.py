"""Probe UVC/V4L2 cameras on the RK3588 without installing anything.

Run on the board:
    python3 probe_v4l2.py

It only reads device nodes, sysfs and v4l2-ctl output; it never changes the
board environment or the coordinator service.
"""
import argparse
import glob
import json
import shutil
import subprocess
import sys
from pathlib import Path


def run(cmd, timeout=8):
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return -1, '', str(exc)


def sysfs_name(device):
    node = Path('/sys/class/video4linux') / Path(device).name / 'name'
    try:
        return node.read_text(encoding='utf-8', errors='replace').strip()
    except OSError:
        return ''


def probe_device(device):
    record = {'device': device, 'sysfs_name': sysfs_name(device)}
    if shutil.which('v4l2-ctl'):
        code, out, err = run(['v4l2-ctl', '-d', device, '--info'])
        record['info'] = out if code == 0 else err
        code, out, err = run(['v4l2-ctl', '-d', device, '--list-formats-ext'])
        record['formats'] = out if code == 0 else err
    else:
        record['info'] = 'v4l2-ctl not installed (do not install on HOLD board)'
        record['formats'] = ''
    return record


def opencv_probe(index, frames=30):
    try:
        import cv2
    except Exception as exc:  # pragma: no cover - board dependent
        return {'index': index, 'error': 'cv2 unavailable: ' + repr(exc)}
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return {'index': index, 'opened': False}
    record = {
        'index': index,
        'opened': True,
        'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        'fps': float(cap.get(cv2.CAP_PROP_FPS)),
        'fourcc': int(cap.get(cv2.CAP_PROP_FOURCC)),
    }
    ok = miss = 0
    shape = None
    for _ in range(frames):
        good, frame = cap.read()
        if good:
            ok += 1
            shape = list(frame.shape)
        else:
            miss += 1
    record.update({'read_ok': ok, 'read_miss': miss, 'shape': shape})
    cap.release()
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json-out', type=Path)
    parser.add_argument('--no-opencv', action='store_true')
    args = parser.parse_args()
    devices = sorted(glob.glob('/dev/video*'))
    result = {
        'devices': [probe_device(device) for device in devices],
        'opencv': [] if args.no_opencv else [opencv_probe(i) for i in range(8)],
        'note': 'Read-only probe; no package installation and no service change.',
    }
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.json_out:
        args.json_out.write_text(text, encoding='utf-8')
    return 0 if devices else 2


if __name__ == '__main__':
    sys.exit(main())
