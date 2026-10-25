"""Probe UVC nodes or capture a bounded, isolated commissioning session.

No model, motor, service or active configuration changes. Run with PYTHONPATH=host:
    python3 -m ai_coordinator.uvc_prepare probe --out probe.json
    python3 -m ai_coordinator.uvc_prepare capture --device /dev/v4l/by-id/USB_LINK \
        --width 1280 --height 720 --fps 30 --fourcc MJPG --out session_720p
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time

from .v4l2_camera import select_device


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n', encoding='utf-8')


def decode_capability(data):
    if len(data) != 104:
        raise ValueError('Expected Linux v4l2_capability ABI (104 bytes)')
    driver, card, bus = [bytes(data[a:b]).split(b'\0', 1)[0].decode('utf-8', 'replace')
                         for a, b in ((0, 16), (16, 48), (48, 80))]
    version, capabilities, device_caps = struct.unpack_from('=III', data, 80)
    effective = device_caps if capabilities & 0x80000000 else capabilities
    return {'driver': driver, 'card': card, 'bus_info': bus, 'version': version,
            'capabilities': capabilities, 'device_caps': device_caps,
            'video_capture': bool(effective & (0x00000001 | 0x00001000)),
            'metadata_capture': bool(effective & 0x00800000),
            'streaming': bool(effective & 0x04000000)}


def query_capability(node):
    import fcntl  # Linux only; importing this module on a test host remains safe.
    fd = os.open(node, os.O_RDONLY | os.O_NONBLOCK)
    try:
        data = bytearray(104)
        fcntl.ioctl(fd, 0x80685600, data, True)  # VIDIOC_QUERYCAP
        return decode_capability(data)
    finally:
        os.close(fd)


def ctl(device, option):
    executable = shutil.which('v4l2-ctl')
    if not executable:
        raise RuntimeError('Existing v4l2-ctl is required; do not auto-install packages')
    result = subprocess.run([executable, '-d', device, option], capture_output=True,
                            text=True, timeout=8, env={**os.environ, 'LC_ALL': 'C'})
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


def discrete_modes(text):
    """Only accept explicitly enumerated discrete modes, never guess a range."""
    result = []
    fourcc = size = None
    for line in text.splitlines():
        match = re.search(r"^\s*\[\d+\]: '(.{4})'", line)
        if match:
            fourcc, size = match[1], None
            continue
        match = re.search(r'Size: Discrete (\d+)x(\d+)', line)
        if match:
            size = (int(match[1]), int(match[2]))
            continue
        if 'Size:' in line:
            size = None
        match = re.search(r'Interval: Discrete .*\(([0-9.]+) fps\)', line)
        if match and fourcc and size:
            fps = float(match[1])
            if math.isfinite(fps) and fps > 0:
                record = {'fourcc': fourcc, 'width': size[0], 'height': size[1], 'fps': fps}
                if record not in result:
                    result.append(record)
    return result


def inspect_device(device):
    identity = select_device(device)
    capability = query_capability(identity['node'])
    record = {'identity': identity, 'capability': capability, 'modes': [],
              'capture_candidate': capability['video_capture'] and capability['streaming']}
    if record['capture_candidate']:
        record['formats_text'] = ctl(device, '--list-formats-ext')
        record['modes'] = discrete_modes(record['formats_text'])
        record['controls_text'] = ctl(device, '--list-ctrls')
    return record


def discover(dev_root=Path('/dev')):
    # Prefer by-id, retain by-path aliases, exclude ISP/CIF and metadata nodes
    # from capture. Discovery itself never starts a stream.
    records, by_node, errors = [], {}, []
    for directory in ('by-id', 'by-path'):
        for link in sorted((dev_root / 'v4l' / directory).glob('*')):
            try:
                node = str(link.resolve(strict=True))
                if node in by_node:
                    by_node[node]['aliases'].append(str(link))
                    continue
                record = inspect_device(str(link))
                record['aliases'] = [str(link)]
                records.append(record)
                by_node[node] = record
            except ValueError:
                # Stable links for non-USB ISP devices are deliberately ignored.
                continue
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                errors.append({'device': str(link), 'error': str(exc)})
    return {'schema_version': 1, 'devices': records, 'errors': errors,
            'capture_devices': [r['identity']['device'] for r in records if r['capture_candidate']],
            'note': 'Read-only UVC discovery; no stream, package, service or motion changes.'}


def validate_request(request):
    for name, low, high in (('width', 1, 8192), ('height', 1, 8192),
                            ('frames', 2, 1800), ('warmup_frames', 0, 120)):
        if type(request.get(name)) is not int or not low <= request[name] <= high:
            raise ValueError('Invalid ' + name)
    if request['width'] * request['height'] > 16777216:
        raise ValueError('Image exceeds bounded capture size')
    fps = request.get('fps')
    if type(fps) not in (int, float) or not math.isfinite(fps) or not 0 < fps <= 120:
        raise ValueError('Invalid fps')
    if request.get('fourcc') not in ('MJPG', 'YUYV'):
        raise ValueError('Select MJPG or YUYV from actual probe')
    device = request.get('device')
    if not isinstance(device, str) or Path(device).parent not in (
            Path('/dev/v4l/by-id'), Path('/dev/v4l/by-path')) or '..' in Path(device).parts:
        raise ValueError('Explicit stable by-id/by-path camera required')


def advertised(record, request):
    return any(mode['fourcc'] == request['fourcc'] and mode['width'] == request['width']
               and mode['height'] == request['height'] and abs(mode['fps'] - request['fps']) < .02
               for mode in record['modes'])


def verify_mode(cap, cv2, request):
    actual = {'width': cap.get(cv2.CAP_PROP_FRAME_WIDTH),
              'height': cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
              'fps': cap.get(cv2.CAP_PROP_FPS), 'fourcc_code': cap.get(cv2.CAP_PROP_FOURCC)}
    if (not all(math.isfinite(v) for v in actual.values()) or
            actual['width'] != request['width'] or actual['height'] != request['height'] or
            abs(actual['fps'] - request['fps']) > max(.5, request['fps'] * .02) or
            actual['fourcc_code'] != cv2.VideoWriter_fourcc(*request['fourcc'])):
        raise RuntimeError('Camera negotiated a different mode: ' + repr(actual))
    return actual


def collect(cap, cv2, request, output):
    import numpy as np
    timestamps, images = [], {}
    sample_indices = {0, request['frames'] // 2, request['frames'] - 1}
    for index in range(-request['warmup_frames'], request['frames']):
        good, image = cap.read()
        received = time.monotonic()
        if not good:
            raise RuntimeError('Camera disconnected or returned no frame')
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or
                image.shape != (request['height'], request['width'], 3)):
            raise RuntimeError('Camera returned unexpected pixel shape/type')
        if index < 0:
            continue
        timestamps.append(received)
        if index in sample_indices:
            images[index] = image.copy()
    # Save after timing, so disk encoding does not inflate inter-frame intervals.
    samples = []
    for index, image in sorted(images.items()):
        path = output / f'frame_{index:04d}.png'
        if not cv2.imwrite(str(path), image):
            raise RuntimeError('Failed to save capture evidence')
        samples.append({'file': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    gaps = np.diff(timestamps)
    rate = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
    return {'frames_read': len(timestamps), 'measured_host_fps': rate,
            'host_gap_ms': {'median': float(np.median(gaps) * 1000),
                            'p95': float(np.percentile(gaps, 95) * 1000),
                            'max': float(max(gaps) * 1000)},
            'samples': samples, 'source_timestamp_verified': False,
            'timestamp_origin': 'host_receipt_only', 'stereo_sync_verified': False,
            'metric_calibration_verified': False}


def capture_worker(request_path):
    request_path = Path(request_path)
    request = json.loads(request_path.read_text(encoding='utf-8'))
    validate_request(request)
    record = inspect_device(request['device'])
    if not record['capture_candidate']:
        raise ValueError('Metadata node is not a video capture device')
    if not advertised(record, request):
        raise ValueError('Requested mode is not explicitly advertised by this device')
    import cv2
    cap = cv2.VideoCapture(request['device'], cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open selected UVC capture node')
        if select_device(request['device']) != record['identity']:
            raise RuntimeError('USB identity changed while opening camera')
        for key, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*request['fourcc'])),
                           (cv2.CAP_PROP_FRAME_WIDTH, request['width']),
                           (cv2.CAP_PROP_FRAME_HEIGHT, request['height']),
                           (cv2.CAP_PROP_FPS, request['fps'])):
            cap.set(key, value)
        negotiated = verify_mode(cap, cv2, request)
        measurements = collect(cap, cv2, request, request_path.parent)
    finally:
        cap.release()
    success = measurements['measured_host_fps'] >= request['fps'] * .8
    summary = {'schema_version': 1, 'success': success, 'device_probe': record,
               'requested': request, 'negotiated': negotiated, 'opencv_version': cv2.__version__,
               **measurements, 'reason': 'capture_verified' if success else 'host_rate_below_80_percent'}
    write_json(request_path.parent / 'summary.json', summary)
    if success:
        write_json(request_path.parent / 'camera.candidate.json', {
            'enabled': False, 'backend': 'v4l2', 'device': request['device'],
            'expected_size': [request['width'], request['height']], 'fps': request['fps'],
            'fourcc': request['fourcc'], 'warmup_frames': request['warmup_frames'],
            'storage': '/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1/camera_frames'})
    return 0 if success else 2


def dynamic_frame_rate(device, value=None):
    """Use the existing driver control; accept only this one verified boolean."""
    if value is not None:
        if type(value) is not int or value not in (0, 1):
            raise ValueError('Dynamic frame rate must be 0 or 1')
        ctl(device, '--set-ctrl=exposure_dynamic_framerate=' + str(value))
        return value
    text = ctl(device, '--get-ctrl=exposure_dynamic_framerate')
    match = re.search(r'(?m)^exposure_dynamic_framerate:\s*([01])\s*$', text)
    if not match:
        raise RuntimeError('Cannot read back dynamic frame rate control')
    return int(match[1])


def capture_session(request, output, timeout, fixed_frame_rate=False):
    validate_request(request)
    if type(fixed_frame_rate) is not bool:
        raise ValueError('fixed_frame_rate must be boolean')
    if not math.isfinite(timeout) or not 1 <= timeout <= 300:
        raise ValueError('Capture timeout must be 1..300 seconds')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    request_path = output / 'request.json'
    write_json(request_path, request)
    result = {'schema_version': 1, 'success': False, 'output': str(output),
              'active_config_changed': False, 'motion_commands_sent': False}
    original_control = None
    try:
        if fixed_frame_rate:
            identity = select_device(request['device'])
            if not query_capability(identity['node'])['video_capture']:
                raise ValueError('Metadata node is not a video capture device')
            original_control = dynamic_frame_rate(request['device'])
            result['original_dynamic_frame_rate'] = original_control
            dynamic_frame_rate(request['device'], 0)
            if dynamic_frame_rate(request['device']) != 0:
                raise RuntimeError('Dynamic frame rate write was not accepted')
            result['capture_uvc_controls'] = {'exposure_dynamic_framerate': 0}
        worker = subprocess.run([sys.executable, '-m', 'ai_coordinator.uvc_prepare',
                                 '_capture', str(request_path)], capture_output=True,
                                text=True, timeout=timeout)
        result.update(success=worker.returncode == 0, returncode=worker.returncode,
                      stdout=worker.stdout, stderr=worker.stderr)
    except subprocess.TimeoutExpired:
        # subprocess.run kills and reaps its child, including a blocked native read.
        result.update(reason='capture_timeout', timeout_s=timeout)
    except (OSError, ValueError, RuntimeError) as exc:
        result.update(reason=type(exc).__name__ + ': ' + str(exc))
    finally:
        if original_control is not None:
            try:
                dynamic_frame_rate(request['device'], original_control)
                result['uvc_control_restored'] = dynamic_frame_rate(request['device']) == original_control
                if not result['uvc_control_restored']:
                    result.update(success=False, reason='uvc_control_restore_mismatch')
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                result.update(success=False, uvc_control_restored=False,
                              reason='uvc_control_restore_failed: ' + str(exc))
    if (output / 'summary.json').is_file():
        summary = json.loads((output / 'summary.json').read_text(encoding='utf-8'))
        result['frames_read'] = summary['frames_read']
        result['measured_host_fps'] = summary['measured_host_fps']
        if not summary['success']:
            result.setdefault('reason', summary['reason'])
    if result['success']:
        write_json(output / 'camera.preconditions.json', {
            'required_uvc_controls': result.get('capture_uvc_controls', {}),
            'controls_applied_by_existing_camera_worker': False,
            'source_timestamp_verified': False, 'metric_calibration_verified': False,
            'motion_authorized': False,
            'note': 'Disabled camera candidate only. The existing worker does not apply these controls; '
                    'verify/control them explicitly before relying on this measured rate.'})
    write_json(output / 'result.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    probe = commands.add_parser('probe', help='Read UVC capabilities; do not open a stream')
    probe.add_argument('--out', type=Path, required=True)
    capture = commands.add_parser('capture', help='Capture in a child with a hard timeout')
    capture.add_argument('--device', required=True)
    capture.add_argument('--width', type=int, required=True)
    capture.add_argument('--height', type=int, required=True)
    capture.add_argument('--fps', type=float, required=True)
    capture.add_argument('--fourcc', choices=('MJPG', 'YUYV'), required=True)
    capture.add_argument('--frames', type=int, default=150)
    capture.add_argument('--warmup-frames', type=int, default=10)
    capture.add_argument('--timeout', type=float, default=30)
    capture.add_argument('--fixed-frame-rate', action='store_true',
                         help='Temporarily disable dynamic frame rate and restore it even on timeout')
    capture.add_argument('--out', type=Path, required=True)
    worker = commands.add_parser('_capture', help=argparse.SUPPRESS)
    worker.add_argument('request')
    args = parser.parse_args(argv)
    if args.command == '_capture':
        return capture_worker(args.request)
    if args.command == 'probe':
        if args.out.exists():
            raise FileExistsError('Retain previous probe evidence')
        result = discover()
        write_json(args.out, result)
        print(json.dumps({'capture_devices': result['capture_devices'], 'out': str(args.out)}))
        return 0 if result['capture_devices'] else 2
    request = {key: getattr(args, key) for key in
               ('device', 'width', 'height', 'fps', 'fourcc', 'frames', 'warmup_frames')}
    result = capture_session(request, args.out, args.timeout, args.fixed_frame_rate)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
