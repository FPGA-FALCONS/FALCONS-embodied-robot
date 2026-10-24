"""Capture UVC frames with OpenCV/V4L2 and report measured FPS.

Run on the RK3588 after the USB camera arrives:
    python3 capture_v4l2.py --index 0 --width 1920 --height 1080 --frames 120
"""
import argparse
import json
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=int, default=0)
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--fourcc', default='MJPG', help='MJPG (recommended) or YUYV')
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--out', type=Path, default=Path('usb_camera_capture'))
    args = parser.parse_args()
    try:
        import cv2
    except Exception as exc:
        raise SystemExit('cv2 unavailable: ' + repr(exc))

    args.out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(args.index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'cannot open /dev/video{args.index}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    # Warm-up; UVC often returns the first frames before exposure settles.
    for _ in range(10):
        cap.read()

    timestamps = []
    saved = 0
    shape = None
    for index in range(args.frames):
        good, frame = cap.read()
        if not good:
            continue
        now = time.monotonic()
        timestamps.append(now)
        shape = list(frame.shape)
        if index in (0, args.frames // 2, args.frames - 1):
            path = args.out / f'frame_{index:04d}.png'
            cv2.imwrite(str(path), frame)
            saved += 1
    cap.release()

    measured = 0.0
    if len(timestamps) >= 2:
        measured = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
    summary = {
        'device': f'/dev/video{args.index}',
        'requested': {'width': args.width, 'height': args.height,
                      'fps': args.fps, 'fourcc': args.fourcc},
        'shape': shape,
        'frames_read': len(timestamps),
        'measured_fps': round(measured, 3),
        'saved_images': saved,
        'output': str(args.out.resolve()),
        'note': 'Measured host receive rate, not a synchronized sensor timestamp.',
    }
    (args.out / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if len(timestamps) >= 2 else 2


if __name__ == '__main__':
    sys.exit(main())
