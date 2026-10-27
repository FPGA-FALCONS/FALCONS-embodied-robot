"""Lock the focus of a USB/UVC camera on Windows (DirectShow backend).

Why DirectShow: with the MSMF backend CAP_PROP_FOCUS writes are accepted but
ignored (read-back stays at a constant value), so autofocus keeps hunting.
DirectShow maps CAP_PROP_AUTOFOCUS/CAP_PROP_FOCUS onto IAMCameraControl and
actually moves the lens.

Examples
--------
    python lock_focus.py --list
    python lock_focus.py --auto-focus            # 自动找当前距离最佳焦距并锁定
    python lock_focus.py --focus 850 --out shot.png
    python lock_focus.py --scan --out-dir scan_20261027
    python lock_focus.py --auto-focus --verify   # 自动定焦后再验证 60 帧不漂移
    python lock_focus.py --preview

Notes
-----
* FOCUS unit is the camera-native UVC range, normally 0..1000 (read back from
  the device, not assumed). Larger usually means farther focus.
* AUTOFOCUS read-back follows DirectShow CameraControl flags: 1 = auto,
  2 = manual. Both 0 and 2 are treated as "autofocus off".
* Physical buttons / vendor apps can re-enable autofocus; re-run after replug.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

try:
    import cv2
except ImportError:  # pragma: no cover
    sys.exit("cv2 not found: python -m pip install opencv-python")

BACKENDS = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}
AUTOFOCUS_OFF = (0, 2)  # V4L2-style off, DirectShow manual
AUTOFOCUS_ON = (1,)


def autofocus_state(value):
    if value in AUTOFOCUS_OFF:
        return "manual"
    if value in AUTOFOCUS_ON:
        return "auto"
    return "unsupported"


def open_capture(index, backend, width, height, fps, fourcc):
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        return None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def read_state(cap):
    return {
        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": round(float(cap.get(cv2.CAP_PROP_FPS)), 2),
        "autofocus_raw": cap.get(cv2.CAP_PROP_AUTOFOCUS),
        "autofocus_state": autofocus_state(cap.get(cv2.CAP_PROP_AUTOFOCUS)),
        "focus_raw": cap.get(cv2.CAP_PROP_FOCUS),
    }


def list_cameras(args):
    rows = []
    for index in range(args.max_index):
        cap = open_capture(index, BACKENDS[args.backend], args.width, args.height,
                           args.fps, args.fourcc)
        if cap is None:
            continue
        state = read_state(cap)
        state["index"] = index
        state["focus_supported"] = state["focus_raw"] >= 0
        rows.append(state)
        cap.release()
    return rows


def pick_camera(args):
    """Open the first camera index that reports focus control."""
    skipped = []
    for index in range(args.max_index):
        if args.index is not None and index != args.index:
            continue
        cap = open_capture(index, BACKENDS[args.backend], args.width, args.height,
                           args.fps, args.fourcc)
        if cap is None:
            skipped.append({"index": index, "reason": "open_failed"})
            continue
        focus = cap.get(cv2.CAP_PROP_FOCUS)
        if focus < 0:
            skipped.append({"index": index, "reason": "no_focus_control",
                            "focus_raw": focus})
            cap.release()
            continue
        return cap, index, skipped
    return None, None, skipped


def lock_focus(cap, focus, settle):
    log = {}
    log["set_autofocus_ret"] = bool(cap.set(cv2.CAP_PROP_AUTOFOCUS, 0))
    time.sleep(0.15)
    raw = cap.get(cv2.CAP_PROP_AUTOFOCUS)
    log["autofocus_raw"] = raw
    log["autofocus_state"] = autofocus_state(raw)
    log["set_focus_ret"] = bool(cap.set(cv2.CAP_PROP_FOCUS, focus))
    time.sleep(settle)  # let the lens motor reach position
    readback = cap.get(cv2.CAP_PROP_FOCUS)
    log["focus_requested"] = focus
    log["focus_readback"] = readback
    log["focus_matches"] = abs(readback - focus) <= 2.0
    return log


def grab_frames(cap, count, warmup):
    for _ in range(warmup):
        cap.read()
    frames = []
    for _ in range(count):
        ok, frame = cap.read()
        if ok:
            frames.append(frame)
    return frames


def sharpness(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def save_best(frames, path):
    if not frames:
        return None
    best = max(frames, key=sharpness)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), best)
    return {"path": str(path), "sharpness": round(sharpness(best), 2)}


def sweep(cap, lo, hi, step, args):
    rows = []
    values = list(range(lo, hi + 1, step))
    best = None
    for value in values:
        log = lock_focus(cap, value, args.settle)
        frames = grab_frames(cap, args.frames, args.warmup)
        score = round(statistics.fmean([sharpness(f) for f in frames]), 2) if frames else None
        row = {"focus": value, "readback": log["focus_readback"],
               "matches": log["focus_matches"], "sharpness": score}
        rows.append(row)
        if score is not None and (best is None or score > best["sharpness"]):
            best = {"focus": value, "sharpness": score}
        print(f"  focus={value:4d} readback={log['focus_readback']:7.1f} "
              f"sharpness={score}")
    return rows, best


def scan(cap, args):
    return sweep(cap, args.scan_from, args.scan_to, args.scan_step, args)


def auto_focus(cap, args):
    """粗扫全局 -> 峰值附近细扫 -> 锁定最优点，返回完整报告。

    这是"自动定焦"：一条命令找当前工作距离的最佳焦点并写进相机，
    不需要先 --scan 再手动抄峰值。
    """
    print(f"粗扫 {args.scan_from}..{args.scan_to} step {args.coarse_step} ...")
    coarse_rows, coarse_best = sweep(cap, args.scan_from, args.scan_to,
                                     args.coarse_step, args)
    if coarse_best is None:
        raise RuntimeError("粗扫没有读到任何清晰度，先确认相机能出图")

    lo = max(args.scan_from, coarse_best["focus"] - args.fine_window)
    hi = min(args.scan_to, coarse_best["focus"] + args.fine_window)
    print(f"细扫 {lo}..{hi} step {args.fine_step}"
          f"（粗扫峰值 focus={coarse_best['focus']} sharpness={coarse_best['sharpness']}）...")
    fine_rows, fine_best = sweep(cap, lo, hi, args.fine_step, args)
    if fine_best is None:
        fine_rows, fine_best = [], coarse_best

    target = fine_best["focus"]
    lock = lock_focus(cap, target, args.settle)
    frames = grab_frames(cap, args.frames, args.warmup)
    locked_score = round(statistics.fmean([sharpness(f) for f in frames]), 2) if frames else None
    print(f"锁定 focus={target} readback={lock['focus_readback']} "
          f"sharpness={locked_score}（粗扫峰值 {coarse_best['sharpness']}）")

    return {
        "coarse": {"from": args.scan_from, "to": args.scan_to,
                   "step": args.coarse_step, "rows": coarse_rows},
        "fine": {"from": lo, "to": hi, "step": args.fine_step, "rows": fine_rows},
        "best": {"focus": target, "sharpness": fine_best["sharpness"]},
        "lock": lock,
        "locked_sharpness": locked_score,
    }


def preview(cap, args):
    focus = int(cap.get(cv2.CAP_PROP_FOCUS))
    focus = max(0, focus)
    print("keys: a/d -25/+25, j/l +-100, s save, q quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        text = f"focus={focus}  autofocus={autofocus_state(cap.get(cv2.CAP_PROP_AUTOFOCUS))}"
        cv2.putText(frame, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("lock_focus", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        step = {"a": -25, "d": 25, "j": -100, "l": 100}.get(chr(key), 0)
        if step:
            focus = max(0, focus + step)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap.set(cv2.CAP_PROP_FOCUS, focus)
            time.sleep(0.1)
        elif key == ord("s"):
            path = Path(args.out)
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), frame)
            print(f"saved {path}")
    cv2.destroyAllWindows()


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true", help="enumerate cameras and focus support")
    p.add_argument("--index", type=int, default=None,
                   help="camera index; default auto-pick the one with focus control")
    p.add_argument("--max-index", type=int, default=5)
    p.add_argument("--backend", choices=sorted(BACKENDS), default="dshow")
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--fourcc", default="MJPG", help="MJPG recommended for USB2.0")
    p.add_argument("--focus", type=int, default=None,
                   help="target focus value in the camera-native range (0..1000)")
    p.add_argument("--scan", action="store_true", help="sweep focus and report sharpness")
    p.add_argument("--auto-focus", "--autofocus", dest="auto_focus", action="store_true",
                   help="自动定焦：粗扫+细扫找当前距离的最佳焦点并锁定（不是打开相机内置AF）")
    p.add_argument("--coarse-step", type=int, default=50, help="自动定焦粗扫步长")
    p.add_argument("--fine-step", type=int, default=5, help="自动定焦细扫步长")
    p.add_argument("--fine-window", type=int, default=50, help="细扫在粗扫峰值两侧的范围")
    p.add_argument("--scan-from", type=int, default=0)
    p.add_argument("--scan-to", type=int, default=1000)
    p.add_argument("--scan-step", type=int, default=50)
    p.add_argument("--preview", action="store_true", help="interactive focus adjust window")
    p.add_argument("--frames", type=int, default=5)
    p.add_argument("--verify", action="store_true",
                   help="after locking, report sharpness stability over N frames")
    p.add_argument("--verify-frames", type=int, default=30)
    p.add_argument("--verify-skip", type=int, default=15,
                   help="frames dropped before verify, lens motor needs to settle")
    p.add_argument("--warmup", type=int, default=6)
    p.add_argument("--settle", type=float, default=0.6)
    p.add_argument("--out", default="focus_locked.png")
    p.add_argument("--out-dir", default=None, help="directory for --scan images")
    p.add_argument("--json-out", default=None)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    backend = BACKENDS[args.backend]

    if args.list:
        rows = list_cameras(args)
        print(json.dumps(rows, indent=2, ensure_ascii=False))
        return 0 if rows else 2

    cap, index, skipped = pick_camera(args)
    if cap is None:
        print(json.dumps({"error": "no camera with focus control", "skipped": skipped},
                         indent=2, ensure_ascii=False))
        return 2

    state = read_state(cap)
    print(json.dumps({"camera_index": index, "backend": args.backend, **state},
                     indent=2, ensure_ascii=False))

    result = {"camera_index": index, "backend": args.backend, "state": state}

    if args.preview:
        preview(cap, args)
        cap.release()
        return 0

    if args.auto_focus:
        print("自动定焦：粗扫 -> 细扫 -> 锁定 ...")
        try:
            report = auto_focus(cap, args)
        except RuntimeError as exc:
            print(json.dumps({"error": str(exc)}, ensure_ascii=False))
            cap.release()
            return 2
        result["auto_focus"] = report
        result["best"] = report["best"]
        frames = grab_frames(cap, args.frames, args.warmup)
        result["image"] = save_best(frames, Path(args.out))
        print(f"最佳 focus={report['best']['focus']}，已写入设备；"
              f"图片 {result['image']['path'] if result['image'] else '未保存'}")
        print(f"以后固定复现用: --focus {report['best']['focus']}")

    if args.scan:
        print("scanning focus ...")
        rows, best = scan(cap, args)
        result["scan"] = rows
        result["best"] = best
        if args.out_dir:
            out_dir = Path(args.out_dir)
            if best is not None:
                lock_focus(cap, best["focus"], args.settle)
                frames = grab_frames(cap, args.frames, args.warmup)
                result["best_image"] = save_best(frames, out_dir / f"focus_{best['focus']:04d}.png")
            for row in rows:
                lock_focus(cap, row["focus"], args.settle)
                frames = grab_frames(cap, args.frames, args.warmup)
                save_best(frames, out_dir / f"focus_{row['focus']:04d}.png")
    if args.focus is not None:
        result["lock"] = lock_focus(cap, args.focus, args.settle)
        frames = grab_frames(cap, args.frames, args.warmup)
        result["image"] = save_best(frames, Path(args.out))
        print(json.dumps(result["lock"], indent=2, ensure_ascii=False))

    if args.verify:
        # Proves the locked lens no longer drifts: the device focus register must
        # stay constant while frames keep arriving.
        for _ in range(args.verify_skip):
            cap.read()
        frames, track = [], []
        for _ in range(args.verify_frames):
            ok, frame = cap.read()
            if not ok:
                continue
            frames.append(frame)
            track.append(cap.get(cv2.CAP_PROP_FOCUS))
        scores = [sharpness(f) for f in frames]
        if scores and track:
            motion = [float(cv2.absdiff(a, b).mean())
                      for a, b in zip(frames, frames[1:])]
            result["verify"] = {
                "frames": len(scores),
                "focus_track_min": min(track),
                "focus_track_max": max(track),
                "focus_stable": (max(track) - min(track)) <= 1.0,
                "sharpness_mean": round(statistics.fmean(scores), 2),
                "sharpness_std": round(statistics.pstdev(scores), 2),
                "scene_motion": round(statistics.fmean(motion), 2) if motion else None,
                "note": "focus_stable is the direct proof; sharpness_std also "
                        "moves with scene motion, not only with focus drift",
            }

    cap.release()
    if args.json_out:
        Path(args.json_out).write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
