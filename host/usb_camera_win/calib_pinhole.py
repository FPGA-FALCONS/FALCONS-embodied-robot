"""本项目 USB 相机（0bda:5856）的棋盘格针孔标定。

与原模板 pinhole_calib.py / camera_capture.py 的区别：

1. 相机不写死 index（模板里是 index=3 / 1280x800@120），改由 DirectShow 自动挑选
   "支持对焦控制"的那只，本项目为 0bda:5856（内置摄像头不支持，会被跳过）。
2. 采图前锁定对焦，按需锁定曝光；实际读回数值写进结果文件，
   后续推理必须使用同一 focus 值。
3. 采集不设门槛：预览里按空格就直接拍，检测到角点就进标定，没检测到只存图并提示；
   覆盖率/清晰度/姿态重叠只作为画面提示，不阻止采集（自动采集模式才静默跳过无效帧）。
4. 输出三份：npz、OpenCV YAML（RK3588 板端可直接读）、JSON 摘要（含逐图重投影误差）。
5. 无 GUI 也能跑：--folder 批量标定、--auto-collect N 自动采集、--no-window 只存不出图。

用法
----
    python calib_pinhole.py                            # 预览 + 空格采集 + c 标定
    python calib_pinhole.py --auto-collect 15          # 无窗口自动采集 15 张并标定
    python calib_pinhole.py --folder calib_images      # 对已有图片批量标定
    python calib_pinhole.py --detect-pattern           # 只探测棋盘格内角点行列数

按键：空格=采集  c=标定  r=重标定（剔除离群）  d=删最后一张  ESC/q=退出
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    from lock_focus import BACKENDS, autofocus_state, lock_focus, open_capture, read_state
except ImportError:  # pragma: no cover
    sys.exit("lock_focus.py 必须与本脚本放在同一目录（相机打开/锁焦逻辑在那边）")

IMAGE_EXTS = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")
DETECT_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                + cv2.CALIB_CB_FAST_CHECK)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)


# ---------------------------------------------------------------- 棋盘格

def board_points(pattern, square_mm):
    """内角点 3D 坐标，z=0，单位 mm。pattern=(cols, rows)，与 findChessboardCorners 同序。"""
    cols, rows = pattern
    objp = np.zeros((cols * rows, 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_mm)
    return objp


def detect_corners(gray, pattern, subpix=True):
    ok, corners = cv2.findChessboardCorners(gray, pattern, DETECT_FLAGS)
    if ok and subpix:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
    return ok, corners


def frame_quality(gray, corners):
    """覆盖率、清晰度、外接框，用于门控与去重。"""
    pts = corners.reshape(-1, 2)
    x0, x1 = float(pts[:, 0].min()), float(pts[:, 0].max())
    y0, y1 = float(pts[:, 1].min()), float(pts[:, 1].max())
    h, w = gray.shape[:2]
    return {
        "coverage": (x1 - x0) * (y1 - y0) / float(w * h),
        "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "bbox": [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)],
        "center": (round((x0 + x1) / 2 / w, 3), round((y0 + y1) / 2 / h, 3)),
        "scale": round(max(x1 - x0, y1 - y0) / w, 3),
    }


def pose_cell(center):
    """把外接框中心归一化坐标落到 3x3 网格。"""
    gx = min(2, max(0, int(center[0] * 3)))
    gy = min(2, max(0, int(center[1] * 3)))
    return gy, gx


def draw_pose_panel(display, cells):
    """右上角画 3x3 姿态覆盖图：数字=该区域已采集张数，帮助用户补全姿态。"""
    h, w = display.shape[:2]
    side = 150
    x0, y0 = w - side - 20, 20
    overlay = display.copy()
    cv2.rectangle(overlay, (x0 - 8, y0 - 8), (x0 + side + 8, y0 + side + 8), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.35, display, 0.65, 0, display)
    cell = side // 3
    counts = {}
    for gy, gx in cells:
        counts[(gy, gx)] = counts.get((gy, gx), 0) + 1
    for gy in range(3):
        for gx in range(3):
            n = counts.get((gy, gx), 0)
            color = (0, 200, 0) if n else (110, 110, 110)
            cv2.rectangle(display, (x0 + gx * cell, y0 + gy * cell),
                          (x0 + (gx + 1) * cell, y0 + (gy + 1) * cell), color, 2)
            if n:
                cv2.putText(display, str(n), (x0 + gx * cell + 8, y0 + gy * cell + 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(display, "pose 3x3", (x0, y0 - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1)


def is_duplicate(corners, kept, min_shift_px):
    """角点平均位移小于阈值视为同一姿态重复帧。"""
    for prev in kept:
        if prev.shape != corners.shape:
            continue
        shift = float(np.linalg.norm(prev.reshape(-1, 2) - corners.reshape(-1, 2), axis=1).mean())
        if shift < min_shift_px:
            return True
    return False


def assess(gray, corners, kept, args):
    """只评估、不拦截：返回 (质量字典, 提示列表)。

    提示只用于画面显示和日志；采集不看这些阈值，按空格就一定拍。
    """
    q = frame_quality(gray, corners)
    notes = []
    if q["coverage"] < args.min_coverage:
        notes.append(f"覆盖率偏低 {q['coverage']*100:.1f}%")
    if q["sharpness"] < args.min_sharpness:
        notes.append(f"清晰度偏低 {q['sharpness']:.0f}")
    if is_duplicate(corners, kept, args.min_shift):
        notes.append("姿态与已有帧接近")
    return q, notes


# ---------------------------------------------------------------- 标定

def calibrate(objpoints, imgpoints, img_size, fix_k3=False):
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
    return cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None,
                               flags=flags, criteria=criteria)


def per_view_errors(objpoints, imgpoints, K, D, rvecs, tvecs):
    errs = []
    for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
        diff = proj.reshape(-1, 2) - imgp.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
    return errs


def calibrate_robust(objpoints, imgpoints, img_size, max_outlier_px, fix_k3=False):
    """标准标定 + 逐图误差离群剔除，返回 (rms, K, D, rvecs, tvecs, errors, dropped)。"""
    keep = list(range(len(objpoints)))
    dropped = []
    while True:
        objs = [objpoints[i] for i in keep]
        imgs = [imgpoints[i] for i in keep]
        if len(objs) < 4:
            raise RuntimeError("有效图片少于 4 张，无法标定")
        rms, K, D, rvecs, tvecs = calibrate(objs, imgs, img_size, fix_k3)
        errs = per_view_errors(objs, imgs, K, D, rvecs, tvecs)
        worst = max(range(len(errs)), key=lambda i: errs[i])
        worst_px = errs[worst]
        med = statistics.median(errs)
        threshold = max(max_outlier_px, 3.0 * med)
        if worst_px <= threshold or len(keep) <= 6:
            break
        dropped.append({"index": keep[worst], "reprojection_px": round(worst_px, 3)})
        print(f"  剔除离群帧 #{keep[worst]}（重投影 {worst_px:.2f}px > {threshold:.2f}px）")
        keep.pop(worst)
        rms, K, D, rvecs, tvecs = calibrate([objpoints[i] for i in keep],
                                            [imgpoints[i] for i in keep],
                                            img_size, fix_k3)
        errs = per_view_errors([objpoints[i] for i in keep],
                               [imgpoints[i] for i in keep], K, D, rvecs, tvecs)
    return rms, K, D, rvecs, tvecs, errs, dropped, keep


# ---------------------------------------------------------------- 输出

def save_results(K, D, rms, img_size, pattern, square_mm, errors, info, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    npz_path = out_dir / "pinhole_calib.npz"
    np.savez(npz_path,
             K=K, D=D, rms=float(rms), img_size=np.array(img_size, dtype=np.int32),
             pattern=np.array(pattern, dtype=np.int32), square_mm=float(square_mm),
             focus=float(info.get("focus") if info.get("focus") is not None else -1.0),
             fps=float(info.get("fps") if info.get("fps") is not None else -1.0))

    yaml_path = out_dir / "pinhole_calib.yaml"
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(img_size[0]))
    fs.write("image_height", int(img_size[1]))
    fs.write("camera_matrix", K)
    fs.write("distortion_coefficients", D)
    fs.write("rms_error", float(rms))
    fs.write("focus_value", float(info.get("focus") if info.get("focus") is not None else -1.0))
    fs.write("pattern_cols", int(pattern[0]))
    fs.write("pattern_rows", int(pattern[1]))
    fs.write("square_mm", float(square_mm))
    fs.release()

    summary = {
        "timestamp": stamp,
        "camera": {k: info.get(k) for k in
                   ("camera_index", "backend", "width", "height", "fps", "focus", "exposure")},
        "board": {"inner_cols": pattern[0], "inner_rows": pattern[1], "square_mm": square_mm},
        "views_used": len(errors),
        "rms_px": round(float(rms), 4),
        "per_view_rms_px": [round(e, 4) for e in errors],
        "camera_matrix": K.tolist(),
        "distortion": D.flatten().tolist(),
        "note": "内参与采集时的 focus/分辨率绑定；推理必须使用同一 focus 值与分辨率。",
        "files": {"npz": str(npz_path), "yaml": str(yaml_path)},
    }
    json_path = out_dir / "pinhole_calib.json"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    summary["files"]["json"] = str(json_path)
    return summary


def save_undistort_preview(frame, K, D, out_path):
    h, w = frame.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1.0, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_16SC2)
    undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
    gap = np.zeros((h, 6, 3), dtype=np.uint8)
    canvas = np.hstack([frame, gap, undistorted])
    cv2.putText(canvas, "original", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(canvas, "undistorted", (w + 26, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), canvas)
    return str(out_path)


# ---------------------------------------------------------------- 相机侧

def find_camera(args):
    """自动挑支持对焦控制的相机；返回 (cap, index)。"""
    for index in range(args.max_index):
        if args.index is not None and index != args.index:
            continue
        cap = open_capture(index, BACKENDS["dshow"], args.width, args.height,
                           args.fps, args.fourcc)
        if cap is None:
            continue
        if cap.get(cv2.CAP_PROP_FOCUS) >= 0:
            return cap, index
        cap.release()
    return None, None


def apply_locks(cap, args):
    """锁焦 + 尽力锁曝光/白平衡，返回记录用的字典。"""
    info = {}
    if args.focus is not None:
        info["lock"] = lock_focus(cap, args.focus, args.settle)
        info["focus"] = float(cap.get(cv2.CAP_PROP_FOCUS))
    else:
        info["lock"] = {"skipped": "未指定 --focus，使用相机当前对焦（不推荐）"}
        info["focus"] = float(cap.get(cv2.CAP_PROP_FOCUS))

    if args.exposure is not None:
        # DirectShow: 0.25 = manual, 0.75 = auto（OpenCV 后端约定）
        ok = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        ok2 = cap.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
        info["exposure"] = {"requested": args.exposure,
                            "set_ret": [bool(ok), bool(ok2)],
                            "readback": cap.get(cv2.CAP_PROP_EXPOSURE)}
    else:
        info["exposure"] = None
    return info


def detect_pattern(cap, candidates):
    """在实时画面上试常见棋盘格内角点尺寸。"""
    hits = []
    frame = None
    for _ in range(8):
        ok, got = cap.read()
        if ok:
            frame = got
    if frame is None:
        return hits
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for cols, rows in candidates:
        ok, _ = cv2.findChessboardCorners(gray, (cols, rows), DETECT_FLAGS)
        if not ok:
            ok, _ = cv2.findChessboardCornersSB(gray, (cols, rows))
        if ok:
            hits.append((cols, rows))
    return hits


# ---------------------------------------------------------------- 采集

def run_capture(args, cap, camera_index, lock_info):
    if args.min_shift is None:
        args.min_shift = 20.0 if args.auto_collect else 40.0
    pattern = (args.cols, args.rows)
    objp = board_points(pattern, args.size)
    objpoints, imgpoints, kept_corners = [], [], []
    cells = []
    image_dir = Path(args.save_dir)
    image_dir.mkdir(parents=True, exist_ok=True)

    print(f"棋盘格: {pattern[0]}x{pattern[1]} 内角点，方格 {args.size}mm")
    print(f"相机: index={camera_index}, {lock_info.get('width')}x{lock_info.get('height')}"
          f"@{lock_info.get('fps')}fps, focus={lock_info.get('focus')}")
    print(f"提示阈值（只给提示，不拦截）: 覆盖率 {args.min_coverage*100:.0f}%  "
          f"清晰度 {args.min_sharpness:.0f}  姿态位移 {args.min_shift}px")
    print("空格=直接拍  c=标定  r=重标定  d=删最后一张  ESC/q=退出")
    if args.auto_collect:
        print(f"自动采集模式：请缓慢平移/旋转棋盘格，目标 {args.auto_collect} 张，"
              f"超时 {args.auto_timeout:.0f}s")

    if not args.no_window:
        cv2.namedWindow("calib", cv2.WINDOW_NORMAL)

    notes = []
    auto_deadline = time.time() + args.auto_timeout if args.auto_collect else None

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("读取失败，退出")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = detect_corners(gray, pattern)
        display = frame.copy()

        q, notes = None, []
        if found:
            q, notes = assess(gray, corners, kept_corners, args)
            cv2.drawChessboardCorners(display, pattern, corners, True)
            cv2.putText(display, f"coverage {q['coverage']*100:.1f}%  sharp {q['sharpness']:.0f}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display, f"center {q['center']}  scale {q['scale']}",
                        (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if notes:
                cv2.putText(display, " / ".join(notes), (20, 104),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        else:
            cv2.putText(display, "NO BOARD - SPACE still saves the frame",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.putText(display, f"collected {len(objpoints)}", (20, display.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        if not args.no_window:
            draw_pose_panel(display, cells)
            cv2.imshow("calib", display)

        key = 255 if (args.no_window or args.auto_collect) else (cv2.waitKey(1) & 0xFF)
        want_grab = key in (32, ord(" ")) or bool(args.auto_collect)
        if args.auto_collect and want_grab:
            # 自动模式无人工把关，只静默跳过没有角点或姿态重复的帧
            want_grab = found and not is_duplicate(corners, kept_corners, args.min_shift)

        if want_grab:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:21]
            path = image_dir / f"calib_{stamp}.png"
            cv2.imwrite(str(path), frame)
            if found:
                objpoints.append(objp)
                imgpoints.append(corners)
                kept_corners.append(corners.copy())
                cells.append(pose_cell(q["center"]))
                hint = f"（{'; '.join(notes)}）" if notes else ""
                print(f"  [{len(objpoints)}] 已拍 {path.name}  "
                      f"覆盖率 {q['coverage']*100:.1f}%  清晰度 {q['sharpness']:.0f}{hint}")
                if len(objpoints) >= 6 and cells.count(cells[-1]) >= max(3, len(cells) // 2):
                    print("  提示：姿态过于集中在一个区域，换个位置/距离/倾角再拍")
                if args.auto_collect and len(objpoints) >= args.auto_collect:
                    print(f"已采集 {len(objpoints)} 张，开始标定")
                    break
            else:
                print(f"  已拍 {path.name}（无角点，仅存图，不计入标定）")

        if key == ord("c") or (args.auto_collect and len(objpoints) >= args.auto_collect):
            if len(objpoints) < 5:
                print(f"至少需要 5 张，当前 {len(objpoints)} 张")
            else:
                do_calibrate(args, objpoints, imgpoints, pattern, lock_info, camera_index)
        elif key == ord("r"):
            if len(objpoints) >= 5:
                do_calibrate(args, objpoints, imgpoints, pattern, lock_info, camera_index,
                             robust=True)
        elif key == ord("d"):
            if objpoints:
                objpoints.pop(); imgpoints.pop(); kept_corners.pop()
                if cells:
                    cells.pop()
                print(f"  删除最后一张，剩 {len(objpoints)} 张")
        elif key == 27 or key == ord("q"):
            break

        if auto_deadline is not None and time.time() > auto_deadline:
            print("自动采集超时：整块棋盘格未能稳定入画，先摆好再跑")
            break
        if args.auto_collect and not args.no_window:
            time.sleep(0.02)

    if not args.no_window:
        cv2.destroyAllWindows()

    if args.auto_collect and len(objpoints) >= 5:
        do_calibrate(args, objpoints, imgpoints, pattern, lock_info, camera_index)
    return objpoints, imgpoints


def do_calibrate(args, objpoints, imgpoints, pattern, lock_info, camera_index, robust=False):
    img_size = (lock_info["width"], lock_info["height"])
    if not robust:
        rms, K, D, rvecs, tvecs = calibrate(objpoints, imgpoints, img_size, args.fix_k3)
        errors = per_view_errors(objpoints, imgpoints, K, D, rvecs, tvecs)
        dropped, used = [], list(range(len(objpoints)))
    else:
        rms, K, D, rvecs, tvecs, errors, dropped, used = calibrate_robust(
            objpoints, imgpoints, img_size, args.outlier_px, args.fix_k3)

    info = {
        "camera_index": camera_index,
        "backend": "dshow",
        "width": img_size[0],
        "height": img_size[1],
        "fps": lock_info.get("fps"),
        "focus": lock_info.get("focus"),
        "exposure": (lock_info.get("exposure") or {}).get("readback"),
    }
    summary = save_results(K, D, rms, img_size, pattern, args.size, errors, info, args.out_dir)

    print(f"\n标定完成  RMS={rms:.4f}px  使用 {len(errors)}/{len(objpoints)} 张"
          f"{'（已剔除离群）' if dropped else ''}")
    print(f"fx={K[0,0]:.3f}  fy={K[1,1]:.3f}  cx={K[0,2]:.3f}  cy={K[1,2]:.3f}")
    print(f"畸变 {np.round(D.flatten(), 5).tolist()}")
    print(f"逐图重投影(px) {[round(e, 3) for e in errors]}")
    print(f"结果: {summary['files']['npz']}\n      {summary['files']['yaml']}"
          f"\n      {summary['files']['json']}")

    samples = sorted(Path(args.save_dir).glob("calib_*.png"))
    if samples:
        frame = cv2.imread(str(samples[0]))
        if frame is not None:
            path = save_undistort_preview(frame, K, D, Path(args.out_dir) / "undistort_check.png")
            print(f"去畸变对比: {path}")
    return summary


# ---------------------------------------------------------------- 文件夹模式

def calibrate_from_folder(args, pattern, objp):
    paths = []
    for ext in IMAGE_EXTS:
        paths.extend(glob.glob(os.path.join(args.folder, ext)))
        paths.extend(glob.glob(os.path.join(args.folder, ext.upper())))
    paths = sorted(set(paths))
    if not paths:
        print(f"文件夹 {args.folder} 中没有图片")
        return None
    print(f"找到 {len(paths)} 张图片")

    objpoints, imgpoints, used_paths = [], [], []
    img_size = None
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  跳过（读不出） {os.path.basename(path)}")
            continue
        size = (img.shape[1], img.shape[0])
        if img_size is not None and size != img_size:
            print(f"  跳过（尺寸不一致 {size} vs {img_size}） {os.path.basename(path)}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = detect_corners(gray, pattern)
        if not found:
            print(f"  FAIL  {os.path.basename(path)}")
            continue
        q = frame_quality(gray, corners)
        print(f"  OK    {os.path.basename(path)}  覆盖率 {q['coverage']*100:.1f}%  "
              f"清晰度 {q['sharpness']:.0f}")
        img_size = size
        objpoints.append(objp)
        imgpoints.append(corners)
        used_paths.append(path)

    if len(objpoints) < 5:
        print(f"只检出 {len(objpoints)} 张，至少需要 5 张")
        return None

    rms, K, D, rvecs, tvecs, errors, dropped, keep = calibrate_robust(
        objpoints, imgpoints, img_size, args.outlier_px, args.fix_k3)
    info = {"camera_index": None, "backend": "folder", "width": img_size[0],
            "height": img_size[1], "fps": None,
            "focus": float(args.focus) if args.focus is not None else None,
            "exposure": None}
    summary = save_results(K, D, rms, img_size, pattern, args.size, errors, info, args.out_dir)

    print(f"\n标定完成  RMS={rms:.4f}px  使用 {len(errors)}/{len(objpoints)} 张")
    print(f"fx={K[0,0]:.3f}  fy={K[1,1]:.3f}  cx={K[0,2]:.3f}  cy={K[1,2]:.3f}")
    print(f"畸变 {np.round(D.flatten(), 5).tolist()}")
    sample = cv2.imread(used_paths[keep[0]])
    if sample is not None:
        out = save_undistort_preview(sample, K, D, Path(args.out_dir) / "undistort_check.png")
        print(f"去畸变对比: {out}")
    print(f"结果: {summary['files']['yaml']}")
    return summary


# ---------------------------------------------------------------- main

def build_parser():
    p = argparse.ArgumentParser(description="本项目 USB 相机棋盘格针孔标定",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--folder", default=None, help="已有图片目录，走批量标定")
    p.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数（默认 9）")
    p.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数（默认 6）")
    p.add_argument("--size", type=float, default=25.0, help="方格边长 mm（默认 25）")
    p.add_argument("--index", type=int, default=None, help="相机索引；默认自动挑支持对焦的")
    p.add_argument("--max-index", type=int, default=5)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--fourcc", default="MJPG")
    p.add_argument("--focus", type=int, default=260, help="锁定对焦位置（默认 260；早期扫描为 285，使用对应标定）")
    p.add_argument("--fix-k3", action=argparse.BooleanOptionalAction, default=True,
                   help="固定 k3=0，估计 k1/k2/p1/p2（默认开启；--no-fix-k3 同时估计 k3）")
    p.add_argument("--exposure", type=float, default=None, help="锁定曝光值（DirectShow log2 秒）")
    p.add_argument("--settle", type=float, default=1.0, help="写入焦距后等待镜头到位")
    p.add_argument("--min-coverage", type=float, default=0.10, help="棋盘格外接框占比下限")
    p.add_argument("--min-sharpness", type=float, default=80.0, help="Laplacian 方差下限")
    p.add_argument("--min-shift", type=float, default=None,
                   help="与已采集帧去重的最小角点位移 px（默认 40，自动采集模式 20）")
    p.add_argument("--outlier-px", type=float, default=1.0, help="离群帧重投影误差阈值")
    p.add_argument("--auto-collect", type=int, default=0, help="自动采集 N 张（无窗口）")
    p.add_argument("--auto-timeout", type=float, default=120.0, help="自动采集超时秒数")
    p.add_argument("--no-window", action="store_true", help="不弹预览窗口")
    p.add_argument("--detect-pattern", action="store_true", help="探测当前画面上的棋盘格尺寸")
    p.add_argument("--save-dir", default="calib_images", help="采集图片保存目录")
    p.add_argument("--out-dir", default="calib_out", help="标定结果目录")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    pattern = (args.cols, args.rows)

    if args.folder:
        return 0 if calibrate_from_folder(args, pattern, board_points(pattern, args.size)) else 2

    cap, camera_index = find_camera(args)
    if cap is None:
        print("没有找到支持对焦控制的相机（内置摄像头会被跳过）；用 --index 指定")
        return 2

    lock_info = apply_locks(cap, args)
    state = read_state(cap)
    lock_info.update(state)
    print(json.dumps({"camera_index": camera_index, "state": state,
                      "focus": lock_info.get("focus")}, ensure_ascii=False))

    if args.detect_pattern:
        candidates = [(9, 6), (8, 6), (7, 5), (10, 7), (8, 5), (11, 8), (6, 4), (9, 7), (12, 9)]
        hits = detect_pattern(cap, candidates)
        print("检测到内角点尺寸:", hits if hits else "无（整块棋盘格需要完整入画）")
        cap.release()
        return 0 if hits else 2

    try:
        run_capture(args, cap, camera_index, lock_info)
    finally:
        cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
