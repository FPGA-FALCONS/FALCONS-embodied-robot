"""calib_pinhole.py 的自检：用已知内参合成棋盘格图，检查标定能否还原真值。

不碰真实相机，只验证「检测 → 标定 → 输出」这条链路和 OpenCV 绑定是否正常。

用法：python calib_selftest.py
通过返回 0，失败返回 1。
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

from calib_pinhole import board_points, calibrate_robust, detect_corners

W, H = 1920, 1080
COLS, ROWS, SQ_MM = 9, 6, 25.0
K_TRUE = np.array([[1200.0, 0.0, 960.0], [0.0, 1200.0, 540.0], [0.0, 0.0, 1.0]])
D_TRUE = np.array([-0.08, 0.05, 0.001, -0.0015, -0.01])


def build_template():
    scale = 8
    cols, rows = COLS + 1, ROWS + 1
    tw, th = int(cols * SQ_MM * scale), int(rows * SQ_MM * scale)
    img = np.full((th, tw), 255, np.uint8)
    sq = int(SQ_MM * scale)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                img[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 0
    return img, scale


def synth_images(out_dir, count=12):
    template, scale = build_template()
    board = board_points((COLS, ROWS), SQ_MM)
    mapx, mapy = cv2.initInverseRectificationMap(
        K_TRUE, D_TRUE, np.eye(3), K_TRUE, (W, H), cv2.CV_32FC1)
    rng = np.random.default_rng(7)
    src = np.float32([[(SQ_MM + c * SQ_MM) * scale, (SQ_MM + r * SQ_MM) * scale]
                      for r in range(ROWS) for c in range(COLS)])
    saved = []
    for _ in range(count * 4):
        if len(saved) >= count:
            break
        rot = np.deg2rad(rng.uniform([-22, -22, -12], [22, 22, 12]))
        tvec = np.array([rng.uniform(-90, 90), rng.uniform(-60, 60), rng.uniform(330, 560)])
        ideal, _ = cv2.projectPoints(board, rot, tvec, K_TRUE, np.zeros(5))
        dst = ideal.reshape(-1, 2).astype(np.float32)
        if dst.min() < 5 or dst[:, 0].max() > W - 5 or dst[:, 1].max() > H - 5:
            continue
        homography, _ = cv2.findHomography(src, dst)
        warped = cv2.warpPerspective(template, homography, (W, H), borderValue=255)
        mask = cv2.warpPerspective(np.full_like(template, 255), homography, (W, H),
                                   flags=cv2.INTER_NEAREST, borderValue=0)
        canvas = np.full((H, W), 255, np.uint8)
        canvas[mask > 0] = warped[mask > 0]
        distorted = cv2.remap(canvas, mapx, mapy, cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=255)
        distorted = cv2.GaussianBlur(distorted, (3, 3), 0.6)
        noisy = np.clip(distorted.astype(np.float32) + rng.normal(0, 2.0, distorted.shape),
                        0, 255).astype(np.uint8)
        path = out_dir / f"synth_{len(saved):02d}.png"
        cv2.imwrite(str(path), noisy)
        saved.append(path)
    return saved


def main():
    tmp = Path(tempfile.mkdtemp(prefix="calib_selftest_"))
    print(f"合成图目录: {tmp}")
    paths = synth_images(tmp)
    board = board_points((COLS, ROWS), SQ_MM)
    objs, imgs = [], []
    for path in paths:
        img = cv2.imread(str(path))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = detect_corners(gray, (COLS, ROWS))
        if ok:
            objs.append(board)
            imgs.append(corners)
    print(f"生成 {len(paths)} 张，角点检测成功 {len(objs)} 张")
    if len(objs) < 6:
        print("FAIL: 检测成功数不足 6 张，链路异常")
        return 1

    rms, K, D, rvecs, tvecs, errors, dropped, keep = calibrate_robust(
        objs, imgs, (W, H), 1.0)
    fx_err = abs(K[0, 0] - K_TRUE[0, 0]) / K_TRUE[0, 0]
    fy_err = abs(K[1, 1] - K_TRUE[1, 1]) / K_TRUE[1, 1]
    cx_err = abs(K[0, 2] - K_TRUE[0, 2])
    cy_err = abs(K[1, 2] - K_TRUE[1, 2])
    k1_err = abs(D.flatten()[0] - D_TRUE[0])

    print(f"\n真值 fx={K_TRUE[0,0]:.1f} fy={K_TRUE[1,1]:.1f} "
          f"cx={K_TRUE[0,2]:.1f} cy={K_TRUE[1,2]:.1f} k1={D_TRUE[0]:.4f}")
    print(f"估计 fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f} k1={D.flatten()[0]:.4f}")
    print(f"RMS={rms:.4f}px  使用 {len(errors)}/{len(objs)} 张"
          f"{'（剔除离群）' if dropped else ''}")

    checks = {
        "RMS < 0.6px": rms < 0.6,
        "fx 相对误差 < 2%": fx_err < 0.02,
        "fy 相对误差 < 2%": fy_err < 0.02,
        "cx 偏差 < 6px": cx_err < 6.0,
        "cy 偏差 < 6px": cy_err < 6.0,
        "k1 偏差 < 0.02": k1_err < 0.02,
    }
    for name, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
