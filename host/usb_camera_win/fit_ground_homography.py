"""Offline raw/undistorted-pixel to measured-ground homography fitting.

Outputs candidates only. Does not change RK configuration or authorize motion.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def project(points, matrix):
    value = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    if np.any(np.abs(value[:, 2]) < 1e-10):
        raise ValueError("Point lies at homography horizon")
    return value[:, :2] / value[:, 2, None]


def fit(source, target):
    for points in (source, target):
        if np.linalg.matrix_rank(np.column_stack((points, np.ones(len(points))))) < 3:
            raise ValueError("Non-collinear image and ground control points required")
    matrix, _ = cv2.findHomography(source, target, method=0)
    if matrix is None or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("Invalid or singular homography")
    return matrix / matrix[2, 2]


def summarize(source, target):
    matrix = fit(source, target)
    predicted = project(source, matrix)
    errors = np.linalg.norm(predicted - target, axis=1)
    inverse = np.linalg.inv(matrix)
    inverse /= inverse[2, 2]
    inverse_errors = np.linalg.norm(project(target, inverse) - source, axis=1)
    np.testing.assert_allclose(project(predicted, inverse), source, atol=1e-6)
    loo = []
    if len(source) > 4:
        for i in range(len(source)):
            keep = np.arange(len(source)) != i
            held = project(source[i:i+1], fit(source[keep], target[keep]))[0]
            loo.append(dict(index=i, predicted_ground_cm=held.tolist(),
                            error_cm=float(np.linalg.norm(held-target[i]))))
    # User axes: right/forward, centimetres. Runtime base axes: forward/left, metres.
    to_base = np.array([[0., .01, 0.], [-.01, 0., 0.], [0., 0., 1.]])
    return dict(H_pixel_to_ground_cm=matrix.tolist(), H_ground_cm_to_pixel=inverse.tolist(),
                H_pixel_to_base_m=(to_base @ matrix).tolist(),
                predicted_ground_cm=predicted.tolist(), errors_cm=errors.tolist(),
                fit_rms_cm=float(np.sqrt(np.mean(errors**2))), fit_max_cm=float(errors.max()),
                inverse_errors_px=inverse_errors.tolist(), leave_one_out=loo,
                leave_one_out_max_cm=max((p['error_cm'] for p in loo), default=None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--intrinsics', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError("Retain previous fit: " + str(args.out))
    evidence = json.loads(args.input.read_text(encoding='utf-8'))
    calibration = json.loads(args.intrinsics.read_text(encoding='utf-8'))
    if evidence.get('axes') != 'right_forward_cm':
        raise ValueError("Explicit right_forward_cm ground axes required")
    size = evidence['image_size']
    camera = calibration['camera']
    if size != [camera['width'], camera['height']] or evidence['focus'] != camera['focus']:
        raise ValueError("Calibration mode/focus mismatch")
    points = evidence['points']
    pixels = np.array([p['pixel_uv'] for p in points], dtype=np.float64)
    ground = np.array([p['ground_cm'] for p in points], dtype=np.float64)
    if (len(points) < 4 or pixels.shape != (len(points), 2) or ground.shape != pixels.shape
            or not np.isfinite(pixels).all() or not np.isfinite(ground).all()
            or np.any(pixels < 0) or np.any(pixels[:, 0] >= size[0]) or np.any(pixels[:, 1] >= size[1])):
        raise ValueError("Finite paired source-image and ground control points required")
    K = np.array(calibration['camera_matrix'], dtype=np.float64)
    D = np.array(calibration['distortion'], dtype=np.float64)
    if (K.shape != (3, 3) or D.size not in (4, 5, 8, 12, 14)
            or not np.isfinite(K).all() or not np.isfinite(D).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0 or not np.array_equal(K[2], [0, 0, 1])):
        raise ValueError("Valid pinhole intrinsics required")
    corrected = cv2.undistortPoints(pixels.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)
    result = dict(version=1, status='candidate_pending_independent_validation',
                  image_size=size, camera_serial=evidence['camera_serial'], focus=evidence['focus'],
                  autofocus=False, axes='right_forward_cm', origin=evidence['origin'],
                  captured_at=evidence['captured_at'], source_images=evidence['source_images'],
                  control_points=points, K=K.tolist(), distortion=D.tolist(),
                  input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  intrinsics_sha256=hashlib.sha256(args.intrinsics.read_bytes()).hexdigest(),
                  fit_objective='all control points; ground-plane squared distance; no outlier removal',
                  control_region_hull_cm=cv2.convexHull(ground.astype(np.float32)).reshape(-1, 2).tolist(),
                  raw_pixel_approximation=summarize(pixels, ground),
                  undistorted_pixel_recommended=summarize(corrected, ground),
                  undistorted_control_pixels=corrected.tolist(),
                  preferred_input='cv2.undistortPoints(raw_uv,K,D,P=K); then H_pixel_to_ground_cm',
                  notes=['Raw-pixel H approximates lens distortion; use only with raw input.',
                         'Do not apply distortion correction twice or use display/model pixels.',
                         'Fit and leave-one-out errors are not independent commissioning evidence.',
                         'Valid only for the unchanged camera/focus/mode and measured ground plane.',
                         'Extrapolation outside the control-point hull has not been validated.',
                         'No operational range or safety error bound has been accepted.'],
                  ground_plane_verified=False, live_config_changed=False)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(output=str(args.out), raw=result['raw_pixel_approximation'],
                          undistorted=result['undistorted_pixel_recommended']), ensure_ascii=False))


if __name__ == '__main__':
    main()
