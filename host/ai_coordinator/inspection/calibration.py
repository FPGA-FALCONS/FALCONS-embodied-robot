"""Measured geometry only. Lens catalogue data never authorizes metric motion."""
import math
from ..ground_target import base_to_map, project_ground_pixel


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class Calibration:
    def __init__(self, profile):
        self.profile = profile

    def problems(self, image_size, camera_serial):
        p = self.profile
        errors = []
        for key in ('intrinsics_measured', 'extrinsics_measured', 'ground_plane_verified', 'timing_verified'):
            if p.get(key) is not True:
                errors.append(key)
        if p.get('image_size') != list(image_size):
            errors.append('image_size_mismatch')
        if not camera_serial or p.get('camera_serial') != camera_serial:
            errors.append('camera_identity_mismatch')
        for key in ('max_range_m', 'xy_error_m'):
            if not finite(p.get(key)) or p[key] <= 0:
                errors.append('missing_' + key)
        K, T, D = p.get('K'), p.get('T_base_camera'), p.get('distortion')
        if (not isinstance(K, list) or len(K) != 3 or any(not isinstance(row, list) or len(row) != 3 for row in K)
                or not all(finite(v) for row in K for v in row) or K[0][0] <= 0 or K[1][1] <= 0
                or K[0][1] != 0 or K[1][0] != 0 or K[2] != [0, 0, 1]):
            errors.append('invalid_intrinsics')
        if not isinstance(D, list) or len(D) not in (4, 5, 8, 12, 14) or not all(finite(v) for v in D):
            errors.append('invalid_distortion')
        if (not isinstance(T, list) or len(T) != 4 or any(not isinstance(row, list) or len(row) != 4 for row in T)
                or not all(finite(v) for row in T for v in row) or T[3] != [0, 0, 0, 1] or T[2][3] <= 0):
            errors.append('invalid_extrinsics')
        else:
            R = [row[:3] for row in T[:3]]
            det = (R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-
                   R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+
                   R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]))
            if abs(det-1) > 1e-6 or any(abs(sum(R[k][i]*R[k][j] for k in range(3))-(i == j)) > 1e-6
                                      for i in range(3) for j in range(3)):
                errors.append('invalid_rotation')
        return errors

    def locate(self, anchor, image_size, camera_serial, pose, *, epoch, captured_at,
               pose_at, now, source_timestamp_verified):
        p = self.profile
        problems = self.problems(image_size, camera_serial)
        if problems:
            raise ValueError('Uncommissioned calibration: ' + ','.join(problems))
        if source_timestamp_verified is not True or anchor.get('ground_verified') is not True:
            raise ValueError('Verified capture timing and ground anchor are required')
        if epoch != pose.get('epoch') or not epoch:
            raise ValueError('Localization epoch mismatch')
        if not all(finite(x) for x in (captured_at, pose_at, now)) or not 0 <= now-captured_at <= .5 or abs(captured_at-pose_at) > .02:
            raise ValueError('Capture and pose do not have verified temporal alignment')
        u, v = anchor.get('u'), anchor.get('v')
        if not all(finite(x) for x in (u, v)) or not 0 <= u < image_size[0] or not 0 <= v < image_size[1]:
            raise ValueError('Ground anchor outside source image')
        import cv2
        import numpy as np
        K = np.asarray(p.get('K'), dtype=float)
        distortion = np.asarray(p.get('distortion'), dtype=float)
        if K.shape != (3, 3) or distortion.size not in (4, 5, 8, 12, 14) or not np.isfinite(K).all() or not np.isfinite(distortion).all():
            raise ValueError('Measured intrinsics/distortion required')
        uv = cv2.undistortPoints(np.array([[[u, v]]], dtype=float), K, distortion, P=K)[0, 0]
        point = project_ground_pixel(float(uv[0]), float(uv[1]), K.tolist(), p.get('T_base_camera'),
                                     pixel_on_ground_verified=True, max_range_m=p['max_range_m'])
        mapped = base_to_map(point['point_base_m'], (pose['x_m'], pose['y_m'], pose['yaw_rad']))
        if not all(-6.4 <= x < 6.4 for x in mapped[:2]):
            raise ValueError('Projected object outside navigation map')
        return {'xy_m': mapped[:2], 'xy_error_m': p['xy_error_m'],
                'epoch': epoch, 'captured_at': captured_at, 'method': 'measured_ground_projection'}
