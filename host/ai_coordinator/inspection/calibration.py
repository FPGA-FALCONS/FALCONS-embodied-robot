"""Measured geometry only. Lens catalogue data never authorizes metric motion."""
import math
from ..ground_target import base_to_map, project_ground_pixel


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def matrix3(value):
    return (isinstance(value, list) and len(value) == 3 and
            all(isinstance(row, list) and len(row) == 3 and all(finite(v) for v in row) for row in value))


def determinant3(a):
    return (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
            - a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
            + a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))


def cross(a, b, p):
    return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])


def convex_hull_valid(hull):
    if (not isinstance(hull, list) or not 3 <= len(hull) <= 64 or
            any(not isinstance(p, list) or len(p) != 2 or not all(finite(x) for x in p) for p in hull)):
        return False
    signs = []
    for i, a in enumerate(hull):
        b = hull[(i+1) % len(hull)]
        if a == b:
            return False
        values = [cross(a, b, p) for p in hull]
        if min(values) < -1e-8 and max(values) > 1e-8:
            return False
        signs.extend(v > 0 for v in values if abs(v) > 1e-8)
    return bool(signs) and (all(signs) or not any(signs))


def inside_hull(point, hull):
    values = [cross(a, hull[(i+1) % len(hull)], point) for i, a in enumerate(hull)]
    return min(values) >= -1e-7 or max(values) <= 1e-7


class Calibration:
    def __init__(self, profile):
        self.profile = profile

    def geometry_problems(self, image_size, camera_serial):
        p = self.profile
        errors = []
        for key in ('intrinsics_measured', 'ground_plane_verified'):
            if p.get(key) is not True:
                errors.append(key)
        if p.get('image_size') != list(image_size):
            errors.append('image_size_mismatch')
        if not camera_serial or p.get('camera_serial') != camera_serial:
            errors.append('camera_identity_mismatch')
        for key in ('max_range_m',):
            if not finite(p.get(key)) or p[key] <= 0:
                errors.append('missing_' + key)
        K, T, D = p.get('K'), p.get('T_base_camera'), p.get('distortion')
        if (not matrix3(K) or K[0][0] <= 0 or K[1][1] <= 0
                or K[0][1] != 0 or K[1][0] != 0 or K[2] != [0, 0, 1]):
            errors.append('invalid_intrinsics')
        if not isinstance(D, list) or len(D) not in (4, 5, 8, 12, 14) or not all(finite(v) for v in D):
            errors.append('invalid_distortion')
        model = p.get('projection_model', 'pinhole_ground')
        if model == 'undistorted_homography':
            h = p.get('homography', {})
            if not isinstance(h, dict):
                return errors + ['invalid_homography_profile']
            if p.get('homography_measured') is not True:
                errors.append('homography_measured')
            H = h.get('H_pixel_to_ground_cm')
            if not matrix3(H) or not finite(determinant3(H)) or abs(determinant3(H)) < 1e-12:
                errors.append('invalid_homography')
            if h.get('pixel_domain') != 'undistorted_pixels_P_K' or h.get('ground_axes') != 'right_forward_cm':
                errors.append('homography_coordinate_contract')
            if not convex_hull_valid(h.get('ground_hull_cm')):
                errors.append('invalid_homography_region')
            if type(p.get('focus')) is not int or p['focus'] < 0 or p.get('autofocus') is not False:
                errors.append('homography_focus_binding')
            return errors
        if model != 'pinhole_ground':
            return errors + ['unknown_projection_model']
        if p.get('extrinsics_measured') is not True:
            errors.append('extrinsics_measured')
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

    def problems(self, image_size, camera_serial):
        errors = self.geometry_problems(image_size, camera_serial)
        if self.profile.get('timing_verified') is not True:
            errors.append('timing_verified')
        if not finite(self.profile.get('xy_error_m')) or self.profile['xy_error_m'] <= 0:
            errors.append('missing_xy_error_m')
        return errors

    def project(self, anchor, image_size, camera_serial, *, camera_metadata=None):
        """Static base-frame geometry; never authorizes a timestamped world observation."""
        p = self.profile
        errors = self.geometry_problems(image_size, camera_serial)
        if errors:
            raise ValueError('Uncommissioned geometry: ' + ','.join(errors))
        if anchor.get('ground_verified') is not True:
            raise ValueError('Verified ground anchor required')
        u, v = anchor.get('u'), anchor.get('v')
        if not all(finite(x) for x in (u, v)) or not 0 <= u < image_size[0] or not 0 <= v < image_size[1]:
            raise ValueError('Ground anchor outside source image')
        import cv2
        import numpy as np
        K = np.asarray(p['K'], dtype=np.float64)
        D = np.asarray(p['distortion'], dtype=np.float64)
        uv = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), K, D, P=K)[0, 0]
        if not np.isfinite(uv).all():
            raise ValueError('Nonfinite undistorted ground anchor')
        if p.get('projection_model') == 'undistorted_homography':
            metadata = camera_metadata or {}
            if (metadata.get('focus_value') != p['focus'] or metadata.get('focus_locked') is not True
                    or metadata.get('autofocus') is not False
                    or metadata.get('focus_verification') != 'v4l2_control_readback_each_frame'):
                raise ValueError('Homography requires matching per-frame verified manual focus')
            h = p['homography']
            H = np.asarray(h['H_pixel_to_ground_cm'], dtype=np.float64)
            ray = np.array([uv[0], uv[1], 1.], dtype=np.float64)
            q = H @ ray
            if not np.isfinite(q).all() or abs(q[2]) <= 1e-9*np.linalg.norm(H[2])*np.linalg.norm(ray):
                raise ValueError('Homography horizon or invalid denominator')
            right, forward = map(float, q[:2]/q[2])
            if not inside_hull([right, forward], h['ground_hull_cm']):
                raise ValueError('Outside measured homography ground region')
            point = [forward/100., -right/100., 0.]
            if math.hypot(*point[:2]) > p['max_range_m']:
                raise ValueError('Outside calibrated operating range')
            return dict(point_base_m=point, ground_right_forward_cm=[right, forward],
                        method='measured_ground_homography', motion_authorized=False)
        return project_ground_pixel(float(uv[0]), float(uv[1]), p['K'], p['T_base_camera'],
                                    pixel_on_ground_verified=True, max_range_m=p['max_range_m'])

    def locate(self, anchor, image_size, camera_serial, pose, *, epoch, captured_at,
               pose_at, now, source_timestamp_verified, camera_metadata=None):
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
        point = self.project(anchor, image_size, camera_serial, camera_metadata=camera_metadata)
        mapped = base_to_map(point['point_base_m'], (pose['x_m'], pose['y_m'], pose['yaw_rad']))
        if not all(-6.4 <= x < 6.4 for x in mapped[:2]):
            raise ValueError('Projected object outside navigation map')
        return {'xy_m': mapped[:2], 'xy_error_m': p['xy_error_m'],
                'epoch': epoch, 'captured_at': captured_at,
                'method': 'measured_ground_homography' if p.get('projection_model') == 'undistorted_homography' else 'measured_ground_projection'}
