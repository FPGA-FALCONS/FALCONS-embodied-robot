import json
import math
from pathlib import Path
import unittest

import cv2
import numpy as np

from ai_coordinator.inspection.commissioning import TimedCalibration as Calibration


def profile():
    path = Path(__file__).parents[1]/'profiles/inspection_ground_homography_20261028.json'
    return json.loads(path.read_text(encoding='utf-8'))


def metadata():
    return dict(focus_value=260, focus_locked=True, autofocus=False,
                focus_verification='v4l2_control_readback_each_frame')


class GroundHomographyTests(unittest.TestCase):
    def project(self, p=None, uv=(615., 349.), **kwargs):
        return Calibration(profile() if p is None else p).project(
            dict(u=uv[0], v=uv[1], ground_verified=True), [1280, 720], '200901010001',
            camera_metadata=kwargs.get('camera_metadata', metadata()))

    def test_static_h_geometry_does_not_require_unmeasured_pinhole_pose(self):
        c = Calibration(profile())
        self.assertEqual(c.geometry_problems([1280, 720], '200901010001'), [])
        self.assertCountEqual(c.problems([1280, 720], '200901010001'), ['timing_verified', 'missing_xy_error_m'])
        self.assertFalse(c.profile['extrinsics_measured'])
        self.assertIsNone(c.profile['T_base_camera'])

    def test_new_seven_held_out_points_and_right_forward_to_base_axis_conversion(self):
        controls = [((615.,349.),(0,70)), ((296.,348.),(-20,70)), ((977.5,262.),(30,90)),
                    ((510.5,213.),(-10,110)), ((743.,184.),(15,125)),
                    ((1036.,173.),(50,130)), ((392.,152.),(-30,150))]
        for uv, (right, forward) in controls:
            result = self.project(uv=uv)
            self.assertLess(np.linalg.norm(np.array(result['point_base_m'])-[forward/100,-right/100,0]), .015)
            np.testing.assert_allclose(result['point_base_m'][:2],
                                       [result['ground_right_forward_cm'][1]/100,
                                        -result['ground_right_forward_cm'][0]/100])
            self.assertEqual(result['method'], 'measured_ground_homography')
            self.assertFalse(result['motion_authorized'])

    def test_far_210cm_point_uses_H_without_pinhole_horizon_cutoff(self):
        point = self.project(uv=(611.,104.))['point_base_m']
        self.assertAlmostEqual(point[0], 2.0892626658, places=6)

    def test_independently_checked_near_five_points_are_in_extended_region(self):
        for uv, right, forward in [((615.5,634.),0,40), ((1186.,630.),20,40),
                                    ((30.5,638.),-20,40), ((389.5,499.),-10,50),
                                    ((726.5,452.),5,55)]:
            point=self.project(uv=uv)['point_base_m']
            self.assertLess(np.linalg.norm(np.array(point)-[forward/100,-right/100,0]),.01)
        self.assertEqual(min(p[1] for p in profile()['homography']['ground_hull_cm']),40)
        self.assertIs(profile()['homography']['region_extension']['matrix_refitted'],False)

    def test_extended_region_still_rejects_below_40cm_and_unmeasured_lateral_area(self):
        p=profile();K=np.array(p['K']);D=np.array(p['distortion']);inv=np.linalg.inv(np.array(p['homography']['H_pixel_to_ground_cm']))
        for ground in ([0.,39.],[50.,90.]):
            rect=inv@np.r_[ground,1.];rect=rect[:2]/rect[2]
            normal=np.linalg.inv(K)@np.r_[rect,1.]
            raw=cv2.projectPoints(normal.reshape(1,3),np.zeros(3),np.zeros(3),K,D)[0][0,0]
            self.assertTrue(0<=raw[0]<1280 and 0<=raw[1]<720)
            with self.subTest(ground=ground),self.assertRaisesRegex(ValueError,'ground region'):
                self.project(uv=tuple(map(float,raw)))

    def test_identity_mode_anchor_and_source_pixel_checks(self):
        c = Calibration(profile())
        for size, serial, anchor in [([1920,1080],'200901010001',dict(u=615.,v=349.,ground_verified=True)),
                                      ([1280,720],'other',dict(u=615.,v=349.,ground_verified=True)),
                                      ([1280,720],'200901010001',dict(u=1280.,v=349.,ground_verified=True))]:
            with self.subTest(size=size, serial=serial, anchor=anchor), self.assertRaises(ValueError):
                c.project(anchor,size,serial,camera_metadata=metadata())

    def test_focus_must_be_read_back_for_this_frame(self):
        for change in ({'focus_value':285}, {'autofocus':True}, {'focus_locked':False},
                       {'focus_verification':'configuration_only'}):
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'manual focus'):
                self.project(camera_metadata={**metadata(), **change})
        with self.assertRaises(ValueError):
            self.project(camera_metadata={})

    def test_no_silent_raw_H_or_coordinate_contract_fallback(self):
        for key, value in [('pixel_domain','raw_pixels'), ('ground_axes','forward_left_m')]:
            p=profile();p['homography'][key]=value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'coordinate_contract'):
                self.project(p)
        p=profile();p['projection_model']='unknown'
        with self.assertRaisesRegex(ValueError, 'unknown_projection_model'):
            self.project(p)

    def test_invalid_matrix_region_and_verification_flags_fail_closed(self):
        for H in (None, [[1,0,0],[2,0,0],[0,0,1]], [[float('nan'),0,0],[0,1,0],[0,0,1]]):
            p=profile();p['homography']['H_pixel_to_ground_cm']=H
            with self.subTest(H=H), self.assertRaises(ValueError):
                self.project(p)
        for hull in ([], [[0,0],[1,1],[2,2]], [[0,0],[1,1],[0,1],[1,0]]):
            p=profile();p['homography']['ground_hull_cm']=hull
            with self.subTest(hull=hull), self.assertRaises(ValueError):
                self.project(p)
        for key in ('intrinsics_measured','ground_plane_verified','homography_measured'):
            p=profile();p[key]=False
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.project(p)

    def test_outside_control_hull_and_range_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'ground region'):
            self.project(uv=(615.,719.))
        c=Calibration(profile());anchor=dict(u=615.,v=719.,ground_verified=True)
        rough=c.project(anchor,[1280,720],'200901010001',camera_metadata=metadata(),check_region=False)
        self.assertLess(rough['point_base_m'][0],.4)
        self.assertFalse(rough['motion_authorized'])
        with self.assertRaisesRegex(ValueError,'ground region'):
            c.locate_stopped(anchor,[1280,720],'200901010001',dict(x_m=0,y_m=0,yaw_rad=0,epoch=1),camera_metadata=metadata())
        p=profile();p['max_range_m']=.1
        with self.assertRaisesRegex(ValueError, 'operating range'):
            self.project(p)

    def test_projective_horizon_is_rejected(self):
        p=profile();K=np.array(p['K']);D=np.array(p['distortion'])
        corrected=cv2.undistortPoints(np.array([[[615.,349.]]]),K,D,P=K)[0,0]
        p['homography']['H_pixel_to_ground_cm'][2]=[0.,1.,-float(corrected[1])]
        with self.assertRaisesRegex(ValueError, 'denominator'):
            self.project(p)

    def test_static_fit_does_not_bypass_timing_epoch_or_error_bound(self):
        args=dict(anchor=dict(u=615.,v=349.,ground_verified=True),image_size=[1280,720],
                  camera_serial='200901010001',pose=dict(x_m=2.,y_m=1.,yaw_rad=math.pi/2,epoch=1),
                  epoch=1,captured_at=1.,pose_at=1.,now=1.1,source_timestamp_verified=True,
                  camera_metadata=metadata())
        with self.assertRaisesRegex(ValueError, 'Uncommissioned calibration'):
            Calibration(profile()).locate(**args)
        p=profile();p.update(timing_verified=True,xy_error_m=.05)
        result=Calibration(p).locate(**args);base=self.project(p)['point_base_m']
        np.testing.assert_allclose(result['xy_m'],[2-base[1],1+base[0]])
        for change in ({'epoch':2},{'pose_at':1.1},{'source_timestamp_verified':False}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                Calibration(p).locate(**{**args,**change})


if __name__=='__main__':
    unittest.main()
