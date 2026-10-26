import unittest
import numpy as np
import cv2
from ai_coordinator.inspection.commissioning import fit_intrinsics,fit_extrinsics,validate_ground,timing_candidate

class CommissioningTests(unittest.TestCase):
    def test_synthetic_intrinsics_fit_does_not_set_measurement_flags(self):
        K=np.array([[800.,0,640],[0,820.,360],[0,0,1.]])
        obj=np.zeros((35,3),np.float32);obj[:,:2]=np.mgrid[0:7,0:5].T.reshape(-1,2)*.03
        objects=[];pixels=[]
        for n in range(12):
            rot=np.array([-.2+n*.035,.1+(n%3)*.04,.03*n]);trans=np.array([-.1+(n%4)*.025,-.08+(n%3)*.025,.8+n*.03])
            pixels.append(cv2.projectPoints(obj,rot,trans,K,np.zeros(5))[0].reshape(-1,2));objects.append(obj)
        result=fit_intrinsics(objects,pixels,[1280,720])
        self.assertFalse(result['intrinsics_measured']);self.assertLess(result['fit_rms_px'],.001)
        self.assertAlmostEqual(result['K'][0][0],800,delta=.2)
    def test_synthetic_ground_fit_and_independent_validation_remain_candidates(self):
        K=[[800.,0,640],[0,800.,360],[0,0,1.]];D=[0.]*5
        points=np.array([[x,y,0] for x in [-.2,0,.2] for y in [-.1,.1]],dtype=np.float64)
        R=np.array([np.pi,0,0]);t=np.array([0.,0,.5])
        uv=cv2.projectPoints(points,R,t,np.array(K),np.array(D))[0].reshape(-1,2)
        result=fit_extrinsics(K,D,points,uv)
        self.assertFalse(result['extrinsics_measured']);self.assertAlmostEqual(result['T_base_camera'][2][3],.5,places=5)
        profile={**result,'K':K,'distortion':D}
        samples=[dict(uv=p.tolist(),xy_base_m=q[:2].tolist(),max_range_m=1.) for p,q in zip(uv,points)]
        validation=validate_ground(profile,samples)
        self.assertFalse(validation['ground_plane_verified']);self.assertLess(validation['observed_max_m'],1e-5)
    def test_timing_data_and_degenerate_geometry_are_not_silently_accepted(self):
        data=[dict(driver_timestamp_s=n,reference_exposure_s=n-.003+(n%2)*.0001) for n in range(20)]
        candidate=timing_candidate(data);self.assertFalse(candidate['measured']);self.assertAlmostEqual(candidate['driver_to_exposure_s'],-.00295)
        with self.assertRaises(ValueError):fit_intrinsics([],[],[1280,720])
        with self.assertRaises(ValueError):fit_extrinsics(np.eye(3),[0]*5,[[0,0,0]]*6,[[0,0]]*6)

if __name__=='__main__':unittest.main()
