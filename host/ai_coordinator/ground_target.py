"""Optional calibrated ray/ground intersection; never infer distance from bbox size.

Pixels MUST be undistorted and on the known ground plane. Base axes: x forward,
y left, z up; optical camera axes: x right, y down, z forward. Metres throughout.
The caller supplies measured calibration and pose; no nominal hardware defaults.
"""
import math
from .schema import number


def project_ground_pixel(u,v,K,T_base_camera,*,pixel_on_ground_verified=False,max_range_m):
    if pixel_on_ground_verified is not True:raise ValueError('A verified ground contact pixel is required')
    if not all(number(x) for x in (u,v,max_range_m)) or max_range_m<=0:raise ValueError('Invalid pixel/range')
    if len(K)!=3 or any(len(row)!=3 for row in K) or len(T_base_camera)!=4 or any(len(row)!=4 for row in T_base_camera):raise ValueError('Invalid calibration dimensions')
    if not all(number(x) for row in K+T_base_camera for x in row):raise ValueError('Nonfinite calibration')
    if K[0][0]<=0 or K[1][1]<=0 or K[0][1]!=0 or K[1][0]!=0 or K[2]!=[0,0,1] or T_base_camera[3]!=[0,0,0,1]:raise ValueError('Expected calibrated zero-skew pinhole transform')
    R=[row[:3] for row in T_base_camera[:3]];origin=[row[3] for row in T_base_camera[:3]]
    for i in range(3):
        for j in range(3):
            if abs(sum(R[k][i]*R[k][j] for k in range(3))-(1 if i==j else 0))>1e-6:raise ValueError('Rotation must be orthonormal')
    det=(R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]))
    if abs(det-1)>1e-6 or origin[2]<=0:raise ValueError('Invalid camera pose above ground')
    ray=[(u-K[0][2])/K[0][0],(v-K[1][2])/K[1][1],1]
    direction=[sum(R[i][j]*ray[j] for j in range(3)) for i in range(3)]
    if direction[2]>=-.15*math.sqrt(sum(x*x for x in direction)):raise ValueError('Near horizon or ray not towards ground')
    scale=-origin[2]/direction[2];point=[origin[i]+scale*direction[i] for i in range(3)]
    if not all(math.isfinite(x) for x in point) or math.hypot(point[0],point[1])>max_range_m:raise ValueError('Outside calibrated operating range')
    return {'point_base_m':point,'frame':'base_link','method':'calibrated_ground_intersection',
            'motion_authorized':False,'requires_independent_calibration_validation':True}


def base_to_map(point,pose):
    if len(point)!=3 or len(pose)!=3 or not all(number(x) for x in list(point)+list(pose)):raise ValueError('Expected base xyz and map x/y/yaw')
    x,y,yaw=pose;c=math.cos(yaw);s=math.sin(yaw)
    return [x+c*point[0]-s*point[1],y+s*point[0]+c*point[1],point[2]]
