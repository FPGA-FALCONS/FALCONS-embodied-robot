"""Offline measured-data preparation; never changes the live config or enables motion.

Candidates retain false verification flags. Independent ground/timing and
vehicle measurements must still be reviewed for the installed camera mode.
"""
import argparse
import hashlib
import json
from pathlib import Path
import math
from statistics import median
from .calibration import finite


def fit_intrinsics(objects,pixels,size):
    import cv2
    import numpy as np
    if len(objects)<12 or len(objects)!=len(pixels) or len(size)!=2 or any(type(v) is not int or v<=0 for v in size):
        raise ValueError('At least12 complete calibration views and exact image size required')
    obj=[np.asarray(x,np.float32).reshape(-1,3) for x in objects]
    uv=[np.asarray(x,np.float32).reshape(-1,2) for x in pixels]
    if any(len(a)!=len(b) or len(a)<6 or not np.isfinite(a).all() or not np.isfinite(b).all() for a,b in zip(obj,uv)):
        raise ValueError('Finite paired board points required')
    rms,K,D,rotations,translations=cv2.calibrateCamera(obj,uv,tuple(size),None,None)
    if not finite(float(rms)) or not np.isfinite(K).all() or not np.isfinite(D).all():raise ValueError('Nonfinite calibration fit')
    errors=[]
    for a,b,R,t in zip(obj,uv,rotations,translations):
        projected=cv2.projectPoints(a,R,t,K,D)[0].reshape(-1,2)
        errors.append(float(np.sqrt(np.mean(np.sum((projected-b)**2,axis=1)))))
    return dict(K=K.tolist(),distortion=D.ravel().tolist(),image_size=list(size),fit_rms_px=float(rms),
                per_view_rms_px=errors,intrinsics_measured=False,requires_independent_validation=True)


def chessboards(paths,columns,rows,square_m):
    import cv2
    import numpy as np
    if not 3<=columns<=30 or not 3<=rows<=30 or not finite(square_m) or square_m<=0:raise ValueError('Measured board geometry required')
    template=np.zeros((columns*rows,3),np.float32);template[:,:2]=np.mgrid[0:columns,0:rows].T.reshape(-1,2)*square_m
    objects=[];pixels=[];used=[];size=None
    for path in list(paths)[:200]:
        image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        if image is None:raise ValueError('Cannot read calibration image: '+str(path))
        current=[image.shape[1],image.shape[0]]
        if size is not None and current!=size:raise ValueError('Mixed calibration resolutions are forbidden')
        size=current;found,corners=cv2.findChessboardCorners(image,(columns,rows))
        if not found:continue
        corners=cv2.cornerSubPix(image,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,.001))
        objects.append(template.copy());pixels.append(corners.reshape(-1,2));used.append(dict(path=str(path),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest()))
    candidate=fit_intrinsics(objects,pixels,size or [])
    candidate['images']=used;candidate['square_m']=square_m;return candidate


def fit_extrinsics(K,distortion,base_xyz,pixels):
    import cv2
    import numpy as np
    K=np.asarray(K,np.float64);D=np.asarray(distortion,np.float64);points=np.asarray(base_xyz,np.float64);uv=np.asarray(pixels,np.float64)
    if (K.shape!=(3,3) or points.ndim!=2 or points.shape[1]!=3 or uv.shape!=(len(points),2) or len(points)<6
            or not all(np.isfinite(a).all() for a in (K,D,points,uv)) or np.linalg.matrix_rank(points-points.mean(axis=0))<2):
        raise ValueError('Finite non-collinear base/image correspondences required')
    ok,R,t=cv2.solvePnP(points,uv,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:raise ValueError('Extrinsic fit failed')
    camera_R=cv2.Rodrigues(R)[0];origin=-camera_R.T@t.reshape(3)
    depth=(points@camera_R.T+t.reshape(3))[:,2]
    if origin[2]<=0 or np.any(depth<=0):raise ValueError('Camera must be above ground and all measured points in front')
    transform=np.eye(4);transform[:3,:3]=camera_R.T;transform[:3,3]=origin
    projected=cv2.projectPoints(points,R,t,K,D)[0].reshape(-1,2);errors=np.linalg.norm(projected-uv,axis=1)
    return dict(T_base_camera=transform.tolist(),fit_rms_px=float(np.sqrt(np.mean(errors**2))),fit_max_px=float(errors.max()),
                extrinsics_measured=False,ground_plane_verified=False,requires_independent_validation=True)


def validate_ground(profile,samples):
    import cv2
    import numpy as np
    from ..ground_target import project_ground_pixel
    if len(samples)<6:raise ValueError('At least6 independent measured ground validation points required')
    errors=[]
    for sample in samples:
        uv=np.asarray(sample['uv'],np.float64).reshape(1,1,2)
        xy=sample['xy_base_m']
        if len(xy)!=2 or not all(finite(x) for x in xy):raise ValueError('Measured base XY required')
        point=cv2.undistortPoints(uv,np.asarray(profile['K'],np.float64),np.asarray(profile['distortion'],np.float64),P=np.asarray(profile['K'],np.float64))[0,0]
        projected=project_ground_pixel(float(point[0]),float(point[1]),profile['K'],profile['T_base_camera'],pixel_on_ground_verified=True,max_range_m=sample['max_range_m'])['point_base_m']
        errors.append(math.dist(projected[:2],xy))
    return dict(samples=len(errors),rms_m=math.sqrt(sum(x*x for x in errors)/len(errors)),observed_max_m=max(errors),
                ground_plane_verified=False,note='Observed errors are evidence, not an automatically accepted operational error bound')


def timing_candidate(samples):
    if len(samples)<20:raise ValueError('At least20 independent hardware timing observations required')
    offsets=[]
    for sample in samples:
        driver,reference=sample['driver_timestamp_s'],sample['reference_exposure_s']
        if not all(finite(x) for x in (driver,reference)):raise ValueError('Finite monotonic timing measurements required')
        offsets.append(reference-driver)
    bias=median(offsets);residual=max(abs(x-bias) for x in offsets)
    return dict(driver_to_exposure_s=bias,observed_max_residual_s=residual,measured=False,
                note='Bind identity/mode/source and independently validate an error bound before acceptance')


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='operation',required=True)
    board=sub.add_parser('intrinsics');board.add_argument('--images',type=Path,required=True);board.add_argument('--columns',type=int,required=True);board.add_argument('--rows',type=int,required=True);board.add_argument('--square-m',type=float,required=True)
    for name in ('extrinsics','ground','timing'):
        command=sub.add_parser(name);command.add_argument('--input',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    if args.out.exists():raise FileExistsError('Retain the previous measurement result')
    if args.operation=='intrinsics':
        result=chessboards(sorted(p for p in args.images.iterdir() if p.suffix.lower() in ('.jpg','.jpeg','.png')),args.columns,args.rows,args.square_m)
    else:
        data=json.loads(args.input.read_text(encoding='utf-8'))
        if args.operation=='extrinsics':result=fit_extrinsics(data['K'],data['distortion'],data['base_xyz_m'],data['pixels_uv'])
        elif args.operation=='ground':result=validate_ground(data['profile'],data['samples'])
        else:result=timing_candidate(data['samples'])
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({'candidate':str(args.out),'live_config_changed':False,'motion_enabled':False}))

if __name__=='__main__':main()
