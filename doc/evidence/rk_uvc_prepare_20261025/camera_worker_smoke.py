from pathlib import Path
import hashlib,json,time
from ai_coordinator.v4l2_camera import Camera
root=Path('/home/orangepi/workspace/falcons_uvc_prepare_20261025_v2')
cfg=json.loads((root/'capture_720p_fixed_rate/camera.candidate.json').read_text())
cfg['storage']=str(root/'worker_frames')
cam=Camera(cfg)
frames=[]
try:
 for _ in range(3):
  result=cam.run({})
  assert result['status']=='frame', result
  assert (result['width'],result['height'])==(1280,720)
  assert result['source_timestamp_verified'] is False
  frames.append({**result,'sha256':hashlib.sha256(Path(result['image']).read_bytes()).hexdigest()})
finally: cam.close()
result={'success':True,'frames':frames,'source_timestamp_verified':False,'models_loaded':False,'active_config_changed':False,'motion_commands_sent':False}
(root/'worker_smoke.json').write_text(json.dumps(result,indent=2))
print('Existing Camera worker delivered',len(frames),'actual UVC PNG frames; no model or motion')
