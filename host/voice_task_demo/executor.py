"""Bounded task replay using actual detections and explicitly simulated navigation."""
import math
from task_graph import compile_task,simulate


def valid_detection(detection,target):
    if detection.get('class')!=target:return False
    score=detection.get('confidence');box=detection.get('bbox')
    if type(score) not in (int,float) or not math.isfinite(score) or not .3<=score<=1:return False
    if not isinstance(box,list) or len(box)!=4:return False
    if not all(type(x) in (int,float) and math.isfinite(x) for x in box):return False
    return 0<=box[0]<box[2] and 0<=box[1]<box[3]


def replay(task,scenes,vision,home_ack=False,interrupted=False):
    graph=compile_task(task);observations={};evidence={}
    for name in ('primary','fallback'):
        if name not in graph['nodes']:continue
        target=graph['nodes'][name]['target'];frames=vision.get(scenes.get(name,''),[])
        matches=[{'image':frame['image'],'detection':det} for frame in frames
                 for det in frame['detections'] if valid_detection(det,target)]
        observations[name]='found' if matches else 'not_found'
        observations[name+'_verify']='ok' if matches else 'reject'
        evidence[name]={'target':target,'matches':matches,'image_window_exhausted':not matches,
                        'verification':'class/score/bbox consistency, not an independent VLM'}
    observations['return_home']='ok' if home_ack else 'timeout'
    result=simulate(task,observations,interrupted)
    # Include only evidence from nodes actually visited; fallback may never run.
    result.update(evidence={name:value for name,value in evidence.items() if name in result['trace']},
                  graph=graph,simulation_only=True,motion_authorized=False,
                  physical_task_completed=False,return_home_ack_is_simulated=True)
    return result
