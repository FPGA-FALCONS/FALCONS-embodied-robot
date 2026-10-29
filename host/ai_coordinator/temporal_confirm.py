"""Small, dependency-free confirmation gate for already-decoded detections.

Inspired by tracking-by-detection confirmation, not a SORT/ByteTrack reproduction.
Outputs are always current observed boxes; never predicted or filled-in boxes.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

def box_iou(a,b):
    inter=max(0.,min(a[2],b[2])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[1],b[1]))
    return inter/max(1e-12,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)

@dataclass
class _Track:
    id:int
    cls:int
    bbox:list[float]
    last_seen:float
    streak:int=1
    confirmed:bool=False

class TemporalConfirmation:
    def __init__(self,min_hits=3,iou_threshold=.3,max_gap_s=.6,confidence=.25):
        if not isinstance(min_hits,int) or min_hits<1:raise ValueError('min_hits must be a positive integer')
        if not 0<iou_threshold<=1 or max_gap_s<=0 or not 0<=confidence<=1:raise ValueError('Invalid confirmation parameters')
        self.min_hits=min_hits;self.iou_threshold=iou_threshold;self.max_gap_s=max_gap_s;self.confidence=confidence
        self._tracks={};self._next_id=1;self._last_frame=None;self._last_time=None;self._shape=None

    def update(self,detections,frame_id:int,timestamp:float,frame_shape=None):
        if not isinstance(frame_id,int) or (self._last_frame is not None and frame_id<=self._last_frame):raise ValueError('Every update requires a NEW increasing frame_id')
        if not math.isfinite(timestamp) or timestamp<0 or (self._last_time is not None and timestamp<=self._last_time):raise ValueError('Timestamps must be finite and strictly increasing')
        valid=[]
        for p in detections:
            b=p.get('bbox',[]);c=p.get('cls');s=p.get('confidence')
            if len(b)!=4 or not all(math.isfinite(float(v)) for v in b) or b[2]<=b[0] or b[3]<=b[1]:raise ValueError('Invalid detection box')
            if not isinstance(c,int) or not 0<=c<4 or s is None or not math.isfinite(float(s)) or not 0<=s<=1:raise ValueError('Invalid detection class/score')
            if s>=self.confidence:valid.append(dict(p,bbox=list(b)))
        if frame_shape is not None:
            shape=tuple(frame_shape)
            if self._shape is not None and shape!=self._shape:self._tracks.clear()
            self._shape=shape
        self._last_frame=frame_id;self._last_time=timestamp
        self._tracks={k:t for k,t in self._tracks.items() if timestamp-t.last_seen<=self.max_gap_s}
        pairs=[]
        for k,t in self._tracks.items():
            for j,p in enumerate(valid):
                if t.cls==p['cls']:
                    overlap=box_iou(t.bbox,p['bbox'])
                    if overlap>=self.iou_threshold:pairs.append((-overlap,k,j))
        matched_tracks=set();matched_detections={}
        for _,k,j in sorted(pairs):
            if k not in matched_tracks and j not in matched_detections:
                matched_tracks.add(k);matched_detections[j]=k
        for k,t in self._tracks.items():
            if k not in matched_tracks:t.streak=0
        output=[]
        for j,p in enumerate(valid):
            if j in matched_detections:
                t=self._tracks[matched_detections[j]];t.streak+=1;t.last_seen=timestamp;t.bbox=p['bbox']
            else:
                t=_Track(self._next_id,p['cls'],p['bbox'],timestamp);self._tracks[t.id]=t;self._next_id+=1
            if t.streak>=self.min_hits:t.confirmed=True
            if t.confirmed:output.append(dict(p,track_id=t.id,confirmation_hits=t.streak))
        return output
