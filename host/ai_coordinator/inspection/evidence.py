"""Bounded immutable inspection artifacts; called in worker threads only."""
import hashlib
import json
from pathlib import Path
import re
import threading
import uuid

class Evidence:
    def __init__(self,root,limit_bytes=128*1024*1024):
        self.root=Path(root);self.limit=limit_bytes;self.used=0;self.count=0;self.lock=threading.Lock()
    def check_storage(self):
        import os,tempfile
        with self.lock:
            self.root.mkdir(parents=True,exist_ok=True)
            files=[p for p in self.root.iterdir() if p.is_file()]
            self.used=sum(p.stat().st_size for p in files)
            self.count=sum(p.suffix=='.json' for p in files)
            if self.used>=self.limit or self.count>=2048:raise ValueError('Evidence budget full; clean/export records first')
            with tempfile.TemporaryFile(dir=self.root) as f:
                f.write(b'falcons evidence check');f.flush();os.fsync(f.fileno())

    def write(self,kind,payload,metadata):
        if not re.fullmatch(r'[a-z_]+',kind) or not isinstance(payload,bytes):raise ValueError('Typed immutable evidence required')
        if len(payload)>32*1024*1024:raise ValueError('Evidence payload exceeds frame bound')
        event={**metadata,'payload_sha256':hashlib.sha256(payload).hexdigest(),'payload_bytes':len(payload)}
        raw=json.dumps(event,ensure_ascii=False,allow_nan=False,indent=2).encode('utf-8')
        with self.lock:
            if self.count>=2048 or self.used+len(payload)+len(raw)>self.limit:raise ValueError('Inspection evidence budget exhausted; export evidence before a new run')
            self.root.mkdir(parents=True,exist_ok=True);stem=kind+'-'+uuid.uuid4().hex
            suffix='.jpg' if kind=='keyframe' and payload.startswith(b'\xff\xd8') else '.png' if kind=='keyframe' and payload.startswith(b'\x89PNG') else '.bin'
            path=self.root/(stem+suffix);meta=self.root/(stem+'.json')
            with path.open('xb') as stream:stream.write(payload)
            with meta.open('xb') as stream:stream.write(raw)
            self.used+=len(payload)+len(raw);self.count+=1
        return {'payload':str(path),'metadata':str(meta),'sha256':event['payload_sha256'],'session':metadata.get('session'),'task':metadata.get('task'),'epoch':metadata.get('epoch')}
