"""Persistent isolated model worker. SDK imports and blocking calls stay here."""
import argparse
import json
from pathlib import Path
import re
import sys
import time
import urllib.request

HOST=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HOST))
from ai_coordinator.schema import DETECTION_CONFIDENCE_THRESHOLD, TARGETS
from ai_coordinator.temporal_confirm import TemporalConfirmation
PREFIX='FALCONS_RPC '


class ASR:
    def __init__(self,cfg):
        import sherpa_onnx
        self.rec=sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=cfg['model'],tokens=cfg['tokens'],num_threads=cfg.get('threads',2),
            provider='cpu',language='zh',use_itn=True)

    def run(self,request):
        from voice_bridge.audio import read_wav,mono16k,quality
        wav=request.get('wav');source='file'
        if request.get('capture'):
            from voice_bridge.capture import record
            c=request['capture']
            if not isinstance(c.get('device'),str) or c['device'] in ('default','0','hw:0','plughw:0'):
                raise ValueError('Select the USB microphone explicitly')
            audio,metadata=record(device=c['device'],duration=c.get('seconds',5),backend='alsa',
                rate=c.get('rate',16000),channels=c.get('channels',1))
            source='microphone'
        else:
            audio=read_wav(wav);metadata={'wav':str(Path(wav).resolve())}
        audio=mono16k(audio,request.get('channel',0));q=quality(audio,-55)
        if q['below_energy_threshold']:return {'text':'','status':'no_speech_energy','source':source,'quality':q}
        stream=self.rec.create_stream();stream.accept_waveform(16000,audio.samples[:,0]);self.rec.decode_stream(stream)
        text=re.sub(r'<\|[^|]*\|>','',stream.result.text).strip()
        return {'text':text,'status':'recognized' if text else 'empty_transcript','source':source,
                'metadata':metadata,'quality':q,'engine':'SenseVoiceSmall INT8','provider':'cpu'}


def nms(boxes,scores,threshold):
    import numpy as np
    order=np.argsort(-scores);keep=[]
    while order.size and len(keep)<300:
        i=int(order[0]);keep.append(i)
        if order.size==1:break
        rest=order[1:];a=boxes[i];b=boxes[rest]
        lo=np.maximum(a[:2],b[:,:2]);hi=np.minimum(a[2:],b[:,2:]);d=np.maximum(hi-lo,0)
        inter=d[:,0]*d[:,1];area=(a[2]-a[0])*(a[3]-a[1]);other=(b[:,2]-b[:,0])*(b[:,3]-b[:,1])
        overlap=inter/np.maximum(area+other-inter,1e-9);order=rest[overlap<=threshold]
    return keep


def decode_outputs(outputs,scale,px,py,width,height,threshold=DETECTION_CONFIDENCE_THRESHOLD,contract='original'):
    import numpy as np
    from ai_coordinator.schema import TARGETS
    if contract=='original':
        if len(outputs)!=1 or np.asarray(outputs[0]).ndim!=3 or np.asarray(outputs[0]).shape[:2]!=(1,8):
            raise ValueError('Expected original [1,8,N]: xywh + four probabilities')
        b=np.asarray(outputs[0])[:,:4];s=np.asarray(outputs[0])[:,4:]
    elif contract=='decoded_probs':
        if len(outputs)!=2:raise ValueError('Expected decoded boxes + four probabilities')
        b=np.asarray(outputs[0]);s=np.asarray(outputs[1])
    else:raise ValueError('Unsupported explicit output contract')
    if b.ndim!=3 or s.ndim!=3 or b.shape[0]!=1 or s.shape[0]!=1 or b.shape[1]!=4 or s.shape[1]!=4 or b.shape[2]!=s.shape[2]:
        raise ValueError('RKNN output contract mismatch')
    boxes=b[0].T.astype(np.float32);scores=s[0].T.astype(np.float32)
    if not np.isfinite(boxes).all() or not np.isfinite(scores).all():raise ValueError('nonfinite RKNN output')
    if np.any(scores < -1e-5) or np.any(scores > 1.00001):raise ValueError('Expected probabilities in [0,1]')
    classes=np.argmax(scores,axis=1)
    result=[]
    for c,name in enumerate(TARGETS):
        indices=np.where((classes==c)&(scores[:,c]>=threshold))[0]
        if not len(indices):continue
        box=boxes[indices];xyxy=np.empty_like(box)
        xyxy[:,0]=(box[:,0]-box[:,2]/2-px)/scale;xyxy[:,2]=(box[:,0]+box[:,2]/2-px)/scale
        xyxy[:,1]=(box[:,1]-box[:,3]/2-py)/scale;xyxy[:,3]=(box[:,1]+box[:,3]/2-py)/scale
        xyxy[:,[0,2]]=np.clip(xyxy[:,[0,2]],0,width);xyxy[:,[1,3]]=np.clip(xyxy[:,[1,3]],0,height)
        valid=(xyxy[:,2]>xyxy[:,0])&(xyxy[:,3]>xyxy[:,1]);xyxy=xyxy[valid];sc=scores[indices,c][valid]
        for k in nms(xyxy,sc,.65):result.append({'class':name,'confidence':float(sc[k]),'bbox':xyxy[k].tolist()})
    return result


class Vision:
    def __init__(self,cfg):
        self.provider=cfg.get('provider','rknn');self.size=cfg.get('size',640);self.contract=cfg.get('contract','original')
        self.gate=TemporalConfirmation(min_hits=3,iou_threshold=.3,max_gap_s=.6,confidence=DETECTION_CONFIDENCE_THRESHOLD)
        self.stream_id=None
        if self.size!=640:raise ValueError('This contract requires 640 input')
        if self.provider=='onnx_cpu':
            import onnxruntime as ort
            opts=ort.SessionOptions();opts.intra_op_num_threads=cfg.get('threads',4);opts.inter_op_num_threads=1
            self.rec=ort.InferenceSession(cfg['model'],sess_options=opts,providers=['CPUExecutionProvider'])
            if len(self.rec.get_inputs())!=1:raise ValueError('Expected one ONNX image input')
            self.input=self.rec.get_inputs()[0]
            if self.input.shape!=[1,3,640,640] or self.input.type!='tensor(float)':raise ValueError('Expected float32 NCHW ONNX input')
        elif self.provider=='rknn':
            from rknnlite.api import RKNNLite
            self.rec=RKNNLite()
            if self.rec.load_rknn(cfg['model'])!=0:raise RuntimeError('RKNN model load failed')
            if self.rec.init_runtime(core_mask=RKNNLite.NPU_CORE_0)!=0:raise RuntimeError('RKNN init failed')
        else:raise ValueError('Explicit supported vision provider required')

    def run(self,request):
        import numpy as np
        import cv2
        if request.get('format')=='rgb565':
            from ai_coordinator.frames import read_raw565
            im=read_raw565(Path(request['image']),request['width'],request['height'],request.get('byteorder','little'))
        else:
            bgr=cv2.imdecode(np.frombuffer(Path(request['image']).read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
            if bgr is None:raise ValueError('Unreadable image')
            im=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        height,width=im.shape[:2]
        if not 1<=width<=8192 or not 1<=height<=8192:raise ValueError('invalid image dimensions')
        scale=min(self.size/width,self.size/height);nw,nh=round(width*scale),round(height*scale)
        px,py=(self.size-nw)//2,(self.size-nh)//2
        # Match the already verified v17 OpenCV RGB letterbox exactly.
        canvas=np.full((self.size,self.size,3),114,dtype=np.uint8)
        canvas[py:py+nh,px:px+nw]=cv2.resize(im,(nw,nh),interpolation=cv2.INTER_LINEAR)
        start=time.monotonic()
        if self.provider=='onnx_cpu':
            x=np.ascontiguousarray(canvas.transpose(2,0,1)[None],dtype=np.float32)/255.
            outputs=self.rec.run(None,{self.input.name:x})
        else:outputs=self.rec.inference(inputs=[canvas[None,...]],data_format='nhwc')
        inference=time.monotonic()-start
        raw=decode_outputs(outputs,scale,px,py,width,height,contract=self.contract)
        if 'frame_id' in request:
            if request['stream_id']!=self.stream_id:
                self.gate=TemporalConfirmation(min_hits=3,iou_threshold=.3,max_gap_s=.6,confidence=DETECTION_CONFIDENCE_THRESHOLD)
                self.stream_id=request['stream_id']
            candidates=[dict(d,cls=TARGETS.index(d['class'])) for d in raw]
            confirmed=self.gate.update(candidates,int(request['frame_id']),request['captured_at_s'],(height,width))
            detections=[{k:v for k,v in d.items() if k!='cls'} for d in confirmed]
        else:
            detections=raw  # 单图离线诊断不冒充连续确认，也不积累轨迹。
        return {'detections':detections,'raw_detections':raw,'confirmation_frames':3,
                'single_image_raw_only':'frame_id' not in request,'width':width,'height':height,
                'confidence_threshold':DETECTION_CONFIDENCE_THRESHOLD,
                'inference_s':inference,'output_shapes':[list(a.shape) for a in outputs],
                'contract':self.contract,'engine':'P02 ONNX FP32' if self.provider=='onnx_cpu' else 'K541 P02 RKNN FP16 + 3-frame confirmation',
                'provider':'onnx_cpu' if self.provider=='onnx_cpu' else 'rknpu_core0'}

    def close(self):
        if self.provider=='rknn':self.rec.release()


class LlamaHTTP:
    def __init__(self,cfg):
        self.cfg=cfg;self.url=cfg['url'].rstrip('/')
        if not self.url.startswith('http://127.0.0.1:'):raise ValueError('Use the managed local model server')
        self.system=Path(cfg['system']).read_text(encoding='utf-8')
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def run(self,request):
        body={'messages':[{'role':'system','content':self.system},{'role':'user','content':request['text']}],
            'temperature':0,'top_p':1,'top_k':0,'min_p':0,'repeat_penalty':1,'seed':20261024,
            'max_tokens':80,'cache_prompt':False,'chat_template_kwargs':{'enable_thinking':False}}
        req=urllib.request.Request(self.url+'/v1/chat/completions',data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        with self.opener.open(req,timeout=self.cfg.get('timeout_s',15)) as response:data=response.read(65537)
        if len(data)>65536:raise ValueError('Oversize model reply')
        result=json.loads(data);choice=result['choices'][0];message=choice['message']
        return {'content':message.get('content') or '', 'finish_reason':choice.get('finish_reason'),
                'reasoning':message.get('reasoning_content') or '', 'usage':result.get('usage'),
                'engine':self.cfg.get('label','specialized GGUF / llama.cpp')}


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--kind',choices=['asr','vision','llm','camera','chat','tts'],required=True);args=ap.parse_args()
    config=json.loads(args.config.read_text(encoding='utf-8'))
    cfg=config['camera'] if args.kind=='camera' else config['models'][args.kind]
    from ai_coordinator.camera_sources import create_camera
    from ai_coordinator.interactive_workers import Chat,DynamicTTS
    factory={'asr':ASR,'vision':Vision,'llm':LlamaHTTP,'camera':create_camera,'chat':Chat,'tts':DynamicTTS}[args.kind];model=None
    try:
        started=time.monotonic();model=factory(cfg)
        print('\n'+PREFIX+json.dumps({'ready':True,'load_s':time.monotonic()-started}),flush=True)
        while True:
            line=sys.stdin.buffer.readline(65537)
            if not line:break
            if len(line)>65536:raise ValueError('Oversize worker request')
            request=json.loads(line);request_id=request.pop('id');start=time.monotonic()
            try:
                result={'health':'ready'} if request.get('op')=='health' else model.run(request)
                result={'id':request_id,'ok':True,'elapsed_s':time.monotonic()-start,'result':result}
            except Exception as exc:result={'id':request_id,'ok':False,'error':type(exc).__name__+': '+str(exc)[:600]}
            print('\n'+PREFIX+json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
    finally:
        if model and hasattr(model,'close'):model.close()


if __name__=='__main__':main()
