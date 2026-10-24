"""P3 plus one2one adaptation; beverage preservation uses frozen-parent one2many.

One2one features are detached in the installed head. P3 receives gradients from
the standard one2many supervised loss and the optional preservation loss.
No modifications to the Ultralytics package or inference graph.
"""
import argparse
import json
import os
from pathlib import Path
import time
from head_repair import sha,save

def allowed(name,prefix):
    return name.startswith(prefix) or name.startswith('model.16.')

def run_train(out,arm):
    protocol=json.loads((out/'protocol.json').read_text(encoding='utf-8'))
    os.environ['YOLO_CONFIG_DIR']=str(out/'runtime/ultralytics');os.environ['WANDB_MODE']='disabled'
    import torch
    import torch.nn.functional as F
    from ultralytics import YOLO
    from ultralytics.models.yolo.detect import DetectionTrainer
    from ultralytics.utils.tal import make_anchors
    source=out/'inputs/student.pt';teacher=out/'inputs/teacher.pt'
    if sha(source)!=protocol['student_sha256'] or sha(teacher)!=protocol['teacher_sha256']:raise ValueError('Frozen weights changed')
    weight=protocol['arms'][arm]
    class Trainer(DetectionTrainer):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs);self.handles=[];self.student_raw=None;self.teacher_raw=None;self.stats=[]
            self.add_callback('on_train_start',self.setup_repair)
            self.add_callback('on_train_epoch_start',self.freeze_norms)
            self.add_callback('on_train_epoch_end',self.record)
            self.add_callback('on_train_end',self.cleanup)
        def setup_repair(self,trainer):
            self.prefix='model.'+str(len(self.model.model)-1)+'.one2one_'
            for name,param in self.model.named_parameters():param.requires_grad_(allowed(name,self.prefix))
            self.frozen={k:v.detach().cpu().clone() for k,v in self.model.state_dict().items() if not allowed(k,self.prefix)}
            cpu_rng=torch.get_rng_state();cuda_rng=torch.cuda.get_rng_state_all()
            self.teacher=YOLO(str(teacher)).model.to(self.device).float().eval()
            torch.set_rng_state(cpu_rng);torch.cuda.set_rng_state_all(cuda_rng)
            for param in self.teacher.parameters():param.requires_grad_(False)
            if self.teacher.names!=self.model.names:raise ValueError('Class mapping differs')
            self.handles=[self.model.model[-1].register_forward_hook(self.capture),self.model.register_forward_hook(self.distill)]
            self.p3_gradient_seen=False
            self.handles.append(next(self.model.model[16].parameters()).register_hook(self.p3_gradient))
            self.freeze_norms(self)
        def p3_gradient(self,grad):
            if torch.isfinite(grad).all() and grad.abs().max()>0:self.p3_gradient_seen=True
            return grad
        def freeze_norms(self,trainer):
            if not hasattr(self,'prefix'):return
            for i,module in enumerate(self.model.model[:-1]):module.train(i==16)
            head=self.model.model[-1];head.cv2.eval();head.cv3.eval()
            head.one2one_cv2.train();head.one2one_cv3.train()
        def preprocess_batch(self,batch):
            self.freeze_norms(self);batch=super().preprocess_batch(batch)
            with torch.no_grad():self.teacher_raw=self.teacher(batch['img'])[1]['one2many']
            return batch
        def capture(self,module,args,output):
            if self.model.training:self.student_raw=output['one2many']
        def distill(self,module,args,output):
            if not module.training or not args or not isinstance(args[0],dict):return output
            s=self.student_raw;t=self.teacher_raw;batch=args[0]
            self.student_raw=None;self.teacher_raw=None
            ss=s['scores'];ts=t['scores'].detach();sb=s['boxes'];tb=t['boxes'].detach()
            if ss.shape!=ts.shape or sb.shape!=tb.shape:raise ValueError('Raw geometry mismatch')
            confidence,classes=ts.sigmoid().max(dim=1)
            points,strides=make_anchors(s['feats'],self.model.model[-1].stride,.5)
            h,w=batch['img'].shape[-2:];shape=torch.tensor([w,h],device=points.device)
            centres=points*strides/shape;margin=strides/shape;inside=torch.zeros_like(confidence,dtype=torch.bool)
            for b in range(ss.shape[0]):
                selected=batch['batch_idx'].view(-1)==b;boxes=batch['bboxes'][selected];labels=batch['cls'].view(-1)[selected].long()
                keep=labels>=2;boxes=boxes[keep];labels=labels[keep]
                if not len(boxes):continue
                lo=boxes[:,:2]-boxes[:,2:]/2;hi=boxes[:,:2]+boxes[:,2:]/2
                spatial=((centres[:,None,:]>=lo[None,:,:]-margin[:,None,:])&(centres[:,None,:]<=hi[None,:,:]+margin[:,None,:])).all(dim=2)
                inside[b]=(spatial&(classes[b,:,None]==labels[None,:])).any(dim=1)
            mask=(confidence>=.25)&(classes>=2)&inside
            anchors=int(mask.sum());loss=ss.sum()*0
            if anchors:
                slog=ss.transpose(1,2)[mask]/2;tlog=ts.transpose(1,2)[mask]/2
                kl=(F.binary_cross_entropy_with_logits(slog,tlog.sigmoid(),reduction='none')-F.binary_cross_entropy_with_logits(tlog,tlog.sigmoid(),reduction='none')).mean(dim=1)*4
                reg=F.smooth_l1_loss(sb.transpose(1,2)[mask],tb.transpose(1,2)[mask],reduction='none').mean(dim=1)/4
                loss=((kl+.25*reg)*confidence[mask]).sum()/confidence[mask].sum()
            if not torch.isfinite(loss):raise ValueError('Nonfinite KD')
            self.stats.append((float(loss.detach()),anchors));base,items=output
            combined=base.clone();combined[1]=combined[1]+weight*loss*batch['img'].shape[0]
            return combined,items
        def record(self,trainer):
            if not self.p3_gradient_seen:raise RuntimeError('P3 received no finite nonzero gradient')
            state=self.model.state_dict()
            changed=[k for k,v in self.frozen.items() if not torch.equal(v,state[k].detach().cpu())]
            if changed:raise RuntimeError('Frozen state changed: '+','.join(changed[:5]))
            if any(p.grad is not None for p in self.teacher.parameters()):raise RuntimeError('Teacher received gradients')
            row={'epoch':self.epoch+1,'batches':len(self.stats),'anchors':sum(x[1] for x in self.stats),
                 'mean_kd':sum(x[0] for x in self.stats)/max(1,len(self.stats)),'frozen_state_exact':True,'p3_gradient_seen':self.p3_gradient_seen,
                 'trainable_parameters':sum(p.numel() for p in self.model.parameters() if p.requires_grad)}
            if hasattr(self,'semantic_counts'):
                row['semantic_matching']=self.semantic_counts.copy()
                self.semantic_counts={k:0 for k in self.semantic_counts}
            with (Path(self.save_dir)/'p3_repair.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
            self.stats=[]
        def cleanup(self,trainer):
            for handle in self.handles:handle.remove()
            self.handles=[]
    status_path=out/(arm+'_status.json')
    if status_path.exists():raise FileExistsError('Retain existing arm result')
    status={'state':'training','started_at':time.time(),'arm':arm};save(status_path,status)
    try:
        trainer=Trainer(overrides=dict(model=str(source),data=str(out/'dataset/train.yaml'),epochs=6,patience=1000,
            imgsz=640,batch=8,device=0,workers=0,cache=False,amp=False,optimizer='AdamW',lr0=protocol['lr0'],lrf=.1,
            seed=106,freeze=0,deterministic=True,warmup_epochs=0.,mosaic=0.,close_mosaic=0,mixup=0.,copy_paste=0.,
            degrees=0.,project=str(out/'runs'),name=arm,exist_ok=False,plots=False,save=True,verbose=False))
        trainer.train()
        baseline=YOLO(str(source)).model.float().cpu().eval();candidate=YOLO(str(trainer.last)).model.float().cpu().eval()
        state=candidate.state_dict();ref=baseline.state_dict();prefix='model.'+str(len(baseline.model)-1)+'.one2one_'
        restored=0
        for key in state:
            if not allowed(key,prefix):state[key]=ref[key].clone();restored+=1
        candidate.load_state_dict(state)
        if any(not torch.equal(candidate.state_dict()[k],v) for k,v in ref.items() if not allowed(k,prefix)):raise ValueError('Projection failed')
        checkpoint=out/'runs'/arm/'p3_projected.pt'
        torch.save({'model':candidate,'ema':None,'train_args':dict(candidate.args),'epoch':5},checkpoint)
        status.update(state='complete',finished_at=time.time(),checkpoint=str(checkpoint),sha256=sha(checkpoint),restored_tensors=restored)
    except BaseException as exc:status.update(state='failed',error=repr(exc));raise
    finally:save(status_path,status)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--arm',choices=['p3_supervised','p3_preserve'],required=True)
    args=ap.parse_args();run_train(args.out.absolute(),args.arm)
