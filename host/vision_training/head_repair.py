"""Fixed-budget, head-only repair of an existing four-class YOLO26 nano.

Run from an existing training environment. Never installs packages, reads a test
split, changes application defaults, or runs a board conversion/inference job.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import sys
import time

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

def prepare(out,research):
    out.mkdir(parents=True,exist_ok=False)
    source=research/'nano_distill_pilot/dataset'
    student=research/'nano_distill_pilot/evaluation/masked_output_distillation/model.pt'
    teacher=research.parent/'field_adaptation_20261021_v16_lab122/deliverables/yolo26_falcons_v17s_replay_640_best.pt'
    original=[Path(line) for line in (source/'train_replay.txt').read_text(encoding='utf-8').splitlines() if line.strip()]
    unique=sorted(set(original));dataset=out/'dataset';manifest=[];training_hashes=set();internal_val_hashes=set()
    for split,images in [('train',unique),('val',sorted((source/'images/val').glob('*')))]:
        for image in images:
            if not image.is_file():continue
            label=source/'labels'/split/(image.stem+'.txt')
            for line in label.read_text(encoding='utf-8').splitlines():
                if not line.strip():continue
                cls,cx,cy,w,h=map(float,line.split())
                if cls!=int(cls) or not 0<=cls<4 or not(0<=cx<=1 and 0<=cy<=1 and 0<w<=1 and 0<h<=1):raise ValueError('Bad label '+str(label))
            for kind,path in [('images',image),('labels',label)]:
                dest=dataset/kind/split/path.name;dest.parent.mkdir(parents=True,exist_ok=True)
                if kind=='images':
                    try:os.link(path,dest)
                    except OSError:shutil.copy2(path,dest)
                else:shutil.copy2(path,dest)
                digest=sha(dest);manifest.append({'path':dest.relative_to(out).as_posix(),'source':str(path),'sha256':digest})
                if kind=='images':(training_hashes if split=='train' else internal_val_hashes).add(digest)
    if training_hashes&internal_val_hashes:raise ValueError('Train/internal validation exact duplicate')
    (dataset/'train.txt').write_text('\n'.join((dataset/'images/train'/p.name).as_posix() for p in original)+'\n',encoding='utf-8')
    (dataset/'train.yaml').write_text('path: '+json.dumps(dataset.as_posix(),ensure_ascii=False)+'\ntrain: train.txt\nval: images/val\nnames:\n  0: tape\n  1: screwdriver\n  2: ad_calcium_milk\n  3: nutri_express\n',encoding='utf-8')
    refs={};seen_eval=set()
    for split in ('hard_val','mini_val','old_val'):
        rows=[json.loads(s) for s in (research/'cpu_boundary_ablation'/(split+'_fp32.jsonl')).read_text(encoding='utf-8').splitlines()]
        refs[split]=[]
        for row in rows:
            image=Path(row['image']);digest=sha(image)
            if digest!=row['sha256'] or digest in training_hashes:raise ValueError('Evaluation changed or overlaps training')
            refs[split].append({'image':str(image),'sha256':digest,'label_sha256':sha(Path(str(image).replace('/images/','/labels/').replace('\\images\\','\\labels\\')).with_suffix('.txt'))})
            seen_eval.add(digest)
    (out/'inputs').mkdir();shutil.copy2(student,out/'inputs/student.pt');shutil.copy2(teacher,out/'inputs/teacher.pt')
    base=json.loads((research/'specialist_head_routing/result.json').read_text(encoding='utf-8'))
    protocol={'v':1,'created_at':time.time(),'research_root':str(research),'student_sha256':sha(student),'teacher_sha256':sha(teacher),
        'data_manifest':manifest,'data_manifest_sha256':hashlib.sha256(json.dumps(manifest,sort_keys=True).encode()).hexdigest(),
        'validation':refs,'training_unique':len(unique),'training_weighted':len(original),'internal_val_unique':len(internal_val_hashes),
        'development_unique':len(seen_eval),'epochs':6,'seed':106,'batch':8,'imgsz':640,'lr0':2e-4,
        'arms':{'head_supervised':0.,'head_teacher_kd':.3},
        'training':'Only one2one_cv2/cv3 parameters; all shared/body/one2many parameters frozen and BN eval. Teacher computed in both arms. Final EMA last only.',
        'distillation':'one2one teacher/tool argmax prob>=.25, anchor within matching annotated tool GT expanded one stride; Bernoulli KL T2 + .25 SmoothL1(ltrb)/4',
        'projection':'Restore all non-one2one state tensors exactly from copied source to eliminate EMA roundoff drift; standard checkpoint, no runtime hooks.',
        'baseline_metrics':base['metrics'],
        'gates':{'hard_ap_min':max(.8,base['metrics']['hard_val']['AP50_95']-.01),
                 'mini_ap_min':base['metrics']['mini_val']['AP50_95']+.01,'old_ap_min':base['metrics']['old_val']['AP50_95']-.01,
                 'hard_tp_min':8,'hard_fp_max':1,'mini_tp_min':7,'mini_fp_max':0,'old_tp_min':12,'old_fp_max':1,
                 'old_per_class_drop_max':.03},
        'scope':'Adaptive single-seed pilot on repeatedly-used development sets; no independent generalization claim. Labels are proposals/review, not human gold.',
        'test_used':False,'board_tested':False,'default_replaced':False}
    save(out/'protocol.json',protocol)
    print('PREPARED',json.dumps({k:protocol[k] for k in ('training_unique','training_weighted','internal_val_unique','development_unique','gates')}),flush=True)

def run_train(out,arm):
    protocol=json.loads((out/'protocol.json').read_text(encoding='utf-8'))
    if arm=='head_semantic_kd':
        followup=json.loads((out/'semantic_followup.json').read_text(encoding='utf-8'))
        if followup['parent_protocol_sha256']!=sha(out/'protocol.json'):raise ValueError('Follow-up parent differs')
        protocol['arms'][arm]=followup['weight']
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
            for name,param in self.model.named_parameters():param.requires_grad_(name.startswith(self.prefix))
            self.frozen={k:v.detach().cpu().clone() for k,v in self.model.state_dict().items() if not k.startswith(self.prefix)}
            cpu_rng=torch.get_rng_state();cuda_rng=torch.cuda.get_rng_state_all()
            self.teacher=YOLO(str(teacher)).model.to(self.device).float().eval()
            torch.set_rng_state(cpu_rng);torch.cuda.set_rng_state_all(cuda_rng)
            for param in self.teacher.parameters():param.requires_grad_(False)
            if self.teacher.names!=self.model.names:raise ValueError('Class mapping differs')
            self.handles=[self.model.model[-1].register_forward_hook(self.capture),self.model.register_forward_hook(self.distill)]
            self.freeze_norms(self)
        def freeze_norms(self,trainer):
            if not hasattr(self,'prefix'):return
            for module in self.model.model[:-1]:module.eval()
            head=self.model.model[-1];head.cv2.eval();head.cv3.eval()
            head.one2one_cv2.train();head.one2one_cv3.train()
        def preprocess_batch(self,batch):
            self.freeze_norms(self);batch=super().preprocess_batch(batch)
            with torch.no_grad():self.teacher_raw=self.teacher(batch['img'])[1]['one2one']
            return batch
        def capture(self,module,args,output):
            if self.model.training:self.student_raw=output['one2one']
        def distill(self,module,args,output):
            if not module.training or not args or not isinstance(args[0],dict):return output
            s=self.student_raw;t=self.teacher_raw;batch=args[0]
            self.student_raw=None;self.teacher_raw=None
            ss=s['scores'];ts=t['scores'].detach();sb=s['boxes'];tb=t['boxes'].detach()
            if ss.shape!=ts.shape or sb.shape!=tb.shape:raise ValueError('Raw geometry mismatch')
            if arm=='head_semantic_kd':
                from semantic_kd import semantic_loss
                loss,parts=semantic_loss(s,t,batch,self.model.model[-1].stride)
                if not hasattr(self,'semantic_counts'):self.semantic_counts={k:0 for k in parts}
                for key,value in parts.items():self.semantic_counts[key]+=value
                self.stats.append((float(loss.detach()),parts['matched_objects']));base,items=output
                combined=base.clone();combined[1]=combined[1]+weight*loss*batch['img'].shape[0]
                return combined,items
            confidence,classes=ts.sigmoid().max(dim=1)
            points,strides=make_anchors(s['feats'],self.model.model[-1].stride,.5)
            h,w=batch['img'].shape[-2:];shape=torch.tensor([w,h],device=points.device)
            centres=points*strides/shape;margin=strides/shape;inside=torch.zeros_like(confidence,dtype=torch.bool)
            for b in range(ss.shape[0]):
                selected=batch['batch_idx'].view(-1)==b;boxes=batch['bboxes'][selected];labels=batch['cls'].view(-1)[selected].long()
                keep=labels<2;boxes=boxes[keep];labels=labels[keep]
                if not len(boxes):continue
                lo=boxes[:,:2]-boxes[:,2:]/2;hi=boxes[:,:2]+boxes[:,2:]/2
                spatial=((centres[:,None,:]>=lo[None,:,:]-margin[:,None,:])&(centres[:,None,:]<=hi[None,:,:]+margin[:,None,:])).all(dim=2)
                inside[b]=(spatial&(classes[b,:,None]==labels[None,:])).any(dim=1)
            mask=(confidence>=.25)&(classes<2)&inside
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
            state=self.model.state_dict()
            changed=[k for k,v in self.frozen.items() if not torch.equal(v,state[k].detach().cpu())]
            if changed:raise RuntimeError('Frozen state changed: '+','.join(changed[:5]))
            if any(p.grad is not None for p in self.teacher.parameters()):raise RuntimeError('Teacher received gradients')
            row={'epoch':self.epoch+1,'batches':len(self.stats),'anchors':sum(x[1] for x in self.stats),
                 'mean_kd':sum(x[0] for x in self.stats)/max(1,len(self.stats)),'frozen_state_exact':True,
                 'trainable_parameters':sum(p.numel() for p in self.model.parameters() if p.requires_grad)}
            if hasattr(self,'semantic_counts'):
                row['semantic_matching']=self.semantic_counts.copy()
                self.semantic_counts={k:0 for k in self.semantic_counts}
            with (Path(self.save_dir)/'head_repair.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
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
            if not key.startswith(prefix):state[key]=ref[key].clone();restored+=1
        candidate.load_state_dict(state)
        if any(not torch.equal(candidate.state_dict()[k],v) for k,v in ref.items() if not k.startswith(prefix)):raise ValueError('Projection failed')
        checkpoint=out/'runs'/arm/'head_only_projected.pt'
        torch.save({'model':candidate,'ema':None,'train_args':dict(candidate.args),'epoch':5},checkpoint)
        status.update(state='complete',finished_at=time.time(),checkpoint=str(checkpoint),sha256=sha(checkpoint),restored_tensors=restored)
    except BaseException as exc:status.update(state='failed',error=repr(exc));raise
    finally:save(status_path,status)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('phase',choices=['prepare','train']);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--research-root',type=Path);ap.add_argument('--arm',choices=['head_supervised','head_teacher_kd','head_semantic_kd']);args=ap.parse_args()
    if args.phase=='prepare':
        if args.research_root is None:ap.error('--research-root is required')
        prepare(args.out.resolve(),args.research_root.resolve())
    else:
        if args.arm is None:ap.error('--arm is required')
        run_train(args.out.resolve(),args.arm)

if __name__=='__main__':main()
