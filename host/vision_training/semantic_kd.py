"""Match teacher/student predictions by annotated object, not anchor index."""
import torch
import torch.nn.functional as F
from ultralytics.utils.tal import make_anchors

def decoded_xyxy(raw,points,strides):
    centre=points.T[None]*strides.T[None]
    return torch.cat((centre-raw[:,:2]*strides.T[None],centre+raw[:,2:]*strides.T[None]),dim=1).transpose(1,2)

def iou(box,boxes):
    intersection=(torch.minimum(box[2:],boxes[:,2:])-torch.maximum(box[:2],boxes[:,:2])).clamp_min(0).prod(dim=1)
    a=(box[2:]-box[:2]).clamp_min(0).prod();b=(boxes[:,2:]-boxes[:,:2]).clamp_min(0).prod(dim=1)
    return intersection/(a+b-intersection).clamp_min(1e-9)

def semantic_loss(student,teacher,batch,head_strides):
    ss=student['scores'];ts=teacher['scores'].detach();sb=student['boxes'];tb=teacher['boxes'].detach()
    if ss.shape!=ts.shape or sb.shape!=tb.shape:raise ValueError('Expected aligned feature pyramid shapes')
    points,strides=make_anchors(student['feats'],head_strides,.5)
    centres=points*strides;sboxes=decoded_xyxy(sb.detach(),points,strides);tboxes=decoded_xyxy(tb,points,strides)
    sp=ss.detach().sigmoid();tp=ts.sigmoid();teacher_classes=tp.argmax(dim=1)
    h,w=batch['img'].shape[-2:];shape=torch.tensor([w,h,w,h],device=ss.device,dtype=sb.dtype)
    terms=[];weights=[];different=0;object_count=0
    for b in range(ss.shape[0]):
        rows=batch['batch_idx'].view(-1)==b;gt=batch['bboxes'][rows]*shape;labels=batch['cls'].view(-1)[rows].long();used=set()
        for box,label in zip(gt,labels):
            c=int(label)
            if c not in (0,1):continue
            object_count+=1;truth=torch.cat((box[:2]-box[2:]/2,box[:2]+box[2:]/2))
            inside=((centres>=truth[:2])&(centres<=truth[2:])).all(dim=1)
            tiou=iou(truth,tboxes[b]);siou=iou(truth,sboxes[b])
            valid=inside&(tp[b,c]>=.25)&(teacher_classes[b]==c)&(tiou>=.5)
            if not valid.any() or not inside.any():continue
            tmetric=(tp[b,c].sqrt()*tiou.pow(6)).masked_fill(~valid,-1)
            smetric=(sp[b,c].sqrt()*siou.pow(6)).masked_fill(~inside,-1)
            tindex=int(tmetric.argmax());sindex=int(smetric.argmax())
            if smetric[sindex]<=0 or sindex in used:continue
            target=torch.cat((centres[sindex]-tboxes[b,tindex,:2],tboxes[b,tindex,2:]-centres[sindex]))/strides[sindex]
            if (target<0).any():continue
            used.add(sindex)
            slog=ss[b,:,sindex]/2;tlog=ts[b,:,tindex]/2
            cls=(F.binary_cross_entropy_with_logits(slog,tlog.sigmoid(),reduction='none')-
                 F.binary_cross_entropy_with_logits(tlog,tlog.sigmoid(),reduction='none')).mean()*4
            reg=F.smooth_l1_loss(sb[b,:,sindex],target,reduction='mean')/4
            terms.append(cls+.25*reg);weights.append(tp[b,c,tindex]);different+=int(tindex!=sindex)
    if terms:
        weights=torch.stack(weights);loss=(torch.stack(terms)*weights).sum()/weights.sum()
    else:loss=ss.sum()*0
    if not torch.isfinite(loss):raise ValueError('Nonfinite semantic KD')
    return loss,{'matched_objects':len(terms),'different_anchor_pairs':different,'tool_objects':object_count}

def smoke():
    features=[torch.zeros(1,1,2,2)];student_scores=torch.full((1,4,4),-8.);teacher_scores=student_scores.clone()
    student_boxes=torch.full((1,4,4),.5);teacher_boxes=student_boxes.clone()
    student_scores[0,1,3]=2;teacher_scores[0,1,0]=2
    student_boxes[0,:,3]=torch.tensor([1.5,1.5,.5,.5]);teacher_boxes[0,:,0]=torch.tensor([.5,.5,1.5,1.5])
    s={'scores':student_scores.requires_grad_(),'boxes':student_boxes.requires_grad_(),'feats':features}
    t={'scores':teacher_scores.requires_grad_(),'boxes':teacher_boxes.requires_grad_()}
    batch={'img':torch.zeros(1,3,16,16),'batch_idx':torch.tensor([0]),'bboxes':torch.tensor([[.5,.5,1.,1.]]),'cls':torch.tensor([[1.]])}
    value,report=semantic_loss(s,t,batch,[8]);assert abs(float(value.detach()))<1e-6 and report['different_anchor_pairs']==1
    changed={**s,'scores':s['scores']-torch.nn.functional.one_hot(torch.tensor([1]),4).view(1,4,1).float()}
    value,report=semantic_loss(changed,t,batch,[8]);value.backward()
    assert value>0 and s['scores'].grad[0,1,3]<0 and s['scores'].grad[0,1,0]==0
    assert t['scores'].grad is None and t['boxes'].grad is None
    batch['cls']=torch.tensor([[2.]])
    _,report=semantic_loss(s,t,batch,[8]);assert report['matched_objects']==0
    print('PASS: equal decoded object at distinct anchors has zero KD; mismatch increases only matched student score; teacher detached; beverage GT excluded')

if __name__=='__main__':smoke()
