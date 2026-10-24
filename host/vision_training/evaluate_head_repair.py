"""CPU ONNX evaluation of a frozen head-repair experiment; no test/NPU access."""
import argparse
from collections import Counter
import copy
import hashlib
import json
import os
from pathlib import Path
import sys

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--experiment',type=Path,required=True)
    ap.add_argument('--arm',action='append',choices=['head_supervised','head_teacher_kd','head_semantic_kd'])
    ap.add_argument('--output-name',default='evaluation');args=ap.parse_args()
    root=args.experiment.resolve();protocol=json.loads((root/'protocol.json').read_text(encoding='utf-8'))
    research=Path(protocol['research_root']);sys.path.insert(0,str(research))
    os.environ['YOLO_CONFIG_DIR']=str(root/'runtime/ultralytics')
    from specialist_head_routing import decode
    from verify_variants import prepare,postprocess,ground_truth,metrics
    from ultralytics import YOLO
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    torch.set_num_threads(4)
    if Path(args.output_name).name!=args.output_name:raise ValueError('Output name must be a directory name')
    out=root/args.output_name;out.mkdir(exist_ok=False)
    source=root/'inputs/student.pt'
    if sha(source)!=protocol['student_sha256']:raise ValueError('Source changed')
    baseline=YOLO(str(source)).model.float().cpu().eval()
    base_sidecar=json.loads((research/'specialist_head_routing/export/nano_specialist_raw.json').read_text(encoding='utf-8'))
    if base_sidecar['source_checkpoint_sha256']!=sha(source):raise ValueError('Wrong specialist baseline')
    class Wrapper(torch.nn.Module):
        def __init__(self,base):super().__init__();self.base=base
        def forward(self,x):
            raw=self.base(x)[1]
            return raw['one2one']['boxes'],raw['one2one']['scores'],raw['one2many']['boxes'],raw['one2many']['scores']
    base_wrap=Wrapper(baseline).eval();inputs={};base_records={};base_raw={}
    for split,rows in protocol['validation'].items():
        base_records[split]=[]
        for row in rows:
            path=Path(row['image']);label=Path(str(path).replace('/images/','/labels/').replace('\\images\\','\\labels\\')).with_suffix('.txt')
            if sha(path)!=row['sha256'] or sha(label)!=row['label_sha256']:raise ValueError('Frozen evaluation data changed')
            x,trans=prepare(path,640);w,h=trans[:2]
            with torch.no_grad():raw=[v.numpy() for v in base_wrap(torch.from_numpy(x))]
            b,s=decode(raw,base_sidecar['contract'],base_sidecar)
            inputs[str(path)]=(x,trans);base_raw[str(path)]=raw
            base_records[split].append({'image':str(path),'sha256':row['sha256'],'width':w,'height':h,
                'gt':ground_truth(path,w,h),'detections':postprocess(b,s,trans)})
    reproduced={s:metrics(rows) for s,rows in base_records.items()}
    if any(abs(reproduced[s]['AP50_95']-protocol['baseline_metrics'][s]['AP50_95'])>1e-5 or
           reproduced[s]['strict']!=protocol['baseline_metrics'][s]['strict'] for s in reproduced):
        save(out/'baseline_mismatch.json',reproduced);raise ValueError('Baseline did not reproduce')
    save(out/'baseline_metrics.json',reproduced)
    original_ops=Counter(n.op_type for n in onnx.load(str(research/'specialist_head_routing/export/nano_specialist_raw.onnx')).graph.node)
    reports={}
    for arm in (args.arm or ['head_supervised','head_teacher_kd']):
        status=json.loads((root/(arm+'_status.json')).read_text(encoding='utf-8'))
        checkpoint=Path(status['checkpoint'])
        if status['state']!='complete' or sha(checkpoint)!=status['sha256']:raise ValueError('Incomplete/changed arm')
        folder=out/arm;folder.mkdir();model=YOLO(str(checkpoint)).model.float().cpu().eval();wrapper=Wrapper(model).eval()
        prefix='model.'+str(len(model.model)-1)+'.one2one_';ref=baseline.state_dict();state=model.state_dict()
        frozen_exact=all(torch.equal(v,state[k]) for k,v in ref.items() if not k.startswith(prefix))
        if not frozen_exact:raise ValueError('Non-head state drifted')
        path=folder/'nano_head_repair_raw.onnx'
        torch.onnx.export(wrapper,torch.zeros(1,3,640,640),str(path),input_names=['images'],
            output_names=base_sidecar['output_names'],opset_version=17,dynamo=False,do_constant_folding=True)
        graph=onnx.load(str(path));onnx.checker.check_model(graph)
        sidecar=copy.deepcopy(base_sidecar);sidecar.update(source_checkpoint_sha256=sha(checkpoint),onnx_sha256=sha(path),
            scope='Head-only adaptive CPU development candidate; no board validation',experiment_protocol_sha256=sha(root/'protocol.json'))
        save(path.with_suffix('.json'),sidecar)
        opts=ort.SessionOptions();opts.intra_op_num_threads=4;opts.inter_op_num_threads=1
        session=ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
        results={};parity=[]
        for split,rows in protocol['validation'].items():
            predictions=[]
            for row in rows:
                image=row['image'];x,trans=inputs[image];w,h=trans[:2]
                outputs=session.run(None,{'images':x});boxes,scores=decode(outputs,sidecar['contract'],sidecar)
                with torch.no_grad():raw=[v.numpy() for v in wrapper(torch.from_numpy(x))]
                tb,ts=decode(raw,sidecar['contract'],sidecar)
                beverage_errors=[float(np.max(np.abs(raw[i]-base_raw[image][i]))) for i in (2,3)]
                p={'image':image,'box_error_px':float(np.max(np.abs(boxes-tb))),
                   'score_error':float(np.max(np.abs(scores-ts))),'source_one2many_raw_errors':beverage_errors}
                p['passed']=p['box_error_px']<=.02 and p['score_error']<=1e-4 and max(beverage_errors)==0
                parity.append(p)
                predictions.append({'image':image,'sha256':row['sha256'],'width':w,'height':h,
                    'gt':ground_truth(Path(image),w,h),'detections':postprocess(boxes,scores,trans)})
            results[split]=metrics(predictions)
            with (folder/(split+'.jsonl')).open('x',encoding='utf-8') as f:
                for row in predictions:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        gates=protocol['gates'];reasons=[]
        for split,key in [('hard_val','hard'),('mini_val','mini'),('old_val','old')]:
            m=results[split]
            if m['AP50_95']<gates[key+'_ap_min']:reasons.append(key+' AP below frozen threshold')
            if m['strict']['tp']<gates[key+'_tp_min']:reasons.append(key+' recall below frozen threshold')
            if m['strict']['fp']>gates[key+'_fp_max']:reasons.append(key+' FP exceeds frozen threshold')
        for name,value in reproduced['old_val']['per_class'].items():
            candidate=results['old_val']['per_class'].get(name)
            if value is not None and (candidate is None or candidate<value-gates['old_per_class_drop_max']):reasons.append('old class regressed: '+name)
        if not all(p['passed'] for p in parity):reasons.append('Output parity / unchanged branch failed')
        params=sum(p.numel() for p in model.parameters());base_params=sum(p.numel() for p in baseline.parameters())
        ops=Counter(n.op_type for n in graph.graph.node)
        if params!=base_params or ops!=original_ops:reasons.append('Inference graph/parameter budget changed')
        report={'metrics':results,'parity':parity,'frozen_state_exact':frozen_exact,'parity_passed':all(p['passed'] for p in parity),
            'parameters':params,'onnx_bytes':path.stat().st_size,'op_counts':dict(ops),'op_counts_match_baseline':ops==original_ops,
            'passed_development_gates':not reasons,'reasons':reasons,'checkpoint_sha256':sha(checkpoint),'onnx_sha256':sha(path),
            'test_used':False,'board_tested':False,'default_replaced':False}
        reports[arm]=report;save(folder/'result.json',report)
        print(arm,json.dumps({'metrics':results,'passed':not reasons,'reasons':reasons},ensure_ascii=True),flush=True)
        del session,wrapper,model
    save(out/'paired_result.json',{'baseline':reproduced,'arms':reports,
        'protocol_sha256':sha(root/'protocol.json'),'test_used':False,'board_tested':False,'default_replaced':False,
        'interpretation':'Single-seed adaptive development evidence; reused tiny image sets and imperfect labels. No generalization, significance, NPU speed or physical readiness claim.'})

if __name__=='__main__':main()
