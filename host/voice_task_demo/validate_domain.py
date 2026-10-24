"""Audit the isolated LoRA merge against its parent without changing either model."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--base',type=Path,required=True);args=ap.parse_args()
    import torch
    from safetensors import safe_open
    torch.set_num_threads(4)
    metadata=json.loads((args.work/'training_metadata.json').read_text(encoding='utf-8'))
    allowed={n+'.weight' for n in metadata['target_modules']}
    parent=next(args.base.glob('*.safetensors'));child=args.work/'merged_hf/model.safetensors'
    changed=[];vision=[];nonfinite=[];unchanged=0
    with safe_open(parent,framework='pt',device='cpu') as a,safe_open(child,framework='pt',device='cpu') as b:
        assert set(a.keys())==set(b.keys()),'Tensor names changed'
        for key in b.keys():
            x=a.get_tensor(key);y=b.get_tensor(key)
            if not torch.isfinite(y).all():nonfinite.append(key)
            equal=x.dtype==y.dtype and torch.equal(x,y)
            if equal:unchanged+=1
            else:
                changed.append(key)
                if key not in allowed:raise ValueError('Unexpected changed tensor: '+key)
            if 'visual.' in key:
                assert equal,key
                vision.append(key)
    assert changed and vision and not nonfinite
    for name in ('tokenizer.json','tokenizer_config.json','vocab.json','merges.txt','chat_template.jinja'):
        assert (args.base/name).read_bytes()==(args.work/'merged_hf'/name).read_bytes(),name
    result={'changed_tensors':len(changed),'unchanged_tensors':unchanged,'visual_tensors_identical':len(vision),
            'nonfinite_tensors':nonfinite,'only_lora_targets_changed':True,'tokenizer_files_identical':True,
            'merged_sha256':hashlib.sha256(child.read_bytes()).hexdigest(),
            'gguf_sha256':hashlib.sha256((args.work/'domain-Q8_0.gguf').read_bytes()).hexdigest(),
            'scope':'Raw weights audited; text runtime is separately tested. Does not prove visual accuracy retention or RKLLM compatibility.'}
    (args.work/'export_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
