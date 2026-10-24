"""Fit a separate final-answer LoRA; keep source weights and board environment intact."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import time


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--base',type=Path,required=True);args=ap.parse_args()
    import torch
    from peft import LoraConfig,get_peft_model
    from safetensors import safe_open
    from transformers import AutoModelForImageTextToText,AutoTokenizer
    out=args.work;base=args.base;protocol=json.loads((out/'protocol.json').read_text(encoding='utf-8'))
    for name,digest in protocol['files'].items():assert hashlib.sha256((out/name).read_bytes()).hexdigest()==digest,name
    if (out/'adapter').exists() or (out/'merged_hf').exists():raise FileExistsError('Choose a fresh training output')
    rows=json.loads((out/'train.json').read_text(encoding='utf-8'))
    system=(out/'system.txt').read_text(encoding='utf-8');cfg=protocol['training']
    torch.manual_seed(protocol['seed']);random.seed(protocol['seed'])
    tokenizer=AutoTokenizer.from_pretrained(base,local_files_only=True)
    samples=[]
    for row in rows:
        prompt=tokenizer.apply_chat_template([{'role':'system','content':system},{'role':'user','content':row['text']}],
                     tokenize=True,return_dict=False,add_generation_prompt=True,enable_thinking=False)
        completion=tokenizer.encode(json.dumps(row['expected'],ensure_ascii=False,separators=(',',':'))+'<|im_end|>',add_special_tokens=False)
        samples.append((prompt+completion,[-100]*len(prompt)+completion))
    model=AutoModelForImageTextToText.from_pretrained(base,dtype=torch.bfloat16,attn_implementation='sdpa',local_files_only=True).to('cuda')
    parameters=dict(model.named_parameters());restored=[]
    for source in base.glob('*.safetensors'):
        with safe_open(source,framework='pt',device='cpu') as f:
            for key in f.keys():
                if key in parameters and f.get_slice(key).get_dtype()=='F32':
                    parameters[key].data=f.get_tensor(key).to(parameters[key].device);restored.append(key)
    names=[name for name,module in model.named_modules() if name.startswith('model.language_model.layers.')
           and isinstance(module,torch.nn.Linear) and name.rsplit('.',1)[-1] in
           ('q_proj','k_proj','v_proj','o_proj','in_proj_qkv','in_proj_z','in_proj_b','in_proj_a','out_proj','gate_proj','up_proj','down_proj')]
    assert names
    model=get_peft_model(model,LoraConfig(r=cfg['rank'],lora_alpha=cfg['alpha'],lora_dropout=0,
                       target_modules=names,bias='none',task_type='CAUSAL_LM'))
    model.config.use_cache=False;model.gradient_checkpointing_enable(gradient_checkpointing_kwargs={'use_reentrant':False})
    model.enable_input_require_grads();model.train()
    assert not any(p.requires_grad for n,p in model.named_parameters() if '.visual.' in n)
    trainable=[p for p in model.parameters() if p.requires_grad]
    optimizer=torch.optim.AdamW(trainable,lr=cfg['lr'],weight_decay=.01)
    rng=random.Random(protocol['seed']);order=list(range(len(samples)));rng.shuffle(order);cursor=0;start=time.perf_counter();losses=[]
    with (out/'training_log.jsonl').open('x',encoding='utf-8') as log:
        for step in range(cfg['steps']):
            chosen=[]
            for _ in range(cfg['batch']):
                if cursor==len(order):rng.shuffle(order);cursor=0
                chosen.append(samples[order[cursor]]);cursor+=1
            width=max(len(s[0]) for s in chosen)
            inputs=torch.full((len(chosen),width),tokenizer.pad_token_id or tokenizer.eos_token_id,device='cuda',dtype=torch.long)
            labels=torch.full_like(inputs,-100);mask=torch.zeros_like(inputs)
            for i,(tokens,lab) in enumerate(chosen):
                inputs[i,:len(tokens)]=torch.tensor(tokens,device='cuda');labels[i,:len(lab)]=torch.tensor(lab,device='cuda');mask[i,:len(tokens)]=1
            optimizer.zero_grad(set_to_none=True)
            lr=cfg['lr']*min(1,(step+1)/8)*(.1+.9*.5*(1+math.cos(math.pi*step/cfg['steps'])))
            for group in optimizer.param_groups:group['lr']=lr
            loss=model(input_ids=inputs,attention_mask=mask,labels=labels,use_cache=False).loss
            if not torch.isfinite(loss):raise RuntimeError('Nonfinite loss')
            loss.backward();norm=torch.nn.utils.clip_grad_norm_(trainable,1)
            if not torch.isfinite(norm):raise RuntimeError('Nonfinite gradients')
            optimizer.step();value=float(loss.detach());losses.append(value)
            row={'step':step+1,'loss':value,'grad_norm':float(norm),'lr':lr,'elapsed_s':time.perf_counter()-start}
            log.write(json.dumps(row)+'\n');log.flush()
            if step==0 or (step+1)%16==0:print('TRAIN',json.dumps(row),flush=True)
    model.save_pretrained(out/'adapter')
    merged=model.merge_and_unload();merged.config.use_cache=True;merged.save_pretrained(out/'merged_hf',safe_serialization=True)
    # Copy tokenizer bytes verbatim: add_bos_token and the shared chat template must stay fixed.
    for source in base.iterdir():
        if source.is_file() and source.suffix in ('.json','.jinja','.txt') and source.name!='config.json' and not source.name.endswith('.index.json'):
            shutil.copy2(source,out/'merged_hf'/source.name)
    metadata={'base':str(base),'train_samples':len(rows),'steps':cfg['steps'],'train_seconds':time.perf_counter()-start,
              'trainable_parameters':sum(p.numel() for p in trainable),'target_modules':names,
              'restored_source_fp32':restored,'loss_first':losses[0],'loss_last':losses[-1],
              'vision_frozen':True,'thinking':False,'labels':'synthetic oracle plus earlier training replay; not teacher reasoning',
              'source_protocol_sha256':hashlib.sha256((out/'protocol.json').read_bytes()).hexdigest(),
              'torch':torch.__version__,'gpu':torch.cuda.get_device_name(),'peak_gpu_bytes':torch.cuda.max_memory_allocated()}
    (out/'training_metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    print('TRAINING_COMPLETE',flush=True)


if __name__=='__main__':main()
