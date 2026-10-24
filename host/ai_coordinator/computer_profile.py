"""Create an isolated PC replay profile using explicitly supplied existing assets."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys


def artifact(path):
    path=Path(path).absolute();digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return {'path':str(path),'bytes':path.stat().st_size,'sha256':digest.hexdigest()}


def main():
    ap=argparse.ArgumentParser()
    for name in ('root','asr-model','asr-tokens','vision-model','llm-model','llama-server','chat-template'):
        ap.add_argument('--'+name,type=Path,required=True)
    ap.add_argument('--input-root',type=Path,action='append',required=True)
    ap.add_argument('--gpu-layers',type=int,default=0)
    args=ap.parse_args();args.root.mkdir(parents=True,exist_ok=False)
    system=Path(__file__).with_name('profiles')/'c1b_system.txt'
    # Match the C1b training/evaluation prompt exactly (no trailing newline).
    copied=args.root/'system.txt';copied.write_text(system.read_text(encoding='utf-8').rstrip('\n'),encoding='utf-8')
    server_cwd=args.llm_model.absolute().parent
    template_copy=args.root/'chat_template.jinja';template_copy.write_bytes(args.chat_template.read_bytes())
    model_arg=os.path.relpath(args.llm_model.absolute(),server_cwd)
    template_arg=os.path.relpath(template_copy.absolute(),server_cwd)
    if not model_arg.isascii() or not template_arg.isascii():raise ValueError('Use an ASCII relative model/template layout for Windows llama.cpp')
    command=[str(args.llama_server.absolute()),'-m',model_arg,'-c','2048','-np','1','-t','8','-tb','8',
             '-ngl',str(args.gpu_layers),'--host','127.0.0.1','--port','18083','--jinja','--reasoning','off','--reasoning-budget','0',
             '--chat-template-file',template_arg,'--cache-ram','0','--ctx-checkpoints','0','--no-webui']
    assets={'asr_model':artifact(args.asr_model),'asr_tokens':artifact(args.asr_tokens),'vision':artifact(args.vision_model),
            'llm':artifact(args.llm_model),'system':artifact(copied),'chat_template':artifact(args.chat_template)}
    cfg={'root':str(args.root.absolute()),'mode':'simulation','execution_profile':'computer_replay',
         'language_profile':'zh-C1b-Q8-v1','model_loads_enabled':True,'port':18767,
         'input_roots':[str(p.absolute()) for p in args.input_root],
         'gateway':{'mode':'shadow','allow_motion':False},'microphone':{'device':None},'camera':{'enabled':False},
         'announcements':{'mode':'log','device':None},'artifacts':assets,
         'policy':{'frame_max_age_s':30,'confirm_gap_s':30},
         'asr_timeout_s':30,'llm_timeout_s':30,'vision_timeout_s':30,
         'models':{'asr':{'model':str(args.asr_model.absolute()),'tokens':str(args.asr_tokens.absolute()),'threads':2},
                   'vision':{'model':str(args.vision_model.absolute()),'size':640,'contract':'original','provider':'onnx_cpu','threads':4},
                   'llm':{'url':'http://127.0.0.1:18083','system':str(copied),'timeout_s':30,
                          'server_cwd':str(server_cwd),'server_command':command,
                          'label':'C1b Q8 / llama.cpp computer replay'}}}
    path=args.root/'config.json';path.write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'config':str(path),'artifacts':assets},ensure_ascii=True))


if __name__=='__main__':main()
