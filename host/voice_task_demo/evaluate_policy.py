"""Post-user-request vocabulary policy ablation; preserve the original evaluation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'voice_bridge'))
from domain_vocab import command_text
from pipeline import process_wav
from executor import replay
from infer import PrivateParser
from run_eval import expected_status


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--pilot',type=Path,required=True);ap.add_argument('--asr-model',type=Path,required=True)
    ap.add_argument('--out',type=Path)
    args=ap.parse_args();work=args.work;out=args.out or work/'policy_after_user_request';out.mkdir(exist_ok=False)
    base=work/'e2e_run_1';vision=json.loads((base/'vision_results.json').read_text(encoding='utf-8'))
    cases={c['id']:c for c in json.loads((work/'cases.json').read_text(encoding='utf-8'))}
    corpus=[json.loads(x) for x in (base/'asr_results.jsonl').read_text(encoding='utf-8').splitlines()]
    corpus=[r for r in corpus if r['mode']=='hotwords']
    protocol={'trigger':'Explicit user request to disallow 交代 in accepted robot command text after initial run',
        'policy_sha256':hashlib.sha256((ROOT.parent/'voice_bridge/domain_vocab.py').read_bytes()).hexdigest(),
        'input_sha256':hashlib.sha256((base/'asr_results.jsonl').read_bytes()).hexdigest(),
        'scope':'Post-hoc development ablation; no retraining and no ASR transcript editing',
        'supplementary_probe':'Chinese spoken object names; original AD failures remain in original scores'}
    (out/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2),encoding='utf-8')
    supplement=work/'spoken_alias_probe'
    extra={c['id']:c for c in json.loads((supplement/'cases.json').read_text(encoding='utf-8'))}
    for source in json.loads((supplement/'audio/manifest.json').read_text(encoding='utf-8-sig')):
        event=process_wav(source['wav'],args.asr_model,out/'asr_supplement'/source['id'],hotwords_file=work/'hotwords.txt')
        corpus.append({'id':source['id'],'case_id':source['case_id'],'voice':source['voice'],'mode':'spoken_alias_probe',
                       'text':event['text'],'status':event['status'],'elapsed_s':event['processing_s']})
    (out/'supplement_asr.json').write_text(json.dumps([r for r in corpus if r['mode']=='spoken_alias_probe'],ensure_ascii=False,indent=2),encoding='utf-8')
    cases.update(extra);results=[]
    for name,model in [('pilot',args.pilot/'student-Q8_0.gguf'),('domain',work/'domain-Q8_0.gguf')]:
        with PrivateParser(model,work,out/name) as parser:
            with (out/(name+'_results.jsonl')).open('x',encoding='utf-8') as log:
                for source in corpus:
                    policy=command_text(source['text'])
                    if policy['blocked']:
                        answer={'task':{'i':'unknown'},'valid':True,'source':'domain_policy','elapsed_s':0}
                    else:answer=parser.parse(policy['task_text'])
                    case=cases[source['case_id']];task=answer['task']
                    execution=replay(task,case['scenes'],vision,case['simulated_home_ack'])
                    correct=bool(answer['valid'] and task==case['expected'] and source['status']=='recognized')
                    row={**source,'model':name,'domain_policy':policy,'parser':answer,'expected':case['expected'],
                         'semantic_correct':correct,'expected_outcome_matched':correct and execution['status']==expected_status(case['expected']),
                         'execution':execution,'physical_task_completed':False}
                    results.append(row);log.write(json.dumps(row,ensure_ascii=False)+'\n');log.flush()
    summary={}
    for name in ('pilot','domain'):
        summary[name]={}
        for mode in ('hotwords','spoken_alias_probe'):
            rows=[r for r in results if r['model']==name and r['mode']==mode]
            summary[name][mode]={'n':len(rows),'semantic_correct':sum(r['semantic_correct'] for r in rows),
                'expected_outcome_matched':sum(r['expected_outcome_matched'] for r in rows),
                'false_search':sum(r['parser']['task']['i']=='search' and r['expected']['i']!='search' for r in rows),
                'blocked_ambiguous':sum(r['domain_policy']['blocked'] for r in rows)}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    for row in results:
        if row['model']=='domain' and row['mode']=='spoken_alias_probe':
            print(json.dumps({'voice':row['voice'],'case':row['case_id'],'raw':row['text'],'task':row['parser']['task'],
                              'outcome':row['expected_outcome_matched']},ensure_ascii=False))


if __name__=='__main__':main()
