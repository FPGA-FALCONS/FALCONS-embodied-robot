"""Post-hoc ASR substitution on unchanged development audio, weights and pictures."""
import argparse
import json
from pathlib import Path
import statistics
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'voice_bridge'))
from pipeline import process_wav
from executor import replay
from infer import PrivateParser
from run_eval import char_error,expected_status,sha
from sensevoice import bounded_decode


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--asr-model',type=Path,required=True)
    ap.add_argument('--pilot',type=Path,required=True);args=ap.parse_args()
    work=args.work;out=args.out;out.mkdir(parents=True,exist_ok=False)
    protocol={'scope':'Post-hoc synthetic development ASR substitution; neither blind nor board validation',
        'engine':'SenseVoiceSmall INT8 / sherpa-onnx','provider':'cpu','threads':2,
        'language':'zh','use_itn':True,'hotwords':False,'deadline_s':15,
        'files':{str(p):sha(p) for p in (args.asr_model/'model.int8.onnx',args.asr_model/'tokens.txt',
            ROOT/'sensevoice.py',ROOT.parent/'voice_bridge/domain_vocab.py',
            work/'cases.json',work/'spoken_alias_probe/cases.json',work/'domain-Q8_0.gguf',args.pilot/'student-Q8_0.gguf')}}
    corpora=[('main',work),('spoken_alias_probe',work/'spoken_alias_probe')]
    cases={};sources=[]
    for group,folder in corpora:
        cases.update({c['id']:c for c in json.loads((folder/'cases.json').read_text(encoding='utf-8'))})
        sources.extend(dict(r,group=group) for r in json.loads((folder/'audio/manifest.json').read_text(encoding='utf-8-sig')))
    protocol['audio_sha256']={r['wav']:sha(r['wav']) for r in sources}
    (out/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2),encoding='utf-8')
    vision=json.loads((work/'e2e_run_1/vision_results.json').read_text(encoding='utf-8'))
    transcripts=[]
    with (out/'asr_results.jsonl').open('x',encoding='utf-8') as log:
        for index,source in enumerate(sources):
            event=process_wav(source['wav'],args.asr_model,out/'asr'/source['id'],decoder=bounded_decode)
            row={**source,'asr':event,'character_error':char_error(cases[source['case_id']]['text'],event['text'])}
            transcripts.append(row);log.write(json.dumps(row,ensure_ascii=False)+'\n');log.flush()
            print(f"ASR {index+1}/{len(sources)}: {event['text']}",flush=True)
    results=[]
    for name,model in [('pilot',args.pilot/'student-Q8_0.gguf'),('domain',work/'domain-Q8_0.gguf')]:
        with PrivateParser(model,work,out/name) as parser:
            with (out/(name+'_results.jsonl')).open('x',encoding='utf-8') as log:
                for source in transcripts:
                    event=source['asr'];case=cases[source['case_id']]
                    if event['status']!='recognized' or not event['task_text']:
                        answer={'task':{'i':'unknown'},'valid':event['status']=='recognized','source':'asr_or_policy'}
                    else:answer=parser.parse(event['task_text'])
                    correct=bool(answer['valid'] and answer['task']==case['expected'] and event['status']=='recognized')
                    execution=replay(answer['task'],case['scenes'],vision,case['simulated_home_ack'])
                    row={**source,'model':name,'parser':answer,'expected':case['expected'],'semantic_correct':correct,
                        'expected_outcome_matched':correct and execution['status']==expected_status(case['expected']),
                        'execution':execution,'physical_task_completed':False}
                    results.append(row);log.write(json.dumps(row,ensure_ascii=False)+'\n');log.flush()
    summary={'asr':{},'models':{}}
    for group,_ in corpora:
        rows=[r for r in transcripts if r['group']==group]
        summary['asr'][group]={'n':len(rows),'cer':sum(r['character_error']['edits'] for r in rows)/sum(r['character_error']['reference_chars'] for r in rows),
            'median_processing_s':statistics.median(r['asr']['processing_s'] for r in rows),
            'errors':sum(r['asr']['status']!='recognized' for r in rows)}
        for name in ('pilot','domain'):
            selected=[r for r in results if r['group']==group and r['model']==name]
            summary['models'].setdefault(name,{})[group]={'n':len(selected),
                'semantic_correct':sum(r['semantic_correct'] for r in selected),
                'expected_outcome_matched':sum(r['expected_outcome_matched'] for r in selected),
                'false_search':sum(r['parser']['task']['i']=='search' and r['expected']['i']!='search' for r in selected),
                'missed_stop':sum(r['expected']['i']=='stop' and r['parser']['task']['i']!='stop' for r in selected)}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
