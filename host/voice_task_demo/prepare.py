"""Freeze four-object domain SFT and synthetic voice development cases before fitting."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import random

NAMES={'tape':'胶带','screwdriver':'螺丝刀','ad_calcium_milk':'AD钙奶','nutri_express':'营养快线'}
ALIASES={'tape':['胶带','胶布','交代','交带','胶袋'],
         'screwdriver':['螺丝刀','改锥','起子','罗丝刀','螺丝到'],
         'ad_calcium_milk':['AD钙奶','钙奶','A钙奶','A概乃'],
         'nutri_express':['营养快线','营养块线','营养快现']}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--pilot',type=Path,required=True);args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    original=json.loads((args.pilot/'teacher_filtered_train.json').read_text(encoding='utf-8'))
    rows=[{'id':'replay_'+r['id'],'text':r['text'],'expected':r['expected'],
           'origin':'previous_synthetic_training_replay'} for r in original]
    def add(text,obj):
        rows.append({'id':f'domain_{len(rows):04}','text':text,'expected':obj,'origin':'synthetic_template_oracle'})
    for target,aliases in ALIASES.items():
        for alias in aliases:
            add(f'执行搜寻：{alias}。',{'i':'search','t':target})
            add(f'当前搜索对象定为{alias}。',{'i':'search','t':target})
            add(f'搜索{alias}，成功时回到起始位置。',{'i':'search','t':target,'r':True})
            for phrase in (f'禁止搜索{alias}。',f'介绍一下{alias}。',f'把{alias}抓起来。',f'把{alias}递给我。'):
                add(phrase,{'i':'unknown'})
            for other in NAMES:
                if other==target:continue
                add(f'先搜寻{alias}，未找到才搜寻{NAMES[other]}。',{'i':'search','t':target,'f':other})
                add(f'先搜寻{alias}，未找到才搜寻{NAMES[other]}，成功找到任意一个再返回起点。',
                    {'i':'search','t':target,'f':other,'r':True})
                add(f'更正任务，不找{NAMES[other]}，而是搜索{alias}。',{'i':'search','t':target})
                add(f'同时寻找{alias}以及{NAMES[other]}。',{'i':'unknown'})
    for text in ('把事情交代清楚。','请交代一下任务。','把事情胶带清楚。','你说的交代是什么意思？',
                 '螺丝到了没有？','今天营养怎么样？','不要停止。','不用停下来。','寻找椅子。',
                 '找一下瓶子。','那个东西在哪？','请拿起瓶子。'):
        add(text,{'i':'unknown'})
    for text in ('立刻停下搜索！','停止现在的任务。','马上停车，不再搜索。'):
        add(text,{'i':'stop'})
    for text in ('我确认当前这个任务。','当前任务确认无误。'):
        add(text,{'i':'confirm'})
    # Deduplicate exact training strings; preserve contradictory-label detection.
    unique={}
    for row in rows:
        if row['text'] in unique:
            assert unique[row['text']]['expected']==row['expected'],row
        else:unique[row['text']]=row
    rows=list(unique.values());random.Random(20261024).shuffle(rows)
    cases=[]
    def case(name,text,obj,primary='background',fallback='background',home=True):
        cases.append({'id':name,'text':text,'expected':obj,'scenes':{'primary':primary,'fallback':fallback},
                      'simulated_home_ack':home,'label_origin':'synthetic_development_not_human_field_label'})
    for target in NAMES:
        case('single_'+target,f'请帮我寻找{NAMES[target]}。',{'i':'search','t':target},target)
    compound='先找胶带，找不到再找螺丝刀，找到后返回起点。'
    task={'i':'search','t':'tape','f':'screwdriver','r':True}
    case('branch_primary',compound,task,'tape','screwdriver')
    case('branch_fallback',compound,task,'background','screwdriver')
    case('return_home','找到胶带以后返回起点。',{'i':'search','t':'tape','r':True},'tape')
    case('correction','找胶带，不对，改成找螺丝刀。',{'i':'search','t':'screwdriver'},'screwdriver')
    case('stop','立即停止。',{'i':'stop'})
    case('negation','不要寻找胶带。',{'i':'unknown'})
    case('multi','同时找胶带和螺丝刀。',{'i':'unknown'})
    case('unsupported','帮我拿起胶带。',{'i':'unknown'})
    case('unknown_object','寻找桌子。',{'i':'unknown'})
    case('confirm','确认。',{'i':'confirm'})
    # Additional controls test hotword bias on similar-sounding general language.
    case('homophone_control','请把事情交代清楚。',{'i':'unknown'})
    case('question','你知道胶带有什么用途吗？',{'i':'unknown'})
    test_texts={c['text'] for c in cases}
    rows=[r for r in rows if r['text'] not in test_texts]
    system=(args.pilot/'system.txt').read_text(encoding='utf-8')
    files={'train.json':rows,'cases.json':cases}
    for name,value in files.items():
        (args.out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    (args.out/'system.txt').write_bytes(system.encode('utf-8'))
    (args.out/'hotwords.txt').write_text('胶带\n螺丝刀\n钙奶\n营养快线\n',encoding='utf-8')
    # Preserve the exact llama.cpp tokenizer/template used by the preceding pilot.
    (args.out/'shared_chat_template.jinja').write_bytes((args.pilot/'shared_chat_template.jinja').read_bytes())
    protocol={'date':'2026-10-24','seed':20261024,'train_n':len(rows),'case_n':len(cases),
              'voices':['Microsoft Huihui Desktop','Microsoft Kangkang'],
              'training':{'steps':128,'batch':4,'lr':5e-5,'rank':8,'alpha':16,'thinking':False},
              'asr':{'baseline':'greedy_search','hotwords':'modified_beam_search','paths':4,'score':1.5},
              'evaluation':'Synthetic development replay; source text used only for TTS/scoring, never fed to ASR/LLM in audio route.',
              'simulator':'Static image windows and explicit return-home acknowledgements, no robot motion.',
              'files':{name:hashlib.sha256((args.out/name).read_bytes()).hexdigest() for name in
                       ('train.json','cases.json','system.txt','hotwords.txt','shared_chat_template.jinja')}}
    (args.out/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(protocol,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
