"""Reviewed admission around Claude's frozen MOT1/Chinese parser contract.

No serial device is opened here. A wire preview is not an execution receipt.
Distance requests are preserved, never silently reduced to a timed drive.
"""
import argparse
import json
import re
import sys
import unicodedata
from .motion_reference.voice_nav import parse_motion,cn_number
from .motion_reference.mot1 import from_intent

VERSION='zh-motion-preview-1'
HINT=re.compile(r'前进|后退|前行|倒退|向前|向后|往前|往后|左转|右转|向左转|向右转|顺时针|逆时针|巡航|巡逻')
GUARD=re.compile(r'不要|别|不许|禁止|不用|不能|如果|假如|等到|找到|完成|然后|之后|以后|再|解释|什么意思|怎么|能不能|可以|是否|吗|[?，,;；:：“”"\n]')
DIGITS='一二两三四五六七八九'
CN_INTEGER=re.compile(rf'(?:[{DIGITS}]百(?:[{DIGITS}]十[{DIGITS}]?|[零〇][{DIGITS}])?|[{DIGITS}]?十[{DIGITS}]?|[零〇{DIGITS}])')
TOKEN=re.compile(r'[0-9零〇一二两三四五六七八九十百半点.]+')


def normalize_number(token):
    if re.fullmatch(r'(?:0|[1-9][0-9]*)(?:\.[0-9]{1,3})?',token):return token
    if token=='半':return '0.5'
    parts=token.split('点')
    if len(parts)>2 or not CN_INTEGER.fullmatch(parts[0]):raise ValueError('Ambiguous or unsupported number')
    integer=int(cn_number(parts[0]))
    if len(parts)==1:return str(integer)
    if not re.fullmatch(r'[零〇一二三四五六七八九]{1,3}',parts[1]):raise ValueError('Unsupported decimal')
    table={c:str(i) for i,c in enumerate('零一二三四五六七八九')};table['〇']='0'
    return str(integer)+'.'+''.join(table[c] for c in parts[1])


def inspect_motion(text):
    """None for the existing C1b domain; otherwise a complete motion decision."""
    value=unicodedata.normalize('NFKC',text).strip()
    compact=re.sub(r'\s+','',value).rstrip('。.!')
    # Negation, quoting and clauses must be checked before removing politeness.
    relevant=bool(HINT.search(compact))
    if GUARD.search(value):
        return {'valid':False,'reason':'motion_not_a_single_immediate_command'} if relevant else None
    compact=re.sub(r'^(?:请帮我|麻烦|帮我|请)?(?:机器人|小车)?','',compact)
    compact=re.sub(r'(?:谢谢|吧)$','',compact)
    compact={'向前走':'前进','向后走':'后退','继续巡航':'开始巡航'}.get(compact,compact)
    compact=re.sub(r'^(前进|后退|往前走|往后退)(?:一点点|一点|一下)$',r'\1',compact)
    try:
        compact=TOKEN.sub(lambda m:normalize_number(m.group()),compact)
        if re.search(r'时针',compact) and (re.search(r'[0-9]',compact) or compact.endswith(('度','圈'))):
            if not re.fullmatch(r'(?:原地)?(?:顺时针|逆时针)转?[0-9]+(?:\.[0-9]+)?(?:度|圈)',compact):
                raise ValueError('Explicit angle/circle amount and unit required')
        parsed=parse_motion(compact)
    except ValueError as exc:
        return {'valid':False,'reason':'invalid_motion_number_or_limit','detail':str(exc)} if relevant else None
    if parsed is None:
        return {'valid':False,'reason':'motion_not_a_single_supported_command'} if relevant else None
    intent={k:v for k,v in parsed.items() if k!='raw_text'}
    if 'angle_deg' in intent and intent['angle_deg']!=int(intent['angle_deg']):
        return {'valid':False,'reason':'mot1_requires_integer_degrees'}
    if 'distance_m' in intent:
        # The current MOT1 drive value is milliseconds, not encoder distance.
        intent.pop('duration_s',None)
        return {'valid':True,'intent':intent,'wire_preview':None,'protocol':'MOT1',
                'execution_supported':False,'reason':'encoder_distance_not_in_current_mot1'}
    frame=from_intent(intent,seq=0,speed=40)
    if frame is None:raise ValueError('Reviewed intent has no MOT1 mapping')
    return {'valid':True,'intent':intent,'wire_preview':frame.encode().hex(),'protocol':'MOT1',
            'wire_preview_only':True,'execution_supported':False,'reason':'motion_firmware_not_commissioned',
            'reference_frame':frame.__dict__}


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description='Offline Chinese motion preview; no serial/model access')
    parser.add_argument('text',nargs='+');args=parser.parse_args()
    for text in args.text:
        print(json.dumps({'raw_text':text,'policy_version':VERSION,'motion':inspect_motion(text),
                          'sent':False,'executed':False,'physical_task_completed':False},ensure_ascii=False))


if __name__=='__main__':main()
