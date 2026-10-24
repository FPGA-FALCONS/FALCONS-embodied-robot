"""Reviewed admission around the frozen MOT1/Chinese parser contract.

No serial device is opened here.  A valid ``wire_preview`` is only a protocol
mapping; it is not a serial write, device acknowledgment or physical motion.
Distance commands are preserved as distance.  Stage 1 converts them to a
timed estimate only when an operator has supplied a **calibrated** speed; it
never silently clips a distance or splits it into several drives.
"""
import argparse
import json
import re
import sys
import unicodedata
from .motion_reference.voice_nav import parse_motion,cn_number
from .motion_reference.mot1 import from_intent

VERSION='zh-motion-timed-estimate-2'
DEFAULT_SPEED_CODE=40
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


def _number(value,name):
    if type(value) not in (int,float) or not value==value or value in (float('inf'),float('-inf')):
        raise ValueError(name+' must be a finite number')
    return float(value)


def _distance_policy(cfg):
    cfg=cfg or {}
    speed=cfg.get('calibrated_speed_mps')
    return {
        'calibrated':cfg.get('calibrated') is True,
        'calibrated_speed_mps':speed,
        'min_speed_mps':cfg.get('min_speed_mps',.05),
        'max_speed_mps':cfg.get('max_speed_mps',.5),
        'max_distance_m':cfg.get('max_distance_m',2.0),
        'max_duration_s':cfg.get('max_duration_s',5.0),
    }


def _distance_result(intent,policy,speed_code):
    distance=_number(intent['distance_m'],'distance')
    intent={**intent,'distance_mode':'timed_estimate','distance_target_m':distance}
    intent.pop('duration_s',None)
    if distance<.05 or distance>policy['max_distance_m']:
        return {'valid':False,'reason':'distance_out_of_range',
                'detail':f"distance {distance} outside [0.05, {policy['max_distance_m']}]",
                'intent':intent,'execution_supported':False,'wire_preview':None}
    speed=policy['calibrated_speed_mps']
    if not policy['calibrated'] or speed is None:
        return {'valid':True,'intent':intent,'wire_preview':None,'protocol':'MOT1',
                'execution_supported':False,'reason':'distance_speed_uncalibrated',
                'distance_estimate':True,'arrival_verified':False}
    try:speed=_number(speed,'calibrated_speed_mps')
    except ValueError as exc:
        return {'valid':False,'reason':'distance_speed_invalid','detail':str(exc),
                'intent':intent,'execution_supported':False,'wire_preview':None}
    low,high=_number(policy['min_speed_mps'],'min_speed_mps'),_number(policy['max_speed_mps'],'max_speed_mps')
    if not 0<low<=speed<=high or not low<high:
        return {'valid':False,'reason':'distance_speed_out_of_range',
                'detail':f"speed {speed} outside ({low}, {high}]",'intent':intent,
                'execution_supported':False,'wire_preview':None}
    duration=distance/speed
    max_duration=float(policy['max_duration_s'])
    if duration>max_duration or round(duration*1000)>5000:
        return {'valid':False,'reason':'distance_exceeds_max_duration',
                'detail':f'{distance} m at {speed} m/s needs {duration:.3f} s, limit {max_duration} s; no truncation or split',
                'intent':intent,'execution_supported':False,'wire_preview':None,
                'distance_estimate':True,'arrival_verified':False}
    if round(duration*1000)<50:
        return {'valid':False,'reason':'distance_duration_too_short','intent':intent,
                'execution_supported':False,'wire_preview':None}
    intent.update(duration_s=round(duration,3),estimated_speed_mps=speed)
    frame=from_intent(intent,seq=0,speed=speed_code)
    if frame is None:raise ValueError('Reviewed distance intent has no MOT1 mapping')
    return {'valid':True,'intent':intent,'wire_preview':frame.encode().hex(),'protocol':'MOT1',
            'wire_preview_only':True,'execution_supported':True,
            'reason':'timed_distance_estimate','distance_estimate':True,
            'arrival_verified':False,'reference_frame':frame.__dict__}


def inspect_motion(text,distance=None,speed_code=DEFAULT_SPEED_CODE):
    """None for the existing C1b domain; otherwise a complete motion decision.

    ``distance`` is the operator-provided stage-1 policy.  Without a
    calibrated speed, a distance request remains valid as an intent but has no
    wire frame and reports ``distance_speed_uncalibrated``.
    """
    value=unicodedata.normalize('NFKC',text).strip()
    compact=re.sub(r'\s+','',value).rstrip('。.!')
    if type(speed_code) is not int or not 1<=speed_code<=40:raise ValueError('speed_code must be 1..40')
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
        return _distance_result(intent,_distance_policy(distance),speed_code)
    if 'duration_s' in intent:
        duration=float(intent['duration_s'])
        if not .1<=duration<=5.0 or round(duration*1000)>5000:
            return {'valid':False,'reason':'duration_out_of_range','intent':intent}
    if 'angle_deg' in intent:
        angle=abs(float(intent['angle_deg']))
        if not 5<=angle<=360:return {'valid':False,'reason':'angle_out_of_range','intent':intent}
    frame=from_intent(intent,seq=0,speed=speed_code)
    if frame is None:raise ValueError('Reviewed intent has no MOT1 mapping')
    return {'valid':True,'intent':intent,'wire_preview':frame.encode().hex(),'protocol':'MOT1',
            'wire_preview_only':True,'execution_supported':True,'reason':'mot1_frame_ready',
            'distance_estimate':False,'arrival_verified':False,'reference_frame':frame.__dict__}


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description='Offline Chinese motion preview; no serial/model access')
    parser.add_argument('text',nargs='+');args=parser.parse_args()
    for text in args.text:
        print(json.dumps({'raw_text':text,'policy_version':VERSION,'motion':inspect_motion(text),
                          'sent':False,'executed':False,'physical_task_completed':False},ensure_ascii=False))


if __name__=='__main__':main()
