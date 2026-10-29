"""Reviewed admission around the frozen MOT1/Chinese parser contract.

No serial device is opened here.  A valid ``wire_preview`` is only a protocol
mapping; it is not a serial write, device acknowledgment or physical motion.
Distance commands use a nominal 0.3 m/s timed estimate, split into MOT1
segments of at most 5 s. This is not measured distance feedback.
"""
import argparse
import json
import re
import sys
import unicodedata
from .motion_reference.voice_nav import parse_motion,cn_number
from .motion_reference.mot1 import from_intent

VERSION='zh-motion-drive300mms-distance-5'
DEFAULT_SPEED_CODE=40
# MOT1: code * 4 counts/10 ms; 61440 counts/rev and 64 mm wheel diameter.
# 229 -> 916 counts/10 ms -> 299.76 mm/s. Independent of inspection config.
DRIVE_SPEED_CODE=229
HINT=re.compile(r'前进|后退|前行|倒退|向前|向后|往前|往后|左转|右转|向左转|向右转|顺时针|逆时针|巡航|巡逻')
GUARD=re.compile(r'不要|别|不许|禁止|不用|不能|如果|假如|等到|找到|完成|然后|之后|以后|再|解释|什么意思|怎么|能不能|可以|是否|吗|[?，,;；:：“”"\n]')
DIGITS='一二两三四五六七八九'
CN_INTEGER=re.compile(rf'(?:[{DIGITS}]百(?:[{DIGITS}]十[{DIGITS}]?|[零〇][{DIGITS}])?|[{DIGITS}]?十[{DIGITS}]?|[零〇{DIGITS}])')
TOKEN=re.compile(r'[0-9零〇一二两三四五六七八九十百半点.]+')


def speed_for_intent(intent, configured_speed):
    """Every forward/backward frame targets approximately 300 mm/s."""
    return DRIVE_SPEED_CODE if intent.get('cmd') in ('forward','backward') else configured_speed


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


def motion_frames(intent, seq=0, speed=DEFAULT_SPEED_CODE):
    durations=intent.get('segments_ms')
    if durations is None:return [from_intent(intent,seq=seq,speed=speed_for_intent(intent,speed))]
    return [from_intent({**intent,'duration_s':ms/1000},seq=(seq+i)&255,
                        speed=speed_for_intent(intent,speed)) for i,ms in enumerate(durations)]


def _distance_result(intent,policy,speed_code):
    distance=_number(intent['distance_m'],'distance')
    maximum=policy.get('max_distance_m',2.)
    if not .05<=distance<=maximum:
        return dict(valid=False,reason='distance_out_of_range',execution_supported=False,
                    wire_preview=None,intent=intent)
    total_ms=round(distance/.3*1000)
    full,remainder=divmod(total_ms,5000)
    segments=[5000]*full+([remainder] if remainder else [])
    intent={**intent,'distance_mode':'timed_estimate','distance_target_m':distance,
            'duration_s':total_ms/1000,'estimated_speed_mps':.3,'segments_ms':segments}
    frames=motion_frames(intent,speed=speed_code)
    return dict(valid=True,intent=intent,wire_preview=frames[0].encode().hex(),
                wire_previews=[f.encode().hex() for f in frames],protocol='MOT1',
                wire_preview_only=True,execution_supported=True,reason='timed_distance_estimate',
                distance_estimate=True,arrival_verified=False,reference_frame=frames[0].__dict__)


def inspect_motion(text,distance=None,speed_code=DEFAULT_SPEED_CODE):
    """None for the existing C1b domain; otherwise a complete motion decision.

    ``distance`` optionally supplies max_distance_m; speed is nominal 0.3 m/s.
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
        return _distance_result(intent,distance or {},speed_code)
    if 'duration_s' in intent:
        duration=float(intent['duration_s'])
        if not .1<=duration<=5.0 or round(duration*1000)>5000:
            return {'valid':False,'reason':'duration_out_of_range','intent':intent}
    if 'angle_deg' in intent:
        angle=abs(float(intent['angle_deg']))
        if not 5<=angle<=360:return {'valid':False,'reason':'angle_out_of_range','intent':intent}
    frame=from_intent(intent,seq=0,speed=speed_for_intent(intent,speed_code))
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
