"""Camera-independent Chinese semantic motion commands.

This is the deterministic first layer in front of Qwen:
- exact/short motion phrases are parsed here (low latency, strict limits);
- anything else returns None and should fall back to C1b for task semantics.

The output is a small intent dictionary, not PWM.  The RK3588 motion bridge
turns it into MOT1 frames for the FPGA.
"""
from __future__ import annotations

import json
import re
import sys

DEFAULT_FORWARD_S = 0.8
DEFAULT_TURN_DEG = 90.0
DEFAULT_LINEAR_SPEED_MPS = 0.20
MAX_DISTANCE_M = 2.0
MAX_ANGLE_DEG = 360.0
MAX_DURATION_S = 5.0

_CN_DIGITS = {'零': 0, '〇': 0, '一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
              '五': 5, '六': 6, '七': 7, '八': 8, '九': 9}
_CN_UNITS = {'十': 10, '百': 100}


def cn_number(text):
    """Parse common Chinese/ASCII positive numbers used in motion phrases."""
    text = text.strip()
    if not text:
        raise ValueError('empty number')
    if text in ('半', '半圈'):
        return 0.5
    try:
        return float(text)
    except ValueError:
        pass
    total = 0
    section = 0
    number = 0
    saw = False
    for char in text:
        if char in _CN_DIGITS:
            number = _CN_DIGITS[char]
            saw = True
        elif char == '十':
            if number == 0:
                number = 1
            section += number * 10
            number = 0
            saw = True
        elif char == '百':
            if number == 0:
                number = 1
            section += number * 100
            number = 0
            saw = True
        else:
            raise ValueError(f'unsupported number character: {char}')
    if not saw:
        raise ValueError('not a number')
    return float(section + number)


def _number_token(pattern):
    return rf'(\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百半]+)'


_STOP = re.compile(r'^(停|停下|停止|别动|不要动|立刻停|马上停|急停)[。.!！\s]*$')
_PATROL_STOP = re.compile(r'^(关闭|停止|结束|退出)(巡航|巡逻)[。.!！\s]*$')
_PATROL_START = re.compile(r'^(开启|开始|启动|进入)(巡航|巡逻)[。.!！\s]*$')
_SPIN = re.compile(
    rf'^(原地)?(顺时针|逆时针)(转)?{_number_token("angle")}?(圈|度)?[。.!！\s]*$')
_TURN = re.compile(
    rf'^(左转|向左转|左转弯|右转|向右转|右转弯)(?:{_number_token("angle")}度)?[。.!！\s]*$')
_LINEAR = re.compile(
    rf'^(前进|向前|往前|往前走|前行|后退|向后|往后退|倒退)'
    rf'(?:{_number_token("value")}(米|m|秒|s))?[。.!！\s]*$')


def _limit(value, low, high, name):
    if not low <= value <= high:
        raise ValueError(f'{name} out of range [{low}, {high}]: {value}')
    return value


def parse_motion(text):
    """Return a motion intent dict, or None when this is not a motion phrase."""
    if not isinstance(text, str):
        return None
    cleaned = re.sub(r'\s+', '', text.strip())
    if not cleaned:
        return None

    if _STOP.match(cleaned):
        return {'cmd': 'stop', 'raw_text': text}
    if _PATROL_STOP.match(cleaned):
        return {'cmd': 'patrol_stop', 'raw_text': text}
    if _PATROL_START.match(cleaned):
        return {'cmd': 'patrol_start', 'raw_text': text}

    match = _SPIN.match(cleaned)
    if match:
        direction = -1.0 if match.group(2) == '逆时针' else 1.0
        token = match.group(4)
        unit = match.group(5)
        if token:
            amount = cn_number(token)
            angle = amount * 360.0 if unit == '圈' else amount
        else:
            angle = 360.0
        angle = _limit(angle, 5.0, MAX_ANGLE_DEG, 'angle')
        return {'cmd': 'spin', 'angle_deg': direction * angle, 'raw_text': text}

    match = _TURN.match(cleaned)
    if match:
        phrase = match.group(1)
        direction = 'left' if '左' in phrase else 'right'
        token = match.group(2)
        angle = cn_number(token) if token else DEFAULT_TURN_DEG
        angle = _limit(angle, 5.0, MAX_ANGLE_DEG, 'angle')
        return {'cmd': 'turn', 'direction': direction, 'angle_deg': angle,
                'raw_text': text}

    match = _LINEAR.match(cleaned)
    if match:
        phrase = match.group(1)
        direction = 'backward' if any(k in phrase for k in ('后', '退')) else 'forward'
        token = match.group(2)
        unit = match.group(3)
        if token and unit in ('秒', 's'):
            duration = _limit(cn_number(token), 0.1, MAX_DURATION_S, 'duration')
            return {'cmd': direction, 'duration_s': duration, 'raw_text': text}
        if token and unit in ('米', 'm'):
            distance = _limit(cn_number(token), 0.05, MAX_DISTANCE_M, 'distance')
            duration = min(distance / DEFAULT_LINEAR_SPEED_MPS, MAX_DURATION_S)
            return {'cmd': direction, 'distance_m': distance,
                    'duration_s': round(duration, 3), 'raw_text': text}
        return {'cmd': direction, 'duration_s': DEFAULT_FORWARD_S, 'raw_text': text}

    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(json.dumps({'error': 'usage: voice_nav.py "前进" ["左转90度"]'},
                         ensure_ascii=False), file=sys.stderr)
        return 2
    result = []
    for text in argv:
        intent = parse_motion(text)
        result.append(intent if intent is not None else {'cmd': None, 'raw_text': text})
    print(json.dumps(result if len(result) > 1 else result[0],
                     ensure_ascii=False, indent=2))
    return 0 if any(item.get('cmd') for item in result) else 1


if __name__ == '__main__':
    raise SystemExit(main())
