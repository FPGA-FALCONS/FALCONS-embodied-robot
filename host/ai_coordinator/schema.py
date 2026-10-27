"""Small strict contracts shared by adapters, replay and the live coordinator."""
import json
import math
import struct
import zlib

TARGETS=('tape','screwdriver','ad_calcium_milk','nutri_express')
DETECTION_CONFIDENCE_THRESHOLD = .60


def loads(data):
    def unique(pairs):
        out={}
        for key,value in pairs:
            if key in out:raise ValueError('duplicate JSON key')
            out[key]=value
        return out
    def invalid(value):raise ValueError('nonfinite JSON number')
    def finite(value):
        result=float(value)
        if not math.isfinite(result):raise ValueError('nonfinite JSON number')
        return result
    return json.loads(data,object_pairs_hook=unique,parse_constant=invalid,parse_float=finite)


def dumps(obj):return json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)


def task(obj):
    if not isinstance(obj,dict) or set(obj)-{'i','t','f','r'}:raise ValueError('invalid task fields')
    if obj.get('i') not in ('search','stop','confirm','unknown'):raise ValueError('invalid task intent')
    if obj['i']!='search':
        if set(obj)!={'i'}:raise ValueError('non-search arguments')
    else:
        if obj.get('t') not in TARGETS:raise ValueError('target outside supported classes')
        if 'f' in obj and (obj['f'] not in TARGETS or obj['f']==obj['t']):raise ValueError('invalid fallback')
        if 'r' in obj and obj['r'] is not True:raise ValueError('invalid return flag')
    return dict(obj)


def number(value):return type(value) in (int,float) and math.isfinite(value)


def detection(obj,width,height,threshold=DETECTION_CONFIDENCE_THRESHOLD):
    if not isinstance(obj,dict) or obj.get('class') not in TARGETS:return False
    if not number(obj.get('confidence')) or not threshold<=obj['confidence']<=1:return False
    b=obj.get('bbox')
    return (isinstance(b,list) and len(b)==4 and all(number(x) for x in b)
            and 0<=b[0]<b[2]<=width and 0<=b[1]<b[3]<=height)


MAGIC=b'FAM1'
MAX_PAYLOAD=4096


def packet(obj):
    """Proposed mission protocol, separate from the existing AA55 observation wire."""
    payload=dumps(obj).encode('utf-8')
    if not 1<=len(payload)<=MAX_PAYLOAD:raise ValueError('mission packet too large')
    body=struct.pack('>H',len(payload))+payload
    return MAGIC+body+struct.pack('>I',zlib.crc32(body)&0xffffffff)


class PacketReader:
    def __init__(self):self.buffer=bytearray();self.errors=0

    def feed(self,data):
        self.buffer.extend(data);out=[]
        while True:
            start=self.buffer.find(MAGIC)
            if start<0:
                self.buffer[:]=self.buffer[-3:];break
            if start:del self.buffer[:start]
            if len(self.buffer)<6:break
            size=struct.unpack('>H',self.buffer[4:6])[0]
            if not 1<=size<=MAX_PAYLOAD:
                del self.buffer[:4];self.errors+=1;continue
            total=10+size
            if len(self.buffer)<total:break
            body=bytes(self.buffer[4:6+size]);crc=struct.unpack('>I',self.buffer[6+size:total])[0]
            if zlib.crc32(body)&0xffffffff!=crc:
                del self.buffer[:1];self.errors+=1;continue
            payload=bytes(self.buffer[6:6+size]);del self.buffer[:total]
            try:
                obj=loads(payload)
                if not isinstance(obj,dict):raise ValueError('message must be object')
                out.append(obj)
            except (ValueError,UnicodeError):self.errors+=1
        return out
