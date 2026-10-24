"""FIM2 line records: bounded reassembly without a full-frame buffer in FPGA.

Only START + consecutive checked rows + END publishes an RGB565 frame.
This validates transport integrity, not source capture time or physical safety.
"""
import struct
import time
import zlib
from .frames import MAX_BYTES

HEADER=struct.Struct('<4sBBHIIHHI')
START,DATA,END,ABORT=1,2,3,4
MAX_ROW_BYTES=8192*2


def dimensions(width,height):
    if (type(width) is not int or type(height) is not int or not 1<=width<=8192
            or not 1<=height<=8192 or width*height*2>MAX_BYTES):
        raise ValueError('Invalid FIM2 dimensions')


def encode_record(kind,frame_id,index,width,height,payload=b''):
    dimensions(width,height)
    if (type(kind) is not int or kind not in (START,DATA,END,ABORT)
            or type(frame_id) is not int or not 0<=frame_id<=0xffffffff
            or type(index) is not int or not 0<=index<=height):
        raise ValueError('Invalid FIM2 record identity')
    if ((kind==START and index!=0) or (kind==DATA and (index>=height or len(payload)!=width*2))
            or (kind==END and index!=height) or (kind!=DATA and payload)):
        raise ValueError('Invalid FIM2 record payload/index')
    header=HEADER.pack(b'FIM2',2,kind,0,frame_id,index,width,height,len(payload))
    return header+payload+struct.pack('<I',zlib.crc32(header+payload)&0xffffffff)


def encode_stream_frame(data,width,height,sequence):
    dimensions(width,height)
    if len(data)!=width*height*2:raise ValueError('Invalid complete FIM2 frame size')
    records=[encode_record(START,sequence,0,width,height)]
    for row in range(height):
        records.append(encode_record(DATA,sequence,row,width,height,data[row*width*2:(row+1)*width*2]))
    records.append(encode_record(END,sequence,height,width,height))
    return b''.join(records)


class StreamingFrameReader:
    def __init__(self,expected_size=None,max_frame_age_s=2.0):
        if expected_size is not None:dimensions(*expected_size)
        if not 0<max_frame_age_s<=60:raise ValueError('Invalid frame assembly deadline')
        self.expected_size=tuple(expected_size) if expected_size is not None else None
        self.max_frame_age_s=max_frame_age_s
        self.buffer=bytearray();self.last_sequence=-1;self.errors=0
        self.aborted_frames=0;self.records=0;self.completed_frames=0;self.crc_errors=0
        self.active=None

    def discard(self):
        if self.active is not None:self.aborted_frames+=1
        self.active=None

    def _record(self,kind,seq,index,width,height,payload,now):
        if kind==START:
            self.discard()
            delta=(seq-self.last_sequence)&0xffffffff
            if self.last_sequence>=0 and not 0<delta<0x80000000:
                self.errors+=1;return None
            self.active={'sequence':seq,'width':width,'height':height,'row':0,
                         'data':bytearray(width*height*2),'started_at_s':now}
            return None
        a=self.active
        if a is None:return None
        if (seq,width,height)!=(a['sequence'],a['width'],a['height']):
            self.errors+=1;self.discard();return None
        if kind==ABORT:
            self.discard();return None
        if kind==DATA:
            if index!=a['row']:
                self.errors+=1;self.discard();return None
            start=index*width*2;a['data'][start:start+len(payload)]=payload;a['row']+=1
            return None
        if kind==END:
            if a['row']!=height:
                self.errors+=1;self.discard();return None
            self.last_sequence=seq;self.completed_frames+=1;self.active=None
            return {'sequence':seq,'width':width,'height':height,'data':bytes(a['data']),
                    'format':'rgb565','byteorder':'little','wire_protocol':'fim2',
                    'source_timestamp_verified':False}
        return None

    def feed(self,data,now_s=None):
        if len(data)>MAX_BYTES+HEADER.size:raise ValueError('USB chunk too large')
        now=time.monotonic() if now_s is None else now_s
        if self.active is not None and now-self.active['started_at_s']>self.max_frame_age_s:
            self.errors+=1;self.discard()
        self.buffer.extend(data);out=[]
        while True:
            start=self.buffer.find(b'FIM2')
            if start<0:self.buffer[:]=self.buffer[-3:];break
            if start:del self.buffer[:start]
            if len(self.buffer)<HEADER.size:break
            _,version,kind,flags,seq,index,width,height,size=HEADER.unpack_from(self.buffer)
            valid=(version==2 and kind in (START,DATA,END,ABORT) and flags==0
                   and 1<=width<=8192 and 1<=height<=8192 and width*height*2<=MAX_BYTES
                   and (self.expected_size is None or (width,height)==self.expected_size)
                   and ((kind==DATA and size==width*2 and index<height)
                        or (kind==START and size==0 and index==0)
                        or (kind==END and size==0 and index==height)
                        or (kind==ABORT and size==0 and index<=height)))
            if not valid:
                self.errors+=1;self.discard();del self.buffer[:4];continue
            total=HEADER.size+size+4
            if len(self.buffer)<total:break
            crc=struct.unpack_from('<I',self.buffer,HEADER.size+size)[0]
            if zlib.crc32(memoryview(self.buffer)[:HEADER.size+size])&0xffffffff!=crc:
                self.errors+=1;self.crc_errors+=1;self.discard()
                del self.buffer[:1];continue  # Preserve a subsequent valid START after a truncated record.
            payload=bytes(self.buffer[HEADER.size:HEADER.size+size]);del self.buffer[:total]
            self.records+=1
            result=self._record(kind,seq,index,width,height,payload,now)
            if result is not None:out.append(result)
        return out
