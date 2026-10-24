"""Bounded FPGA frame parser; a JPEG/WAV replay can never become a live source.

FIMG v1 is the legacy whole-frame format.  FIMG v2 is the direct-stream
format: the 24-byte header's last field is CRC32(header[0:20]), and the RGB565
payload is followed by a 4-byte CRC32(header[0:20] + payload).  Both versions
remain accepted; v2 bad-trailer frames are discarded like any other CRC error.
"""
from pathlib import Path
import struct
import zlib

HEADER=struct.Struct('<4sBBHHHIII')
TRAILER=struct.Struct('<I')
MAX_BYTES=16*1024*1024


def read_raw565(path,width,height,byteorder='little'):
    import numpy as np
    if type(width) is not int or type(height) is not int or not 1<=width<=8192 or not 1<=height<=8192:
        raise ValueError('Invalid raw image dimensions')
    size=width*height*2
    if size>MAX_BYTES or Path(path).stat().st_size!=size:raise ValueError('Incomplete or oversized RGB565 frame')
    if byteorder not in ('little','big'):raise ValueError('Invalid RGB565 byte order')
    raw=np.fromfile(path,dtype='<u2' if byteorder=='little' else '>u2').reshape(height,width)
    out=np.empty((height,width,3),dtype=np.uint8)
    out[:,:,0]=((raw>>11)&31)*255//31;out[:,:,1]=((raw>>5)&63)*255//63;out[:,:,2]=(raw&31)*255//31
    return out


def encode_frame(data,width,height,sequence):
    if (type(width) is not int or type(height) is not int or not 1<=width<=8192 or not 1<=height<=8192
            or type(sequence) is not int or not 0<=sequence<=0xffffffff):
        raise ValueError('invalid frame dimensions or uint32 sequence')
    if len(data)!=width*height*2 or len(data)>MAX_BYTES:raise ValueError('invalid frame length')
    prefix=HEADER.pack(b'FIMG',1,1,HEADER.size,width,height,sequence,len(data),0)[:-4]
    return prefix+struct.pack('<I',zlib.crc32(prefix+data)&0xffffffff)+data


def encode_frame_v2(data,width,height,sequence):
    if (type(width) is not int or type(height) is not int or not 1<=width<=8192 or not 1<=height<=8192
            or type(sequence) is not int or not 0<=sequence<=0xffffffff):
        raise ValueError('invalid frame dimensions or uint32 sequence')
    if len(data)!=width*height*2 or len(data)>MAX_BYTES:raise ValueError('invalid frame length')
    prefix=HEADER.pack(b'FIMG',2,1,HEADER.size,width,height,sequence,len(data),0)[:20]
    header_crc=zlib.crc32(prefix)&0xffffffff
    header=HEADER.pack(b'FIMG',2,1,HEADER.size,width,height,sequence,len(data),header_crc)
    trailer=zlib.crc32(prefix+data)&0xffffffff
    return header+data+TRAILER.pack(trailer)


class FrameReader:
    def __init__(self,keep_invalid=False,expected_size=None,versions=(1,2)):
        if expected_size is not None:
            if (not isinstance(expected_size,(tuple,list)) or len(expected_size)!=2
                    or any(type(v) is not int or not 1<=v<=8192 for v in expected_size)
                    or expected_size[0]*expected_size[1]*2>MAX_BYTES):
                raise ValueError('Invalid configured FIMG image size')
        if (not isinstance(versions,(tuple,list)) or not versions
                or any(type(v) is not int or v not in (1,2) for v in versions)):
            raise ValueError('Invalid configured FIMG versions')
        self.expected_size=tuple(expected_size) if expected_size is not None else None
        self.versions=tuple(versions)
        self.buffer=bytearray();self.last_sequence=-1;self.errors=0
        self.header_errors=0;self.crc_errors=0;self.sequence_errors=0
        self.keep_invalid=keep_invalid

    def feed(self,data):
        if len(data)>MAX_BYTES+HEADER.size+TRAILER.size:raise ValueError('USB chunk too large')
        self.buffer.extend(data);out=[]
        while True:
            start=self.buffer.find(b'FIMG')
            if start<0:self.buffer[:]=self.buffer[-3:];break
            if start:del self.buffer[:start]
            if len(self.buffer)<HEADER.size:break
            _,version,fmt,hlen,width,height,seq,size,crc=HEADER.unpack_from(self.buffer)
            if (version not in self.versions or fmt!=1 or hlen!=HEADER.size or
                    not 1<=width<=8192 or not 1<=height<=8192 or
                    size!=width*height*2 or size>MAX_BYTES or
                    (self.expected_size is not None and (width,height)!=self.expected_size)):
                del self.buffer[:4];self.errors+=1;self.header_errors+=1;continue
            prefix=bytes(self.buffer[:20])
            if version==2:
                if zlib.crc32(prefix)&0xffffffff!=crc:
                    del self.buffer[:4];self.errors+=1;self.header_errors+=1;continue
                total=HEADER.size+size+TRAILER.size
            else:
                total=HEADER.size+size
            if len(self.buffer)<total:break
            raw=bytes(self.buffer[HEADER.size:HEADER.size+size])
            if version==2:
                trailer=TRAILER.unpack_from(self.buffer,HEADER.size+size)[0]
                crc_ok=(zlib.crc32(prefix+raw)&0xffffffff)==trailer
            else:
                crc_ok=(zlib.crc32(prefix+raw)&0xffffffff)==crc
            del self.buffer[:total]
            delta=(seq-self.last_sequence)&0xffffffff
            sequence_ok=self.last_sequence<0 or 0<delta<0x80000000
            if not crc_ok:self.crc_errors+=1
            if not sequence_ok:self.sequence_errors+=1
            if not crc_ok or not sequence_ok:
                self.errors+=1
                if not self.keep_invalid:continue
            if sequence_ok:self.last_sequence=seq
            out.append({'data':raw,'width':width,'height':height,'sequence':seq,'format':'rgb565',
                        'byteorder':'little','fimg_version':version,'crc_ok':crc_ok,
                        'sequence_ok':sequence_ok,'source_timestamp_verified':False})
        return out
