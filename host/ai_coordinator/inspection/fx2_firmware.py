"""Reviewed ACG720 Slave FIFO firmware, loaded into volatile 8051 RAM only.

Default CLI operation reviews the supplied image. Explicit --load-volatile
uses A0 RAM downloads; there is no EEPROM request or persistent-device write.
Only the exact vendor image is accepted. Its single IFCONFIG immediate is
changed E3->A3: synchronous 16-bit Slave FIFO with internal 30MHz IFCLK output.
"""
import ctypes as C
from dataclasses import dataclass
import hashlib
from pathlib import Path
from .usb_link import USBLink

SOURCE_SHA256='0a11e7a3de3cbcc2feafafbb7077bfadf191e4a2f23fa70b5edf87c15ee60e40'
IMAGE=Path(__file__).with_name('firmware')/'acg720_slave_for_adc.iic'
IFCONFIG_IMMEDIATE=0x08de
IFCLK_HZ=30_000_000


@dataclass(frozen=True)
class Record:
    address:int
    data:bytes


def parse_iic(image):
    if not isinstance(image,bytes) or len(image)<13 or image[:8]!=bytes.fromhex('c247053121000004'):
        raise ValueError('Exact ACG720 C2 IIC header required')
    result=[];offset=8;occupied=set()
    while offset+4<=len(image):
        count=int.from_bytes(image[offset:offset+2],'big');address=int.from_bytes(image[offset+2:offset+4],'big');offset+=4
        last=bool(count&0x8000);count &= 0x7fff
        if not count or offset+count>len(image):raise ValueError('Truncated or empty IIC record')
        data=image[offset:offset+count];offset+=count
        if last:
            if address!=0xe600 or data!=b'\0' or offset!=len(image):raise ValueError('Exact final CPUCS release required')
            return tuple(result)
        if address+count>0x4000 or any(a in occupied for a in range(address,address+count)):
            raise ValueError('Only non-overlapping internal program RAM is allowed')
        occupied.update(range(address,address+count));result.append(Record(address,data))
    raise ValueError('Missing final CPUCS release')


def reviewed(image=None):
    image=IMAGE.read_bytes() if image is None else image
    if hashlib.sha256(image).hexdigest()!=SOURCE_SHA256:raise ValueError('Vendor firmware SHA256 mismatch')
    records=parse_iic(image);memory={r.address+k:v for r in records for k,v in enumerate(r.data)}
    if bytes(memory.get(k,0) for k in range(0x08da,0x08e0))!=bytes.fromhex('90e60174e3f0'):
        raise ValueError('Expected single IFCONFIG initialization not found')
    patched=[]
    for record in records:
        data=bytearray(record.data)
        if record.address<=IFCONFIG_IMMEDIATE<record.address+len(data):data[IFCONFIG_IMMEDIATE-record.address]=0xa3
        patched.append(Record(record.address,bytes(data)))
    digest=hashlib.sha256()
    for record in patched:digest.update(record.address.to_bytes(2,'big')+len(record.data).to_bytes(2,'big')+record.data)
    review={'source_sha256':SOURCE_SHA256,'runtime_records_sha256':digest.hexdigest(),
            'patch_address':IFCONFIG_IMMEDIATE,'patch_from':0xe3,'patch_to':0xa3,'ifclk_hz':IFCLK_HZ,
            'ram_bytes':sum(len(r.data) for r in patched),'records':len(patched),'writes_eeprom':False}
    return tuple(patched),review


def download(records,write):
    # This pure boundary is tested with a fake writer: only CPUCS and validated
    # program-RAM addresses can be reached, with <=64 bytes per control request.
    if not records or any(not isinstance(r,Record) or not r.data or r.address<0 or r.address+len(r.data)>0x4000 for r in records):
        raise ValueError('Validated internal program RAM records required')
    halted=False
    def transfer(address,data):
        if write(address,data)!=len(data):raise OSError('Incomplete FX2 A0 RAM transfer')
    try:
        transfer(0xe600,b'\x01');halted=True
        for record in records:
            for offset in range(0,len(record.data),64):transfer(record.address+offset,record.data[offset:offset+64])
        transfer(0xe600,b'\0');halted=False
    finally:
        if halted:
            try:write(0xe600,b'\0')
            except OSError:pass


def load_volatile(cfg):
    records,review=reviewed()
    with USBLink(cfg) as link:
        fn=link.lib.libusb_control_transfer
        fn.argtypes=[C.c_void_p,C.c_uint8,C.c_uint8,C.c_uint16,C.c_uint16,C.POINTER(C.c_ubyte),C.c_uint16,C.c_uint]
        fn.restype=C.c_int
        def write(address,data):
            buffer=(C.c_ubyte*len(data)).from_buffer_copy(data)
            result=fn(link.handle,0x40,0xa0,address,0,buffer,len(data),1000)
            if result<0:raise OSError('FX2 A0 RAM request rejected: '+str(result))
            return result
        download(records,write)
    return {**review,'operation':'volatile_ram_download','protocol_verified':False}


def main():
    import argparse,json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--load-volatile',action='store_true')
    args=parser.parse_args()
    if args.load_volatile:
        if not args.config:parser.error('--load-volatile requires explicit device config')
        cfg=json.loads(args.config.read_text(encoding='utf-8'));cfg=cfg.get('inspection',cfg).get('highspeed',cfg)
        result=load_volatile(cfg)
    else:result={**reviewed()[1],'operation':'offline_image_review'}
    print(json.dumps(result))


if __name__=='__main__':main()
