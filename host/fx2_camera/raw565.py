"""Convert exactly one headerless packed RGB565 frame to a PPM image; no USB I/O."""
import argparse
from pathlib import Path

MAX_PIXELS=4_194_304


def frame_bytes(width,height):
    if type(width) is not int or type(height) is not int or width<=0 or height<=0 or width*height>MAX_PIXELS:
        raise ValueError('use positive dimensions, at most 4194304 pixels')
    return width*height*2


def rgb888(payload,width,height,byteorder):
    expected=frame_bytes(width,height)
    if byteorder not in ('little','big'):
        raise ValueError('explicit little or big RGB565 byte order is required')
    if len(payload)!=expected:
        raise ValueError(f'expected one complete RGB565 frame ({expected} bytes), got {len(payload)}')
    result=bytearray(width*height*3)
    for i in range(width*height):
        a,b=payload[2*i:2*i+2]
        value=(a | b<<8) if byteorder=='little' else (a<<8 | b)
        r=(value>>11)&31;g=(value>>5)&63;b=value&31
        result[3*i:3*i+3]=bytes(((r<<3)|(r>>2),(g<<2)|(g>>4),(b<<3)|(b>>2)))
    return bytes(result)


def convert(source,destination,width,height,byteorder):
    expected=frame_bytes(width,height)
    # Bound reads independently of the input's stat size (including special files).
    with Path(source).open('rb') as file:
        payload=file.read(expected+1)
    pixels=rgb888(payload,width,height,byteorder)
    destination=Path(destination)
    with destination.open('xb') as file:
        file.write(f'P6\n{width} {height}\n255\n'.encode('ascii'))
        file.write(pixels)
    return destination


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--width',type=int,required=True)
    parser.add_argument('--height',type=int,required=True)
    parser.add_argument('--byteorder',choices=('little','big'),required=True)
    args=parser.parse_args(argv)
    try:
        print(convert(args.input,args.output,args.width,args.height,args.byteorder))
        return 0
    except (OSError,ValueError) as exc:
        parser.exit(2,str(exc)+'\n')


if __name__=='__main__':
    raise SystemExit(main())
