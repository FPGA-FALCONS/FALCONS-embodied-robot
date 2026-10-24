"""Read Linux USB sysfs descriptors without opening or configuring a USB device."""
import argparse
import json
from pathlib import Path
import sys


def read_value(path, errors):
    try:
        return path.read_text(encoding='utf-8').strip()
    except OSError as exc:
        errors.append({'path':str(path),'error':str(exc)})
        return None


def hex_id(value):
    try:
        result = int(value,16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('use a hexadecimal USB ID, e.g. 04b4') from exc
    if not 0 <= result <= 0xffff:
        raise argparse.ArgumentTypeError('USB IDs must be 0000..ffff')
    return f'{result:04x}'


def inspect_usb(root='/sys/bus/usb/devices', vid=None, pid=None):
    root=Path(root)
    if not root.is_dir():
        raise FileNotFoundError(f'Linux USB sysfs is unavailable: {root}')
    found=[]
    errors=[]
    for path in sorted(root.iterdir()):
        if not (path/'idVendor').is_file() or not (path/'idProduct').is_file():
            continue  # Interface and endpoint entries are inspected below their device.
        vendor=read_value(path/'idVendor',errors)
        product=read_value(path/'idProduct',errors)
        if vendor is None or product is None:
            continue  # Device may have been unplugged during enumeration.
        vendor=vendor.lower();product=product.lower()
        if (vid is not None and vendor!=vid) or (pid is not None and product!=pid):
            continue
        item={'path':path.name,'vid':vendor,'pid':product,'interfaces':[]}
        for key in ('manufacturer','product','speed','bcdDevice','busnum','devnum','bConfigurationValue'):
            item[key]=read_value(path/key,errors) if (path/key).exists() else None
        item['cypress_vendor_hint']=vendor=='04b4'
        # VID and product strings are hints, not proof of chip type or loaded firmware.
        for interface in sorted(path.iterdir()):
            if not (interface/'bInterfaceNumber').is_file():
                continue
            entry={'name':interface.name,'endpoints':[]}
            for key in ('bInterfaceNumber','bAlternateSetting','bInterfaceClass','bInterfaceSubClass','bInterfaceProtocol'):
                entry[key]=read_value(interface/key,errors)
            driver=interface/'driver'
            entry['driver']=driver.resolve().name if driver.is_symlink() else None
            for endpoint in sorted(interface.glob('ep_*')):
                ep={key:read_value(endpoint/key,errors) for key in ('bEndpointAddress','bmAttributes','wMaxPacketSize')}
                try:
                    address=int(ep['bEndpointAddress'],16)
                    attributes=int(ep['bmAttributes'],16)
                    ep['direction']='IN' if address & 0x80 else 'OUT'
                    ep['transfer_type']=('control','isochronous','bulk','interrupt')[attributes & 3]
                    ep['bulk_in_candidate']=ep['direction']=='IN' and ep['transfer_type']=='bulk'
                except (TypeError,ValueError):
                    ep['bulk_in_candidate']=False
                entry['endpoints'].append(ep)
            item['interfaces'].append(entry)
        found.append(item)
    return {'schema_version':1,'devices':found,'errors':errors,
            'operation':'read_sysfs_only','firmware_loaded':False,'device_configured':False,
            'stream_tested':False,'environment_changed':False,
            'note':'Enumeration is not a pixel-stream test. IDs and names do not prove FX2 firmware compatibility.'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vid',type=hex_id)
    parser.add_argument('--pid',type=hex_id)
    args=parser.parse_args(argv)
    try:
        result=inspect_usb(vid=args.vid,pid=args.pid)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result['devices'] else 2
    except OSError as exc:
        print(json.dumps({'error':str(exc),'operation':'read_sysfs_only','environment_changed':False},ensure_ascii=False))
        return 2


if __name__=='__main__':
    sys.exit(main())
