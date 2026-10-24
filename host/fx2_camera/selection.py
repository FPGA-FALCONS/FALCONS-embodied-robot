"""Verify an explicit Linux USB stream selection before claiming an interface."""
from .probe import inspect_usb


def select_device(cfg, sysfs='/sys/bus/usb/devices'):
    for key, low, high in (('vid',1,65535),('pid',1,65535),('endpoint',0x81,0x8f),('interface',0,255)):
        if type(cfg.get(key)) is not int or not low<=cfg[key]<=high:
            raise ValueError('Configure verified USB '+key)
    devices=inspect_usb(sysfs, f"{cfg['vid']:04x}", f"{cfg['pid']:04x}")['devices']
    if cfg.get('usb_path'):
        devices=[d for d in devices if d['path']==cfg['usb_path']]
    if len(devices)!=1:
        raise RuntimeError('Expected exactly one matching USB device; use verified usb_path for ambiguous IDs')
    device=devices[0]
    if device['speed']!='480':
        raise RuntimeError('Camera stream requires verified USB High Speed (480M)')
    interfaces=[i for i in device['interfaces'] if int(i['bInterfaceNumber'],16)==cfg['interface']]
    if len(interfaces)!=1 or interfaces[0]['bAlternateSetting']!='0':
        raise RuntimeError('Configured active USB interface/alternate setting is unavailable')
    interface=interfaces[0]
    if interface['driver']:
        raise RuntimeError('USB interface has a kernel driver; refusing detach')
    endpoints=[e for e in interface['endpoints'] if e['bEndpointAddress'] is not None
               and int(e['bEndpointAddress'],16)==cfg['endpoint'] and e['bulk_in_candidate']]
    if len(endpoints)!=1 or int(endpoints[0]['wMaxPacketSize'],16)!=512:
        raise RuntimeError('Configured endpoint is not a 512-byte High Speed Bulk-IN endpoint')
    try:
        device['bus_number']=int(device['busnum'])
        device['device_address']=int(device['devnum'])
    except (ValueError,TypeError) as exc:
        raise RuntimeError('USB bus/address disappeared during descriptor inspection') from exc
    return device
