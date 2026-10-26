"""Resolve USB roles by identity, not enumeration number or current socket.

No device is opened for streaming. Ambiguous identities require configuration;
plugging in a second identical camera must never silently change the AI input.
"""
from pathlib import Path


def choose(selector, inventory, role):
    if not isinstance(selector, dict) or set(selector) - {'vid', 'pid', 'serial', 'path_hint'}:
        raise ValueError('Invalid USB role selector')
    for key in ('vid', 'pid'):
        if type(selector.get(key)) is not int or not 0 <= selector[key] <= 65535:
            raise ValueError('USB selector requires numeric vid and pid')
    if 'serial' in selector and (not isinstance(selector['serial'], str) or not selector['serial']):
        raise ValueError('Serial must be a non-empty identity')
    matches = [r for r in inventory if r.get('role') == role and
               all(r.get(k) == selector[k] for k in ('vid', 'pid', 'serial') if k in selector)]
    if selector.get('path_hint'):
        matches = [r for r in matches if selector['path_hint'] in r.get('aliases', [])]
    if len(matches) != 1:
        raise ValueError('USB role %s has %d matches; explicit identity/disambiguation required' % (role, len(matches)))
    return dict(matches[0])


def camera_inventory():
    from .uvc_prepare import discover
    result = discover()
    cameras = []
    for record in result['devices']:
        if not record['capture_candidate']:
            continue
        identity = record['identity']
        serial = Path(identity['usb_path']) / 'serial'
        cameras.append({**identity, 'serial': serial.read_text().strip() if serial.is_file() else '',
                        'aliases': record['aliases'], 'role': 'camera'})
    return cameras


def camera_config(cfg, inventory=None):
    if 'selector' not in cfg:
        return dict(cfg)  # Existing explicit stable-link configurations remain valid.
    selected = choose(cfg['selector'], camera_inventory() if inventory is None else inventory, 'camera')
    return {**cfg, 'device': selected['device']}


def audio_inventory(sys_root=Path('/sys'),proc_root=Path('/proc')):
    import re
    records=[];sound=sys_root/'class/sound'
    for card in sound.glob('card[0-9]*'):
        if not re.fullmatch(r'card[0-9]+',card.name):continue
        device=(card/'device').resolve()
        usb=next((p for p in [device,*device.parents] if (p/'idVendor').is_file() and (p/'idProduct').is_file()),None)
        if usb is None:continue
        identity=dict(vid=int((usb/'idVendor').read_text().strip(),16),pid=int((usb/'idProduct').read_text().strip(),16),
                      serial=(usb/'serial').read_text().strip() if (usb/'serial').is_file() else '')
        card_id=(proc_root/'asound'/card.name/'id').read_text().strip()
        if not re.fullmatch(r'[A-Za-z0-9_]+',card_id):raise ValueError('Invalid ALSA card identity')
        for node in sound.glob('pcmC'+card.name[4:]+'D*'):
            match=re.fullmatch(r'pcmC[0-9]+D([0-9]+)([cp])',node.name)
            if not match:continue
            device_name='plughw:CARD=%s,DEV=%s'%(card_id,match[1])
            records.append({**identity,'role':'microphone' if match[2]=='c' else 'speaker','device':device_name,
                'aliases':[device_name,str(usb)]})
    return records


def audio_device(selector,role,legacy=None,inventory=None):
    if selector is None:return legacy
    if role not in ('microphone','speaker'):raise ValueError('Explicit audio role required')
    return choose(selector,audio_inventory() if inventory is None else inventory,role)['device']
