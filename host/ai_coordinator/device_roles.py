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
