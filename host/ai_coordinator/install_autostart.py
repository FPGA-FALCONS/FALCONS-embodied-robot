"""Prepare the reviewed boot configuration; --apply installs it on the RK board."""
import argparse
import datetime
import getpass
import hashlib
import json
import os
from pathlib import Path
import subprocess
import uuid


def quoted(value):
    value = str(value)
    if '\n' in value or '\r' in value:raise ValueError('Invalid systemd path')
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def unit_text(script, config):
    return '\n'.join((
        '[Unit]', 'Description=FALCONS local AI models and FPGA coordinator',
        'After=basic.target', '', '[Service]', 'Type=simple',
        'ExecStart=/bin/bash ' + quoted(script) + ' ' + quoted(config),
        'Restart=on-failure', 'RestartSec=5', 'TimeoutStopSec=15',
        'KillMode=control-group', 'MemoryMax=6G', 'Nice=5', 'NoNewPrivileges=true',
        '', '[Install]', 'WantedBy=default.target', ''))


def prepare(config):
    config = Path(config).resolve()
    cfg = json.loads(config.read_text(encoding='utf-8'))
    if not cfg.get('ai_module', {}).get('enabled') or cfg.get('model_loads_enabled') is not True:
        raise ValueError('Enable the reviewed AI module and model loads before boot installation')
    if not all(name in cfg.get('models', {}) for name in ('asr', 'llm', 'chat', 'tts')):
        raise ValueError('Existing ASR, semantic LLM, Qwen chat and TTS configurations are required')
    cfg['startup'] = {**cfg.get('startup', {}), 'enabled':True,
                      'required_models':['asr', 'llm', 'chat', 'tts'], 'optional_models':['vision']}
    script = Path(__file__).with_name('start_board.sh').resolve()
    return cfg, unit_text(script, config)


def atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp-' + uuid.uuid4().hex)
    temporary.write_bytes(data)
    os.replace(temporary, path)


def checked_manifest(release):
    release=Path(release).resolve()
    manifest=next((release/name for name in ('MANIFEST.json','manifest.json') if (release/name).is_file()),None)
    if manifest is None:raise ValueError('Install from a staged release with a source manifest')
    raw=manifest.read_bytes();entries=json.loads(raw)
    for name,item in entries.items():
        path=(release/name).resolve()
        if not path.is_relative_to(release) or hashlib.sha256(path.read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Release source checksum mismatch: '+name)
    return entries,hashlib.sha256(raw).hexdigest()


def install(config, cfg, unit):
    if os.name != 'posix' or os.getuid() == 0:
        raise RuntimeError('Install as the RK account that owns the existing user service')
    release=Path(__file__).resolve().parents[2]
    manifest,manifest_sha=checked_manifest(release)
    account = getpass.getuser()
    # A user service needs linger to start before any SSH or desktop login.
    linger = subprocess.check_output(['loginctl', 'show-user', account, '-p', 'Linger', '--value'], text=True).strip()
    if linger != 'yes':
        subprocess.run(['loginctl', 'enable-linger', account], check=True)
        linger = subprocess.check_output(['loginctl', 'show-user', account, '-p', 'Linger', '--value'], text=True).strip()
        if linger != 'yes':raise RuntimeError('Enable systemd user linger before installing boot service')
    config = Path(config).resolve()
    target = Path.home() / '.config/systemd/user/falcons-ai-coordinator.service'
    service = 'falcons-ai-coordinator.service'
    old_config = config.read_bytes()
    active_path=Path(cfg['root'])/'active_release.json'
    old_active=active_path.read_bytes() if active_path.exists() else None
    old_unit = target.read_bytes() if target.exists() else None
    active = subprocess.run(['systemctl', '--user', 'is-active', '--quiet', service]).returncode == 0
    enabled = subprocess.run(['systemctl', '--user', 'is-enabled', '--quiet', service]).returncode == 0
    backup = Path(cfg['root']) / 'autostart_backups' / (datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8])
    backup.mkdir(parents=True)
    (backup/'config.previous.json').write_bytes(old_config)
    if old_active is not None:(backup/'active.previous.json').write_bytes(old_active)
    if old_unit is not None:(backup/'service.previous.txt').write_bytes(old_unit)
    def control(*args):subprocess.run(['systemctl', '--user', *args], check=True)
    try:
        control('stop', service)
        atomic(config, (json.dumps(cfg, ensure_ascii=False, indent=2)+'\n').encode())
        atomic(target, unit.encode())
        provenance={'release':str(release),'manifest':manifest,'source_manifest_sha256':manifest_sha,
                    'previous_release':json.loads(old_active).get('release') if old_active else None,
                    'installed_by':'install_autostart.py'}
        atomic(active_path,(json.dumps(provenance,ensure_ascii=False,indent=2)+'\n').encode())
        control('daemon-reload')
        control('enable', '--now', service)
        control('is-active', '--quiet', service)
    except BaseException:
        subprocess.run(['systemctl', '--user', 'stop', service])
        if not enabled:subprocess.run(['systemctl', '--user', 'disable', service])
        atomic(config, old_config)
        if old_active is not None:atomic(active_path,old_active)
        else:active_path.unlink(missing_ok=True)
        if old_unit is not None:atomic(target, old_unit)
        else:target.unlink(missing_ok=True)
        control('daemon-reload')
        if enabled:control('enable', service)
        if active:control('start', service)
        raise
    return {'installed':True, 'linger':linger, 'backup':str(backup), 'service':str(target),
            'models_ready_verified':False, 'note':'Inspect /status startup readiness; enabled service alone is not model-load acceptance'}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, required=True)
    ap.add_argument('--apply', action='store_true')
    args = ap.parse_args()
    cfg, unit = prepare(args.config)
    review = Path(cfg['root']) / 'autostart_prepared'
    review.mkdir(parents=True, exist_ok=True)
    (review/'config.json').write_text(json.dumps(cfg, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    (review/'service.txt').write_text(unit, encoding='utf-8')
    result = install(args.config, cfg, unit) if args.apply else {'prepared':str(review), 'live_changed':False}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':main()
