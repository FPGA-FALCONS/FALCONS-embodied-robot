"""Deploy to one verified RK3588 serial; never choose an emulator implicitly."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--adb',type=Path,required=True);ap.add_argument('--serial',required=True)
    ap.add_argument('--bundle',type=Path,required=True);ap.add_argument('--activate',action='store_true');args=ap.parse_args()
    if args.serial.startswith(('emulator-','127.0.0.1:')):raise ValueError('Select the physical board serial')
    def adb(*parts,timeout=30):
        return subprocess.run([str(args.adb),'-s',args.serial,*parts],capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=timeout,check=True).stdout
    state=adb('get-state',timeout=10).strip()
    if state!='device':raise RuntimeError('Board ADB is unavailable')
    identity=adb('shell','uname -m; test -d /home/orangepi/workspace && echo FALCONS_BOARD',timeout=10)
    if 'aarch64' not in identity or 'FALCONS_BOARD' not in identity:raise ValueError('Device is not the expected Linux RK3588 deployment')
    suffix=hashlib.sha256(args.bundle.read_bytes()).hexdigest()[:16]
    remote='/tmp/falcons-stack-'+suffix+'.zip';installer='/tmp/falcons-stack-'+suffix+'.py'
    adb('push',str(args.bundle),remote,timeout=90)
    adb('push',str(Path(__file__).with_name('deploy_board.py')),installer,timeout=30)
    command=['python3',installer,'--bundle',remote,'--root','/home/orangepi/workspace/falcons_ai_coordinator_20261024_r1']
    if args.activate:command.append('--activate')
    print(adb('shell',*command,timeout=240))


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    main()
