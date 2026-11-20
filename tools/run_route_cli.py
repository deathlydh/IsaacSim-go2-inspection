"""Launch/monitor the already loaded factory route. Does not launch Isaac Sim."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
CLIENT = Path(os.environ.get('GO2_ISAAC_DIR', 'D:/isaacsim')) / 'skills/isaac-sim-remote/scripts/isaacsim_send.py'

def send(script, *args):
    command = [sys.executable,'-I',str(CLIENT),'--file',str(ROOT/'tools/isaac'/script),'--arg',f'project_root={ROOT.as_posix()}',*args]
    result = subprocess.run(command,cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--limit',type=int,default=None)
    parser.add_argument('--site',type=Path)
    parser.add_argument('--config',type=Path,default=ROOT/'config/inspection_route.json')
    parser.add_argument('--cancel',action='store_true')
    args=parser.parse_args()
    if args.cancel:
        print(send('cancel_route.py','--timeout','30'))
        return 0
    rid=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_manual_'+uuid.uuid4().hex[:8]
    sys.path.insert(0,str(ROOT))
    from inspection.site import load_config
    config,site=load_config(args.config,args.site)
    if site.get('offline_only'):raise ValueError('This site is offline_only; use tools/check_site.py instead of live motion.')
    count=len(site['route']) if args.limit is None else args.limit
    if not 1<=count<=len(site['route']):raise ValueError('limit must fit the selected route')
    report=Path(config['reports_root'])/rid/'route.json'
    print('Preparing loaded Go2 (start_factory.bat must already be running).',flush=True)
    print(send('prepare_route.py','--timeout','45'),flush=True)
    print('Starting route. ESC in Isaac or cancel_route.bat cancels. Ctrl+C here also cancels.',flush=True)
    print('Report: '+str(report),flush=True)
    print(send('run_route.py','--arg',f'limit={count}','--arg',f'config_path={args.config.resolve().as_posix()}','--arg',f'site_path={site["source_path"]}','--arg',f'run_id={rid}',
               '--fire-and-forget','--execution-timeout','1800','--timeout','30'),flush=True)
    started=time.monotonic()
    previous=None
    try:
        while time.monotonic()-started < 1810:
            try:
                saved=json.loads(report.read_text(encoding='utf-8'))
            except (FileNotFoundError,json.JSONDecodeError):
                time.sleep(1)
                continue
            completed=sum(r.get('outcome')=='inspected' for r in saved['results'])
            state=(saved['outcome'],completed,saved.get('error'))
            if state != previous:
                print(f'{state[0]} | photographed {completed}/{count} | {state[2] or ""}',flush=True)
                previous=state
            if saved['outcome'] not in ('running','preflight'):
                print('Open: '+str(report.with_suffix('.html')),flush=True)
                return 0 if saved['outcome']=='completed' else 1
            time.sleep(1)
        print('No completion received; requesting cancellation.',flush=True)
        print(send('cancel_route.py','--timeout','30'))
        return 1
    except KeyboardInterrupt:
        print(send('cancel_route.py','--timeout','30'))
        return 130

if __name__=='__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(str(exc),file=sys.stderr)
        print('A transport error does not prove that motion stopped. Use cancel_route.bat.',file=sys.stderr)
        raise SystemExit(1)
