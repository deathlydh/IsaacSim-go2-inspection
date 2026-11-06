"""Collect and monitor an already-running Isaac dataset. Never launches Kit.

python tools/collect_dataset_cli.py --per-gauge 40 --seed 3701
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import time
import uuid
from run_route_cli import send,ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--per-gauge',type=int,default=40)
    parser.add_argument('--seed',type=int,default=3701)
    parser.add_argument('--gauges',nargs='+')
    parser.add_argument('--site',type=Path,default=ROOT/'sites/factory_hall/site.json')
    parser.add_argument('--timeout',type=float,default=1800)
    args=parser.parse_args()
    if args.per_gauge<2 or args.per_gauge%2:parser.error('--per-gauge must be even and >=2')
    rid=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_dataset_'+uuid.uuid4().hex[:8]
    folder=ROOT/'data/gauges'/rid
    kwargs=['--arg',f'per_gauge={args.per_gauge}','--arg',f'seed={args.seed}','--arg',f'dataset_id={rid}',
            '--arg',f'site_path={args.site.resolve().as_posix()}']
    if args.gauges:kwargs+=['--arg',f'gauge_ids={args.gauges!r}']
    print('Dataset: '+str(folder),flush=True)
    print(send('collect_gauge_dataset.py',*kwargs,'--fire-and-forget','--execution-timeout',str(args.timeout),'--timeout','30'),flush=True)
    start=time.monotonic();last=None
    try:
        while time.monotonic()-start<args.timeout+10:
            try:state=json.loads((folder/'collection.json').read_text(encoding='utf-8'))
            except (FileNotFoundError,json.JSONDecodeError):time.sleep(1);continue
            key=state['outcome'],state['saved']
            if key!=last:
                print(f'{key[0]} | {key[1]}/{state["planned"]}',flush=True);last=key
            if state['outcome'] not in ('running',):
                print(json.dumps(state,ensure_ascii=False),flush=True)
                return 0 if state['outcome']=='completed' and state['restored'] else 1
            time.sleep(1)
        print(send('cancel_route.py','--timeout','30'));return 1
    except KeyboardInterrupt:
        print(send('cancel_route.py','--timeout','30'));return 130


if __name__=='__main__':raise SystemExit(main())
