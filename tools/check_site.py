"""Offline configuration check through the same runner; no physics or obstacle survey.

python tools/check_site.py --site sites/rotated_demo/site.json --start 10 19 0
"""
import argparse,asyncio,json,math,sys,types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from inspection.actions import Pose
from inspection.site import load_config
from inspection.adapters.known_aisle import KnownAisle
from inspection.mission import run_route


async def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--site',required=True,type=Path)
    p.add_argument('--start',type=float,nargs=3,required=True,metavar=('X','Y','YAW_DEG'));a=p.parse_args()
    config,site=load_config(ROOT/'config/inspection_route.json',a.site)
    io=types.SimpleNamespace(pose=lambda:Pose(a.start[0],a.start[1],.3,math.radians(a.start[2]),0),
                             healthy=lambda:True,cancelled=lambda:False,control_time=lambda:0)
    nav=KnownAisle(site['navigation'],[],.6)
    record,folder=await run_route(io,nav,None,site,config,dry_run=True)
    result={'scope':'offline configuration only; mock pose, no obstacles, no physical traversal',
            'site_id':site['site_id'],'targets':len(record['gauge_order']),'outcome':record['outcome'],'route':str(folder/'route.json')}
    (folder/'site_check.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))


if __name__=='__main__':asyncio.run(main())
