"""python -m cv.read_image image.png --site sites/factory_hall/site.json --gauge Gauge_01"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import uuid
import cv2
from .gauge_reader import read_gauge,annotate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image',type=Path)
    parser.add_argument('--site',type=Path,default=Path('sites/factory_hall/site.json'))
    parser.add_argument('--gauge',default='Gauge_01')
    parser.add_argument('--roi',type=int,nargs=4)
    parser.add_argument('--output-root',type=Path,default=Path('reports'))
    args=parser.parse_args()
    site=json.loads(args.site.read_text(encoding='utf-8'))
    g=next(g for g in site['gauges'] if g['id']==args.gauge)
    image=cv2.imread(str(args.image))
    if image is None: raise ValueError('image_not_readable')
    rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    result=read_gauge(rgb,g['calibration'],args.gauge,args.roi)
    folder=args.output_root/('cv_read_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    cv2.imwrite(str(folder/'annotated.png'),cv2.cvtColor(annotate(rgb,result),cv2.COLOR_RGB2BGR))
    print(json.dumps({'output':str(folder),'result':result},ensure_ascii=False))


if __name__=='__main__': main()
