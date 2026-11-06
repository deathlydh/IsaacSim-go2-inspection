"""Create a truth-labelled contact sheet for independent dataset QA, not CV.

python tools/dataset_contact_sheet.py data/gauges/<id> --split dev
"""
import argparse,json
from pathlib import Path
import uuid
from PIL import Image,ImageDraw


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path)
    p.add_argument('--split',choices=['dev','test','all'],default='dev');p.add_argument('--limit',type=int,default=25)
    a=p.parse_args();rows=[json.loads(line) for line in (a.dataset/'manifest.jsonl').read_text(encoding='utf-8').splitlines()]
    rows=[r for r in rows if a.split=='all' or r['split']==a.split][:a.limit]
    width=1000;height=((len(rows)+4)//5)*240
    out=Image.new('RGB',(width,height),'#eee');d=ImageDraw.Draw(out)
    for n,r in enumerate(rows):
        with Image.open(a.dataset/r['image_path']) as im:
            x,y,w,h=r['truth']['bbox'];crop=im.crop((x,y,x+w,y+h)).resize((180,180))
        x=n%5*200;y=n//5*240;out.paste(crop,(x+10,y+25))
        d.text((x+5,y+4),r['sample_id'],fill='black')
        d.text((x+5,y+210),f"{r['truth']['value']:.3f} {r['calibration']['unit']} {r['truth']['scenario']}",fill='black')
    target=a.dataset/('qa_'+a.split+'_'+uuid.uuid4().hex[:8]+'.png');out.save(target);print(target)
