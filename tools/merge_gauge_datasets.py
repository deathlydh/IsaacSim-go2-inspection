"""Copy saved rows into a new unique dataset; never modify originals or drop rows.

Allows a restored partial collection stopped by stabilization_timeout, not a
visually rejected dataset. Preserves seeds, capture metadata and byte hashes.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
import uuid
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from inspection.dataset import validate


def merge(sources,output_root):
    rows=[];plan=[];source_states=[];seen=set();signatures=set()
    for source in sources:
        source=Path(source).resolve()
        state=json.loads((source/'collection.json').read_text(encoding='utf-8'))
        if not state['restored']:raise ValueError('unrestored_source')
        audit=source/'validation.json'
        if audit.exists() and json.loads(audit.read_text(encoding='utf-8')).get('valid_for_evaluation') is False:
            raise ValueError('visually_rejected_source')
        if state['outcome']!='completed' and not (state['errors'] and all('stabilization_' in e and 'timeout' in e for e in state['errors'])):
            raise ValueError('unsupported_partial_source')
        signatures.add((state['reader_sha256_before_collection'],state['settings_sha256_before_collection']))
        conditions={p['sample_id']:p for p in json.loads((source/'split_plan.json').read_text(encoding='utf-8'))}
        for line in (source/'manifest.jsonl').read_text(encoding='utf-8').splitlines():
            r=json.loads(line);sample=r['sample_id']
            if sample in seen:raise ValueError('duplicate_sample')
            seen.add(sample);plan.append(conditions[sample])
            original=source/r['image_path']
            r['provenance']={'dataset':str(source),'image_path':r['image_path'],'sha256':hashlib.sha256(original.read_bytes()).hexdigest()}
            rows.append((r,original))
        source_states.append({'path':str(source),'outcome':state['outcome'],'errors':state['errors'],'saved':state['saved']})
    if len(signatures)!=1:raise ValueError('algorithm_changed_between_sources')
    out=Path(output_root)/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_merged_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True,exist_ok=False);(out/'images').mkdir()
    with (out/'manifest.jsonl').open('x',encoding='utf-8') as manifest:
        for r,original in rows:
            r['image_path']='images/'+r['sample_id']+'.png'
            target=out/r['image_path'];shutil.copyfile(original,target)
            if hashlib.sha256(target.read_bytes()).hexdigest()!=r['provenance']['sha256']:raise ValueError('copy_hash_mismatch')
            manifest.write(json.dumps(r,ensure_ascii=False)+'\n')
    (out/'split_plan.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
    shutil.copyfile(Path(sources[0])/'reader_settings.json',out/'reader_settings.json')
    reader,settings=next(iter(signatures))
    state={'schema_version':1,'dataset_id':out.name,'outcome':'completed','saved':len(rows),'planned':len(rows),'restored':True,
           'errors':[],'sources':source_states,'reader_sha256_before_collection':reader,'settings_sha256_before_collection':settings,
           'note':'Every saved row copied. Failed fifth-point stabilization resumed as separate source; no CV-based filtering.'}
    (out/'collection.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
    result=validate(out)
    (out/'integrity.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if not result['valid']:raise ValueError(result)
    return out,result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('sources',nargs='+',type=Path)
    p.add_argument('--output-root',type=Path,default=Path('data/gauges'));a=p.parse_args()
    out,result=merge(a.sources,a.output_root);print(json.dumps({'dataset':str(out),'validation':result},indent=2))
