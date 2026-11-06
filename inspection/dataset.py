"""Dataset integrity validation; no predictions and no image filtering by quality."""
from collections import Counter
import json
import hashlib
import math
from pathlib import Path
from PIL import Image


def validate(folder):
    folder=Path(folder)
    state=json.loads((folder/'collection.json').read_text(encoding='utf-8'))
    plan=json.loads((folder/'split_plan.json').read_text(encoding='utf-8'))
    manifest=folder/'manifest.jsonl'
    rows=[json.loads(line) for line in manifest.read_text(encoding='utf-8').splitlines()] if manifest.exists() else []
    errors=[]
    if len({r['sample_id'] for r in plan}) != len(plan):errors.append('duplicate_plan_ids')
    audit=folder/'validation.json'
    if audit.exists() and json.loads(audit.read_text(encoding='utf-8')).get('valid_for_evaluation') is False:
        errors.append('visual_or_semantic_audit_rejected_dataset')
    if state['outcome']!='completed' or not state['restored']:errors.append('collection_not_completed_and_restored')
    if {r['sample_id'] for r in rows}!={r['sample_id'] for r in plan}:errors.append('manifest_plan_mismatch')
    if len({r['sample_id'] for r in rows})!=len(rows):errors.append('duplicate_ids')
    conditions={r['sample_id']:r for r in plan}
    seeds={s:{r['seed'] for r in rows if r['split']==s} for s in ('dev','test')}
    if seeds['dev']&seeds['test']:errors.append('split_seed_overlap')
    for r in rows:
        if r.get('provenance') and hashlib.sha256((folder/r['image_path']).read_bytes()).hexdigest()!=r['provenance']['sha256']:
            errors.append(r['sample_id']+':image_hash_mismatch')
        with Image.open(folder/r['image_path']) as im:
            width,height=im.size;im.verify()
        c=r['capture'];x,y,w,h=r['truth']['bbox']
        if not 0<=x<x+w<=width or not 0<=y<y+h<=height:errors.append(r['sample_id']+':invalid_bbox')
        if c['render_count_after']<c['render_count_before']+2:errors.append(r['sample_id']+':not_fresh')
        if r['schema_version']!=1 or c['source']!='isaac':errors.append(r['sample_id']+':invalid_schema_source')
        expected=conditions.get(r['sample_id'])
        if expected:
            if (r['seed']!=expected['seed'] or r['split']!=expected['split']
                    or r['gauge_id']!=expected['gauge_id']):errors.append(r['sample_id']+':condition_mismatch')
            if r['truth'].get('readable') is not (not expected['hidden']):
                errors.append(r['sample_id']+':visibility_plan_mismatch')
            calibration=r['calibration']
            if (not all(isinstance(v,(int,float)) and math.isfinite(v) for v in
                        (calibration['vmin'],calibration['vmax'],r['truth']['value'],expected['value_fraction']))
                    or calibration['vmax'] <= calibration['vmin']):
                errors.append(r['sample_id']+':invalid_numeric_label')
                continue
            value=calibration['vmin']+expected['value_fraction']*(calibration['vmax']-calibration['vmin'])
            if abs(value-r['truth']['value'])>1e-8:errors.append(r['sample_id']+':truth_plan_mismatch')
    result={'valid':not errors,'samples':len(rows),'splits':dict(Counter(r['split'] for r in rows)),
            'gauges':dict(Counter(r['gauge_id'] for r in rows)),'errors':errors,
            'note':'Structural integrity only; visual/label QA is separate.'}
    return result
