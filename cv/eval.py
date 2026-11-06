"""Pixel predictions first, truth scoring second. Local, reproducible CV report.

python -m cv.eval data/gauges/<id> --workers 4
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime,timezone
import hashlib
import html
import json
import math
import os
from pathlib import Path
import platform
import time
import uuid
import cv2
import numpy as np
from .gauge_reader import read_gauge,annotate,failure,default_settings

THRESHOLDS={'confident':.8,'normalized_error':.02,'detection_iou':.5}


def load_saved_predictions(path, expected_protocol):
    """Reject replay from another dataset/model even when sample IDs coincide."""
    path=Path(path)
    source_path=path.with_name('protocol.json')
    source_bytes=source_path.read_bytes()
    source=json.loads(source_bytes)
    for key in ('manifest_sha256','reader_sha256','settings_sha256'):
        if source.get(key)!=expected_protocol[key]:
            raise ValueError('saved_prediction_protocol_mismatch:'+key)
    raw=path.read_bytes()
    return [json.loads(line) for line in raw.decode('utf-8').splitlines()], {
        'path':str(path.resolve()),'predictions_sha256':hashlib.sha256(raw).hexdigest(),
        'source_protocol_sha256':hashlib.sha256(source_bytes).hexdigest(),
        'note':'Imported predictions; scoring rerun, not fresh inference.'}


def prediction_jobs(rows,folder,modes):
    """The predictor never receives rows/truth values. Known ROI is explicit oracle scope."""
    jobs=[]
    for row in rows:
        for mode in modes:
            jobs.append({'sample_id':row['sample_id'],'gauge_id':row['gauge_id'],'mode':mode,
                         'path':str(Path(folder)/row['image_path']),
                         'calibration':dict(row['calibration']),
                         'roi':list(row['truth']['bbox']) if mode=='known_roi' else None})
    return jobs


def predict(job,settings):
    start=time.monotonic()
    try:
        image=cv2.imread(job['path'])
        if image is None:raise ValueError('image_not_readable')
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        result=read_gauge(rgb,job['calibration'],job['gauge_id'],job['roi'],settings)
    except Exception as error:
        result=failure(job['gauge_id'],job['calibration']['unit'],'unreadable',f'{type(error).__name__}: {error}')
    return {'sample_id':job['sample_id'],'mode':job['mode'],'prediction':result,'duration_s':time.monotonic()-start}


def iou(a,b):
    if a is None or b is None:return 0.
    x=max(a[0],b[0]);y=max(a[1],b[1]);right=min(a[0]+a[2],b[0]+b[2]);bottom=min(a[1]+a[3],b[1]+b[3])
    intersection=max(0,right-x)*max(0,bottom-y)
    return intersection/max(1,a[2]*a[3]+b[2]*b[3]-intersection)


def score_predictions(rows,predictions,thresholds=None):
    thresholds=dict(THRESHOLDS if thresholds is None else thresholds)
    lookup={r['sample_id']:r for r in rows}
    details=[]
    for item in predictions:
        r=lookup[item['sample_id']];p=item['prediction']
        # Only here, after the prediction is obtained/persisted, read truth.value.
        truth=r['truth'];readable=truth.get('readable',True) is not False
        numeric=p['read_status']=='ok' and p['value'] is not None and math.isfinite(p['value'])
        error=abs(p['value']-truth['value']) if numeric and readable and truth['value'] is not None else None
        normalized=error/(r['calibration']['vmax']-r['calibration']['vmin']) if error is not None else None
        overlap=iou(p.get('bbox'),truth.get('bbox'))
        wrong=numeric and (not readable or normalized is None or normalized>thresholds['normalized_error'] or overlap<thresholds['detection_iou'])
        details.append({**item,'gauge_id':r['gauge_id'],'split':r['split'],'unit':r['calibration']['unit'],
                        'readable':readable,'truth_value':truth['value'],'numeric':numeric,
                        'absolute_error':error,'normalized_error':normalized,'iou':overlap,
                        'confident':numeric and p['confidence']>=thresholds['confident'],
                        'wrong':bool(wrong),'confident_wrong':bool(wrong and p['confidence']>=thresholds['confident'])})
    def aggregate(items):
        n=len(items);visible=[i for i in items if i['readable']];accepted=[i for i in visible if i['numeric']]
        errors=[i['absolute_error'] for i in accepted if i['absolute_error'] is not None]
        norm=[i['normalized_error'] for i in accepted if i['normalized_error'] is not None]
        confident=sum(i['confident'] for i in items);wrong=sum(i['confident_wrong'] for i in items)
        units={i['unit'] for i in items}
        return {'total':n,'readable':len(visible),'hidden':n-len(visible),'numeric_reads':sum(i['numeric'] for i in items),
                'refusals':sum(not i['numeric'] for i in items),
                'refusal_rate':sum(not i['numeric'] for i in items)/n if n else None,
                'readable_success_rate':len(accepted)/len(visible) if visible else None,
                'readable_refusals':len(visible)-len(accepted),
                'readable_refusal_rate':1-len(accepted)/len(visible) if visible else None,
                'readable_accurate_fraction':sum(i['numeric'] and not i['wrong'] for i in visible)/len(visible) if visible else None,
                'mae':sum(errors)/len(errors) if errors and len(units)==1 else None,
                'unit':next(iter(units)) if len(units)==1 else 'mixed (see per_gauge MAE)',
                'normalized_mae':sum(norm)/len(norm) if norm else None,
                'visible_detection_recall':sum(i['iou']>=thresholds['detection_iou'] for i in visible)/len(visible) if visible else None,
                'hidden_false_reads':sum(i['numeric'] and not i['readable'] for i in items),
                'confident_reads':confident,'confident_wrong':wrong,
                'confident_wrong_fraction_all':wrong/n if n else None,
                'confident_wrong_fraction_confident':wrong/confident if confident else None,
                'refusal_reasons':dict(Counter(i['prediction']['error'] for i in items if not i['numeric']))}
    metrics={'thresholds':thresholds,'modes':{}}
    for mode in sorted({i['mode'] for i in details}):
        metrics['modes'][mode]={}
        for split in ('dev','test','all'):
            subset=[i for i in details if i['mode']==mode and (split=='all' or i['split']==split)]
            stats=aggregate(subset)
            stats['per_gauge']={gid:aggregate([i for i in subset if i['gauge_id']==gid]) for gid in sorted({i['gauge_id'] for i in subset})}
            metrics['modes'][mode][split]=stats
    return metrics,details


def evaluate(dataset,output_root,workers=4,split='all',saved_predictions=None):
    from inspection.dataset import validate
    dataset=Path(dataset).resolve()
    integrity=validate(dataset)
    if not integrity['valid']:raise ValueError('dataset_validation_failed: '+str(integrity))
    all_rows=[json.loads(line) for line in (dataset/'manifest.jsonl').read_text(encoding='utf-8').splitlines()]
    rows=[r for r in all_rows if split=='all' or r['split']==split]
    modes=('known_roi','full')
    collection=json.loads((dataset/'collection.json').read_text(encoding='utf-8'))
    reader_hash=hashlib.sha256(Path(__file__).with_name('gauge_reader.py').read_bytes()).hexdigest()
    settings_bytes=(dataset/'reader_settings.json').read_bytes();settings=json.loads(settings_bytes)
    if hashlib.sha256(settings_bytes).hexdigest()!=collection['settings_sha256_before_collection']:
        raise ValueError('settings_changed_since_collection')
    if reader_hash!=collection['reader_sha256_before_collection']:raise ValueError('reader_changed_since_collection; document dev tuning and freeze a new evaluation protocol before test')
    folder=Path(output_root)/('cv_eval_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    folder.mkdir(parents=True,exist_ok=False);(folder/'annotated').mkdir()
    protocol={'schema_version':1,'dataset':str(dataset),'manifest_sha256':hashlib.sha256((dataset/'manifest.jsonl').read_bytes()).hexdigest(),
              'reader_sha256':reader_hash,'settings':settings,'settings_sha256':hashlib.sha256(settings_bytes).hexdigest(),
              'thresholds':THRESHOLDS,'modes':modes,'split':split,
              'sample_ids':{s:[r['sample_id'] for r in rows if r['split']==s] for s in ('dev','test')},
              'runtime':{'python':platform.python_version(),'opencv':cv2.__version__,'numpy':np.__version__,'opencv_threads':1,'workers':workers},
              'note':'Settings frozen before collection, no tuning on this test. All saved rows evaluated. Virtual rig data, not real photos.'}
    (folder/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2),encoding='utf-8')
    cv2.setNumThreads(1)  # Offline process only; does not change Kit or system settings.
    start=time.monotonic()
    if saved_predictions:
        predictions,source=load_saved_predictions(saved_predictions,protocol)
        protocol['prediction_source']=source
    else:
        jobs=prediction_jobs(rows,dataset,modes)
        with ThreadPoolExecutor(max_workers=workers) as pool:
            predictions=list(pool.map(lambda job:predict(job,settings),jobs))
    protocol['evaluation_code_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (folder/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2),encoding='utf-8')
    expected={(r['sample_id'],mode) for r in rows for mode in modes}
    if {(p['sample_id'],p['mode']) for p in predictions}!=expected or len(predictions)!=len(expected):
        raise ValueError('predictions_do_not_match_frozen_sample_list')
    # Persist predictions before any scoring/true-value access.
    with (folder/'predictions.jsonl').open('x',encoding='utf-8') as f:
        for item in predictions:f.write(json.dumps(item,ensure_ascii=False)+'\n')
    metrics,details=score_predictions(rows,predictions)
    metrics.update(dataset=str(dataset),dataset_integrity=integrity,prediction_wall_s=time.monotonic()-start,
                   reader_sha256=reader_hash,settings_sha256=protocol['settings_sha256'],sources=collection.get('sources',[]),
                   quality_note='MAE uses numeric reads of visible gauges; refusal rates and all-frame confident errors shown alongside. Readable labels mean visible scenario, not expert readability assessment.')
    (folder/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding='utf-8')
    row_lookup={r['sample_id']:r for r in rows}
    cards=[];examples=set()
    for item in details:
        row=row_lookup[item['sample_id']];key=item['mode'],item['gauge_id'],item['split']
        rgb=cv2.cvtColor(cv2.imread(str(dataset/row['image_path'])),cv2.COLOR_BGR2RGB)
        annotation=annotate(rgb,item['prediction'])
        name=f"{item['mode']}_{item['sample_id']}.png"
        cv2.imwrite(str(folder/'annotated'/name),cv2.cvtColor(annotation,cv2.COLOR_RGB2BGR))
        is_error=not item['numeric'] or item['wrong']
        if is_error or key not in examples:
            examples.add(key)
            source=Path(os.path.relpath(dataset/row['image_path'],folder.resolve())).as_posix()
            value=item['prediction']['value'];error=item['normalized_error']
            reason=item['prediction']['error'] or ('wrong value/detection' if item['wrong'] else 'ok')
            label=f"{item['sample_id']} · {item['split']} · {item['mode']}"
            caption=f"truth={item['truth_value']:.5g} {item['unit']}; visible={item['readable']}; prediction={value}; confidence={item['prediction']['confidence']:.3f}; normalized error={error}; IoU={item['iou']:.3f}; {reason}"
            cards.append(f'<figure class="{"error" if is_error else "ok"}"><h3>{html.escape(label)}</h3><a href="annotated/{name}"><img loading="lazy" src="annotated/{name}"></a><figcaption>{html.escape(caption)} · <a href="{html.escape(source,quote=True)}">original</a></figcaption></figure>')
    def percent(value):return '—' if value is None else f'{100*value:.2f}%'
    summary=[];per_gauge=[]
    for mode,splits in metrics['modes'].items():
        for s in ('dev','test'):
            m=splits[s]
            summary.append(f'<tr><td>{mode}</td><td>{s}</td><td>{m["total"]}</td><td>{m["numeric_reads"]}</td><td>{m["refusals"]}</td><td>{percent(m["readable_success_rate"])}</td><td>{percent(m["normalized_mae"])}</td><td>{m["confident_wrong"]}/{m["total"]}</td><td>{m["hidden_false_reads"]}/{m["hidden"]}</td></tr>')
            for gid,g in m['per_gauge'].items():
                mae='—' if g['mae'] is None else f'{g["mae"]:.5g}'
                per_gauge.append(f'<tr><td>{mode}</td><td>{s}</td><td>{gid}</td><td>{mae} {html.escape(g["unit"])}</td><td>{percent(g["normalized_mae"])}</td><td>{g["readable_refusals"]}/{g["readable"]}</td><td>{g["confident_wrong"]}/{g["total"]}</td></tr>')
    page='<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Go2 — CV evaluation</title><style>body{font:16px system-ui;max-width:1300px;margin:25px auto;padding:0 18px;color:#17263b}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border-bottom:1px solid #ccd;padding:8px;text-align:left}.gallery{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:15px}figure{margin:0;padding:12px;border:1px solid #bbb}figure.error{border-top:4px solid #c74}figure.ok{border-top:4px solid #3a8}h3{font-size:15px}img{max-width:100%;height:auto}figcaption{overflow-wrap:anywhere}a{color:#075caa}</style>'
    page+=f'<h1>Чтение приборов: {len(rows)} кадров Isaac</h1><p>Пять приборов, dev/test разделены до оценки. Это виртуальный camera rig в симуляции; перенос на реальную камеру не проверен. ID приходит из задания, визуальное распознавание ID не выполняется.</p>'
    page+='<p><a href="metrics.json">Метрики JSON</a> · <a href="predictions.jsonl">Все предсказания</a> · <a href="protocol.json">Фиксированный протокол / test IDs / настройки</a></p>'
    page+='<p>known_roi: область из разметки, только оценка чтения. full: поиск во всём RGB без разметки на входе. Показания/углы truth не передаются CV. Предсказания сохранены до расчёта ошибок. Все кадры учитываются; скрытые приборы выделены отдельно.</p>'
    page+='<h2>Фактические метрики</h2><p>MAE — по числовым чтениям видимых приборов, рядом показаны отказы. Нормированная MAE — доля диапазона. Coverage — доля числовых чтений среди visible; это не точность. Confident wrong: confidence ≥0.8 и ошибка >2% диапазона, неверная область (IoU &lt;0.5) или числовой ответ на hidden. Знаменатель — все кадры группы; второй знаменатель confident reads есть в JSON. Confidence — эвристика, не вероятность.</p><table><tr><th>Режим</th><th>Split</th><th>N</th><th>Числа</th><th>Отказы</th><th>Visible coverage</th><th>nMAE</th><th>Уверенные ошибки</th><th>Hidden false read</th></tr>'+''.join(summary)+'</table>'
    page+='<h2>По приборам</h2><table><tr><th>Режим</th><th>Split</th><th>Прибор</th><th>MAE</th><th>nMAE</th><th>Visible отказы</th><th>Уверенные ошибки</th></tr>'+''.join(per_gauge)+'</table>'
    page+='<h2>Происхождение и ограничения</h2><p>Цель 2% MAE / 95% coverage — ориентир, а не обещанный результат. Набор собран с двумя уровнями света, разными значениями, дистанцией 0.8–1.6 м и yaw до ±15°. После остановки на стабилизации 160 кадров объединены с досъёмкой 40; все сохранённые строки включены, SHA256 оригиналов записаны в manifest.provenance. Ранние пробные наборы с самоперекрытием камеры/ошибками генерации отвергнуты до оценки; исходники и validation.json сохранены. Текущий reader/settings совпадает с хешами до сбора. Реальных фотографий и аппаратных timestamps экспозиции нет.</p>'
    page+='<h2>Все ошибки/отказы и примеры успеха</h2><p>Включены также ожидаемые отказы на hidden. Все 400 размеченных изображений доступны в каталоге annotated; исходные результаты без эталона — predictions.jsonl.</p><div class="gallery">'+''.join(cards)+'</div></html>'
    (folder/'evaluation.html').write_text(page,encoding='utf-8')
    return folder,metrics


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path)
    p.add_argument('--output-root',type=Path,default=Path('reports'));p.add_argument('--workers',type=int,default=4)
    p.add_argument('--split',choices=['all','dev','test'],default='all');p.add_argument('--predictions',type=Path)
    a=p.parse_args();folder,metrics=evaluate(a.dataset,a.output_root,a.workers,a.split,a.predictions)
    print(json.dumps({'report':str(folder/'evaluation.html'),'summary':{mode:{split:{k:v for k,v in value.items() if k!='per_gauge'} for split,value in splits.items()} for mode,splits in metrics['modes'].items()}},ensure_ascii=False,indent=2))
