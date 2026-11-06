"""Build an offline, local HTML evidence bundle from saved route logs.

python tools/route_report.py --route reports/<run_id>/route.json
"""
import argparse
from datetime import datetime, timezone
import html
import json
import math
import os
from pathlib import Path
import uuid


def build(route_path, output_root, attempts=()):
    route_path = Path(route_path).resolve()
    record = json.loads(route_path.read_text(encoding="utf-8"))
    folder = Path(output_root)/("route_summary_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid.uuid4().hex[:8])
    folder.mkdir(parents=True,exist_ok=False)
    def link(path):
        return html.escape(Path(os.path.relpath(Path(path).resolve(),folder.resolve())).as_posix(),quote=True)
    successes = [r for r in record["results"] if r.get("outcome") == "inspected" and r.get("image_path")]
    poses = [record["initial_pose"]]+[e["pose"] for e in record["events"] if e["kind"] == "pose"]
    if record.get("final_pose"):
        poses.append(record["final_pose"])
    distances = [r["position_error_m"] for r in successes]
    angles = [r["yaw_error_deg"] for r in successes]
    evidence = []
    for attempt in attempts:
        p = Path(attempt).resolve()
        saved = json.loads(p.read_text(encoding="utf-8"))
        evidence.append({"run_id":saved["run_id"],"outcome":saved["outcome"],"error":saved["error"],
                         "final_command":saved.get("final_command"),"source":str(p)})
    metrics = {"schema_version":1,"source":str(route_path),"outcome":record["outcome"],
               "planned_gauges":len(record["gauge_order"]),"inspected_gauges":len(successes),
               "duration_wall_s":record["duration_s"],
               "position_error_max_m":max(distances,default=None),
               "position_error_mean_m":sum(distances)/len(distances) if distances else None,
               "yaw_error_max_deg":max(angles,default=None),
               "yaw_error_mean_deg":sum(angles)/len(angles) if angles else None,
               "sampled_path_length_m":sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(poses,poses[1:])),
               "sampled_max_tilt_deg":max(p["tilt_deg"] for p in poses),
               "all_frames_after_request":bool(successes) and all(r["render_count_after"]>=r["render_count_before"]+2 for r in successes),
               "final_command":record.get("final_command"),"command_source":record.get("command_source"),
               "attempts":evidence,
               "limitations":["Known static scene, simulator pose, no SLAM/Nav2 or dynamic obstacle detection.",
                               "Path length and max tilt use sampled trace, not continuous odometry.",
                               "Camera render events, not hardware acquisition timestamps.",
                               "Gauge readings and CV accuracy are not evaluated in this report."]}
    (folder/"metrics.json").write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding="utf-8")
    all_xy = [(p["x"],p["y"]) for p in poses]+[(p["target"][0],p["target"][1]) for p in record["plan"]]
    xmin,xmax = min(p[0] for p in all_xy)-0.8,max(p[0] for p in all_xy)+0.8
    ymin,ymax = -3.2,3.2
    scale = min(900/(xmax-xmin),440/(ymax-ymin))
    width,height = (xmax-xmin)*scale+60,(ymax-ymin)*scale+60
    def xy(x,y): return 30+(x-xmin)*scale,30+(ymax-y)*scale
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width:.1f} {height:.1f}" role="img" aria-label="Recorded robot path in factory">',
           '<rect width="100%" height="100%" fill="#f8fafc"/>']
    for o in record["obstacles"]:
        a,b = o["box"]
        if b[0]<xmin or a[0]>xmax or b[1]<ymin or a[1]>ymax:
            continue
        left,right,bottom,top = max(xmin,a[0]),min(xmax,b[0]),max(ymin,a[1]),min(ymax,b[1])
        x,y = xy(left,top)
        svg.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{(right-left)*scale:.1f}" height="{(top-bottom)*scale:.1f}" fill="#cbd5e1"><title>{html.escape(o["path"])}</title></rect>')
    planned = [(record["initial_pose"]["x"],record["initial_pose"]["y"])]+[(p["target"][0],p["target"][1]) for p in record["plan"]]
    def polyline(points,colour,dash=""):
        coords = " ".join(f"{xy(x,y)[0]:.1f},{xy(x,y)[1]:.1f}" for x,y in points)
        return f'<polyline points="{coords}" fill="none" stroke="{colour}" stroke-width="2" stroke-dasharray="{dash}"/>'
    svg.append(polyline(planned,"#64748b","6 4"))
    svg.append(polyline([(p["x"],p["y"]) for p in poses],"#059669"))
    for r in successes:
        p = r["capture_pose"]
        x,y = xy(p["x"],p["y"])
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#2563eb"/><text x="{x+9:.1f}" y="{y-9:.1f}" font-size="12">{html.escape(r["gauge_id"])}</text>')
    x,y = xy(poses[0]["x"],poses[0]["y"])
    svg.append(f'<text x="{x:.1f}" y="{y+20:.1f}" font-size="12">START</text></svg>')
    (folder/"trajectory.svg").write_text("".join(svg),encoding="utf-8")
    rows, gallery = [],[]
    for r in successes:
        rows.append(f'<tr><td>{html.escape(r["gauge_id"])}</td><td>{r["position_error_m"]*100:.2f}</td><td>{r["yaw_error_deg"]:.3f}</td><td>{r["render_count_before"]} → {r["render_count_after"]}</td></tr>')
        image = link(route_path.parent/r["image_path"])
        gallery.append(f'<figure><a href="{image}"><img src="{image}" loading="lazy"></a><figcaption>{html.escape(r["gauge_id"])}, original RGB, no CV overlay</figcaption></figure>')
    failed_rows = ''.join(f'<tr><td><a href="{link(a["source"])}">{html.escape(a["run_id"])}</a></td><td>{html.escape(a["outcome"])}</td><td>{html.escape(str(a["error"]))}</td></tr>' for a in evidence)
    page = '<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Go2 — маршрут</title><style>body{font:16px system-ui;max-width:1050px;margin:30px auto;padding:0 18px;color:#182334}table{border-collapse:collapse;width:100%}th,td{text-align:left;padding:10px;border-bottom:1px solid #ddd}img{max-width:100%;height:auto}figure{margin:25px 0}code{background:#eee}a{color:#075ab5}</style>'
    page += f'<h1>Go2: обход {len(successes)} / {len(record["gauge_order"])} приборов</h1><p>{html.escape(record["started_utc"])} (UTC). Исход: {html.escape(record["outcome"])}. Настенное время: {record["duration_s"]:.1f} с.</p>'
    page += '<p>Существующая походка robot_lab; команды скорости задаёт маршрутный контроллер. Фактическая поза из симулятора. Контролируемый обход известной статической сцены, без Nav2 и SLAM. Новые препятствия не обнаруживаются.</p>'
    page += f'<p><a href="{link(route_path)}">Исходный JSON</a> · <a href="metrics.json">Метрики</a></p><h2>Траектория</h2><p>Зелёная — фактические отсчёты; пунктир — план; серое — AABB коллайдеров. Старт взят из живой сессии.</p><img src="trajectory.svg" alt="Траектория робота">'
    page += '<h2>Ошибки позы в момент съёмки</h2><p>Допуск: 15 см и 3°. Свежесть: минимум два render events после запроса.</p><table><tr><th>Прибор</th><th>Позиция, см</th><th>Курс, °</th><th>Render count</th></tr>'+''.join(rows)+'</table>'
    page += '<h2>Кадры</h2><p>Это серия кадров живого обхода, не видео. Чтение показаний и точность CV ещё не оценивались.</p>'+''.join(gallery)
    page += '<h2>Предшествующие проверки и отказы</h2><p>Включены успешные проверки и сохранённые неудачные попытки разработки; один успешный маршрут не является оценкой надёжности на разных условиях.</p><table><tr><th>Запуск</th><th>Исход</th><th>Причина</th></tr>'+failed_rows+'</table></html>'
    (folder/"evaluation.html").write_text(page,encoding="utf-8")
    return folder,metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route",required=True)
    parser.add_argument("--output-root",default="reports")
    parser.add_argument("--attempt",action="append",default=[])
    args = parser.parse_args()
    output,metrics = build(args.route,args.output_root,args.attempt)
    print(json.dumps({"report":str(output/"evaluation.html"),"metrics":metrics},ensure_ascii=False,indent=2))
