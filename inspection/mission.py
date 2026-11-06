"""Common route runner. No Kit imports, scene paths, fixed target count or truth."""
import asyncio
from dataclasses import asdict
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import time
import uuid
from . import actions


async def run_route(io,navigation,camera,site,config,*,limit=None,gauge=None,dry_run=False,
                    timeout_s=None,cancel_after_s=None,run_id=None):
    motion=dict(config['motion'])
    if timeout_s is not None: motion['timeout_s']=float(timeout_s)
    cfg=actions.MotionConfig(**motion)
    if limit is not None and (not isinstance(limit,int) or not 1<=limit<=len(site['route'])): raise ValueError('invalid_limit_for_selected_route')
    names=[gauge] if gauge else site['route'][:limit]
    if not names: raise ValueError('empty_route')
    gauges=[next(g for g in site['gauges'] if g['id']==name) for name in names]
    start_pose=io.pose()
    if actions.safety_error(io,start_pose,cfg): raise RuntimeError('session_not_ready_or_unsafe_pose')
    points=navigation.plan(start_pose,gauges)
    clearance=navigation
    obstacles,excluded=navigation.obstacles,navigation.excluded
    previous=[start_pose.x,start_pose.y]
    for point in points:
        point['start']=list(previous[:2])
        point['blockers']=navigation.blockers(previous,point['target'])
        previous=point['target']
    rid = run_id if run_id else datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid.uuid4().hex[:8]
    if not isinstance(rid,str) or not rid or Path(rid).name != rid or rid in (".",".."):
        raise ValueError("invalid_run_id")
    folder = Path(config["reports_root"])/rid
    folder.mkdir(parents=True,exist_ok=False)
    record = {"schema_version":1,"run_id":rid,"started_utc":datetime.now(timezone.utc).isoformat(),
              "mode":"known_static_route", "site_id":site["site_id"],"map":site["map"],"site_sha256":site["sha256"],"initial_pose":asdict(start_pose),
              "config":config,"effective_motion":asdict(cfg),"gauge_order":names,
              "plan":points,"obstacles":obstacles,"excluded_by_height":excluded,
              "events":[],"results":[],"outcome":"preflight","error":None}
    begun = time.monotonic()
    control_start = io.control_time()


    def write_report():
        record["duration_s"] = time.monotonic()-begun
        (folder/"route.json").write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
        rows = []
        for result in record["results"]:
            picture = f'<img width="640" src="{html.escape(result["image_path"],quote=True)}">' if result.get("image_path") else ""
            rows.append(f'<section><h2>{html.escape(str(result["gauge_id"]))}</h2><pre>{html.escape(json.dumps(result,ensure_ascii=False,indent=2))}</pre>{picture}</section>')
        page = '<!doctype html><meta charset="utf-8"><title>Go2 route</title><style>body{font:16px system-ui;max-width:1050px;margin:30px auto}pre{white-space:pre-wrap}section{border-top:1px solid #bbb;padding:15px}img{max-width:100%}</style>'
        page += f'<h1>Inspection — {html.escape(rid)}</h1><p>Site: {html.escape(site["site_id"])}; navigation: {html.escape(site["navigation"]["adapter"])}; localization: {html.escape(site["map"].get("localization","adapter_provided"))}.</p><p>Outcome: {html.escape(record["outcome"])}; error: {html.escape(str(record["error"]))}</p>'
        page += '<a href="route.json">Full log and clearance audit</a>'+''.join(rows)
        (folder/"route.html").write_text(page,encoding="utf-8")


    def log(kind, **values):
        record["events"].append({"elapsed_s":time.monotonic()-begun,"control_time_s":io.control_time(),"kind":kind,**values})


    dry = dry_run
    blocked = any(p["blockers"] for p in points)
    if dry or blocked:
        record["outcome"] = "failed" if blocked else "preflight_passed"
        record["error"] = "unverified_or_blocked_segment" if blocked else None
        write_report()
    else:
        owner = object()
        io.acquire(owner)
        cancel_task = None
        try:
            record["outcome"] = "running"
            write_report()
            if cancel_after_s is not None:
                async def cancel_later(delay):
                    await asyncio.sleep(delay)
                    io.cancel()
                cancel_task = asyncio.ensure_future(cancel_later(float(cancel_after_s)))
            io.pose_safe = lambda p: not clearance.blockers([p.x,p.y],[p.x,p.y])
            last_log = [0.0]
            def trace(p):
                if time.monotonic()-last_log[0] >= 0.5:
                    log("pose",pose=asdict(p),command=io.command_state())
                    last_log[0] = time.monotonic()
                    if len(record["events"]) % 10 == 0:
                        write_report()
            for point in points:
                current = io.pose()
                blockers = clearance.blockers([current.x,current.y],point["target"])
                if blockers:
                    raise RuntimeError("actual_segment_blocked: "+str(blockers))
                log(point["phase"],gauge_id=point["gauge_id"],target=point["target"],pose=asdict(current))
                result = await navigation.navigate_to(io,point,cfg,event=trace)
                log("approach_result",**result)
                if result["outcome"] != "reached":
                    record["outcome"],record["error"] = result["outcome"],result["error"]
                    record["results"].append({"gauge_id":point["gauge_id"],"phase":point["phase"],**result})
                    break
                if point["phase"] == "approach":
                    log("stabilizing",gauge_id=point["gauge_id"])
                    settled = await actions.stabilize(io,point["target"],cfg)
                    item = {"gauge_id":point["gauge_id"],"approach":result,"stabilization":settled,"outcome":"pending_capture" if settled["outcome"] == "reached" else settled["outcome"]}
                    record["results"].append(item)
                    if settled["outcome"] != "reached":
                        record["outcome"],record["error"] = settled["outcome"],settled["error"]
                        break
                    image, capture_meta = await actions.capture_at_pose(io,camera,point['target'],cfg)
                    before, after = capture_meta['render_count_before'],capture_meta['render_count_after']
                    pose_after = io.pose()
                    error = actions.safety_error(io,pose_after,cfg)
                    dist,yaw = actions.errors(pose_after,point["target"])
                    if error == "cancelled":
                        raise asyncio.CancelledError()
                    if error or dist > cfg.position_tolerance_m or yaw > math.radians(cfg.yaw_tolerance_deg):
                        raise RuntimeError(error or "pose_changed_during_capture")
                    filename = f'{len(record["results"]):02d}_{point["gauge_id"]}.png'
                    camera.save(folder/filename,image)
                    item.update(outcome="inspected",image_path=filename,capture_pose=asdict(pose_after),position_error_m=dist,
                                yaw_error_deg=math.degrees(yaw),render_count_before=before,render_count_after=after,
                                intrinsics=capture_meta["intrinsics"],captured_utc=capture_meta["timestamp_utc"],capture=capture_meta)
                    log("captured",gauge_id=point["gauge_id"],image_path=filename)
                write_report()
            else:
                record["outcome"] = "completed"
        except asyncio.CancelledError:
            record["outcome"],record["error"] = "cancelled","task_cancelled"
            if record["results"] and record["results"][-1]["outcome"] == "pending_capture":
                record["results"][-1].update(outcome="cancelled",error="task_cancelled")
        except Exception as error:
            record["outcome"],record["error"] = "failed",f"{type(error).__name__}: {error}"
            if record["results"] and record["results"][-1]["outcome"] == "pending_capture":
                record["results"][-1].update(outcome="failed",error=record["error"])
        finally:
            if cancel_task is not None:
                cancel_task.cancel()
            io.release(owner)
            record["final_pose"] = asdict(io.pose()) if io.healthy() else None
            record["final_command"] = io.command_state()
            record["command_source"] = io.command_source()
            record["control_duration_s"] = io.control_time()-control_start
            write_report()
    return record,folder
