"""Collect real Isaac RGB into a unique dataset, without teleporting the robot.

Robot walks to each station on the existing route. Camera extrinsics are varied
on a virtual rig for data generation (explicitly recorded); this is NOT a route
proof or physical camera-mount validation. All authored parameters restored.
Injected: project_root, per_gauge=40, gauge_ids=None, seed=1701, dataset_id.
"""
import sys
from pathlib import Path
import importlib
import importlib.util
import math
import json
import hashlib
from datetime import datetime,timezone
from dataclasses import asdict
import uuid
import asyncio
import time
import numpy as np
import omni.usd
from pxr import Gf,Usd,UsdGeom

root=Path(project_root).resolve() if 'project_root' in dir() else next(Path(p).resolve().parents[1] for p in sys.path if p and (Path(p)/'run_route.py').is_file())
sys.path.insert(0,str(root));sys.path.insert(0,str(root/'tools/isaac'))
from omni.ext._impl import stat_cache
stat_cache._stat_cache.clear();importlib.invalidate_caches()
import inspection,inspection.adapters
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m
    exec(compile(path.read_text(encoding='utf-8'),str(path),'exec'),m.__dict__)
    if '.' in name:
        parent,child=name.rsplit('.',1);setattr(sys.modules[parent],child,m)
    return m
for name in ('actions','geometry','site','dataset_plan'):
    load('inspection.'+name,root/'inspection'/(name+'.py'))
for name in ('known_aisle','isaac'):
    load('inspection.adapters.'+name,root/'inspection/adapters'/(name+'.py'))
from inspection.site import load_config,verify_scene
from inspection.actions import MotionConfig,stabilize,safety_error
from inspection.dataset_plan import make_plan
from inspection.adapters.isaac import KitIO,KitCamera,stage_obstacles
from inspection.adapters.known_aisle import KnownAisle
gauge_module=load('make_gauge',root/'tools/isaac/make_gauge.py')
command=load('command_control',root/'tools/isaac/command_control.py')
s=sys._go2_inspection_session
control=command.install_control(s)
if control.owner is not None:raise RuntimeError('mission_already_running')
config,site=load_config(root/'config/inspection_route.json',site_path if 'site_path' in dir() else None)
stage=omni.usd.get_context().get_stage();verify_scene(site,stage.GetRootLayer().realPath)
if s.camera is None:raise RuntimeError('camera_required')
cam_module=load('go2_camera',root/'tools/isaac/go2_camera.py')
import types
for name in ('wait_for_rgb','save_png'):
    setattr(s.camera,name,types.MethodType(getattr(cam_module.FrontCamera,name),s.camera))
ids=list(gauge_ids) if 'gauge_ids' in dir() else site['route']
gauges=[next(g for g in site['gauges'] if g['id']==name) for name in ids]
per=int(per_gauge) if 'per_gauge' in dir() else 40
base_seed=int(seed) if 'seed' in dir() else 1701
plan=make_plan(ids,per,base_seed)
rid=str(dataset_id) if 'dataset_id' in dir() else datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
if Path(rid).name!=rid or rid in ('.','..'):raise ValueError('invalid_dataset_id')
folder=root/'data/gauges'/rid;folder.mkdir(parents=True,exist_ok=False);(folder/'images').mkdir()
settings_bytes=(root/'cv/settings.json').read_bytes();reader_bytes=(root/'cv/gauge_reader.py').read_bytes()
(folder/'split_plan.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
(folder/'reader_settings.json').write_bytes(settings_bytes)
state={'schema_version':1,'dataset_id':rid,'site_id':site['site_id'],'outcome':'running','saved':0,'planned':len(plan),
       'errors':[],'restored':False,'navigation':[],
       'reader_sha256_before_collection':hashlib.sha256(reader_bytes).hexdigest(),
       'settings_sha256_before_collection':hashlib.sha256(settings_bytes).hexdigest(),
       'camera_variation':'virtual camera extrinsics; robot walks normally; no robot teleport',
       'split_rule':'disjoint seeds and distance bands; declared in split_plan.json before images'}
def status():
    (folder/'collection.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
status()
cfg=MotionConfig(**config['motion']);env=config['robot_envelope']
obs,excluded=stage_obstacles(stage,env['z_min_m'],env['z_max_m'])
nav=KnownAisle(site['navigation'],obs,env['radius_m']+env['tracking_margin_m'],excluded)
io=KitIO(s,control,cfg);io.frame_id=site['map']['frame_id'];camera=KitCamera(s,io)
io.pose_safe=lambda p:not nav.blockers([p.x,p.y],[p.x,p.y])
camera_prim=stage.GetPrimAtPath(s.camera.camera_path)
camera_attrs={str(op.GetAttr().GetPath()):op.GetAttr().Get() for op in UsdGeom.Xformable(camera_prim).GetOrderedXformOps()}
lights={str(p.GetAttribute('inputs:intensity').GetPath()):p.GetAttribute('inputs:intensity').Get()
        for p in stage.Traverse() if p.GetAttribute('inputs:intensity').IsValid() and p.GetAttribute('inputs:intensity').Get() is not None}
truth_before={g['id']:gauge_module.gauge_ground_truth(stage,g['simulation_prim_path']) for g in gauges}
vis_before={g['id']:UsdGeom.Imageable(stage.GetPrimAtPath(g['simulation_prim_path'])).GetVisibilityAttr().Get() for g in gauges}
robot_visuals={}  # Geometry is not hidden; keep the virtual lens in front of the robot.
state['camera_variation']+='; robot staged 2.2m from dial so virtual views 0.8..1.6m stay in front of the body'

def restore_camera():
    with Usd.EditContext(stage,stage.GetSessionLayer()):
        for path,value in camera_attrs.items():stage.GetAttributeAtPath(path).Set(value)

def place_camera(truth,condition):
    centre=np.asarray(truth['center_world']);normal=np.asarray(truth['face_normal_world'])
    world=centre+normal*condition['distance_m']
    # Keep physical camera height; virtual translation/yaw perturbations are explicit.
    m=UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    world[2]=m.ExtractTranslation()[2]
    yaw=math.atan2(-normal[1],-normal[0])+math.radians(condition['yaw_offset_deg'])
    world_q=Gf.Quatd(math.cos(yaw/2),Gf.Vec3d(0,0,math.sin(yaw/2)))*Gf.Quatd(.5,Gf.Vec3d(.5,-.5,-.5))
    parent=UsdGeom.Xformable(camera_prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    local=parent.GetInverse().Transform(Gf.Vec3d(*world))
    local_q=parent.ExtractRotationQuat().GetInverse()*world_q
    with Usd.EditContext(stage,stage.GetSessionLayer()):
        s.camera.rtx_camera.set_local_poses(translations=[list(local)],orientations=[[local_q.GetReal(),*local_q.GetImaginary()]])

def projection(truth,intrinsics):
    matrix=UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    inv=matrix.GetInverse();centre=np.asarray(truth['center_world']);right=np.asarray(truth['right_world']);up=np.asarray(truth['up_world'])
    pixels=[]
    for angle in np.linspace(0,2*math.pi,64,endpoint=False):
        p=centre+(right*math.cos(angle)+up*math.sin(angle))*truth['diameter_m']/2
        local=inv.Transform(Gf.Vec3d(*p));depth=-local[2]
        if depth<=0:raise RuntimeError('gauge_behind_camera')
        pixels.append([intrinsics['cx']+intrinsics['fx']*local[0]/depth,intrinsics['cy']-intrinsics['fy']*local[1]/depth])
    points=np.array(pixels);lo=np.floor(points.min(axis=0)-5).astype(int);hi=np.ceil(points.max(axis=0)+5).astype(int)
    lo=np.maximum(lo,0);hi=np.minimum(hi,[intrinsics['width'],intrinsics['height']])
    if np.any(hi-lo<16):raise RuntimeError('invalid_projected_bbox')
    return [int(lo[0]),int(lo[1]),int(hi[0]-lo[0]),int(hi[1]-lo[1])],[[float(x) for x in row] for row in matrix]

owner=object();control.acquire(owner)
try:
    if not io.healthy():raise RuntimeError('session_unhealthy')
    for g in gauges:
        restore_camera()
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            for path,value in robot_visuals.items():UsdGeom.Imageable(stage.GetPrimAtPath(path)).GetVisibilityAttr().Set(value)
        staging=truth_before[g['id']]
        centre=np.asarray(staging['center_world']);normal=np.asarray(staging['face_normal_world'])
        staging_xy=centre+2.2*normal
        collection_g={**g,'inspection_pose':{**g['inspection_pose'],'x':float(staging_xy[0]),'y':float(staging_xy[1])}}
        p=io.pose();points=nav.plan(p,[collection_g]);previous=[p.x,p.y]
        for point in points:
            blockers=nav.blockers(previous,point['target'])
            if blockers:raise RuntimeError('blocked_collection_segment:'+str(blockers))
            previous=point['target']
        for point in points:
            p=io.pose()
            if nav.blockers([p.x,p.y],point['target']):raise RuntimeError('actual_collection_segment_blocked')
            result=await nav.navigate_to(io,point,cfg)
            state['navigation'].append({'gauge_id':g['id'],'phase':point['phase'],'target':point['target'],**result});status()
            if result['outcome']!='reached':raise RuntimeError('navigation_'+str(result))
        settled=await stabilize(io,points[-1]['target'],cfg)
        state.setdefault('stabilization',[]).append({'gauge_id':g['id'],'attempt':1,**settled})
        # A stopped gait can coast outside position tolerance. One bounded
        # reposition is allowed; do not weaken thresholds or accept that frame.
        if settled['outcome']=='timeout' and io.healthy() and not control.cancel_requested:
            p=io.pose()
            if nav.blockers([p.x,p.y],points[-1]['target']):raise RuntimeError('stabilization_retry_blocked')
            retried=await nav.navigate_to(io,points[-1],cfg)
            state['navigation'].append({'gauge_id':g['id'],'phase':'stabilization_retry',**retried})
            if retried['outcome']=='reached':
                settled=await stabilize(io,points[-1]['target'],cfg)
                state['stabilization'].append({'gauge_id':g['id'],'attempt':2,**settled})
        if settled['outcome']!='reached':raise RuntimeError('stabilization_'+str(settled))
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            for path in robot_visuals:UsdGeom.Imageable(stage.GetPrimAtPath(path)).GetVisibilityAttr().Set('invisible')
        for condition in [p for p in plan if p['gauge_id']==g['id']]:
            if control.cancel_requested or not io.healthy():raise RuntimeError('collection_cancelled_or_unhealthy')
            value=g['calibration']['vmin']+condition['value_fraction']*(g['calibration']['vmax']-g['calibration']['vmin'])
            with Usd.EditContext(stage,stage.GetSessionLayer()):
                gauge_module.set_gauge_value(stage,g['simulation_prim_path'],value)
                UsdGeom.Imageable(stage.GetPrimAtPath(g['simulation_prim_path'])).GetVisibilityAttr().Set('invisible' if condition['hidden'] else vis_before[g['id']])
                for path,old in lights.items():stage.GetAttributeAtPath(path).Set(old*condition['light_scale'])
            truth=gauge_module.gauge_ground_truth(stage,g['simulation_prim_path'])
            place_camera(truth,condition)
            # Rendering has temporal history after USD/value/light/pose edits.
            # Wait in simulation time and for five sensor render completions,
            # then request a separately confirmed fresh frame. Never a fixed FPS count.
            settle_sim=io.control_time();settle_render=s.camera.render_count;deadline=time.monotonic()+10
            while io.control_time()-settle_sim<.5 or s.camera.render_count-settle_render<5:
                if time.monotonic()>deadline:raise RuntimeError('scene_render_settle_timeout')
                if control.cancel_requested or not io.healthy():raise RuntimeError('cancelled_during_render_settle')
                await io.update()
            image,capture=await camera.capture(cfg.frame_timeout_s)
            if image.std()<1:raise RuntimeError('uniform_sensor_frame')
            after_truth=gauge_module.gauge_ground_truth(stage,g['simulation_prim_path'])
            if after_truth['value']!=truth['value']:raise RuntimeError('truth_changed_during_capture')
            if safety_error(io,io.pose(),cfg):raise RuntimeError('unsafe_after_capture')
            bbox,world_matrix=projection(truth,capture['intrinsics'])
            filename=condition['sample_id']+'.png';camera.save(folder/'images'/filename,image)
            robot_sample=io.pose_sample()
            cm=UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            bm=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Robot/base')).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            camera_in_base=cm*bm.GetInverse();q=camera_in_base.ExtractRotationQuat()
            delta=np.asarray(cm.ExtractTranslation())-np.asarray(truth['center_world'])
            capture['extrinsics']={'parent_frame':'base','translation_m':list(camera_in_base.ExtractTranslation()),
                                   'quaternion_wxyz':[q.GetReal(),*q.GetImaginary()]}
            capture.update(pose={'position_m':[robot_sample['pose'][k] for k in ('x','y','z')],
                                 'quaternion_wxyz':robot_sample['quaternion_wxyz'],
                                 'yaw_rad':robot_sample['pose']['yaw'],'frame_id':io.frame_id},
                           camera_world_matrix_row_vectors=world_matrix,source='isaac',
                           light={'scale':condition['light_scale'],'count':len(lights)},
                           virtual_rig={'distance_requested_m':condition['distance_m'],'distance_actual_m':float(np.linalg.norm(delta)),
                                        'yaw_offset_deg':condition['yaw_offset_deg']},
                           robot_render_geometry_hidden=False,
                           frame_timestamp_limitation='USD pose sampled at receive; hardware exposure sync unavailable')
            entry={'schema_version':1,'sample_id':condition['sample_id'],'image_path':'images/'+filename,
                   'gauge_id':g['id'],'seed':condition['seed'],'split':condition['split'],'calibration':g['calibration'],
                   'capture':capture,'truth':{'value':truth['value'],'readable':not condition['hidden'],
                   'bbox':bbox,'geometry':truth,'scenario':'hidden' if condition['hidden'] else 'visible'}}
            with (folder/'manifest.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(entry,ensure_ascii=False)+'\n')
            state['saved']+=1;status()
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            gauge_module.set_gauge_value(stage,g['simulation_prim_path'],truth_before[g['id']]['value'])
            UsdGeom.Imageable(stage.GetPrimAtPath(g['simulation_prim_path'])).GetVisibilityAttr().Set(vis_before[g['id']])
    state['outcome']='completed'
except asyncio.CancelledError:
    state['outcome']='cancelled';state['errors'].append('task_cancelled')
except Exception as error:
    state['outcome']='failed';state['errors'].append(f'{type(error).__name__}: {error}')
finally:
    try:
        restore_camera()
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            for path,value in lights.items():stage.GetAttributeAtPath(path).Set(value)
            for path,value in robot_visuals.items():UsdGeom.Imageable(stage.GetPrimAtPath(path)).GetVisibilityAttr().Set(value)
            for g in gauges:
                gauge_module.set_gauge_value(stage,g['simulation_prim_path'],truth_before[g['id']]['value'])
                UsdGeom.Imageable(stage.GetPrimAtPath(g['simulation_prim_path'])).GetVisibilityAttr().Set(vis_before[g['id']])
        restored=(all(stage.GetAttributeAtPath(path).Get()==value for path,value in lights.items())
                  and all(stage.GetAttributeAtPath(path).Get()==value for path,value in camera_attrs.items())
                  and all(gauge_module.gauge_ground_truth(stage,g['simulation_prim_path'])['value']==truth_before[g['id']]['value'] for g in gauges)
                  and all(UsdGeom.Imageable(stage.GetPrimAtPath(g['simulation_prim_path'])).GetVisibilityAttr().Get()==vis_before[g['id']] for g in gauges)
                  and all(UsdGeom.Imageable(stage.GetPrimAtPath(path)).GetVisibilityAttr().Get()==value for path,value in robot_visuals.items()))
        state['restored']=restored
        if not restored:state['outcome']='failed';state['errors'].append('restore_verification_failed')
    finally:
        control.release(owner);io.close();state['final_command']=s.command.tolist();status()
print(json.dumps({'dataset':str(folder),'state':state},ensure_ascii=False))
