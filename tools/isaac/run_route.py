"""Isaac entry for the common site-configured mission. Existing injected arguments supported."""
import sys
from pathlib import Path
import importlib.util
import json
s=getattr(sys,'_go2_inspection_session',None)
if s is None: raise RuntimeError('no_live_session')
if 'project_root' in dir():
    root=Path(project_root).resolve()
else:
    candidates=[Path(p).resolve().parents[1] for p in sys.path if p and (Path(p)/'run_route.py').is_file()]
    if not candidates: raise RuntimeError('inject project_root or use run_inspection_route.bat')
    root=candidates[0]
if str(root) not in sys.path: sys.path.insert(0,str(root))
import importlib
importlib.invalidate_caches()
try:
    from omni.ext._impl import stat_cache
    stat_cache._stat_cache.clear()
except ImportError:
    pass
# Compile current source for a persistent Kit without cache/lifecycle changes.
def load_source(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec); sys.modules[name]=module
    exec(compile(path.read_text(encoding='utf-8'),str(path),'exec'),module.__dict__)
    if '.' in name:
        parent,child=name.rsplit('.',1)
        setattr(sys.modules[parent],child,module)
    return module
import inspection,inspection.adapters
for name in ('actions','geometry','site'):
    load_source('inspection.'+name,root/'inspection'/(name+'.py'))
for name in ('known_aisle','isaac'):
    load_source('inspection.adapters.'+name,root/'inspection/adapters'/(name+'.py'))
mission=load_source('inspection.mission',root/'inspection/mission.py')
from inspection.site import load_config,verify_scene
from inspection.adapters.isaac import KitIO,KitCamera,stage_obstacles
from inspection.adapters.known_aisle import KnownAisle
from inspection.actions import MotionConfig
import omni.usd
cfg_path=Path(config_path) if 'config_path' in dir() else root/'config/inspection_route.json'
config,site=load_config(cfg_path,site_path if 'site_path' in dir() else None)
stage=omni.usd.get_context().get_stage()
verify_scene(site,stage.GetRootLayer().realPath)
command=load_source('command_control',root/'tools/isaac/command_control.py')
control=command.install_control(s)
if control.owner is not None: raise RuntimeError('mission_already_running')
control.cancel_requested=False
if s.policy!='robot_lab': raise RuntimeError('expected_robot_lab')
# Temporary backwards camera compatibility; no render product create/close.
cam_module=load_source('go2_camera',root/'tools/isaac/go2_camera.py')
if s.camera is not None:
    import types
    for name in ('wait_for_rgb','save_png'):
        setattr(s.camera,name,types.MethodType(getattr(cam_module.FrontCamera,name),s.camera))
env=config['robot_envelope']
obstacles,excluded=stage_obstacles(stage,env['z_min_m'],env['z_max_m'])
nav=KnownAisle(site['navigation'],obstacles,env['radius_m']+env['tracking_margin_m'],excluded)
io=KitIO(s,control,MotionConfig(**config['motion'])); io.frame_id=site['map']['frame_id']
kwargs={key:locals()[key] for key in ()}
for key in ('limit','gauge','dry_run','timeout_s','cancel_after_s','run_id'):
    if key in locals(): kwargs[key]=locals()[key]
if 'limit' not in kwargs and 'gauge' not in kwargs:
    kwargs['limit']=1  # legacy direct IPC entry; CLI explicitly passes the selected route length
try:
    record,folder=await mission.run_route(io,nav,KitCamera(s,io),site,config,**kwargs)
    print(json.dumps({'report':str(folder/'route.json'),'outcome':record['outcome'],'error':record['error']},ensure_ascii=False))
finally:
    io.close()
