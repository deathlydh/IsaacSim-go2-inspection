"""Read-only live frame/physics/camera probe; never starts or stops the timeline.

Injected: seconds=20, label='baseline', output_path (JSON). App updates are not
GPU frame times. Physics pre/post scope includes callbacks; not solver-only.
"""
import json
import os
from pathlib import Path
import statistics
import sys
import time
import carb
import omni.kit.app
import omni.timeline
import omni.physx
import omni.usd
from pxr import PhysxSchema
from isaacsim.core.simulation_manager import SimulationManager

seconds = float(seconds) if 'seconds' in dir() else 20.0
label = str(label) if 'label' in dir() else 'baseline'
output_path = str(output_path) if 'output_path' in dir() else 'D:/go2-inspection/reports/performance/live.json'
app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
s = getattr(sys, '_go2_inspection_session', None)
settings = carb.settings.get_settings()
stage = omni.usd.get_context().get_stage()
physics = []
for prim in stage.Traverse():
    if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        api = PhysxSchema.PhysxSceneAPI(prim)
        physics.append({'path': str(prim.GetPath()), 'gpu_dynamics': api.GetEnableGPUDynamicsAttr().Get(),
                        'broadphase': api.GetBroadphaseTypeAttr().Get()})
viewports = []
try:
    from omni.kit.viewport.window import get_viewport_window_instances
    for w in get_viewport_window_instances():
        viewports.append({'name': w.name, 'resolution': list(w.viewport_api.resolution),
                          'camera': str(w.viewport_api.camera_path), 'visible': bool(w.visible)})
except Exception as error:
    viewports.append({'error': str(error)})
keys = ['/app/runLoops/main/rateLimitEnabled','/app/runLoops/main/rateLimitFrequency',
        '/rtx/rendermode','/rtx/post/dlss/execMode','/rtx/post/aa/op']
steps, step_ms, start_step = [], [], []
def pre(dt):
    start_step[:] = [time.perf_counter()]
def post(dt):
    steps.append(float(dt))
    if start_step:
        step_ms.append((time.perf_counter()-start_step.pop())*1000)
px = omni.physx.get_physx_interface()
sub = px.subscribe_physics_step_events(post)
pre_sub = px.subscribe_physics_on_step_events(pre, pre_step=True, order=-10000) if hasattr(px,'subscribe_physics_on_step_events') else None
frames = []
cam = getattr(s,'camera',None)
count0 = cam.render_count if cam else 0
playing0 = tl.is_playing()
started = last = time.perf_counter()
try:
    while time.perf_counter()-started < seconds:
        await app.next_update_async()
        now = time.perf_counter()
        frames.append((now-last)*1000)
        last = now
finally:
    sub = pre_sub = None
elapsed = time.perf_counter()-started
result = {'label':label,'pid':os.getpid(),'wall_s':elapsed,'frames':len(frames),
          'app_fps':len(frames)/elapsed,'frame_ms_median':statistics.median(frames),
          'frame_ms_p90':sorted(frames)[min(len(frames)-1,int(len(frames)*.9))],
          'physics_steps':len(steps),'physics_sim_s':sum(steps),'real_time_factor':sum(steps)/elapsed,
          'physics_scope_ms_mean':statistics.mean(step_ms) if step_ms else None,
          'physics_scope_ms_per_frame':sum(step_ms)/len(frames),
          'physics_timing_available':bool(step_ms),
          'camera_render_hz':((cam.render_count-count0)/elapsed) if cam else None,
          'playing_start':playing0,'playing_end':tl.is_playing(),
          'physics':physics,'physics_dt':SimulationManager.get_physics_dt(),
          'tensor_device':str(SimulationManager.get_physics_sim_device()),
          'viewports':viewports,'settings':{k:settings.get(k) for k in keys},
          'limitations':['App update FPS, not independently measured GPU frame time.',
                          'Pre/post physics scope includes callbacks, not just solver.']}
Path(output_path).parent.mkdir(parents=True,exist_ok=True)
Path(output_path).write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
