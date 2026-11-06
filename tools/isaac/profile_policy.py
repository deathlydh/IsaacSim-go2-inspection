"""Read-only timing of the live runner; restores the original step method."""
import json
from pathlib import Path
import statistics
import sys
import time
import carb
import omni.kit.app
import torch
from omni.kit.viewport.window import get_viewport_window_instances

s=sys._go2_inspection_session
original=s.runner.step
durations=[]
def timed(*args,**kwargs):
    start=time.perf_counter()
    try: return original(*args,**kwargs)
    finally: durations.append((time.perf_counter()-start)*1000)
s.runner.step=timed
settings=carb.settings.get_settings()
start=time.perf_counter()
frames=0
try:
    while time.perf_counter()-start < 15:
        await omni.kit.app.get_app().next_update_async()
        frames+=1
finally:
    s.runner.step=original
keys=['/plugins/carb.tasking.plugin/threadCount','/plugins/omni.tbb.globalcontrol/maxThreadCount',
      '/persistent/physics/numThreads','/physics/numThreads']
result={'wall_s':time.perf_counter()-start,'frames':frames,'runner_calls':len(durations),
        'runner_ms_per_frame':sum(durations)/max(1,frames),
        'runner_ms_mean':statistics.mean(durations) if durations else None,
        'runner_ms_p90':sorted(durations)[int(.9*len(durations))] if durations else None,
        'torch_intra_threads':torch.get_num_threads(),'torch_interop_threads':torch.get_num_interop_threads(),
        'engine':str(s.runner.engine),'thread_settings':{k:settings.get(k) for k in keys},
        'viewports':[{'name':w.name,'resolution':list(w.viewport_api.resolution),'visible':bool(w.visible)}
                      for w in get_viewport_window_instances()],
        'note':'Runner scope includes observation, inference and actuator targets; not pure inference.'}
path=Path('D:/go2-inspection/reports/performance_review/policy.json')
path.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
