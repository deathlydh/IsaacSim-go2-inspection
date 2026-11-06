"""Sample host/Kit resource use alongside a live probe; requires existing psutil."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import psutil

p = argparse.ArgumentParser()
p.add_argument('--seconds',type=float,default=25)
p.add_argument('--output',required=True)
a = p.parse_args()
processes = [p for p in psutil.process_iter(['name']) if p.info['name'].lower() == 'kit.exe']
psutil.cpu_percent(percpu=True)
for proc in processes: proc.cpu_percent()
rows=[]
started=time.monotonic()
while time.monotonic()-started < a.seconds:
    time.sleep(1)
    cpus=psutil.cpu_percent(percpu=True)
    ram=psutil.virtual_memory()
    row={'elapsed_s':time.monotonic()-started,'cpu_pct_mean':sum(cpus)/len(cpus),
         'cpu_pct_max_core':max(cpus),'ram_used_gb':ram.used/2**30,'ram_available_gb':ram.available/2**30,'kit':[]}
    for proc in processes:
        try:
            row['kit'].append({'pid':proc.pid,'cpu_pct_machine':proc.cpu_percent()/psutil.cpu_count(),
                               'rss_gb':proc.memory_info().rss/2**30})
        except psutil.NoSuchProcess: pass
    gpu=subprocess.run(['nvidia-smi','--query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,power.draw,temperature.gpu',
                        '--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=5)
    row['gpu_raw']=gpu.stdout.strip()
    rows.append(row)
path=Path(a.output)
path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(json.dumps({'samples':rows},indent=2),encoding='utf-8')
print(str(path))
