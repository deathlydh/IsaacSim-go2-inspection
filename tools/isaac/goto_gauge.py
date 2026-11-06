"""Single-gauge entry point into shared route action and safety checks.
Injected args: gauge, timeout_s, run_id. Output: reports/<unique run_id>/.
"""
import ast
from pathlib import Path

if "gauge" not in dir():
    gauge = "Gauge_01"
import sys
root = Path(project_root).resolve() if "project_root" in dir() else next(Path(p).resolve().parents[1] for p in sys.path if p and (Path(p)/"run_route.py").is_file())
path = root/"tools/isaac/run_route.py"
namespace = dict(globals())
namespace.update(gauge=gauge)
for key in ("timeout_s","run_id"):
    if key in locals():
        namespace[key] = locals()[key]
await eval(compile(path.read_text(encoding="utf-8"),str(path),"exec",flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT),namespace)
