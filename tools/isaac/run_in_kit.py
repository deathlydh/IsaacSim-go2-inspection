"""Run a project script that uses top-level `await` from places that can't await directly.

Kit's `--exec` and the Script Editor run code synchronously, while our scripts (start_go2.py,
build_scene.py) are written for the python_server, which allows top-level await. This wrapper
compiles the target with top-level await enabled and schedules it on Kit's event loop.

Usage:
    isaac-sim.bat --exec "D:/go2-inspection/tools/isaac/run_in_kit.py D:/go2-inspection/tools/isaac/start_go2.py"
    (or open this file in Window > Script Editor, set TARGET below, Run)
"""

import ast
import asyncio
import sys

import carb
import omni.kit.app

DEFAULT_TARGET = "D:/go2-inspection/tools/isaac/start_go2.py"
WARMUP_FRAMES = 60  # let the GUI finish starting before the script swaps the stage


async def _run(target: str) -> None:
    app = omni.kit.app.get_app()
    for _ in range(WARMUP_FRAMES):
        await app.next_update_async()
    with open(target, encoding="utf-8") as f:
        source = f.read()
    code = compile(source, target, "exec", flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT)
    scope = {"__name__": "__main__", "__file__": target}
    try:
        result = eval(code, scope)
        if asyncio.iscoroutine(result):
            await result
        carb.log_warn(f"run_in_kit: finished {target}")
    except Exception as error:  # surface in the Console window instead of failing silently
        carb.log_error(f"run_in_kit: {target} failed: {error!r}")
        raise


_target = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith(".py") else DEFAULT_TARGET
asyncio.ensure_future(_run(_target))
