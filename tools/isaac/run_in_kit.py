"""Run a project script that uses top-level `await` from places that can't await directly.

Kit's `--exec` and the Script Editor run code synchronously, while our scripts (start_go2.py,
build_scene.py) are written for the python_server, which allows top-level await. This wrapper
compiles the target with top-level await enabled and schedules it on Kit's event loop.

Usage:
    isaac-sim.bat --exec "D:/go2-inspection/tools/isaac/run_in_kit.py D:/go2-inspection/tools/isaac/start_go2.py"
    isaac-sim.bat --exec "...run_in_kit.py ...start_go2.py policy=robot_lab scene_path=D:/go2-inspection/scenes/factory_hall.usda"
    (extra name=value pairs are injected like isaacsim_send.py --arg; no spaces inside a value)
    (or open this file in Window > Script Editor, set TARGET below, Run)
"""

import ast
import asyncio
import sys

import carb
import omni.kit.app

DEFAULT_TARGET = "D:/go2-inspection/tools/isaac/start_go2.py"
WARMUP_FRAMES = 60  # let the GUI finish starting before the script swaps the stage


def _parse_args(items):
    """`name=value` pairs -> dict; values are Python literals (True, 0.5, 'x'), else plain strings."""
    args = {}
    for item in items:
        if "=" not in item:
            continue
        name, raw = item.split("=", 1)
        try:
            args[name.strip()] = ast.literal_eval(raw)
        except (ValueError, SyntaxError):
            args[name.strip()] = raw
    return args


async def _run(target: str, args: dict) -> None:
    app = omni.kit.app.get_app()
    for _ in range(WARMUP_FRAMES):
        await app.next_update_async()
    with open(target, encoding="utf-8") as f:
        source = f.read()
    code = compile(source, target, "exec", flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT)
    # Same injection as isaacsim_send.py --arg: scripts check `if "name" not in dir()`.
    scope = {"__name__": "__main__", "__file__": target, **args}
    carb.log_warn(f"run_in_kit: running {target} with {args}")
    try:
        result = eval(code, scope)
        if asyncio.iscoroutine(result):
            await result
        carb.log_warn(f"run_in_kit: finished {target}")
    except Exception as error:  # surface in the Console window instead of failing silently
        carb.log_error(f"run_in_kit: {target} failed: {error!r}")
        raise


_argv = sys.argv[1:]
_target = _argv.pop(0) if _argv and _argv[0].endswith(".py") else DEFAULT_TARGET
asyncio.ensure_future(_run(_target, _parse_args(_argv)))
