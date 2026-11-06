"""Prepare an already loaded session for a user-requested route. No scene reload.

Play is started if needed; it may reset a previously stopped physics episode.
Does not start movement or change the camera/render configuration.
"""
import ast
from pathlib import Path
import sys
import time
import types
import omni.kit.app
import omni.timeline

s = getattr(sys,'_go2_inspection_session',None)
if s is None:
    raise RuntimeError('Start start_factory.bat first and wait for the robot.')
if getattr(getattr(s,'control',None),'owner',None) is not None:
    raise RuntimeError('A route is already running.')
if s.failed:
    raise RuntimeError('Policy failed; inspect the Kit error before restarting.')
# Load just the corrected callback into old live sessions, without restarting Kit
# or touching the scene. Existing registered callback dispatches via the class.
root=Path(project_root).resolve() if 'project_root' in dir() else next(Path(p).resolve().parents[1] for p in sys.path if p and (Path(p)/'run_route.py').is_file())
tree=ast.parse((root/'tools/isaac/start_go2.py').read_text(encoding='utf-8'))
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Go2Session')
method=next(n for n in cls.body if getattr(n,'name',None)=='_on_physics_step')
namespace=dict(type(s)._on_physics_step.__globals__)
exec(compile(ast.Module(body=[method],type_ignores=[]),'start_go2.py','exec'),namespace)
# Existing callback can hold the old bound method; replace its code in place.
type(s)._on_physics_step.__code__=namespace['_on_physics_step'].__code__
if getattr(s,'control',None) is not None:
    control_tree=ast.parse((root/'tools/isaac/command_control.py').read_text(encoding='utf-8'))
    control_cls=next(n for n in control_tree.body if isinstance(n,ast.ClassDef) and n.name=='CommandControl')
    cancel_method=next(n for n in control_cls.body if getattr(n,'name',None)=='cancel')
    control_ns={}
    exec(compile(ast.Module(body=[cancel_method],type_ignores=[]),'command_control.py','exec'),control_ns)
    type(s.control).cancel.__code__=control_ns['cancel'].__code__
    s.control.pressed.clear()
    s.control.stop()
s.command[:]=0.0
s._heading_target=None
tl=omni.timeline.get_timeline_interface()
if not tl.is_playing():
    tl.play()
app=omni.kit.app.get_app()
begun=time.monotonic()
while time.monotonic()-begun < 30:
    await app.next_update_async()
    if s.failed: raise RuntimeError('Policy failed during initialization.')
    if s.ready and time.monotonic()-begun >= 3:
        break
else:
    raise RuntimeError('Robot initialization timeout.')
print('Ready. Play is active; no route commanded yet.')
