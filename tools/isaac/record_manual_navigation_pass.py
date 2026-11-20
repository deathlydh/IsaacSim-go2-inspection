"""Short keyboard-controlled demonstration in the open factory scene, with ownership checks."""

import sys
import time
from types import SimpleNamespace

import carb.input
import omni.kit.app

session = sys._go2_inspection_session
nav = session.navigation
assert nav.timeline.is_playing() and session.ready and not session.failed
assert nav.lidar.stage.GetRootLayer().identifier.replace("\\", "/").endswith("/factory_hall.usda")
assert nav.control.owner in (None, nav) and not any(session.command)

press = carb.input.KeyboardEventType.KEY_PRESS
release = carb.input.KeyboardEventType.KEY_RELEASE
up = SimpleNamespace(input=SimpleNamespace(name="UP"), type=press)
if nav.control.owner is nav:
    nav.control.on_key(up)
    assert not any(session.command)
    try:
        nav.control.acquire(object())
    except RuntimeError:
        pass
    else:
        raise AssertionError("Route acquired ROS-owned motion")
nav.set_mode("manual")
route = object()
nav.control.acquire(route)
try:
    nav.control.on_key(up)
    assert not any(session.command)
    try:
        nav.set_mode("ros")
    except RuntimeError:
        pass
    else:
        raise AssertionError("ROS acquired route-owned motion")
finally:
    nav.control.release(route)

app = omni.kit.app.get_app()
start = nav.sim_ns
try:
    for key, duration in (("UP", 3.5), ("N", 2.4), ("UP", 2.8)):
        event = SimpleNamespace(input=SimpleNamespace(name=key), type=press)
        nav.control.on_key(event)
        assert any(session.command)
        until = time.monotonic() + duration
        try:
            while time.monotonic() < until:
                await app.next_update_async()
                assert nav.control.source == "manual" and nav.timeline.is_playing() and not session.failed
        finally:
            event.type = release
            nav.control.on_key(event)
        assert not any(session.command)
        for _ in range(12):
            await app.next_update_async()
finally:
    nav.control.cancel()

print("Keyboard manual pass complete; route/ROS contention refused; sim seconds:",
      (nav.sim_ns-start) / 1e9, "| command:", session.command.tolist(), "| owner:", nav.control.source,
      "| ROS received:", nav.received, "| accepted:", nav.accepted, "| expirations:", nav.expirations)
