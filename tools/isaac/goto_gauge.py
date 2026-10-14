"""Walk the live Go2 to a gauge's inspection waypoint (from <scene>_meta.json) and photograph it.

A tiny go-to controller on top of the walking policy: velocity command toward the waypoint in the
robot's body frame, heading via the session's heading hold. Not a planner - the straight line
must be free (the meta waypoints sit next to the aisle). Saves the front-camera frame on arrival.

Injected args: gauge (e.g. "Gauge_01", default), meta_path, timeout_s (default 40), out_dir.
"""

import json
import math
import os
import sys

import numpy as np
import omni.kit.app

if "gauge" not in dir():
    gauge = "Gauge_01"
if "meta_path" not in dir():
    meta_path = "D:/go2-inspection/scenes/factory_hall_meta.json"
if "timeout_s" not in dir():
    timeout_s = 40.0
if "out_dir" not in dir():
    out_dir = "D:/go2-inspection/shots"

MAX_V = 0.5          # m/s, inspection walking speed
POS_GAIN = 1.0       # m/s per m of position error
# The policy ignores very small speed commands, so the last ~10 cm are not reachable precisely.
POS_TOL, YAW_TOL = 0.15, math.radians(3.0)

app = omni.kit.app.get_app()
s = sys._go2_inspection_session
art = s.runner.articulation

with open(meta_path, encoding="utf-8") as f:
    meta = json.load(f)
g = next(item for item in meta["gauges"] if gauge in (item.get("name"), item.get("prim_path", "").split("/")[-1]))
wp = g["waypoint"]
tx, ty, tyaw = float(wp["x"]), float(wp["y"]), math.radians(float(wp["yaw_deg"]))


def pose():
    pos, quat = art.get_world_poses()
    p = np.asarray(pos.numpy() if hasattr(pos, "numpy") else pos).reshape(-1)[:3]
    w, x, y, z = np.asarray(quat.numpy() if hasattr(quat, "numpy") else quat).reshape(-1)[:4]
    return p, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


arrived = False
try:
    for frame in range(int(timeout_s * 50)):
        p, yaw = pose()
        dx, dy = tx - p[0], ty - p[1]
        dist = math.hypot(dx, dy)
        # Face the walking direction while far away, the gauge when close.
        s._heading_target = math.atan2(dy, dx) if dist > 0.5 else tyaw
        yaw_err = abs((tyaw - yaw + math.pi) % (2 * math.pi) - math.pi)
        if dist < POS_TOL and yaw_err < YAW_TOL:
            arrived = True
            break
        c, sn = math.cos(yaw), math.sin(yaw)
        vx_b, vy_b = c * dx + sn * dy, -sn * dx + c * dy           # world error -> body frame
        speed = min(MAX_V, POS_GAIN * dist)
        norm = max(math.hypot(vx_b, vy_b), 1e-6)
        s.command[:] = [speed * vx_b / norm, speed * vy_b / norm, 0.0]
        await app.next_update_async()
finally:
    s.command[:] = 0.0
    s._heading_target = tyaw

for _ in range(50):  # settle 1 s so the camera is still
    await app.next_update_async()
p, yaw = pose()
print(f"{gauge} ({g.get('label')}): arrived={arrived} at ({p[0]:.2f}, {p[1]:.2f}) yaw {math.degrees(yaw):.1f} "
      f"target ({tx:.2f}, {ty:.2f}, {wp['yaw_deg']}) | truth {g.get('value')} {g.get('unit')}")
if s.camera is not None:
    img = await s.camera.wait_for_rgb(fresh=True)
    path = os.path.join(out_dir, f"camera_{gauge}.png").replace("\\", "/")
    print("camera frame:", s.camera.save_png(path), path, None if img is None else img.shape)
