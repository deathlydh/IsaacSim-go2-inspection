"""Measure drift at zero command: yaw rate, xy drift, base height over `seconds` of standing."""

import math
import sys

import numpy as np
import omni.kit.app
import omni.timeline

if "seconds" not in dir():
    seconds = 6.0

app = omni.kit.app.get_app()
s = sys._go2_inspection_session
art = s.runner.articulation
print("policy:", s.policy, "| command:", s.command.tolist(), "| ready:", s.ready, "| failed:", s.failed,
      "| playing:", omni.timeline.get_timeline_interface().is_playing())
print("camera:", None if s.camera is None else s.camera.camera_path)


def pose():
    pos, quat = art.get_world_poses()
    pos = np.asarray(pos.numpy() if hasattr(pos, "numpy") else pos).reshape(-1)[:3]
    w, x, y, z = np.asarray(quat.numpy() if hasattr(quat, "numpy") else quat).reshape(-1)[:4]
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return pos, yaw


saved = s.command.copy()
s.command[:] = 0.0
p0, y0 = pose()
t0_frames = 0
heights = []
frames = int(seconds * 50)
for _ in range(frames):
    await app.next_update_async()
    heights.append(pose()[0][2])
p1, y1 = pose()
dyaw = (y1 - y0 + 180) % 360 - 180
print(f"over {seconds:.0f}s at zero command: yaw {y0:.1f} -> {y1:.1f} deg ({dyaw / seconds:+.2f} deg/s), "
      f"xy drift {np.linalg.norm(p1[:2] - p0[:2]):.3f} m, base z mean {np.mean(heights):.3f} m")
s.command[:] = saved
