"""A/B/A measurement in the current scene, then inspect wall hits in the last scan."""

import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import omni.kit.app
from pxr import Usd, UsdGeom

session = sys._go2_inspection_session
navigation = session.navigation
app = omni.kit.app.get_app()
duration = float(window_s) if "window_s" in dir() else 8.0
assert navigation.control.source == "manual" and not any(session.command)
if not navigation.timeline.is_playing():
    navigation.timeline.play()
for _ in range(60):
    await app.next_update_async()
assert session.ready and not session.failed


async def window(app, nav, duration, enabled, monotonic, assert_on):
    nav.scan_enabled = enabled
    start, sim, scans = monotonic(), nav.sim_ns, nav.scans
    frames, costs = 0, []
    previous = scans
    while monotonic() - start < duration:
        await app.next_update_async()
        assert assert_on.navigation is nav and nav.timeline.is_playing() and not assert_on.failed
        assert nav.control.source == "manual" and not any(assert_on.command)
        frames += 1
        if nav.scans != previous:
            costs.append(nav.sample[4] * 1000)
            previous = nav.scans
    elapsed = monotonic() - start
    sim_dt = (nav.sim_ns - sim) / 1e9
    return dict(lidar=enabled, seconds=elapsed, fps=frames / elapsed, rtf=sim_dt / elapsed,
                scan_hz_wall=(nav.scans - scans) / elapsed, scan_hz_sim=(nav.scans - scans) / sim_dt,
                scan_query_ms=sum(costs) / len(costs) if costs else None)


try:
    metrics = [
        await window(app, navigation, duration, enabled, time.monotonic, session)
        for enabled in (False, True, False)
    ]
finally:
    navigation.scan_enabled = True

for _ in range(15):
    await app.next_update_async()
stamp, origin, ranges, hits, _, basis = navigation.sample
stage = navigation.lidar.stage
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"], useExtentsHint=False)
directions = navigation.lidar.directions @ basis.T
wall_checks = {}
for index in sorted(range(len(hits)), key=lambda i: abs(i - len(hits) / 2)):
    path, distance = hits[index], ranges[index]
    if not path or "WallA" not in path or path in wall_checks or not math.isfinite(distance):
        continue
    box = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    lo, hi = np.asarray(box.GetMin()), np.asarray(box.GetMax())
    direction = directions[index]
    axis = int(np.argmin((hi - lo)[:2]))
    if not abs(direction[axis]) > 0.1:
        continue
    plane = lo[axis] if direction[axis] > 0 else hi[axis]
    expected = (plane - origin[axis]) / direction[axis]
    wall_checks[path] = dict(
        angle_deg=math.degrees(navigation.lidar.first + index * navigation.lidar.increment),
        axis="xy"[axis], plane_m=float(plane), scan_m=distance, aabb_plane_m=float(expected),
        error_m=abs(distance - expected))
    if len(wall_checks) >= 5:
        break
report = dict(
    metrics=metrics, stamp_ns=stamp, origin_m=origin, rays=len(ranges),
    finite=sum(map(math.isfinite, ranges)), invalid=sum(map(math.isnan, ranges)),
    free=sum(map(math.isinf, ranges)), wall_checks=wall_checks,
    policy=session.policy, camera=session.camera is not None,
    layer=stage.GetRootLayer().identifier)
if "output" in dir():
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
