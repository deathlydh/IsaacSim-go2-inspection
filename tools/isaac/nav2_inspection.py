"""Kit endpoint for the Nav2 demo. Use through the existing localhost Python server."""
import json
import math
from pathlib import Path
import sys
import time


root = Path("D:/go2-inspection")
if str(root) not in sys.path:
    sys.path.insert(0, str(root))
from inspection.adapters.isaac import KitCamera


session = getattr(sys, "_go2_inspection_session", None)
nav = getattr(session, "navigation", None)
if nav is None or nav.ros is None:
    raise RuntimeError("Attach the Go2 bridge first")
scene = nav.lidar.stage.GetRootLayer().realPath
if Path(scene).resolve() != (root / "scenes/factory_hall.usda").resolve():
    raise RuntimeError("Open the factory hall using start_factory.bat")

if action == "manual":
    nav.set_mode("manual")
    nav.control.cancel()  # Also clears any held keyboard command in manual mode.
elif action == "arm":
    nav.set_mode("ros")
elif action in ("obstacle_add", "obstacle_remove"):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    if nav.control.source != "manual" or any(session.command):
        raise RuntimeError("Place or remove the obstacle with the robot stopped in manual")
    stage = nav.lidar.stage
    path = "/World/Nav2StaticTestObstacle"
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        if action == "obstacle_add":
            if stage.GetPrimAtPath(path):
                raise RuntimeError("Remove the previous test obstacle first")
            if not all(math.isfinite(v) for v in obstacle_xy):
                raise ValueError("Obstacle coordinates must be finite Isaac world X/Y")
            pos = nav.sample[1] - nav.sample[5] @ nav.lidar.translation
            if math.hypot(pos[0] - obstacle_xy[0], pos[1] - obstacle_xy[1]) < 1.25:
                raise RuntimeError("Place the obstacle at least 1.25 m from the stopped robot center")
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*obstacle_xy, 0.40))
            cube.AddScaleOp().Set(Gf.Vec3f(0.45, 0.45, 0.80))
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.35, 0.05)])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        elif stage.GetSessionLayer().GetPrimAtPath(path):
            stage.RemovePrim(path)
elif action not in ("status", "capture"):
    raise ValueError(f"Unknown action: {action}")

if action == "capture":
    if (not nav.timeline.is_playing() or not session.ready or session.failed
            or nav.control.source != "manual" or any(session.command)):
        raise RuntimeError("Fresh capture requires Play and a stopped Go2 in manual")
    sample_ns = nav.sim_ns
    before, _ = session.runner.articulation.get_world_poses()
    before = before.numpy().copy() if hasattr(before, "numpy") else before.copy()
    deadline = time.monotonic() + 8.0
    import omni.kit.app
    import numpy as np
    while nav.sim_ns - sample_ns < 500_000_000:
        if time.monotonic() >= deadline or nav.control.source != "manual" or any(session.command):
            raise RuntimeError("Could not settle before capture")
        await omni.kit.app.get_app().next_update_async()

    class BridgeClock:
        def control_time(self):
            return nav.sim_ns / 1e9

    camera = KitCamera(session, BridgeClock())
    image, capture = await camera.capture(timeout_s=8.0)
    after, _ = session.runner.articulation.get_world_poses()
    after = after.numpy() if hasattr(after, "numpy") else after
    if (nav.control.source != "manual" or any(session.command)
            or not nav.timeline.is_playing() or np.linalg.norm(after - before) > 0.08):
        raise RuntimeError("Robot moved during settling or capture")
    output = Path(output_path)
    if output.exists() or not camera.save(output, image):
        raise RuntimeError("Could not save fresh FrontCamera frame")

status = {
    "owner": nav.control.source, "command": session.command.tolist(),
    "playing": nav.timeline.is_playing(), "sim_ns": nav.sim_ns,
    "watchdog_expirations": nav.expirations, "commands_accepted": nav.accepted,
}
if nav.sample is not None:
    stamp, origin, _, _, _, basis = nav.sample
    xyz = origin - basis @ nav.lidar.translation
    status["scan_pose"] = {
        "stamp_ns": stamp, "x": float(xyz[0]), "y": float(xyz[1]),
        "yaw_rad": math.atan2(basis[1, 0], basis[0, 0]),
    }
if action == "capture":
    status["capture"] = capture
print(json.dumps(status))
