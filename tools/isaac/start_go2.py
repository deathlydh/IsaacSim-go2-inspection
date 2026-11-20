"""Open a scene, spawn Go2 with a locomotion policy, arrow-key control and the front camera.

Safe to re-run: the previous session (callbacks, keyboard, camera, runner) is closed first.
Arrows: forward/back/strafe, N/M: turn. Click the viewport first so it gets the keys.

Injected args (isaacsim_send.py --arg):
    scene_path: scene to open (default D:/go2-inspection/scenes/inspection.usd).
                If <scene>_meta.json exists next to it (factory_hall.usda does), the robot spawns
                at its robot_spawn x/y/yaw.
    policy:     "bundled"      - NVIDIA flat-terrain example policy (crouched "beetle" gait)
                "robot_lab"    - robot_lab policy from rl_sar (trained for a trot), tools/isaac/policies
                "nvidia_rough" - NVIDIA Isaac Lab rough-terrain checkpoint with a height scan
                (default "robot_lab", see docs/gait_ab/RESULTS.md)
    camera:     create the Go2 front camera (default True); camera_window: show its 640x360 window (default True)
    heading_hold: keep the heading while N/M are not pressed (default True)
    kp:         robot_lab only: actuator stiffness 25 (default, training) or 20 (rl_sar deployment)
    speed:      m/s (and rad/s for turning) per arrow key, clipped to the policy's training range (default 0.5)
    autoplay:   press Play after setup (default True)
    sim_device: "cpu" (default) - physics tensors device; the PhysX solver always runs on the CPU
"""

import importlib
import json
import math
import os
import sys

import carb
import carb.eventdispatcher
import numpy as np
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import PhysxSchema
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.robot.policy.examples import RobotPolicyRunner, get_go2_spec
from isaacsim.robot.policy.examples.interactive.example_base import make_velocity_keyboard_mapping

if "scene_path" not in dir():
    scene_path = "D:/go2-inspection/scenes/inspection.usd"
if "policy" not in dir():
    policy = "robot_lab"  # winner of the gait A/B test (docs/gait_ab/RESULTS.md)
if "camera" not in dir():
    camera = True
if "camera_window" not in dir():
    camera_window = True
if "speed" not in dir():
    speed = 0.5
if "autoplay" not in dir():
    autoplay = True
if "heading_hold" not in dir():
    heading_hold = True  # keep the heading while no turn key is pressed
if "kp" not in dir():
    kp = 25.0  # robot_lab only: actuator stiffness, 25 (training value) or 20 (rl_sar deployment)
if "sim_device" not in dir():
    sim_device = "cpu"  # measured: "cuda" for tensors/policy gives the same ~38 FPS for one robot

TOOLS_DIR = "D:/go2-inspection/tools/isaac"
ROBOT_PATH = "/World/Robot"  # not /World/Go2: avoids clashing with a still-loaded NVIDIA example
SPAWN_HEIGHT = 0.5           # base height at spawn; the robot drops onto its feet
RENDER_HZ = 50               # 4 physics steps (200 Hz) per rendered frame, as in training
COMMAND_LIMITS = np.array([1.0, 1.0, 1.0], dtype=np.float32)  # training range of all three policies
HEADING_GAIN = 2.0           # rad/s of turn command per rad of heading error (heading hold)

# Project modules: Kit caches os.stat for imports, so clear it or reload() may run stale bytecode.
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)
try:
    from omni.ext._impl import stat_cache as _stat_cache

    _stat_cache._stat_cache.clear()
except Exception:
    pass
import go2_camera
import command_control
import policies.nvidia_rough_go2
import policies.robot_lab_go2

for _module in (go2_camera, policies.robot_lab_go2, policies.nvidia_rough_go2):
    importlib.reload(_module)
FrontCamera = go2_camera.FrontCamera


def build_runner(name, position, orientation):
    """Create (not spawn) the policy runner for the chosen gait policy."""
    if name == "bundled":
        return RobotPolicyRunner(get_go2_spec(), prim_path=ROBOT_PATH, position=position, orientation=orientation)
    if name == "robot_lab":
        return policies.robot_lab_go2.build_runner(ROBOT_PATH, position, orientation=orientation, kp=float(kp))
    if name == "nvidia_rough":
        return policies.nvidia_rough_go2.build_runner(ROBOT_PATH, position, orientation=orientation)
    raise ValueError(f"unknown policy {name!r}: use bundled, robot_lab or nvidia_rough")


def spawn_pose(scene):
    """Spawn x/y/yaw from <scene>_meta.json if present, else the origin. Returns (position, wxyz, source)."""
    meta_path = os.path.splitext(scene)[0] + "_meta.json"
    x, y, yaw_deg, source = 0.0, 0.0, 0.0, "default"
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8") as f:
            spawn = json.load(f).get("robot_spawn", {})
        x, y = (float(v) for v in spawn.get("position", (0.0, 0.0))[:2])
        yaw_deg = float(spawn.get("yaw_deg", 0.0))
        source = os.path.basename(meta_path)
    half = math.radians(yaw_deg) / 2.0
    # meta stores the floor point; the runner wants the base position, so we use our own height.
    return [x, y, SPAWN_HEIGHT], [math.cos(half), 0.0, 0.0, math.sin(half)], source


class Go2Session:
    """Owns the runner, physics callbacks, keyboard subscription and camera for one run."""

    def __init__(self, policy_name, speed_mps, heading_hold_on=True):
        self.policy = policy_name
        self.heading_hold = heading_hold_on
        self._heading_target = None
        self.command = np.zeros(3, dtype=np.float32)
        self.keymap = make_velocity_keyboard_mapping(float(speed_mps))
        self.ready = False
        self.failed = False
        self.runner = None
        self.camera = None
        self._ids = []
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._key_sub = None
        self._stage_closing_sub = None

    def start(self, position, orientation, with_camera, with_window):
        self.runner = build_runner(self.policy, position, orientation)
        self.runner.spawn()
        try:
            # Startup properties exported with the policy (e.g. foot friction), as in training.
            self.runner.apply_scene_properties()
        except Exception as error:
            carb.log_warn(f"Go2Session: apply_scene_properties skipped: {error}")
        self._ids.append(SimulationManager.register_callback(self._on_physics_step, IsaacEvents.POST_PHYSICS_STEP))
        self._ids.append(SimulationManager.register_callback(self._on_stop, IsaacEvents.TIMELINE_STOP))
        command_control.install_control(self)
        if with_camera:
            self.camera = FrontCamera.create(robot_path=ROBOT_PATH, window=with_window)
        # If the stage is replaced (File > Open/New or a re-run), release everything BEFORE teardown:
        # a camera annotator outliving its render product corrupts the CUDA context (seen: error 700).
        self._stage_closing_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=lambda *_: self.close(),
            observer_name="go2_inspection.session.stage_closing",
        )

    def _on_physics_step(self, dt, context):
        runner = self.runner
        if runner is None or runner.articulation is None or self.failed:
            self.command.fill(0.0)
            return
        valid = runner.articulation.is_physics_tensor_entity_valid()
        source = getattr(getattr(self, "control", None), "source", "manual")
        if source in ("route", "ros") and (not valid or not self.ready):
            self.failed = True
            self.command.fill(0.0)
            carb.log_error(f"Go2Session: physics unavailable during {source}; refusing reset")
            return
        if not valid:
            self.ready = False
        command = np.clip(self.command, -COMMAND_LIMITS, COMMAND_LIMITS)
        try:
            if self.ready:
                runner.step(dt, self._hold_heading(runner, command).tolist())
            else:
                # Reset changes the pose. A heading sampled before the reset
                # belongs to the previous episode and can cause an unsolicited turn.
                self._heading_target = None
                runner.restart_from_default_state(command.tolist())
                self.ready = True
        except Exception as error:  # a physics callback must not raise
            self.failed = True
            self.command.fill(0.0)
            carb.log_error(f"Go2Session: policy failed, stopping: {error}")

    def _hold_heading(self, runner, command):
        """Heading hold, like Isaac Lab's heading command: while no turn key is pressed, steer the
        yaw-rate command toward the heading remembered when turning stopped. The robot_lab policy
        otherwise spins ~10 deg/s at zero command (measured, docs/gait_ab)."""
        if not self.heading_hold:
            return command
        if abs(command[2]) > 1e-3:          # user is turning: follow the key, forget the target
            self._heading_target = None
            return command
        pos, quat = runner.articulation.get_world_poses()
        w, x, y, z = np.asarray(quat.numpy() if hasattr(quat, "numpy") else quat).reshape(-1)[:4]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        if self._heading_target is None:
            self._heading_target = yaw
        error = (self._heading_target - yaw + math.pi) % (2.0 * math.pi) - math.pi
        command = command.copy()
        command[2] = float(np.clip(HEADING_GAIN * error, -COMMAND_LIMITS[2], COMMAND_LIMITS[2]))
        return command

    def _on_stop(self, event):
        if getattr(self, "control", None) is not None:
            self.control.cancel()
        self._heading_target = None
        self.ready = False
        self.failed = False
        self.command.fill(0.0)

    def _on_key(self, event, *args, **kwargs):
        return self.control.on_key(event, *args, **kwargs)

    def close(self):
        """Release camera first, then callbacks, keyboard and runner. Idempotent; never touches the timeline."""
        if getattr(self, "navigation", None) is not None:
            self.navigation.close()
        if getattr(self, "control", None) is not None:
            self.control.cancel()
            self.control._timeline_sub = None
        self._stage_closing_sub = None
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        for cb_id in self._ids:
            SimulationManager.deregister_callback(cb_id)
        self._ids.clear()
        if self._key_sub is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._key_sub)
            self._key_sub = None
        if self.runner is not None:
            self.runner.close()
            self.runner = None


app = omni.kit.app.get_app()
timeline = omni.timeline.get_timeline_interface()
ctx = omni.usd.get_context()

# 1. Tear down a previous run (camera first, see Go2Session.close).
previous = getattr(sys, "_go2_inspection_session", None)
if previous is not None:
    if getattr(previous, "navigation", None) is not None:
        previous.navigation.close()
    previous.close()
    sys._go2_inspection_session = None
# Stop and settle before replacing the stage: Isaac Sim 6.1 crashes
# (virtual_gantry -> debug_draw) if Stop fires during stage teardown.
timeline.stop()
for _ in range(10):
    await app.next_update_async()

# 2. Open the scene from disk (always fresh, so manual edits must be saved to persist).
await ctx.open_stage_async(scene_path)
for _ in range(10):
    await app.next_update_async()
stage = ctx.get_stage()
if stage.GetPrimAtPath(ROBOT_PATH).IsValid():  # robot accidentally saved into the scene
    stage.RemovePrim(ROBOT_PATH)

# 3. Physics on CPU, rendering at 50 Hz.
SimulationManager.set_physics_sim_device(sim_device)
# Keep the PhysX solver itself on CPU (faster for one robot) even when tensors/policy use CUDA.
_physx_scene = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/PhysicsScene"))
_physx_scene.GetEnableGPUDynamicsAttr().Set(False)
_physx_scene.GetBroadphaseTypeAttr().Set("MBP")
# set_dt keeps rate limit, timeline time codes and the Isaac loop's manual step size in sync;
# setting only the carb rate limit left the loop stepping 0.01 s (2 physics steps per frame).
carb.settings.get_settings().set("/app/runLoops/main/rateLimitEnabled", True)
RenderingManager.set_dt(1.0 / RENDER_HZ)

# 4. Robot + policy + keyboard + camera.
position, orientation, spawn_source = spawn_pose(scene_path)
session = Go2Session(policy, speed, heading_hold)
session.start(position, orientation, camera, camera_window)
sys._go2_inspection_session = session
await app.next_update_async()

if autoplay:
    timeline.play()
    for _ in range(30):
        await app.next_update_async()

print("scene:", stage.GetRootLayer().identifier, "| spawn:", [round(v, 2) for v in position], "from", spawn_source)
print("policy:", policy, "| engine:", session.runner.engine, "| speed per key:", speed,
      "| sim device:", SimulationManager.get_physics_sim_device(),
      "| gpu dynamics:", _physx_scene.GetEnableGPUDynamicsAttr().Get())
print("camera:", None if session.camera is None else session.camera.intrinsics())
print("playing:", timeline.is_playing(), "| policy ready:", session.ready, "| failed:", session.failed)
