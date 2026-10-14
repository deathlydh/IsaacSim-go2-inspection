"""Open the inspection scene, spawn Go2 with its locomotion policy and arrow-key control.

Safe to re-run: the previous session (callbacks, keyboard, runner) is closed first.
Arrows: forward/back/strafe, N/M: turn. Click the viewport first so it gets the keys.

Injected args (isaacsim_send.py --arg):
    scene_path: scene to open (default D:/go2-inspection/scenes/inspection.usd)
    autoplay:   press Play after setup (default True)
"""

import sys

import carb
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
if "autoplay" not in dir():
    autoplay = True
if "sim_device" not in dir():
    sim_device = "cpu"  # measured: "cuda" for tensors/policy gives the same ~38 FPS for one robot

ROBOT_PATH = "/World/Robot"  # not /World/Go2: avoids clashing with a still-loaded NVIDIA example
SPAWN_POSITION = [0.0, 0.0, 0.5]
RENDER_HZ = 50  # 4 physics steps (200 Hz) per rendered frame, as in training


class Go2Session:
    """Owns the runner, physics callbacks and keyboard subscription for one run."""

    def __init__(self):
        self.command = np.zeros(3, dtype=np.float32)
        self.keymap = make_velocity_keyboard_mapping(1.5)
        self.ready = False
        self.failed = False
        self.runner = None
        self._ids = []
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._key_sub = None

    def start(self):
        self.runner = RobotPolicyRunner(get_go2_spec(), prim_path=ROBOT_PATH, position=SPAWN_POSITION)
        self.runner.spawn()
        self._ids.append(SimulationManager.register_callback(self._on_physics_step, IsaacEvents.POST_PHYSICS_STEP))
        self._ids.append(SimulationManager.register_callback(self._on_stop, IsaacEvents.TIMELINE_STOP))
        self._key_sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)

    def _on_physics_step(self, dt, context):
        runner = self.runner
        if runner is None or runner.articulation is None or self.failed:
            return
        if not runner.articulation.is_physics_tensor_entity_valid():
            self.ready = False
        try:
            if self.ready:
                runner.step(dt, self.command.tolist())
            else:
                runner.restart_from_default_state(self.command.tolist())
                self.ready = True
        except Exception as error:  # a physics callback must not raise
            self.failed = True
            carb.log_error(f"Go2Session: policy failed, stopping: {error}")

    def _on_stop(self, event):
        self.ready = False
        self.failed = False
        self.command.fill(0.0)

    def _on_key(self, event, *args, **kwargs):
        delta = self.keymap.get(event.input.name)
        if delta is not None:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self.command += np.asarray(delta, dtype=np.float32)
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                self.command -= np.asarray(delta, dtype=np.float32)
        return True

    def close(self):
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

# 1. Tear down a previous run.
previous = getattr(sys, "_go2_inspection_session", None)
if previous is not None:
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

# 4. Robot + policy + keyboard.
session = Go2Session()
session.start()
sys._go2_inspection_session = session
await app.next_update_async()

if autoplay:
    timeline.play()
    for _ in range(30):
        await app.next_update_async()

print("scene:", stage.GetRootLayer().identifier)
print("sim device:", SimulationManager.get_physics_sim_device(), "| policy engine:", session.runner.engine,
      "| gpu dynamics:", _physx_scene.GetEnableGPUDynamicsAttr().Get(), "| broadphase:", _physx_scene.GetBroadphaseTypeAttr().Get())
print("playing:", timeline.is_playing(), "| policy ready:", session.ready, "| failed:", session.failed)
