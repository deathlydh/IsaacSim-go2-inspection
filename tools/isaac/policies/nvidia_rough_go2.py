"""Gait option "B": NVIDIA's Isaac Lab rough-terrain Go2 policy, run by Isaac Sim's RobotPolicyRunner.

WHAT
    ``build_runner()`` returns an ``isaacsim.robot.policy.examples.RobotPolicyRunner`` (not yet
    spawned) that drives the same Isaac Lab ``go2.usd`` as the bundled NVIDIA example, but with the
    policy trained by NVIDIA on the *rough-terrain* task ``Isaac-Velocity-Rough-UnitreeGo2``
    (stairs, boxes, slopes). It is a drop-in replacement for
    ``RobotPolicyRunner(get_go2_spec(), ...)`` in ``start_go2.py``: same ``spawn()``,
    ``step(dt, command)``, ``restart_from_default_state(command)`` and ``[vx, vy, wz]`` command.

WHERE THE FILES COME FROM (see assets/policies/nvidia_rough_go2/README.md for hashes)
    - ``policy.pt``: TorchScript export of the actor network of the raw rsl_rl training checkpoint
      ``<assets_root>/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/
      Isaac-Velocity-Rough-UnitreeGo2_physx_none_rsl_rl.pt`` (Isaac Sim 6.1 asset library, iteration
      1499). The raw checkpoint is a *training* file (actor + critic + optimizer) that the runner
      cannot load; :func:`export_torchscript` rebuilds the actor and saves the deployable file.
    - ``env.yaml``: the bundled ``/Isaac/Samples/Policies/go2/physx_env.yaml`` unchanged except for
      a header comment. Despite being shipped next to the *flat* policy, that file is an export of
      the Go2 *rough* task config (height scanner, terrain generator, rough reward weights), i.e.
      the simulation settings this checkpoint was trained with.

WHY A HAND-WRITTEN BINDING
    NVIDIA ships no "IO descriptor" for this checkpoint, and the descriptor format of the bundled
    examples cannot express a height scan anyway. So the observation/action layout is authored here
    and handed to the runner through ``PolicySpec(binding=...)`` -- the escape hatch NVIDIA uses for
    its Franka example (``bundled/franka.py``). The height scan reaches the runner as a *task term*:
    every control tick the runner calls ``provider.read()`` (see :class:`HeightScanProvider`) and the
    binding copies ``context["height_scan.joint_pos"]`` into the observation. (The runtime's
    term is called ``task_joint_pos`` because Franka uses it for a drawer joint; it simply copies a
    named float vector from the task context, which is all we need.) This is an internal extension
    point of Isaac Sim 6.1, not a stable public API.

POLICY INTERFACE (from the env yaml + the checkpoint's weight shapes; verified offline)
    Observation: 235 floats, no history, no scaling, no normalizer::

        [  0:  3] base linear velocity, body frame (m/s)
        [  3:  6] base angular velocity, body frame (rad/s)
        [  6:  9] gravity direction, body frame ((0, 0, -1) when level)
        [  9: 12] command [vx m/s, vy m/s, wz rad/s]
        [ 12: 24] joint position - default pose (rad)
        [ 24: 36] joint velocity (rad/s)
        [ 36: 48] previous raw network output
        [ 48:235] height scan: 17 x 11 rays, 0.1 m apart, 1.6 m (x, forward) by 1.0 m (y, left)
                  around the base, yaw-aligned; value = base_z - ground_z - 0.5, clipped to [-1, 1]

    The first 48 floats are exactly the bundled flat Go2 policy's layout. 12-wide blocks use the
    Isaac Lab PhysX joint order (FL/FR/RL/RR hips, then thighs, then calves) == go2.usd DOF order.

    Action: 12 raw outputs ``a`` -> joint position targets ``default_pose + 0.25 * a``.
    Network: MLP 235 -> 512 -> 256 -> 128 -> 12, ELU (checked against the state dict).
    Timing: physics 200 Hz, one policy step every 4 physics steps (50 Hz).
    Actuators: explicit DC-motor model, kp 25, kd 0.5, 23.5 N*m, 30 rad/s (the Isaac Lab Go2 config
    used when this checkpoint was made; see the README for why not 23.7/45.43).

HEIGHT SCAN (how Isaac Lab computes it, and how this module reproduces it)
    Isaac Lab attaches a ray-caster to ``Robot/base``: 187 vertical rays start 20 m above the base
    (rotated by the base *yaw* only) and hit the terrain mesh. Observation = ``base_z - hit_z - 0.5``
    with noise during training, clipped to [-1, 1]. On a flat floor every value is
    ``base_z - floor_z - 0.5`` (about -0.17 when standing at 0.33 m).

    :class:`HeightScanProvider` offers two modes:

    - ``"raycast"`` (default): real PhysX scene queries (``omni.physx`` ``raycast_all``), one per
      ray, ignoring hits on the robot itself. Rays start 0.6 m above the base instead of 20 m:
      anything higher than ``base_z + 0.5`` saturates at -1 anyway, and the low start keeps
      ceilings, overhead pipes and the camera mast out of the scan. Measured cost of one read
      (187 rays + pose): median 1.3 ms from a script, 1.8 ms (max 3.8 ms) inside the physics
      callback; it runs once per control tick (50 Hz, i.e. once per rendered frame).
    - ``"flat"``: no queries; assumes a flat floor at ``floor_z`` (0.0 in our scenes); ~0.13 ms.

    Known limits of the raycast mode: obstacles whose top is above the ray start and whose
    collider is a triangle mesh are invisible from inside (single-sided triangles); convex/box
    colliders that contain the ray start should report a hit at the start (reads -1), but this was
    not tested (the live stage had no obstacles). Isaac Lab sees only the terrain mesh; here every
    non-robot collider counts (boxes, other robots, people).

USAGE (inside Isaac Sim, e.g. in a copy of start_go2.py)::

    import sys
    sys.path.insert(0, "D:/go2-inspection/tools/isaac")
    from policies import nvidia_rough_go2

    runner = nvidia_rough_go2.build_runner("/World/Robot", position=(0.0, 0.0, 0.45))
    runner.spawn()                    # before Play, exactly like the bundled runner
    runner.apply_scene_properties()   # optional: foot friction 0.8/0.6 as at training
    keymap = make_velocity_keyboard_mapping(nvidia_rough_go2.KEYBOARD_SCALE)
    # in the physics callback:
    runner.step(dt, nvidia_rough_go2.clip_command(command).tolist())
"""

from __future__ import annotations

import functools
import hashlib
import io
import math
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
from isaacsim.robot.policy.examples import PolicyArtifact, PolicySpec, RobotPolicyRunner, get_go2_spec
from isaacsim.robot.policy.examples.binding import PolicyBinding, Term

__all__ = [
    "ACTION_SCALE",
    "COMMAND_LIMITS",
    "GO2_JOINT_NAMES",
    "HEIGHT_SCAN_WIDTH",
    "HeightScanProvider",
    "KEYBOARD_SCALE",
    "MODEL_SHA256",
    "build_runner",
    "clip_command",
    "export_torchscript",
    "get_nvidia_rough_go2_spec",
    "height_scan_pattern",
    "make_height_scan_provider",
    "nominal_observation",
    "observation_layout",
    "rough_go2_binding",
]

# ---------------------------------------------------------------------------------------------
# Files and constants
# ---------------------------------------------------------------------------------------------

#: tools/isaac/policies/nvidia_rough_go2.py -> project root is three folders up.
ASSET_DIR = Path(__file__).resolve().parents[3] / "assets" / "policies" / "nvidia_rough_go2"
MODEL_PATH = ASSET_DIR / "policy.pt"
ENV_YAML_PATH = ASSET_DIR / "env.yaml"

#: SHA-256 of policy.pt as written by export_torchscript() on 2026-10-14 (torch 2.11.0+cu128 in
#: Isaac Sim 6.1). The runner refuses to deserialize a TorchScript file whose bytes do not match
#: (loading TorchScript can run code). A re-export gives the same weights but usually NOT the same
#: bytes (TorchScript embeds class names that depend on what was compiled earlier in the process,
#: and the source lines of this file); after a re-export put the new digest here and in the README.
MODEL_SHA256 = "dcb7e8ab29a65625b03ecd73f43b8bb540533b658c4fd1b7a638342ea25f6d8f"

#: The raw rsl_rl checkpoint in the Isaac Sim 6.1 asset library (relative to the assets root).
CHECKPOINT_REL_PATH = (
    "/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-Rough-UnitreeGo2_physx_none_rsl_rl.pt"
)
#: Its SHA-256 and size as read on 2026-10-14 (server timestamp 2026-08-14 07:15:23).
CHECKPOINT_SHA256 = "f15732c603c581c623edd7c0227b4f41b1d1916a15a748f96d8d4de071fb9570"
CHECKPOINT_SIZE = 6_882_293

#: Actor network layer widths: input, hidden..., output (checked against the state dict).
ACTOR_DIMS = (235, 512, 256, 128, 12)

#: Isaac Lab PhysX joint order of go2.usd (identical to the live articulation's dof_names and to
#: the bundled go2_physx_IO_descriptors.yaml). The policy reads and writes 12-wide blocks in it.
GO2_JOINT_NAMES: tuple[str, ...] = (
    "FL_hip_joint",
    "FR_hip_joint",
    "RL_hip_joint",
    "RR_hip_joint",
    "FL_thigh_joint",
    "FR_thigh_joint",
    "RL_thigh_joint",
    "RR_thigh_joint",
    "FL_calf_joint",
    "FR_calf_joint",
    "RL_calf_joint",
    "RR_calf_joint",
)

#: JointPositionAction scale of the rough task (env.yaml ``actions.joint_pos.scale``).
ACTION_SCALE = 0.25

#: Largest |command| per axis [vx m/s, vy m/s, wz rad/s]: the training command ranges in env.yaml
#: (``commands.base_velocity.ranges``: lin_vel_x/y and ang_vel_z all (-1, 1)).
COMMAND_LIMITS = (1.0, 1.0, 1.0)

#: Command step per arrow/N/M key for make_velocity_keyboard_mapping(). Half of COMMAND_LIMITS:
#: 0.5 m/s is the inspection walking speed recommended in docs/PLAN_factory_camera_gait.md, and two
#: keys pressed together (forward + strafe) still stay inside the training range.
KEYBOARD_SCALE = 0.5

# Height scanner of the rough task (env.yaml ``scene.height_scanner`` and
# ``observations.policy.height_scan``): grid_pattern(resolution=0.1, size=[1.6, 1.0], ordering="xy").
HEIGHT_SCAN_RESOLUTION = 0.1
HEIGHT_SCAN_SIZE = (1.6, 1.0)
HEIGHT_SCAN_OFFSET = 0.5  # isaaclab.envs.mdp.observations.height_scan(offset=0.5)
HEIGHT_SCAN_CLIP = (-1.0, 1.0)
HEIGHT_SCAN_WIDTH = 187  # 17 x 11
#: Task-context namespace; the binding reads ``context[f"{HEIGHT_SCAN_CONTEXT}.joint_pos"]``.
HEIGHT_SCAN_CONTEXT = "height_scan"
#: Surfaces higher than base_z + 0.5 all give -1 (clip), surfaces lower than base_z - 1.5 all give +1.
SATURATION_ABOVE_BASE = -(HEIGHT_SCAN_OFFSET + HEIGHT_SCAN_CLIP[0])  # 0.5 m
SATURATION_BELOW_BASE = HEIGHT_SCAN_OFFSET + HEIGHT_SCAN_CLIP[1]  # 1.5 m
#: Raycast mode: rays start this far above the base (Isaac Lab: 20 m). Anything above
#: SATURATION_ABOVE_BASE reads -1 anyway, so starting just above it loses nothing.
RAY_START_ABOVE_BASE = 0.6
#: Raycast mode: rays end this far below the base (a little past SATURATION_BELOW_BASE).
RAY_END_BELOW_BASE = 1.6

#: Approximate base height when standing in the default pose (planar leg model: thigh and calf
#: 0.213 m, foot radius 0.022 m, thigh 0.8 rad, calf -1.5 rad). Only used by nominal_observation().
NOMINAL_BASE_HEIGHT = 0.333


# ---------------------------------------------------------------------------------------------
# Height scan geometry (pure numpy; mirrors Isaac Lab's grid_pattern + RayCaster "yaw" alignment)
# ---------------------------------------------------------------------------------------------


def height_scan_pattern() -> np.ndarray:
    """Return the 187 ray start points in the base yaw frame, in Isaac Lab's order.

    Isaac Lab ``grid_pattern`` with ``ordering="xy"`` builds ``x = arange(-0.8, 0.8, 0.1)``
    (17 values, forward axis), ``y = arange(-0.5, 0.5, 0.1)`` (11 values, left axis) and flattens
    ``torch.meshgrid(x, y, indexing="xy")``, so **x changes fastest**: ray ``k`` has
    ``x = x[k % 17]``, ``y = y[k // 17]``.

    Returns:
        float32 array of shape (187, 2): local (x, y) of each ray in metres.
    """
    half_x, half_y = HEIGHT_SCAN_SIZE[0] / 2.0, HEIGHT_SCAN_SIZE[1] / 2.0
    xs = np.arange(-half_x, half_x + 1.0e-9, HEIGHT_SCAN_RESOLUTION)
    ys = np.arange(-half_y, half_y + 1.0e-9, HEIGHT_SCAN_RESOLUTION)
    grid_x, grid_y = np.meshgrid(xs, ys, indexing="xy")  # shape (11, 17), same as torch "xy"
    pattern = np.stack([grid_x.ravel(), grid_y.ravel()], axis=1).astype(np.float32)
    if pattern.shape != (HEIGHT_SCAN_WIDTH, 2):
        raise RuntimeError(f"height scan pattern has shape {pattern.shape}, expected ({HEIGHT_SCAN_WIDTH}, 2).")
    return pattern


def yaw_from_quaternion_wxyz(quaternion: Sequence[float]) -> float:
    """Heading angle (rotation about world z) of a WXYZ quaternion, in radians.

    Args:
        quaternion: Base orientation (w, x, y, z).

    Returns:
        Yaw in (-pi, pi].
    """
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def height_scan_ray_xy(base_position: Sequence[float], base_orientation_wxyz: Sequence[float]) -> np.ndarray:
    """World (x, y) of each ray: the grid rotated by the base yaw only, then moved to the base.

    Roll and pitch are ignored on purpose (Isaac Lab ``ray_alignment="yaw"`` /
    ``attach_yaw_only=True``): the rays always point straight down.

    Args:
        base_position: Base (root link) world position (x, y, z).
        base_orientation_wxyz: Base world orientation (w, x, y, z).

    Returns:
        float64 array of shape (187, 2).
    """
    yaw = yaw_from_quaternion_wxyz(base_orientation_wxyz)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    rotation_t = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]])  # row vectors: p @ R^T
    return _PATTERN.astype(np.float64) @ rotation_t + np.asarray(base_position, dtype=np.float64)[:2]


def height_scan_values(base_z: float, hit_z: np.ndarray | float) -> np.ndarray:
    """Isaac Lab height-scan formula ``clip(base_z - hit_z - 0.5, -1, 1)``.

    ``hit_z = -inf`` (nothing below the ray) gives +1, like a hole deeper than 1.5 m.

    Args:
        base_z: Base (sensor) height in the world.
        hit_z: Ground height(s) under the rays.

    Returns:
        float32 array of 187 values (or of the shape of ``hit_z``).
    """
    values = float(base_z) - np.asarray(hit_z, dtype=np.float64) - HEIGHT_SCAN_OFFSET
    return np.clip(values, HEIGHT_SCAN_CLIP[0], HEIGHT_SCAN_CLIP[1]).astype(np.float32)


_PATTERN = height_scan_pattern()


# ---------------------------------------------------------------------------------------------
# Task state provider: feeds the 187-wide height scan to the runner
# ---------------------------------------------------------------------------------------------


def _flat_float64(values: Any) -> np.ndarray:
    """Convert a warp/numpy container to a flat float64 numpy vector."""
    to_numpy = getattr(values, "numpy", None)
    if callable(to_numpy):
        values = to_numpy()
    return np.asarray(values, dtype=np.float64).reshape(-1)


class HeightScanProvider:
    """Computes the rough policy's height scan from the robot's base pose every control tick.

    The runner creates it once (through :func:`make_height_scan_provider`) with the spawned
    articulation and calls :meth:`read` on every policy step (50 Hz), from the physics callback.

    Args:
        articulation: The robot articulation (``runner.articulation``); only its root pose is read.
        mode: ``"raycast"`` (PhysX scene queries, sees obstacles) or ``"flat"`` (assume a flat
            floor at ``floor_z``, no queries).
        floor_z: Floor height used by ``"flat"`` mode and as the fallback when raycasting fails.
        robot_prim_path: Prim path of the robot; ray hits on this prim or any prim under it (links,
            attached sensors) are ignored. Defaults to the articulation's path. Hits on any of the
            articulation's own links (``articulation.link_paths``) are ignored in every case:
            Isaac Sim reports the articulation path as its ROOT LINK (``/World/Robot/base`` for
            go2.usd), and the leg links are siblings of it, not children.
        ray_start_above_base: Raycast mode: ray origin height above the base (m).
        ray_end_below_base: Raycast mode: lowest point a ray reaches below the base (m).

    Attributes:
        last_values: The 187 values returned by the last :meth:`read` (float32).
        last_hit_z: Ground height under each ray at the last read (``-inf`` = nothing found).
        reads: Number of :meth:`read` calls.
        seconds: Total wall time spent in :meth:`read` (for profiling: ``seconds / reads``).
    """

    def __init__(
        self,
        articulation: Any,
        *,
        mode: str = "raycast",
        floor_z: float = 0.0,
        robot_prim_path: str | None = None,
        ray_start_above_base: float = RAY_START_ABOVE_BASE,
        ray_end_below_base: float = RAY_END_BELOW_BASE,
    ) -> None:
        if mode not in ("raycast", "flat"):
            raise ValueError(f"HeightScanProvider mode must be 'raycast' or 'flat', got {mode!r}.")
        if ray_start_above_base <= SATURATION_ABOVE_BASE or ray_end_below_base < SATURATION_BELOW_BASE:
            raise ValueError(
                f"Rays must cover base_z + {SATURATION_ABOVE_BASE} m down to base_z - {SATURATION_BELOW_BASE} m "
                f"(got start +{ray_start_above_base}, end -{ray_end_below_base})."
            )
        self._articulation = articulation
        self.mode = mode
        self.floor_z = float(floor_z)
        path = robot_prim_path if robot_prim_path is not None else str(articulation.paths[0])
        self._robot_path = path.rstrip("/")
        self._robot_prefix = self._robot_path + "/"
        # Every rigid-body link of the robot (base, hips, thighs, calves, feet, head). A collider's
        # hit reports the link it belongs to, so this set alone filters the whole robot even when
        # robot_prim_path is only the root link (the articulation path).
        try:
            link_paths = articulation.link_paths[0]
        except (AttributeError, IndexError, TypeError):
            link_paths = ()
        self._robot_links = frozenset(str(link) for link in link_paths)
        self._start_above = float(ray_start_above_base)
        self._max_distance = float(ray_start_above_base + ray_end_below_base)
        self._scene_query = None
        if mode == "raycast":
            from omni.physx import get_physx_scene_query_interface

            self._scene_query = get_physx_scene_query_interface()
        self._warned = False
        self.last_values = np.zeros(HEIGHT_SCAN_WIDTH, dtype=np.float32)
        self.last_hit_z = np.full(HEIGHT_SCAN_WIDTH, self.floor_z, dtype=np.float64)
        self.reads = 0
        self.seconds = 0.0

    # -- runner contract ----------------------------------------------------------------------

    def read(self) -> Mapping[str, np.ndarray]:
        """Return the task context for one control tick: ``{"height_scan.joint_pos": 187 floats}``."""
        started = time.perf_counter()
        positions, orientations = self._articulation.get_world_poses()
        values = self.scan(_flat_float64(positions)[:3], _flat_float64(orientations)[:4])
        self.reads += 1
        self.seconds += time.perf_counter() - started
        return {f"{HEIGHT_SCAN_CONTEXT}.joint_pos": values}

    def reset(self) -> None:
        """Nothing to reset (the scan has no memory); present for the runner's reset() hook."""

    # -- computation ----------------------------------------------------------------------------

    def scan(self, base_position: Sequence[float], base_orientation_wxyz: Sequence[float]) -> np.ndarray:
        """Compute the 187 height-scan values for a given base pose.

        Args:
            base_position: Base world position (x, y, z).
            base_orientation_wxyz: Base world orientation (w, x, y, z).

        Returns:
            float32 array of 187 values in [-1, 1], Isaac Lab ray order.
        """
        base_z = float(base_position[2])
        if self.mode == "raycast":
            try:
                hit_z = self._raycast_ground(height_scan_ray_xy(base_position, base_orientation_wxyz), base_z)
            except Exception as error:  # never let a scene-query problem stop the robot
                if not self._warned:
                    import carb

                    carb.log_warn(f"HeightScanProvider: raycast failed ({error}); using a flat floor at {self.floor_z}.")
                    self._warned = True
                hit_z = np.full(HEIGHT_SCAN_WIDTH, self.floor_z, dtype=np.float64)
        else:
            hit_z = np.full(HEIGHT_SCAN_WIDTH, self.floor_z, dtype=np.float64)
        self.last_hit_z = hit_z
        self.last_values = height_scan_values(base_z, hit_z)
        return self.last_values

    def _raycast_ground(self, ray_xy: np.ndarray, base_z: float) -> np.ndarray:
        """Cast one vertical ray per grid point and return the highest non-robot surface under it.

        ``raycast_all`` reports every surface along the ray; the callback keeps the nearest one
        that does not belong to the robot (its rigid body is not one of the robot's links and its
        path is not under ``robot_prim_path``).
        Rays that find nothing within ``ray_start_above_base + ray_end_below_base`` return -inf.
        """
        query = self._scene_query
        start_z = base_z + self._start_above
        max_distance = self._max_distance
        robot_path, robot_prefix, robot_links = self._robot_path, self._robot_prefix, self._robot_links
        down = (0.0, 0.0, -1.0)
        nearest = [math.inf]

        def report(hit) -> bool:
            distance = hit.distance
            if distance < nearest[0]:
                body = hit.rigid_body
                if body not in robot_links and body != robot_path and not body.startswith(robot_prefix):
                    nearest[0] = distance
            return True  # keep traversing: a robot hit may hide the ground behind it

        hit_z = np.full(HEIGHT_SCAN_WIDTH, -np.inf, dtype=np.float64)
        for index in range(HEIGHT_SCAN_WIDTH):
            nearest[0] = math.inf
            query.raycast_all((float(ray_xy[index, 0]), float(ray_xy[index, 1]), start_z), down, max_distance, report)
            if nearest[0] != math.inf:
                hit_z[index] = start_z - nearest[0]
        return hit_z


def make_height_scan_provider(mode: str = "raycast", floor_z: float = 0.0, **kwargs: Any) -> Callable[[Any], HeightScanProvider]:
    """Build the ``task_state_provider`` factory that :class:`RobotPolicyRunner` expects.

    The runner calls the factory once, when it builds its runtime, with the spawned articulation.

    Args:
        mode: ``"raycast"`` or ``"flat"`` (see :class:`HeightScanProvider`).
        floor_z: Flat-floor height (flat mode and raycast fallback).
        **kwargs: Further :class:`HeightScanProvider` options.

    Returns:
        Factory ``articulation -> HeightScanProvider``.
    """

    def factory(articulation: Any) -> HeightScanProvider:
        return HeightScanProvider(articulation, mode=mode, floor_z=floor_z, **kwargs)

    return factory


# ---------------------------------------------------------------------------------------------
# Binding: 48 standard locomotion terms + the 187-wide height scan
# ---------------------------------------------------------------------------------------------


def rough_go2_binding(env_config: Any) -> PolicyBinding:
    """PolicySpec binding hook: the 235-wide observation / 12-wide action layout of the policy.

    Term order and settings follow ``observations.policy`` of env.yaml (Isaac Lab
    ``LocomotionVelocityRoughEnvCfg``): no observation scales, only the height scan is clipped.
    Default joint positions (observation and action offsets) come from env.yaml by joint name, so
    the reset pose and the policy's reference pose cannot disagree.

    Args:
        env_config: ``PolicyEnvConfig`` of env.yaml (the runner passes it).

    Returns:
        The policy binding.

    Raises:
        ValueError: If env.yaml timing differs from the training timing (200 Hz / decimation 4).
    """
    timing = env_config.timing
    if abs(timing.physics_dt - 0.005) > 1e-9 or timing.decimation != 4:
        raise ValueError(f"env.yaml timing dt={timing.physics_dt}, decimation={timing.decimation}; expected 0.005 / 4.")
    joints = GO2_JOINT_NAMES
    properties = env_config.joint_properties(joints)
    default_positions = tuple(float(value) for value in properties.default_positions)
    default_velocities = tuple(float(value) for value in properties.default_velocities)
    dofs = len(joints)

    observation_terms = (
        Term(semantic="base_lin_vel", width=3),
        Term(semantic="base_ang_vel", width=3),
        Term(semantic="projected_gravity", width=3),
        Term(semantic="generated_commands", width=3),
        Term(semantic="joint_pos_rel", width=dofs, joint_names=joints, offsets=default_positions),
        Term(semantic="joint_vel_rel", width=dofs, joint_names=joints, offsets=default_velocities),
        Term(semantic="last_action", width=dofs),
        # Generic "copy a vector from the task context" term (see module docstring).
        Term(semantic="task_joint_pos", width=HEIGHT_SCAN_WIDTH, task_refs=(HEIGHT_SCAN_CONTEXT,), clip=HEIGHT_SCAN_CLIP),
    )
    action_term = Term(
        semantic="joint_position",
        width=dofs,
        joint_names=joints,
        offsets=default_positions,
        scale=ACTION_SCALE,
    )
    return PolicyBinding(observation_terms=observation_terms, action_terms=(action_term,))


def observation_layout() -> list[tuple[str, int, int]]:
    """Return ``(name, start, stop)`` for each observation slice (names as in env.yaml)."""
    names_widths = (
        ("base_lin_vel", 3),
        ("base_ang_vel", 3),
        ("projected_gravity", 3),
        ("velocity_commands", 3),
        ("joint_pos", 12),
        ("joint_vel", 12),
        ("actions", 12),
        ("height_scan", HEIGHT_SCAN_WIDTH),
    )
    layout, start = [], 0
    for name, width in names_widths:
        layout.append((name, start, start + width))
        start += width
    return layout


# ---------------------------------------------------------------------------------------------
# Spec and runner
# ---------------------------------------------------------------------------------------------


def get_nvidia_rough_go2_spec() -> PolicySpec:
    """Build the frozen :class:`PolicySpec` for the rough Go2 policy (PhysX only).

    Returns:
        TorchScript model with pinned SHA-256, local env.yaml, the bundled Go2 USD, identity spawn
        orientation and the hand-written binding hook.
    """
    artifact = PolicyArtifact.from_files(
        MODEL_PATH.as_posix(),
        ENV_YAML_PATH.as_posix(),
        None,  # no IO descriptor: the interface comes from rough_go2_binding()
        model_sha256=MODEL_SHA256,
    )
    return PolicySpec(
        name="go2_nvidia_rough",
        engines={"physx": artifact},
        usd_path=get_go2_spec().usd_path,  # exactly the asset of the bundled NVIDIA Go2 example
        default_spawn_orientation=(1.0, 0.0, 0.0, 0.0),
        binding=rough_go2_binding,
    )


def build_runner(
    prim_path: str = "/World/Robot",
    position: Sequence[float] = (0.0, 0.0, 0.45),
    *,
    height_scan: str = "raycast",
    floor_z: float = 0.0,
    orientation: Sequence[float] | None = None,
) -> RobotPolicyRunner:
    """Create (but do not spawn) the runner for the NVIDIA rough-terrain Go2 policy.

    Nothing touches the stage here. The caller then follows the usual runner life cycle:
    ``spawn()`` before Play (references go2.usd at ``prim_path``), then from a physics callback
    ``restart_from_default_state(command)`` once and ``step(dt, command)`` on every physics step.
    The height-scan provider is created by the runner at its first initialize.

    Args:
        prim_path: Stage path of the robot.
        position: World spawn position in metres.
        height_scan: ``"raycast"`` (PhysX rays, sees steps/obstacles; ~1 ms per policy step) or
            ``"flat"`` (assume a flat floor at ``floor_z``; free).
        floor_z: Floor height for flat mode and the raycast fallback.
        orientation: World spawn orientation (w, x, y, z), or None for the spec default (identity).

    Returns:
        The unspawned :class:`RobotPolicyRunner`.
    """
    return RobotPolicyRunner(
        get_nvidia_rough_go2_spec(),
        prim_path=prim_path,
        position=[float(value) for value in position],
        orientation=None if orientation is None else [float(value) for value in orientation],
        task_state_provider=make_height_scan_provider(height_scan, floor_z, robot_prim_path=prim_path),
    )


# ---------------------------------------------------------------------------------------------
# Small helpers for the integrator and for tests
# ---------------------------------------------------------------------------------------------


def clip_command(command: Sequence[float]) -> np.ndarray:
    """Clip a ``[vx, vy, wz]`` command to :data:`COMMAND_LIMITS` (keeps keyboard sums in range).

    Args:
        command: Base-frame velocity command.

    Returns:
        float32 array of 3 values.
    """
    limits = np.asarray(COMMAND_LIMITS, dtype=np.float32)
    return np.clip(np.asarray(command, dtype=np.float32).reshape(3), -limits, limits)


def nominal_observation(
    base_height: float = NOMINAL_BASE_HEIGHT,
    command: Sequence[float] = (0.0, 0.0, 0.0),
    floor_z: float = 0.0,
) -> np.ndarray:
    """Observation of a robot standing still and level in its default pose on a flat floor.

    Args:
        base_height: Base height above the world origin (m).
        command: ``[vx, vy, wz]`` command.
        floor_z: Floor height.

    Returns:
        float32 vector of 235 values.
    """
    observation = np.zeros(ACTOR_DIMS[0], dtype=np.float32)
    observation[6:9] = (0.0, 0.0, -1.0)
    observation[9:12] = np.asarray(command, dtype=np.float32)
    observation[48:] = height_scan_values(base_height, np.full(HEIGHT_SCAN_WIDTH, floor_z))
    return observation


# ---------------------------------------------------------------------------------------------
# One-time export: raw rsl_rl checkpoint -> deployable TorchScript (run inside Isaac Sim)
# ---------------------------------------------------------------------------------------------


def _import_torch() -> Any:
    import torch

    return torch


def load_checkpoint(checkpoint_url: str | None = None) -> dict:
    """Read the raw rsl_rl checkpoint from the asset library (``omni.client``) and verify it.

    ``torch.load(weights_only=True)`` only unpickles tensors and plain containers, so no code from
    the file is executed. The SHA-256 is checked against :data:`CHECKPOINT_SHA256` first.

    Args:
        checkpoint_url: Override URL/path; default ``<assets_root>`` + :data:`CHECKPOINT_REL_PATH`.

    Returns:
        The checkpoint dictionary (``actor_state_dict``, ``critic_state_dict``, ``iter``, ...).
    """
    import omni.client

    if checkpoint_url is None:
        from isaacsim.storage.native import get_assets_root_path

        checkpoint_url = f"{get_assets_root_path()}{CHECKPOINT_REL_PATH}"
    result, _, content = omni.client.read_file(checkpoint_url)
    if result != omni.client.Result.OK:
        raise FileNotFoundError(f"Could not read {checkpoint_url!r}: {result}.")
    data = memoryview(content).tobytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != CHECKPOINT_SHA256:
        raise RuntimeError(f"Checkpoint {checkpoint_url!r} has SHA-256 {digest}, expected {CHECKPOINT_SHA256}.")
    torch = _import_torch()
    return torch.load(io.BytesIO(data), map_location="cpu", weights_only=True)


def check_actor_state_dict(actor_state_dict: Mapping[str, Any]) -> None:
    """Verify that the actor is a plain ELU MLP of :data:`ACTOR_DIMS` with no input normalizer.

    rsl_rl 4/5 stores the actor as ``mlp.{0,2,4,6}.weight/bias`` (Linear layers; the odd indices
    are the parameter-free ELU activations) plus ``distribution.std_param`` (exploration noise,
    unused at deployment). An observation normalizer would add ``obs_normalizer.*`` keys.

    Raises:
        ValueError: On any unexpected key or shape.
    """
    expected = {"distribution.std_param"}
    for layer in range(len(ACTOR_DIMS) - 1):
        expected |= {f"mlp.{2 * layer}.weight", f"mlp.{2 * layer}.bias"}
    keys = set(actor_state_dict)
    if keys != expected:
        raise ValueError(f"Unexpected actor keys: extra {sorted(keys - expected)}, missing {sorted(expected - keys)}.")
    for layer in range(len(ACTOR_DIMS) - 1):
        weight = tuple(actor_state_dict[f"mlp.{2 * layer}.weight"].shape)
        bias = tuple(actor_state_dict[f"mlp.{2 * layer}.bias"].shape)
        if weight != (ACTOR_DIMS[layer + 1], ACTOR_DIMS[layer]) or bias != (ACTOR_DIMS[layer + 1],):
            raise ValueError(f"Layer {layer}: weight {weight}, bias {bias} do not match {ACTOR_DIMS}.")


@functools.cache
def _policy_module_class(torch: Any) -> type:
    """The exported module: ``actor(normalizer(x))``, the layout of Isaac Lab's policy exporter.

    Created lazily (torch is imported only for exporting) and cached, so repeated exports in one
    process reuse the same class and TorchScript does not rename it.
    """

    class RoughGo2Policy(torch.nn.Module):
        """Deterministic actor (mean action) of the rough Go2 policy; input 235, output 12."""

        def __init__(self) -> None:
            super().__init__()
            layers = []
            for index in range(len(ACTOR_DIMS) - 1):
                layers.append(torch.nn.Linear(ACTOR_DIMS[index], ACTOR_DIMS[index + 1]))
                if index < len(ACTOR_DIMS) - 2:
                    layers.append(torch.nn.ELU())
            self.actor = torch.nn.Sequential(*layers)
            self.normalizer = torch.nn.Identity()  # the checkpoint has no observation normalizer

        def forward(self, x: torch.Tensor) -> torch.Tensor:
            return self.actor(self.normalizer(x))

    return RoughGo2Policy


def build_actor(actor_state_dict: Mapping[str, Any]) -> Any:
    """Rebuild the actor as an eager ``torch.nn.Module`` from the checkpoint's state dict."""
    torch = _import_torch()
    check_actor_state_dict(actor_state_dict)
    module = _policy_module_class(torch)()
    module.actor.load_state_dict({key[len("mlp.") :]: value for key, value in actor_state_dict.items() if key.startswith("mlp.")})
    return module.eval()


def reference_actor_forward(actor_state_dict: Mapping[str, Any], observations: Any) -> Any:
    """Independent re-implementation of the actor (``F.linear`` + ``F.elu``) used to check the export."""
    torch = _import_torch()
    x = observations
    layers = len(ACTOR_DIMS) - 1
    for layer in range(layers):
        x = torch.nn.functional.linear(x, actor_state_dict[f"mlp.{2 * layer}.weight"], actor_state_dict[f"mlp.{2 * layer}.bias"])
        if layer < layers - 1:
            x = torch.nn.functional.elu(x)
    return x


def export_torchscript(out_path: str | Path = MODEL_PATH, checkpoint_url: str | None = None, samples: int = 4096) -> dict:
    """Export the checkpoint's actor to TorchScript and verify it (run once, inside Isaac Sim).

    Steps: read + hash-check the checkpoint, check the actor structure, ``torch.jit.script`` the
    rebuilt actor, save it, reload the saved bytes and compare against
    :func:`reference_actor_forward` on random inputs (batched and single 1-D inputs, as the runner
    feeds them) and on the nominal standing observation.

    Args:
        out_path: Where to write policy.pt.
        checkpoint_url: Override checkpoint location (default: asset library).
        samples: Number of random observations for the comparison.

    Returns:
        Report dict: ``sha256``, ``bytes``, ``iteration``, ``max_abs_diff_batch``,
        ``max_abs_diff_single``, ``nominal_action``.
    """
    torch = _import_torch()
    checkpoint = load_checkpoint(checkpoint_url)
    actor_state_dict = checkpoint["actor_state_dict"]
    module = build_actor(actor_state_dict)
    scripted = torch.jit.script(module)
    buffer = io.BytesIO()
    torch.jit.save(scripted, buffer)
    data = buffer.getvalue()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(data)

    loaded = torch.jit.load(io.BytesIO(out_path.read_bytes()), map_location="cpu").eval()
    generator = torch.Generator().manual_seed(0)
    inputs = torch.randn(samples, ACTOR_DIMS[0], generator=generator) * 2.0
    with torch.no_grad():
        reference = reference_actor_forward(actor_state_dict, inputs)
        batch_diff = float((loaded(inputs) - reference).abs().max())
        # The runner feeds one 1-D observation per call; compare that path against a 1-D reference.
        single_diff = max(
            float((loaded(inputs[i]) - reference_actor_forward(actor_state_dict, inputs[i])).abs().max())
            for i in range(64)
        )
        nominal = torch.from_numpy(nominal_observation())
        nominal_action = loaded(nominal).numpy().astype(float).round(4).tolist()
    return {
        "path": out_path.as_posix(),
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
        "iteration": int(checkpoint.get("iter", -1)),
        "max_abs_diff_batch": batch_diff,
        "max_abs_diff_single": single_diff,
        "nominal_action": nominal_action,
        "torch": torch.__version__,
    }
