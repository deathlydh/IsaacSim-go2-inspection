"""Gait option "C": the robot_lab Go2 velocity policy from rl_sar, run by Isaac Sim's RobotPolicyRunner.

WHAT
    ``build_runner()`` returns an ``isaacsim.robot.policy.examples.RobotPolicyRunner`` (not yet
    spawned) that drives the same Isaac Lab ``go2.usd`` as the bundled NVIDIA example with the
    robot_lab policy ``assets/third_party/rl_sar_go2/policy.pt``. It is a drop-in replacement for
    ``RobotPolicyRunner(get_go2_spec(), ...)`` in ``start_go2.py``: same ``spawn()``, ``step(dt,
    command)``, ``restart_from_default_state(command)`` API and the same ``[vx, vy, wz]`` command.

WHY A HAND-WRITTEN BINDING
    NVIDIA's bundled policies ship an "IO descriptor" file from which the runner derives the
    observation/action layout automatically. rl_sar ships only its own deployment config
    (``config.yaml`` + ``base.yaml``), so this module builds the layout itself and hands it to the
    runner through ``PolicySpec(binding=...)`` -- the same escape hatch NVIDIA uses for its Franka
    example (``bundled/franka.py``). The layout is read from the two yaml files, not hard-coded.

POLICY INTERFACE (from config.yaml/base.yaml; network shape checked in the TorchScript graph)
    Observation: 45 floats, no history (``observations_history: []``)::

        [ 0: 3] base angular velocity, body frame, * 0.25   rl_sar "ang_vel"
        [ 3: 6] gravity direction, body frame ((0,0,-1) when level)   "gravity_vec"
        [ 6: 9] command [vx m/s, vy m/s, wz rad/s] * 1.0             "commands"
        [ 9:21] joint position - default pose, * 1.0                 "dof_pos"
        [21:33] joint velocity * 0.05                                "dof_vel"
        [33:45] previous raw network output                          "actions"
        whole vector clipped to +-100 (``clip_obs``; never reached in practice)

    Action: 12 raw network outputs ``a`` -> joint position targets::

        target = default_pos + action_scale * a       (a clipped to +-100 first, never reached)
        default_pos  = [0.0, 0.8, -1.5] per leg
        action_scale = [0.125, 0.25, 0.25] per leg (hip, thigh, calf)

    Joint order of the 12-wide blocks: per LEG -- FR, FL, RR, RL, each (hip, thigh, calf), i.e.
    ``base.yaml`` ``joint_names`` through the identity ``joint_mapping``. The Go2 USD orders its
    DOFs by TYPE instead (FL/FR/RL/RR hips, then thighs, then calves). Terms bind joints by NAME,
    so the runner resolves that permutation itself; ``policy_to_usd_permutation()`` shows it
    (policy slot i -> USD DOF index [1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10][i]).
    Offline check of this order: tilting the body in the observation makes the network extend the
    downhill legs and flex the uphill legs ONLY when the outputs are read as FR, FL, RR, RL.

    Timing: physics dt 0.005 s (200 Hz), one policy step every 4 physics steps (50 Hz).
    Actuators (default ``env_isaacsim_kp25.yaml``): explicit DC-motor model, kp 25, kd 0.5,
    23.5 N*m, 30 rad/s -- robot_lab's TRAINING values (``UNITREE_GO2_CFG`` in robot_lab
    ``source/robot_lab/robot_lab/assets/unitree.py``, identical in v2.1, v2.2.1 and main; read on
    GitHub, not in the downloaded files). rl_sar deploys the same network with kp 20 (its
    ``rl_kp``, a sim-to-real choice); ``kp=20.0`` selects ``env_isaacsim.yaml`` for comparison.

USAGE (inside Isaac Sim, e.g. in a copy of start_go2.py)::

    import sys
    sys.path.insert(0, "D:/go2-inspection/tools/isaac")
    from policies import robot_lab_go2

    runner = robot_lab_go2.build_runner("/World/Robot", position=(0.0, 0.0, 0.45))
    runner.spawn()                    # before Play, exactly like the bundled runner. NOTE: spawn()
                                      # reuses a prim that already exists at prim_path, so close the
                                      # old session and remove the old robot (or reopen the scene) first
    runner.apply_scene_properties()   # optional: foot friction 0.8/0.6 as at training
    keymap = make_velocity_keyboard_mapping(robot_lab_go2.KEYBOARD_SCALE)
    # in the physics callback:
    runner.step(dt, robot_lab_go2.clip_command(command).tolist())

Every number that is NOT in the downloaded files is marked in the yaml: "[robot_lab]" when it was
read from robot_lab's training config on GitHub, "[assumed]" otherwise.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from isaacsim.robot.policy.examples import PolicyArtifact, PolicySpec, RobotPolicyRunner, get_go2_spec
from isaacsim.robot.policy.examples.binding import PolicyBinding, Term

__all__ = [
    "COMMAND_LIMITS",
    "KEYBOARD_SCALE",
    "MODEL_SHA256",
    "RobotLabConfig",
    "build_runner",
    "clip_command",
    "get_robot_lab_go2_spec",
    "load_config",
    "make_binding",
    "nominal_observation",
    "observation_layout",
    "policy_to_usd_permutation",
    "robot_lab_binding",
]

# ---------------------------------------------------------------------------------------------
# Files (downloaded from rl_sar, see assets/third_party/rl_sar_go2/SOURCE.txt) and constants
# ---------------------------------------------------------------------------------------------

#: tools/isaac/policies/robot_lab_go2.py -> project root is three folders up.
ASSET_DIR = Path(__file__).resolve().parents[3] / "assets" / "third_party" / "rl_sar_go2"
MODEL_PATH = ASSET_DIR / "policy.pt"
CONFIG_PATH = ASSET_DIR / "config.yaml"  # rl_sar policy/go2/robot_lab/config.yaml
BASE_CONFIG_PATH = ASSET_DIR / "base.yaml"  # rl_sar policy/go2/base.yaml

#: Env configs for the runner, keyed by actuator stiffness (kp). 25 = robot_lab's training value
#: (DCMotor stiffness of its UNITREE_GO2_CFG; the default, because the simulation should match
#: training); 20 = rl_sar's deployment rl_kp, kept for an A/B comparison.
ENV_YAML_BY_KP = {
    20.0: ASSET_DIR / "env_isaacsim.yaml",
    25.0: ASSET_DIR / "env_isaacsim_kp25.yaml",
}

#: SHA-256 of policy.pt as downloaded (rl_sar commit 376d42c9, 2026-08-16). The runner refuses to
#: deserialize a TorchScript file whose bytes do not match (TorchScript loading can run code).
MODEL_SHA256 = "9f14cb95e74ac9e5e30954da0fc0c33eaa41e337852ebed19fcee869279ade0b"

#: Section names inside the rl_sar yaml files.
_POLICY_SECTION = "go2/robot_lab"
_ROBOT_SECTION = "go2"

#: Largest |command| per axis [vx m/s, vy m/s, wz rad/s] to send to the policy.
#: robot_lab's training command ranges are not in the downloaded config.yaml (rl_sar keeps only
#: deployment settings); +-1.0 on every axis is what robot_lab's velocity_env_cfg.py CommandsCfg
#: declares (the Go2 override lines are commented out there; read on GitHub, main branch).
#: Commands outside the training range drive the policy into states it never saw.
COMMAND_LIMITS = (1.0, 1.0, 1.0)

#: Command step per arrow/N/M key for make_velocity_keyboard_mapping(). Half of COMMAND_LIMITS:
#: 0.5 m/s is the inspection walking speed recommended in docs/PLAN_factory_camera_gait.md (3.1),
#: and two keys pressed together (forward + strafe) still stay inside the training range.
KEYBOARD_SCALE = 0.5


# ---------------------------------------------------------------------------------------------
# rl_sar config -> typed values
# ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class RobotLabConfig:
    """The deployment settings of the robot_lab Go2 policy, as rl_sar uses them.

    All per-joint tuples are in POLICY order (see ``joint_names``).

    Args:
        joint_names: USD/URDF joint names in the order the network reads and writes them.
        observations: Ordered rl_sar observation term names (e.g. ``"ang_vel"``).
        num_observations: Width of the network input declared by the config.
        default_dof_pos: Default joint pose: observation offset, action offset and reset pose.
        action_scale: Per-joint multiplier from raw network output to radians.
        clip_obs: Symmetric clip of the (scaled) observation vector.
        clip_actions_lower: Per-joint lower clip of the raw network output.
        clip_actions_upper: Per-joint upper clip of the raw network output.
        lin_vel_scale: Scale of the "lin_vel" term (unused by this policy, kept for completeness).
        ang_vel_scale: Scale of the "ang_vel" term.
        dof_pos_scale: Scale of the "dof_pos" term.
        dof_vel_scale: Scale of the "dof_vel" term.
        commands_scale: Per-axis scale of the "commands" term.
        rl_kp: Per-joint PD stiffness rl_sar uses while the policy runs.
        rl_kd: Per-joint PD damping rl_sar uses while the policy runs.
        torque_limits: Per-joint torque limit in N*m.
        dt: Physics step in seconds.
        decimation: Physics steps per policy step.
    """

    joint_names: tuple[str, ...]
    observations: tuple[str, ...]
    num_observations: int
    default_dof_pos: tuple[float, ...]
    action_scale: tuple[float, ...]
    clip_obs: float
    clip_actions_lower: tuple[float, ...]
    clip_actions_upper: tuple[float, ...]
    lin_vel_scale: float
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    commands_scale: tuple[float, ...]
    rl_kp: tuple[float, ...]
    rl_kd: tuple[float, ...]
    torque_limits: tuple[float, ...]
    dt: float
    decimation: int


#: rl_sar observation name -> (Isaac Sim binding semantic, width). Widths of joint terms = DOF count.
_TERM_SEMANTICS = {
    "lin_vel": ("base_lin_vel", 3),
    "ang_vel": ("base_ang_vel", 3),
    "gravity_vec": ("projected_gravity", 3),
    "commands": ("generated_commands", 3),
    "dof_pos": ("joint_pos_rel", None),
    "dof_vel": ("joint_vel_rel", None),
    "actions": ("last_action", None),
}


def load_config(config_path: str | Path = CONFIG_PATH, base_path: str | Path = BASE_CONFIG_PATH) -> RobotLabConfig:
    """Read rl_sar's ``config.yaml`` (policy section) on top of ``base.yaml`` (robot section).

    rl_sar loads the robot file first and lets the policy file override it; the same is done here.
    The joint order follows rl_sar's convention: policy slot ``i`` is robot joint
    ``joint_names[joint_mapping[i]]`` (identity mapping for this policy).

    Args:
        config_path: rl_sar ``policy/go2/robot_lab/config.yaml``.
        base_path: rl_sar ``policy/go2/base.yaml``.

    Returns:
        The parsed and validated :class:`RobotLabConfig`.

    Raises:
        ValueError: If the files are inconsistent (lengths, widths, unknown terms).
        NotImplementedError: If the config asks for observation history (not used by this policy).
    """
    base = yaml.safe_load(Path(base_path).read_text(encoding="utf-8"))[_ROBOT_SECTION]
    policy = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))[_POLICY_SECTION]
    merged = {**base, **policy}

    dofs = int(merged["num_of_dofs"])
    robot_joint_names = list(base["joint_names"])
    mapping = [int(index) for index in merged["joint_mapping"]]
    if sorted(mapping) != list(range(dofs)) or len(robot_joint_names) != dofs:
        raise ValueError(f"joint_mapping {mapping} is not a permutation of {dofs} joints {robot_joint_names}.")
    joint_names = tuple(robot_joint_names[index] for index in mapping)

    if merged.get("observations_history"):
        raise NotImplementedError(
            f"observations_history {merged['observations_history']} is set; this module only implements "
            "single-frame observations (robot_lab Go2 uses none)."
        )
    observations = tuple(merged["observations"])
    width = 0
    for name in observations:
        if name not in _TERM_SEMANTICS:
            raise ValueError(f"Unsupported rl_sar observation term {name!r}; known: {sorted(_TERM_SEMANTICS)}.")
        width += _TERM_SEMANTICS[name][1] or dofs
    if width != int(merged["num_observations"]):
        raise ValueError(f"Observation terms {observations} add up to {width}, config says {merged['num_observations']}.")

    def per_joint(key: str) -> tuple[float, ...]:
        values = tuple(float(value) for value in merged[key])
        if len(values) != dofs:
            raise ValueError(f"{key} has {len(values)} entries, expected {dofs}.")
        return values

    config = RobotLabConfig(
        joint_names=joint_names,
        observations=observations,
        num_observations=width,
        default_dof_pos=per_joint("default_dof_pos"),
        action_scale=per_joint("action_scale"),
        clip_obs=float(merged["clip_obs"]),
        clip_actions_lower=per_joint("clip_actions_lower"),
        clip_actions_upper=per_joint("clip_actions_upper"),
        lin_vel_scale=float(merged["lin_vel_scale"]),
        ang_vel_scale=float(merged["ang_vel_scale"]),
        dof_pos_scale=float(merged["dof_pos_scale"]),
        dof_vel_scale=float(merged["dof_vel_scale"]),
        commands_scale=tuple(float(value) for value in merged["commands_scale"]),
        rl_kp=per_joint("rl_kp"),
        rl_kd=per_joint("rl_kd"),
        torque_limits=per_joint("torque_limits"),
        dt=float(merged["dt"]),
        decimation=int(merged["decimation"]),
    )
    if any(scale <= 0.0 for scale in config.action_scale):
        raise ValueError("action_scale must be positive (the action clip conversion relies on it).")
    return config


# ---------------------------------------------------------------------------------------------
# Binding: rl_sar observation/action formulas expressed as Isaac Sim binding terms
# ---------------------------------------------------------------------------------------------


def _uniform(scale: float | Sequence[float]) -> float:
    """Return a scalar scale (the binding clips with one scalar pair per observation term)."""
    values = np.atleast_1d(np.asarray(scale, dtype=np.float64))
    if not np.allclose(values, values[0]):
        raise NotImplementedError(f"Per-element observation scale {values.tolist()} with clipping is not supported.")
    return float(values[0])


def make_binding(config: RobotLabConfig) -> PolicyBinding:
    """Translate the rl_sar formulas into an ordered Isaac Sim :class:`PolicyBinding`.

    rl_sar computes ``obs = clip(concat(term * scale), -clip_obs, clip_obs)``; the Isaac Sim binding
    computes ``clip(term, lower, upper) * scale`` per term. Clipping the raw term at
    ``+-clip_obs / scale`` before scaling gives exactly the same numbers (scales are positive).

    rl_sar decodes ``target = clip(a, lo, hi) * scale + default``; the binding computes
    ``clip(a * scale + default, lower, upper)``. With ``lower = lo * scale + default`` (same for
    ``upper``) the two are identical. ``last_action`` is the raw, unclipped network output, as in
    Isaac Lab training (rl_sar feeds back the clipped one; with +-100 bounds they never differ).

    Args:
        config: Parsed rl_sar settings.

    Returns:
        The 45-wide observation / 12-wide action binding in policy joint order.
    """
    joints = config.joint_names
    dofs = len(joints)
    default = tuple(config.default_dof_pos)
    zeros = (0.0,) * dofs

    def clipped_term(semantic: str, width: int, scale: float | Sequence[float] | None, **kwargs) -> Term:
        scalar = 1.0 if scale is None else _uniform(scale)
        bound = config.clip_obs / scalar
        return Term(semantic=semantic, width=width, scale=scale, clip=(-bound, bound), **kwargs)

    builders = {
        "lin_vel": lambda: clipped_term("base_lin_vel", 3, config.lin_vel_scale),
        "ang_vel": lambda: clipped_term("base_ang_vel", 3, config.ang_vel_scale),
        "gravity_vec": lambda: clipped_term("projected_gravity", 3, None),
        "commands": lambda: clipped_term("generated_commands", 3, tuple(config.commands_scale)),
        "dof_pos": lambda: clipped_term(
            "joint_pos_rel", dofs, config.dof_pos_scale, joint_names=joints, offsets=default
        ),
        "dof_vel": lambda: clipped_term("joint_vel_rel", dofs, config.dof_vel_scale, joint_names=joints, offsets=zeros),
        "actions": lambda: clipped_term("last_action", dofs, None),
    }
    observation_terms = tuple(builders[name]() for name in config.observations)

    scale = np.asarray(config.action_scale, dtype=np.float64)
    lower = np.asarray(config.clip_actions_lower) * scale + np.asarray(default)
    upper = np.asarray(config.clip_actions_upper) * scale + np.asarray(default)
    action_term = Term(
        semantic="joint_position",
        width=dofs,
        joint_names=joints,
        offsets=default,
        scale=tuple(config.action_scale),
        clip=tuple((float(lo), float(hi)) for lo, hi in zip(lower, upper)),
    )
    return PolicyBinding(observation_terms=observation_terms, action_terms=(action_term,))


def robot_lab_binding(env_config) -> PolicyBinding:
    """PolicySpec binding hook: build the binding and cross-check it against the env yaml.

    The runner calls this once, at the first ``initialize()``/``restart_from_default_state()``,
    with the parsed env yaml. The env yaml configures the physics side (reset pose, timing); the
    policy side comes from rl_sar's files. If the two disagree the robot would be reset into one
    pose while the policy measures offsets from another, so that is refused here.

    Args:
        env_config: ``PolicyEnvConfig`` of the deployed artifact (``env_isaacsim*.yaml``).

    Returns:
        The policy binding from :func:`make_binding`.

    Raises:
        ValueError: If the env yaml default pose or timing differs from rl_sar's config.
    """
    config = load_config()
    properties = env_config.joint_properties(config.joint_names)
    if not np.allclose(properties.default_positions, config.default_dof_pos, atol=1e-6):
        raise ValueError(
            f"env yaml default pose {properties.default_positions} != rl_sar default_dof_pos "
            f"{config.default_dof_pos} (policy order {config.joint_names})."
        )
    timing = env_config.timing
    if abs(timing.physics_dt - config.dt) > 1e-9 or timing.decimation != config.decimation:
        raise ValueError(
            f"env yaml timing dt={timing.physics_dt}, decimation={timing.decimation} != rl_sar "
            f"dt={config.dt}, decimation={config.decimation}."
        )
    return make_binding(config)


# ---------------------------------------------------------------------------------------------
# Spec and runner
# ---------------------------------------------------------------------------------------------


def _env_yaml_for(kp: float) -> Path:
    """Return the env yaml for an actuator stiffness; only the shipped variants exist."""
    try:
        return ENV_YAML_BY_KP[float(kp)]
    except KeyError:
        raise ValueError(f"No env yaml for kp={kp}; available: {sorted(ENV_YAML_BY_KP)}.") from None


def get_robot_lab_go2_spec(kp: float = 25.0) -> PolicySpec:
    """Build the frozen :class:`PolicySpec` for the robot_lab Go2 policy (PhysX only).

    Args:
        kp: Actuator stiffness variant: 25.0 (robot_lab training value, default) or 20.0 (rl_sar's
            deployment value, A/B comparison).

    Returns:
        The policy spec: TorchScript model with pinned SHA-256, local env yaml, the bundled Go2
        USD, identity spawn orientation and the hand-written binding hook.
    """
    artifact = PolicyArtifact.from_files(
        MODEL_PATH.as_posix(),
        _env_yaml_for(kp).as_posix(),
        None,  # no IO descriptor: the interface comes from robot_lab_binding()
        model_sha256=MODEL_SHA256,
    )
    return PolicySpec(
        name=f"go2_robot_lab_kp{float(kp):g}",
        engines={"physx": artifact},
        usd_path=get_go2_spec().usd_path,  # exactly the asset of the bundled NVIDIA Go2 example
        default_spawn_orientation=(1.0, 0.0, 0.0, 0.0),
        binding=robot_lab_binding,
    )


def build_runner(
    prim_path: str = "/World/Robot",
    position: Sequence[float] = (0.0, 0.0, 0.45),
    *,
    kp: float = 25.0,
    orientation: Sequence[float] | None = None,
) -> RobotPolicyRunner:
    """Create (but do not spawn) the runner for the robot_lab Go2 policy.

    Nothing touches the stage here. The caller then follows the usual runner life cycle:
    ``spawn()`` before Play (references go2.usd at ``prim_path``), then from a physics callback
    ``restart_from_default_state(command)`` once and ``step(dt, command)`` on every physics step.

    Args:
        prim_path: Stage path of the robot.
        position: World spawn position in metres (the base settles from 0.45 m to ~0.3 m).
        kp: Actuator stiffness variant, 25.0 (robot_lab training value, default) or 20.0 (rl_sar).
        orientation: World spawn orientation (w, x, y, z), or None for the spec default (identity).

    Returns:
        The unspawned :class:`RobotPolicyRunner`.
    """
    return RobotPolicyRunner(
        get_robot_lab_go2_spec(kp),
        prim_path=prim_path,
        position=[float(value) for value in position],
        orientation=None if orientation is None else [float(value) for value in orientation],
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


def policy_to_usd_permutation(dof_names: Sequence[str], config: RobotLabConfig | None = None) -> np.ndarray:
    """Return ``perm`` with ``perm[i]`` = USD DOF index of policy joint ``i``.

    ``usd_vector[perm] = policy_vector`` converts policy order to USD order and
    ``policy_vector = usd_vector[perm]`` converts back. The runner does this internally by joint
    name; this function only makes the mapping visible (diagnostics, tests).

    Args:
        dof_names: Articulation DOF names in USD order (``runner.articulation.dof_names``).
        config: Parsed rl_sar settings, or None to load them.

    Returns:
        int64 array of 12 indices.
    """
    config = config or load_config()
    index_by_name = {str(name): index for index, name in enumerate(dof_names)}
    missing = [name for name in config.joint_names if name not in index_by_name]
    if missing:
        raise ValueError(f"Policy joints {missing} are not in the articulation {list(dof_names)}.")
    return np.array([index_by_name[name] for name in config.joint_names], dtype=np.int64)


def observation_layout(config: RobotLabConfig | None = None) -> list[tuple[str, int, int]]:
    """Return ``(rl_sar term name, start, stop)`` for each observation slice.

    Args:
        config: Parsed rl_sar settings, or None to load them.

    Returns:
        Ordered slices covering the whole observation vector.
    """
    config = config or load_config()
    layout, start = [], 0
    for name in config.observations:
        stop = start + (_TERM_SEMANTICS[name][1] or len(config.joint_names))
        layout.append((name, start, stop))
        start = stop
    return layout


def nominal_observation(config: RobotLabConfig | None = None, command: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """Observation of a robot standing still and level in its default pose (for offline checks).

    Args:
        config: Parsed rl_sar settings, or None to load them.
        command: ``[vx, vy, wz]`` command to place in the command slice.

    Returns:
        float32 vector of ``num_observations`` values.
    """
    config = config or load_config()
    observation = np.zeros(config.num_observations, dtype=np.float32)
    for name, start, stop in observation_layout(config):
        if name == "gravity_vec":
            observation[start:stop] = (0.0, 0.0, -1.0)
        elif name == "commands":
            observation[start:stop] = np.asarray(command, dtype=np.float32) * np.asarray(config.commands_scale)
    return observation
