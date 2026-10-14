# NVIDIA Isaac Lab rough-terrain Go2 policy (gait option "B")

**Кратко (RU).** Нейросеть походки Go2 от NVIDIA, обученная в Isaac Lab на задаче
`Isaac-Velocity-Rough-UnitreeGo2` (лестницы, ящики, склоны). Взята из библиотеки ассетов
Isaac Sim 6.1 (сырой чекпоинт rsl_rl) и переведена в TorchScript, который понимает
`RobotPolicyRunner`. Кроме обычных 48 входов ей нужна «карта высот» из 187 точек вокруг
корпуса; её считает `tools/isaac/policies/nvidia_rough_go2.py` (лучами PhysX или по формуле
для ровного пола). Запуск: `nvidia_rough_go2.build_runner(...)`, см. docstring модуля.

## Files

| File | Bytes | SHA-256 | What |
|---|---|---|---|
| `policy.pt` | 1 158 457 | `dcb7e8ab29a65625b03ecd73f43b8bb540533b658c4fd1b7a638342ea25f6d8f` | TorchScript actor (input 235, output 12) |
| `env.yaml` | 29 501 | `989aac8a52067cabcfeaf00d8a7b25c465f72247876abcba2ba5c3e68f2cb961` | Isaac Sim `PolicyEnvConfig` (header + bundled yaml) |

`MODEL_SHA256` in `tools/isaac/policies/nvidia_rough_go2.py` pins `policy.pt`. The runner
refuses to load a TorchScript file with other bytes.

## Provenance

**Checkpoint (source of `policy.pt`)**

- Path: `<assets_root>/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-Rough-UnitreeGo2_physx_none_rsl_rl.pt`.
  Here `<assets_root>` is `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1`.
- Size and hash: 6 882 293 bytes, SHA-256 `f15732c603c581c623edd7c0227b4f41b1d1916a15a748f96d8d4de071fb9570`.
- Server timestamp: 2026-08-14 07:15:23.
- Read inside Isaac Sim with `omni.client.read_file` on 2026-10-14. Only the derived `policy.pt` is stored here.
- Contents, loaded with `torch.load(weights_only=True)`, which executes no code:
  - `actor_state_dict`, `critic_state_dict`, `optimizer_state_dict`, `iter` = 1499, `infos` = None.
  - The actor has `mlp.{0,2,4,6}.weight/bias`. The layer shapes 235→512→256→128→12 confirm the hidden sizes; ELU is the rsl_rl default.
  - The actor also has `distribution.std_param`, which is exploration noise and unused here.
  - There are no `obs_normalizer.*` keys, so the observations are not normalized.

**`env.yaml`**

- Contains a provenance header followed by a byte-for-byte copy of `<assets_root>/Isaac/Samples/Policies/go2/physx_env.yaml`.
- Source file: 27 280 bytes, SHA-256 `2ba72cb8c9242442a8ac2cd5f7e6f90b25323882a99466a32e0f43fd36d0c90c`, server timestamp 2026-08-31.
- NVIDIA ships this yaml next to the *flat* policy, but it is an export of the Go2 *rough* task. Evidence:
  - It has a `height_scanner` on `Robot/base` and a `height_scan` observation clipped to (-1, 1).
  - It has a terrain generator with stairs, boxes and slopes, and the `terrain_levels` curriculum.
  - Its reward weights match `UnitreeGo2RoughEnvCfg`: `feet_air_time` 0.01 and `flat_orientation_l2` 0.
- It is kept unchanged as this checkpoint's training configuration.

**Export**

- `nvidia_rough_go2.export_torchscript()` was run inside Isaac Sim 6.1 with torch 2.11.0+cu128 on 2026-10-14. It:
  1. Checks the checkpoint hash and the actor structure.
  2. Rebuilds the actor as `actor(normalizer(x))` with an identity normalizer, which is the layout of Isaac Lab's exporter.
  3. Runs `torch.jit.script`, saves the file, reloads it, and compares it with an independent `F.linear` + `F.elu` implementation.
- A re-export gives the same network but usually not the same bytes. TorchScript embeds class names that depend on the session's compile history, and it also embeds source line info. After a re-export, update `MODEL_SHA256` and this table.

## License

From the Isaac Lab checkpoints *Model Card v1.1*, which sits next to the checkpoint and was read on 2026-10-14:

- "These checkpoints are ready for commercial or non-commercial use."
- License: Isaac Lab's BSD-3-Clause, with certain components under Apache-2.0 (https://github.com/isaac-sim/IsaacLab/blob/main/LICENSE).
- The card states the checkpoints are meant to be used only with the designated robots in the provided simulation environments, and must be paired with the env/agent configuration used in training.

`env.yaml` is NVIDIA Isaac Sim sample data. The robot model `go2.usd` is not copied here; it is referenced from the asset library.

## Policy interface

| Obs slice | Term | Notes |
|---|---|---|
| 0:3 | base linear velocity, body frame | m/s |
| 3:6 | base angular velocity, body frame | rad/s |
| 6:9 | projected gravity | (0,0,-1) when level |
| 9:12 | command vx, vy, wz | training range ±1 on each axis |
| 12:24 | joint pos − default | default hip ±0.1, front thigh 0.8, rear thigh 1.0, calf −1.5 |
| 24:36 | joint velocity | |
| 36:48 | previous raw action | |
| 48:235 | height scan, 17 (x) × 11 (y) rays, 0.1 m | `clip(base_z − ground_z − 0.5, −1, 1)` |

- The interface uses no observation scaling and no history. Training added noise, which is not reproduced at deployment.
- Joint order is the Isaac Lab PhysX order, which matches the go2.usd DOF order: FL/FR/RL/RR hips, then thighs, then calves.
- The height-scan rays are aligned with the base yaw only (roll and pitch are ignored). x is the fastest-changing index, as in Isaac Lab `grid_pattern(ordering="xy")`. Rays cover x ∈ [−0.8, 0.8] m forward and y ∈ [−0.5, 0.5] m left.
- Action: `target = default + 0.25·a`.
- Timing: physics at 200 Hz, policy at 50 Hz (decimation 4).

## Actuator limits

The plan proposed per-joint limits of 23.7 / 23.7 / 45.43 N·m and 30.1 / 30.1 / 15.7 rad/s. **This yaml does not use them.** It keeps the exported values: DCMotor, effort 23.5 N·m, saturation 23.5, velocity 30.0 rad/s, kp 25, kd 0.5. The reasons:

- In Isaac Lab, `UNITREE_GO2_CFG` used 23.5 / 30.0 for all joints until PR #7564, "Fix Unitree Go1 and Go2 calf actuator limits ignoring the knee reduction".
  - That PR was merged on 2026-09-04 and backported into v3.0.0-EA.
  - It introduced the per-joint values: `actuator_effort_limit` = {hip/thigh 23.7, calf 45.43}, `actuator_velocity_limit` = {30.1, 30.1, 15.70}.
  - It states that existing checkpoints were not retrained and that policies trained on the old configuration should be retrained.
- The checkpoint is dated 2026-08-14, before that change, so it was trained with 23.5 / 30.0.
- Both bundled Go2 env exports (`physx_env.yaml` and the newer-format `newton_env.yaml`) also say 23.5 / 30.0.
- The physics-engine limits are not set by the yaml, which has no `*_sim` keys. They stay at the values authored in go2.usd: 23.7 / 45.43 N·m and 30.1 / 15.7 rad/s, read from the live robot. The DC-motor model clamps torque to 23.5 before the engine sees it.

## Verification (2026-10-14, inside the running Isaac Sim 6.1; nothing written to the stage)

- **TorchScript vs. raw actor.** On 500 random inputs, loaded through the runner's own `TorchScriptPolicyModel` (1-D input), the max |diff| is 0.0. In the export check it is 0.0 on a batch of 4096 and ≤ 1.2e-5 on single inputs.
- **Binding.** On synthetic states, both with the USD joint order and with a permuted per-leg order:
  - the 235 observations match a hand computation (max |diff| 1.2e-7);
  - the actions decode to `default + 0.25·a` by joint name.
- **First 48 observations and the action term.** They are identical to `derive_binding()` of the bundled flat Go2 IO descriptor.
- **Ray pattern.** It is identical to Isaac Lab `grid_pattern` (torch reimplementation).
- **Height-scan provider on the live robot (read-only).**
  - Raycast and flat modes agree on the flat floor (floor at z = 0, `/World/ground/GroundPlane/CollisionPlane`).
  - The robot's own body is filtered out. Without the filter, 7 rays would hit it.
  - Cost per control tick: about 1.3 ms median when called from a script, and 1.8 ms median (max 3.8 ms) inside a POST_PHYSICS_STEP callback. Flat mode costs 0.13 ms.
- **Nominal standing observation.** Standing level in the default pose at a base height of 0.333 m, with zero command:
  - The first action reaches |a| ≤ 1.10, i.e. targets up to 0.27 rad from the default pose. The calves straighten by about 0.23–0.27 rad.
  - The bundled flat policy gives |a| ≤ 0.97 on the same input.
  - Changing the height scan by ±3 cm changes the actions by ≤ 0.11 (0.03 rad).
- **Not tested.** Walking in closed loop: no second robot could be spawned on the shared stage. Behaviour on real obstacles: the live stage has no colliders besides the ground plane.
