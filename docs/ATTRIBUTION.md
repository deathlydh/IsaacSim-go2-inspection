# Attribution and licenses

Third-party content used by this project. NVIDIA assets are **referenced by URL** (not copied or
modified) from the Isaac Sim 6.1 asset library:
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1`.

| What | Source | License | Used as |
|---|---|---|---|
| Warehouse hall | `/Isaac/Environments/Simple_Warehouse/warehouse.usd`, `Props/SM_FuseBox_01.usd` | CC BY 4.0 file in the folder; NVIDIA documents conflict (environment supplement "Limited Use", Isaac Sim Additional Software and Materials License, Omniverse License Agreement) | Reference in `scenes/factory_hall.usda` |
| Pipes, valve | `/Isaac/Environments/Office/Props/SM_PipeBig_3m.usd`, `SM_PipeSmall_3m.usd`, `SM_ValveBig.usd` | as above | Reference |
| Tanks, drums, pipe bundle, rail guards | `/Isaac/SimReady/Industrial/Warehouse/...` (Ibc_Tank_Steel_Stainless_A01, Ibc_Tank_A02, Drum_Steel_A01/A02, Pipe_Plastic_Gray_B08, Rail_Guard_A09) | CC BY 4.0 per folder (see above) | Reference, physics overrides in our layer |
| Go2 robot model | `/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd` | Isaac Lab / NVIDIA | Reference |
| Go2 flat policy (bundled example) | `/Isaac/Samples/Policies/go2/` | NVIDIA Isaac Sim sample | Loaded at runtime |
| Go2 rough-terrain policy | `/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-Rough-UnitreeGo2_physx_none_rsl_rl.pt` → TorchScript actor in `assets/policies/nvidia_rough_go2/` | Isaac Lab checkpoints Model Card v1.1: Isaac Lab BSD-3-Clause (some components Apache-2.0), commercial and non-commercial use, simulation only. "Isaac Lab checkpoints, NVIDIA / The Isaac Lab Project Developers" | Copied (derived TorchScript) |
| Go2 robot_lab policy | [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) commit `376d42c9`: `policy/go2/robot_lab/policy.pt`, `config.yaml`, `policy/go2/base.yaml` in `assets/third_party/rl_sar_go2/` | Apache-2.0, © 2024-2025 Ziqi Fan. "Go2 locomotion policy (robot_lab) and deployment config from rl_sar by Ziqi Fan" | Copied unmodified, LICENSE alongside |
| Front camera mount offset | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) `go2_description.urdf` | BSD-3-Clause | Numbers only |
| Front camera focal length | go2_ros2_sdk `calibration/front_camera_720.yaml` (fx = 864.4 px) | — | Number only |
| Gauge dial text | Windows Arial font | Licensed with Windows | Rendered into our own PNGs |

Our own: all scripts in `tools/`, gauge geometry and dial PNGs in `scenes/textures/gauges/`,
env yaml files for the policies (derived from the NVIDIA Go2 `physx_env.yaml` format).
Non-commercial (CC BY-NC) assets are not used.
