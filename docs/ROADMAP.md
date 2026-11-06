# Roadmap — Go2 inspection case

> Актуальный рабочий план с 2026-11-05: [TODO.md](TODO.md).
> Этот документ сохранён как исторический план первоначального демонстратора.
> Задание для следующей модели: [IMPLEMENTATION_PROMPT.md](IMPLEMENTATION_PROMPT.md).

Status on 2026-10-15 (commit bf31482): Isaac Sim 6.1 on Windows 10 works; factory hall with 5
procedural gauges (`scenes/factory_hall.usda`, ground truth in `factory_hall_meta.json`); Go2 with
robot_lab gait + heading hold; front camera 1280x720 @ 10 Hz; `goto_gauge.py` walks to a gauge
waypoint and saves a camera frame (PI-101 readable, ~100 px dial at 1.2 m standoff).
One-click start: `start_factory.bat`. Control from Claude: python_server 127.0.0.1:8226 (see CLAUDE.md).

## Remaining blocks

| # | Block | Deliverable | Installs |
|---|---|---|---|
| 1 | Gauge dataset | `tools/isaac/collect_gauge_dataset.py`: randomize needle values (`make_gauge.set_gauge_value`), robot pose offsets (distance 0.8–1.6 m, yaw ±15°), light intensity; save frames + JSON labels (value, bbox/center from projecting `gauge_ground_truth()` with camera intrinsics) to `data/gauges/` | none |
| 2 | Gauge reading (CV) | `cv/gauge_reader.py`: detect dial (Hough circles / ellipse), find needle (line detection / dark-pixel ray voting), map angle -> value with per-gauge calibration from meta (vmin, vmax, sweep 270°); `cv/eval.py` with MAE per gauge, failure gallery | none (OpenCV/numpy inside Kit python, or system python if available) |
| 3 | Inspection mission | `tools/isaac/run_inspection.py`: route over the 5 waypoints (meta `inspection_route`), stop, capture, read, compare with red zone, write `reports/<timestamp>/report.json` + `report.html` with crops | none |
| 4 | ROS 2 bridge | camera image + CameraInfo, odom, TF, `/cmd_vel` subscriber driving `session.command` | ROS 2 Jazzy: native Windows (pixi; docs target Win11, Win10 unverified) **or** WSL2 Ubuntu 24.04 + DDS/Zenoh to Isaac on Windows — needs user decision |
| 5 | LiDAR + map | RTX lidar on Go2 (L1-like), occupancy map of the hall (`isaacsim.asset.gen.omap` / skill `occupancy-map`) | none for map; ROS for /scan |
| 6 | Nav2 | autonomous navigation to inspection waypoints with obstacle avoidance | ROS 2 + Nav2 (WSL2 recommended) |
| 7 | Demo | screen recording of a full mission + final report | none |
| 8 | Optional | own gait training in Isaac Lab (base height/gait rewards), stairs/rough terrain | Isaac Lab (multi-GB) |

## Known pitfalls (also in CLAUDE.md)
- Never launch Kit from Claude; the user starts it from Explorer.
- Stop + ~10 frames before new/open stage (virtual_gantry crash).
- Never pause the camera render / destroy its render product with annotators attached (CUDA error 700).
- Long scripts: generous `--timeout` (sim runs ~30 FPS).
- The policy ignores very small speed commands: waypoint accuracy ~10–15 cm.
