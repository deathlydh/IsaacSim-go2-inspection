# Gait A/B test (2026-10-14)

Flat grid scene `scenes/inspection.usd`, CPU PhysX 200 Hz, policy 50 Hz, camera off.
Program: `tools/isaac/probe_drift.py` (stand still) + `tools/isaac/measure_go2_gait.py`
(stand 3 s, 0.5 m/s 6 s, 1.0 m/s 6 s, stop 2 s). Raw logs: `*.txt` in this folder.

| Metric | bundled (NVIDIA flat) | robot_lab kp25 | robot_lab kp20 | nvidia_rough | **robot_lab kp25 + heading hold** |
|---|---|---|---|---|---|
| Base height standing | 0.23 m | 0.33 m | 0.32 m | 0.36 m | **0.31 m** |
| Base height at 0.5 / 1.0 m/s | 0.145 / 0.124 m | 0.316 / 0.329 m | 0.313 / 0.327 m | 0.318 (std 0.05, min 0.24) / 0.349 m | **0.319 / 0.330 m** |
| Foot clearance at 0.5 m/s | 0–1 mm (slides) | 1.2–6.3 cm | 1.1–6.4 cm | 1.4–10.7 cm, irregular | **1.6–7.8 cm** |
| Achieved speed for 0.5 / 1.0 | 0.44 / 0.96 | 0.50 / 1.06 | 0.48 / 1.05 | 0.35 / 0.96 | **0.54 / 1.07** |
| Step frequency at 1.0 m/s | 4.2 Hz | 3.6 Hz | 3.5 Hz | 2.4 Hz | 3.5 Hz |
| Yaw drift standing (zero command) | −1.0 °/s | +10.0 °/s | +12.5 °/s | −0.2 °/s | **+0.2 °/s** |
| Steps in place when standing | no | yes | yes | no | **no** |

Real Go2 for reference: body height ~0.30 m, trot ~2 Hz.

**Decision:** `robot_lab` (kp 25, the training value) with heading hold is the default in
`start_go2.py`. Heading hold steers the yaw-rate command toward the remembered heading while no
turn key is pressed (like Isaac Lab's heading command); without it the policy spins ~10 °/s.
`nvidia_rough` stands best and has the most natural step frequency at 1 m/s but stumbles at the
0.5 m/s inspection speed; kept as an option (`policy=nvidia_rough`).
