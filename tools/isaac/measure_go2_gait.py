"""Measure the live Go2 gait: stand 3 s, walk 0.5 m/s 6 s, walk 1.0 m/s 6 s, stop 2 s.

Runs inside the already-running Isaac Sim via isaacsim_send.py. Samples once per app update
(50 Hz render = 4 physics steps of 5 ms). Always restores session.command to zeros.

Optional injected args: out_path (npz with the raw samples).
"""
import sys
import math
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import PhysxSchema

if "out_path" not in dir():
    out_path = None

app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
s = getattr(sys, "_go2_inspection_session", None)
assert s is not None and s.runner is not None, "no live Go2 session (run start_go2.py first)"


def npy(x):
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    if hasattr(x, "cpu"):
        return x.cpu().numpy()
    return np.asarray(x)


def quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# --- context: physx scene settings vs training (TGS, stabilization on, bounce 0.5, ...) ---
stage = omni.usd.get_context().get_stage()
ps = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/PhysicsScene"))
print("physx scene: solver", ps.GetSolverTypeAttr().Get(), "| steps/s", ps.GetTimeStepsPerSecondAttr().Get(),
      "| stabilization", ps.GetEnableStabilizationAttr().Get(), "| bounceThr", ps.GetBounceThresholdAttr().Get(),
      "| frictionOffsetThr", ps.GetFrictionOffsetThresholdAttr().Get(),
      "| frictionCorrDist", ps.GetFrictionCorrelationDistanceAttr().Get())

if not tl.is_playing():
    print("timeline was not playing -> play() and wait ~2 s")
    tl.play()
    for _ in range(100):
        await app.next_update_async()

a = s.runner.articulation
dof_names = list(a.dof_names)
link_names = list(a.link_names)
foot_names = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
foot_idx = [link_names.index(n) for n in foot_names]
view = a._physics_articulation_view
print("ready:", s.ready, "| failed:", s.failed, "| dofs:", dof_names)

PHASES = [  # (name, command, app updates at 50 Hz)
    ("stand", (0.0, 0.0, 0.0), 150),
    ("walk_0.5", (0.5, 0.0, 0.0), 300),
    ("walk_1.0", (1.0, 0.0, 0.0), 300),
    ("stop", (0.0, 0.0, 0.0), 100),
]

rows = {k: [] for k in ("t", "phase", "pos", "quat", "lv", "av", "q", "qt", "tau", "feet")}
fell = None
effort_ok = True
try:
    for pi, (name, cmd, n_upd) in enumerate(PHASES):
        s.command[:] = np.asarray(cmd, dtype=np.float32)
        for _ in range(n_upd):
            await app.next_update_async()
            if s.failed or not s.ready:
                fell = f"session failed={s.failed} ready={s.ready} in phase {name}"
                break
            p, qw = a.get_world_poses()
            lv, av = a.get_velocities()
            rows["t"].append(tl.get_current_time())
            rows["phase"].append(pi)
            rows["pos"].append(npy(p)[0].copy())
            rows["quat"].append(npy(qw)[0].copy())
            rows["lv"].append(npy(lv)[0].copy())
            rows["av"].append(npy(av)[0].copy())
            rows["q"].append(npy(a.get_dof_positions())[0].copy())
            rows["qt"].append(npy(a.get_dof_position_targets())[0].copy())
            if effort_ok:
                try:
                    rows["tau"].append(npy(a.get_dof_efforts())[0].copy())
                except Exception as e:
                    effort_ok = False
                    print("efforts not readable:", e)
            lt = npy(view.get_link_transforms())
            rows["feet"].append(lt.reshape(-1, len(link_names), 7)[0, foot_idx, :3].copy())
            R = quat_wxyz_to_R(rows["quat"][-1])
            if rows["pos"][-1][2] < 0.10 or R[2, 2] < 0.5:  # base < 10 cm or tilt > 60 deg
                fell = f"fall detected in phase {name}: z={rows['pos'][-1][2]:.3f}, R22={R[2,2]:.2f}"
                break
        if fell:
            break
finally:
    s.command[:] = 0.0
    print("command reset to", s.command.tolist())

if fell:
    print("!!!", fell)

D = {k: np.asarray(v) for k, v in rows.items() if len(v)}
if out_path:
    np.savez(out_path, dof_names=np.array(dof_names), **D)
    print("saved raw samples:", out_path)

t = D["t"]
dt_med = float(np.median(np.diff(t))) if len(t) > 1 else float("nan")
print(f"samples: {len(t)} | median sim dt per update: {dt_med:.4f} s")

FL = [dof_names.index(n) for n in ("FL_hip_joint", "FL_thigh_joint", "FL_calf_joint")]
calves = [dof_names.index(f"{l}_calf_joint") for l in ("FL", "FR", "RL", "RR")]
thighs = [dof_names.index(f"{l}_thigh_joint") for l in ("FL", "FR", "RL", "RR")]
hips = [dof_names.index(f"{l}_hip_joint") for l in ("FL", "FR", "RL", "RR")]


def zero_cross_freq(x, T):
    x = x - x.mean()
    h = 0.15 * x.std()
    state, n = 0, 0
    for v in x:
        if v > h and state <= 0:
            n += state != 0
            state = 1
        elif v < -h and state >= 0:
            n += state != 0
            state = -1
    return n / 2.0 / T


def fft_peak(x, fs):
    x = (x - x.mean()) * np.hanning(len(x))
    nfft = 8 * len(x)
    spec = np.abs(np.fft.rfft(x, nfft))
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    m = (f > 0.5) & (f < 10)
    return float(f[m][np.argmax(spec[m])])


def analyse(pi, skip_s):
    m = D["phase"] == pi
    if m.sum() < 10:
        return None
    tt = t[m]
    keep = tt >= tt[0] + skip_s
    idx = np.where(m)[0][keep]
    tt = t[idx]
    T = tt[-1] - tt[0]
    fs = 1.0 / dt_med
    pos, quat, lv = D["pos"][idx], D["quat"][idx], D["lv"][idx]
    q, qt, feet = D["q"][idx], D["qt"][idx], D["feet"][idx]
    Rs = np.array([quat_wxyz_to_R(qq) for qq in quat])
    vb = np.einsum("nji,nj->ni", Rs, lv)  # body-frame linear velocity
    g_b = np.einsum("nji,j->ni", Rs, np.array([0, 0, -1.0]))
    roll = np.degrees(np.arctan2(g_b[:, 1], -g_b[:, 2]))
    pitch = np.degrees(np.arcsin(np.clip(g_b[:, 0], -1, 1)))
    disp = np.linalg.norm(pos[-1, :2] - pos[0, :2]) / T if T > 0 else float("nan")
    feet_b = np.einsum("nji,nkj->nki", Rs, feet - pos[:, None, :])  # feet in base frame
    fz = feet[:, :, 2]
    contact_z = np.percentile(fz, 5, axis=0)
    clearance = np.percentile(fz, 98, axis=0) - contact_z
    res = {
        "T": T, "base_z_mean": pos[:, 2].mean(), "base_z_std": pos[:, 2].std(), "base_z_min": pos[:, 2].min(),
        "vx_body_mean": vb[:, 0].mean(), "vy_body_mean": vb[:, 1].mean(),
        "speed_xy_mean": np.linalg.norm(lv[:, :2], axis=1).mean(), "disp_speed": disp,
        "yaw_rate_mean": D["av"][idx][:, 2].mean(),
        "roll_mean": roll.mean(), "pitch_mean": pitch.mean(),
        "q_mean": q.mean(0), "qt_mean": qt.mean(0), "q_min": q.min(0), "q_max": q.max(0),
        "foot_contact_z": contact_z, "foot_clearance": clearance,
        "feet_b_mean": feet_b.mean(0),
    }
    # step frequency from joint oscillation
    res["zc_FL_thigh"] = zero_cross_freq(q[:, FL[1]], T)
    res["zc_FL_calf"] = zero_cross_freq(q[:, FL[2]], T)
    res["fft_calves"] = [fft_peak(q[:, j], fs) for j in calves]
    res["fft_thighs"] = [fft_peak(q[:, j], fs) for j in thighs]
    # foot lift-offs (rising edges above contact + 1 cm), duty factor
    lifts, duty = [], []
    for k in range(4):
        up = fz[:, k] > contact_z[k] + 0.01
        lifts.append(int(np.sum(up[1:] & ~up[:-1])) / T)
        duty.append(float(np.mean(fz[:, k] < contact_z[k] + 0.005)))
    res["liftoff_hz"] = lifts
    res["duty"] = duty
    c = q[:, calves] - q[:, calves].mean(0)
    cc = np.corrcoef(c.T)
    res["corr_FL_RR"], res["corr_FR_RL"], res["corr_FL_FR"], res["corr_FL_RL"] = cc[0, 3], cc[1, 2], cc[0, 1], cc[0, 2]
    if "tau" in D and len(D["tau"]) == len(t):
        tau = D["tau"][idx]
        res["tau_absmean_hip_thigh_calf"] = [np.abs(tau[:, g]).mean() for g in (hips, thighs, calves)]
        res["tau_sat_frac"] = float(np.mean(np.abs(tau) >= 23.4))
        res["tau_absmax"] = float(np.abs(tau).max())
    return res


fmt = lambda v: np.round(np.asarray(v, dtype=float), 3).tolist()
for pi, (name, cmd, _) in enumerate(PHASES):
    skip = 1.5 if name == "stand" else (2.0 if name.startswith("walk") else 0.5)
    r = analyse(pi, skip)
    if r is None:
        continue
    print(f"\n=== {name} cmd={cmd} window={r['T']:.2f}s (after {skip}s) ===")
    print(f"base z mean {r['base_z_mean']:.3f} std {r['base_z_std']:.3f} min {r['base_z_min']:.3f} m | roll {r['roll_mean']:.1f} pitch {r['pitch_mean']:.1f} deg")
    print(f"vx_body {r['vx_body_mean']:.3f} vy_body {r['vy_body_mean']:.3f} | |v_xy| {r['speed_xy_mean']:.3f} | displacement speed {r['disp_speed']:.3f} m/s | yaw rate {r['yaw_rate_mean']:.3f}")
    print("joint mean  (hips FL FR RL RR | thighs | calves):", fmt(r["q_mean"]))
    print("target mean (hips | thighs | calves):           ", fmt(r["qt_mean"]))
    print("FL leg range hip/thigh/calf:", [fmt([r["q_min"][j], r["q_max"][j]]) for j in FL])
    print("foot contact z (p5):", fmt(r["foot_contact_z"]), "| foot clearance p98-p5:", fmt(r["foot_clearance"]))
    print("feet in base frame mean xyz:", fmt(r["feet_b_mean"]))
    print(f"step freq: zero-cross FL thigh {r['zc_FL_thigh']:.2f} Hz, FL calf {r['zc_FL_calf']:.2f} Hz | FFT calves {fmt(r['fft_calves'])} thighs {fmt(r['fft_thighs'])}")
    print("lift-offs per s (>1cm) FL FR RL RR:", fmt(r["liftoff_hz"]), "| duty factor:", fmt(r["duty"]))
    print(f"calf corr diag FL-RR {r['corr_FL_RR']:.2f} FR-RL {r['corr_FR_RL']:.2f} | lateral FL-FR {r['corr_FL_FR']:.2f} | ipsi FL-RL {r['corr_FL_RL']:.2f}")
    if "tau_absmean_hip_thigh_calf" in r:
        print("mean |tau| hip/thigh/calf:", fmt(r["tau_absmean_hip_thigh_calf"]), "| |tau|max", round(r["tau_absmax"], 2), "| frac >=23.4 Nm:", round(r["tau_sat_frac"], 3))
print("\nfinal: ready", s.ready, "failed", s.failed, "command", s.command.tolist(), "base", fmt(D["pos"][-1]))
