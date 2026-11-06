"""Reusable async movement/capture actions. Core is testable without Isaac.

Known static scene only: no obstacle detection, SLAM or learned navigation.
The caller validates every segment before acquiring command ownership.
"""
import asyncio
from dataclasses import asdict, dataclass
import math
import time


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class MotionConfig:
    max_speed_mps: float = 0.5
    position_gain: float = 1.0
    position_tolerance_m: float = 0.15
    yaw_tolerance_deg: float = 3.0
    timeout_s: float = 90.0
    no_progress_s: float = 18.0
    progress_m: float = 0.025
    max_tilt_deg: float = 30.0
    min_height_m: float = 0.16
    max_height_m: float = 0.65
    settle_timeout_s: float = 18.0
    settle_window_s: float = 1.0
    settle_position_m: float = 0.035
    settle_yaw_deg: float = 2.0
    frame_timeout_s: float = 8.0
    heading_integral_gain: float = 0.8
    heading_bias_limit_rad: float = 0.35

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid {name}")
        if self.max_speed_mps > 0.5:
            raise ValueError("first route speed must be <= 0.5 m/s")


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    tilt_deg: float


def errors(pose, target):
    return math.hypot(target[0]-pose.x, target[1]-pose.y), abs(wrap(target[2]-pose.yaw))


def safety_error(io, p, cfg):
    if io.cancelled():
        return "cancelled"
    if not io.healthy():
        return "timeline_or_policy_unavailable"
    if not all(math.isfinite(v) for v in asdict(p).values()):
        return "invalid_pose"
    if p.tilt_deg > cfg.max_tilt_deg or not cfg.min_height_m <= p.z <= cfg.max_height_m:
        return "unsafe_posture"
    if hasattr(io, "pose_safe") and not io.pose_safe(p):
        return "outside_verified_clearance"
    return None


async def approach_pose(io, target, cfg=None, *, clock=time.monotonic, event=None, keep_heading=False):
    """Return reached/timeout/cancelled/failed; always zero the command on exit."""
    cfg = cfg or MotionConfig()
    start = clock()
    progress_at, best_score = start, float("inf")
    outcome, reason, final = "failed", None, None
    try:
        while True:
            final = io.pose()
            reason = safety_error(io, final, cfg)
            if reason:
                outcome = "cancelled" if reason == "cancelled" else "failed"
                break
            dist, yaw_err = errors(final, target)
            if clock() - start >= cfg.timeout_s:
                outcome, reason = "timeout", "approach_timeout"
                break
            # Leave margin for the residual gait oscillation while obtaining a fresh frame.
            if dist <= cfg.position_tolerance_m * 0.7 and yaw_err <= math.radians(cfg.yaw_tolerance_deg) * 0.5:
                outcome = "reached"
                break
            # Heading changes are progress even when rotating at the waypoint.
            score = dist + (0.3 * yaw_err if dist <= 0.5 else 0.0)
            if score < best_score - cfg.progress_m:
                best_score, progress_at = score, clock()
            if clock() - progress_at >= cfg.no_progress_s:
                reason = "no_progress"
                break
            dx, dy = target[0]-final.x, target[1]-final.y
            heading = math.atan2(dy, dx) if dist > 0.5 and not keep_heading else target[2]
            # Turn before translating; avoid sweeping an uncontrolled arc.
            alignment = 1.0 if keep_heading else max(0.0, math.cos(wrap(heading-final.yaw)))
            speed = min(cfg.max_speed_mps, cfg.position_gain * dist) * alignment
            if dist <= cfg.position_tolerance_m * 0.7:
                speed = 0.0
            norm = max(dist, 1e-9)
            c, s = math.cos(final.yaw), math.sin(final.yaw)
            io.command(speed*(c*dx+s*dy)/norm, speed*(-s*dx+c*dy)/norm, heading)
            if event:
                event(final)
            await io.update()
    except asyncio.CancelledError:
        outcome, reason = "cancelled", "task_cancelled"
    except Exception as error:
        reason = f"{type(error).__name__}: {error}"
    finally:
        io.stop()
        try:
            final = io.pose()
        except Exception:
            pass
    if final is not None and not all(math.isfinite(v) for v in asdict(final).values()):
        final = None
    dist, yaw_err = errors(final, target) if final is not None else (None, None)
    return {"outcome": outcome, "error": reason, "final_pose": asdict(final) if final else None,
            "position_error_m": dist, "yaw_error_deg": math.degrees(yaw_err) if yaw_err is not None else None,
            "duration_s": clock()-start}


async def stabilize(io, target, cfg, *, clock=time.monotonic):
    # Heading correction is motion too: stop it on every exit, including
    # cancellation/exceptions, before a potentially slow camera capture.
    try:
        return await _stabilize(io, target, cfg, clock=clock)
    finally:
        io.stop()


async def capture_at_pose(io, camera, target, cfg):
    """Renew bounded heading commands while awaiting the sensor; stop on exit."""
    task=asyncio.create_task(camera.capture(cfg.frame_timeout_s))
    deadline=time.monotonic()+cfg.frame_timeout_s
    try:
        while not task.done():
            reason=safety_error(io,io.pose(),cfg)
            if reason:
                raise RuntimeError(reason)
            if time.monotonic()>=deadline:
                raise TimeoutError('fresh_frame_timeout')
            io.command(0.0,0.0,target[2])
            await io.update()
            await asyncio.sleep(0)
        return await task
    finally:
        if not task.done():
            task.cancel()
        try:
            await task
        except (Exception,asyncio.CancelledError):
            pass
        io.stop()


async def _stabilize(io, target, cfg, *, clock=time.monotonic):
    start, samples = clock(), []
    control_clock = getattr(io, "control_time", clock)
    io.stop()
    while clock()-start < cfg.settle_timeout_s:
        p = io.pose()
        error = safety_error(io, p, cfg)
        if error:
            return {"outcome": "cancelled" if error == "cancelled" else "failed", "error": error}
        samples.append((control_clock(), p))
        # Retain a complete control-time window; timeout uses wall time.
        while len(samples) > 1 and samples[1][0] <= control_clock()-cfg.settle_window_s:
            samples.pop(0)
        if control_clock()-samples[0][0] >= cfg.settle_window_s:
            anchor = samples[0][1]
            stable = all(math.dist((q.x,q.y,q.z),(anchor.x,anchor.y,anchor.z)) <= cfg.settle_position_m
                         and abs(wrap(q.yaw-anchor.yaw)) <= math.radians(cfg.settle_yaw_deg) for _,q in samples)
            dist, yaw_err = errors(p, target)
            if stable and dist <= cfg.position_tolerance_m and yaw_err <= math.radians(cfg.yaw_tolerance_deg):
                return {"outcome": "reached", "error": None, "duration_s": clock()-start,
                        "position_error_m": dist, "yaw_error_deg": math.degrees(yaw_err), "final_pose": asdict(p)}
        io.command(0.0,0.0,target[2])
        await io.update()
    return {"outcome": "timeout", "error": "stabilization_timeout", "duration_s": clock()-start}

