"""Validate scan-time transforms, calibrate a station, or run one NavigateToPose."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
import re
import signal
from pathlib import Path
import subprocess
import sys
import time

import yaml

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


ROOT = Path(__file__).resolve().parents[2]
SITE = ROOT / "sites/factory_hall/site.json"
WINDOWS_PYTHON = "/mnt/c/Users/Admin/AppData/Local/Programs/Python/Python312/python.exe"
KIT_SEND = "D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def windows_path(path):
    return subprocess.check_output(["wslpath", "-m", str(path.resolve())], text=True).strip()


def kit(action, output=None):
    args = {"action": action}
    if output is not None:
        args["output_path"] = windows_path(output)
    command = [
        os.environ.get("GO2_WINDOWS_PYTHON", WINDOWS_PYTHON), "-I", KIT_SEND,
        "--file", windows_path(ROOT / "tools/isaac/nav2_inspection.py"),
        "--args-json", json.dumps(args), "--timeout", "24",
    ]
    result = subprocess.run(command, capture_output=True, text=True, timeout=28)
    if result.returncode:
        raise RuntimeError(f"Kit {action}: {result.stderr.strip() or result.stdout.strip()}")
    return json.loads(result.stdout.strip().splitlines()[-1])


def wrap(angle):
    return math.remainder(angle, math.tau)


def transform_pose(tf):
    p, q = tf.transform.translation, tf.transform.rotation
    return {"x": p.x, "y": p.y, "yaw_rad": math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))}


class Observe:
    def __init__(self, node, executor):
        self.node = node
        self.executor = executor
        self.clock = None
        self.reset = False
        self.map = None
        self.scans = deque(maxlen=400)
        self.map_tf_seen = False
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, node)
        node.create_subscription(Clock, "/clock", self.tick, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, "/map", self.grid, LATCHED)
        node.create_subscription(TFMessage, "/tf", self.tf, 100)

    def tick(self, message):
        stamp = Time.from_msg(message.clock)
        if self.clock is not None and stamp.nanoseconds <= self.clock.nanoseconds:
            self.reset |= stamp.nanoseconds < self.clock.nanoseconds
            return
        self.clock = stamp

    def scan(self, message):
        self.scans.append((Time.from_msg(message.header.stamp), message.header.frame_id))

    def grid(self, message):
        self.map = message

    def tf(self, message):
        if any(t.header.frame_id == "map" and t.child_frame_id == "odom" for t in message.transforms):
            self.map_tf_seen = True

    def map_publishers(self):
        # The two supported localizers have node names distinct from relay/player
        # and from the interactive-marker TF endpoints in SLAM Toolbox.
        return sorted(n for n in self.node.get_node_names() if n in ("slam_toolbox", "amcl"))

    def pump(self, seconds, test=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if test is not None and test():
                return True
            self.executor.spin_once(timeout_sec=0.03)
            if self.reset:
                raise RuntimeError("Clock reset. Restart the episode.")
        return False

    def pose(self, stamp):
        return transform_pose(self.buffer.lookup_transform("map", "base_link", stamp))

    def check(self):
        errors = []
        frames = ("map", "odom", "base_link", "lidar_link")
        checked = 0
        # Allow half a simulated second for asynchronous TF delivery.
        for stamp, frame in list(self.scans):
            if self.clock.nanoseconds - stamp.nanoseconds < 500_000_000:
                continue
            try:
                if frame != frames[-1]:
                    raise ValueError(f"Unexpected scan frame {frame}")
                for parent, child in zip(frames, frames[1:]):
                    self.buffer.lookup_transform(parent, child, stamp)
                self.buffer.lookup_transform("map", frame, stamp)
                checked += 1
            except (TransformException, ValueError) as exc:
                errors.append({"stamp_ns": stamp.nanoseconds, "error": str(exc)})
        return {
            "scan_tf_checked": checked, "tf_errors": errors,
            "map_to_odom_publishers": self.map_publishers(), "frames": frames,
            "cmd_vel_publishers": self.node.count_publishers("/cmd_vel"),
            "map": {"width": self.map.info.width, "height": self.map.info.height,
                    "resolution_m": self.map.info.resolution,
                    "free_cells": self.map.data.count(0),
                    "occupied_cells": self.map.data.count(100),
                    "unknown_cells": self.map.data.count(-1) + self.map.data.count(255)},
        }

    def ready(self, timeout):
        def ready():
            return (self.map is not None and self.map.header.frame_id == "map"
                    and self.clock is not None and len(self.scans) >= 2 and self.map_tf_seen
                    and self.map_publishers() in (["slam_toolbox"], ["amcl"])
                    and self.buffer.can_transform("map", "lidar_link", self.scans[-2][0]))
        if not self.pump(timeout, ready):
            raise RuntimeError("No map and scan-time TF from a single map→odom publisher")


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def calibrate(observer, args, site):
    if not args.map.is_file():
        raise ValueError("Save the live SLAM map before calibration")
    snapshot = kit("status")
    if not snapshot["playing"] or snapshot["owner"] != "manual" or any(snapshot["command"]):
        raise RuntimeError("Calibration requires a stationary Go2 in manual with Play active")
    world = snapshot["scan_pose"]
    stamp = Time(nanoseconds=world["stamp_ns"], clock_type=observer.clock.clock_type)
    if not observer.pump(4, lambda: observer.buffer.can_transform("map", "base_link", stamp)):
        raise RuntimeError("Unable to synchronize calibration anchor to its scan timestamp")
    solved = observer.pose(stamp)
    delta = wrap(solved["yaw_rad"] - world["yaw_rad"])
    c, s = math.cos(delta), math.sin(delta)

    def in_map(x, y, yaw):
        dx, dy = x - world["x"], y - world["y"]
        return {"x": solved["x"] + c * dx - s * dy, "y": solved["y"] + s * dx + c * dy,
                "yaw_rad": wrap(yaw + delta)}

    nav = site["nav2"]
    gauge = next(g for g in site["gauges"] if g["id"] == nav["inspection"]["gauge_id"])
    target = gauge["inspection_pose"]
    spawn = json.loads((ROOT / "scenes/factory_hall_meta.json").read_text())["robot_spawn"]
    nav["map_yaml"] = os.path.relpath(args.map.resolve(), SITE.parent)
    nav["initial_pose"] = solved
    nav["inspection"]["goal"] = in_map(target["x"], target["y"], math.radians(target["yaw_deg"]))
    nav["calibration"] = {
        "world_anchor": world, "map_anchor": solved,
        "map_pose_at_factory_spawn": in_map(*spawn["position"][:2], math.radians(spawn["yaw_deg"])),
    }
    save_json(SITE, site)
    print(json.dumps(nav, indent=2), flush=True)


def action_transcript(log, result):
    transcript = log.read_text(encoding="utf-8")
    goal_id = re.search(r"Goal accepted with ID: ([0-9a-f]+)", transcript)
    status = re.search(r"Goal finished with status: ([A-Z]+)", transcript)
    if goal_id:
        result["goal_uuid"] = goal_id[1]
    if status:
        result["action_status"] = status[1]
    trajectory = result["trajectory"]
    loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
    for message in re.finditer(r"^Feedback:\n(.*?)(?=\n\n)", transcript, re.M | re.S):
        feedback = yaml.load(message[1].lstrip(), Loader=loader)["current_pose"]
        stamp = feedback["header"]["stamp"]
        ns = stamp["sec"] * 1_000_000_000 + stamp["nanosec"]
        if trajectory and ns <= trajectory[-1]["stamp_ns"]:
            continue
        position = feedback["pose"]["position"]
        q = feedback["pose"]["orientation"]
        yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                         1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
        trajectory.append({"stamp_ns": ns, "x": position["x"], "y": position["y"], "yaw_rad": yaw})


def navigate(args, site, result):
    inspection = site["nav2"]["inspection"]
    goal = (dict(zip(("x", "y", "yaw_rad"), args.goal)) if args.goal else inspection["goal"])
    if goal is None or not all(math.isfinite(v) for v in goal.values()):
        raise RuntimeError("Calibrate a finite station goal in map first")
    live = os.environ["ROS_DOMAIN_ID"] == "42"

    request = {"pose": {"header": {"frame_id": "map"}, "pose": {
        "position": {"x": goal["x"], "y": goal["y"]},
        "orientation": {"z": math.sin(goal["yaw_rad"] / 2), "w": math.cos(goal["yaw_rad"] / 2)},
    }}}
    result.update(goal=goal, frame_id="map", station_id=inspection["station_id"],
                  gauge_id=inspection["gauge_id"], map_yaml=site["nav2"]["map_yaml"],
                  action_log="action.log", trajectory=[])
    start = time.monotonic()
    accepted_at = None
    action = None
    log = args.output / result["action_log"]
    try:
        if live:
            result["arm_attempted"] = True
            result["armed"] = kit("arm")
        with log.open("w", encoding="utf-8") as stream:
            action = subprocess.Popen([
                "ros2", "action", "send_goal", "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose", json.dumps(request), "--feedback",
            ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, "PYTHONUNBUFFERED": "1"})
        while action.poll() is None:
            time.sleep(0.1)
            now = time.monotonic()
            if accepted_at is None and "Goal accepted with ID:" in log.read_text(encoding="utf-8"):
                accepted_at = now
            if accepted_at is None and now - start >= 35.0:
                raise TimeoutError("No acknowledgement for NavigateToPose")
            if args.cancel_after is not None and accepted_at is not None and now - accepted_at >= args.cancel_after:
                result["cancel_requested"] = True
                break
            if now - start >= args.timeout:
                raise TimeoutError("NavigateToPose wall timeout")
    except KeyboardInterrupt:
        result["cancel_requested"] = True
    finally:
        try:
            if live and "arm_attempted" in result:
                result["stopped"] = kit("manual")
        finally:
            if action is not None and action.poll() is None:
                action.send_signal(signal.SIGINT)
                try:
                    action.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    action.kill()
                    action.wait()
            if log.is_file():
                action_transcript(log, result)
            result["duration_wall_s"] = time.monotonic() - start
    if result.get("action_status") != "SUCCEEDED":
        return 0 if args.cancel_after is not None and result.get("action_status") == "CANCELED" else 1
    if live and not args.goal and args.cancel_after is None:
        captured = kit("capture", args.output / "front.png")
        result.update(capture=captured["capture"], image_path="front.png",
                      site_profile=str(SITE.relative_to(ROOT)), stopped=captured)
    if result["trajectory"]:
        final = result["trajectory"][-1]
        result.update(final_pose=final, final_pose_source="last NavigateToPose feedback",
                      xy_error_m=math.hypot(goal["x"] - final["x"], goal["y"] - final["y"]),
                      yaw_error_rad=wrap(goal["yaw_rad"] - final["yaw_rad"]))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    check = sub.add_parser("check")
    check.add_argument("--seconds", type=float, default=3.0)
    check.add_argument("--drive-ready", action="store_true")
    calibration = sub.add_parser("calibrate")
    calibration.add_argument("--map", type=Path, required=True)
    demo = sub.add_parser("demo")
    demo.add_argument("--output", type=Path, required=True)
    demo.add_argument("--timeout", type=float, default=150.0)
    demo.add_argument("--cancel-after", type=float)
    demo.add_argument("--goal", type=float, nargs=3, metavar=("X", "Y", "YAW_RAD"))
    args = parser.parse_args()
    if args.mode == "demo" and (
            not math.isfinite(args.timeout) or args.timeout <= 0
            or (args.cancel_after is not None and (
                not math.isfinite(args.cancel_after) or args.cancel_after <= 0))):
        parser.error("Demo timeouts must be finite and positive")
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain not in ("42", "43") or (args.mode == "calibrate" and domain != "42"):
        parser.error("Set ROS_DOMAIN_ID=42 (live) or 43 (replay). Calibration requires live SLAM.")
    if args.mode == "demo":
        args.output.mkdir(parents=True, exist_ok=False)
    site = json.loads(SITE.read_text(encoding="utf-8"))
    result = {"domain": int(domain), "started_utc": datetime.now(timezone.utc).isoformat()}
    try:
        if args.mode == "demo":
            if domain == "42":
                # Let the DDS probe process exit before starting the action client.
                with (args.output / "preflight.json").open("w", encoding="utf-8") as probe_log:
                    probe = subprocess.run([
                        sys.executable, str(Path(__file__).resolve()), "check", "--drive-ready",
                    ], stdout=probe_log, check=False, timeout=40)
                if probe.returncode:
                    raise RuntimeError("Drive preflight failed. See preflight.json.")
            return navigate(args, site, result)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("go2_nav2_run", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            observer = Observe(node, executor)
            observer.ready(30)
            if args.mode == "check":
                observer.scans.clear()
                observer.pump(args.seconds)
                result.update(observer.check())
                if args.drive_ready and (domain != "42" or result["map_to_odom_publishers"] != ["amcl"]
                                         or result["cmd_vel_publishers"] != 1):
                    raise RuntimeError("Driving requires live AMCL and exactly one /cmd_vel publisher")
                return int(not result["scan_tf_checked"] or bool(result["tf_errors"])
                           or len(result["map_to_odom_publishers"]) != 1)
            if args.mode == "calibrate":
                if not any(n == "slam_toolbox" for n in node.get_node_names()):
                    raise RuntimeError("Calibration requires the mapping launch")
                calibrate(observer, args, site)
                return 0
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
    except (RuntimeError, TimeoutError, TransformException, subprocess.SubprocessError) as exc:
        result["error"] = str(exc)
        return 1
    finally:
        if args.mode == "demo":
            path = args.output / "result.json"
            save_json(path, result)
            result = {"result_path": str(path), "action_status": result.get("action_status"),
                      "error": result.get("error")}
        print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
