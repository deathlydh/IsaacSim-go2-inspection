"""Launch-owned clock watchdog and one-shot Twist to TwistStamped adapter."""
import argparse
import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


RAW_TOPIC = "/go2/nav2_cmd_vel"
FINAL_TOPIC = "/cmd_vel"
PROFILE = Path(__file__).resolve().parents[2] / "robots/go2_edu.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true")
    args = parser.parse_args()
    live = os.environ.get("ROS_DOMAIN_ID") == "42"
    if args.drive and not live:
        parser.error("Motion is restricted to live domain 42")
    return run(drive=args.drive, live=live)


def run(*, drive, live):
    limits = json.loads(PROFILE.read_text(encoding="utf-8"))["bridge"]
    rclpy.init()
    node = rclpy.create_node("go2_nav2_episode", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", value=True),
    ])
    publisher = node.create_publisher(TwistStamped, FINAL_TOPIC, 1) if drive else None
    stamp_ns = 0
    tick_at = None
    pending = None
    error = None

    def receive(msg):
        nonlocal pending
        pending = None
        now = time.monotonic()
        if tick_at is None or now - tick_at >= 0.25:
            return
        linear, angular = msg.linear, msg.angular
        values = (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
        if all(map(math.isfinite, values)) and not any((linear.z, angular.x, angular.y)):
            scale = max(1.0, math.hypot(linear.x, linear.y) / limits["max_linear_mps"])
            linear.x /= scale
            linear.y /= scale
            yaw_limit = limits["max_yaw_rad_s"]
            angular.z = min(yaw_limit, max(-yaw_limit, angular.z))
            pending = (now, msg)

    def tick(msg):
        nonlocal stamp_ns, tick_at, pending, error
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if stamp < stamp_ns:
            error = "Clock reset. Start a new episode."
        if stamp <= stamp_ns:
            return
        now = time.monotonic()
        previous_tick = tick_at
        stamp_ns, tick_at = stamp, now
        command, pending = pending, None
        if command is None or now - previous_tick >= 0.25 or now - command[0] >= 0.15:
            return
        if node.count_publishers(FINAL_TOPIC) != 1:
            error = "Expected exactly one /cmd_vel publisher"
            return
        out = TwistStamped()
        out.header.frame_id = "base_link"
        out.header.stamp = msg.clock
        out.twist = command[1]
        publisher.publish(out)

    node.create_subscription(Clock, "/clock", tick, qos_profile_sensor_data)
    if publisher is not None:
        node.create_subscription(Twist, RAW_TOPIC, receive, 1)
    try:
        while rclpy.ok() and error is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            # Mapping in domain 43 can save the last map after playback ends.
            if live and tick_at is not None and time.monotonic() - tick_at >= 2.0:
                error = "Live clock stopped. Start a new episode."
        if error:
            node.get_logger().error(error)
            return 1
        return 0
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
