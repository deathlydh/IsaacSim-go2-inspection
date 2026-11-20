"""Verify scan-time TF and optional physical response to stamped commands."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Check:
    def __init__(self, node):
        self.node = node
        self.count = Counter()
        self.clock = self.pose = None
        self.buffer = Buffer(cache_time=Duration(seconds=120))
        self.scans = []
        self.first = self.last = None
        self.frames = set()
        self.tf_stamps = set()
        self.odom_stamps = set()
        for msg, topic, fn, qos in (
            (LaserScan, "/scan", self.scan, qos_profile_sensor_data),
            (Odometry, "/odom", self.odom, 10),
            (TFMessage, "/tf", lambda m: self.tf(m, False), 100),
            (TFMessage, "/tf_static", lambda m: self.tf(m, True),
             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            (Clock, "/clock", self.tick, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        ):
            node.create_subscription(msg, topic, fn, qos)

    def scan(self, msg):
        assert msg.header.frame_id == "lidar_link" and len(msg.ranges) >= 2
        assert math.isclose(msg.angle_min + (len(msg.ranges)-1)*msg.angle_increment,
                            msg.angle_max, abs_tol=1e-5)
        self.count["/scan"] += 1
        self.scans.append((msg.header.stamp, msg.header.frame_id))
        self.first = self.first or msg
        self.last = msg

    def odom(self, msg):
        self.count["/odom"] += 1
        self.odom_stamps.add(ns(msg.header.stamp))
        assert (msg.header.frame_id, msg.child_frame_id) == ("odom", "base_link")
        self.pose = msg.pose.pose

    def tf(self, msg, static):
        self.count["/tf_static" if static else "/tf"] += 1
        for tf in msg.transforms:
            self.frames.add((tf.header.frame_id, tf.child_frame_id))
            if static:
                self.buffer.set_transform_static(tf, "go2_check")
            else:
                self.buffer.set_transform(tf, "go2_check")
                self.tf_stamps.add(ns(tf.header.stamp))

    def tick(self, msg):
        self.count["/clock"] += 1
        assert self.clock is None or ns(msg.clock) > ns(self.clock)
        self.clock = msg.clock

    def spin(self, seconds, command=None):
        until = time.monotonic() + seconds
        next_command = 0
        stamp = None
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if command and self.clock and ns(self.clock) != stamp and time.monotonic() >= next_command:
                stamp = ns(self.clock)
                command(self.clock)
                next_command = time.monotonic() + 0.08

    @staticmethod
    def yaw(pose):
        q = pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def motion(self):
        pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 1)
        self.spin(1.0)
        assert pub.get_subscription_count() > 0
        phases = {}
        for name, (vx, vy, wz) in (
            ("forward", (0.30, 0., 0.)),
            ("left", (0., 0.25, 0.)),
            ("ccw", (0., 0., 0.55)),
            ("stale", (0.45, 0., 0.)),
        ):
            before = self.pose
            yaw = self.yaw(before)

            def command(stamp):
                if name == "stale":
                    stamp = Time(nanoseconds=max(0, ns(stamp) - 2_000_000_000)).to_msg()
                self.publish_command(pub, stamp, vx, vy, wz)

            self.spin(3.5 if name != "stale" else 2.2, command)
            after = self.pose
            dx = after.position.x - before.position.x
            dy = after.position.y - before.position.y
            phases[name] = {
                "forward_m": math.cos(yaw)*dx + math.sin(yaw)*dy,
                "left_m": -math.sin(yaw)*dx + math.cos(yaw)*dy,
                "yaw_rad": math.remainder(self.yaw(after) - yaw, math.tau),
            }
            self.spin(1.1)
        # Re-enter motion, destroy the publisher without sending a zero, then measure settled drift.
        self.spin(2.0, lambda stamp: self.publish_command(pub, stamp, 0.30, 0., 0.))
        self.node.destroy_publisher(pub)
        self.spin(1.2)
        before = self.pose
        self.spin(1.5)
        after = self.pose
        phases["publisher_lost"] = {"settled_drift_m": math.hypot(
            after.position.x - before.position.x, after.position.y - before.position.y)}
        assert phases["forward"]["forward_m"] > 0.05
        assert phases["left"]["left_m"] > 0.05
        assert phases["ccw"]["yaw_rad"] > 0.10
        assert abs(phases["stale"]["forward_m"]) < 0.04
        assert phases["publisher_lost"]["settled_drift_m"] < 0.05
        return phases

    @staticmethod
    def publish_command(pub, stamp, vx, vy, wz):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = stamp
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", action="store_true", help="Go2 must be in ROS mode with a clear area")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("go2_nav_check")
    try:
        check = Check(node)
        check.spin(3)
        assert all(check.count[t] > 0 for t in ("/scan", "/odom", "/tf", "/tf_static", "/clock")), check.count
        started = time.monotonic()
        before = check.count.copy()
        check.spin(args.seconds)
        elapsed = time.monotonic() - started
        hz = {topic: count / elapsed for topic, count in (check.count - before).items()}
        phases = check.motion() if args.motion else None
        tf_errors = []
        # Allow delivery of transforms queued just after the final best-effort scan.
        samples = list(check.scans)
        check.spin(0.3)
        for stamp, frame in samples:
            try:
                if ns(stamp) not in check.tf_stamps or ns(stamp) not in check.odom_stamps:
                    raise ValueError("No odometry and dynamic TF with identical stamp")
                check.buffer.lookup_transform("odom", frame, Time.from_msg(stamp))
            except (TransformException, ValueError) as exc:
                tf_errors.append((ns(stamp), str(exc)))
        finite = sum(map(math.isfinite, check.last.ranges))
        changed_rays = sum(
            math.isfinite(a) and math.isfinite(b) and abs(a-b) > 0.03
            for a, b in zip(check.first.ranges, check.last.ranges)
        )
        result = dict(messages=check.count, hertz_wall=hz, frames=sorted(check.frames),
                      scan_tf_checked=len(samples), tf_errors=tf_errors,
                      scan_size=len(check.last.ranges), finite=finite, changed_rays=changed_rays, motion=phases)
        output = json.dumps(result, indent=2)
        if args.output:
            args.output.write_text(output, encoding="utf-8")
        print(output)
        assert not tf_errors and finite > 0
        if args.motion:
            assert changed_rays > 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
