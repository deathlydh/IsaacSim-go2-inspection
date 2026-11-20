"""Physics-synchronous planar lidar, simulator odometry and ROS command ownership."""

import json
import math
from pathlib import Path
import time

import carb
import carb.eventdispatcher
import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from pxr import Gf, Usd, UsdGeom

from command_control import install_control


ROBOT = "/World/Robot"
LIDAR = f"{ROBOT}/base/lidar_link"
PROFILE = Path(__file__).resolve().parents[2] / "robots" / "go2_edu.json"
NS = 1_000_000_000


def xyz(vector, values):
    vector.x, vector.y, vector.z = map(float, values)


def xyzw(quaternion, wxyz):
    quaternion.w, quaternion.x, quaternion.y, quaternion.z = map(float, wxyz)


def vector(value):
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value).reshape(-1)


def rotation(wxyz):
    w, x, y, z = wxyz
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


class PlanarLidar:
    def __init__(self, stage, cfg):
        self.stage = stage
        self.translation = np.asarray(cfg["translation_m"], dtype=float)
        self.yaw = math.radians(cfg["yaw_deg"])
        self.quat = (math.cos(self.yaw / 2), 0., 0., math.sin(self.yaw / 2))
        fov = math.radians(cfg["fov_deg"])
        count = cfg["rays"]
        self.near, self.far = cfg["range_min_m"], cfg["range_max_m"]
        self.floor_z = cfg["floor_max_z_m"]
        if (self.translation.shape != (3,) or not np.isfinite(self.translation).all()
                or not math.isfinite(self.yaw) or not 0 < fov <= math.tau
                or not isinstance(count, int) or count < 2
                or not 0 < self.near < self.far < math.inf or not math.isfinite(self.floor_z)):
            raise ValueError("Invalid planar LiDAR profile")
        self.first = -fov / 2
        self.increment = fov / (count if fov == math.tau else count - 1)
        self.last = self.first + (count - 1) * self.increment
        angles = self.first + np.arange(count) * self.increment + self.yaw
        self.directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(count)))
        self.query = omni.physx.get_physx_scene_query_interface()
        if not stage.GetPrimAtPath(f"{ROBOT}/base") or stage.GetPrimAtPath(LIDAR):
            raise RuntimeError(f"Expected robot base and an unused mount path: {LIDAR}")
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            mount = UsdGeom.Xform.Define(stage, LIDAR)
            mount.AddTranslateOp().Set(Gf.Vec3d(*self.translation.tolist()))
            mount.AddRotateZOp().Set(float(cfg["yaw_deg"]))

    def sample(self, position, basis):
        origin = tuple(position + basis @ self.translation)
        ranges, hits = [], []
        start = time.perf_counter()
        for direction in self.directions @ basis.T:
            hit = self.query.raycast_closest(origin, tuple(direction), self.far, True)
            if not hit.get("hit", False):
                ranges.append(math.inf)
                hits.append(None)
                continue
            distance = float(hit["distance"])
            path = hit["collision"]
            self_hit = path == ROBOT or path.startswith(ROBOT + "/")
            floor_hit = hit["position"][2] <= self.floor_z and abs(hit["normal"][2]) >= 0.85
            ranges.append(math.nan if self_hit or floor_hit or distance < self.near else distance)
            hits.append(path)
        return origin, ranges, hits, time.perf_counter() - start

    def close(self):
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            self.stage.RemovePrim(LIDAR)


class RosTransport:
    def __init__(self, receive):
        import rclpy
        from geometry_msgs.msg import TransformStamped, TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        self.Transform, self.Odometry = TransformStamped, Odometry
        self.Clock, self.Scan, self.TF = Clock, LaserScan, TFMessage
        self.context = Context()
        self.node = self.executor = None
        try:
            rclpy.init(args=[], context=self.context)
            self.node = rclpy.create_node("go2_isaac_sim", context=self.context, enable_rosout=False)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.scan = self.node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.odom = self.node.create_publisher(Odometry, "/odom", 10)
            self.tf = self.node.create_publisher(TFMessage, "/tf", 100)
            static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.static = self.node.create_publisher(TFMessage, "/tf_static", static_qos)
            clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.clock = self.node.create_publisher(Clock, "/clock", clock_qos)
            self.node.create_subscription(TwistStamped, "/cmd_vel", receive, 1)
        except Exception:
            self.close()
            raise

    def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        self.context.try_shutdown()

    def transform(self, parent, child, position, quat):
        tf = self.Transform()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        xyz(tf.transform.translation, position)
        xyzw(tf.transform.rotation, quat)
        return tf

    def publish_odom(self, stamp, pose, quat, linear, angular):
        tf = self.transform("odom", "base_link", pose, quat)
        tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(stamp, NS)
        msg = self.Odometry()
        msg.header, msg.child_frame_id = tf.header, tf.child_frame_id
        xyz(msg.pose.pose.position, pose)
        xyzw(msg.pose.pose.orientation, quat)
        xyz(msg.twist.twist.linear, linear)
        xyz(msg.twist.twist.angular, angular)
        self.tf.publish(self.TF(transforms=[tf]))
        self.odom.publish(msg)


class Go2Navigation:
    def __init__(self, session, profile=PROFILE, *, bridge=True):
        with open(profile, encoding="utf-8") as file:
            config = json.load(file)
        lidar_cfg, self.cfg = config["lidar"], config["bridge"]
        self.periods = {
            name: self.period(rate) for name, rate in (
                ("scan", lidar_cfg["rate_hz"]), ("odom", self.cfg["odom_hz"]),
                ("clock", self.cfg["clock_hz"]),
            )
        }
        for name in ("command_timeout_wall_s", "command_max_age_sim_s", "max_linear_mps", "max_yaw_rad_s"):
            if not 0 < self.cfg[name] < math.inf:
                raise ValueError(f"{name} must be finite and positive")
        if max(self.cfg["max_linear_mps"], self.cfg["max_yaw_rad_s"]) > 1:
            raise ValueError("Command limits exceed locomotion policy limits")
        self.session = session
        self.control = install_control(session, upgrade=True)
        self.timeline = omni.timeline.get_timeline_interface()
        self.callbacks = []
        self.update_sub = self.stage_sub = self.ros = self.lidar = None
        self.epoch = None
        self.sim_ns = 0
        self.due = dict.fromkeys(self.periods, 0)
        self.scan_enabled = True
        self.sample = None
        self.scans = 0
        self.received = 0
        self.accepted = 0
        self.last_stamp = 0
        self.deadline = None
        self.expirations = 0
        try:
            ctx = omni.usd.get_context()
            stage = ctx.get_stage()
            if UsdGeom.GetStageMetersPerUnit(stage) != 1.0 or UsdGeom.GetStageUpAxis(stage) != "Z":
                raise RuntimeError("LiDAR requires a meter-scale Z-up stage")
            self.lidar = PlanarLidar(stage, lidar_cfg)
            if bridge:
                self.ros = RosTransport(self.receive)
                tf = self.ros.transform("base_link", "lidar_link", self.lidar.translation, self.lidar.quat)
                self.ros.static.publish(self.ros.TF(transforms=[tf]))
            for callback, event in (
                (self.before_step, IsaacEvents.PRE_PHYSICS_STEP),
                (self.after_step, IsaacEvents.POST_PHYSICS_STEP),
                (self.on_stop, IsaacEvents.TIMELINE_STOP),
            ):
                self.callbacks.append(SimulationManager.register_callback(callback, event))
            self.update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self.on_update, name="go2 navigation update")
            self.stage_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=ctx.stage_event_name(omni.usd.StageEventType.CLOSING),
                on_event=lambda *_: self.close(), observer_name="go2 navigation closing")
        except Exception:
            self.close()
            raise

    @staticmethod
    def period(rate):
        if not math.isfinite(rate) or not 0 < rate <= 1000:
            raise ValueError("Profile values must be finite and positive")
        return round(NS / rate)

    def set_mode(self, mode):
        if mode == "manual":
            if self.control.owner is self:
                self.control.release(self)
            elif self.control.owner is not None:
                raise RuntimeError("Route owns motion; cancel the route through its own entry point")
            self.deadline = None
        elif mode == "ros":
            if self.ros is None or not self.timeline.is_playing() or not self.session.ready or self.session.failed:
                raise RuntimeError("ROS mode requires a healthy running Go2 and ROS transport")
            self.control.acquire(self, source="ros")
            self.last_stamp = self.sim_ns
            self.deadline = None
        else:
            raise ValueError("Expected command mode manual or ros")

    def receive(self, msg):
        self.received += 1
        if self.control.owner is not self or self.control.cancel_requested or not self.timeline.is_playing():
            return
        stamp = msg.header.stamp.sec * NS + msg.header.stamp.nanosec
        age = self.sim_ns - stamp
        if (msg.header.frame_id not in ("", "base_link") or stamp <= self.last_stamp
                or age < 0 or age > self.cfg["command_max_age_sim_s"] * NS):
            return
        linear, angular = msg.twist.linear, msg.twist.angular
        values = (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
        if not all(map(math.isfinite, values)) or any((linear.z, angular.x, angular.y)):
            return
        scale = max(1., math.hypot(linear.x, linear.y) / self.cfg["max_linear_mps"])
        yaw = max(-self.cfg["max_yaw_rad_s"], min(self.cfg["max_yaw_rad_s"], angular.z))
        self.session.command[:] = (linear.x / scale, linear.y / scale, yaw)
        self.last_stamp = stamp
        self.deadline = time.monotonic() + self.cfg["command_timeout_wall_s"]
        self.accepted += 1

    def before_step(self, *_):
        if self.control.owner is not self:
            return
        art = self.session.runner.articulation if self.session.runner is not None else None
        if self.session.failed or not self.session.ready or art is None or not art.is_physics_tensor_entity_valid():
            self.session.failed = True
            self.set_mode("manual")
        elif self.control.cancel_requested or not self.timeline.is_playing():
            self.set_mode("manual")
        elif self.deadline is not None and time.monotonic() >= self.deadline:
            self.control.stop()
            self.deadline = None
            self.expirations += 1

    def on_update(self, *_):
        try:
            self.before_step()
            if self.ros is not None:
                self.ros.executor.spin_once(timeout_sec=0)
        except Exception as exc:
            self.fail(exc)

    def on_stop(self, *_):
        if self.control.owner is self:
            self.set_mode("manual")
        self.epoch = None
        self.sample = None

    def due_now(self, name):
        if self.sim_ns < self.due[name]:
            return False
        period = self.periods[name]
        self.due[name] = (self.sim_ns // period + 1) * period
        return True

    def after_step(self, dt, _context):
        try:
            self.publish_step(dt)
        except Exception as exc:
            self.fail(exc)

    def publish_step(self, dt):
        self.sim_ns += round(dt * NS)
        ros = self.ros
        sec, nsec = divmod(self.sim_ns, NS)
        if ros is not None and self.due_now("clock"):
            msg = ros.Clock()
            msg.clock.sec, msg.clock.nanosec = sec, nsec
            ros.clock.publish(msg)
        if not self.session.ready or self.session.failed:
            return
        scan_now = self.scan_enabled and self.due_now("scan")
        odom_now = ros is not None and (self.due_now("odom") or scan_now)
        if not scan_now and not odom_now:
            return
        art = self.session.runner.articulation
        pos, quat = map(vector, art.get_world_poses())
        basis = rotation(quat)
        if self.epoch is None:
            yaw = math.atan2(basis[1, 0], basis[0, 0])
            self.epoch = (np.array([pos[0], pos[1], 0.]), math.cos(yaw/2), -math.sin(yaw/2))
        if odom_now:
            origin, c, s = self.epoch
            w, x, y, z = quat
            odom_quat = (c*w - s*z, c*x - s*y, c*y + s*x, c*z + s*w)
            linear, angular = (basis.T @ vector(v) for v in art.get_velocities())
            ros.publish_odom(self.sim_ns, rotation((c, 0., 0., s)) @ (pos-origin), odom_quat, linear, angular)
        if scan_now:
            lidar = self.lidar
            origin, ranges, hits, cost = lidar.sample(pos, basis)
            self.scans += 1
            self.sample = (self.sim_ns, origin, ranges, hits, cost, basis)
            if ros is not None:
                scan = ros.Scan()
                scan.header.frame_id = "lidar_link"
                scan.header.stamp.sec, scan.header.stamp.nanosec = sec, nsec
                scan.angle_min, scan.angle_max, scan.angle_increment = lidar.first, lidar.last, lidar.increment
                scan.scan_time = self.periods["scan"] / NS
                scan.range_min, scan.range_max = lidar.near, lidar.far
                scan.ranges = ranges
                ros.scan.publish(scan)

    def fail(self, exc):
        carb.log_error(f"Go2 navigation disabled: {exc!r}")
        self.close()

    def close(self):
        if self.control.owner is self:
            self.set_mode("manual")
        self.update_sub = self.stage_sub = None
        for callback in self.callbacks:
            SimulationManager.deregister_callback(callback)
        self.callbacks.clear()
        if self.ros is not None:
            self.ros.close()
            self.ros = None
        if self.lidar is not None:
            self.lidar.close()
            self.lidar = None
        if getattr(self.session, "navigation", None) is self:
            self.session.navigation = None
