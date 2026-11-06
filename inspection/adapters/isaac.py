"""Isaac-only pose/motion/clock/camera adapters. Oracle localization, not real hardware."""
import math
import time
from datetime import datetime, timezone
from dataclasses import asdict
from inspection.actions import MotionConfig, Pose, wrap

class KitIO:
    def __init__(self, session, control, cfg=None):
        import omni.kit.app
        import omni.timeline
        self.session, self.control = session, control
        self.app = omni.kit.app.get_app()
        self.timeline = omni.timeline.get_timeline_interface()
        self.cfg = cfg or MotionConfig()
        self.frame_id = "map"
        self.heading_bias = 0.0
        self.last_heading = None
        self._sim_elapsed = 0.0
        import omni.physx
        self._physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        self.last_command_at = self.control_time()
        self._command_deadline = None
        self._command_expired = False

    def pose(self):
        import numpy as np
        pos, quat = self.session.runner.articulation.get_world_poses()
        p = np.asarray(pos.numpy() if hasattr(pos,"numpy") else pos).reshape(-1)[:3]
        w,x,y,z = np.asarray(quat.numpy() if hasattr(quat,"numpy") else quat).reshape(-1)[:4]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        tilt = math.degrees(math.acos(max(-1.0,min(1.0,1-2*(x*x+y*y)))))
        return Pose(float(p[0]),float(p[1]),float(p[2]),yaw,tilt)

    def healthy(self):
        s = self.session
        return (self.timeline.is_playing() and s.ready and not s.failed and s.heading_hold and not self._command_expired
                and s.runner is not None and s.runner.articulation.is_physics_tensor_entity_valid())

    def cancelled(self):
        return self.control.cancel_requested

    def command(self, vx, vy, heading):
        # Integral trim compensates the measured steady yaw drift of robot_lab.
        # This belongs to the route controller, not the gait policy. Reuse the
        # session's existing proportional heading hold and reset trim on turns.
        now = self.control_time()
        dt = min(0.1,now-self.last_command_at)
        self.last_command_at = now
        if self.last_heading is None or abs(wrap(heading-self.last_heading)) > 0.1:
            self.heading_bias = 0.0
        self.last_heading = heading
        error = wrap(heading-self.pose().yaw)
        limit = self.cfg.heading_bias_limit_rad
        if abs(error) < math.radians(45):
            self.heading_bias = max(-limit,min(limit,self.heading_bias+self.cfg.heading_integral_gain*error*dt))
        else:
            # Do not shift a 180-degree target across the wrap boundary; that
            # reverses turn direction every update and can trap the robot.
            self.heading_bias = 0.0
        self.session._heading_target = heading+self.heading_bias
        self.body_velocity(vx,vy,0.0,now+0.25)

    def stop(self):
        self._command_deadline = None
        self.control.stop()

    async def update(self):
        await self.app.next_update_async()

    def _on_step(self, dt):
        self._sim_elapsed += float(dt)
        if getattr(self,'_command_deadline',None) is not None and (self.control_time() >= self._command_deadline or time.monotonic() >= self._wall_command_deadline):
            self.stop()
            self._command_expired = True

    def body_velocity(self,vx,vy,wz,valid_until_control_s):
        if not all(math.isfinite(v) for v in (vx,vy,wz,valid_until_control_s)):
            raise ValueError('invalid_velocity_command')
        speed=math.hypot(vx,vy)
        if speed>self.cfg.max_speed_mps+1e-12 or abs(wz)>1+1e-12:
            raise ValueError('velocity_limit')
        if speed>self.cfg.max_speed_mps:
            vx,vy=vx*self.cfg.max_speed_mps/speed,vy*self.cfg.max_speed_mps/speed
        wz=max(-1.,min(1.,wz))
        if valid_until_control_s<=self.control_time():
            self.stop()
            raise ValueError('expired_command')
        self.session.command[:]=[vx,vy,wz]
        self._command_deadline=valid_until_control_s
        self._wall_command_deadline=time.monotonic()+1.0

    def control_time(self):
        return self._sim_elapsed

    def pose_sample(self):
        import numpy as np
        _,quat=self.session.runner.articulation.get_world_poses()
        q=np.asarray(quat.numpy() if hasattr(quat,'numpy') else quat).reshape(-1)[:4]
        return {"pose": asdict(self.pose()), "quaternion_wxyz":[float(v) for v in q], "frame_id": self.frame_id,
                "control_timestamp_s": self.control_time(), "received_monotonic_s": time.monotonic(),
                "valid": self.healthy(), "quality": "simulator_oracle", "covariance": None}

    def acquire(self, owner): self.control.acquire(owner)
    def release(self, owner): self.control.release(owner)
    def cancel(self): self.control.cancel()
    def command_state(self): return self.session.command.tolist()
    def command_source(self): return self.control.source
    def close(self): self._physics_sub = None


class KitCamera:
    def __init__(self, session, io):
        self.session, self.io = session, io

    async def capture(self, timeout_s):
        import uuid
        camera = self.session.camera
        if camera is None: raise RuntimeError("no_camera")
        before = camera.render_count
        requested_sim = self.io.control_time()
        image = await camera.wait_for_rgb(fresh=True,max_updates=1000,timeout_s=timeout_s)
        if image is None: raise RuntimeError("fresh_frame_timeout")
        return image, {"capture_id":uuid.uuid4().hex, "render_count_before":before,
                       "render_count_after":camera.render_count,"frame_id":camera.render_count,
                       "requested_control_s":requested_sim,"control_timestamp_s":self.io.control_time(),
                       "received_monotonic_s":time.monotonic(),
                       "timestamp_utc":datetime.now(timezone.utc).isoformat(),
                       "timestamp_source":"render_event_and_receive_time",
                       "intrinsics":camera.intrinsics(),"extrinsics":{
                           "parent_frame":"base", "translation_m":[0.32715,0,0.04297],
                           "quaternion_wxyz":[0.5,0.5,-0.5,-0.5]}}

    def save(self, path, image):
        return self.session.camera.save_png(str(path),image=image)

def stage_obstacles(stage, z_min=0.08, z_max=0.90):
    """Enabled collider world AABBs, including instance proxies.

    Exclude only robot subtree and colliders outside the robot height interval.
    No prop is excluded on the strength of aisle metadata.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default","render","proxy"], useExtentsHint=False)
    obstacles, excluded = [], []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if str(prim.GetPath()).startswith("/World/Robot/"):
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        if prim.GetTypeName() == "Plane":
            from pxr import Gf
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            axis = prim.GetAttribute("axis").Get() or "Z"
            normal = matrix.TransformDir(Gf.Vec3d(*{"X":(1,0,0),"Y":(0,1,0),"Z":(0,0,1)}[axis])).GetNormalized()
            origin = matrix.Transform(Gf.Vec3d(0,0,0))
            if abs(normal[2]) > 0.9999 and origin[2] < z_min:
                excluded.append({"path":str(prim.GetPath()),"reason":"horizontal_support_plane","z":origin[2]})
                continue
            raise RuntimeError(f"unsupported_infinite_plane: {prim.GetPath()}")
        bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if bounds.IsEmpty():
            raise RuntimeError(f"empty_collider_bounds: {prim.GetPath()}")
        lo, hi = list(bounds.GetMin()), list(bounds.GetMax())
        item = {"path": str(prim.GetPath()), "box": [lo,hi]}
        if hi[2] < z_min or lo[2] > z_max:
            excluded.append(item)
        else:
            obstacles.append(item)
    if not obstacles:
        raise RuntimeError("no_obstacles_found")
    return obstacles, excluded
