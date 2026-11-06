import asyncio
import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools/isaac"))
from inspection_actions import KitIO, MotionConfig, Pose, approach_pose, stabilize
from route_planning import Clearance, intersects_box, plan_route
from inspection.actions import capture_at_pose


class FakeIO:
    def __init__(self):
        self.p = Pose(0,0,0.3,0,0)
        self.t = 0.0
        self.cmd = [0,0,0]
        self.cancel = False
        self.ok = True
        self.raise_on_update = False
        self.drift = False
        self.commands = []

    def pose(self): return self.p
    def healthy(self): return self.ok
    def cancelled(self): return self.cancel
    def stop(self): self.cmd = [0,0,0]
    def command(self,x,y,h):
        self.cmd = [x,y,h]
        self.commands.append(list(self.cmd))
    async def update(self):
        self.t += 0.1
        if self.raise_on_update:
            raise RuntimeError("policy_exception")
        if self.drift:
            self.p = replace(self.p,x=self.p.x+0.03)


class ActionsTest(unittest.IsolatedAsyncioTestCase):
    async def test_capture_renews_heading_and_stops_on_camera_failure(self):
        import types
        for fail in (False,True):
            io=FakeIO()
            async def capture(timeout):
                for _ in range(3):await asyncio.sleep(0)
                if fail:raise RuntimeError('camera_failed')
                return 'image',{'capture_id':'test'}
            camera=types.SimpleNamespace(capture=capture)
            if fail:
                with self.assertRaisesRegex(RuntimeError,'camera_failed'):
                    await capture_at_pose(io,camera,[0,0,1],MotionConfig())
            else:
                result=await capture_at_pose(io,camera,[0,0,1],MotionConfig())
                self.assertEqual(result[0],'image')
            self.assertGreaterEqual(len(io.commands),3)
            self.assertTrue(all(c==[0,0,1] for c in io.commands))
            self.assertEqual(io.cmd,[0,0,0])

    async def test_timeout_is_wall_clock_and_stops(self):
        io = FakeIO()
        r = await approach_pose(io,[2,0,0],MotionConfig(timeout_s=0.25),clock=lambda:io.t)
        self.assertEqual(r["outcome"],"timeout")
        self.assertLess(r["duration_s"],0.5)
        self.assertEqual(io.cmd,[0,0,0])

    async def test_cancel_and_policy_failure_stop(self):
        for state in ("cancel","ok","raise_on_update"):
            io = FakeIO()
            setattr(io,state,state != "ok")
            r = await approach_pose(io,[2,0,0],clock=lambda:io.t)
            self.assertEqual(r["outcome"],"cancelled" if state == "cancel" else "failed")
            self.assertEqual(io.cmd,[0,0,0])

    async def test_no_progress_and_unsafe_posture(self):
        io = FakeIO()
        r = await approach_pose(io,[2,0,0],MotionConfig(no_progress_s=0.25),clock=lambda:io.t)
        self.assertEqual(r["error"],"no_progress")
        io.p = replace(io.p,tilt_deg=40)
        r = await approach_pose(io,[2,0,0],clock=lambda:io.t)
        self.assertEqual(r["error"],"unsafe_posture")
        self.assertEqual(io.cmd,[0,0,0])

    async def test_reached_has_actual_errors(self):
        io = FakeIO()
        r = await approach_pose(io,[0.1,0,0],clock=lambda:io.t)
        self.assertEqual(r["outcome"],"reached")
        self.assertAlmostEqual(r["position_error_m"],0.1)

    async def test_stabilization_measures_window_and_rejects_drift(self):
        io = FakeIO()
        cfg = MotionConfig(settle_window_s=0.5,settle_timeout_s=1.0)
        r = await stabilize(io,[0,0,0],cfg,clock=lambda:io.t)
        self.assertEqual(r["outcome"],"reached")
        self.assertGreaterEqual(r["duration_s"],0.5)
        io = FakeIO()
        io.drift = True
        r = await stabilize(io,[0,0,0],cfg,clock=lambda:io.t)
        self.assertEqual(r["outcome"],"timeout")

    async def test_reverse_out_keeps_heading_without_turning_near_gauge(self):
        io = FakeIO()
        io.p = replace(io.p,y=-1,yaw=-math.pi/2)
        await approach_pose(io,[0,0,-math.pi/2],MotionConfig(timeout_s=0.25),
                            clock=lambda:io.t,keep_heading=True)
        self.assertLess(io.commands[0][0],0)
        self.assertAlmostEqual(io.commands[0][2],-math.pi/2)

    async def test_stabilization_clears_turn_on_success_timeout_and_exception(self):
        for scenario in ('success','timeout','exception'):
            io=FakeIO()
            io.p=replace(io.p,yaw=1.0)
            io.drift=scenario=='timeout'
            io.raise_on_update=scenario=='exception'
            cfg=MotionConfig(settle_window_s=.2,settle_timeout_s=.5)
            if scenario=='exception':
                with self.assertRaises(RuntimeError):
                    await stabilize(io,[0,0,1.0],cfg,clock=lambda:io.t)
            else:
                result=await stabilize(io,[0,0,1.0],cfg,clock=lambda:io.t)
                self.assertEqual(result['outcome'],'reached' if scenario=='success' else 'timeout')
            self.assertEqual(io.cmd,[0,0,0])


class ClearanceTest(unittest.TestCase):
    def test_entire_segment_and_robot_envelope(self):
        box = [[0.9,0.9,0],[1.1,1.1,1]]
        self.assertTrue(intersects_box([0,0],[2,2],box,0.1))
        self.assertFalse(intersects_box([0,0],[2,0],box,0.1))
        self.assertTrue(intersects_box([0,0.6],[2,0.6],box,0.4))
        self.assertTrue(intersects_box([1,1],[1,1],box,0.1))

    def test_unknown_start_refused(self):
        c = Clearance([],0.5,[-9,8,-1.6,1.6])
        self.assertEqual(c.blockers([-10,0],[0,0]),["outside_known_aisle"])
        self.assertTrue(c.blockers([float("nan"),0],[0,0]))

    def test_actual_start_joins_centreline(self):
        plan = plan_route([-5,-1,-1], [{"id":"g","waypoint":{"x":-4,"y":1,"yaw_deg":90}}],{"centerline_y":0})
        self.assertEqual(plan[0]["target"][:2],[-5,0])
        self.assertEqual(plan[-1]["target"][:2],[-4,1])
        self.assertTrue(plan[0]["keep_heading"])
        self.assertEqual(plan[0]["target"][2],-1)

    def test_heading_integral_disabled_at_half_turn(self):
        import types
        io = KitIO.__new__(KitIO)
        io.cfg = MotionConfig()
        io.session = types.SimpleNamespace(command=[0,0,0],_heading_target=None)
        io.heading_bias,io.last_heading = 0.2,math.pi
        import time
        io._sim_elapsed = 1.0
        io.last_command_at = .95
        io.pose = lambda:Pose(0,0,0.3,0,0)
        io.command(0,0,math.pi)
        self.assertEqual(io.heading_bias,0)
        self.assertEqual(io.session._heading_target,math.pi)


if __name__ == "__main__":
    unittest.main()
