import asyncio
import json
from pathlib import Path
import tempfile
import types
import unittest
from inspection.actions import Pose,MotionConfig
from inspection.site import load_site,load_config,verify_scene
from inspection.adapters.known_aisle import KnownAisle
from inspection.mission import run_route
from inspection.adapters.isaac import KitIO

ROOT=Path(__file__).resolve().parents[1]


class SitesTest(unittest.IsolatedAsyncioTestCase):
    async def test_same_runner_seven_targets_rotated_aisle(self):
        config,site=load_config(ROOT/'config/inspection_route.json',ROOT/'sites/rotated_demo/site.json')
        nav=KnownAisle(site['navigation'],[],.6)
        io=types.SimpleNamespace(pose=lambda:Pose(10,19,.3,0,0),healthy=lambda:True,
                                 cancelled=lambda:False,control_time=lambda:0)
        with tempfile.TemporaryDirectory() as folder:
            config['reports_root']=folder
            record,path=await run_route(io,nav,None,site,config,dry_run=True)
            self.assertEqual(record['outcome'],'preflight_passed')
            self.assertEqual(len(record['gauge_order']),7)
            self.assertEqual(record['map']['frame_id'],'demo_map')
            first=record['plan'][0]['target']
            self.assertAlmostEqual(first[0],10)
            self.assertAlmostEqual(first[1],20)

    def test_factory_targets_unchanged_and_truth_absent(self):
        _,site=load_config(ROOT/'config/inspection_route.json')
        old=json.loads((ROOT/'scenes/factory_hall_meta.json').read_text(encoding='utf-8'))
        self.assertEqual(site['route'],old['inspection_route'])
        for actual,previous in zip(site['gauges'],old['gauges']):
            self.assertEqual(actual['inspection_pose'],previous['waypoint'])
            self.assertNotIn('value',actual)
        verify_scene(site,ROOT/'scenes/factory_hall.usda')
        with self.assertRaises(ValueError): verify_scene(site,ROOT/'scenes/inspection.usd')
        with self.assertRaises(ValueError): verify_scene(load_site(ROOT/'sites/rotated_demo/site.json'),ROOT/'scenes/factory_hall.usda')

    def test_unsupported_map_transform_refused_instead_of_silent_frame_mix(self):
        site=json.loads((ROOT/'sites/factory_hall/site.json').read_text(encoding='utf-8'))
        site['map']['world_from_map']={'translation_m':[1,0,0],'yaw_deg':90}
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'site.json';p.write_text(json.dumps(site),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'localization_adapter'):load_site(p)

    def test_controller_dt_uses_simulation_not_wall_time(self):
        def simulate(wall_dt):
            io=KitIO.__new__(KitIO)
            io.cfg=MotionConfig()
            io.session=types.SimpleNamespace(command=[0,0,0],_heading_target=None)
            io.heading_bias=0; io.last_heading=0; io._sim_elapsed=0; io.last_command_at=0
            io.pose=lambda:Pose(0,0,.3,-.1,0)
            from unittest.mock import patch
            for n in range(20):
                io._sim_elapsed+=.02
                with patch('time.monotonic',return_value=n*wall_dt): io.command(0,0,0)
            return io.heading_bias
        self.assertAlmostEqual(simulate(.02),simulate(.2))

    def test_velocity_roundoff_and_ttl_are_separate_from_true_overspeed(self):
        from unittest.mock import patch
        io=KitIO.__new__(KitIO);io.cfg=MotionConfig();io._sim_elapsed=0
        io._command_expired=False
        io.session=types.SimpleNamespace(command=[0,0,0])
        io.control=types.SimpleNamespace(stop=lambda:io.session.command.__setitem__(slice(None),[0,0,0]))
        io.body_velocity(.5+1e-16,0,0,.1)
        self.assertLessEqual(io.session.command[0],.5)
        with self.assertRaises(ValueError):io.body_velocity(.51,0,0,.1)
        io._on_step(.11)
        self.assertTrue(io._command_expired)
        self.assertEqual(io.session.command,[0,0,0])

    def test_heading_only_command_expires_and_stops_turn(self):
        io=KitIO.__new__(KitIO);io.cfg=MotionConfig();io._sim_elapsed=0
        io._command_expired=False;io.heading_bias=0;io.last_heading=None;io.last_command_at=0
        io.session=types.SimpleNamespace(command=[0,0,0],_heading_target=None)
        io.pose=lambda:Pose(0,0,.3,0,0)
        def stop():
            io.session.command[:]=[0,0,0]
            io.session._heading_target=0
        io.control=types.SimpleNamespace(stop=stop)
        io.command(0,0,1.0)
        self.assertEqual(io.session._heading_target,1.0)
        io._on_step(.26)
        self.assertTrue(io._command_expired)
        self.assertEqual(io.session._heading_target,0)


if __name__=='__main__': unittest.main()
