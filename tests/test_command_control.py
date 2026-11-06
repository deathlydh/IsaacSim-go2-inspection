from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools/isaac"))
from command_control import CommandControl, guard_policy_reset


class ControlTest(unittest.TestCase):
    def setUp(self):
        art = types.SimpleNamespace(get_world_poses=lambda:([[0,0,0.3]],[[1,0,0,0]]))
        self.s = types.SimpleNamespace(command=np.zeros(3),runner=types.SimpleNamespace(articulation=art),
                                       keymap={"UP":[0.5,0,0]},_heading_target=None)
        self.c = CommandControl(self.s)
        self.carb = types.ModuleType("carb")
        self.carb.input = types.ModuleType("carb.input")
        self.carb.input.KeyboardEventType = types.SimpleNamespace(KEY_PRESS=1,KEY_RELEASE=2)

    def event(self,key,kind):
        with patch.dict(sys.modules,{"carb":self.carb,"carb.input":self.carb.input}):
            self.c.on_key(types.SimpleNamespace(input=types.SimpleNamespace(name=key),type=kind))

    def test_manual_repeat_and_stale_release_do_not_accumulate(self):
        self.event("UP",1)
        self.event("UP",1)
        np.testing.assert_array_equal(self.s.command,[0.5,0,0])
        self.event("UP",2)
        self.event("UP",2)
        np.testing.assert_array_equal(self.s.command,[0,0,0])

    def test_exclusive_route_and_cancel(self):
        owner = object()
        self.c.acquire(owner)
        with self.assertRaises(RuntimeError): self.c.acquire(object())
        self.s.command[:] = [0.2,0,0]
        self.event("UP",1)
        np.testing.assert_array_equal(self.s.command,[0.2,0,0])
        self.event("ESCAPE",1)
        self.assertTrue(self.c.cancel_requested)
        np.testing.assert_array_equal(self.s.command,[0,0,0])
        self.c.release(owner)
        self.assertEqual(self.c.source,"manual")
        self.event("UP",2)
        np.testing.assert_array_equal(self.s.command,[0,0,0])

    def test_manual_escape_does_not_restore_held_velocity_on_next_key(self):
        self.s.keymap["LEFT"] = [0,0.5,0]
        self.event("UP",1)
        self.event("ESCAPE",1)
        self.event("LEFT",1)
        np.testing.assert_array_equal(self.s.command,[0,0.5,0])

    def test_live_legacy_runner_cannot_reset_pose_during_route(self):
        resets = []
        self.s.runner.restart_from_default_state = lambda cmd:resets.append(cmd)
        guard_policy_reset(self.s,self.c)
        self.s.runner.restart_from_default_state([0,0,0])
        self.assertEqual(len(resets),1)
        self.c.acquire(object())
        self.s.command[:] = [0.5,0,0]
        with self.assertRaisesRegex(RuntimeError,"policy_reset_forbidden"):
            self.s.runner.restart_from_default_state([0,0,0])
        self.assertEqual(len(resets),1)
        np.testing.assert_array_equal(self.s.command,[0,0,0])


if __name__ == "__main__": unittest.main()
