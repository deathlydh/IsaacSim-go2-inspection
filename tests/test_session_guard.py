"""Test the real startup physics callback source without importing Kit."""
import ast
from pathlib import Path
import types
import unittest
import numpy as np


class SessionGuardTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1]/"tools/isaac/start_go2.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == "Go2Session")
        method = next(n for n in cls.body if getattr(n,"name",None) == "_on_physics_step")
        ns = {"np":np,"COMMAND_LIMITS":np.ones(3),"carb":types.SimpleNamespace(log_error=lambda *args:None)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),"exec"),ns)
        self.step = ns["_on_physics_step"]
        self.called = []
        art = types.SimpleNamespace(is_physics_tensor_entity_valid=lambda:True)
        runner = types.SimpleNamespace(articulation=art,step=lambda *args:self.called.append("step"),
                                       restart_from_default_state=lambda *args:self.called.append("reset"))
        self.s = types.SimpleNamespace(runner=runner,failed=False,ready=True,command=np.ones(3),
                                       control=types.SimpleNamespace(source="route"),_hold_heading=lambda runner,cmd:cmd)

    def test_route_refuses_physics_reset(self):
        self.s.runner.articulation.is_physics_tensor_entity_valid = lambda:False
        self.step(self.s,0.02,None)
        self.assertEqual(self.called,[])
        self.assertTrue(self.s.failed)
        np.testing.assert_array_equal(self.s.command,[0,0,0])

    def test_route_not_ready_refuses_reset(self):
        self.s.ready = False
        self.step(self.s,0.02,None)
        self.assertEqual(self.called,[])
        np.testing.assert_array_equal(self.s.command,[0,0,0])

    def test_healthy_route_only_steps_existing_policy(self):
        self.step(self.s,0.02,None)
        self.assertEqual(self.called,["step"])

    def test_manual_reset_does_not_use_previous_episode_heading(self):
        self.s.control.source = "manual"
        self.s.ready = False
        self.s._heading_target = -1.3
        self.s.command[:] = 0
        self.s._hold_heading = lambda *args: self.fail("must not hold heading before reset")
        def reset(cmd):
            np.testing.assert_array_equal(cmd,[0,0,0])
            self.assertIsNone(self.s._heading_target)
            self.called.append("reset")
        self.s.runner.restart_from_default_state = reset
        self.step(self.s,0.02,None)
        self.assertTrue(self.s.ready)
        self.assertEqual(self.called,["reset"])

    def test_policy_exception_zeros_command(self):
        def fail(*args): raise RuntimeError("policy_failed")
        self.s.runner.step = fail
        self.step(self.s,0.02,None)
        self.assertTrue(self.s.failed)
        np.testing.assert_array_equal(self.s.command,[0,0,0])


if __name__ == "__main__": unittest.main()
