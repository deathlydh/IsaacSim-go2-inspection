"""Alternative locomotion policies for the Go2 inspection robot.

Each module builds an ``isaacsim.robot.policy.examples.RobotPolicyRunner`` that can replace the
bundled NVIDIA Go2 flat-terrain policy used by ``tools/isaac/start_go2.py``:

- ``robot_lab_go2`` -- gait option "C": the robot_lab Go2 policy shipped by rl_sar
  (weights in ``assets/third_party/rl_sar_go2``).
- ``nvidia_rough_go2`` -- gait option "B": NVIDIA Isaac Lab rough-terrain Go2 checkpoint with a
  187-ray height scan (weights in ``assets/policies/nvidia_rough_go2``).

Select one with ``start_go2.py --arg "policy='robot_lab'"`` (or ``'nvidia_rough'`` / ``'bundled'``).

Modules here are imported inside Isaac Sim (Kit python), e.g.::

    import sys
    sys.path.insert(0, "D:/go2-inspection/tools/isaac")
    from policies import robot_lab_go2
"""
