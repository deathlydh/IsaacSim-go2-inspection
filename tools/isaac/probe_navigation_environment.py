"""Read-only environment inventory for an open Kit session."""
import importlib.util
import os
import sys

import omni.kit.app
import omni.usd

manager = omni.kit.app.get_app().get_extension_manager()

stage = omni.usd.get_context().get_stage()
print('stage:', stage.GetRootLayer().identifier)
print('ROS:', {key: os.environ.get(key) for key in (
    'ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
    'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ZENOH_SESSION_CONFIG_URI')})
for name in ('isaacsim.ros2.bridge', 'isaacsim.ros2.core'):
    print(name, manager.get_enabled_extension_id(name))
print('rclpy:', importlib.util.find_spec('rclpy'))
robot = stage.GetPrimAtPath('/World/Robot')
print('robot children:', [str(prim.GetPath()) for prim in robot.GetChildren()] if robot else [])
session = getattr(sys, '_go2_inspection_session', None)
if session and session.runner:
    art = session.runner.articulation
    print('articulation:', type(art), [name for name in dir(art) if 'velocit' in name])
