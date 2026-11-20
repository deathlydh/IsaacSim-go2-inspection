"""Attach/navigation mode/status/detach for the existing scene. Never opens a stage."""

import importlib
import os
import sys

import omni.kit.app
from omni.ext._impl import stat_cache

directory = "D:/go2-inspection/tools/isaac"
if directory not in sys.path:
    sys.path.insert(0, directory)

session = getattr(sys, "_go2_inspection_session", None)
if session is None or session.runner is None:
    raise RuntimeError("Start the Go2 session in Kit first")
if "action" not in dir():
    action = "attach"
navigation = getattr(session, "navigation", None)

if action == "attach":
    if navigation is not None:
        raise RuntimeError("Navigation is already attached")
    with_bridge = bridge if "bridge" in dir() else True
    if with_bridge:
        manager = omni.kit.app.get_app().get_extension_manager()
        if "rmw" in dir() and rmw != os.environ.get("RMW_IMPLEMENTATION"):
            if manager.is_extension_enabled("isaacsim.ros2.core") or "rclpy" in sys.modules:
                raise RuntimeError("RMW must be selected before ROS core/rclpy loads")
            os.environ["RMW_IMPLEMENTATION"] = rmw
        manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        if not manager.is_extension_enabled("isaacsim.ros2.bridge"):
            raise RuntimeError("Isaac ROS 2 bridge failed to load")
    stat_cache._stat_cache.clear()
    import command_control

    importlib.reload(command_control)
    import go2_navigation

    importlib.reload(go2_navigation)

    navigation = go2_navigation.Go2Navigation(
        session, profile if "profile" in dir() else go2_navigation.PROFILE, bridge=with_bridge)
    session.navigation = navigation
elif navigation is None:
    raise RuntimeError("Navigation is not attached")
elif action in ("manual", "ros"):
    navigation.set_mode(action)
elif action == "close":
    navigation.close()
    navigation = None
elif action != "status":
    raise ValueError(f"Unknown navigation action: {action}")

if navigation is None:
    print("Navigation detached")
else:
    print("ROS connected to Kit:", navigation.ros is not None,
          "| playing:", navigation.timeline.is_playing(), "| policy:", session.policy,
          "| owner:", navigation.control.source, "| command:", session.command.tolist())
    print("simulation seconds:", navigation.sim_ns / 1e9, "| scans:", navigation.scans,
          "| commands received:", navigation.received, "| accepted:", navigation.accepted,
          "| watchdog expirations:", navigation.expirations)
