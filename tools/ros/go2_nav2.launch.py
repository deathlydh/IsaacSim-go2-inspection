"""Run one SLAM or Nav2 episode on the existing Go2 relay."""
import json
import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent
SITE = HERE.parents[1] / "sites/factory_hall/site.json"
SIM = {"use_sim_time": True}


def episode(context):
    mode = LaunchConfiguration("mode").perform(context)
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain not in ("42", "43"):
        raise ValueError("Set ROS_DOMAIN_ID=42 (live) or 43 (replay)")
    drive = mode == "navigation" and domain == "42"
    guard = ExecuteProcess(
        cmd=[sys.executable, str(HERE / "nav2_episode.py")] + (["--drive"] if drive else []),
        output="screen",
    )
    if mode == "mapping":
        nodes = [Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                      name="slam_toolbox", parameters=[str(HERE / "go2_slam.yaml"), SIM],
                      output="screen")]
    elif mode == "navigation":
        profile = json.loads(SITE.read_text(encoding="utf-8"))["nav2"]
        map_override = LaunchConfiguration("map").perform(context)
        initial_override = LaunchConfiguration("initial_pose").perform(context)
        if domain == "42" and (map_override or initial_override not in ("", "spawn")):
            raise ValueError("Save the live map and initial pose in the site profile")
        map_file = Path(map_override) if map_override else (
            SITE.parent / profile["map_yaml"] if profile["map_yaml"] else None)
        if map_file is None or not map_file.is_file():
            raise ValueError("Save the LiDAR map and calibrate the site first")
        if initial_override == "spawn":
            initial = profile["calibration"]["map_pose_at_factory_spawn"]
        else:
            initial = json.loads(initial_override) if initial_override else profile["initial_pose"]
        if initial is None:
            raise ValueError("Set a measured initial pose in the map frame")
        params = str(HERE / "go2_nav2.yaml")
        nodes = [
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 parameters=[SIM, {"yaml_filename": str(map_file.resolve())}], output="screen"),
            Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                 parameters=[params, {"set_initial_pose": True, "initial_pose": {
                     "x": float(initial["x"]), "y": float(initial["y"]), "z": 0.0,
                     "yaw": float(initial["yaw_rad"])}}]),
            Node(package="nav2_planner", executable="planner_server", name="planner_server",
                 parameters=[params], output="screen"),
            Node(package="nav2_controller", executable="controller_server", name="controller_server",
                 parameters=[params], output="screen", remappings=[("cmd_vel", "/go2/nav2_cmd_vel")]),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
                 parameters=[params, {
                     "default_nav_to_pose_bt_xml": str(HERE / "go2_navigate.xml"),
                     "default_nav_through_poses_bt_xml": str(HERE / "go2_navigate_through.xml"),
                 }],
                 output="screen"),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_navigation", output="screen",
                 parameters=[SIM, {"autostart": True, "node_names": [
                     "map_server", "amcl", "planner_server", "controller_server", "bt_navigator"]}]),
        ]
    else:
        raise ValueError("Mode must be mapping or navigation")
    processes = [guard, *nodes]
    return [
        *[RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=[Shutdown(reason="Go2 episode process exited")],
        )) for process in processes],
        *processes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", choices=["mapping", "navigation"]),
        DeclareLaunchArgument("map", default_value="", description="Replay-only map YAML override"),
        DeclareLaunchArgument("initial_pose", default_value="", description="spawn or replay-only JSON pose"),
        OpaqueFunction(function=episode),
    ])
