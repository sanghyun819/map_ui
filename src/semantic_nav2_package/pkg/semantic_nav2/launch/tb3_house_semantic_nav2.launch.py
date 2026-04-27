#!/usr/bin/env python3
"""
tb3_house_semantic_nav2.launch.py
=================================
Runs TurtleBot3 House Gazebo simulation and semantic_nav2 stack together.

Launches
---------
  1) turtlebot3_gazebo/launch/turtlebot3_house.launch.py
  2) semantic_nav2/launch/semantic_nav2.launch.py (Nav2 + semantic nodes)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "turtlebot3_model",
            default_value=os.environ.get("TURTLEBOT3_MODEL", "burger"),
            description="TurtleBot3 model (burger/waffle/waffle_pi)",
        ),
        DeclareLaunchArgument("map_yaml", default_value=os.path.expanduser("~/maps/map.yaml")),
        DeclareLaunchArgument("semantic_map", default_value=os.path.expanduser("~/maps/semantic_map.json")),
        DeclareLaunchArgument("waypoints", default_value=os.path.expanduser("~/maps/waypoints.yaml")),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("semantic_nav2"), "config", "nav2_params.yaml"]
            ),
        ),
        DeclareLaunchArgument("auto_start_nav", default_value="false"),
        DeclareLaunchArgument("loop_waypoints", default_value="false"),
        DeclareLaunchArgument("add_semantic_costmap", default_value="false"),
        DeclareLaunchArgument("inflate_types", default_value="obstacle"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    turtlebot3_model = LaunchConfiguration("turtlebot3_model")
    map_yaml = LaunchConfiguration("map_yaml")
    semantic_map = LaunchConfiguration("semantic_map")
    waypoints = LaunchConfiguration("waypoints")
    nav2_params = LaunchConfiguration("nav2_params")
    auto_start_nav = LaunchConfiguration("auto_start_nav")
    loop_waypoints = LaunchConfiguration("loop_waypoints")
    add_semantic_costmap = LaunchConfiguration("add_semantic_costmap")
    inflate_types = LaunchConfiguration("inflate_types")
    use_sim_time = LaunchConfiguration("use_sim_time")

    set_tb3_model = SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot3_model)

    tb3_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("turtlebot3_gazebo"), "launch", "turtlebot3_house.launch.py"]
            )
        )
    )

    semantic_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("semantic_nav2"), "launch", "semantic_nav2.launch.py"])
        ),
        launch_arguments={
            "map_yaml": map_yaml,
            "semantic_map": semantic_map,
            "waypoints": waypoints,
            "nav2_params": nav2_params,
            "auto_start_nav": auto_start_nav,
            "loop_waypoints": loop_waypoints,
            "add_semantic_costmap": add_semantic_costmap,
            "inflate_types": inflate_types,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            *args,
            LogInfo(msg="[semantic_nav2] Launching TurtleBot3 House Gazebo + Nav2 + semantic stack"),
            set_tb3_model,
            tb3_house,
            semantic_nav2,
        ]
    )
