"""
semantic_nav2.launch.py
=======================
Launches:
  1. Nav2 (nav2_bringup) with the map from the editor
  2. semantic_map_server  – loads semantic_map.json, publishes markers + services
  3. waypoint_navigator   – loads waypoints.yaml, drives Nav2
  4. semantic_costmap_layer (optional) – adds object obstacles to costmap

Usage
-----
  ros2 launch semantic_nav2 semantic_nav2.launch.py \
      map_yaml:=/path/to/map.yaml \
      semantic_map:=/path/to/semantic_map.json \
      waypoints:=/path/to/waypoints.yaml \
      nav2_params:=/path/to/nav2_params.yaml

Arguments (all have defaults that assume files in ~/maps/)
----------------------------------------------------------
  map_yaml        – base map YAML for Nav2's map_server
  semantic_map    – semantic_map.json from the web editor
  waypoints       – waypoints.yaml from the web editor
  nav2_params     – your Nav2 params YAML
  auto_start_nav  – "true" to begin waypoint navigation immediately
  loop_waypoints  – "true" to loop indefinitely
  add_semantic_costmap – "true" to publish /semantic_costmap as extra layer
  inflate_types   – comma list, default "obstacle"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("map_yaml",       default_value=os.path.expanduser("~/maps/map.yaml")),
        DeclareLaunchArgument("semantic_map",   default_value=os.path.expanduser("~/maps/semantic_map.json")),
        DeclareLaunchArgument("waypoints",      default_value=os.path.expanduser("~/maps/waypoints.yaml")),
        DeclareLaunchArgument("nav2_params",    default_value=PathJoinSubstitution([
                                                    FindPackageShare("semantic_nav2"), "config", "nav2_params.yaml"])),
        DeclareLaunchArgument("auto_start_nav", default_value="false"),
        DeclareLaunchArgument("loop_waypoints", default_value="false"),
        DeclareLaunchArgument("add_semantic_costmap", default_value="false"),
        DeclareLaunchArgument("inflate_types",  default_value="obstacle"),
        DeclareLaunchArgument("use_sim_time",   default_value="false"),
    ]

    use_sim_time      = LaunchConfiguration("use_sim_time")
    map_yaml          = LaunchConfiguration("map_yaml")
    semantic_map      = LaunchConfiguration("semantic_map")
    waypoints_file    = LaunchConfiguration("waypoints")
    nav2_params       = LaunchConfiguration("nav2_params")
    auto_start_nav    = LaunchConfiguration("auto_start_nav")
    loop_waypoints    = LaunchConfiguration("loop_waypoints")
    add_sem_costmap   = LaunchConfiguration("add_semantic_costmap")
    inflate_types     = LaunchConfiguration("inflate_types")

    # ── Nav2 bringup ──────────────────────────────────────────────────────────
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":          map_yaml,
            "params_file":  nav2_params,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ── Semantic map server ────────────────────────────────────────────────────
    semantic_server = Node(
        package="semantic_nav2",
        executable="semantic_map_server.py",
        name="semantic_map_server",
        output="screen",
        parameters=[{
            "map_file":        semantic_map,
            "frame_id":        "map",
            "publish_rate_hz": 1.0,
            "use_sim_time":    use_sim_time,
        }],
    )

    # ── Waypoint navigator ────────────────────────────────────────────────────
    wp_navigator = Node(
        package="semantic_nav2",
        executable="waypoint_navigator.py",
        name="waypoint_navigator",
        output="screen",
        parameters=[{
            "waypoints_file": waypoints_file,
            "auto_start":     auto_start_nav,
            "loop":           loop_waypoints,
            "frame_id":       "map",
            "use_sim_time":   use_sim_time,
        }],
    )

    # ── Semantic costmap layer (optional) ─────────────────────────────────────
    sem_costmap = Node(
        package="semantic_nav2",
        executable="semantic_costmap_layer.py",
        name="semantic_costmap_layer",
        output="screen",
        condition=IfCondition(add_sem_costmap),
        parameters=[{
            "semantic_map_file": semantic_map,
            "base_map_yaml":     map_yaml,
            "inflate_types":     inflate_types,
            "frame_id":          "map",
            "use_sim_time":      use_sim_time,
        }],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[semantic_nav2] Starting Nav2 + Semantic Map Server + Waypoint Navigator"),
        nav2_launch,
        semantic_server,
        wp_navigator,
        sem_costmap,
    ])
