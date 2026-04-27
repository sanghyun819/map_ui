#!/usr/bin/env python3
"""
waypoint_navigator.py
=====================
Loads the waypoints.yaml exported by the Nav2 Map Editor and drives the robot
through them using Nav2's FollowWaypoints action server.

Usage
-----
  ros2 run semantic_nav2 waypoint_navigator.py \
      --ros-args \
      -p waypoints_file:=/path/to/waypoints.yaml \
      -p auto_start:=false

Parameters
----------
  waypoints_file  (str)   path to waypoints.yaml from the editor
  auto_start      (bool)  start navigating immediately on launch (default: false)
  loop            (bool)  loop through waypoints forever            (default: false)
  frame_id        (str)   "map"

Services (callable from CLI or another node)
--------------------------------------------
  /waypoint_navigator/start   std_srvs/Trigger   begin navigation
  /waypoint_navigator/stop    std_srvs/Trigger   cancel navigation
  /waypoint_navigator/reload  std_srvs/Trigger   reload waypoints file

Topics
------
  /waypoint_navigator/status  std_msgs/String     current status
  /waypoint_navigator/markers visualization_msgs/MarkerArray  (waypoint arrows in RViz)
"""

import math
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Header, ColorRGBA
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import FollowWaypoints


def _yaw_to_quat(theta: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(theta / 2.0)
    q.w = math.cos(theta / 2.0)
    return q


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__("waypoint_navigator")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("loop", False)
        self.declare_parameter("frame_id", "map")

        self._frame = self.get_parameter("frame_id").value
        self._loop = self.get_parameter("loop").value
        self._waypoints: list[dict] = []   # raw dicts from YAML
        self._nav_handle = None
        self._navigating = False

        # Action client
        self._ac = ActionClient(self, FollowWaypoints, "follow_waypoints")

        # Publishers
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status = self.create_publisher(String, "/waypoint_navigator/status", 10)
        self._pub_markers = self.create_publisher(MarkerArray, "/waypoint_navigator/markers", latched)

        # Services
        self.create_service(Trigger, "/waypoint_navigator/start", self._cb_start)
        self.create_service(Trigger, "/waypoint_navigator/stop", self._cb_stop)
        self.create_service(Trigger, "/waypoint_navigator/reload", self._cb_reload)

        # Load
        wp_file = self.get_parameter("waypoints_file").value
        if wp_file:
            self._load(wp_file)

        if self.get_parameter("auto_start").value and self._waypoints:
            self._start_navigation()

    # ── Load YAML ─────────────────────────────────────────────────────────────
    def _load(self, path: str):
        p = Path(path)
        if not p.exists():
            self.get_logger().error(f"waypoints.yaml not found: {path}")
            return
        with open(p) as f:
            data = yaml.safe_load(f)

        raw = data.get("waypoints", [])
        self._waypoints = raw
        self.get_logger().info(f"Loaded {len(self._waypoints)} waypoints from {path}")
        self._publish_markers()
        self._pub_status.publish(String(data=f"Loaded {len(self._waypoints)} waypoints"))

    # ── Build PoseStamped list ────────────────────────────────────────────────
    def _build_poses(self) -> list[PoseStamped]:
        stamp = self.get_clock().now().to_msg()
        poses = []
        for wp in self._waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self._frame
            ps.header.stamp = stamp
            ps.pose.position.x = float(wp["x"])
            ps.pose.position.y = float(wp["y"])
            ps.pose.position.z = 0.0
            ps.pose.orientation = _yaw_to_quat(float(wp.get("theta", 0.0)))
            poses.append(ps)
        return poses

    # ── Start / Stop ──────────────────────────────────────────────────────────
    def _start_navigation(self):
        if not self._waypoints:
            self.get_logger().warn("No waypoints loaded")
            return
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowWaypoints action server not available!")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self._build_poses()

        self.get_logger().info(f"Sending {len(goal.poses)} waypoints to Nav2...")
        self._pub_status.publish(String(data=f"Navigating: 0/{len(goal.poses)}"))

        send_future = self._ac.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)
        self._navigating = True

    def _goal_response_cb(self, future):
        self._nav_handle = future.result()
        if not self._nav_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self._navigating = False
            return
        self.get_logger().info("Goal accepted – robot is navigating")
        result_future = self._nav_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        current = fb.current_waypoint
        total = len(self._waypoints)
        self.get_logger().info(f"Waypoint {current + 1}/{total}")
        self._pub_status.publish(String(data=f"Navigating: {current + 1}/{total}"))

    def _result_cb(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        total = len(self._waypoints)
        reached = total - len(missed)
        self._navigating = False

        if missed:
            self.get_logger().warn(
                f"Navigation complete: {reached}/{total} reached. "
                f"Missed indices: {list(missed)}"
            )
        else:
            self.get_logger().info(f"Navigation complete: all {total} waypoints reached!")

        self._pub_status.publish(
            String(data=f"Done: {reached}/{total} reached")
        )

        if self._loop and not missed:
            self.get_logger().info("Loop mode: restarting navigation")
            self._start_navigation()

    # ── Service callbacks ─────────────────────────────────────────────────────
    def _cb_start(self, req, res):
        if self._navigating:
            res.success = False
            res.message = "Already navigating"
            return res
        self._start_navigation()
        res.success = True
        res.message = f"Navigation started: {len(self._waypoints)} waypoints"
        return res

    def _cb_stop(self, req, res):
        if self._nav_handle and self._navigating:
            self._nav_handle.cancel_goal_async()
            self._navigating = False
            self._pub_status.publish(String(data="Cancelled"))
            res.success = True
            res.message = "Navigation cancelled"
        else:
            res.success = False
            res.message = "Not currently navigating"
        return res

    def _cb_reload(self, req, res):
        wp_file = self.get_parameter("waypoints_file").value
        if not wp_file:
            res.success = False
            res.message = "waypoints_file parameter not set"
            return res
        self._load(wp_file)
        res.success = True
        res.message = f"Reloaded {len(self._waypoints)} waypoints"
        return res

    # ── RViz markers ──────────────────────────────────────────────────────────
    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Delete previous
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.ns = "waypoints"
        ma.markers.append(del_m)

        for i, wp in enumerate(self._waypoints):
            x, y = float(wp["x"]), float(wp["y"])
            theta = float(wp.get("theta", 0.0))

            # Arrow
            arrow = Marker()
            arrow.header = Header(frame_id=self._frame, stamp=stamp)
            arrow.ns = "waypoints"
            arrow.id = i * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.1
            arrow.pose.orientation = _yaw_to_quat(theta)
            arrow.scale = Vector3(x=0.4, y=0.08, z=0.08)
            arrow.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
            ma.markers.append(arrow)

            # Number label
            txt = Marker()
            txt.header = Header(frame_id=self._frame, stamp=stamp)
            txt.ns = "waypoints"
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.text = f"{i+1}: {wp.get('label', f'WP{i+1}')}"
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = 0.4
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.18
            txt.color = ColorRGBA(r=1.0, g=0.9, b=0.3, a=1.0)
            ma.markers.append(txt)

        self._pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
