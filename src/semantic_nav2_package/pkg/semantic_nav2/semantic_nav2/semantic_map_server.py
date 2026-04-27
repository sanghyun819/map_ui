#!/usr/bin/env python3
"""
semantic_map_server.py
======================
ROS2 node that loads semantic_map.json produced by the Nav2 Map Editor web tool
and provides:

  Publishers
  ----------
  /semantic_map/rooms_markers   visualization_msgs/MarkerArray   (latched)
  /semantic_map/objects_markers visualization_msgs/MarkerArray   (latched)
  /semantic_map/rooms           semantic_nav2/SemanticRoom[]     → custom topic
  /semantic_map/objects         semantic_nav2/SemanticObject[]   → custom topic

  Services
  ---------
  /semantic_map/query_room             QueryRoom
  /semantic_map/query_objects_in_room  QueryObjectsInRoom
  /semantic_map/query_nearest_object   QueryNearestObject
  /semantic_map/reload                 std_srvs/Trigger   (hot-reload JSON)

Usage
-----
  ros2 run semantic_nav2 semantic_map_server.py \
      --ros-args -p map_file:=/path/to/semantic_map.json

Parameters
----------
  map_file        (str)   path to semantic_map.json
  frame_id        (str)   "map"  (default)
  publish_rate_hz (float) 1.0    periodic re-publish for RViz late-joiners
"""

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Point32, Vector3
from visualization_msgs.msg import Marker, MarkerArray

# Custom interfaces (generated from msg/ srv/ in this package)
from semantic_nav2.msg import SemanticRoom, SemanticObject
from semantic_nav2.srv import QueryRoom, QueryObjectsInRoom, QueryNearestObject


# ── colour palette (type → RGBA) ──────────────────────────────────────────────
ROOM_COLORS = {
    "bedroom":     (0.29, 0.56, 0.85, 0.35),
    "living_room": (0.48, 0.41, 0.93, 0.35),
    "kitchen":     (1.00, 0.55, 0.00, 0.35),
    "bathroom":    (0.13, 0.70, 0.67, 0.35),
    "office":      (0.24, 0.70, 0.44, 0.35),
    "corridor":    (0.83, 0.66, 0.26, 0.25),
    "storage":     (0.74, 0.56, 0.37, 0.35),
    "entrance":    (0.91, 0.30, 0.54, 0.35),
    "lab":         (0.00, 0.83, 1.00, 0.30),
    "conference":  (0.61, 0.35, 0.71, 0.35),
    "custom":      (0.67, 0.67, 0.67, 0.30),
}
OBJ_COLORS = {
    "desk":      (0.31, 0.76, 0.97, 0.85),
    "chair":     (0.51, 0.78, 0.52, 0.85),
    "table":     (1.00, 0.95, 0.46, 0.85),
    "sofa":      (0.65, 0.84, 0.65, 0.85),
    "bed":       (0.96, 0.56, 0.70, 0.85),
    "shelf":     (0.81, 0.58, 0.85, 0.85),
    "cabinet":   (0.74, 0.67, 0.65, 0.85),
    "door":      (1.00, 0.72, 0.30, 0.85),
    "window":    (0.50, 0.87, 0.92, 0.85),
    "screen":    (0.56, 0.79, 0.98, 0.85),
    "charger":   (1.00, 0.93, 0.35, 0.85),
    "plant":     (0.41, 0.94, 0.68, 0.85),
    "obstacle":  (1.00, 0.32, 0.32, 0.85),
    "custom":    (0.93, 0.93, 0.93, 0.85),
}


def _rgba(t: str, alpha_override: float | None, palette: dict) -> ColorRGBA:
    r, g, b, a = palette.get(t, (0.7, 0.7, 0.7, 0.5))
    c = ColorRGBA()
    c.r, c.g, c.b = float(r), float(g), float(b)
    c.a = float(alpha_override if alpha_override is not None else a)
    return c


def _poly_area(pts):
    """Shoelace formula – returns signed area."""
    n = len(pts)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += pts[i]["x"] * pts[j]["y"]
        area -= pts[j]["x"] * pts[i]["y"]
    return abs(area) / 2.0


def _centroid(pts):
    cx = sum(p["x"] for p in pts) / len(pts)
    cy = sum(p["y"] for p in pts) / len(pts)
    return cx, cy


def _point_in_poly(px, py, pts):
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]["x"], pts[i]["y"]
        xj, yj = pts[j]["x"], pts[j]["y"]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _rect_to_world_poly(rect: dict) -> list[dict]:
    """
    Convert editor's world_rect into polygon vertices.
    world_rect uses top-left origin from image coordinates projected to map frame,
    so +h in pixels maps to -y in world.
    """
    x = float(rect.get("x", 0.0))
    y = float(rect.get("y", 0.0))
    w = float(rect.get("w", 0.0))
    h = float(rect.get("h", 0.0))
    return [
        {"x": x, "y": y},
        {"x": x + w, "y": y},
        {"x": x + w, "y": y - h},
        {"x": x, "y": y - h},
    ]


# ── Main node ──────────────────────────────────────────────────────────────────
class SemanticMapServer(Node):

    def __init__(self):
        super().__init__("semantic_map_server")

        # Parameters
        self.declare_parameter("map_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 1.0)

        self._frame = self.get_parameter("frame_id").value
        self._map_file = self.get_parameter("map_file").value
        self._rooms: list[dict] = []
        self._objects: list[dict] = []

        # Latched QoS so RViz always gets the latest even after late-join
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self._pub_room_markers = self.create_publisher(
            MarkerArray, "/semantic_map/rooms_markers", latched)
        self._pub_obj_markers = self.create_publisher(
            MarkerArray, "/semantic_map/objects_markers", latched)

        # Services
        self.create_service(QueryRoom, "/semantic_map/query_room", self._cb_query_room)
        self.create_service(QueryObjectsInRoom, "/semantic_map/query_objects_in_room", self._cb_query_objects)
        self.create_service(QueryNearestObject, "/semantic_map/query_nearest", self._cb_query_nearest)
        self.create_service(Trigger, "/semantic_map/reload", self._cb_reload)

        # Periodic republish timer
        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / max(rate, 0.1), self._timer_publish)

        # Load map
        if self._map_file:
            self._load(self._map_file)
        else:
            self.get_logger().warn("map_file not set – waiting for /semantic_map/reload call")

    # ── Load ──────────────────────────────────────────────────────────────────
    def _load(self, path: str):
        p = Path(path)
        if not p.exists():
            self.get_logger().error(f"semantic_map.json not found: {path}")
            return
        with open(p) as f:
            data = json.load(f)
        self._rooms = data.get("rooms", [])
        self._objects = data.get("objects", [])
        self._normalise_geometry()
        self.get_logger().info(
            f"Loaded semantic map: {len(self._rooms)} rooms, {len(self._objects)} objects  ({path})"
        )
        self._publish()

    def _normalise_geometry(self):
        converted_rooms = 0
        converted_objects = 0

        for room in self._rooms:
            if room.get("world_polygon"):
                continue
            rect = room.get("world_rect")
            if isinstance(rect, dict):
                room["world_polygon"] = _rect_to_world_poly(rect)
                converted_rooms += 1

        for obj in self._objects:
            if obj.get("world_polygon"):
                continue
            rect = obj.get("world_rect")
            if isinstance(rect, dict):
                obj["world_polygon"] = _rect_to_world_poly(rect)
                converted_objects += 1

        if converted_rooms or converted_objects:
            self.get_logger().info(
                "Converted world_rect -> world_polygon "
                f"({converted_rooms} rooms, {converted_objects} objects)"
            )

    def _cb_reload(self, req, res):
        path = self.get_parameter("map_file").value
        if not path:
            res.success = False
            res.message = "map_file parameter not set"
            return res
        self._load(path)
        res.success = True
        res.message = f"Reloaded {len(self._rooms)} rooms, {len(self._objects)} objects"
        return res

    # ── Publish ───────────────────────────────────────────────────────────────
    def _timer_publish(self):
        if self._rooms or self._objects:
            self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self._pub_room_markers.publish(self._build_room_markers(stamp))
        self._pub_obj_markers.publish(self._build_obj_markers(stamp))

    def _header(self, stamp) -> Header:
        h = Header()
        h.stamp = stamp
        h.frame_id = self._frame
        return h

    # ── Room markers ──────────────────────────────────────────────────────────
    def _build_room_markers(self, stamp) -> MarkerArray:
        ma = MarkerArray()
        # Delete-all first
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.ns = "rooms"
        ma.markers.append(del_marker)

        for idx, room in enumerate(self._rooms):
            poly_w = room.get("world_polygon", [])
            if not poly_w:
                continue

            # ── Filled polygon (LINE_STRIP closed) ──
            fill = Marker()
            fill.header = self._header(stamp)
            fill.ns = "rooms"
            fill.id = idx * 3
            fill.type = Marker.LINE_STRIP
            fill.action = Marker.ADD
            fill.scale = Vector3(x=0.02, y=0.0, z=0.0)
            fill.color = _rgba(room["type"], None, ROOM_COLORS)
            fill.pose.orientation.w = 1.0
            for p in poly_w + [poly_w[0]]:   # close the loop
                pt = Point()
                pt.x, pt.y, pt.z = p["x"], p["y"], 0.05
                fill.points.append(pt)
            ma.markers.append(fill)

            # ── Translucent fill (TRIANGLE_LIST via triangle fan) ──
            cx, cy = _centroid(poly_w)
            tfill = Marker()
            tfill.header = self._header(stamp)
            tfill.ns = "rooms"
            tfill.id = idx * 3 + 1
            tfill.type = Marker.TRIANGLE_LIST
            tfill.action = Marker.ADD
            tfill.scale = Vector3(x=1.0, y=1.0, z=1.0)
            tfill.color = _rgba(room["type"], ROOM_COLORS.get(room["type"], (0,0,0,0.3))[3] * 0.5, ROOM_COLORS)
            tfill.pose.orientation.w = 1.0
            n = len(poly_w)
            for i in range(n):
                j = (i + 1) % n
                for px, py in [(cx, cy), (poly_w[i]["x"], poly_w[i]["y"]), (poly_w[j]["x"], poly_w[j]["y"])]:
                    pt = Point(); pt.x, pt.y, pt.z = px, py, 0.03
                    tfill.points.append(pt)
            ma.markers.append(tfill)

            # ── Text label ──
            txt = Marker()
            txt.header = self._header(stamp)
            txt.ns = "rooms"
            txt.id = idx * 3 + 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.text = f"{room['label']}\n[{room['type']}]"
            txt.scale.z = 0.25
            txt.color = _rgba(room["type"], 1.0, ROOM_COLORS)
            txt.pose.orientation.w = 1.0
            txt.pose.position.x = cx
            txt.pose.position.y = cy
            txt.pose.position.z = 0.3
            ma.markers.append(txt)

        return ma

    # ── Object markers ────────────────────────────────────────────────────────
    def _build_obj_markers(self, stamp) -> MarkerArray:
        ma = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.ns = "objects"
        ma.markers.append(del_marker)

        for idx, obj in enumerate(self._objects):
            base_id = idx * 3

            if obj.get("is_point") or "world_pos" in obj:
                pos = obj.get("world_pos", obj.get("pixel_pos", {"x": 0.0, "y": 0.0}))
                # Cylinder marker for point objects
                cyl = Marker()
                cyl.header = self._header(stamp)
                cyl.ns = "objects"
                cyl.id = base_id
                cyl.type = Marker.CYLINDER
                cyl.action = Marker.ADD
                cyl.scale = Vector3(x=0.15, y=0.15, z=0.4)
                cyl.color = _rgba(obj["type"], None, OBJ_COLORS)
                cyl.pose.orientation.w = 1.0
                cyl.pose.position.x = float(pos["x"])
                cyl.pose.position.y = float(pos["y"])
                cyl.pose.position.z = 0.2
                ma.markers.append(cyl)
            else:
                poly_w = obj.get("world_polygon", [])
                if not poly_w:
                    continue
                # Outline
                outline = Marker()
                outline.header = self._header(stamp)
                outline.ns = "objects"
                outline.id = base_id
                outline.type = Marker.LINE_STRIP
                outline.action = Marker.ADD
                outline.scale = Vector3(x=0.03, y=0.0, z=0.0)
                outline.color = _rgba(obj["type"], None, OBJ_COLORS)
                outline.pose.orientation.w = 1.0
                for p in poly_w + [poly_w[0]]:
                    pt = Point(); pt.x, pt.y, pt.z = p["x"], p["y"], 0.1
                    outline.points.append(pt)
                ma.markers.append(outline)

                # Fill
                cx, cy = _centroid(poly_w)
                n = len(poly_w)
                tfill = Marker()
                tfill.header = self._header(stamp)
                tfill.ns = "objects"
                tfill.id = base_id + 1
                tfill.type = Marker.TRIANGLE_LIST
                tfill.action = Marker.ADD
                tfill.scale = Vector3(x=1.0, y=1.0, z=1.0)
                tfill.color = _rgba(obj["type"], OBJ_COLORS.get(obj["type"], (0,0,0,0.5))[3] * 0.4, OBJ_COLORS)
                tfill.pose.orientation.w = 1.0
                for i in range(n):
                    j = (i + 1) % n
                    for px, py in [(cx, cy), (poly_w[i]["x"], poly_w[i]["y"]), (poly_w[j]["x"], poly_w[j]["y"])]:
                        pt = Point(); pt.x, pt.y, pt.z = px, py, 0.08
                        tfill.points.append(pt)
                ma.markers.append(tfill)

            # Text
            pos_data = obj.get("world_pos")
            if not pos_data:
                poly = obj.get("world_polygon", [])
                if poly:
                    cx, cy = _centroid(poly)
                    pos_data = {"x": cx, "y": cy}
                else:
                    pos_data = {"x": 0.0, "y": 0.0}
            txt = Marker()
            txt.header = self._header(stamp)
            txt.ns = "objects"
            txt.id = base_id + 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.text = obj["label"]
            txt.scale.z = 0.14
            txt.color = _rgba(obj["type"], 1.0, OBJ_COLORS)
            txt.pose.orientation.w = 1.0
            txt.pose.position.x = float(pos_data["x"])
            txt.pose.position.y = float(pos_data["y"])
            txt.pose.position.z = 0.5
            ma.markers.append(txt)

        return ma

    # ── Service: query_room ───────────────────────────────────────────────────
    def _cb_query_room(self, req, res):
        px, py = req.point.x, req.point.y
        for room in self._rooms:
            poly = room.get("world_polygon", [])
            if poly and _point_in_poly(px, py, poly):
                res.found = True
                res.room = self._room_to_msg(room)
                return res
        res.found = False
        return res

    # ── Service: query_objects_in_room ────────────────────────────────────────
    def _cb_query_objects(self, req, res):
        room_filter = req.room_id.strip()
        type_filter = req.object_type.strip()
        out = []
        for obj in self._objects:
            if room_filter and obj.get("room_id", "") != room_filter:
                continue
            if type_filter and obj.get("type", "") != type_filter:
                continue
            out.append(self._obj_to_msg(obj))
        res.objects = out
        return res

    # ── Service: query_nearest ────────────────────────────────────────────────
    def _cb_query_nearest(self, req, res):
        px, py = req.point.x, req.point.y
        max_r = req.max_radius_m

        best_dist = float("inf")
        best_obj = None

        for obj in self._objects:
            if req.object_type and obj.get("type") != req.object_type:
                continue
            pos = obj.get("world_pos")
            if pos is None:
                poly = obj.get("world_polygon", [])
                if not poly:
                    continue
                cx, cy = _centroid(poly)
                pos = {"x": cx, "y": cy}
            d = math.hypot(px - pos["x"], py - pos["y"])
            if d < best_dist and (max_r <= 0 or d <= max_r):
                best_dist = d
                best_obj = obj

        if best_obj:
            res.found = True
            res.object = self._obj_to_msg(best_obj)
            res.distance_m = float(best_dist)
        else:
            res.found = False
        return res

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _room_to_msg(self, room: dict) -> SemanticRoom:
        msg = SemanticRoom()
        msg.id = room.get("id", "")
        msg.type = room.get("type", "custom")
        msg.label = room.get("label", "")
        poly = room.get("world_polygon", [])
        msg.polygon_world = [Point32(x=float(p["x"]), y=float(p["y"]), z=0.0) for p in poly]
        if poly:
            cx, cy = _centroid(poly)
            msg.centroid = Point(x=cx, y=cy, z=0.0)
            msg.area_m2 = float(_poly_area(poly))
        return msg

    def _obj_to_msg(self, obj: dict) -> SemanticObject:
        msg = SemanticObject()
        msg.id = obj.get("id", "")
        msg.type = obj.get("type", "custom")
        msg.label = obj.get("label", "")
        msg.room_id = obj.get("room_id", "")
        msg.is_point = bool(obj.get("is_point") or "world_pos" in obj)
        if msg.is_point:
            p = obj.get("world_pos", {"x": 0.0, "y": 0.0})
            msg.world_pos = Point(x=float(p["x"]), y=float(p["y"]), z=0.0)
        else:
            poly = obj.get("world_polygon", [])
            msg.polygon_world = [Point32(x=float(p["x"]), y=float(p["y"]), z=0.0) for p in poly]
            if poly:
                cx, cy = _centroid(poly)
                msg.world_pos = Point(x=cx, y=cy, z=0.0)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
