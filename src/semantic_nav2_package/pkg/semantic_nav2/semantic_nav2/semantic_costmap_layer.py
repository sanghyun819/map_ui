#!/usr/bin/env python3
"""
semantic_costmap_layer.py
=========================
Subscribes to /semantic_map/objects_markers and re-publishes a
nav_msgs/OccupancyGrid overlay that Nav2's static layer can merge,
OR acts as a standalone node that sends costmap update messages.

For a proper Nav2 Python costmap plugin you would inherit from
nav2_costmap_2d.CostmapLayer – this file shows the standalone
OccupancyGrid approach which is drop-in compatible with Nav2's
static_layer when set as the "map_topic".

Parameters
----------
  semantic_map_file  (str)   path to semantic_map.json
  base_map_yaml      (str)   path to the .yaml that describes the PGM
  inflate_types      (str)   comma-separated object types to add as
                             lethal obstacles, e.g. "obstacle,cabinet"
                             empty = only objects with type "obstacle"
  frame_id           (str)   "map"

Topic published
---------------
  /semantic_costmap   nav_msgs/OccupancyGrid  (latched)
"""

import json
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import numpy as np

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid


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


class SemanticCostmapLayer(Node):

    def __init__(self):
        super().__init__("semantic_costmap_layer")

        self.declare_parameter("semantic_map_file", "")
        self.declare_parameter("base_map_yaml", "")
        self.declare_parameter("inflate_types", "obstacle")
        self.declare_parameter("frame_id", "map")

        self._frame = self.get_parameter("frame_id").value
        inflate_str = self.get_parameter("inflate_types").value
        self._inflate_types = set(t.strip() for t in inflate_str.split(",") if t.strip())

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(OccupancyGrid, "/semantic_costmap", latched)

        sem_file = self.get_parameter("semantic_map_file").value
        map_yaml  = self.get_parameter("base_map_yaml").value

        if sem_file and map_yaml:
            self._build_and_publish(sem_file, map_yaml)
        else:
            self.get_logger().warn(
                "semantic_map_file or base_map_yaml not provided – costmap not published"
            )

    def _build_and_publish(self, sem_file: str, map_yaml_file: str):
        # ── Load base map YAML ────────────────────────────────────────────────
        with open(map_yaml_file) as f:
            yaml_data = yaml.safe_load(f)

        resolution = float(yaml_data.get("resolution", 0.05))
        origin = yaml_data.get("origin", [0.0, 0.0, 0.0])
        ox, oy = float(origin[0]), float(origin[1])

        # Determine map size from the PGM
        pgm_path = Path(map_yaml_file).parent / yaml_data["image"]
        width, height = self._pgm_size(pgm_path)

        # ── Load semantic JSON ────────────────────────────────────────────────
        with open(sem_file) as f:
            sem = json.load(f)
        objects = sem.get("objects", [])
        self._normalise_objects(objects)

        # Start with clear grid (0 = free)
        grid = np.zeros(height * width, dtype=np.int8)

        for obj in objects:
            obj_type = obj.get("type", "")
            if obj_type not in self._inflate_types:
                continue

            if "world_polygon" in obj:
                poly = obj["world_polygon"]
                bbox = self._poly_bbox(poly)
                # Rasterise polygon into costmap cells
                x0c = max(0, int((bbox["xmin"] - ox) / resolution))
                y0c = max(0, int((bbox["ymin"] - oy) / resolution))
                x1c = min(width  - 1, int((bbox["xmax"] - ox) / resolution) + 1)
                y1c = min(height - 1, int((bbox["ymax"] - oy) / resolution) + 1)
                for row in range(y0c, y1c + 1):
                    for col in range(x0c, x1c + 1):
                        wx = ox + col * resolution + resolution / 2
                        wy = oy + row * resolution + resolution / 2
                        if _point_in_poly(wx, wy, poly):
                            grid[row * width + col] = 100  # lethal

            elif "world_pos" in obj:
                pos = obj["world_pos"]
                col = int((pos["x"] - ox) / resolution)
                row = int((pos["y"] - oy) / resolution)
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        r, c = row + dr, col + dc
                        if 0 <= r < height and 0 <= c < width:
                            grid[r * width + c] = 100

        # ── Publish OccupancyGrid ─────────────────────────────────────────────
        msg = OccupancyGrid()
        msg.header = Header(frame_id=self._frame, stamp=self.get_clock().now().to_msg())
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.tolist()

        self._pub.publish(msg)
        n_lethal = int(np.sum(grid == 100))
        self.get_logger().info(
            f"Semantic costmap published: {width}×{height} cells, {n_lethal} lethal cells "
            f"from types {self._inflate_types}"
        )

    def _normalise_objects(self, objects: list[dict]):
        converted = 0
        for obj in objects:
            if obj.get("world_polygon"):
                continue
            rect = obj.get("world_rect")
            if isinstance(rect, dict):
                obj["world_polygon"] = _rect_to_world_poly(rect)
                converted += 1
        if converted:
            self.get_logger().info(f"Converted world_rect -> world_polygon for {converted} objects")

    @staticmethod
    def _pgm_size(path: Path):
        with open(path, "rb") as f:
            assert f.readline().strip() in (b"P5", b"P2")
            while True:
                line = f.readline().strip()
                if not line.startswith(b"#"):
                    break
            w, h = map(int, line.split())
        return w, h

    @staticmethod
    def _poly_bbox(poly):
        xs = [p["x"] for p in poly]
        ys = [p["y"] for p in poly]
        return {"xmin": min(xs), "xmax": max(xs), "ymin": min(ys), "ymax": max(ys)}


def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmapLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
