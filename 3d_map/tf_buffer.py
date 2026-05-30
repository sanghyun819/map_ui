"""Minimal, dependency-free TF buffer for resolving transforms from a rosbag.

This is a pragmatic re-implementation of the small slice of tf2 that we need:
collect every transform that appears on /tf and /tf_static, then resolve the
chain between any two frames at a given time using nearest-in-time lookups.

Nearest-in-time (rather than full interpolation) is good enough for slow indoor
mapping where the robot pose changes smoothly. It keeps the code free of a ROS
install so the whole pipeline runs from plain pip packages.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field

import numpy as np


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a 4x4 homogeneous matrix for a quaternion (no translation)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    m = np.eye(4)
    m[0, 0] = 1.0 - (yy + zz)
    m[0, 1] = xy - wz
    m[0, 2] = xz + wy
    m[1, 0] = xy + wz
    m[1, 1] = 1.0 - (xx + zz)
    m[1, 2] = yz - wx
    m[2, 0] = xz - wy
    m[2, 1] = yz + wx
    m[2, 2] = 1.0 - (xx + yy)
    return m


def transform_to_matrix(translation, rotation) -> np.ndarray:
    """Build a 4x4 matrix from a geometry_msgs Transform (translation/rotation)."""
    m = _quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    m[0, 3] = translation.x
    m[1, 3] = translation.y
    m[2, 3] = translation.z
    return m


@dataclass
class _EdgeSamples:
    """Time-ordered samples for one parent->child edge."""

    stamps: list = field(default_factory=list)   # nanoseconds, sorted
    matrices: list = field(default_factory=list)  # 4x4 np.ndarray, parent_T_child

    def add(self, stamp_ns: int, matrix: np.ndarray) -> None:
        idx = bisect.bisect_left(self.stamps, stamp_ns)
        if idx < len(self.stamps) and self.stamps[idx] == stamp_ns:
            self.matrices[idx] = matrix
            return
        self.stamps.insert(idx, stamp_ns)
        self.matrices.insert(idx, matrix)

    def nearest(self, stamp_ns: int) -> np.ndarray:
        idx = bisect.bisect_left(self.stamps, stamp_ns)
        if idx == 0:
            return self.matrices[0]
        if idx >= len(self.stamps):
            return self.matrices[-1]
        before, after = self.stamps[idx - 1], self.stamps[idx]
        return self.matrices[idx - 1] if (stamp_ns - before) <= (after - stamp_ns) else self.matrices[idx]


class TfBuffer:
    """Stores parent->child edges and resolves source_T_target chains."""

    def __init__(self) -> None:
        # child_frame -> {parent_frame -> _EdgeSamples}
        self._edges: dict[str, dict[str, _EdgeSamples]] = {}
        self._static: dict[str, dict[str, np.ndarray]] = {}
        self.frames: set[str] = set()

    def add_transform(self, parent: str, child: str, stamp_ns: int, matrix: np.ndarray, static: bool = False) -> None:
        parent = parent.lstrip("/")
        child = child.lstrip("/")
        self.frames.add(parent)
        self.frames.add(child)
        if static:
            self._static.setdefault(child, {})[parent] = matrix
        else:
            self._edges.setdefault(child, {}).setdefault(parent, _EdgeSamples()).add(stamp_ns, matrix)

    def add_tf_message(self, msg, static: bool = False) -> None:
        """Ingest a tf2_msgs/TFMessage (has .transforms list of TransformStamped)."""
        for t in msg.transforms:
            stamp_ns = int(t.header.stamp.sec) * 1_000_000_000 + int(t.header.stamp.nanosec)
            mat = transform_to_matrix(t.transform.translation, t.transform.rotation)
            self.add_transform(t.header.frame_id, t.child_frame_id, stamp_ns, mat, static=static)

    # --- resolution ---------------------------------------------------------

    def _parents_of(self, child: str):
        """Yield (parent, is_static) edges where `child` is the child frame."""
        if child in self._static:
            for parent in self._static[child]:
                yield parent, True
        if child in self._edges:
            for parent in self._edges[child]:
                yield parent, False

    def _edge_matrix(self, parent: str, child: str, stamp_ns: int, is_static: bool) -> np.ndarray:
        """Return parent_T_child at the given time."""
        if is_static:
            return self._static[child][parent]
        return self._edges[child][parent].nearest(stamp_ns)

    def lookup(self, target: str, source: str, stamp_ns: int) -> np.ndarray | None:
        """Return target_T_source (4x4): maps points expressed in `source` into `target`.

        Walks the undirected frame graph from `source` to `target`. Each parent->child
        edge stores parent_T_child; traversing it child->parent multiplies parent_T_child,
        parent->child multiplies its inverse.
        """
        target = target.lstrip("/")
        source = source.lstrip("/")
        if target == source:
            return np.eye(4)

        # BFS over the undirected graph, accumulating target_T_frame on the way.
        # We search starting from `target` so the accumulated matrix is target_T_frame.
        from collections import deque

        # adjacency: frame -> list of (neighbour, matrix neighbour_T_frame? ...)
        # Easier: search from source, accumulate target side via parent chains.
        visited = {source}
        # store frame -> matrix mapping point in `frame` to `source` coordinates? No.
        # We accumulate source_T_frame, then invert at the end. Start: source_T_source = I.
        queue = deque([(source, np.eye(4))])  # (frame, source_T_frame)
        while queue:
            frame, source_T_frame = queue.popleft()
            if frame == target:
                # source_T_target -> we want target_T_source = inverse
                return np.linalg.inv(source_T_frame)
            # edges where `frame` is the child: parent_T_frame known -> frame_T_parent = inv
            for parent, is_static in self._parents_of(frame):
                if parent in visited:
                    continue
                parent_T_frame = self._edge_matrix(parent, frame, stamp_ns, is_static)
                source_T_parent = source_T_frame @ np.linalg.inv(parent_T_frame)
                visited.add(parent)
                queue.append((parent, source_T_parent))
            # edges where `frame` is the parent: parent_T_child known -> child reachable
            for child, parents in self._all_children(frame):
                if child in visited:
                    continue
                # find the edge frame(parent)->child
                parent_T_child, ok = self._child_edge_matrix(frame, child, stamp_ns)
                if not ok:
                    continue
                source_T_child = source_T_frame @ parent_T_child
                visited.add(child)
                queue.append((child, source_T_child))
        return None

    def _all_children(self, parent: str):
        """Yield (child, _) for edges where `parent` is the parent frame."""
        for child, parents in self._static.items():
            if parent in parents:
                yield child, parents
        for child, parents in self._edges.items():
            if parent in parents:
                yield child, parents

    def _child_edge_matrix(self, parent: str, child: str, stamp_ns: int):
        if child in self._static and parent in self._static[child]:
            return self._static[child][parent], True
        if child in self._edges and parent in self._edges[child]:
            return self._edges[child][parent].nearest(stamp_ns), True
        return None, False
