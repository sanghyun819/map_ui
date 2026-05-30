"""Inject a URDF's fixed joints into a TfBuffer as static transforms.

rosbag2 often drops `/tf_static` (it is latched, so a recording started after the
publisher misses it). The fixed-joint mounts — lidar, cameras, base offsets — then
never reach the TF tree, leaving the sensor frame disconnected. The URDF holds
exactly those constant transforms, so we parse its `type="fixed"` joints and add
them as static edges.

Only fixed joints are injected. Movable joints (revolute/continuous/prismatic) are
left alone because robot_state_publisher already streams their live values on /tf;
injecting their zero pose would clobber the real motion.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis rpy -> 3x3 rotation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_fixed_joints(urdf_path: str):
    """Return a list of (parent, child, 4x4 parent_T_child) for every fixed joint."""
    root = ET.parse(urdf_path).getroot()
    out = []
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(v) for v in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(v) for v in origin.get("rpy").split()]
        m = np.eye(4)
        m[:3, :3] = _rpy_to_matrix(*rpy)
        m[:3, 3] = xyz
        out.append((parent.get("link"), child.get("link"), m))
    return out


def inject_urdf_static(tf_buffer, urdf_path: str) -> int:
    """Add all URDF fixed joints to the buffer as static transforms. Returns count."""
    joints = load_fixed_joints(urdf_path)
    for parent, child, matrix in joints:
        tf_buffer.add_transform(parent, child, 0, matrix, static=True)
    return len(joints)
