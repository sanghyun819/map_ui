"""Parse sensor_msgs/PointCloud2 (and Livox CustomMsg) into numpy arrays.

Returns geometry and (optional) per-point RGB separately:
    read_cloud(msg, msgtype) -> (xyz: (N,3) float64, rgb: (N,3) float64 0..1 or None)

Works on the deserialized message objects produced by the `rosbags` library, so no
ROS install is required.
"""

from __future__ import annotations

import numpy as np

# sensor_msgs/PointField datatype enum -> numpy dtype
_PF_TO_NP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def _unpack_rgb(raw: np.ndarray, name: str) -> np.ndarray | None:
    """Unpack a packed PointCloud2 colour field into (N,3) floats in 0..1.

    RealSense/PCL pack colour as a float32 whose bits are 0x00RRGGBB (the 'rgb'
    field) or as a uint32 'rgba'. We reinterpret to uint32 and split the bytes.
    """
    col = raw[name].reshape(-1)
    u = col.astype(np.float32).view(np.uint32) if col.dtype != np.uint32 else col
    r = (u >> 16) & 0xFF
    g = (u >> 8) & 0xFF
    b = u & 0xFF
    return np.column_stack([r, g, b]).astype(np.float64) / 255.0


def read_pointcloud2(msg):
    """Return (xyz (N,3), rgb (N,3)|None). Invalid (NaN/Inf) points removed."""
    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PointCloud2 has no x/y/z fields, only: {list(fields)}")

    names, formats, offsets = [], [], []
    for f in msg.fields:
        np_dtype = _PF_TO_NP.get(f.datatype)
        if np_dtype is None:
            continue
        count = max(1, int(getattr(f, "count", 1)))
        names.append(f.name)
        formats.append((np.dtype(np_dtype), (count,)) if count > 1 else np.dtype(np_dtype))
        offsets.append(int(f.offset))

    dt = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": int(msg.point_step),
    })
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=int(msg.width) * int(msg.height))

    xyz = np.column_stack([
        raw["x"].astype(np.float64).reshape(-1),
        raw["y"].astype(np.float64).reshape(-1),
        raw["z"].astype(np.float64).reshape(-1),
    ])

    rgb = None
    color_name = next((n for n in ("rgb", "rgba") if n in raw.dtype.names), None)
    if color_name is not None:
        rgb = _unpack_rgb(raw, color_name)

    good = np.isfinite(xyz).all(axis=1)
    xyz = xyz[good]
    if rgb is not None:
        rgb = rgb[good]
    return xyz, rgb


def read_livox_custom(msg):
    """Parse livox_ros_driver(2)/CustomMsg into (xyz (N,3), None)."""
    pts = getattr(msg, "points", None)
    if not pts:
        return np.empty((0, 3)), None
    arr = np.array([(p.x, p.y, p.z) for p in pts], dtype=np.float64)
    good = np.isfinite(arr).all(axis=1)
    return arr[good], None


def read_cloud(msg, msgtype: str):
    """Dispatch on the connection msgtype string. Returns (xyz, rgb|None)."""
    if "CustomMsg" in msgtype:
        return read_livox_custom(msg)
    return read_pointcloud2(msg)
