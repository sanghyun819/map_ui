"""Colorise LiDAR points by projecting them into a camera's colour image.

Needs the camera intrinsics (sensor_msgs/CameraInfo) and the colour image stream
from the bag. For each LiDAR scan we transform its points into the camera optical
frame (via TF, done by the caller), project with the pinhole + plumb_bob model,
and sample the RGB of the image captured nearest in time.

Images are kept as raw (compressed) bytes and decoded on demand with a 1-frame
cache — scans are processed in time order so the "current" image advances
monotonically and we rarely decode twice.
"""

from __future__ import annotations

import bisect
import io

import numpy as np

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None


class Colorizer:
    def __init__(self):
        self.K = None            # 3x3 intrinsics
        self.dist = np.zeros(5)  # plumb_bob k1,k2,p1,p2,k3
        self.width = 0
        self.height = 0
        self.optical_frame = None
        self._stamps: list[int] = []
        self._raws: list[bytes] = []
        self._compressed: list[bool] = []
        self._encodings: list[str] = []
        self._cur_idx = -1
        self._cur_img = None

    # ---- ingest -----------------------------------------------------------
    def set_camera_info(self, msg) -> None:
        k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.K = k
        d = np.asarray(getattr(msg, "d", []), dtype=np.float64)
        self.dist = np.pad(d, (0, max(0, 5 - len(d))))[:5] if d.size else np.zeros(5)
        self.width = int(msg.width)
        self.height = int(msg.height)
        if getattr(msg, "header", None) is not None and msg.header.frame_id:
            self.optical_frame = msg.header.frame_id.lstrip("/")

    def add_image(self, msg, compressed: bool) -> None:
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self._stamps.append(stamp)
        self._raws.append(bytes(msg.data))
        self._compressed.append(compressed)
        self._encodings.append(getattr(msg, "format", "") if compressed else getattr(msg, "encoding", "rgb8"))
        if self.optical_frame is None and getattr(msg, "header", None) is not None:
            self.optical_frame = (msg.header.frame_id or "").lstrip("/")

    def ready(self) -> bool:
        return self.K is not None and len(self._stamps) > 0 and Image is not None

    # ---- image lookup -----------------------------------------------------
    def _image_at(self, stamp_ns: int):
        if not self._stamps:
            return None
        i = bisect.bisect_left(self._stamps, stamp_ns)
        if i <= 0:
            idx = 0
        elif i >= len(self._stamps):
            idx = len(self._stamps) - 1
        else:
            idx = i - 1 if (stamp_ns - self._stamps[i - 1]) <= (self._stamps[i] - stamp_ns) else i
        if idx != self._cur_idx:
            self._cur_img = self._decode(idx)
            self._cur_idx = idx
        return self._cur_img

    def _decode(self, idx: int) -> np.ndarray:
        raw = self._raws[idx]
        if self._compressed[idx]:
            img = Image.open(io.BytesIO(raw)).convert("RGB")
            return np.asarray(img)
        enc = self._encodings[idx]
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(self.height, self.width, -1)
        if enc.startswith("bgr"):
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr[:, :, :3])

    # ---- projection -------------------------------------------------------
    def colorize(self, xyz_cam: np.ndarray, stamp_ns: int) -> np.ndarray:
        """xyz_cam: (N,3) in the camera optical frame. Returns (N,3) rgb 0..1,
        NaN for points outside the image / behind the camera."""
        n = len(xyz_cam)
        rgb = np.full((n, 3), np.nan)
        img = self._image_at(stamp_ns)
        if img is None:
            return rgb
        H, W = img.shape[:2]

        z = xyz_cam[:, 2]
        front = z > 1e-3
        if not front.any():
            return rgb
        x = xyz_cam[front, 0] / z[front]
        y = xyz_cam[front, 1] / z[front]

        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        u = np.round(fx * xd + cx).astype(np.int64)
        v = np.round(fy * yd + cy).astype(np.int64)

        valid = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        idx_front = np.where(front)[0][valid]
        cols = img[v[valid], u[valid], :3].astype(np.float64) / 255.0
        rgb[idx_front] = cols
        return rgb
