#!/usr/bin/env python3
"""Build a 3D height map from a Livox (or any PointCloud2) topic in a rosbag,
aligned to a 2D occupancy grid produced by slam_toolbox / Nav2.

The idea
--------
slam_toolbox gives you a 2D occupancy grid in the `map` frame. The same bag
contains the lidar cloud plus the /tf tree that localises every scan in `map`.
So we can:

  1. read the map YAML to know resolution / origin / size (so outputs line up
     pixel-for-pixel with the 2D map),
  2. replay /tf (+ /tf_static) to know map_T_lidar at each scan time,
  3. transform every lidar cloud into `map` and accumulate the points,
  4. rasterise them into a 2.5D elevation grid (per-cell height statistic) that
     sits exactly on top of the occupancy grid.

Everything runs from plain pip packages (see requirements.txt) — no sourced ROS.

Works with ANY map + ANY bag: pass --bag and --map; topic and frames are
auto-detected but can be overridden.

Outputs (written to --out, default ./out):
  cloud_map.pcd        merged point cloud in the map frame (if open3d present, else .ply/.xyz)
  elevation.npy        HxW float32 grid of cell heights (NaN where no points)
  elevation.tif        same grid as GeoTIFF-less 32-bit TIFF (if Pillow supports)
  elevation_color.png  colourised height map for quick inspection
  elevation_meta.json  resolution/origin/size + height range, for downstream use
"""

from __future__ import annotations

import argparse
import bisect
import json
import sys
from pathlib import Path

import numpy as np

from pointcloud2 import read_cloud
from tf_buffer import TfBuffer
from urdf_static import inject_urdf_static
from colorize import Colorizer

try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
except ImportError:  # pragma: no cover
    sys.exit("rosbags not installed. Run: pip install -r requirements.txt")


def make_typestore(distro: str):
    """Return a typestore for bags that ship no embedded type definitions.

    rosbag2 'version 5' bags often omit message definitions, so AnyReader needs a
    default typestore. All topics here use stock ROS 2 message types, so any
    recent distro store works.
    """
    key = f"ROS2_{distro.upper()}"
    try:
        return get_typestore(getattr(Stores, key))
    except AttributeError:
        avail = ", ".join(s.name.replace("ROS2_", "").lower() for s in Stores if s.name.startswith("ROS2_"))
        sys.exit(f"Unknown --ros-distro {distro!r}. Available: {avail}")


# ---------------------------------------------------------------------------
# Map metadata
# ---------------------------------------------------------------------------

def load_map_yaml(yaml_path: Path):
    """Return (resolution, origin_xy, width, height) from a Nav2 map YAML.

    Reads the PGM/PNG header to get width/height without a full image decode
    where possible, but falls back to Pillow.
    """
    import yaml as _yaml

    with open(yaml_path) as f:
        meta = _yaml.safe_load(f)

    resolution = float(meta["resolution"])
    origin = [float(meta["origin"][0]), float(meta["origin"][1])]
    image_path = (yaml_path.parent / meta["image"]).resolve()

    from PIL import Image

    with Image.open(image_path) as im:
        width, height = im.size
    return resolution, origin, width, height, image_path


# ---------------------------------------------------------------------------
# Bag inspection
# ---------------------------------------------------------------------------

def pick_lidar_topic(reader, requested: str | None) -> tuple[str, str]:
    """Return (topic, msgtype). Auto-pick the PointCloud2/Livox topic if not given."""
    conns = {c.topic: c.msgtype for c in reader.connections}
    if requested:
        if requested not in conns:
            sys.exit(f"Topic {requested!r} not in bag. Available:\n  " + "\n  ".join(sorted(conns)))
        return requested, conns[requested]

    candidates = [
        (t, mt) for t, mt in conns.items()
        if "PointCloud2" in mt or "CustomMsg" in mt
    ]
    if not candidates:
        sys.exit("No PointCloud2/Livox topic found. Pass --lidar-topic explicitly.\n"
                 "Topics:\n  " + "\n  ".join(f"{t}  ({mt})" for t, mt in sorted(conns.items())))
    # Prefer one with 'livox' in the name, else the first.
    candidates.sort(key=lambda tm: (0 if "livox" in tm[0].lower() else 1, tm[0]))
    return candidates[0]


def find_tf_topics(reader) -> tuple[str | None, str | None]:
    tf, tf_static = None, None
    for c in reader.connections:
        if c.msgtype.endswith("TFMessage"):
            if c.topic.rstrip("/").endswith("tf_static"):
                tf_static = c.topic
            elif c.topic.rstrip("/").endswith("tf"):
                tf = c.topic
    return tf, tf_static


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="Path to rosbag2 directory (or .mcap/.db3 file)")
    ap.add_argument("--map", help="Path to Nav2 map .yaml (aligns output to the 2D grid). "
                                  "If omitted, the grid is auto-sized to the cloud bounds.")
    ap.add_argument("--out", default="out", help="Output directory (default: ./out)")
    ap.add_argument("--lidar-topic", help="PointCloud2/Livox topic (auto-detected if omitted)")
    ap.add_argument("--map-frame", default="map", help="Target/world frame (default: map)")
    ap.add_argument("--ros-distro", default="humble",
                    help="Typestore for bags without embedded type defs (default: humble)")
    ap.add_argument("--urdf", help="Robot URDF — injects its fixed joints as static TF. "
                                   "Use this when the bag has no /tf_static (sensor mount missing).")
    ap.add_argument("--resolution", type=float, help="Grid resolution [m] when --map is not given (default 0.05)")
    ap.add_argument("--stat", choices=["max", "mean", "min", "median"], default="max",
                    help="Per-cell height statistic (default: max)")
    ap.add_argument("--z-min", type=float, default=-1e9, help="Drop points below this z in map frame")
    ap.add_argument("--z-max", type=float, default=1e9, help="Drop points above this z in map frame")
    ap.add_argument("--range-max", type=float, default=0.0,
                    help="Drop lidar points farther than this [m] from the sensor (0 = keep all)")
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="Voxel-downsample the merged cloud to this leaf size [m] (0 = off)")
    ap.add_argument("--stride", type=int, default=1, help="Use every Nth lidar message (speed/size tradeoff)")
    ap.add_argument("--extra-stride", type=int, default=0,
                    help="Separate stride for --extra-cloud topics (depth is dense; default = 5x --stride)")
    # ICP registration refinement (corrects SLAM drift; does NOT remove moving objects)
    ap.add_argument("--icp", action="store_true", help="Refine each scan onto the running map with point-to-point ICP")
    ap.add_argument("--icp-voxel", type=float, default=0.1, help="Voxel leaf [m] for ICP scan/map (default 0.1)")
    ap.add_argument("--icp-max-dist", type=float, default=0.3, help="Max ICP correspondence distance [m] (default 0.3)")
    ap.add_argument("--icp-iters", type=int, default=20, help="Max ICP iterations per scan (default 20)")
    ap.add_argument("--icp-rebuild", type=int, default=5, help="Rebuild map KD-tree every N scans (default 5)")
    # Dynamic-object removal (THIS is what drops a person following the robot)
    ap.add_argument("--min-hits", type=int, default=0,
                    help="Drop voxels observed fewer than N times — removes transient/moving points (0 = off)")
    ap.add_argument("--dyn-voxel", type=float, default=0.1, help="Voxel leaf [m] for --min-hits counting (default 0.1)")
    ap.add_argument("--save-raw", action="store_true", help="Also save raw accumulated cloud (pre-filter) as raw_cloud.npy")
    # Robot self-removal: drop points inside the robot footprint (the robot hits its own lidar)
    ap.add_argument("--footprint", help='Robot footprint polygon in the footprint frame, e.g. '
                    '"[[0.097,-0.30],[0.097,0.30],[-0.260,0.30],[-0.563,0.15],[-0.563,-0.15],[-0.260,-0.30]]"')
    ap.add_argument("--footprint-frame", default="base_nav", help="Frame the footprint is defined in (default: base_nav)")
    ap.add_argument("--footprint-margin", type=float, default=0.0,
                    help="Grow the footprint outward by this many metres before removing (default 0)")
    # Colorisation: project LiDAR onto a camera image, and/or fuse already-coloured clouds
    ap.add_argument("--color-image", help="Colour image topic (CompressedImage/Image) to paint LiDAR with")
    ap.add_argument("--color-info", help="CameraInfo topic for --color-image (intrinsics)")
    ap.add_argument("--color-frame", help="Camera optical frame (else taken from camera_info/image header)")
    ap.add_argument("--extra-cloud", action="append", default=[],
                    help="Extra PointCloud2 topic to fuse, e.g. /camera/.../depth/color/points (repeatable)")
    ap.add_argument("--grid-res", type=float, default=0.0,
                    help="Height-grid resolution [m] for elevation/height_view3d outputs, finer than the "
                         "2D map (e.g. 0.02). 0 = use the map's resolution. Same origin, so still aligned.")
    ap.add_argument("--view3d-cloud", type=int, default=0,
                    help="Also export cloud_view3d.json: the FULL 3D coloured cloud (keeps vertical detail, "
                         "not just the top surface) downsampled to this many points for map_ui. e.g. 300000.")
    ap.add_argument("--max-points", type=int, default=0,
                    help="Stop accumulating after this many raw points (0 = no limit)")
    args = ap.parse_args()

    bag_path = Path(args.bag)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- map metadata -----------------------------------------------------
    map_res = args.resolution or 0.05
    map_origin = None
    map_size = None
    if args.map:
        map_res, map_origin, mw, mh, _img = load_map_yaml(Path(args.map))
        map_size = (mw, mh)
        print(f"[map] {args.map}: {mw}x{mh} cells @ {map_res} m, origin {map_origin}")
    else:
        print(f"[map] no --map given; grid auto-sized @ {map_res} m")

    # ---- read bag ---------------------------------------------------------
    typestore = make_typestore(args.ros_distro)
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        lidar_topic, lidar_type = pick_lidar_topic(reader, args.lidar_topic)
        tf_topic, tf_static_topic = find_tf_topics(reader)
        print(f"[bag] lidar topic: {lidar_topic}  ({lidar_type})")
        print(f"[bag] tf: {tf_topic}   tf_static: {tf_static_topic}")
        if tf_topic is None and tf_static_topic is None:
            sys.exit("No /tf in bag — cannot localise scans into the map frame.")

        tf_buf = TfBuffer()
        if args.urdf:
            n = inject_urdf_static(tf_buf, args.urdf)
            print(f"[tf]  injected {n} fixed joints from URDF as static transforms")
        # First pass: ingest the whole TF history so lookups can interpolate
        # backwards/forwards regardless of message ordering.
        tf_conns = [c for c in reader.connections if c.topic in (tf_topic, tf_static_topic)]
        for conn, _ts, raw in reader.messages(connections=tf_conns):
            msg = reader.deserialize(raw, conn.msgtype)
            tf_buf.add_tf_message(msg, static=(conn.topic == tf_static_topic))
        print(f"[tf]  frames seen: {sorted(tf_buf.frames)}")

        # Optional colouriser: load camera intrinsics + colour images up front.
        colorizer = setup_colorizer(reader, args)

        # ICP state: a running voxel map (target) + a forward-carried correction.
        icp = ICPState(args) if args.icp else None

        footprint = parse_footprint(args.footprint, args.footprint_margin) if args.footprint else None
        if footprint is not None:
            print(f"[self] footprint removal in '{args.footprint_frame}' ({len(footprint)} verts, "
                  f"margin {args.footprint_margin} m)")

        # Cloud sources to fuse: primary lidar (gets ICP) + any extra (already-coloured) clouds.
        sources = [(lidar_topic, True)] + [(t, False) for t in args.extra_cloud]

        # Per-source accumulation. Each block is (N,6) = x,y,z,r,g,b (rgb NaN if unknown).
        chunks: list[np.ndarray] = []
        stats = {"total": 0, "self_removed": 0, "missing_tf": 0, "colored": 0, "last_frame": None}

        for topic, is_primary in sources:
            n_used = accumulate_source(
                reader, topic, is_primary, tf_buf, args, icp, footprint, colorizer, chunks, stats,
            )
            tag = "lidar" if is_primary else "extra"
            print(f"[scan] {topic} ({tag}): {n_used} clouds used")

        sensor_frame = stats["last_frame"]
        if stats["missing_tf"]:
            print(f"[tf]  {stats['missing_tf']} scans had no map->sensor transform (skipped)")
        if footprint is not None:
            print(f"[self] removed {stats['self_removed']} robot self-points inside the footprint")
        if colorizer is not None:
            print(f"[color] painted {stats['colored']} lidar points from the camera image")
        if icp is not None:
            print(f"[icp] refined {icp.refined} scans (mean inlier {icp.mean_inlier():.0%}, "
                  f"mean corr {icp.mean_err():.3f} m)")
        if not chunks:
            hint = ""
            if stats["missing_tf"] and not args.urdf and sensor_frame and sensor_frame not in tf_buf.frames:
                default_urdf = Path(__file__).resolve().parent / "rby1.urdf"
                suggest = "rby1.urdf" if default_urdf.exists() else "<robot.urdf>"
                hint = (f"\nThe sensor frame '{sensor_frame}' is not in the TF tree — this bag has no "
                        f"/tf_static, so the sensor mount is missing.\nFix: add  --urdf {suggest}")
            sys.exit("No points accumulated. Check --map-frame / sensor frame / TF tree." + hint)

        cloud = np.vstack(chunks)
        print(f"[scan] total {cloud.shape[0]} points in map frame")

    if args.save_raw:
        np.save(out_dir / "raw_cloud.npy", cloud[:, :3].astype(np.float32))
        print(f"[raw]  saved raw_cloud.npy ({cloud.shape[0]} pts)")

    # ---- optional dynamic-object removal (do this BEFORE downsampling, so the
    # per-voxel hit counts still reflect how many scans actually saw each spot) --
    if args.min_hits > 0:
        before = cloud.shape[0]
        cloud = filter_min_hits(cloud, args.dyn_voxel, args.min_hits)
        print(f"[dyn]  {cloud.shape[0]} points after --min-hits {args.min_hits} "
              f"@ {args.dyn_voxel} m (removed {before - cloud.shape[0]})")

    # ---- optional voxel downsample ---------------------------------------
    if args.voxel > 0:
        cloud = voxel_downsample(cloud, args.voxel)
        print(f"[voxel] {cloud.shape[0]} points after {args.voxel} m downsample")

    # ---- save merged cloud -----------------------------------------------
    save_cloud(out_dir, cloud)
    if args.view3d_cloud > 0:
        save_cloud_view3d(out_dir / "cloud_view3d.json", cloud, args.view3d_cloud, args.map_frame)

    # ---- rasterise to elevation grid -------------------------------------
    if map_origin is None:
        # auto bounds with a small margin
        min_xy = cloud[:, :2].min(axis=0) - map_res
        max_xy = cloud[:, :2].max(axis=0) + map_res
        map_origin = [float(min_xy[0]), float(min_xy[1])]
        mw = int(np.ceil((max_xy[0] - min_xy[0]) / map_res))
        mh = int(np.ceil((max_xy[1] - min_xy[1]) / map_res))
        map_size = (mw, mh)

    # Height grid can be finer than the 2D map (denser view), same origin so still aligned.
    grid_res = args.grid_res if args.grid_res > 0 else map_res
    if grid_res != map_res:
        ext_x = map_size[0] * map_res
        ext_y = map_size[1] * map_res
        grid_size = (int(np.ceil(ext_x / grid_res)), int(np.ceil(ext_y / grid_res)))
        print(f"[grid] height grid @ {grid_res} m -> {grid_size[0]}x{grid_size[1]} cells")
    else:
        grid_size = map_size

    grid, cgrid, zmin, zmax = rasterise(cloud, grid_res, map_origin, grid_size, args.stat)
    np.save(out_dir / "elevation.npy", grid.astype(np.float32))
    save_geotiff_like(out_dir / "elevation.tif", grid)
    save_color_png(out_dir / "elevation_color.png", grid, cgrid)
    save_view3d_json(out_dir / "height_view3d.json", grid, cgrid, grid_res, map_origin, zmin, zmax, args.map_frame)

    meta = {
        "resolution": grid_res,
        "origin": map_origin,
        "width": grid_size[0],
        "height": grid_size[1],
        "z_min": float(zmin),
        "z_max": float(zmax),
        "stat": args.stat,
        "frame": args.map_frame,
        "note": "grid[row, col]; row 0 is the bottom of the map (origin), matching Nav2 image flip.",
    }
    (out_dir / "elevation_meta.json").write_text(json.dumps(meta, indent=2))
    print(f"[done] outputs in {out_dir}/  (height range {zmin:.3f}..{zmax:.3f} m)")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _voxel_keys(xyz: np.ndarray, leaf: float) -> np.ndarray:
    """Hash 3D voxel indices into one int64 key — far faster than np.unique(axis=0)."""
    g = np.floor(xyz / leaf).astype(np.int64)
    g -= g.min(axis=0)  # shift to non-negative so the hash stays well-spread
    # large primes keep collisions negligible for room-scale grids
    return g[:, 0] * np.int64(73856093) ^ g[:, 1] * np.int64(19349663) ^ g[:, 2] * np.int64(83492791)


def voxel_downsample(cloud: np.ndarray, leaf: float) -> np.ndarray:
    keys = _voxel_keys(cloud[:, :3], leaf)
    _, idx = np.unique(keys, return_index=True)
    return cloud[np.sort(idx)]


def parse_footprint(spec: str, margin: float = 0.0) -> np.ndarray:
    """Parse a footprint polygon string ("[[x,y],...]") into an (N,2) array.

    A positive `margin` grows the polygon outward from its centroid so points just
    outside the nominal outline are removed too.
    """
    poly = np.asarray(json.loads(spec), dtype=np.float64)
    if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3:
        sys.exit(f"--footprint must be a list of >=3 [x,y] pairs, got: {spec}")
    if margin:
        c = poly.mean(axis=0)
        v = poly - c
        norm = np.linalg.norm(v, axis=1, keepdims=True)
        poly = poly + (v / np.clip(norm, 1e-9, None)) * margin
    return poly


def points_in_polygon(pts_xy: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Vectorised even-odd ray-cast. pts_xy: (N,2), poly: (M,2) -> bool (N,) inside."""
    x = pts_xy[:, 0]
    y = pts_xy[:, 1]
    inside = np.zeros(len(pts_xy), dtype=bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        inside ^= cond
        j = i
    return inside


def filter_min_hits(cloud: np.ndarray, leaf: float, min_hits: int) -> np.ndarray:
    """Drop points whose 3D voxel was observed fewer than `min_hits` times.

    Static surfaces are hit by many scans -> high count. A person walking behind the
    robot occupies each voxel only briefly -> low count, so this removes them. Caveat:
    if the robot dwells while the person stands still, that spot gets many hits and
    survives.
    """
    keys = np.floor(cloud[:, :3] / leaf).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    keep = counts[inv] >= min_hits
    return cloud[keep]


class ICPState:
    """Online point-to-point ICP: refines each scan onto a running voxel map and
    carries a forward correction so SLAM drift is reduced scan-to-scan."""

    def __init__(self, args):
        from scipy.spatial import cKDTree  # local import keeps scipy optional
        from icp import icp_refine

        self._KDTree = cKDTree
        self._icp = icp_refine
        self.voxel = args.icp_voxel
        self.max_dist = args.icp_max_dist
        self.iters = args.icp_iters
        self.rebuild = max(1, args.icp_rebuild)
        self.correction = np.eye(4)
        self.vox: dict = {}          # voxel key -> representative xyz
        self.tree = None
        self.target = None
        self.since_rebuild = 0
        self.refined = 0
        self._errs: list = []
        self._ratios: list = []
        self._hist_stamps: list = []   # for correction_at(): time-ordered snapshots
        self._hist_corr: list = []

    def _downsample(self, xyz: np.ndarray) -> np.ndarray:
        keys = np.floor(xyz / self.voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return xyz[idx]

    def refine(self, xyz_lidar: np.ndarray, map_T_lidar: np.ndarray, stamp_ns: int = 0) -> np.ndarray:
        initial = self.correction @ map_T_lidar
        if self.tree is None or self.target is None or len(self.target) < 50:
            self._record(stamp_ns)
            return initial  # map not seeded yet -> trust TF
        src = self._downsample(xyz_lidar)
        src_h = np.column_stack([src, np.ones(len(src))])
        src_world = (initial @ src_h.T).T[:, :3]
        delta, err, ratio = self._icp(src_world, self.tree, self.target,
                                      self.max_dist, self.iters)
        if ratio < 0.2:
            self._record(stamp_ns)
            return initial  # poor overlap -> don't trust ICP
        self.correction = delta @ self.correction
        self.refined += 1
        if err is not None:
            self._errs.append(err)
        self._ratios.append(ratio)
        self._record(stamp_ns)
        return delta @ initial

    def _record(self, stamp_ns: int) -> None:
        if stamp_ns:
            self._hist_stamps.append(stamp_ns)
            self._hist_corr.append(self.correction.copy())

    def correction_at(self, stamp_ns: int) -> np.ndarray:
        """Nearest-in-time correction, so extra (camera) clouds get the same drift fix."""
        if not self._hist_stamps:
            return self.correction
        i = bisect.bisect_left(self._hist_stamps, stamp_ns)
        if i <= 0:
            return self._hist_corr[0]
        if i >= len(self._hist_stamps):
            return self._hist_corr[-1]
        before, after = self._hist_stamps[i - 1], self._hist_stamps[i]
        return self._hist_corr[i - 1] if (stamp_ns - before) <= (after - stamp_ns) else self._hist_corr[i]

    def add_to_map(self, xyz_world: np.ndarray) -> None:
        ds = self._downsample(xyz_world)
        keys = np.floor(ds / self.voxel).astype(np.int64)
        for k, p in zip(map(tuple, keys), ds):
            self.vox[k] = p
        self.since_rebuild += 1
        if self.since_rebuild >= self.rebuild:
            self.target = np.array(list(self.vox.values()))
            self.tree = self._KDTree(self.target)
            self.since_rebuild = 0

    def mean_err(self) -> float:
        return float(np.mean(self._errs)) if self._errs else 0.0

    def mean_inlier(self) -> float:
        return float(np.mean(self._ratios)) if self._ratios else 0.0


def effective_stamp(header_ns: int, recv_ns: int) -> int:
    """Use the message header stamp, unless it's on a different clock than the bag
    receive time (some cameras stamp with boot/sim time, not wall-clock). A gap of
    more than 1 s means the header epoch is wrong, so fall back to the receive time —
    which is consistent across topics and matches the /tf wall-clock.
    """
    if header_ns and abs(header_ns - recv_ns) < 1_000_000_000:
        return header_ns
    return recv_ns


def setup_colorizer(reader, args):
    """Build a Colorizer from the camera_info + colour image topics, or None."""
    if not (args.color_image or args.color_info):
        return None
    if not (args.color_image and args.color_info):
        print("[color] need BOTH --color-image and --color-info; colour disabled")
        return None

    conns = {c.topic: c for c in reader.connections}
    if args.color_info not in conns or args.color_image not in conns:
        print(f"[color] topic not in bag (info={args.color_info}, image={args.color_image}); colour disabled")
        return None

    cz = Colorizer()
    for _conn, _ts, raw in reader.messages(connections=[conns[args.color_info]]):
        cz.set_camera_info(reader.deserialize(raw, conns[args.color_info].msgtype))
        break
    img_conn = conns[args.color_image]
    compressed = img_conn.msgtype.endswith("CompressedImage")
    for _conn, recv, raw in reader.messages(connections=[img_conn]):
        m = reader.deserialize(raw, img_conn.msgtype)
        hs = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
        cz.add_image(m, compressed, effective_stamp(hs, recv))
    if args.color_frame:
        cz.optical_frame = args.color_frame.lstrip("/")
    if not cz.ready():
        print("[color] camera_info/image missing or Pillow absent; colour disabled")
        return None
    print(f"[color] loaded {len(cz._stamps)} images, optical frame '{cz.optical_frame}'")
    return cz


def accumulate_source(reader, topic, is_primary, tf_buf, args, icp, footprint, colorizer, chunks, stats):
    """Read one cloud topic, transform to map, (self-remove / colour), append (N,6) blocks."""
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        print(f"[scan] topic {topic} not in bag — skipped")
        return 0
    frame = None
    used = 0
    # Extra (e.g. depth) clouds are far denser per frame, so thin them harder.
    stride = args.stride if is_primary else (args.extra_stride or max(1, args.stride * 5))
    for i, (conn, ts, raw) in enumerate(reader.messages(connections=conns)):
        if stride > 1 and (i % stride) != 0:
            continue
        msg = reader.deserialize(raw, conn.msgtype)
        frame = (msg.header.frame_id or frame or "").lstrip("/")
        stats["last_frame"] = frame
        hs = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        # Cameras here stamp with a different clock than /tf; fall back to receive time.
        stamp_ns = effective_stamp(hs, ts)

        xyz, rgb = read_cloud(msg, conn.msgtype)
        if xyz.shape[0] == 0:
            continue

        if args.range_max > 0:
            keep = np.linalg.norm(xyz, axis=1) <= args.range_max
            xyz = xyz[keep]
            rgb = rgb[keep] if rgb is not None else None
            if xyz.shape[0] == 0:
                continue

        map_T_sensor = tf_buf.lookup(args.map_frame, frame, stamp_ns)
        if map_T_sensor is None:
            stats["missing_tf"] += 1
            continue

        # robot self-removal in the footprint frame
        if footprint is not None:
            fp_T_sensor = tf_buf.lookup(args.footprint_frame, frame, stamp_ns)
            if fp_T_sensor is not None:
                fp_xy = (fp_T_sensor @ np.column_stack([xyz, np.ones(len(xyz))]).T).T[:, :2]
                inside = points_in_polygon(fp_xy, footprint)
                if inside.any():
                    xyz = xyz[~inside]
                    rgb = rgb[~inside] if rgb is not None else None
                    stats["self_removed"] += int(inside.sum())
                    if xyz.shape[0] == 0:
                        continue

        # pose: primary lidar gets ICP; extra clouds reuse the nearest correction
        if is_primary and icp is not None:
            pose = icp.refine(xyz, map_T_sensor, stamp_ns)
        elif icp is not None:
            pose = icp.correction_at(stamp_ns) @ map_T_sensor
        else:
            pose = map_T_sensor

        xyz_map = (pose @ np.column_stack([xyz, np.ones(len(xyz))]).T).T[:, :3]

        # colour: paint uncoloured (lidar) points by projecting into the camera image
        if rgb is None and colorizer is not None and colorizer.optical_frame:
            cam_T_sensor = tf_buf.lookup(colorizer.optical_frame, frame, stamp_ns)
            if cam_T_sensor is not None:
                xyz_cam = (cam_T_sensor @ np.column_stack([xyz, np.ones(len(xyz))]).T).T[:, :3]
                rgb = colorizer.colorize(xyz_cam, stamp_ns)
                stats["colored"] += int(np.isfinite(rgb).all(axis=1).sum())
        if rgb is None:
            rgb = np.full((len(xyz), 3), np.nan)

        zsel = (xyz_map[:, 2] >= args.z_min) & (xyz_map[:, 2] <= args.z_max)
        if is_primary and icp is not None:
            icp.add_to_map(xyz_map[zsel])

        chunks.append(np.column_stack([xyz_map[zsel], rgb[zsel]]).astype(np.float32))
        n = int(zsel.sum())
        stats["total"] += n
        stats["pending"] = stats.get("pending", 0) + n
        used += 1
        if used % 100 == 0:                                  # live progress (streamed to the UI)
            print(f"[progress] {topic}: {used} clouds, {stats['total'] // 1000}k pts", flush=True)
        # Memory cap: when --voxel is set, periodically collapse what we've
        # accumulated so dense streams (e.g. depth/color/points) don't OOM.
        # Skipped with --min-hits, which needs the raw per-voxel hit counts.
        if args.voxel > 0 and not args.min_hits and stats["pending"] > _COMPACT_EVERY:
            compacted = voxel_downsample(np.vstack(chunks), args.voxel)
            chunks.clear()
            chunks.append(compacted)
            stats["pending"] = 0
        if args.max_points and stats["total"] >= args.max_points:
            print(f"[scan] reached --max-points ({args.max_points}); stopping")
            break
    return used


_COMPACT_EVERY = 8_000_000  # points held before an incremental voxel collapse


def rasterise(cloud, res, origin, size, stat):
    """Project points to a (H, W) height grid + (H, W, 3) colour grid.

    Row 0 = map origin (bottom), like Nav2. The colour grid takes the RGB of each
    cell's tallest point (NaN where no point or that point had no camera colour).
    Returns (height_grid, colour_grid, zmin, zmax).
    """
    w, h = size
    col = np.floor((cloud[:, 0] - origin[0]) / res).astype(np.int64)
    row = np.floor((cloud[:, 1] - origin[1]) / res).astype(np.int64)
    inb = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    col, row, z = col[inb], row[inb], cloud[inb, 2]
    has_color = cloud.shape[1] >= 6
    rgb = cloud[inb, 3:6] if has_color else None

    grid = np.full((h, w), np.nan, dtype=np.float64)
    cgrid = np.full((h, w, 3), np.nan, dtype=np.float64)
    if z.size == 0:
        return grid, cgrid, 0.0, 0.0

    flat = row * w + col
    if stat in ("max", "min"):
        out = grid.reshape(-1)
        (np.fmax if stat == "max" else np.fmin).at(out, flat, z)
        grid = out.reshape(h, w)
    else:  # mean / median
        from collections import defaultdict
        buckets: dict[int, list] = defaultdict(list)
        for fidx, zz in zip(flat, z):
            buckets[fidx].append(zz)
        out = grid.reshape(-1)
        agg = np.mean if stat == "mean" else np.median
        for fidx, vals in buckets.items():
            out[fidx] = agg(vals)
        grid = out.reshape(h, w)

    if has_color:
        # colour each cell from its tallest point: sort by (cell, z), keep last per cell.
        order = np.lexsort((z, flat))
        flat_s = flat[order]
        last = np.ones(len(flat_s), dtype=bool)
        last[:-1] = flat_s[1:] != flat_s[:-1]
        sel = order[last]
        cgrid.reshape(-1, 3)[flat[sel]] = rgb[sel]

    finite = grid[np.isfinite(grid)]
    return grid, cgrid, float(finite.min()), float(finite.max())


def _cloud_colors(cloud: np.ndarray) -> np.ndarray:
    """Per-point RGB (0..1): camera colour where present, else height colormap."""
    z = cloud[:, 2]
    zn = (z - z.min()) / (np.ptp(z) + 1e-9)
    colors = _turbo(zn)[:, :3].copy()
    if cloud.shape[1] >= 6:
        rgb = cloud[:, 3:6]
        have = np.isfinite(rgb).all(axis=1)
        colors[have] = rgb[have]
    return colors


def save_cloud(out_dir: Path, cloud: np.ndarray) -> None:
    xyz = cloud[:, :3]
    colors = _cloud_colors(cloud)
    has_rgb = cloud.shape[1] >= 6 and np.isfinite(cloud[:, 3:6]).all(axis=1).any()
    name = "cloud_map_rgb.pcd" if has_rgb else "cloud_map.pcd"
    try:
        import open3d as o3d

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        pc.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(str(out_dir / name), pc)
        print(f"[cloud] wrote {out_dir/name}")
    except ImportError:
        # plain XYZRGB fallback
        np.savetxt(out_dir / "cloud_map.xyz", np.column_stack([xyz, colors]), fmt="%.4f")
        print(f"[cloud] open3d missing; wrote {out_dir/'cloud_map.xyz'}")


def save_view3d_json(path: Path, grid, cgrid, res, origin, zmin, zmax, frame) -> None:
    """Export filled cells as coloured points (map-frame world coords) for the
    map_ui 3D view. One point per cell center: x,y in metres, z = height.

    Colour = camera colour where available, else height colormap (turbo).
    Format: {type, frame, resolution, origin, z_min, z_max, count, xyz[], rgb[]}
    """
    rows, cols = np.where(np.isfinite(grid))
    if rows.size == 0:
        path.write_text(json.dumps({"type": "height_map_view3d", "count": 0, "xyz": [], "rgb": []}))
        return
    z = grid[rows, cols]
    x = origin[0] + (cols + 0.5) * res
    y = origin[1] + (rows + 0.5) * res

    zn = (z - zmin) / (zmax - zmin + 1e-9)
    rgb = _turbo(np.clip(zn, 0, 1))[:, :3].copy()
    if cgrid is not None:
        cam = cgrid[rows, cols]
        have = np.isfinite(cam).all(axis=1)
        rgb[have] = cam[have]

    xyz = np.column_stack([x, y, z]).astype(np.float32).reshape(-1)
    rgb_flat = rgb.astype(np.float32).reshape(-1)
    payload = {
        "type": "height_map_view3d",
        "frame": frame,
        "resolution": res,
        "origin": [float(origin[0]), float(origin[1])],
        "z_min": float(zmin),
        "z_max": float(zmax),
        "count": int(rows.size),
        "xyz": [round(float(v), 4) for v in xyz],
        "rgb": [round(float(v), 3) for v in rgb_flat],
    }
    path.write_text(json.dumps(payload))
    print(f"[view3d] wrote {path}  ({rows.size} points)")


def save_cloud_view3d(path: Path, cloud: np.ndarray, max_pts: int, frame: str) -> None:
    """Export the full 3D coloured cloud (down to max_pts) for the map_ui 3D view.
    Unlike the 2.5D grid, this keeps every height — the real volumetric cloud."""
    xyz = cloud[:, :3].astype(np.float64)
    colors = _cloud_colors(cloud)
    n = len(xyz)
    if n > max_pts:
        sel = np.random.default_rng(0).choice(n, max_pts, replace=False)
        xyz, colors = xyz[sel], colors[sel]
    z = xyz[:, 2]
    payload = {
        "type": "height_map_view3d",
        "frame": frame,
        "count": int(len(xyz)),
        "z_min": float(z.min()) if len(z) else 0.0,
        "z_max": float(z.max()) if len(z) else 0.0,
        "xyz": [round(float(v), 4) for v in xyz.reshape(-1)],
        "rgb": [round(float(v), 3) for v in colors.reshape(-1)],
    }
    path.write_text(json.dumps(payload))
    print(f"[view3d] wrote {path}  ({len(xyz)} points, full 3D)")


def save_geotiff_like(path: Path, grid: np.ndarray) -> None:
    try:
        from PIL import Image

        Image.fromarray(grid.astype(np.float32), mode="F").save(path)
    except Exception as e:  # noqa: BLE001
        print(f"[tif]  skipped ({e})")


def save_color_png(path: Path, grid: np.ndarray, cgrid=None) -> None:
    from PIL import Image

    finite = np.isfinite(grid)
    img = np.zeros((*grid.shape, 4), dtype=np.uint8)  # RGBA, transparent where no data
    if finite.any():
        z = grid[finite]
        zn = (grid - z.min()) / (np.ptp(z) + 1e-9)
        rgb = _turbo(np.clip(zn, 0, 1))[..., :3].copy()
        if cgrid is not None:
            have = np.isfinite(cgrid).all(axis=2)
            rgb[have] = cgrid[have]
        img[..., :3] = (rgb * 255).astype(np.uint8)
        img[..., 3] = np.where(finite, 255, 0)
    # flip vertically so the PNG matches how Nav2 displays the map (origin bottom-left)
    Image.fromarray(img[::-1], mode="RGBA").save(path)
    print(f"[png]  wrote {path}")


def _turbo(x: np.ndarray) -> np.ndarray:
    """Approximate Google's Turbo colormap; x in [0,1] -> (..,3) floats in [0,1]."""
    try:
        import matplotlib.cm as cm

        return cm.get_cmap("turbo")(x)[..., :3]
    except Exception:  # noqa: BLE001
        # fallback: simple blue->red ramp
        x = np.clip(x, 0, 1)
        return np.stack([x, np.zeros_like(x), 1 - x], axis=-1)


if __name__ == "__main__":
    main()
