#!/usr/bin/env python3
"""Quick viewers for the build_height_map.py outputs.

Usage:
  python visualize.py out/cloud_map.pcd          # 3D point cloud (open3d)
  python visualize.py out/elevation.npy          # 2.5D surface (matplotlib)
  python visualize.py out/elevation.npy --map ../map_0430.pgm  # surface over the occupancy image
  python visualize.py out/height_view3d.json     # the 3D-view JSON, as a coloured scatter
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def view_height_json(path: Path) -> None:
    """Render the height_view3d.json (map-frame coloured points) as a 3D scatter."""
    import matplotlib.pyplot as plt

    d = json.loads(path.read_text())
    xyz = np.asarray(d.get("xyz", []), dtype=float).reshape(-1, 3)
    rgb = np.asarray(d.get("rgb", []), dtype=float).reshape(-1, 3)
    if xyz.size == 0:
        print("empty height map"); return
    if rgb.shape[0] != xyz.shape[0]:
        rgb = None
    print(f"{xyz.shape[0]} points  z {xyz[:,2].min():.2f}..{xyz[:,2].max():.2f} m")

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=rgb, s=2, depthshade=False)
    ax.set_title(f"Height map: {path.name}")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("height [m]")
    try:
        ax.set_box_aspect((np.ptp(xyz[:, 0]), np.ptp(xyz[:, 1]), max(np.ptp(xyz[:, 2]), 0.1)))
    except Exception:
        pass
    plt.tight_layout()
    plt.show()


def view_cloud(path: Path) -> None:
    import open3d as o3d

    pc = o3d.io.read_point_cloud(str(path))
    print(f"{len(pc.points)} points")
    o3d.visualization.draw_geometries([pc], window_name=str(path))


def view_elevation(npy: Path, map_img: Path | None) -> None:
    import matplotlib.pyplot as plt

    grid = np.load(npy)
    h, w = grid.shape
    ys, xs = np.mgrid[0:h, 0:w]

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    z = np.where(np.isfinite(grid), grid, np.nan)
    ax.plot_surface(xs, ys, z, cmap="turbo", linewidth=0, antialiased=False)
    ax.set_title(f"Elevation: {npy.name}")
    ax.set_xlabel("col"); ax.set_ylabel("row"); ax.set_zlabel("height [m]")
    plt.tight_layout()
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="cloud_map.pcd / .xyz  OR  elevation.npy  OR  height_view3d.json")
    ap.add_argument("--map", help="occupancy image to underlay (elevation mode)")
    args = ap.parse_args()
    p = Path(args.path)
    if p.suffix in (".pcd", ".ply", ".xyz"):
        view_cloud(p)
    elif p.suffix == ".json":
        view_height_json(p)
    else:
        view_elevation(p, Path(args.map) if args.map else None)


if __name__ == "__main__":
    main()
