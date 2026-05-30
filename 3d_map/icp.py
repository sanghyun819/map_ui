"""Lightweight point-to-point ICP for refining each scan against the running map.

This is a *registration* refinement only: it nudges each scan's pose so it lines
up better with everything accumulated so far, correcting slow SLAM drift. It does
NOT remove moving objects (a person walking behind the robot) — for that see the
voxel hit-count filter in build_height_map.py (`--min-hits`). A moving cluster can
actually bias ICP, so points are matched with a max correspondence distance and
outliers are rejected each iteration.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def best_fit_transform(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Kabsch: 4x4 rigid T mapping corresponding points a -> b (both Nx3)."""
    ca = a.mean(axis=0)
    cb = b.mean(axis=0)
    aa = a - ca
    bb = b - cb
    h = aa.T @ bb
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:        # reflection fix
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = cb - r @ ca
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = t
    return out


def icp_refine(src: np.ndarray, tree: cKDTree, target: np.ndarray,
               max_dist: float = 0.3, max_iter: int = 20, tol: float = 1e-4):
    """Refine src (Nx3, already at initial guess) onto target via point-to-point ICP.

    Returns (total_T, mean_inlier_err, inlier_ratio) where total_T is the 4x4 that
    should be applied to the initial guess: refined = total_T @ initial.
    """
    total = np.eye(4)
    cur = src.copy()
    prev_err = None
    inlier_ratio = 0.0
    for _ in range(max_iter):
        dist, idx = tree.query(cur, k=1)
        mask = dist < max_dist
        inlier_ratio = float(mask.mean())
        if int(mask.sum()) < 10:
            break
        t = best_fit_transform(cur[mask], target[idx[mask]])
        cur = (t[:3, :3] @ cur.T).T + t[:3, 3]
        total = t @ total
        err = float(dist[mask].mean())
        if prev_err is not None and abs(prev_err - err) < tol:
            prev_err = err
            break
        prev_err = err
    return total, prev_err, inlier_ratio
