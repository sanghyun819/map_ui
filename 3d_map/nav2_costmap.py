#!/usr/bin/env python3
"""Build & visualise the Nav2 GLOBAL costmap offline — no running Nav2 node.

Parses nav2_params.yaml (footprint / robot_radius + inflation_layer) and applies
the Nav2 static + inflation cost model to a Nav2 map (yaml + pgm). Replaces having
to launch costmap_2d just to inspect the global cost / footprint.

Standalone:
  python nav2_costmap.py --map map.yaml --nav2-params nav2_params.yaml --out costmap.png

Reusable: parse_costmap_params(), build_costmap() → uint8 cost grid (Nav2 convention:
0 free … 252 inflated, 253 inscribed, 254 lethal).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

LETHAL = 254
INSCRIBED = 253
MAX_NON_OBSTACLE = 252


# ---------------------------------------------------------------------------
# nav2_params parsing
# ---------------------------------------------------------------------------

def _costmap_node(data, ns):
    """Return the ros__parameters dict for global_costmap/local_costmap, tolerant of nesting."""
    node = data.get(ns, data)
    if isinstance(node, dict) and ns in node:      # global_costmap: { global_costmap: {...} }
        node = node[ns]
    if isinstance(node, dict) and "ros__parameters" in node:
        node = node["ros__parameters"]
    return node or {}


def parse_costmap_params(path, ns="global_costmap"):
    import yaml as _yaml
    data = _yaml.safe_load(open(path))
    p = _costmap_node(data, ns)
    infl = p.get("inflation_layer", {}) or {}
    fp = p.get("footprint")
    if isinstance(fp, str):
        fp = fp.strip()
        try:
            fp = json.loads(fp) if fp and fp[0] == "[" else None
        except json.JSONDecodeError:
            fp = None
    return {
        "robot_radius": p.get("robot_radius"),
        "footprint": fp,
        "resolution": p.get("resolution"),
        "inflation_radius": float(infl.get("inflation_radius", 0.55)),
        "cost_scaling_factor": float(infl.get("cost_scaling_factor", 3.0)),
    }


def _pt_seg_dist(p, a, b):
    p, a, b = np.asarray(p, float), np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = 0.0 if np.dot(ab, ab) < 1e-12 else np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


def inscribed_radius(params):
    """Largest circle (at robot centre) fitting inside the footprint, else robot_radius."""
    fp = params.get("footprint")
    if fp and len(fp) >= 3:
        pts = np.array(fp, float)
        return min(_pt_seg_dist((0, 0), pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts)))
    return float(params.get("robot_radius") or 0.22)


# ---------------------------------------------------------------------------
# Costmap build
# ---------------------------------------------------------------------------

def load_occupancy(yaml_path):
    import yaml as _yaml
    from PIL import Image
    meta = _yaml.safe_load(open(yaml_path))
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"]]
    occ_th = float(meta.get("occupied_thresh", 0.65))
    negate = int(meta.get("negate", 0))
    img = np.asarray(Image.open((Path(yaml_path).parent / meta["image"]).resolve()).convert("L"))
    p = img.astype(np.float64) / 255.0
    occ_prob = p if negate else 1.0 - p
    occupied = occ_prob > occ_th
    return occupied, res, origin


def build_costmap(occupied, res, params):
    """Return (cost uint8 HxW, inscribed_radius_m) using Nav2's inflation model."""
    from scipy.ndimage import distance_transform_edt
    insc = inscribed_radius(params)
    inf_r = params["inflation_radius"]
    scale = params["cost_scaling_factor"]
    d = distance_transform_edt(~occupied) * res        # metres to nearest lethal cell

    cost = np.zeros(occupied.shape, np.uint8)
    cost[occupied] = LETHAL
    insc_mask = (~occupied) & (d <= insc)
    cost[insc_mask] = INSCRIBED
    band = (~occupied) & (d > insc) & (d <= inf_r)
    # Nav2: cost = (INSCRIBED-1) * exp(-scaling * (d - inscribed))
    factor = np.exp(-scale * (d[band] - insc))
    cost[band] = np.clip(np.round((INSCRIBED - 1) * factor), 0, MAX_NON_OBSTACLE).astype(np.uint8)
    return cost, insc


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def colorize(cost):
    """Nav2-style colours: free=dark, inflated=blue→cyan→yellow→red, inscribed=magenta, lethal=red."""
    import matplotlib.cm as cm
    h, w = cost.shape
    rgb = np.zeros((h, w, 3), np.uint8)
    band = (cost > 0) & (cost < INSCRIBED)
    if band.any():
        rgb[band] = (cm.get_cmap("turbo")(cost[band] / float(MAX_NON_OBSTACLE))[:, :3] * 255).astype(np.uint8)
    rgb[cost == INSCRIBED] = (255, 0, 255)
    rgb[cost == LETHAL] = (255, 40, 40)
    rgb[cost == 0] = (25, 30, 40)
    return rgb


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, help="Nav2 map .yaml")
    ap.add_argument("--nav2-params", required=True, help="nav2_params.yaml")
    ap.add_argument("--ns", default="global_costmap", help="costmap namespace (default global_costmap)")
    ap.add_argument("--out", help="annotated preview PNG (matplotlib, with footprint/inscribed)")
    ap.add_argument("--overlay", help="pixel-exact RGBA overlay PNG (same size as map, transparent where free)")
    ap.add_argument("--npy", help="also save the raw cost grid (uint8) here")
    args = ap.parse_args()

    params = parse_costmap_params(args.nav2_params, args.ns)
    occupied, res, origin = load_occupancy(args.map)
    cost, insc = build_costmap(occupied, res, params)
    print(f"[costmap] {cost.shape[1]}x{cost.shape[0]} @ {res} m  inscribed={insc:.3f} m  "
          f"inflation_radius={params['inflation_radius']} scaling={params['cost_scaling_factor']}")
    print(f"[footprint] {'polygon('+str(len(params['footprint']))+' pts)' if params.get('footprint') else 'robot_radius '+str(params.get('robot_radius'))}")

    rgb = colorize(cost)

    if args.overlay:                       # pixel-exact RGBA overlay for map_ui (transparent where free)
        from PIL import Image
        rgba = np.zeros((*cost.shape, 4), np.uint8)
        rgba[..., :3] = rgb
        rgba[..., 3] = np.where(cost > 0, 200, 0).astype(np.uint8)
        Image.fromarray(rgba, "RGBA").save(args.overlay)
        print(f"[overlay] wrote {args.overlay}")
    if args.npy:
        np.save(args.npy, cost)
        print(f"[npy] wrote {args.npy}")
    if not args.out:
        return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(rgb, origin="upper")
    # draw footprint at the map centre for scale reference
    cx, cy = cost.shape[1] / 2, cost.shape[0] / 2
    fp = params.get("footprint")
    if fp:
        pp = np.array(fp, float) / res
        pp = np.column_stack([cx + pp[:, 0], cy - pp[:, 1]])
        ax.plot(np.append(pp[:, 0], pp[0, 0]), np.append(pp[:, 1], pp[0, 1]), "w-", lw=2)
    else:
        th = np.linspace(0, 2 * math.pi, 40)
        r = float(params.get("robot_radius") or 0.22) / res
        ax.plot(cx + r * np.cos(th), cy + r * np.sin(th), "w-", lw=2)
    ci = plt.Circle((cx, cy), insc / res, fill=False, color="magenta", ls="--", lw=1)
    ax.add_patch(ci)
    ax.set_title(f"Nav2 {args.ns}  inscribed={insc:.2f}m  inflation={params['inflation_radius']}m")
    ax.axis("off")
    plt.tight_layout(); plt.savefig(args.out, dpi=90)
    print(f"[viz] wrote {args.out}")


if __name__ == "__main__":
    main()
