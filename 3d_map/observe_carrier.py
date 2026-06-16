#!/usr/bin/env python3
"""Compute a Nav2 goal pose from which the robot camera can SEE a carrier.

Reads map_ui's semantic JSON (carriers as world polygons) + the Nav2 map, derives
a cost field from the occupancy grid (clearance via distance transform), and picks
the lowest-cost, reachable, line-of-sight goal whose camera view frames the chosen
carrier face within the camera FOV.

Per-carrier annotations (read from the semantic JSON if present, else CLI/defaults):
  view_face : "front" | "top"            which face must be visible
  front_edge: <int>                      index of the polygon edge that is the front
  front_yaw : <rad>                      outward normal of the front face (alt. to front_edge)
  z_start,z_end / height : <m>           vertical band of the front face to frame

Camera frame = the colour optical frame (z forward). FOV from --camera-info (K+size)
or --hfov/--vfov. Goal is the robot BASE pose (x, y, theta) facing the carrier.

Outputs the goal (x,y,theta + quaternion) as JSON and, with --viz, a preview PNG.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from nav2_costmap import parse_costmap_params, build_costmap, INSCRIBED


# ---------------------------------------------------------------------------
# Map + geometry helpers
# ---------------------------------------------------------------------------

def load_map(yaml_path: Path):
    import yaml as _yaml
    from PIL import Image

    meta = _yaml.safe_load(open(yaml_path))
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"]]
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.25))
    negate = int(meta.get("negate", 0))
    img = np.asarray(Image.open((yaml_path.parent / meta["image"]).resolve()).convert("L"))
    p = img.astype(np.float64) / 255.0
    if negate == 0:
        occ_prob = 1.0 - p          # dark = occupied
    else:
        occ_prob = p
    occupied = occ_prob > occ_th    # lethal cells
    free = occ_prob < free_th       # known-free cells
    return {"res": res, "origin": origin, "h": img.shape[0], "w": img.shape[1],
            "occupied": occupied, "free": free}


def world_to_px(wx, wy, m):
    ox, oy = m["origin"][0], m["origin"][1]
    yaw = m["origin"][2] if len(m["origin"]) > 2 else 0.0
    dx, dy = wx - ox, wy - oy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    return lx / m["res"], m["h"] - ly / m["res"]


def px_to_world(px, py, m):
    ox, oy = m["origin"][0], m["origin"][1]
    yaw = m["origin"][2] if len(m["origin"]) > 2 else 0.0
    lx = px * m["res"]
    ly = (m["h"] - py) * m["res"]
    c, s = math.cos(yaw), math.sin(yaw)
    return ox + lx * c - ly * s, oy + lx * s + ly * c


def line_clear(p0, p1, occupied):
    """Bresenham line-of-sight: True if no occupied cell between two pixel points."""
    x0, y0 = int(round(p0[0])), int(round(p0[1]))
    x1, y1 = int(round(p1[0])), int(round(p1[1]))
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    H, W = occupied.shape
    while True:
        if 0 <= y0 < H and 0 <= x0 < W and occupied[y0, x0]:
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 < dx:
            err += dx; y0 += sy


# ---------------------------------------------------------------------------
# Carrier + camera
# ---------------------------------------------------------------------------

def find_carrier(sem, key):
    for c in sem.get("carriers", []):
        if str(c.get("id")) == str(key) or str(c.get("label")) == str(key):
            return c
    sys.exit(f"carrier {key!r} not found. Have: " +
             ", ".join(f"{c.get('id')}({c.get('label')})" for c in sem.get("carriers", [])))


def poly_world(c):
    return np.array([[p["x"], p["y"]] if isinstance(p, dict) else p for p in c["polygon"]], float)


def front_normal(poly, c, args):
    """Return (front_yaw, front_midpoint world). Edge index or yaw or auto (most-free)."""
    centroid = poly.mean(axis=0)
    edge = args.front_edge if args.front_edge is not None else c.get("front_edge")
    yaw = args.front_yaw if args.front_yaw is not None else c.get("front_yaw")
    if yaw is None and isinstance(c.get("front_face"), dict):
        yaw = c["front_face"].get("normal_yaw")
    if edge is not None:
        a = poly[int(edge) % len(poly)]
        b = poly[(int(edge) + 1) % len(poly)]
        mid = (a + b) / 2
        d = b - a
        n = np.array([d[1], -d[0]], float)            # edge normal
        if np.dot(n, mid - centroid) < 0:
            n = -n                                    # point outward
        return math.atan2(n[1], n[0]), mid
    if yaw is not None:
        yaw = float(yaw)
        return yaw, centroid + np.array([math.cos(yaw), math.sin(yaw)]) * _radius(poly)
    return None, centroid                              # auto handled by caller


def _radius(poly):
    c = poly.mean(axis=0)
    return float(np.max(np.linalg.norm(poly - c, axis=1)))


def _fov_from_k(k, w, h):
    fx, fy = k[0], k[4]
    return (2 * math.atan(w / (2 * fx)), 2 * math.atan(h / (2 * fy)))


def fov_from_bag(bag, topic):
    """Read the first CameraInfo from a rosbag → (hfov, vfov) radians, or None."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        return None
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag)], default_typestore=ts) as r:
        conns = [c for c in r.connections if c.topic == topic] if topic else []
        if not conns:
            conns = [c for c in r.connections if c.msgtype.endswith("CameraInfo") and "color" in c.topic]
        if not conns:
            conns = [c for c in r.connections if c.msgtype.endswith("CameraInfo")]
        if not conns:
            return None
        for conn, _ts, raw in r.messages(connections=[conns[0]]):
            msg = r.deserialize(raw, conn.msgtype)
            return _fov_from_k(list(msg.k), int(msg.width), int(msg.height))
    return None


def cam_pose_from_bag(bag, base, optical):
    """Read (camera_height_m, tilt_rad) from the bag TF: base → camera optical frame.
    tilt = pitch of the optical z-axis (− = looking down)."""
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
        from tf_buffer import TfBuffer
    except ImportError:
        return None
    ts = get_typestore(Stores.ROS2_HUMBLE)
    tf = TfBuffer()
    with AnyReader([Path(bag)], default_typestore=ts) as r:
        for c in r.connections:
            if c.msgtype.endswith("TFMessage"):
                for conn, _ts, raw in r.messages(connections=[c]):
                    tf.add_tf_message(r.deserialize(raw, conn.msgtype),
                                      static=conn.topic.rstrip("/").endswith("tf_static"))
    T = tf.lookup(base.lstrip("/"), optical.lstrip("/"), 10 ** 18)   # nearest (last) sample
    if T is None:
        return None
    z = T[:3, 2]
    n = float(np.linalg.norm(z))
    if n < 1e-9:
        return None
    z = z / n
    tilt = math.asin(max(-1.0, min(1.0, z[2])))     # + up, − down
    return float(T[2, 3]), tilt


def fov_from_args(args):
    if args.camera_info_bag:                       # usually the bag carries camera_info
        fov = fov_from_bag(args.camera_info_bag, args.camera_info_topic)
        if fov:
            print(f"[fov] from bag camera_info: {math.degrees(fov[0]):.1f}° x {math.degrees(fov[1]):.1f}°")
            return fov
        print("[fov] no camera_info in bag; using --hfov/--vfov defaults")
    if args.camera_info:
        ci = json.load(open(args.camera_info))
        k = ci.get("k") or ci.get("K")
        return _fov_from_k(k, ci["width"], ci["height"])
    return (math.radians(args.hfov), math.radians(args.vfov))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--semantic", required=True, help="map_ui *_semantic.json")
    ap.add_argument("--map", required=True, help="Nav2 map .yaml")
    ap.add_argument("--carrier", required=True, help="carrier id or label to observe")
    ap.add_argument("--view-face", choices=["front", "top"], help="override carrier.view_face")
    ap.add_argument("--front-edge", type=int, help="polygon edge index that is the front")
    ap.add_argument("--front-yaw", type=float, help="front-face outward normal [rad]")
    ap.add_argument("--z-start", type=float, help="front-face band bottom [m]")
    ap.add_argument("--z-end", type=float, help="front-face band top [m]")
    ap.add_argument("--nav2-params", help="nav2_params.yaml → use the real Nav2 global costmap for cost/reachability")
    ap.add_argument("--ns", default="global_costmap", help="costmap namespace in nav2_params (default global_costmap)")
    ap.add_argument("--camera-info", help="CameraInfo JSON ({k,width,height}) for FOV")
    ap.add_argument("--camera-info-bag", help="rosbag with a CameraInfo topic → FOV (usually the case)")
    ap.add_argument("--camera-info-topic", default="/camera/camera_head/color/camera_info",
                    help="CameraInfo topic in --camera-info-bag")
    ap.add_argument("--hfov", type=float, default=55.0, help="horizontal FOV [deg] if no camera-info")
    ap.add_argument("--vfov", type=float, default=42.0, help="vertical FOV [deg] if no camera-info")
    ap.add_argument("--cam-height", type=float, default=1.3, help="camera height above floor [m]")
    ap.add_argument("--cam-tilt", type=float, default=-20.0, help="camera downward tilt [deg] (operating pitch, default -20)")
    ap.add_argument("--cam-pose-bag", help="rosbag → read camera height + tilt from TF (base→optical frame)")
    ap.add_argument("--base-frame", default="base_nav", help="base frame for --cam-pose-bag")
    ap.add_argument("--optical-frame", default="camera_head_color_optical_frame", help="camera optical frame for --cam-pose-bag")
    ap.add_argument("--robot-radius", type=float, default=0.3, help="robot radius for clearance [m]")
    ap.add_argument("--dist-min", type=float, default=0.4, help="min viewing distance [m]")
    ap.add_argument("--dist-max", type=float, default=4.0, help="max viewing distance [m]")
    ap.add_argument("--margin", type=float, default=1.15, help="FOV-fit distance margin factor")
    ap.add_argument("--restrict-room", action="store_true", help="keep goals inside the carrier's room polygon")
    # goal preference: open space + near the robot's current pose
    ap.add_argument("--robot-x", type=float, help="current robot x [m] (prefer goals nearby)")
    ap.add_argument("--robot-y", type=float, help="current robot y [m]")
    ap.add_argument("--w-cost", type=float, default=1.0, help="weight on Nav2 cost (avoid inflation)")
    ap.add_argument("--w-open", type=float, default=12.0, help="weight on openness (clearance) — prefer wide areas")
    ap.add_argument("--w-robot", type=float, default=3.0, help="weight on distance to robot [per m] — prefer nearby")
    ap.add_argument("--w-dist", type=float, default=20.0, help="weight on |viewing dist − ideal| [per m]")
    ap.add_argument("--top", type=int, default=1, help="return the top-N recommended goals (spatially spread)")
    ap.add_argument("--sep", type=float, default=0.6, help="min separation [m] between recommended goals")
    ap.add_argument("--out", help="write goal JSON here")
    ap.add_argument("--viz", help="save a preview PNG here")
    args = ap.parse_args()

    sem = json.load(open(args.semantic))
    m = load_map(Path(args.map))
    c = find_carrier(sem, args.carrier)
    poly = poly_world(c)
    centroid = poly.mean(axis=0)
    view_face = args.view_face or c.get("view_face", "front")
    hfov, vfov = fov_from_args(args)

    cam_h, cam_tilt = args.cam_height, math.radians(args.cam_tilt)
    if args.cam_pose_bag:
        cp = cam_pose_from_bag(args.cam_pose_bag, args.base_frame, args.optical_frame)
        if cp:
            cam_h, cam_tilt = cp
            print(f"[cam] from bag TF: height={cam_h:.2f} m, tilt={math.degrees(cam_tilt):.1f}°")
        else:
            print("[cam] could not read camera pose from bag TF; using --cam-height/--cam-tilt")

    # Openness (clearance to nearest obstacle) — always used to prefer WIDE areas.
    from scipy.ndimage import distance_transform_edt
    clearance = distance_transform_edt(~m["occupied"]) * m["res"]
    # Reachability + cost: prefer the real Nav2 global costmap; else a clearance proxy.
    if args.nav2_params:
        params = parse_costmap_params(args.nav2_params, args.ns)
        costmap, insc = build_costmap(m["occupied"], m["res"], params)
        traversable = costmap < INSCRIBED          # robot centre must not be inscribed/lethal
        print(f"[cost] Nav2 global costmap (inscribed={insc:.2f} m, inflation={params['inflation_radius']} m)")
    else:
        traversable = (clearance >= args.robot_radius) & m["free"]
        costmap = None
        print("[cost] clearance proxy (no --nav2-params)")
    robot = (args.robot_x is not None and args.robot_y is not None)
    if robot:
        print(f"[goal] preferring goals near robot ({args.robot_x:.2f}, {args.robot_y:.2f})")

    room_poly = None
    if args.restrict_room and c.get("room_id"):
        for r in sem.get("rooms", []):
            if str(r.get("id")) == str(c.get("room_id")):
                room_poly = poly_world(r)

    # face geometry
    fyaw, fmid = front_normal(poly, c, args)
    z_start = args.z_start if args.z_start is not None else float(c.get("z_start", 0.0))
    z_end = args.z_end if args.z_end is not None else float(c.get("z_end", c.get("height", 1.0)))
    face_h = max(0.05, z_end - z_start)

    def apparent_width(view_dir):
        """Carrier width perpendicular to the view direction = what the camera must frame."""
        perp = np.array([-view_dir[1], view_dir[0]])
        proj = poly @ perp
        return float(proj.max() - proj.min())

    half_v = vfov / 2

    def band_fits(d):
        """Does the z band fall inside the (fixed-tilt) vertical FOV window from distance d?"""
        a_bot = math.atan2(z_start - cam_h, d)
        a_top = math.atan2(z_end - cam_h, d)
        return (a_bot >= cam_tilt - half_v) and (a_top <= cam_tilt + half_v)

    # nominal apparent width along the designated front normal (camera looks opposite to it)
    if fyaw is not None:
        nom_view = np.array([math.cos(fyaw + math.pi), math.sin(fyaw + math.pi)])
        w_nom = apparent_width(nom_view)
    else:
        w_nom = float(np.ptp(poly, axis=0).max())   # auto: loose upper bound

    # FOV-fit lower bound from horizontal width; vertical handled per-distance by band_fits().
    d_fit = w_nom / (2 * math.tan(hfov / 2)) * args.margin
    d_fit = min(max(d_fit, args.dist_min), args.dist_max)

    if view_face == "top":
        # Top face: look down from directly beside; approximate with a ring at d_fit.
        base_dirs = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        dirs = [(centroid, a) for a in base_dirs]
        anchor = centroid
    else:
        anchor = fmid
        if fyaw is None:
            base_dirs = np.linspace(0, 2 * math.pi, 48, endpoint=False)   # auto: try all sides
        else:
            base_dirs = fyaw + np.radians(np.arange(-40, 41, 5))           # fan around front normal
        dirs = [(anchor, a) for a in base_dirs]

    cands = []
    a_px = world_to_px(anchor[0], anchor[1], m)
    for _, a in dirs:
        ux, uy = math.cos(a), math.sin(a)
        for d in np.linspace(d_fit, args.dist_max, 16):
            gx, gy = anchor[0] + ux * d, anchor[1] + uy * d
            gpx = world_to_px(gx, gy, m)
            ix, iy = int(round(gpx[0])), int(round(gpx[1]))
            if not (0 <= iy < m["h"] and 0 <= ix < m["w"]):
                continue
            if not traversable[iy, ix]:
                continue
            if room_poly is not None and not _point_in_poly(gx, gy, room_poly):
                continue
            if not line_clear(gpx, a_px, m["occupied"]):     # camera must see the face
                continue
            # the SPECIFIED face must fully fit the FOV from here
            vdir = np.array([gx - centroid[0], gy - centroid[1]])
            nv = np.linalg.norm(vdir)
            vdir = vdir / nv if nv > 1e-9 else np.array([1.0, 0.0])
            w_app = apparent_width(vdir)                       # lateral width seen from this pose
            if 2 * math.atan2(w_app / 2, d) > hfov:           # horizontal fit
                continue
            if view_face == "front" and not band_fits(d):     # band inside the fixed-tilt FOV window
                continue
            clr = float(clearance[iy, ix])
            cval = int(costmap[iy, ix]) if costmap is not None else 0
            r_dist = math.hypot(gx - args.robot_x, gy - args.robot_y) if robot else 0.0
            # low cost = open (high clearance) + near robot + good framing + off inflation
            cost = (args.w_cost * cval
                    - args.w_open * clr
                    + args.w_robot * r_dist
                    + args.w_dist * abs(d - d_fit))
            theta = math.atan2(centroid[1] - gy, centroid[0] - gx)        # face the carrier
            cands.append({"cost": cost, "x": gx, "y": gy, "theta": theta, "dist": d,
                          "clearance": clr, "nav2_cost": cval, "robot_dist": r_dist,
                          "face_w": round(w_app, 3), "cam_tilt": round(cam_tilt, 4)})

    if not cands:
        sys.exit("No valid observation goal found (try larger --dist-max, smaller --robot-radius, "
                 "or check the front face / room restriction).")

    # Greedily pick top-N spatially-separated goals (so recommendations are distinct).
    cands.sort(key=lambda g: g["cost"])
    picked = []
    for g in cands:
        if all(math.hypot(g["x"] - p["x"], g["y"] - p["y"]) >= args.sep for p in picked):
            picked.append(g)
            if len(picked) >= max(1, args.top):
                break

    def to_goal(b, rank):
        qz, qw = math.sin(b["theta"] / 2), math.cos(b["theta"] / 2)
        return {
            "rank": rank, "carrier_id": c.get("id"), "carrier_label": c.get("label"), "view_face": view_face,
            "position": {"x": round(b["x"], 4), "y": round(b["y"], 4), "z": 0.0},
            "orientation": {"x": 0, "y": 0, "z": round(qz, 5), "w": round(qw, 5)},
            "theta_rad": round(b["theta"], 4),
            "viewing_distance": round(b["dist"], 3),
            "face_w_framed": b["face_w"], "face_h": round(face_h, 3),
            "camera_tilt_rad": b["cam_tilt"],
            "clearance": round(b["clearance"], 3),
            "nav2_cost": b["nav2_cost"],
            "robot_dist": round(b["robot_dist"], 3) if robot else None,
        }

    goals = [to_goal(b, i) for i, b in enumerate(picked)]
    out = ({"carrier_id": c.get("id"), "carrier_label": c.get("label"),
            "fov_deg": {"h": round(math.degrees(hfov), 1), "v": round(math.degrees(vfov), 1)},
            "goals": goals} if args.top > 1 else goals[0])
    print(json.dumps(out, indent=2, ensure_ascii=False))
    if args.out:
        Path(args.out).write_text(json.dumps(out, indent=2, ensure_ascii=False))
        print(f"[out] wrote {args.out}")
    if args.viz:
        _viz(args.viz, m, poly, centroid, picked, view_face)


def _point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _viz(path, m, poly, centroid, picked, view_face):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(np.where(m["occupied"], 0, np.where(m["free"], 255, 128)), cmap="gray", origin="upper")
    pp = np.array([world_to_px(x, y, m) for x, y in poly])
    ax.plot(np.append(pp[:, 0], pp[0, 0]), np.append(pp[:, 1], pp[0, 1]), "c-", lw=2, label="carrier")
    cpx = world_to_px(centroid[0], centroid[1], m)
    for i, b in enumerate(picked):
        gpx = world_to_px(b["x"], b["y"], m)
        col = "r" if i == 0 else "orange"
        ax.plot([gpx[0], cpx[0]], [gpx[1], cpx[1]], "--", color=col, lw=0.8, alpha=0.7)
        ax.plot(*gpx, "o", color=col, ms=10 if i == 0 else 7)
        ax.annotate("", xy=(gpx[0] + math.cos(b["theta"]) * 14, gpx[1] - math.sin(b["theta"]) * 14),
                    xytext=gpx, arrowprops=dict(arrowstyle="->", color=col, lw=2))
        ax.text(gpx[0] + 4, gpx[1] - 4, str(i), color=col, fontsize=9, fontweight="bold")
    b0 = picked[0]
    ax.set_title(f"observe {view_face}  top{len(picked)}  best: d={b0['dist']:.2f}m clr={b0['clearance']:.2f}m cost={b0['nav2_cost']}")
    ax.legend(); ax.axis("off")
    plt.tight_layout(); plt.savefig(path, dpi=90)
    print(f"[viz] wrote {path}")


if __name__ == "__main__":
    main()
