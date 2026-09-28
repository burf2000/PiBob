#!/usr/bin/env python3
"""Pose + render PiBob using ONLY pibob_fusion.json - the same data the Fusion script uses.

    uv run --with numpy --with trimesh --with pillow fusion/preview_pose.py \
        --pose head_pan_joint=0.785 --view front_left --out /tmp/p.png
    uv run ... fusion/preview_pose.py --pivot-check

Motion semantics are exactly what a Fusion revolute joint does: for each revolute
joint (parents first) rotate the whole CHILD SUBTREE by the angle about the line
through the joint's CURRENT world pivot along its CURRENT world axis. Nothing here
reads the URDF, so agreement with RViz (robot_state_publisher) is a real check of
the JSON pivots/axes, not a restatement of them.

The renderer is a tiny z-buffered pinhole-camera rasteriser (numpy + Pillow),
using the same camera model as RViz's Orbit view (eye = focal + d*(cos p cos y,
cos p sin y, sin p), Z up, vertical FOV 45 deg), so preview and RViz screenshots can
be laid side by side and overlaid pixel-for-pixel.
"""
import argparse
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_JSON = os.path.join(HERE, "pibob_fusion.json")

# Named cameras shared with verify/run_verification.py (RViz Orbit parameters).
VIEWS = {
    "front_left":  dict(yaw=0.70, pitch=0.30, distance=0.75, focal=(0.0, 0.0, 0.29)),
    "front_right": dict(yaw=-0.70, pitch=0.30, distance=0.75, focal=(0.0, 0.0, 0.29)),
    "left_side":   dict(yaw=1.5708, pitch=0.15, distance=0.75, focal=(0.0, 0.0, 0.29)),
}
FOVY = math.pi / 4          # RViz/Ogre default camera vertical field of view


# ---------------------------------------------------------------- data
def load(json_path=DEFAULT_JSON):
    with open(json_path) as f:
        data = json.load(f)
    data["_dir"] = os.path.dirname(os.path.abspath(json_path))
    return data


_MESH_CACHE = {}


def load_mesh(data, visual):
    """STL -> (vertices Nx3 in METRES in the link frame, faces Mx3)."""
    import trimesh
    path = os.path.join(data["_dir"], visual["stl"])
    if path not in _MESH_CACHE:
        m = trimesh.load(path, force="mesh", process=False)
        _MESH_CACHE[path] = (np.asarray(m.vertices, float), np.asarray(m.faces, int))
    v, f = _MESH_CACHE[path]
    v = v * np.asarray(visual["scale"])
    T = np.asarray(visual["origin"])
    return (T[:3, :3] @ v.T).T + T[:3, 3], f


# ---------------------------------------------------------------- kinematics
def axis_angle(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def rot_about_line(point, axis, ang):
    """4x4 world transform: rotate by ang about the line (point, axis)."""
    R = axis_angle(axis, ang)
    p = np.asarray(point, float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p - R @ p
    return T


def subtree(data, link):
    kids = {}
    for j in data["joints"]:
        kids.setdefault(j["parent"], []).append(j["child"])
    out, stack = [], [link]
    while stack:
        n = stack.pop()
        out.append(n)
        stack.extend(kids.get(n, []))
    return out


def pose_world(data, angles):
    """{link: posed 4x4 world transform}. angles = {joint_name: rad} (missing = 0)."""
    W0 = {l["name"]: np.asarray(l["world"], float) for l in data["links"]}
    W = {k: v.copy() for k, v in W0.items()}
    for j in data["joints"]:                        # JSON order = parents first
        ang = float(angles.get(j["name"], 0.0))
        if j["type"] == "fixed" or ang == 0.0:
            continue
        # The joint's pivot/axis move with everything upstream of it: carry the
        # zero-pose world pivot/axis through the parent link's accumulated motion.
        D = W[j["parent"]] @ np.linalg.inv(W0[j["parent"]])
        pivot = D[:3, :3] @ np.asarray(j["origin_world"]) + D[:3, 3]
        axis = D[:3, :3] @ np.asarray(j["axis_world"])
        R = rot_about_line(pivot, axis, ang)
        for n in subtree(data, j["child"]):
            W[n] = R @ W[n]
    return W


def posed_meshes(data, W):
    """[(link, verts_world, faces, rgba)] at the posed transforms."""
    out = []
    for l in data["links"]:
        for vis in l["visuals"]:
            v, f = load_mesh(data, vis)
            T = W[l["name"]]
            out.append((l["name"], (T[:3, :3] @ v.T).T + T[:3, 3], f, vis.get("rgba") or [0.8, 0.8, 0.8, 1]))
    return out


# ---------------------------------------------------------------- render
def camera(view):
    y, p, d = view["yaw"], view["pitch"], view["distance"]
    f = np.asarray(view["focal"], float)
    eye = f + d * np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p)])
    fwd = (f - eye) / np.linalg.norm(f - eye)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return eye, fwd, right, up


def project(pts, view, w, h):
    eye, fwd, right, up = camera(view)
    v = pts - eye
    zc = v @ fwd
    k = (h / 2) / math.tan(FOVY / 2)
    sx = w / 2 + k * (v @ right) / zc
    sy = h / 2 - k * (v @ up) / zc
    return np.stack([sx, sy], 1), zc


def _rasterise(img, zbuf, s, z, cols):
    """Z-buffered flat-shaded triangle fill. s: Mx3x2 screen pts, z: Mx3 depths, cols: Mx3 (0..1)."""
    h, w = zbuf.shape
    for t in range(len(s)):
        (x0, y0), (x1, y1), (x2, y2) = s[t]
        if min(z[t]) <= 0:
            continue
        xa, xb = max(int(math.floor(min(x0, x1, x2))), 0), min(int(math.ceil(max(x0, x1, x2))), w - 1)
        ya, yb = max(int(math.floor(min(y0, y1, y2))), 0), min(int(math.ceil(max(y0, y1, y2))), h - 1)
        if xa > xb or ya > yb:
            continue
        den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(den) < 1e-12:
            continue
        X, Y = np.meshgrid(np.arange(xa, xb + 1) + 0.5, np.arange(ya, yb + 1) + 0.5)
        l0 = ((y1 - y2) * (X - x2) + (x2 - x1) * (Y - y2)) / den
        l1 = ((y2 - y0) * (X - x2) + (x0 - x2) * (Y - y2)) / den
        l2 = 1 - l0 - l1
        inside = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9)
        if not inside.any():
            continue
        # perspective-correct depth: interpolate 1/z
        zz = 1.0 / (l0 / z[t][0] + l1 / z[t][1] + l2 / z[t][2])
        sub = zbuf[ya:yb + 1, xa:xb + 1]
        win = inside & (zz < sub)
        sub[win] = zz[win]
        img[ya:yb + 1, xa:xb + 1][win] = cols[t]


def render(data, W, view, w=850, h=790, highlight=(), pivots=(), out=None, title=None,
           grid=True, bg=(1, 1, 1)):
    """Render posed meshes (z-buffered). highlight = links tinted orange; pivots = [(point, axis)] in red.
    Returns an HxWx3 uint8 image (and saves it to `out` if given)."""
    from PIL import Image, ImageDraw

    eye, fwd, _, _ = camera(view)
    img = np.zeros((h, w, 3), float) + np.asarray(bg, float)
    zbuf = np.full((h, w), np.inf)

    if grid:  # RViz default grid: 10 x 10 cells of 0.1 m on z=0, centred at origin (drawn under the model)
        g = Image.fromarray((img * 255).astype(np.uint8))
        d = ImageDraw.Draw(g)
        for i in range(11):
            c = -0.5 + i * 0.1
            for a, b in (([c, -0.5, 0], [c, 0.5, 0]), ([-0.5, c, 0], [0.5, c, 0])):
                sp, _ = project(np.array([a, b], float), view, w, h)
                d.line([tuple(sp[0]), tuple(sp[1])], fill=(180, 180, 180), width=1)
        img = np.asarray(g, float) / 255.0

    for link, v, f, rgba in posed_meshes(data, W):
        tri = v[f]                                           # M x 3 x 3
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.35 + 0.65 * np.abs(n @ -fwd)
        base = np.array([1.0, 0.55, 0.1]) if link in highlight else np.array(rgba[:3])
        sp, z = project(tri.reshape(-1, 3), view, w, h)
        _rasterise(img, zbuf, sp.reshape(-1, 3, 2), z.reshape(-1, 3),
                   np.clip(base[None, :] * shade[:, None], 0, 1))

    im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    d = ImageDraw.Draw(im)
    for p, a in pivots:  # pivot dot + 60 mm axis line through it (drawn on top)
        a = np.asarray(a) / np.linalg.norm(a)
        sp, _ = project(np.array([p - 0.03 * a, p + 0.03 * a, p]), view, w, h)
        d.line([tuple(sp[0]), tuple(sp[1])], fill=(230, 0, 0), width=3)
        x, y = sp[2]
        d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(230, 0, 0))
    if title:
        d.text((10, 10), title, fill=(0, 0, 0) if sum(bg) > 1.5 else (255, 255, 255))
    if out:
        im.save(out)
    return np.asarray(im)


# ---------------------------------------------------------------- pivot sanity check
def _point_tri_dist(p, tri):
    """Distance from p to each triangle (M x 3 x 3), vectorised (Ericson, closest point)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = (ab * ap).sum(1), (ac * ap).sum(1)
    bp = p - b
    d3, d4 = (ab * bp).sum(1), (ac * bp).sum(1)
    cp = p - c
    d5, d6 = (ab * cp).sum(1), (ac * cp).sum(1)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    denom = va + vb + vc
    denom[denom == 0] = 1e-18
    v, w = vb / denom, vc / denom
    closest = a + ab * v[:, None] + ac * w[:, None]
    # region tests (vertex / edge regions)
    m = (d1 <= 0) & (d2 <= 0); closest[m] = a[m]
    m = (d3 >= 0) & (d4 <= d3); closest[m] = b[m]
    m = (d6 >= 0) & (d5 <= d6); closest[m] = c[m]
    m = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    t = d1 / np.where(d1 - d3 == 0, 1e-18, d1 - d3); closest[m] = (a + ab * t[:, None])[m]
    m = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    t = d2 / np.where(d2 - d6 == 0, 1e-18, d2 - d6); closest[m] = (a + ac * t[:, None])[m]
    m = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    t = (d4 - d3) / np.where((d4 - d3) + (d5 - d6) == 0, 1e-18, (d4 - d3) + (d5 - d6))
    closest[m] = (b + (c - b) * t[:, None])[m]
    return np.linalg.norm(p - closest, axis=1)


def _inside(p, tri):
    """Ray-parity point-in-mesh test (+X ray; ok for closed printable STLs)."""
    d = np.array([1.0, 0.1234, 0.0567]); d /= np.linalg.norm(d)
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    h = np.cross(d, e2)
    a = (e1 * h).sum(1)
    ok = np.abs(a) > 1e-15
    f = np.where(ok, 1 / np.where(ok, a, 1), 0)
    s = p - tri[:, 0]
    u = f * (s * h).sum(1)
    q = np.cross(s, e1)
    v = f * (q @ d)
    t = f * (e2 * q).sum(1)
    hits = ok & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (t > 1e-12)
    return bool(hits.sum() % 2)


def pivot_check(data):
    """For each revolute joint: distance (mm) from the pivot to the parent- and child-link meshes."""
    W = pose_world(data, {})
    meshes = {}
    for link, v, f, _ in posed_meshes(data, W):
        meshes.setdefault(link, []).append(v[f])
    rows = []
    for j in data["joints"]:
        if j["type"] == "fixed":
            continue
        p = np.asarray(j["origin_world"])
        row = {"joint": j["name"]}
        for side in ("parent", "child"):
            tris = np.concatenate(meshes.get(j[side], [np.zeros((0, 3, 3))]))
            if len(tris) == 0:
                row[side] = None
                continue
            dist = float(_point_tri_dist(p, tris).min()) * 1000
            ins = any(_inside(p, t) for t in meshes[j[side]])
            lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
            # how far outside the part's axis-aligned envelope (0 = within it, e.g. in the servo pocket)
            out_bb = float(np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi)))) * 1000
            row[side] = {"link": j[side], "surface_mm": round(dist, 2), "inside_material": ins,
                         "outside_envelope_mm": round(out_bb, 2)}
        rows.append(row)
    return rows


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--pose", nargs="*", default=[], help="joint=radians ...")
    ap.add_argument("--view", default="front_left", choices=sorted(VIEWS))
    ap.add_argument("--out", default="preview.png")
    ap.add_argument("--pivot-check", action="store_true")
    a = ap.parse_args()
    data = load(a.json)
    if a.pivot_check:
        for r in pivot_check(data):
            def fmt(s):
                return "-" if s is None else (f"{s['link']:21s} surf {s['surface_mm']:5.1f}mm "
                                              f"env+{s['outside_envelope_mm']:4.1f}mm")
            print(f"{r['joint']:24s} parent: {fmt(r['parent'])} | child: {fmt(r['child'])}")
        return
    angles = {k: float(v) for k, v in (p.split("=") for p in a.pose)}
    W = pose_world(data, angles)
    render(data, W, VIEWS[a.view], out=a.out, title=" ".join(a.pose) or "zero pose")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
