#!/usr/bin/env python3
"""Do the joints turn about the physical pins? Evidence from the meshes, not the URDF.

    uv run --with numpy --with trimesh --with pillow fusion/verify/pin_check.py [--ref OLD.json]

1. Finds every cylindrical feature in the STLs (bosses = pins / servo splines, holes =
   pin holes, hub holes, horn seats): groups the curved facets per principal axis,
   fits a circle, and classifies boss vs hole from the facet normals.
2. For each revolute joint: the PIN is the boss on that joint's axis (DS3218 holders'
   dia-5.8 pins, the 9 g holders' dia-4.7 pins / servo spline). Reports the joint
   origin's distance to the pin axis, the axis angle error, and every hole of the other
   part on that axis (how far off the pin it sits = a seating error of that part).
3. Sweeps each joint across its limits (<= +-45 deg) with preview_pose.pose_world (the
   Fusion joint semantics) and measures how far the CHILD's own rotation feature
   (its hole, or its pin) moves: 0 = it turns on the spot about the pin.
4. With --ref: max vertex displacement of every part at zero pose vs an older JSON
   (proves the joint frames moved but the parts did not).
5. Renders a close-up per joint (verify/pins/<joint>.png): lower limit / zero / upper,
   pin axis in red, child feature centre in green.
Writes verify/pin_check.json.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import preview_pose as pp  # noqa: E402

SWEEP = math.radians(45)


# ---------------------------------------------------------------- feature detection
def cyl_features(path, link_mm=2.5):
    """[(kind, axis_index, centre(3,), radius, lo, hi)] in the STL's own units (mm)."""
    m = trimesh.load(path, force="mesh", process=False)
    v, F, N, C = np.asarray(m.vertices), np.asarray(m.faces), m.face_normals, m.triangles_center
    out = []
    for a in range(3):
        o = [i for i in range(3) if i != a]
        idx = np.where((np.abs(N[:, a]) < 1e-3) & (np.abs(N[:, o[0]]) > 1e-3) & (np.abs(N[:, o[1]]) > 1e-3))[0]
        if len(idx) < 6:
            continue
        P = C[idx]
        lab = -np.ones(len(idx), int)
        k = 0
        for i in range(len(idx)):              # single-linkage clusters of curved facets
            if lab[i] >= 0:
                continue
            stack, lab[i] = [i], k
            while stack:
                j = stack.pop()
                for n in np.where((np.linalg.norm(P - P[j], axis=1) < link_mm) & (lab < 0))[0]:
                    lab[n] = k
                    stack.append(n)
            k += 1
        for c in range(k):
            fi = idx[lab == c]
            if len(fi) < 8:
                continue
            pts = v[np.unique(F[fi])]
            q = pts[:, o]
            A = np.c_[2 * q, np.ones(len(q))]
            sol, *_ = np.linalg.lstsq(A, (q ** 2).sum(1), rcond=None)
            r = math.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2)
            if np.abs(np.linalg.norm(q - sol[:2], axis=1) - r).max() > 0.05 or r > 25:
                continue
            outward = np.sign(((C[fi][:, o] - sol[:2]) * N[fi][:, o]).sum(1)).mean() > 0
            ctr = np.zeros(3)
            ctr[o] = sol[:2]
            out.append(("BOSS" if outward else "HOLE", a, ctr, r, pts[:, a].min(), pts[:, a].max(),
                        m.bounds.mean(0)))
    return out


_FEAT = {}


def world_features(data, W, link):
    """Features of every visual of `link`, in metres, at link transforms W."""
    out = []
    lk = next(l for l in data["links"] if l["name"] == link)
    for vis in lk["visuals"]:
        path = os.path.normpath(os.path.join(data["_dir"], vis["stl"]))
        if path not in _FEAT:
            _FEAT[path] = cyl_features(path)
        T = W[link] @ np.asarray(vis["origin"])
        s = vis["scale"][0]
        for kind, a, ctr, r, lo, hi, cen in _FEAT[path]:
            e = np.zeros(3)
            e[a] = 1
            ends = []
            for t in (lo, hi):
                q = ctr.copy()
                q[a] = t
                ends.append(q)
            base, tip = sorted(ends, key=lambda q: abs(q[a] - cen[a]))   # base = end at the part
            tw = lambda q: T[:3, :3] @ (q * s) + T[:3, 3]  # noqa: E731
            out.append(dict(link=link, part=os.path.splitext(os.path.basename(path))[0], kind=kind,
                            dia_mm=round(2 * r, 2), base=tw(base), tip=tw(tip), dir=T[:3, :3] @ e))
    return out


def line_dist(p, a, d):
    v = p - a
    return float(np.linalg.norm(v - (v @ d) * d))


# ---------------------------------------------------------------- checks
def rigid_groups(data):
    """link -> set of links rigidly attached to it through fixed joints."""
    adj = {}
    for j in data["joints"]:
        if j["type"] == "fixed":
            adj.setdefault(j["parent"], []).append(j["child"])
            adj.setdefault(j["child"], []).append(j["parent"])

    def group(link):
        seen, stack = {link}, [link]
        while stack:
            for n in adj.get(stack.pop(), []):
                if n not in seen:
                    seen.add(n)
                    stack.append(n)
        return seen
    return group


def joint_report(data):
    W0 = pp.pose_world(data, {})
    group = rigid_groups(data)
    rows = []
    for j in data["joints"]:
        if j["type"] != "revolute":
            continue
        o, a = np.asarray(j["origin_world"]), np.asarray(j["axis_world"])
        side_of = {lk: "parent" for lk in group(j["parent"])}
        side_of.update({lk: "child" for lk in group(j["child"])})
        feats = [dict(f, side=side_of[lk]) for lk in side_of for f in world_features(data, W0, lk)
                 if abs(f["dir"] @ a) > math.cos(math.radians(5))]
        pins = [f for f in feats if f["kind"] == "BOSS"]
        if not pins:
            rows.append({"joint": j["name"], "pin": None})
            continue
        # the pin nearest the joint origin (on-axis first, then closest root)
        pin = min(pins, key=lambda f: (round(line_dist(o, f["base"], f["dir"]), 4),
                                       float(np.linalg.norm(f["base"] - o))))
        others = [f for f in feats if f is not pin]
        row = {
            "joint": j["name"],
            "pin": {"link": pin["link"], "part": pin["part"], "dia_mm": pin["dia_mm"],
                    "root_mm": (pin["base"] * 1000).round(2).tolist()},
            "origin_to_pin_axis_mm": round(line_dist(o, pin["base"], pin["dir"]) * 1000, 3),
            "axis_angle_deg": round(math.degrees(math.acos(min(1.0, abs(float(pin["dir"] @ a))))), 3),
            "on_axis": [{"link": f["link"], "part": f["part"], "kind": f["kind"], "dia_mm": f["dia_mm"],
                         "centre_mm": (f["base"] * 1000).round(2).tolist(),
                         "off_pin_axis_mm": round(line_dist(f["base"], pin["base"], pin["dir"]) * 1000, 2)}
                        for f in others],
        }
        # sweep: the child's own feature on this axis (its hole, or the pin if the child owns it)
        # the child's pin hole = its smallest hole on this axis
        child_feats = [f for f in feats if f["side"] == "child" and f["kind"] == "HOLE"]
        cf = pin if pin["side"] == "child" else (min(child_feats, key=lambda f: f["dia_mm"]) if child_feats else None)
        lo, hi = max(j["lower"], -SWEEP), min(j["upper"], SWEEP)
        drift = 0.0
        if cf is not None:
            c0 = cf["base"]
            Tc0 = W0[cf["link"]]
            for ang in np.linspace(lo, hi, 13):
                W = pp.pose_world(data, {j["name"]: float(ang)})
                M = W[cf["link"]] @ np.linalg.inv(Tc0)
                drift = max(drift, float(np.linalg.norm(M[:3, :3] @ c0 + M[:3, 3] - c0)))
            row["child_feature"] = f"{cf['part']} {cf['kind'].lower()} dia {cf['dia_mm']}"
            row["child_feature_centre"] = {"link": cf["link"], "p": cf["base"].tolist()}
        row["sweep_deg"] = [round(math.degrees(lo), 1), round(math.degrees(hi), 1)]
        row["child_feature_drift_mm"] = round(drift * 1000, 3) if cf is not None else None
        rows.append(row)
    return rows


def zero_pose_diff(data, ref_path):
    ref = pp.load(ref_path)
    ref["_dir"] = data["_dir"]          # STL paths in the JSON are relative to fusion/
    cur = {(l, i): v for i, (l, v, f, _) in enumerate(pp.posed_meshes(data, pp.pose_world(data, {})))}
    old = {(l, i): v for i, (l, v, f, _) in enumerate(pp.posed_meshes(ref, pp.pose_world(ref, {})))}
    worst = 0.0
    for k, v in old.items():
        if k in cur and cur[k].shape == v.shape:
            worst = max(worst, float(np.abs(cur[k] - v).max()))
        else:
            raise SystemExit(f"visual list differs from the reference at {k}")
    return worst * 1000


def render_joints(data, rows):
    """Per joint: lower / zero / upper, only the two parts at the joint, looking down the
    pin (from its tip). Red = pin axis through the joint origin; green ring = where the
    child's pin hole (or its pin) is at that angle, slid along the axis into the pin's
    plane so only its RADIAL offset shows - it should sit on the red dot."""
    from PIL import Image, ImageDraw
    os.makedirs(os.path.join(HERE, "pins"), exist_ok=True)
    spec = {j["name"]: j for j in data["joints"]}
    group = rigid_groups(data)
    W0 = pp.pose_world(data, {})
    for r in rows:
        if not r.get("pin"):
            continue
        j = spec[r["joint"]]
        a = np.asarray(j["axis_world"])
        o = np.asarray(j["origin_world"])
        pin = next(f for lk in group(j["parent"]) | group(j["child"]) for f in world_features(data, W0, lk)
                   if f["part"] == r["pin"]["part"] and np.allclose(f["base"] * 1000, r["pin"]["root_mm"], atol=0.01))
        look = pin["tip"] - pin["base"]
        look = look / np.linalg.norm(look)
        look = look + 0.35 * np.cross(look, [0, 0, 1] if abs(look[2]) < 0.9 else [1, 0, 0])   # a little off-axis
        look = look / np.linalg.norm(look)
        view = dict(yaw=math.atan2(look[1], look[0]), pitch=math.asin(look[2]), distance=0.16, focal=tuple(o))
        links = sorted(group(j["parent"]) | group(j["child"]))
        lo, hi = max(j["lower"], -SWEEP), min(j["upper"], SWEEP)
        cf = r.get("child_feature_centre")
        tiles = []
        for ang in (lo, 0.0, hi):
            W = pp.pose_world(data, {j["name"]: ang})
            marks = []
            if cf is not None:
                M = W[cf["link"]] @ np.linalg.inv(W0[cf["link"]])
                c = M[:3, :3] @ np.asarray(cf["p"]) + M[:3, 3]
                marks.append((c - ((c - o) @ a) * a, (0, 170, 0)))     # slid along the axis into the pin's plane
            img = pp.render(data, W, view, 520, 480, grid=False, links=links,
                            highlight=set(group(j["child"])), pivots=[(o, a)], marks=marks,
                            title=f"{j['name']}  {math.degrees(ang):+.0f} deg")
            tiles.append(Image.fromarray(img))
        sheet = Image.new("RGB", (1560, 500), "white")
        for i, t in enumerate(tiles):
            sheet.paste(t, (i * 520, 0))
        ImageDraw.Draw(sheet).text(
            (10, 484), f"pin: {r['pin']['part']} dia {r['pin']['dia_mm']} (red axis)  |  green ring: child "
                       f"{r.get('child_feature', '-')}  |  origin->pin axis {r['origin_to_pin_axis_mm']} mm  |  "
                       f"ring drift over sweep {r['child_feature_drift_mm']} mm", fill=(0, 0, 0))
        sheet.save(os.path.join(HERE, "pins", f"{r['joint']}.png"), optimize=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", help="older pibob_fusion.json to prove zero-pose geometry is unchanged")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args()
    data = pp.load()
    rows = joint_report(data)
    out = {"joints": rows}
    if a.ref:
        out["zero_pose_max_vertex_displacement_mm"] = round(zero_pose_diff(data, a.ref), 6)
        print(f"zero pose vs {a.ref}: max vertex displacement {out['zero_pose_max_vertex_displacement_mm']} mm")
    for r in rows:
        if not r.get("pin"):
            print(f"{r['joint']:24s} no pin found")
            continue
        print(f"{r['joint']:24s} pin {r['pin']['part']:28s} d{r['pin']['dia_mm']:<4} root {r['pin']['root_mm']}  "
              f"origin->axis {r['origin_to_pin_axis_mm']:.3f} mm  axis err {r['axis_angle_deg']:.3f} deg  "
              f"child drift {r['child_feature_drift_mm']} mm over {r['sweep_deg']} ({r.get('child_feature')})")
        for f in r["on_axis"]:
            print(f"{'':26s}{f['kind'].lower():4s} d{f['dia_mm']:<5} {f['part']:28s} off pin axis {f['off_pin_axis_mm']:.2f} mm")
    with open(os.path.join(HERE, "pin_check.json"), "w") as f:
        json.dump(out, f, indent=1, default=float)
    if not a.no_render:
        render_joints(data, rows)


if __name__ == "__main__":
    main()
