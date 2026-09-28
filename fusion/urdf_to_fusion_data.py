#!/usr/bin/env python3
"""URDF -> pibob_fusion.json: the data the Fusion 360 assembly script consumes.

Run from anywhere (plain Python 3 + the `xacro` package):

    uv run --with xacro --with numpy fusion/urdf_to_fusion_data.py
    # or:  pip install xacro numpy && python3 fusion/urdf_to_fusion_data.py

It expands ros2/pibob_description/urdf/pibob.urdf.xacro, walks the kinematic
tree at the ZERO pose, and writes fusion/pibob_fusion.json with:

  links[]:  name, parent_joint, parent_link,
            world      4x4 (metres) link frame in the world at zero pose
            visuals[]: stl (path relative to the JSON's folder), stl_repo (relative to repo root),
                       origin 4x4 (metres) mesh frame inside the link (URDF <visual><origin>),
                       scale [sx,sy,sz], rgba (URDF material colour, or null)
  joints[]: name, type, parent, child,
            origin_local 4x4 (metres) joint frame in the parent link frame (URDF <origin>),
            origin_world [x,y,z] (metres) pivot in the world at zero pose,
            axis_local   unit vector in the CHILD frame (URDF <axis>),
            axis_world   unit vector in the world at zero pose,
            lower/upper  (rad), or null for fixed joints

Everything is metres/radians, exactly as in the URDF. The Fusion script does the
metre -> centimetre conversion (Fusion's API works in cm internally).

URDF semantics this relies on: at the zero pose a child link's frame IS the joint
frame, so the pivot = the child link's origin and the rotation axis is the URDF
<axis> expressed in the child link's frame.
"""
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
XACRO = os.path.join(REPO, "ros2", "pibob_description", "urdf", "pibob.urdf.xacro")
OUT = os.path.join(HERE, "pibob_fusion.json")

# package://<pkg>/<rest>  ->  repo path. Only pibob_description is used.
PACKAGE_DIRS = {"pibob_description": os.path.join("ros2", "pibob_description")}


# ---------------------------------------------------------------- maths
def rpy_to_matrix(r, p, y):
    """URDF rpy = fixed-axis X(roll), Y(pitch), Z(yaw):  R = Rz(y) @ Ry(p) @ Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_to_T(el):
    """<origin xyz rpy> element (or None) -> 4x4."""
    T = np.eye(4)
    if el is None:
        return T
    xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in el.get("rpy", "0 0 0").split()]
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def clean(T):
    """Round away float noise so the JSON is readable (1e-12 -> 0)."""
    return [[0.0 if abs(v) < 1e-12 else round(float(v), 12) for v in row] for row in np.asarray(T)]


# ---------------------------------------------------------------- xacro
def expand_xacro(path):
    """Expand the xacro. Tries the python `xacro` module, then the `xacro` CLI."""
    try:
        import xacro  # noqa: F401  (pip/uv package "xacro")
        doc = xacro.process_file(path)
        return doc.toxml()
    except ImportError:
        pass
    try:
        return subprocess.check_output(["xacro", path], text=True)
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit(f"Could not expand {path}: install xacro (`uv run --with xacro ...`). ({e})")


def resolve_mesh(uri):
    if uri.startswith("package://"):
        pkg, _, rest = uri[len("package://"):].partition("/")
        if pkg not in PACKAGE_DIRS:
            sys.exit(f"Unknown package in mesh uri: {uri}")
        return os.path.join(PACKAGE_DIRS[pkg], rest)
    if uri.startswith("file://"):
        return os.path.relpath(uri[len("file://"):], REPO)
    return uri


# ---------------------------------------------------------------- main
def build(urdf_xml):
    robot = ET.fromstring(urdf_xml)
    links = {l.get("name"): l for l in robot.findall("link")}
    materials = {m.get("name"): [float(c) for c in m.find("color").get("rgba").split()]
                 for m in robot.findall("material") if m.find("color") is not None}
    joints = robot.findall("joint")
    child_of = {j.find("child").get("link"): j for j in joints}
    roots = [n for n in links if n not in child_of]
    if len(roots) != 1:
        sys.exit(f"Expected one root link, got {roots}")

    children = {}
    for j in joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)

    # Breadth-first from the root, so parents always precede children in the JSON
    # (the Fusion script and preview_pose.py rely on that order).
    world = {roots[0]: np.eye(4)}
    order, jorder, queue = [roots[0]], [], [roots[0]]
    while queue:
        p = queue.pop(0)
        for j in children.get(p, []):
            c = j.find("child").get("link")
            world[c] = world[p] @ origin_to_T(j.find("origin"))
            order.append(c)
            jorder.append(j)
            queue.append(c)

    out_links = []
    for name in order:
        vis = []
        for v in links[name].findall("visual"):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                print(f"WARNING: {name} has a non-mesh visual; skipped (primitives not supported)")
                continue
            scale = [float(s) for s in mesh.get("scale", "1 1 1").split()]
            if any(s <= 0 for s in scale):
                sys.exit(f"{name}: negative/zero mesh scale {scale} = a mirror; Fusion occurrences can't mirror")
            T = origin_to_T(v.find("origin"))
            if np.linalg.det(T[:3, :3]) < 0.999:
                sys.exit(f"{name}: visual origin is not a proper rotation")
            repo_rel = resolve_mesh(mesh.get("filename"))
            mat = v.find("material")
            rgba = None
            if mat is not None:
                col = mat.find("color")
                rgba = [float(c) for c in col.get("rgba").split()] if col is not None else materials.get(mat.get("name"))
            vis.append({
                "stl_repo": repo_rel,
                "stl": os.path.relpath(os.path.join(REPO, repo_rel), HERE),
                "origin": clean(T),
                "scale": scale,
                "rgba": rgba,
            })
        j = child_of.get(name)
        out_links.append({
            "name": name,
            "parent_joint": j.get("name") if j is not None else None,
            "parent_link": j.find("parent").get("link") if j is not None else None,
            "world": clean(world[name]),
            "visuals": vis,
        })

    out_joints = []
    for j in jorder:
        jtype = j.get("type")
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        ax_el = j.find("axis")
        axis = np.array([float(a) for a in (ax_el.get("xyz") if ax_el is not None else "1 0 0").split()])
        axis = axis / np.linalg.norm(axis)
        lim = j.find("limit")
        entry = {
            "name": j.get("name"), "type": jtype, "parent": p, "child": c,
            "origin_local": clean(origin_to_T(j.find("origin"))),
            "origin_world": [round(float(v), 12) for v in world[c][:3, 3]],
            "axis_local": [round(float(v), 12) for v in axis],
            "axis_world": [round(float(v), 12) for v in world[c][:3, :3] @ axis],
            "lower": float(lim.get("lower")) if (lim is not None and jtype == "revolute") else None,
            "upper": float(lim.get("upper")) if (lim is not None and jtype == "revolute") else None,
        }
        if jtype not in ("revolute", "fixed", "continuous"):
            print(f"WARNING: joint {entry['name']} type {jtype} not handled by the Fusion script (treated as rigid)")
        out_joints.append(entry)

    return {
        "robot": robot.get("name"),
        "units": {"length": "m", "angle": "rad"},
        "source": os.path.relpath(XACRO, REPO),
        "root_link": roots[0],
        "links": out_links,
        "joints": out_joints,
    }


def main():
    data = build(expand_xacro(XACRO))
    missing = [v["stl_repo"] for l in data["links"] for v in l["visuals"]
               if not os.path.isfile(os.path.join(REPO, v["stl_repo"]))]
    if missing:
        sys.exit(f"Missing STL(s): {missing}")
    with open(OUT, "w") as f:
        json.dump(data, f, indent=1)
    nrev = sum(j["type"] == "revolute" for j in data["joints"])
    print(f"wrote {os.path.relpath(OUT, REPO)}: {len(data['links'])} links, "
          f"{len(data['joints'])} joints ({nrev} revolute)")
    for j in data["joints"]:
        if j["type"] != "fixed":
            print(f"  {j['name']:24s} pivot(mm)={[round(v*1000, 1) for v in j['origin_world']]} "
                  f"axis={j['axis_world']} [{j['lower']:.3f}, {j['upper']:.3f}]")


if __name__ == "__main__":
    main()
