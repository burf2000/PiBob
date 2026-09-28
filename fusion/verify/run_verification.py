#!/usr/bin/env python3
"""Compare the Fusion-semantics preview (pibob_fusion.json only) against RViz.

Needs the local pibob-rviz container running (./ros2/docker/dev.sh, or
`docker run -d --name pibob-rviz -p 6080:6080 pibob-ros2-rviz:humble`).

    uv run --with numpy --with trimesh --with pillow \
        fusion/verify/run_verification.py

For each test pose it:
  1. drives /joint_states in the container (rviz_reference.py, replacing the GUI sliders),
  2. dumps every link's TF from robot_state_publisher and compares it numerically with
     preview_pose.pose_world() (the exact motion a Fusion revolute joint applies),
  3. screenshots RViz and renders the preview from the SAME camera, then saves
     preview | RViz | overlay  (red = preview only, cyan = RViz only, grey = both)
     plus the silhouette IoU, to fusion/verify/<pose>.png,
and writes fusion/verify/results.json.
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import preview_pose as pp  # noqa: E402

C = "pibob-rviz"
BG = (60, 90, 160)                      # RViz + preview background (never a model colour)
VIEWS = ["front_left", "front_right"]
D45 = math.radians(45)


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True, **kw).stdout


def dexec(script):
    return subprocess.run(["docker", "exec", C, "bash", "-c",
                           "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; " + script],
                          check=True, capture_output=True, text=True).stdout


def test_poses(data):
    poses = [("zero", {})]
    for j in data["joints"]:
        if j["type"] == "revolute":
            ang = min(D45, j["upper"]) if j["upper"] > 0 else max(-D45, j["lower"])
            poses.append((f"{j['name']}_{round(math.degrees(ang))}deg", {j["name"]: ang}))
    poses.append(("combined", {
        "head_pan_joint": 0.6, "head_tilt_joint": -0.4,
        "r_shoulder_rot_joint": 0.8, "r_shoulder_lift_joint": 0.3, "r_arm_rot_joint": -0.6, "r_bicep_joint": 1.2,
        "l_shoulder_rot_joint": -0.3, "l_shoulder_lift_joint": -0.3, "l_arm_rot_joint": 0.7, "l_bicep_joint": 0.9}))
    return poses


def rviz_config(view):
    v = pp.VIEWS[view]
    return f"""Panels: []
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
  Global Options:
    Fixed Frame: base_link
    Background Color: {BG[0]}; {BG[1]}; {BG[2]}
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: {v['distance']}
      Focal Point:
        X: {v['focal'][0]}
        Y: {v['focal'][1]}
        Z: {v['focal'][2]}
      Pitch: {v['pitch']}
      Yaw: {v['yaw']}
Window Geometry:
  Height: 900
  Width: 900
  X: 0
  Y: 0
  Hide Left Dock: true
  Hide Right Dock: true
"""


def setup_container():
    sh(f"docker cp {os.path.join(HERE, 'rviz_reference.py')} {C}:/tmp/rviz_reference.py")
    # Replace the slider GUI with our file-driven publisher (GUI ignores SIGTERM -> -9).
    # Kill in one exec, start in another: `pkill -f` matches its own bash -c command line
    # if that line also contains the start command.
    dexec("pkill -9 -f '[j]oint_state_publisher_gui'; pkill -9 -x rviz2; pkill -f '[r]viz_reference'; true")
    dexec("echo '{}' > /tmp/pose.json; nohup python3 /tmp/rviz_reference.py >/tmp/pub.log 2>&1 & sleep 1")
    if "import" not in dexec("command -v import || true"):
        dexec("apt-get update -qq && apt-get install -y -qq imagemagick >/dev/null")


def start_rviz(view):
    tmp = f"/tmp/verify_{view}.rviz"
    with open("/tmp/_v.rviz", "w") as f:
        f.write(rviz_config(view))
    sh(f"docker cp /tmp/_v.rviz {C}:{tmp}")
    dexec(f"pkill -9 -x rviz2; sleep 0.5; DISPLAY=:1 nohup rviz2 -d {tmp} >/tmp/rviz.log 2>&1 & sleep 9")


def set_pose(angles):
    with open("/tmp/_pose.json", "w") as f:
        json.dump(angles, f)
    sh(f"docker cp /tmp/_pose.json {C}:/tmp/pose.json")
    time.sleep(2.0)


def screenshot():
    dexec("DISPLAY=:1 import -window root /tmp/shot.png")
    sh(f"docker cp {C}:/tmp/shot.png /tmp/_shot.png")
    img = np.asarray(Image.open("/tmp/_shot.png").convert("RGB"))
    bgm = np.all(np.abs(img.astype(int) - BG) <= 3, axis=2)
    ys, xs = np.where(bgm)
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1    # the RViz render panel
    return img[y0:y1, x0:x1]


def mask(img):
    i = img.astype(int)
    return (np.abs(i[..., 0] - i[..., 1]) < 25) & (np.abs(i[..., 1] - i[..., 2]) < 25)


def tf_compare(data, W):
    ref = json.loads(dexec("python3 /tmp/rviz_reference.py --dump").strip().splitlines()[-1])
    worst_mm, worst_deg, per = 0.0, 0.0, {}
    for link, T in ref.items():
        T = np.asarray(T)
        P = W[link]
        dp = float(np.linalg.norm(T[:3, 3] - P[:3, 3])) * 1000
        c = (np.trace(T[:3, :3].T @ P[:3, :3]) - 1) / 2
        dr = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        per[link] = (round(dp, 4), round(dr, 4))
        worst_mm, worst_deg = max(worst_mm, dp), max(worst_deg, dr)
    return worst_mm, worst_deg, per


def main():
    data = pp.load()
    setup_container()
    poses = test_poses(data)
    results = {"poses": {}, "pivot_check": pp.pivot_check(data)}
    shots = {}
    for view in VIEWS:
        start_rviz(view)
        for name, ang in poses:
            set_pose(ang)
            shots[(name, view)] = screenshot()
            if view == VIEWS[0]:
                W = pp.pose_world(data, ang)
                mm, deg, per = tf_compare(data, W)
                results["poses"][name] = {"angles": ang, "tf_max_pos_err_mm": round(mm, 4),
                                          "tf_max_rot_err_deg": round(deg, 4), "per_link": per}
            print(f"{view:12s} {name:32s} captured", flush=True)

    for name, ang in poses:
        W = pp.pose_world(data, ang)
        moved = set()
        pivots = []
        for j in data["joints"]:
            if j["name"] in ang:
                moved |= set(pp.subtree(data, j["child"]))
                D = W[j["parent"]] @ np.linalg.inv(np.asarray(next(l for l in data["links"] if l["name"] == j["parent"])["world"]))
                pivots.append((D[:3, :3] @ np.asarray(j["origin_world"]) + D[:3, 3], D[:3, :3] @ np.asarray(j["axis_world"])))
        rows = []
        for view in VIEWS:
            rv = shots[(name, view)]
            h, w = rv.shape[:2]
            plain = pp.render(data, W, pp.VIEWS[view], w, h, grid=False, bg=tuple(c / 255 for c in BG))
            nice = pp.render(data, W, pp.VIEWS[view], w, h, grid=False, bg=tuple(c / 255 for c in BG),
                             highlight=moved, pivots=pivots)
            mp, mr = mask(plain[:h, :w]), mask(rv)
            iou = float((mp & mr).sum() / max(1, (mp | mr).sum()))
            ov = np.zeros((h, w, 3), np.uint8) + np.array(BG, np.uint8)
            ov[mp & mr] = (200, 200, 200)
            ov[mp & ~mr] = (255, 40, 40)
            ov[~mp & mr] = (40, 255, 255)
            results["poses"][name].setdefault("silhouette_iou", {})[view] = round(iou, 4)
            row = np.concatenate([nice[:h, :w], rv, ov], axis=1)
            im = Image.fromarray(row)
            d = ImageDraw.Draw(im)
            d.text((10, 8), f"PREVIEW (pibob_fusion.json, Fusion joint semantics) - {view}", fill="white")
            d.text((w + 10, 8), "RViz (robot_state_publisher from the URDF)", fill="white")
            d.text((2 * w + 10, 8), f"overlay: red=preview only, cyan=RViz only, grey=both   IoU={iou:.3f}",
                   fill="white")
            rows.append(np.asarray(im))
        full = Image.fromarray(np.concatenate(rows, axis=0))
        d = ImageDraw.Draw(full)
        r = results["poses"][name]
        d.text((10, full.height - 18),
               f"pose {name}: {json.dumps({k: round(v, 3) for k, v in ang.items()})}   "
               f"TF max err {r['tf_max_pos_err_mm']:.4f} mm / {r['tf_max_rot_err_deg']:.4f} deg", fill="yellow")
        full = full.resize((full.width // 2, full.height // 2), Image.LANCZOS)
        full.save(os.path.join(HERE, f"{name}.png"), optimize=True)
        print(f"{name:32s} TF {r['tf_max_pos_err_mm']:.4f} mm {r['tf_max_rot_err_deg']:.4f} deg  "
              f"IoU {r['silhouette_iou']}")

    with open(os.path.join(HERE, "results.json"), "w") as f:
        json.dump(results, f, indent=1, default=float)


if __name__ == "__main__":
    main()
