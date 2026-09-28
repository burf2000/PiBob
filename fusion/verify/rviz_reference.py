#!/usr/bin/env python3
"""Reference side of the Fusion verification: runs INSIDE the pibob-rviz container.

Publishes /joint_states from /tmp/pose.json (re-read every tick, so the host can
change the pose by rewriting the file), and on request dumps the TF of every
link relative to base_link as computed by robot_state_publisher from the URDF.

    python3 rviz_reference.py            # publisher loop (background it)
    python3 rviz_reference.py --dump     # print {link: 4x4} for the current pose, exit

This is the independent ground truth: robot_state_publisher composes the URDF's
local <origin>/<axis> chain, whereas preview_pose.py only uses the WORLD pivots
and axes stored in pibob_fusion.json (the same data the Fusion script uses).
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

POSE = "/tmp/pose.json"
JOINTS = ["head_pan_joint", "head_tilt_joint"] + [
    f"{s}_{j}_joint" for s in ("r", "l") for j in ("shoulder_rot", "shoulder_lift", "arm_rot", "bicep")]
LINKS = ["shoulder_beam", "head_base_link", "head_pan_link", "head_tilt_link", "camera_link"] + [
    f"{s}_{l}" for s in ("r", "l") for l in (
        "shoulder_top_link", "shoulder_rot_link", "shoulder_lift_link", "upper_arm_link", "forearm_link")]


def read_pose():
    try:
        with open(POSE) as f:
            return json.load(f)
    except Exception:
        return {}


def publish_loop():
    rclpy.init()
    node = Node("fusion_verify_pose")
    pub = node.create_publisher(JointState, "joint_states", 10)

    def tick():
        pose = read_pose()
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(pose.get(j, 0.0)) for j in JOINTS]
        pub.publish(msg)

    node.create_timer(0.05, tick)
    rclpy.spin(node)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def dump_tf():
    import tf2_ros
    rclpy.init()
    node = Node("fusion_verify_dump")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end = time.time() + 3.0          # let the buffer fill with the current pose
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {}
    for link in LINKS:
        t = buf.lookup_transform("base_link", link, rclpy.time.Time()).transform
        R = quat_to_matrix(t.rotation)
        out[link] = [R[0] + [t.translation.x], R[1] + [t.translation.y],
                     R[2] + [t.translation.z], [0, 0, 0, 1]]
    print(json.dumps(out))


if __name__ == "__main__":
    if "--dump" in sys.argv:
        dump_tf()
    else:
        publish_loop()
