# PiBob ROS 2

Two packages:

- **`pibob_description`** — URDF (xacro), STL meshes, RViz config, display launch.
- **`pibob_driver`** — Python node that bridges `sensor_msgs/JointState` to PiBob's existing Flask servo API.

The driver POSTs to the Flask app already running on the Pi (`servo-control.service`),
so you don't need ROS on the Pi. Run ROS on your dev machine and aim it at `pibob.local`.

## Build

```bash
mkdir -p ~/pibob_ws/src
ln -s $(pwd)/pibob_description ~/pibob_ws/src/
ln -s $(pwd)/pibob_driver      ~/pibob_ws/src/
cd ~/pibob_ws
colcon build --symlink-install
source install/setup.bash
```

Tested with ROS 2 Humble / Jazzy.

## View the model in RViz

```bash
ros2 launch pibob_description display.launch.py
```

The `joint_state_publisher_gui` sliders let you pose the robot virtually.

## Drive the real robot

Make sure the Flask service is up: `http://pibob.local/` should load.

```bash
ros2 launch pibob_driver bringup.launch.py
```

Send commands by publishing to `/joint_commands`:

```bash
ros2 topic pub --once /joint_commands sensor_msgs/JointState \
  '{name: ["head_pan_joint", "head_tilt_joint"], position: [0.5, -0.2]}'
```

To let the GUI slider drive the real robot, remap on launch:

```bash
ros2 launch pibob_driver bringup.launch.py
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args -r joint_states:=/joint_commands
```

## Configuration

Joint → channel mapping, sign and safe limits live in
`pibob_driver/config/joint_map.yaml`. The `sign` field is your get-out for
direction mismatches; `min_deg`/`max_deg` are clamped before each POST as a
safety net on top of the Flask app's own limits.

## What needs calibration

The URDF link offsets (origin xyz/rpy on each joint) are approximate. To
sharpen visual fidelity, measure the real frame and update the `<origin>`
tags in `urdf/pibob.urdf.xacro`. Joint limits already match the safe ranges
in `app.py` and should not need changing.
