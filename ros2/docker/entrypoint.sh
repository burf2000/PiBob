#!/usr/bin/env bash
# Start a virtual X display, a window manager, a VNC server and the noVNC web
# bridge, then launch the PiBob RViz bring-up inside that display.
set -e

export DISPLAY="${DISPLAY:-:1}"
GEOMETRY="${GEOMETRY:-1920x1080x24}"

echo "[entrypoint] starting Xvfb on ${DISPLAY} (${GEOMETRY})"
Xvfb "${DISPLAY}" -screen 0 "${GEOMETRY}" +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
sleep 1

echo "[entrypoint] starting window manager (openbox)"
openbox >/tmp/openbox.log 2>&1 &
sleep 0.5

echo "[entrypoint] starting taskbar (tint2)"
# tint2 gives a bottom panel listing every window, so minimised windows are one
# click away (openbox alone has no taskbar).
tint2 >/tmp/tint2.log 2>&1 &

echo "[entrypoint] starting x11vnc on :5900"
x11vnc -display "${DISPLAY}" -nopw -forever -shared -rfbport 5900 -bg -quiet >/tmp/x11vnc.log 2>&1

echo "[entrypoint] starting noVNC web bridge on :6080  ->  open http://localhost:6080/vnc.html"
websockify -D --web=/usr/share/novnc 6080 localhost:5900

# ROS 2 environment
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

CONTROL_REAL="${CONTROL_REAL:-false}"
BASE_URL="${BASE_URL:-http://192.168.20.227}"
echo "[entrypoint] ros2 launch  control_real_robot:=${CONTROL_REAL}  base_url:=${BASE_URL}"

exec ros2 launch pibob_driver pibob_bringup.launch.py \
    control_real_robot:="${CONTROL_REAL}" \
    base_url:="${BASE_URL}"
