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
URDF_FILE=/ws/install/pibob_description/share/pibob_description/urdf/pibob.urdf.xacro

# (Re)start the whole bring-up. RViz reliably loads the current URDF on a fresh
# launch (it does NOT hot-reload from a republished /robot_description), so for
# live editing we just bounce the launch.
start_stack() {
    # Hard-kill the previous launch + all its nodes by name (SIGKILL) so nothing
    # piles up across reloads (ros2 launch isolates children, so a group kill
    # misses them).
    pkill -9 -f 'pibob_bringup.launch' 2>/dev/null || true
    pkill -9 -x rviz2 2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f joint_state_publisher 2>/dev/null || true
    sleep 1.5
    echo "[entrypoint] ros2 launch  control_real_robot:=${CONTROL_REAL}  base_url:=${BASE_URL}"
    ros2 launch pibob_driver pibob_bringup.launch.py \
        control_real_robot:="${CONTROL_REAL}" \
        base_url:="${BASE_URL}" >/tmp/launch.log 2>&1 &
}

start_stack

if [ "${WATCH:-0}" = "1" ]; then
    # Live-reload (dev): with the urdf/ dir volume-mounted from the host, bounce
    # the launch whenever the xacro changes. Edit on your Mac, save, RViz redraws
    # in ~10s. Syntax errors are reported and the old model is kept.
    echo "[entrypoint] LIVE-RELOAD ON — edit the mounted xacro and save to update RViz (~10s)"
    last=""
    while sleep 1; do
        cur=$(stat -c %Y "$URDF_FILE" 2>/dev/null) || continue
        [ "$cur" = "$last" ] && continue
        if [ -z "$last" ]; then last="$cur"; continue; fi
        last="$cur"
        if xacro "$URDF_FILE" >/dev/null 2>/tmp/xacro.err; then
            echo "[watch] xacro changed -> restarting launch"
            start_stack
        else
            echo "[watch] XACRO ERROR (fix & re-save, keeping current model):"; cat /tmp/xacro.err
        fi
    done
else
    wait
fi
