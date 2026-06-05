# PiBob ROS 2 + RViz (browser) — Docker

Run the PiBob ROS 2 stack in a container and **see the robot in RViz in your
browser**, move the joint sliders to pose it, and optionally drive the **real**
PiBob with the same sliders.

No ROS install needed on your Mac — only Docker.

## Quick start

```bash
cd ros2/docker
docker compose up --build
```

Then open **http://localhost:6080/vnc.html** and click **Connect**. You'll see a
Linux desktop with **RViz** showing the PiBob model and a **Joint State
Publisher** window of sliders. Drag a slider → the model moves in RViz. A
**taskbar** along the bottom lists every window — click it to restore a
minimised RViz or slider window.

> **If it looks small / has scrollbars:** open the noVNC settings (the little
> toolbar tab on the left edge) → **Settings → Scaling Mode → Local Scaling**,
> or just open **http://localhost:6080/vnc.html?resize=scale** so the desktop
> scales to fill your browser window. The desktop is 1920×1080 by default;
> override with `GEOMETRY` (e.g. `GEOMETRY=2560x1440x24`).

First build takes a few minutes (pulls `ros:humble-desktop`); later runs are
instant.

## Drive the real robot too

```bash
cd ros2/docker
CONTROL_REAL=true docker compose up
```

Now the same sliders also POST to the physical PiBob's Flask `/set` API, so the
RViz model and the real servos move together.

Point at a specific robot (the IP, not `pibob.local`, because mDNS does not
resolve inside the container):

```bash
CONTROL_REAL=true BASE_URL=http://192.168.20.227 docker compose up
```

> Tip: PiBob's DHCP address can change. Check it with
> `dscacheutil -q host -a name pibob.local` on the Mac, or give it a static
> lease. Make sure the robot's Flask UI is reachable first (open
> `http://pibob.local` in a browser).

## Live-edit the URDF (hot reload)

Tuning the model? Run in dev mode and edits to the xacro auto-reload in RViz —
no rebuild:

```bash
chmod +x ros2/docker/dev.sh   # first time
./ros2/docker/dev.sh          # add --build after Dockerfile changes
```

Then open **http://localhost:6080/vnc.html**, edit
`ros2/pibob_description/urdf/pibob.urdf.xacro` on your Mac, and **save** — RViz
redraws in ~12 s (the container re-runs xacro and bounces the launch; a syntax
error is logged and the last good model is kept). This is why the Docker route
beats native Mac viewers: the container has ROS + xacro + the package, so
`package://` meshes and the xacro macro just work.

Under the hood it's the same image with two extras: the `urdf/` dir is
volume-mounted from the host, and `WATCH=1` turns on the file-watcher (see
`entrypoint.sh`). `docker logs -f pibob-rviz` shows reloads and any xacro errors.

## How it works

- `robot_state_publisher` loads the URDF from `pibob_description`.
- `joint_state_publisher_gui` provides the sliders → publishes `/joint_states`.
- `rviz2` renders the model from the URL + TF (config: `pibob_description/rviz/pibob.rviz`).
- With `CONTROL_REAL=true`, `servo_bridge` subscribes to that same
  `/joint_states` (remapped from `joint_commands`) and POSTs each joint to the
  robot. Joint→channel mapping and safe limits live in
  `pibob_driver/config/joint_map.yaml`.

RViz is rendered inside the container with software OpenGL (Mesa llvmpipe) and
streamed to your browser over noVNC — the reliable way to get RViz GUI out of a
Linux container on macOS (XQuartz/X11 forwarding is unreliable with RViz's
OpenGL).

## Without compose (plain docker)

```bash
# from the ros2/ dir
docker build -f docker/Dockerfile -t pibob-ros2-rviz:humble .
docker run --rm -p 6080:6080 -e CONTROL_REAL=false pibob-ros2-rviz:humble
```

## Run ROS 2 commands inside the running container

```bash
docker exec -it pibob-rviz bash -lc 'source /ws/install/setup.bash && ros2 topic echo /joint_states'
```
