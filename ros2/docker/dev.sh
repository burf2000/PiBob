#!/usr/bin/env bash
# Live-edit the PiBob URDF: start the container with the urdf/ dir mounted and
# the file-watcher on. Edit ros2/pibob_description/urdf/pibob.urdf.xacro on your
# Mac, save, and RViz redraws in ~12s at http://localhost:6080/vnc.html
#
#   ./ros2/docker/dev.sh
#   ./ros2/docker/dev.sh --build     # rebuild the image first (after Dockerfile/code changes)
set -e

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE=pibob-ros2-rviz:humble

if [ "${1:-}" = "--build" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Building $IMAGE …"
    docker build --platform linux/arm64 -f "$REPO/ros2/docker/Dockerfile" -t "$IMAGE" "$REPO/ros2"
fi

docker rm -f pibob-rviz >/dev/null 2>&1 || true
docker run -d --name pibob-rviz -p 6080:6080 \
    -v "$REPO/ros2/pibob_description/urdf:/ws/install/pibob_description/share/pibob_description/urdf" \
    -e WATCH=1 \
    "$IMAGE" >/dev/null

cat <<EOF

PiBob live-edit is up.
  • Open:  http://localhost:6080/vnc.html   (click Connect)
  • Edit:  ros2/pibob_description/urdf/pibob.urdf.xacro
  • Save → RViz redraws in ~12s. A xacro typo is reported in the log and the
    last good model is kept (see: docker logs -f pibob-rviz).
  • Stop:  docker rm -f pibob-rviz
EOF
