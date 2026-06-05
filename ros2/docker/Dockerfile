# PiBob ROS 2 Humble + RViz, viewable in a browser via noVNC.
#
# RViz is an OpenGL GUI; on a Mac the reliable way to see it from a Linux
# container is to render it inside the container (software GL) and stream the
# desktop over noVNC. Open http://localhost:6080/vnc.html after `docker compose up`.
#
# Build context is the ros2/ workspace dir (see docker-compose.yml).
# ros:humble (official) is multi-arch incl. arm64, so RViz runs natively on
# Apple Silicon (the osrf/ros desktop image is amd64-only and would emulate).
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Viz packages (ros-base has no GUI), driver dep, plus the noVNC/X stack and a
# software GL driver so RViz renders without a real GPU.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        python3-requests \
        xvfb x11vnc openbox novnc websockify \
        libgl1-mesa-dri mesa-utils \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace (pibob_description + pibob_driver).
WORKDIR /ws
COPY pibob_description src/pibob_description
COPY pibob_driver      src/pibob_driver
RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Defaults — override at run time.
ENV CONTROL_REAL=false
ENV BASE_URL=http://192.168.20.227
# Force software OpenGL so RViz renders without a real GPU.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DISPLAY=:1

EXPOSE 6080
ENTRYPOINT ["/entrypoint.sh"]
