# syntax=docker/dockerfile:1
# ROS 2 Humble + MuJoCo physics. Requires an internet-connected Docker host.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    MUJOCO_GL=egl
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    libgl1 libegl1 libglfw3 libosmesa6 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init 2>/dev/null || true
WORKDIR /workspace/humanoid_lab_ws
COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY . .
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install'
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
