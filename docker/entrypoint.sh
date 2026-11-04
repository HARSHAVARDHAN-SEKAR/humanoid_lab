#!/bin/bash
# Sources the ROS underlay and this workspace's overlay, then execs
# whatever command the container was given (default: bash).
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /workspace/humanoid_lab_ws/install/setup.bash ]; then
    source /workspace/humanoid_lab_ws/install/setup.bash
else
    echo "WARNING: install/setup.bash not found -- was 'colcon build' run?" >&2
fi

exec "$@"
